
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
btCentral.py — MyoWare Wireless Shield BLE client (Bleak)
- Scans for a device by advertised name (default: 'MyoWareSensor1') or by address
- Connects via Bleak's async context manager (already connected inside the block)
- Prints full GATT map
- Auto-discovers a characteristic with props Read+Notify (MyoWare's EMG string)
- Subscribes to notifications & logs decoded values (float/int/string) to CSV

Fixes:
- Removed `await client.is_connected()` (is_connected is a bool property).       # [1](https://www.researchgate.net/publication/375167648_Gym_training_muscle_fatigue_monitoring_using_EMG_myoware_and_arduino_with_envelope_and_sliding_window_methods/fulltext/65428513ff8d8f507ce1fbe3/Gym-training-muscle-fatigue-monitoring-using-EMG-myoware-and-arduino-with-envelope-and-sliding-window-methods.pdf)
- Use `async with BleakClient(...) as client:`; no explicit connect check needed. # [1](https://www.researchgate.net/publication/375167648_Gym_training_muscle_fatigue_monitoring_using_EMG_myoware_and_arduino_with_envelope_and_sliding_window_methods/fulltext/65428513ff8d8f507ce1fbe3/Gym-training-muscle-fatigue-monitoring-using-EMG-myoware-and-arduino-with-envelope-and-sliding-window-methods.pdf)

Usage examples:
  python btCentral.py --name MyoWareSensor1 --runtime 60
  python btCentral.py --address AA:BB:CC:DD:EE:FF --runtime 120
  python btCentral.py --char <CHAR_UUID> --runtime 90 --csv emg.csv

Requires:
  pip install bleak
Bleak docs: https://bleak.readthedocs.io (APIs used: Scanner, Client, services, notify). # [2](https://www.cnx-software.com/2024/07/02/myoware-2-0-muscle-sensor-wireless-kit-targets-electromyography-emg-applications/)
"""

import asyncio
import sys
import csv
import time
from dataclasses import dataclass
from typing import Optional

# Bleak imports (scanner/client, exceptions)
from bleak import BleakScanner, BleakClient, BLEDevice  # [2](https://www.cnx-software.com/2024/07/02/myoware-2-0-muscle-sensor-wireless-kit-targets-electromyography-emg-applications/)
from bleak.exc import BleakError

# ----------------------------- Windows event loop ---------------------------- #
# On some Windows setups, SelectorEventLoopPolicy avoids quirky behavior with
# certain libraries; it's safe and commonly used with Bleak in user projects.     #
if sys.platform.startswith("win"):
    try:
        import asyncio as _asyncio
        _asyncio.set_event_loop_policy(_asyncio.WindowsSelectorEventLoopPolicy())
    except Exception:
        pass

# ------------------------------ Configuration -------------------------------- #
@dataclass
class Config:
    target_name: Optional[str] = "MyoWareSensor1"
    address: Optional[str] = None
    service_uuid: Optional[str] = None
    char_uuid: Optional[str] = None
    scan_time: float = 5.0
    runtime: float = 60.0
    csv_path: Optional[str] = None


# ------------------------------- Utilities ----------------------------------- #
def _is_custom_uuid(uuid: str) -> bool:
    """Treat 128-bit non-SIG shortform as 'custom'."""
    # SIG short UUIDs expand to 0000xxxx-0000-1000-8000-00805f9b34fb
    return "-" in uuid and not uuid.startswith("0000")


def _decode_value(data: bytes):
    """Try to decode as UTF‑8 (MyoWare publishes strings). Fall back to hex string."""
    try:
        s = data.decode("utf-8").strip()
        # Try to parse numeric
    # return int if integer, float if decimal, else keep string
        try:
            if "." in s or "e" in s.lower():
                return float(s)
            else:
                return int(s)
        except Exception:
            return s
    except UnicodeDecodeError:
        return "0x" + data.hex()


async def scanner_find_device(cfg: Config) -> Optional[BLEDevice]:
    """Discover devices for cfg.scan_time seconds; match by address or advertised name."""
    print(f"[scan] scanning for {cfg.scan_time:.1f}s...")
    devices = await BleakScanner.discover(timeout=cfg.scan_time)  # [4](https://www.researchgate.net/profile/Vicky-Andria-Kusuma/publication/373292132_Gym_training_muscle_fatigue_monitoring_using_EMG_myoware_and_arduino_with_envelope_and_sliding_window_methods/links/64e4bb010453074fbda766d0/Gym-training-muscle-fatigue-monitoring-using-EMG-myoware-and-arduino-with-envelope-and-sliding-window-methods.pdf?origin=publication_detail)
    if not devices:
        print("[scan] no BLE devices found")
        return None

    if cfg.address:
        for d in devices:
            if d.address.lower() == cfg.address.lower():
                print(f"[scan] found by address: {d}")
                return d
        print("[scan] address not found among discovered devices")
        return None

    if cfg.target_name:
        for d in devices:
            name = (d.name or "").strip()
            if cfg.target_name.lower() in name.lower():
                print(f"[scan] found by name: {d}")
                return d

    print(f"[scan] no exact match; using first discovered: {devices[0]}")
    return devices[0]


async def explore_gatt(client: BleakClient):
    """Print full GATT map (services + characteristics) for debug/UUID capture."""
    print("\n[GATT] Services & Characteristics")
    print("================================")
    for svc in client.services:  # services are enumerated on connect             # [1](https://www.researchgate.net/publication/375167648_Gym_training_muscle_fatigue_monitoring_using_EMG_myoware_and_arduino_with_envelope_and_sliding_window_methods/fulltext/65428513ff8d8f507ce1fbe3/Gym-training-muscle-fatigue-monitoring-using-EMG-myoware-and-arduino-with-envelope-and-sliding-window-methods.pdf)
        print(f"Service {svc.uuid}  ({svc.description})")
        for ch in svc.characteristics:
            props = ",".join(ch.properties)
            print(f"  Char {ch.uuid}  props=[{props}]  handle={ch.handle}  ({ch.description})")


def auto_find_read_notify_char(client: BleakClient) -> Optional[str]:
    """
    Locate an EMG‑like characteristic automatically:
    - Prefer a char with props Read+Notify under a custom (128‑bit) service,
      else return the first Read+Notify char found.
    """
    # First pass: custom services
    for svc in client.services:
        if _is_custom_uuid(svc.uuid):
            for ch in svc.characteristics:
                prop_set = set(ch.properties)
                if "notify" in prop_set and "read" in prop_set:
                    return ch.uuid
    # Second pass: any service
    for svc in client.services:
        for ch in svc.characteristics:
            prop_set = set(ch.properties)
            if "notify" in prop_set and "read" in prop_set:
                return ch.uuid
    return None


# ------------------------------- Streaming ----------------------------------- #
async def stream_notifications(cfg: Config) -> None:
    """Scan (or use given address), connect, print GATT, subscribe, stream, log CSV."""
    dev = None
    if cfg.address and not cfg.target_name:
        # Connect directly by address if provided (portable; name is advisory).
        dev = BLEDevice(address=cfg.address, name="(manual)")  # [5](https://github.com/Hermiedapwdrman/CH_Myoware)
    else:
        dev = await scanner_find_device(cfg)
        if not dev:
            return

    def on_disconnect(_client: BleakClient):
        print("[disconnect] peripheral disconnected")

    print(f"[connect] connecting to {dev.address} ({dev.name}) ...")
    async with BleakClient(dev.address, disconnected_callback=on_disconnect) as client:
        # Inside the async context manager, Bleak is already connected.           # [1](https://www.researchgate.net/publication/375167648_Gym_training_muscle_fatigue_monitoring_using_EMG_myoware_and_arduino_with_envelope_and_sliding_window_methods/fulltext/65428513ff8d8f507ce1fbe3/Gym-training-muscle-fatigue-monitoring-using-EMG-myoware-and-arduino-with-envelope-and-sliding-window-methods.pdf)
        # If you really want a guard, use the property (not awaited nor called):
        connected = client.is_connected  # bool property                           # [1](https://www.researchgate.net/publication/375167648_Gym_training_muscle_fatigue_monitoring_using_EMG_myoware_and_arduino_with_envelope_and_sliding_window_methods/fulltext/65428513ff8d8f507ce1fbe3/Gym-training-muscle-fatigue-monitoring-using-EMG-myoware-and-arduino-with-envelope-and-sliding-window-methods.pdf)
        print(f"[connect] connected: {connected}")
        if not connected:
            return

        # Explore services/characteristics (helps you capture the concrete UUIDs).
        await explore_gatt(client)

        # Resolve characteristic: CLI-provided or auto-discover Read+Notify char
        char_uuid = cfg.char_uuid or auto_find_read_notify_char(client)
        if not char_uuid:
            print("[error] could not auto-locate a Read+Notify characteristic. "
                  "Pass --char explicitly after checking the map above.")
            return

        print(f"[notify] subscribing to characteristic: {char_uuid}")

        # Prepare CSV logging if requested
        csv_file = None
        csv_writer = None
        if cfg.csv_path:
            csv_file = open(cfg.csv_path, "w", newline="")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["t_sec", "value"])

        start_t = time.time()

        def handler(sender: int, data: bytes):
            val = _decode_value(data)
            t = time.time() - start_t
            print(f"[data] t={t:7.3f}s  val={val}")
            if csv_writer:
                csv_writer.writerow([f"{t:.3f}", val])

        # Subscribe to notifications (Bleak GATT ops)
        await client.start_notify(char_uuid, handler)  # [2](https://www.cnx-software.com/2024/07/02/myoware-2-0-muscle-sensor-wireless-kit-targets-electromyography-emg-applications/)
        try:
            print(f"[notify] streaming for {cfg.runtime:.1f}s ... Ctrl+C to stop.")
            await asyncio.sleep(cfg.runtime)
        except KeyboardInterrupt:
            pass
        finally:
            await client.stop_notify(char_uuid)  # [2](https://www.cnx-software.com/2024/07/02/myoware-2-0-muscle-sensor-wireless-kit-targets-electromyography-emg-applications/)
            if csv_file:
                csv_file.close()
            print("[notify] stopped.")

    print("[connect] disconnected.")


# ---------------------------------- CLI -------------------------------------- #
def parse_args(argv) -> Config:
    import argparse
    p = argparse.ArgumentParser(description="MyoWare BLE client (Bleak)")
    p.add_argument("--name", default="MyoWareSensor1",
                   help="advertised device name to match (default: MyoWareSensor1)")
    p.add_argument("--address", default=None,
                   help="device MAC/Address to connect directly (overrides --name)")
    p.add_argument("--service", default=None,
                   help="service UUID (optional; for filtering/your own logic)")
    p.add_argument("--char", default=None,
                   help="characteristic UUID (optional; if unset, auto-discover Read+Notify)")
    p.add_argument("--scan-time", type=float, default=5.0, help="seconds to scan (default 5)")
    p.add_argument("--runtime", type=float, default=60.0, help="seconds to stream (default 60)")
    p.add_argument("--csv", default=None, help="path to CSV log (optional)")
    args = p.parse_args(argv)

    return Config(
        target_name=args.name,
        address=args.address,
        service_uuid=args.service,
        char_uuid=args.char,
        scan_time=args.scan_time,
        runtime=args.runtime,
        csv_path=args.csv,
    )


async def main_async(cfg: Config):
    try:
        await stream_notifications(cfg)
    except BleakError as e:
        print(f"[bleak] error: {e}")


def main():
    cfg = parse_args(sys.argv[1:])
    # Reminder: DO NOT name your script 'bleak.py': it will shadow the package.   # [6](https://www.mdpi.com/1424-8220/23/18/7873)
    asyncio.run(main_async(cfg))


if __name__ == '__main__':
    main()