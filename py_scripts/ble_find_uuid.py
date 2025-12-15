#!/usr/bin/env python3
"""
ble_find_uuid.py

Enumerate services/characteristics for a BLE device by local name.
Relies on Bleak's service discovery performed during connect().

Usage:
  python ble_find_uuid.py --name MyoWareSensor1
"""

import asyncio
import argparse
from bleak import BleakScanner, BleakClient


async def enumerate_services(name: str) -> None:
    """Find device by local name, connect, and list services/characteristics."""
    print(f"Scanning for device named '{name}' ...")
    device = await BleakScanner.find_device_by_name(name, timeout=10.0)
    if not device:
        print("Device not found. Is it advertising?")
        return

    print(f"Connecting to: {device.name} @ {device.address}")
    async with BleakClient(device, timeout=15.0) as client:
        # Service discovery is performed during connect(); read via client.services
        services = list(client.services)
        if not services:
            await asyncio.sleep(0.5)
            services = list(client.services)
        if not services:
            print("No services returned. Try forgetting the device in Windows Bluetooth settings and reconnect.")
            return

        print("\nServices & Characteristics:\n")
        for svc in services:
            print(f"Service {svc.uuid}")
            for ch in svc.characteristics:
                props = ",".join(ch.properties)
                print(f"  Characteristic {ch.uuid}  [{props}]")

        print("\nCopy the UUID for the characteristic that has 'notify' in its properties.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Enumerate BLE services/characteristics (Bleak)")
    parser.add_argument("--name", required=True, help="BLE local name (e.g., MyoWareSensor1)")
    args = parser.parse_args()
    asyncio.run(enumerate_services(args.name))


if __name__ == "__main__":
    main()
