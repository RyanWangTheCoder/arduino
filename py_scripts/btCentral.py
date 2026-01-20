
#!/usr/bin/env python3
"""
btCentral.py — MyoWare EMG + ICM-20948 IMU + Vitals BLE recorder (Windows/Bleak)

Features:
- Connects to Nordic UART Service (NUS) TX notifications
- Parses binary frames (MAGIC + header + TLVs + optional CRC16)
- Logs to separate CSVs for EMG and IMU
- Time-based rotation (default 30 minutes, configurable via --rotate-min)
- Timestamped filenames include BOTH:
    * SESSIONID (run start time)
    * OPENID (file open time for each rotated segment)
- Logs use device-derived timestamps (dev_us, 64-bit monotonic extension of micros())
- Records IMU FSR values (acc_fsr_g, gyro_fsr_dps) in IMU CSV and meta JSON
- Vitals log is single file (low rate, no rotation by default)
- Queue-based notify callback + consumer parser for stability on Windows
- Parser sanity checks to avoid stalls on corrupted headers
- Thread-safe enqueue from WinRT callback using loop.call_soon_threadsafe

Usage examples:
  # Connect by advertised name (recommended; no MAC needed)
  python btCentral.py --name MyoW1 --outdir logs --rotate-min 20

  # Or connect by address (optional)
  python btCentral.py --device 88:13:BF:28:9A:EE --outdir logs --rotate-min 20

  # If scan is flaky, increase scan time
  python btCentral.py --name MyoW1 --scan-timeout 20 --outdir logs

Notes:
- For max stability, keep it headless (no plotting in this script).
"""

import argparse
import asyncio
import csv
import json
import os
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---- Windows COM / WinRT quirks (Bleak) ----
import sys
if sys.platform == "win32":
    # Prefer MTA for BLE; must be set before importing WinRT/COM related modules.
    sys.coinit_flags = 0
    try:
        # Allow STA in case a GUI/IDE configured COM as STA.
        from bleak.backends.winrt.util import allow_sta
        allow_sta()
    except Exception:
        pass

from bleak import BleakClient, BleakScanner

# ---- UUIDs (Nordic UART Service) ----
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

# ---- Binary protocol constants ----
MAGIC = b"\xA5\x5A"
HEADER_LEN = 14
CRC_LEN = 2

TLV_EMG_BATCH = 0x01
TLV_IMU_BATCH = 0x02
TLV_VITALS    = 0x03

FLAG_CRC16       = 0x01
FLAG_IMU_META_V2 = 0x02

ACC_FSR_CODE_TO_G    = {0: 2, 1: 4, 2: 8, 3: 16}
GYRO_FSR_CODE_TO_DPS = {0: 250, 1: 500, 2: 1000, 3: 2000}

# Sanity cap to prevent parser stall on corrupted headers.
# BLE notify payload is usually <= ~244 bytes with MTU 247, but allow headroom.
MAX_PAYLOAD_LEN = 512


def now_ms() -> int:
    return int(time.time() * 1000)


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-CCITT (0x1021), init 0xFFFF."""
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) & 0xFFFF) if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc & 0xFFFF


def imu_scale_factors(acc_fsr_g: int, gyro_fsr_dps: int) -> Tuple[float, float]:
    """Return (acc_lsb_per_g, gyro_lsb_per_dps) for ICM-20948."""
    acc_map = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
    gyr_map = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}
    return acc_map.get(acc_fsr_g, 16384.0), gyr_map.get(gyro_fsr_dps, 131.0)


class FrameParser:
    """Incremental parser for MAGIC-framed packets with sanity checks."""
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []

        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                # keep last byte as possible partial MAGIC
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break

            if i > 0:
                del self.buf[:i]

            if len(self.buf) < HEADER_LEN:
                break

            # Header: <BBBBIIH
            # magic0, magic1, ver, flags, frame_seq(u32), t_us(u32), payload_len(u16)
            _, _, ver, flags, frame_seq, t_us_u32, payload_len = struct.unpack_from("<BBBBIIH", self.buf, 0)

            # Sanity check payload_len to avoid stalling forever on corrupted header.
            if payload_len > MAX_PAYLOAD_LEN:
                del self.buf[0:1]
                continue

            total_len = HEADER_LEN + payload_len + CRC_LEN
            if len(self.buf) < total_len:
                break

            frame_bytes = bytes(self.buf[:total_len])
            del self.buf[:total_len]

            if flags & FLAG_CRC16:
                pre_crc = frame_bytes[:-2]
                rx_crc = struct.unpack_from("<H", frame_bytes, len(frame_bytes) - 2)[0]
                if crc16_ccitt(pre_crc) != rx_crc:
                    frames.append((ver, flags, frame_seq, t_us_u32, [(0xFF, b"CRC_ERROR")]))
                    continue

            payload = frame_bytes[HEADER_LEN:HEADER_LEN + payload_len]
            tlvs = []
            off = 0
            while off + 3 <= len(payload):
                tlv_type = payload[off]
                tlv_len = struct.unpack_from("<H", payload, off + 1)[0]
                off += 3
                if off + tlv_len > len(payload):
                    break
                tlvs.append((tlv_type, payload[off:off + tlv_len]))
                off += tlv_len

            frames.append((ver, flags, frame_seq, t_us_u32, tlvs))

        return frames


# ---- TLV decoders ----

def decode_emg(val: bytes):
    # meta: seq0(u32), t0_off(i16), sr_hz(u16), channels(u8), rsv(u16), count(u16)
    seq0, t0_off, sr_hz = struct.unpack_from("<IhH", val, 0)
    channels = val[8]
    count = struct.unpack_from("<H", val, 11)[0]
    samples = np.frombuffer(val[13:], dtype="<u2").reshape(count, channels)
    return int(seq0), int(t0_off), int(sr_hz), int(channels), samples


def decode_imu(val: bytes, ver: int, flags: int):
    # meta v1 (12): seq0(u32), t0_off(i16), sr_hz(u16), count(u8), rsv(u8), mask(u16)
    # meta v2 (14): v1 + acc_fsr_code(u8) + gyro_fsr_code(u8)
    seq0, t0_off, sr_hz = struct.unpack_from("<IhH", val, 0)
    count = val[8]

    meta_len = 12
    acc_fsr_g = None
    gyro_fsr_dps = None

    if (ver >= 2) or (flags & FLAG_IMU_META_V2):
        meta_len = 14
        if len(val) >= 14:
            acc_fsr_g = ACC_FSR_CODE_TO_G.get(int(val[12]))
            gyro_fsr_dps = GYRO_FSR_CODE_TO_DPS.get(int(val[13]))

    samples = np.frombuffer(val[meta_len:], dtype="<i2").reshape(count, 6)
    return int(seq0), int(t0_off), int(sr_hz), int(count), acc_fsr_g, gyro_fsr_dps, samples


def decode_vitals(val: bytes) -> Dict[str, Optional[float]]:
    # spo2(u16), hr_x10(u16), age_ms(u16), flags(u8), brightness(u8)
    spo2_u16, hr_x10_u16, age_ms, flags, bright = struct.unpack_from("<HHHBB", val, 0)
    spo2 = None if spo2_u16 == 0xFFFF else float(spo2_u16)
    hr = None if hr_x10_u16 == 0xFFFF else float(hr_x10_u16) / 10.0
    return {"spo2": spo2, "hr": hr, "age_ms": int(age_ms), "flags": int(flags), "brightness": int(bright)}


# ---- Device time extension (micros wrap) ----

class DeviceClockExtender:
    """Extend 32-bit micros() timestamps to a monotonic 64-bit timeline."""
    def __init__(self) -> None:
        self.wraps = 0
        self.last_u32: Optional[int] = None

    def extend(self, t_us_u32: int) -> int:
        t = int(t_us_u32) & 0xFFFFFFFF
        if self.last_u32 is None:
            self.last_u32 = t
            return t
        # Detect wrap on large backward jump
        if t < self.last_u32 and (self.last_u32 - t) > 0x40000000:
            self.wraps += 1
        self.last_u32 = t
        return (self.wraps << 32) + t


# ---- Time-based rotating CSV writer ----

@dataclass
class TimeRotatingCsv:
    outdir: Path
    base: str
    name_prefix: str  # session id
    header: List[str]
    rotate_sec: float
    meta: Dict

    idx: int = 0
    t_open: float = 0.0
    open_id: str = ""  # per-file open timestamp
    fp: Optional[object] = None
    w: Optional[csv.writer] = None

    def _csv_path(self) -> Path:
        return self.outdir / f"{self.base}_{self.name_prefix}_{self.open_id}_{self.idx:03d}.csv"

    def _meta_path(self) -> Path:
        return self.outdir / f"{self.base}_{self.name_prefix}_{self.open_id}_{self.idx:03d}.meta.json"

    def _write_meta(self) -> None:
        # Atomic replace via temp file; written only on open/rotate + periodic throttle
        tmp = str(self._meta_path()) + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.meta, f, indent=2, sort_keys=True)
        os.replace(tmp, self._meta_path())

    def open(self) -> None:
        self.outdir.mkdir(parents=True, exist_ok=True)
        self.t_open = time.time()
        self.open_id = time.strftime("%Y%m%d_%H%M%S")

        self.fp = open(self._csv_path(), "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.fp)
        self.w.writerow(self.header)

        self.meta["file_index"] = int(self.idx)
        self.meta["opened_host_ms"] = int(time.time() * 1000)
        self.meta["opened_id"] = self.open_id
        self._write_meta()

        print(f"[CSV] Logging to {self._csv_path()}")

    def rotate_if_needed(self) -> None:
        if self.fp is None:
            self.open()
            return
        if (time.time() - self.t_open) >= float(self.rotate_sec):
            self.close()
            self.idx += 1
            self.open()

    def writerows(self, rows: List[List]) -> None:
        if not rows:
            return
        if self.fp is None:
            self.open()
        self.rotate_if_needed()
        assert self.w is not None
        self.w.writerows(rows)

    def flush(self) -> None:
        if self.fp:
            self.fp.flush()

    def close(self) -> None:
        if self.fp:
            self.fp.flush()
            self.fp.close()
            self.fp = None
            self.w = None


@dataclass
class Stats:
    rx_bytes: int = 0
    frames_ok: int = 0
    crc_errors: int = 0
    queue_drops: int = 0
    imu_sat: int = 0


def is_imu_saturated(sample6: np.ndarray) -> bool:
    v = np.asarray(sample6, dtype=np.int32)
    return bool(np.any(np.abs(v) >= 32767))


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="btCentral: robust BLE recorder")
    ap.add_argument("--outdir", default="logs", help="Output directory")
    ap.add_argument("--device", default=None, help="BLE address. If omitted, you can use --name or scan by NUS service.")
    ap.add_argument("--name", default=None, help="BLE advertised name to match (e.g., MyoW1).")
    ap.add_argument("--scan-timeout", type=float, default=10.0, help="Scan timeout seconds when using --name or scanning.")
    ap.add_argument("--rotate-min", type=float, default=30.0, help="Rotate EMG/IMU files every N minutes")
    ap.add_argument("--queue-max", type=int, default=20000, help="Max queued notification chunks")
    ap.add_argument("--flush-sec", type=float, default=2.0, help="Flush interval (seconds)")
    ap.add_argument("--timeout", type=float, default=60.0, help="Connect/service discovery timeout (seconds)")
    ap.add_argument("--address-type", choices=["public", "random"], default=None,
                    help="Windows only: address type hint")
    return ap.parse_args()


async def main() -> None:
    args = parse_args()
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    rotate_sec = float(args.rotate_min) * 60.0
    session_id = time.strftime("%Y%m%d_%H%M%S")  # run start id (local time)

    # Vitals CSV (no rotation by default)
    vitals_path = outdir / "vitals_log.csv"
    vit_fp = open(vitals_path, "a", newline="", encoding="utf-8")
    vit_w = csv.writer(vit_fp)
    if vit_fp.tell() == 0:
        vit_w.writerow(["dev_us_frame", "dev_us_computed", "spo2", "hr_bpm", "age_ms", "flags",
                        "brightness", "frame_seq", "host_rx_ms"])
    print(f"[CSV] Logging vitals to {vitals_path}")

    started_host_ms = now_ms()
    emg_meta = {
        "proto_ver": None,
        "device": None,
        "emg_rate_hz": None,
        "channels": None,
        "started_host_ms": started_host_ms,
        "rotation_minutes": float(args.rotate_min),
        "session_id": session_id,
    }
    imu_meta = {
        "proto_ver": None,
        "device": None,
        "imu_rate_hz": None,
        "imu_acc_fsr_g": None,
        "imu_gyro_fsr_dps": None,
        "started_host_ms": started_host_ms,
        "rotation_minutes": float(args.rotate_min),
        "session_id": session_id,
    }

    emg_csv = TimeRotatingCsv(
        outdir=outdir,
        base="emg_log",
        name_prefix=session_id,
        header=["dev_us", "emg_seq", "env", "raw", "ref", "frame_seq", "host_rx_ms"],
        rotate_sec=rotate_sec,
        meta=emg_meta,
    )
    imu_csv = TimeRotatingCsv(
        outdir=outdir,
        base="imu_log",
        name_prefix=session_id,
        header=["dev_us", "imu_seq", "ax", "ay", "az", "gx", "gy", "gz",
                "acc_fsr_g", "gyro_fsr_dps", "frame_seq", "host_rx_ms"],
        rotate_sec=rotate_sec,
        meta=imu_meta,
    )

    # Queue for raw BLE notification chunks
    q: asyncio.Queue[bytes] = asyncio.Queue(maxsize=int(args.queue_max))
    stats = Stats()

    parser = FrameParser()
    clock = DeviceClockExtender()

    # Current IMU scaling + FSR (updated from IMU TLV meta v2)
    cur_acc_fsr_g = 2
    cur_gyro_fsr_dps = 250
    acc_lsb_per_g, gyro_lsb_per_dps = imu_scale_factors(cur_acc_fsr_g, cur_gyro_fsr_dps)

    # Buffers for batch disk writes
    emg_rows: List[List] = []
    imu_rows: List[List] = []

    # Throttle meta writes (write on open/rotate + every 10 seconds)
    last_meta_write = 0.0

    async def writer_task() -> None:
        nonlocal emg_rows, imu_rows
        last_flush = time.time()
        while True:
            await asyncio.sleep(0.05)
            now = time.time()
            if (now - last_flush) >= float(args.flush_sec):
                if emg_rows:
                    emg_csv.writerows(emg_rows)
                    emg_rows = []
                if imu_rows:
                    imu_csv.writerows(imu_rows)
                    imu_rows = []
                emg_csv.flush()
                imu_csv.flush()
                vit_fp.flush()
                last_flush = now

    async def health_task() -> None:
        last_frames = 0
        last_bytes = 0
        while True:
            await asyncio.sleep(1.0)
            fps = stats.frames_ok - last_frames
            bps = stats.rx_bytes - last_bytes
            last_frames = stats.frames_ok
            last_bytes = stats.rx_bytes
            print(f"[Health] frames/s={fps} rx_kB/s={bps/1024.0:.1f} q={q.qsize()} "
                  f"crc_err={stats.crc_errors} q_drop={stats.queue_drops} imu_sat={stats.imu_sat}")

    async def consumer_task() -> None:
        nonlocal cur_acc_fsr_g, cur_gyro_fsr_dps, acc_lsb_per_g, gyro_lsb_per_dps
        nonlocal last_meta_write, emg_rows, imu_rows

        while True:
            # Drain a batch of chunks to reduce overhead
            chunks = [await q.get()]
            q.task_done()
            for _ in range(63):
                try:
                    chunks.append(q.get_nowait())
                    q.task_done()
                except asyncio.QueueEmpty:
                    break

            host_rx = now_ms()

            for data in chunks:
                for ver, flags, frame_seq, t_us_u32, tlvs in parser.feed(data):
                    if tlvs and tlvs[0][0] == 0xFF:
                        stats.crc_errors += 1
                        continue

                    stats.frames_ok += 1
                    dev_frame_us = clock.extend(t_us_u32)

                    # Store proto ver once
                    if emg_meta["proto_ver"] is None:
                        emg_meta["proto_ver"] = int(ver)
                    if imu_meta["proto_ver"] is None:
                        imu_meta["proto_ver"] = int(ver)

                    for t, v in tlvs:
                        if t == TLV_EMG_BATCH:
                            seq0, t0_off, sr_hz, ch, samples = decode_emg(v)
                            t0_us = dev_frame_us + int(t0_off)
                            step_us = int(round(1_000_000 / float(sr_hz))) if sr_hz else 0

                            emg_meta["emg_rate_hz"] = int(sr_hz)
                            emg_meta["channels"] = int(ch)

                            # Per sample rows
                            for i in range(samples.shape[0]):
                                dev_us = t0_us + i * step_us
                                env = int(samples[i, 0])
                                raw = int(samples[i, 1])
                                ref = int(samples[i, 2]) if ch > 2 else 0
                                emg_rows.append([dev_us, seq0 + i, env, raw, ref, int(frame_seq), host_rx])

                        elif t == TLV_IMU_BATCH:
                            seq0, t0_off, sr_hz, count, acc_fsr_g, gyro_fsr_dps, samples = decode_imu(
                                v, ver=int(ver), flags=int(flags)
                            )
                            t0_us = dev_frame_us + int(t0_off)
                            step_us = int(round(1_000_000 / float(sr_hz))) if sr_hz else 0

                            if acc_fsr_g is not None:
                                cur_acc_fsr_g = int(acc_fsr_g)
                            if gyro_fsr_dps is not None:
                                cur_gyro_fsr_dps = int(gyro_fsr_dps)
                            acc_lsb_per_g, gyro_lsb_per_dps = imu_scale_factors(cur_acc_fsr_g, cur_gyro_fsr_dps)

                            imu_meta["imu_rate_hz"] = int(sr_hz)
                            imu_meta["imu_acc_fsr_g"] = int(cur_acc_fsr_g)
                            imu_meta["imu_gyro_fsr_dps"] = int(cur_gyro_fsr_dps)

                            for i in range(samples.shape[0]):
                                s = samples[i, :]
                                dev_us = t0_us + i * step_us
                                ax, ay, az, gx, gy, gz = [int(x) for x in s.tolist()]
                                imu_rows.append([dev_us, seq0 + i, ax, ay, az, gx, gy, gz,
                                                 int(cur_acc_fsr_g), int(cur_gyro_fsr_dps),
                                                 int(frame_seq), host_rx])
                                if is_imu_saturated(s):
                                    stats.imu_sat += 1

                        elif t == TLV_VITALS:
                            vit = decode_vitals(v)
                            age_ms = int(vit["age_ms"]) if vit["age_ms"] is not None else 65535
                            dev_computed_us = dev_frame_us - int(age_ms) * 1000
                            vit_w.writerow([
                                int(dev_frame_us),
                                int(dev_computed_us),
                                "" if vit["spo2"] is None else f"{vit['spo2']:.0f}",
                                "" if vit["hr"] is None else f"{vit['hr']:.1f}",
                                int(age_ms),
                                int(vit["flags"]),
                                int(vit["brightness"]),
                                int(frame_seq),
                                host_rx,
                            ])

                    # Periodic meta write (keeps current rate/FSR updated)
                    now = time.time()
                    if (now - last_meta_write) > 10.0:
                        emg_csv.meta = emg_meta
                        imu_csv.meta = imu_meta
                        # Write meta for current files
                        try:
                            emg_csv._write_meta()
                            imu_csv._write_meta()
                        except PermissionError:
                            # Windows can transiently lock files due to AV/indexing; skip and retry later.
                            pass
                        last_meta_write = now

    # ----- Device discovery -----
    device = None
    if args.device:
        device = await BleakScanner.find_device_by_address(args.device, timeout=float(args.scan_timeout))
        if device is None:
            print(f"[FATAL] Could not find device by address: {args.device}")
            return
    elif args.name:
        target = str(args.name).strip().lower()
        print(f"[BLE] Scanning for device name='{args.name}' (timeout={args.scan_timeout}s)...")

        # Use return_adv=True so we can check adv.local_name as well as dev.name.
        found = await BleakScanner.discover(timeout=float(args.scan_timeout), return_adv=True)

        candidates = []
        for _addr, (dev, adv) in found.items():
            dev_name = (dev.name or "").strip().lower()
            adv_name = (getattr(adv, "local_name", None) or "").strip().lower()
            if (dev_name == target) or (adv_name == target):
                su = set((adv.service_uuids or []))
                has_nus = (NUS_SERVICE_UUID.lower() in [s.lower() for s in su]) if su else False
                rssi = getattr(adv, "rssi", None)
                candidates.append((1 if has_nus else 0, -999 if rssi is None else int(rssi), dev))

        if not candidates:
            print(f"[FATAL] No device found with name '{args.name}'")
            return

        candidates.sort(reverse=True)
        device = candidates[0][2]
        print(f"[BLE] Selected {device.address} name={device.name}")
    else:
        print("[BLE] Scanning for NUS device...")
        device = await BleakScanner.find_device_by_filter(
            lambda d, adv: NUS_SERVICE_UUID.lower() in (adv.service_uuids or []),
            timeout=float(args.scan_timeout),
        )
        if device is None:
            print("[FATAL] Device not found (NUS service UUID).")
            return

    print(f"[BLE] Connecting to {device.address}...")
    emg_meta["device"] = str(device.address)
    imu_meta["device"] = str(device.address)

    # Start background tasks
    writer_t = asyncio.create_task(writer_task())
    consumer_t = asyncio.create_task(consumer_task())
    health_t = asyncio.create_task(health_task())

    # Thread-safe enqueue from BLE callback
    loop = asyncio.get_running_loop()

    def _enqueue_chunk(b: bytes):
        try:
            q.put_nowait(b)
        except asyncio.QueueFull:
            stats.queue_drops += 1

    def on_notify(_: int, data: bytearray):
        stats.rx_bytes += len(data)
        loop.call_soon_threadsafe(_enqueue_chunk, bytes(data))

    # Connect and subscribe
    winrt_args = {}
    if args.address_type:
        winrt_args["address_type"] = args.address_type

    try:
        async with BleakClient(
            device,
            timeout=float(args.timeout),
            services=[NUS_SERVICE_UUID],
            winrt=winrt_args,
        ) as client:
            try:
                await client.request_mtu(247)
            except Exception:
                pass

            print("[BLE] Subscribing...")
            await client.start_notify(NUS_TX_UUID, on_notify)
            print("[BLE] Recording... Ctrl+C to stop")

            try:
                while True:
                    await asyncio.sleep(0.25)
            except KeyboardInterrupt:
                print("\n[INFO] Stopping...")

            await client.stop_notify(NUS_TX_UUID)

    finally:
        # Shutdown tasks
        for t in (writer_t, consumer_t, health_t):
            t.cancel()
            try:
                await t
            except Exception:
                pass

        # Final flush
        if emg_rows:
            emg_csv.writerows(emg_rows)
        if imu_rows:
            imu_csv.writerows(imu_rows)

        emg_csv.close()
        imu_csv.close()
        vit_fp.flush()
        vit_fp.close()

        print("[DONE] Logs written to", outdir)


if __name__ == "__main__":
    asyncio.run(main())
