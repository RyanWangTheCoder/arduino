#!/usr/bin/env python3
"""
btCentral_watchdog.py — MyoWare EMG + ICM-20948 IMU + Vitals BLE recorder (Windows/Bleak)

This is a hardened version of your original btCentral.py with:
- Windows-side application watchdog (no retransmission):
  * Tracks last notification time + last good frame time
  * Sends periodic PING writes to NUS RX characteristic
  * Detects stalls (no notifications) and auto-reconnects with backoff
- Stronger health reporting: notify rate, ages, ping status, reconnect reason
- Thread-safe stats: notify callback only schedules work onto asyncio loop
- Frame parser robustness fix: append parsed frame once (not per TLV)

Usage:
  python btCentral_watchdog.py --name MyoW1 --outdir logs --rotate-min 20

Notes:
- Keep it headless for stability (no plotting here). Use separate tools for realtime viz.
"""

import argparse
import asyncio
import csv
import json
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---- Windows COM / WinRT quirks (Bleak) ----
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
NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # notifications from device
NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # writes to device

# ---- Binary protocol constants ----
MAGIC = b"\xA5\x5A"
HEADER_LEN = 14
CRC_LEN = 2

TLV_EMG_BATCH = 0x01
TLV_IMU_BATCH = 0x02
TLV_VITALS = 0x03

FLAG_CRC16 = 0x01
FLAG_IMU_META_V2 = 0x02

ACC_FSR_CODE_TO_G = {0: 2, 1: 4, 2: 8, 3: 16}
GYRO_FSR_CODE_TO_DPS = {0: 250, 1: 500, 2: 1000, 3: 2000}

# Sanity cap to prevent parser stall on corrupted headers.
# BLE notify payload is usually <= ~244 bytes with MTU 247, but allow headroom.
MAX_PAYLOAD_LEN = 512


def now_ms() -> int:
    return int(time.time() * 1000)


def fmt_mmss(seconds: float) -> str:
    """Format seconds as mm:ss (clamped >=0)."""
    seconds = max(0.0, float(seconds))
    m = int(seconds // 60)
    s = int(seconds % 60)
    return f"{m:02d}:{s:02d}"


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
                # Keep last byte as possible partial MAGIC
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

            # Append ONE frame (not once per TLV)
            frames.append((ver, flags, frame_seq, t_us_u32, tlvs))

        return frames


# ---- TLV decoders ----

def decode_emg(val: bytes):
    # meta: seq0(u32), t0_off(i16), sr_hz(u16), channels(u8), mask(u8), rsv(u8), count(u16)
    # Firmware meta bytes: 13
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
    ap = argparse.ArgumentParser(description="btCentral: robust BLE recorder (with watchdog)")
    ap.add_argument("--outdir", default="logs", help="Output directory")
    ap.add_argument("--device", default=None,
                    help="BLE address. If omitted, you can use --name or scan by NUS service.")
    ap.add_argument("--name", default=None, help="BLE advertised name to match (e.g., MyoW1).")
    ap.add_argument("--scan-timeout", type=float, default=10.0, help="Scan timeout seconds")
    ap.add_argument("--rotate-min", type=float, default=30.0, help="Rotate EMG/IMU files every N minutes")
    ap.add_argument("--queue-max", type=int, default=20000, help="Max queued notification chunks")
    ap.add_argument("--flush-sec", type=float, default=2.0, help="Flush interval (seconds)")
    ap.add_argument("--timeout", type=float, default=60.0, help="Connect/service discovery timeout (seconds)")
    ap.add_argument("--address-type", choices=["public", "random"], default=None,
                    help="Windows only: address type hint")

    # Watchdog / keepalive tuning
    ap.add_argument("--ping-sec", type=float, default=1.0, help="Send PING to device every N seconds")
    ap.add_argument("--stall-warn-ms", type=int, default=2000, help="Warn if no notify for this long")
    ap.add_argument("--stall-reconnect-ms", type=int, default=5000,
                    help="Reconnect if no notify for this long")
    ap.add_argument("--max-backoff-sec", type=float, default=20.0, help="Max reconnect backoff")

    return ap.parse_args()


async def main() -> None:
    args = parse_args()
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    rotate_sec = float(args.rotate_min) * 60.0
    session_id = time.strftime("%Y%m%d_%H%M%S")

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

    # Buffers for batch disk writes
    emg_rows: List[List] = []
    imu_rows: List[List] = []

    # Throttle meta writes (write on open/rotate + every 10 seconds)
    last_meta_write = 0.0

    # Liveliness / watchdog state
    last_notify_ms = now_ms()
    last_good_frame_ms = now_ms()
    notify_calls = 0
    subscribed = False

    last_frame_seq: Optional[int] = None
    frame_gaps = 0

    last_ping_ok_ms = 0
    ping_fail = 0

    reconnects = 0
    last_reconnect_reason = ""

    loop = asyncio.get_running_loop()

    # Thread-safe enqueue (runs on asyncio loop thread)
    def enqueue_chunk(b: bytes):
        nonlocal last_notify_ms, notify_calls
        last_notify_ms = now_ms()
        notify_calls += 1
        stats.rx_bytes += len(b)
        try:
            q.put_nowait(b)
        except asyncio.QueueFull:
            stats.queue_drops += 1

    def on_notify(_: int, data: bytearray):
        # Do NOT touch shared stats here; schedule onto loop thread.
        loop.call_soon_threadsafe(enqueue_chunk, bytes(data))

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
        last_calls = 0
        last_gaps = 0
        while True:
            await asyncio.sleep(1.0)
            fps = stats.frames_ok - last_frames
            bps = stats.rx_bytes - last_bytes
            nps = notify_calls - last_calls
            gaps_s = frame_gaps - last_gaps

            last_frames = stats.frames_ok
            last_bytes = stats.rx_bytes
            last_calls = notify_calls
            last_gaps = frame_gaps

            age_notify = now_ms() - last_notify_ms
            age_good = now_ms() - last_good_frame_ms
            age_pingok = (now_ms() - last_ping_ok_ms) if last_ping_ok_ms else -1

            state = "OK" if age_notify <= args.stall_warn_ms else ("WARN" if age_notify <= args.stall_reconnect_ms else "STALL")

            # Segment info: index and offset since current rotated EMG file opened
            if emg_csv.fp is None:
                seg_idx = -1
                seg_off = 'NA'
            else:
                seg_idx = int(emg_csv.idx)
                seg_off = fmt_mmss(time.time() - float(emg_csv.t_open))

            print(
                f"[Health] seg={seg_idx} off={seg_off} state={state} conn_sub={int(subscribed)} fps={fps} notify/s={nps} rx_kB/s={bps/1024.0:.1f} q={q.qsize()} "
                f"crc={stats.crc_errors} q_drop={stats.queue_drops} imu_sat={stats.imu_sat} "
                f"last_notify={age_notify}ms last_good={age_good}ms ping_age={age_pingok}ms ping_fail={ping_fail} "
                f"gaps/s={gaps_s} recon={reconnects} reason={last_reconnect_reason}"
            )


    async def consumer_task() -> None:
        nonlocal last_meta_write, emg_rows, imu_rows
        nonlocal last_good_frame_ms, last_frame_seq, frame_gaps

        # Current IMU scaling + FSR (updated from IMU TLV meta v2)
        cur_acc_fsr_g = 2
        cur_gyro_fsr_dps = 250
        imu_scale_factors(cur_acc_fsr_g, cur_gyro_fsr_dps)  # keep function hot

        while True:
            # Drain a batch of chunks to reduce overhead
            first = await q.get()
            chunks = [first]
            for _ in range(63):
                try:
                    chunks.append(q.get_nowait())
                except asyncio.QueueEmpty:
                    break

            host_rx = now_ms()

            for data in chunks:
                for ver, flags, frame_seq, t_us_u32, tlvs in parser.feed(data):
                    if tlvs and tlvs[0][0] == 0xFF:
                        stats.crc_errors += 1
                        continue

                    stats.frames_ok += 1
                    last_good_frame_ms = now_ms()

                    # frameSeq gap tracking
                    if last_frame_seq is not None:
                        expected = (last_frame_seq + 1) & 0xFFFFFFFF
                        if int(frame_seq) != expected:
                            frame_gaps += 1
                    last_frame_seq = int(frame_seq)

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

                            imu_meta["imu_rate_hz"] = int(sr_hz)
                            imu_meta["imu_acc_fsr_g"] = int(cur_acc_fsr_g)
                            imu_meta["imu_gyro_fsr_dps"] = int(cur_gyro_fsr_dps)

                            for i in range(samples.shape[0]):
                                s = samples[i, :]
                                dev_us = t0_us + i * step_us
                                ax, ay, az, gx, gy, gz = [int(x) for x in s.tolist()]
                                imu_rows.append([
                                    dev_us, seq0 + i, ax, ay, az, gx, gy, gz,
                                    int(cur_acc_fsr_g), int(cur_gyro_fsr_dps),
                                    int(frame_seq), host_rx
                                ])
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

                    # Periodic meta write
                    now = time.time()
                    if (now - last_meta_write) > 10.0:
                        emg_csv.meta = emg_meta
                        imu_csv.meta = imu_meta
                        try:
                            emg_csv._write_meta()
                            imu_csv._write_meta()
                        except PermissionError:
                            pass
                        last_meta_write = now

            for _ in chunks:
                q.task_done()

    # Start background tasks (single instance for the run)
    writer_t = asyncio.create_task(writer_task())
    consumer_t = asyncio.create_task(consumer_task())
    health_t = asyncio.create_task(health_task())

    # ---- Device discovery ----
    device = None
    if args.device:
        device = await BleakScanner.find_device_by_address(args.device, timeout=float(args.scan_timeout))
        if device is None:
            print(f"[FATAL] Could not find device by address: {args.device}")
            return

    elif args.name:
        target = str(args.name).strip().lower()
        print(f"[BLE] Scanning for device name='{args.name}' (timeout={args.scan_timeout}s)...")
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
        print(f"[BLE] Selected {device.address} name={device.name}")

    emg_meta["device"] = str(device.address)
    imu_meta["device"] = str(device.address)

    # ---- Connection loop with watchdog-driven reconnect ----
    backoff = 0.5
    reconnect_evt = asyncio.Event()

    async def ping_task(client: BleakClient) -> None:
        nonlocal last_ping_ok_ms, ping_fail
        while True:
            await asyncio.sleep(float(args.ping_sec))
            try:
                await client.write_gatt_char(NUS_RX_UUID, b"PING", response=False)
                last_ping_ok_ms = now_ms()
            except Exception:
                ping_fail += 1

    async def watchdog_task(client: BleakClient) -> None:
        nonlocal last_reconnect_reason
        while True:
            await asyncio.sleep(0.5)
            age = now_ms() - last_notify_ms
            if age > int(args.stall_reconnect_ms):
                # Try a lightweight resubscribe once.
                last_reconnect_reason = f"stall_no_notify_{age}ms"
                try:
                    await client.stop_notify(NUS_TX_UUID)
                    await asyncio.sleep(0.1)
                    await client.start_notify(NUS_TX_UUID, on_notify)
                    last_reconnect_reason = f"resubscribe_attempted_{age}ms"
                    await asyncio.sleep(1.0)
                except Exception:
                    pass

                # If still stalled after resubscribe attempt, request reconnect
                if (now_ms() - last_notify_ms) > int(args.stall_reconnect_ms):
                    last_reconnect_reason = f"reconnect_after_resub_{now_ms() - last_notify_ms}ms"
                    reconnect_evt.set()
                    return

    print(f"[BLE] Connecting to {device.address}...")

    try:
        while True:
            winrt_args = {}
            if args.address_type:
                winrt_args["address_type"] = args.address_type

            reconnect_evt.clear()

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
                    subscribed = True
                    last_reconnect_reason = "connected"
                    backoff = 0.5

                    # Reset notify age at connect
                    last_notify_ms = now_ms()

                    ping_t = asyncio.create_task(ping_task(client))
                    wd_t = asyncio.create_task(watchdog_task(client))

                    print("[BLE] Recording... Ctrl+C to stop")

                    try:
                        while not reconnect_evt.is_set():
                            await asyncio.sleep(0.25)
                    except KeyboardInterrupt:
                        print("\n[INFO] Stopping...")
                        break
                    finally:
                        subscribed = False
                        try:
                            await client.stop_notify(NUS_TX_UUID)
                        except Exception:
                            pass
                        for t in (ping_t, wd_t):
                            t.cancel()
                            try:
                                await t
                            except Exception:
                                pass

                    if reconnect_evt.is_set():
                        raise RuntimeError("WATCHDOG_RECONNECT")

            except KeyboardInterrupt:
                break
            except Exception as e:
                reconnects += 1
                if "WATCHDOG_RECONNECT" not in str(e):
                    last_reconnect_reason = f"exception:{type(e).__name__}"

                # Clear queue to prevent old buffered chunks after reconnect
                try:
                    while not q.empty():
                        q.get_nowait()
                        q.task_done()
                except Exception:
                    pass

                sleep_s = min(float(args.max_backoff_sec), backoff)
                print(f"[BLE] Reconnecting in {sleep_s:.1f}s (reason={last_reconnect_reason})")
                await asyncio.sleep(sleep_s)
                backoff = min(float(args.max_backoff_sec), backoff * 2.0)

    finally:
        # Shutdown tasks
        for t in (writer_t, consumer_t, health_t):
            t.cancel()
            try:
                await t
            except Exception:
                pass

        # Final flush
        try:
            if emg_rows:
                emg_csv.writerows(emg_rows)
            if imu_rows:
                imu_csv.writerows(imu_rows)
        except Exception:
            pass

        emg_csv.close()
        imu_csv.close()
        vit_fp.flush()
        vit_fp.close()
        print("[DONE] Logs written to", outdir)


if __name__ == "__main__":
    asyncio.run(main())
