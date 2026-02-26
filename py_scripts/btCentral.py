#!/usr/bin/env python3
"""btCentral_fixed_plotudp.py — Robust BLE recorder + watchdog + UDP telemetry for plotting.

Why your RAW and ENV looked too similar before:
- If you downsample RAW too aggressively (e.g., 50 Hz), you lose the high-frequency EMG oscillations.
  RAW then appears like a slow-varying signal similar to ENV.

This recorder sends:
- RAW at a higher UDP rate (default 500 Hz) for oscilloscope-like display
- ENV at a lower UDP rate (default 50 Hz)
- IMU accel magnitude at a lower UDP rate (default 50 Hz)

All UDP sends are best-effort and never block recording.

Usage (record only):
  python btCentral_fixed_plotudp.py --name MyoW1 --outdir logs --rotate-min 20

Usage (record + UDP for realtime plotter):
  python btCentral_fixed_plotudp.py --name MyoW1 --outdir logs --rotate-min 20 --udp-enable --udp-port 9999

Run separate plotter:
  python udp_plotter2.py --mode live --listen-port 9999

"""

import argparse
import asyncio
import csv
import json
import os
import struct
import sys
import time
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---- Windows COM / WinRT quirks (Bleak) ----
if sys.platform == "win32":
    sys.coinit_flags = 0
    try:
        from bleak.backends.winrt.util import allow_sta
        allow_sta()
    except Exception:
        pass

from bleak import BleakClient, BleakScanner

# ---- UUIDs (Nordic UART Service) ----
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # notifications
NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # writes

# ---- Binary protocol ----
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

MAX_PAYLOAD_LEN = 512


def now_ms() -> int:
    return int(time.time() * 1000)


def fmt_mmss(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    m = int(seconds // 60)
    s = int(seconds % 60)
    return f"{m:02d}:{s:02d}"


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) & 0xFFFF) if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc & 0xFFFF


def imu_scale_factors(acc_fsr_g: int, gyro_fsr_dps: int) -> Tuple[float, float]:
    acc_map = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
    gyr_map = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}
    return acc_map.get(acc_fsr_g, 16384.0), gyr_map.get(gyro_fsr_dps, 131.0)


class FrameParser:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < HEADER_LEN:
                break

            _, _, ver, flags, frame_seq, t_us_u32, payload_len = struct.unpack_from("<BBBBIIH", self.buf, 0)
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


def decode_emg(val: bytes):
    # meta: seq0(u32) t0_off(i16) sr(u16) ch(u8) mask(u8) rsv(u8) count(u16)
    seq0, t0_off, sr_hz = struct.unpack_from("<IhH", val, 0)
    channels = val[8]
    count = struct.unpack_from("<H", val, 11)[0]
    samples = np.frombuffer(val[13:], dtype="<u2").reshape(count, channels)
    return int(seq0), int(t0_off), int(sr_hz), int(channels), samples


def decode_imu(val: bytes, ver: int, flags: int):
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
    """Decode Vitals TLV payload (with battery data, backward compatible)."""
    if len(val) < 11:
        # Old format (no battery) - backward compatible
        if len(val) >= 8:
            spo2_u16, hr_x10_u16, age_ms, flags, bright = struct.unpack_from("<HHHBB", val, 0)
            spo2 = None if spo2_u16 == 0xFFFF else float(spo2_u16)
            hr = None if hr_x10_u16 == 0xFFFF else float(hr_x10_u16) / 10.0
            return {
                "spo2": spo2,
                "hr": hr,
                "age_ms": int(age_ms),
                "flags": int(flags),
                "brightness": int(bright),
                "battery_pct": None,  # Not available in old format
                "battery_v": None,
            }
        return {}
    
    # New format (with battery) - 11 bytes
    spo2_u16, hr_x10_u16, age_ms, flags, bright, batt_pct, batt_mv = struct.unpack_from("<HHHBBBH", val, 0)
    spo2 = None if spo2_u16 == 0xFFFF else float(spo2_u16)
    hr = None if hr_x10_u16 == 0xFFFF else float(hr_x10_u16) / 10.0
    
    return {
        "spo2": spo2,
        "hr": hr,
        "age_ms": int(age_ms),
        "flags": int(flags),
        "brightness": int(bright),
        "battery_pct": int(batt_pct) if batt_pct < 255 else None,  # 255 = unknown
        "battery_v": float(batt_mv) / 1000.0 if batt_mv > 0 else None,  # 0 = unknown
    }


class DeviceClockExtender:
    def __init__(self) -> None:
        self.wraps = 0
        self.last_u32: Optional[int] = None

    def extend(self, t_us_u32: int) -> int:
        t = int(t_us_u32) & 0xFFFFFFFF
        if self.last_u32 is None:
            self.last_u32 = t
            return t
        if t < self.last_u32 and (self.last_u32 - t) > 0x40000000:
            self.wraps += 1
        self.last_u32 = t
        return (self.wraps << 32) + t


@dataclass
class TimeRotatingCsv:
    outdir: Path
    base: str
    name_prefix: str
    header: List[str]
    rotate_sec: float
    meta: Dict
    idx: int = 0
    t_open: float = 0.0
    open_id: str = ""
    fp: Optional[object] = None
    w: Optional[csv.writer] = None

    def _csv_path(self) -> Path:
        return self.outdir / f"{self.base}_{self.name_prefix}_{self.open_id}_{self.idx:03d}.csv"


    def _meta_path(self) -> Path:
        return self.outdir / f"{self.base}_{self.name_prefix}_{self.open_id}_{self.idx:03d}.meta.json"


    def _write_meta(self) -> None:
        tmp = str(self._meta_path()) + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.meta, f, indent=2, sort_keys=True)
        os.replace(tmp, self._meta_path())


    def open(self) -> None:
        self.outdir.mkdir(parents=True, exist_ok=True)
        self.t_open = time.time()

        # Use a timestamp with milliseconds to avoid rare collisions where a rotate happens
        # within the same second as the previous open (which could otherwise overwrite files).
        self.open_id = time.strftime("%Y%m%d_%H%M%S") + f"_{int((self.t_open * 1000) % 1000):03d}"

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


@dataclass
class RecorderState:
    q: asyncio.Queue
    stats: Stats
    parser: FrameParser
    clock: DeviceClockExtender

    last_notify_ms: int
    last_good_frame_ms: int
    notify_calls: int

    last_frame_seq: Optional[int]
    frame_gaps: int

    last_ping_ok_ms: int
    ping_fail: int

    reconnects: int
    last_reconnect_reason: str
    subscribed: bool

    last_battery_pct: Optional[int]
    last_battery_v: Optional[float]

    emg_rows: List[List]
    imu_rows: List[List]
    last_meta_write: float

    emg_csv: TimeRotatingCsv
    imu_csv: TimeRotatingCsv
    vit_fp: object
    vit_w: csv.writer

    udp_sock: Optional[socket.socket]
    udp_target: Optional[Tuple[str, int]]

    udp_raw_hz: int
    udp_env_hz: int
    udp_imu_hz: int

    # UDP plot meta (for receiver)
    udp_adc_bits: int
    udp_emg_ymax: int
    last_emg_sr_hz: int

    emg_raw_decim: int
    emg_env_decim: int
    imu_decim: int


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="btCentral: robust BLE recorder (plot-friendly UDP)")
    ap.add_argument("--outdir", default="logs", help="Output directory")
    ap.add_argument("--device", default=None, help="BLE address")
    ap.add_argument("--name", default=None, help="BLE advertised name to match (e.g., MyoW1)")
    ap.add_argument("--scan-timeout", type=float, default=10.0)
    ap.add_argument("--rotate-min", type=float, default=30.0)
    ap.add_argument("--queue-max", type=int, default=20000)
    ap.add_argument("--flush-sec", type=float, default=2.0)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--address-type", choices=["public", "random"], default=None)

    # Watchdog / keepalive
    ap.add_argument("--ping-sec", type=float, default=1.0)
    ap.add_argument("--stall-warn-ms", type=int, default=2000)
    ap.add_argument("--stall-reconnect-ms", type=int, default=5000)
    ap.add_argument("--max-backoff-sec", type=float, default=20.0)

    # UDP
    ap.add_argument("--udp-enable", action="store_true")
    ap.add_argument("--udp-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=9999)
    ap.add_argument("--udp-raw-hz", type=int, default=500, help="RAW plot sample rate")
    ap.add_argument("--udp-env-hz", type=int, default=50, help="ENV plot sample rate")
    ap.add_argument("--udp-imu-hz", type=int, default=50, help="IMU plot sample rate")

    return ap.parse_args()


def udp_send(st: RecorderState, obj: dict) -> None:
    if st.udp_sock is None or st.udp_target is None:
        return

    # Best-effort UDP: never block recording. Also add a simple backoff if sends start failing
    # (e.g., receiver not running / network stack issues) to avoid burning CPU.
    now = time.time()
    next_ok = getattr(st, "udp_next_ok_ts", 0.0)
    if now < next_ok:
        return

    try:
        st.udp_sock.sendto(json.dumps(obj, separators=(",", ":")).encode("utf-8"), st.udp_target)
        setattr(st, "udp_fail_streak", 0)
    except Exception:
        fail_streak = int(getattr(st, "udp_fail_streak", 0)) + 1
        setattr(st, "udp_fail_streak", fail_streak)

        # Exponential backoff up to 1s between attempts
        backoff_s = min(1.0, 0.01 * (2 ** min(fail_streak, 7)))
        setattr(st, "udp_next_ok_ts", now + backoff_s)



async def writer_task(st: RecorderState, flush_sec: float) -> None:
    last_flush = time.time()
    while True:
        await asyncio.sleep(0.05)
        now = time.time()
        if (now - last_flush) >= float(flush_sec):
            if st.emg_rows:
                st.emg_csv.writerows(st.emg_rows)
                st.emg_rows.clear()
            if st.imu_rows:
                st.imu_csv.writerows(st.imu_rows)
                st.imu_rows.clear()
            st.emg_csv.flush()
            st.imu_csv.flush()
            st.vit_fp.flush()
            last_flush = now


async def health_task(st: RecorderState, args: argparse.Namespace) -> None:
    last_frames = last_bytes = last_calls = last_gaps = 0
    while True:
        await asyncio.sleep(1.0)
        fps = st.stats.frames_ok - last_frames
        bps = st.stats.rx_bytes - last_bytes
        nps = st.notify_calls - last_calls
        gaps_s = st.frame_gaps - last_gaps

        last_frames = st.stats.frames_ok
        last_bytes = st.stats.rx_bytes
        last_calls = st.notify_calls
        last_gaps = st.frame_gaps

        age_notify = now_ms() - st.last_notify_ms
        age_good = now_ms() - st.last_good_frame_ms
        age_pingok = (now_ms() - st.last_ping_ok_ms) if st.last_ping_ok_ms else -1

        state = "OK" if age_notify <= args.stall_warn_ms else ("WARN" if age_notify <= args.stall_reconnect_ms else "STALL")

        if st.emg_csv.fp is None:
            seg_idx = -1
            seg_off = "NA"
        else:
            seg_idx = int(st.emg_csv.idx)
            seg_off = fmt_mmss(time.time() - float(st.emg_csv.t_open))

        # Format battery string
        if st.last_battery_pct is not None:
            batt_str = f"batt={st.last_battery_pct}%"
            if st.last_battery_v is not None:
                batt_str += f" ({st.last_battery_v:.2f}V)"
        else:
            batt_str = "batt=N/A"

        print(
            f"[Health] seg={seg_idx} off={seg_off} state={state} conn_sub={int(st.subscribed)} "
            f"fps={fps} notify/s={nps} rx_kB/s={bps/1024.0:.1f} q={st.q.qsize()} "
            f"crc={st.stats.crc_errors} q_drop={st.stats.queue_drops} imu_sat={st.stats.imu_sat} "
            f"{batt_str}  "
            f"last_notify={age_notify}ms last_good={age_good}ms ping_age={age_pingok}ms ping_fail={st.ping_fail} "
            f"gaps/s={gaps_s} recon={st.reconnects} reason={st.last_reconnect_reason}"
        )
        
        # Low battery warnings
        if st.last_battery_pct is not None:
            if st.last_battery_pct < 20:
                print(f"[WARN] Battery low: {st.last_battery_pct}% - Consider stopping recording soon.")
            if st.last_battery_pct < 10:
                print(f"[CRITICAL] Battery critical: {st.last_battery_pct}% - Recording may stop soon!")


async def consumer_task(st: RecorderState, args: argparse.Namespace) -> None:
    cur_acc_fsr_g = 4     # matches firmware IMU_ACC_FSR_G=4 (was 2, saturated)
    cur_gyro_fsr_dps = 1000  # matches firmware IMU_GYRO_FSR_DPS=1000 (was 250, saturated)

    # Cooperative scheduling: periodically yield to the event loop so writer_task/health_task
    # can run even when we are processing large EMG/IMU batches.
    yield_every_frames = 50
    yield_every_samples = 1000
    frames_since_yield = 0
    samples_since_yield = 0

    while True:
        first = await st.q.get()
        chunks = [first]
        for _ in range(63):
            try:
                chunks.append(st.q.get_nowait())
            except asyncio.QueueEmpty:
                break

        host_rx = now_ms()

        for data in chunks:
            for ver, flags, frame_seq, t_us_u32, tlvs in st.parser.feed(data):
                if tlvs and tlvs[0][0] == 0xFF:
                    st.stats.crc_errors += 1
                    continue

                st.stats.frames_ok += 1
                st.last_good_frame_ms = now_ms()

                if st.last_frame_seq is not None:
                    expected = (st.last_frame_seq + 1) & 0xFFFFFFFF
                    if int(frame_seq) != expected:
                        st.frame_gaps += 1
                st.last_frame_seq = int(frame_seq)

                dev_frame_us = st.clock.extend(t_us_u32)

                for t, v in tlvs:
                    if t == TLV_EMG_BATCH:
                        seq0, t0_off, sr_hz, ch, samples = decode_emg(v)

                        # Update EMG meta for files
                        st.emg_csv.meta["emg_rate_hz"] = int(sr_hz)
                        st.emg_csv.meta["channels"] = int(ch)

                        # Send EMG meta when sample rate changes (for plotter filter/timebase)
                        if sr_hz and int(sr_hz) != int(st.last_emg_sr_hz):
                            st.last_emg_sr_hz = int(sr_hz)
                            print(f"[EMG] sr_hz={int(sr_hz)} Hz ch={int(ch)}")
                            udp_send(
                                st,
                                {
                                    "k": "emg_meta",
                                    "sr_hz": int(sr_hz),
                                    "ch": int(ch),
                                    "adc_bits": int(st.udp_adc_bits),
                                    "ymax": int(st.udp_emg_ymax),
                                    "seg": int(st.emg_csv.idx),
                                },
                            )

                        t0_us = dev_frame_us + int(t0_off)
                        step_us = int(round(1_000_000 / float(sr_hz))) if sr_hz else 0

                        # Update decims based on actual sample rate
                        if sr_hz:
                            st.emg_raw_decim = max(1, int(int(sr_hz) // max(1, int(st.udp_raw_hz))))
                            st.emg_env_decim = max(1, int(int(sr_hz) // max(1, int(st.udp_env_hz))))

                        for i in range(samples.shape[0]):
                            dev_us = t0_us + i * step_us
                            env = int(samples[i, 0])
                            raw = int(samples[i, 1])
                            ref = int(samples[i, 2]) if ch > 2 else 0
                            st.emg_rows.append([dev_us, seq0 + i, env, raw, ref, int(frame_seq), host_rx])

                            # UDP plot telemetry (decimated)
                            if st.udp_sock is not None:
                                if ((seq0 + i) % st.emg_raw_decim) == 0:
                                    udp_send(
                                        st,
                                        {"k": "raw", "dev_us": int(dev_us), "t": host_rx, "raw": raw, "seg": int(st.emg_csv.idx)},
                                    )
                                if ((seq0 + i) % st.emg_env_decim) == 0:
                                    udp_send(
                                        st,
                                        {"k": "env", "dev_us": int(dev_us), "t": host_rx, "env": env, "seg": int(st.emg_csv.idx)},
                                    )

                            samples_since_yield += 1
                            if samples_since_yield >= yield_every_samples:
                                samples_since_yield = 0
                                await asyncio.sleep(0)

                    elif t == TLV_IMU_BATCH:
                        seq0, t0_off, sr_hz, count, acc_fsr_g, gyro_fsr_dps, samples = decode_imu(v, int(ver), int(flags))
                        t0_us = dev_frame_us + int(t0_off)
                        step_us = int(round(1_000_000 / float(sr_hz))) if sr_hz else 0

                        if acc_fsr_g is not None:
                            cur_acc_fsr_g = int(acc_fsr_g)
                        if gyro_fsr_dps is not None:
                            cur_gyro_fsr_dps = int(gyro_fsr_dps)

                        if sr_hz:
                            st.imu_decim = max(1, int(int(sr_hz) // max(1, int(st.udp_imu_hz))))

                        acc_lsb_per_g, gyro_lsb_per_dps = imu_scale_factors(cur_acc_fsr_g, cur_gyro_fsr_dps)

                        for i in range(samples.shape[0]):
                            s = samples[i, :]
                            dev_us = t0_us + i * step_us
                            ax, ay, az, gx, gy, gz = [int(x) for x in s.tolist()]
                            st.imu_rows.append(
                                [
                                    dev_us,
                                    seq0 + i,
                                    ax,
                                    ay,
                                    az,
                                    gx,
                                    gy,
                                    gz,
                                    cur_acc_fsr_g,
                                    cur_gyro_fsr_dps,
                                    int(frame_seq),
                                    host_rx,
                                ]
                            )

                            if st.udp_sock is not None and ((seq0 + i) % st.imu_decim) == 0:
                                amag = ((ax * ax + ay * ay + az * az) ** 0.5) / float(acc_lsb_per_g)
                                gmag = ((gx * gx + gy * gy + gz * gz) ** 0.5) / float(gyro_lsb_per_dps)
                                udp_send(
                                    st,
                                    {"k": "imu", "dev_us": int(dev_us), "t": host_rx, "amag": amag, "gmag": gmag, "seg": int(st.emg_csv.idx)},
                                )

                            if is_imu_saturated(s):
                                st.stats.imu_sat += 1

                            samples_since_yield += 1
                            if samples_since_yield >= yield_every_samples:
                                samples_since_yield = 0
                                await asyncio.sleep(0)

                    elif t == TLV_VITALS:
                        vit = decode_vitals(v)
                        age_ms = int(vit["age_ms"]) if vit["age_ms"] is not None else 65535
                        dev_computed_us = dev_frame_us - int(age_ms) * 1000

                        # Update cached battery values for health report
                        if vit.get("battery_pct") is not None:
                            st.last_battery_pct = int(vit["battery_pct"])
                        if vit.get("battery_v") is not None:
                            st.last_battery_v = float(vit["battery_v"])

                        # Send vitals to UDP plotter
                        if st.udp_sock is not None:
                            udp_send(
                                st,
                                {
                                    "k": "vitals",
                                    "pct": int(vit["battery_pct"]) if vit.get("battery_pct") is not None else None,
                                    "v": float(vit["battery_v"]) if vit.get("battery_v") is not None else None,
                                    "spo2": int(vit["spo2"]) if vit.get("spo2") is not None else None,
                                    "hr": float(vit["hr"]) if vit.get("hr") is not None else None,
                                    "seg": int(st.emg_csv.idx),
                                },
                            )

                        st.vit_w.writerow(
                            [
                                int(dev_frame_us),
                                int(dev_computed_us),
                                "" if vit["spo2"] is None else f"{vit['spo2']:.0f}",
                                "" if vit["hr"] is None else f"{vit['hr']:.1f}",
                                int(age_ms),
                                int(vit["flags"]),
                                int(vit["brightness"]),
                                "" if vit["battery_pct"] is None else int(vit["battery_pct"]),
                                "" if vit["battery_v"] is None else f"{vit['battery_v']:.2f}",
                                int(frame_seq),
                                host_rx,
                            ]
                        )

                frames_since_yield += 1
                if frames_since_yield >= yield_every_frames:
                    frames_since_yield = 0
                    await asyncio.sleep(0)

                now = time.time()
                if (now - st.last_meta_write) > 10.0:
                    try:
                        st.emg_csv._write_meta()
                        st.imu_csv._write_meta()
                    except PermissionError:
                        pass
                    st.last_meta_write = now

        for _ in chunks:
            st.q.task_done()



async def main() -> None:
    args = parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    rotate_sec = float(args.rotate_min) * 60.0
    session_id = time.strftime("%Y%m%d_%H%M%S")

    udp_sock = None
    udp_target = None
    if args.udp_enable:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setblocking(False)
        udp_target = (str(args.udp_host), int(args.udp_port))
        print(f"[UDP] Enabled -> {udp_target[0]}:{udp_target[1]} (RAW~{args.udp_raw_hz}Hz ENV~{args.udp_env_hz}Hz IMU~{args.udp_imu_hz}Hz)")

        # Vitals CSV
    vitals_path = outdir / "vitals_log.csv"
    vit_fp = open(vitals_path, "a", newline="", encoding="utf-8")
    vit_w = csv.writer(vit_fp)
    if vit_fp.tell() == 0:
        vit_w.writerow(["dev_us_frame", "dev_us_computed", "spo2", "hr_bpm", "age_ms", "flags", "brightness", "battery_pct", "battery_v", "frame_seq", "host_rx_ms"])
    print(f"[CSV] Logging vitals to {vitals_path}")

    started_host_ms = now_ms()

    emg_meta = {"proto_ver": None, "device": None, "emg_rate_hz": None, "channels": None,
                "started_host_ms": started_host_ms, "rotation_minutes": float(args.rotate_min), "session_id": session_id}
    imu_meta = {"proto_ver": None, "device": None, "imu_rate_hz": None, "imu_acc_fsr_g": None, "imu_gyro_fsr_dps": None,
                "started_host_ms": started_host_ms, "rotation_minutes": float(args.rotate_min), "session_id": session_id}

    emg_csv = TimeRotatingCsv(outdir, "emg_log", session_id,
                             ["dev_us", "emg_seq", "env", "raw", "ref", "frame_seq", "host_rx_ms"],
                             rotate_sec, emg_meta)
    imu_csv = TimeRotatingCsv(outdir, "imu_log", session_id,
                             ["dev_us", "imu_seq", "ax", "ay", "az", "gx", "gy", "gz", "acc_fsr_g", "gyro_fsr_dps", "frame_seq", "host_rx_ms"],
                             rotate_sec, imu_meta)

    q: asyncio.Queue[bytes] = asyncio.Queue(maxsize=int(args.queue_max))

    st = RecorderState(
        q=q,
        stats=Stats(),
        parser=FrameParser(),
        clock=DeviceClockExtender(),
        last_notify_ms=now_ms(),
        last_good_frame_ms=now_ms(),
        notify_calls=0,
        last_frame_seq=None,
        frame_gaps=0,
        last_ping_ok_ms=0,
        ping_fail=0,
        reconnects=0,
        last_reconnect_reason="",
        subscribed=False,
        last_battery_pct=None,
        last_battery_v=None,
        emg_rows=[],
        imu_rows=[],
        last_meta_write=0.0,
        emg_csv=emg_csv,
        imu_csv=imu_csv,
        vit_fp=vit_fp,
        vit_w=vit_w,
        udp_sock=udp_sock,
        udp_target=udp_target,
        udp_raw_hz=int(args.udp_raw_hz),
        udp_env_hz=int(args.udp_env_hz),
        udp_imu_hz=int(args.udp_imu_hz),
        udp_adc_bits=12,
        udp_emg_ymax=4095,
        last_emg_sr_hz=0,
        emg_raw_decim=4,
        emg_env_decim=40,
        imu_decim=4,
    )

    loop = asyncio.get_running_loop()

    def enqueue_chunk(b: bytes):
        st.last_notify_ms = now_ms()
        st.notify_calls += 1
        st.stats.rx_bytes += len(b)
        try:
            st.q.put_nowait(b)
        except asyncio.QueueFull:
            # Drop policy: drop newest chunk (this one). Recording continues.
            # We keep counters and emit occasional warnings so data loss isn't silent.
            st.stats.queue_drops += 1
            if (st.stats.queue_drops % 100) == 1:
                print(
                    f"[WARN] BLE queue full: dropped_chunks={st.stats.queue_drops} "
                    f"qsize={st.q.qsize()}/{st.q.maxsize}. "
                    "System may be overloaded (CPU/disk). Consider increasing --queue-max, "
                    "reducing UDP rates, or writing logs to a faster drive."
                )

    def on_notify(_: int, data: bytearray):
        loop.call_soon_threadsafe(enqueue_chunk, bytes(data))


    writer_t = asyncio.create_task(writer_task(st, args.flush_sec))
    consumer_t = asyncio.create_task(consumer_task(st, args))
    health_t = asyncio.create_task(health_task(st, args))

    # discovery
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
        device = await BleakScanner.find_device_by_filter(lambda d, adv: NUS_SERVICE_UUID.lower() in (adv.service_uuids or []),
                                                          timeout=float(args.scan_timeout))
        if device is None:
            print("[FATAL] Device not found (NUS service UUID).")
            return
        print(f"[BLE] Selected {device.address} name={device.name}")

    st.emg_csv.meta["device"] = str(device.address)
    st.imu_csv.meta["device"] = str(device.address)

    reconnect_evt = asyncio.Event()
    backoff = 0.5

    async def ping_task(client: BleakClient) -> None:
        while True:
            await asyncio.sleep(float(args.ping_sec))
            try:
                await client.write_gatt_char(NUS_RX_UUID, b"PING", response=False)
                st.last_ping_ok_ms = now_ms()
            except Exception:
                st.ping_fail += 1

    async def watchdog_task(client: BleakClient) -> None:
        while True:
            await asyncio.sleep(0.5)
            age = now_ms() - st.last_notify_ms
            if age > int(args.stall_reconnect_ms):
                st.last_reconnect_reason = f"stall_no_notify_{age}ms"
                # resubscribe once
                try:
                    await client.stop_notify(NUS_TX_UUID)
                    await asyncio.sleep(0.1)
                    await client.start_notify(NUS_TX_UUID, on_notify)
                    st.last_reconnect_reason = f"resubscribe_attempted_{age}ms"
                    await asyncio.sleep(1.0)
                except Exception:
                    pass
                if (now_ms() - st.last_notify_ms) > int(args.stall_reconnect_ms):
                    st.last_reconnect_reason = f"reconnect_after_resub_{now_ms() - st.last_notify_ms}ms"
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
                async with BleakClient(device, timeout=float(args.timeout), services=[NUS_SERVICE_UUID], winrt=winrt_args) as client:
                    try:
                        await client.request_mtu(247)
                    except Exception:
                        pass

                    print("[BLE] Subscribing...")
                    await client.start_notify(NUS_TX_UUID, on_notify)
                    # Kick device heartbeat
                    try:
                        await client.write_gatt_char(NUS_RX_UUID, b"PING", response=False)
                        st.last_ping_ok_ms = now_ms()
                    except Exception:
                        pass

                    st.subscribed = True
                    st.last_reconnect_reason = "connected"
                    backoff = 0.5
                    st.last_notify_ms = now_ms()

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
                        st.subscribed = False
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
                st.reconnects += 1
                if "WATCHDOG_RECONNECT" not in str(e):
                    st.last_reconnect_reason = f"exception:{type(e).__name__}"

                # Clear any queued BLE chunks from the previous connection attempt
                try:
                    while not st.q.empty():
                        st.q.get_nowait()
                        st.q.task_done()
                except Exception:
                    pass

                # IMPORTANT: FrameParser is stateful (it buffers partial frames). If we reconnect
                # mid-frame, the parser may contain stale bytes from the previous connection.
                # Reset it so parsing restarts cleanly on the next connection.
                st.parser = FrameParser()


                # IMPORTANT: If the firmware resets its device clock/sequence counters on reconnect
                # (frameSeq/emgSeq/imuSeq restart at 0, and t_us_u32 restarts), we must reset the
                # DeviceClockExtender too. Otherwise the post-reconnect dev_us values will be
                # "continued" from the previous session and become incorrect.
                st.clock = DeviceClockExtender()

                # Also reset frame-gap tracking so gap stats are not polluted by the reset.
                st.last_frame_seq = None



                sleep_s = min(float(args.max_backoff_sec), backoff)
                print(f"[BLE] Reconnecting in {sleep_s:.1f}s (reason={st.last_reconnect_reason})")
                await asyncio.sleep(sleep_s)
                backoff = min(float(args.max_backoff_sec), backoff * 2.0)

    finally:
        for t in (writer_t, consumer_t, health_t):
            t.cancel()
            try:
                await t
            except Exception:
                pass

        try:
            if st.emg_rows:
                st.emg_csv.writerows(st.emg_rows)
            if st.imu_rows:
                st.imu_csv.writerows(st.imu_rows)
        except Exception:
            pass

        st.emg_csv.close()
        st.imu_csv.close()
        st.vit_fp.flush()
        st.vit_fp.close()

        if st.udp_sock is not None:
            try:
                st.udp_sock.close()
            except Exception:
                pass

        print("[DONE] Logs written to", outdir)


if __name__ == "__main__":
    asyncio.run(main())
