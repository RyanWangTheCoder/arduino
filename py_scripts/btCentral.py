
#!/usr/bin/env python3
"""
MyoWare BLE Central for Fatigue Analysis with Bandpass Filtering
-----------------------------------------------------------------
- Connects to Nordic UART Service (NUS)
- Receives ASCII lines: ENV,RAW (updated: no REF)
- Computes PC envelopes (Butterworth & RMS)
- Applies bandpass filter (20–450 Hz) before MDF/MNF
- Logs CSV: ts_ms, env_shield, raw, env_pc_butter, env_pc_rms, mdf, mnf
- Real-time plotting: RAW + PC Butterworth envelope + PC RMS envelope + Shield ENV (top), MDF/MNF trend (bottom)
"""

import asyncio
import argparse
import time
import csv
import os
import sys
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

try:
    from scipy.signal import butter, lfilter, lfilter_zi, get_window, filtfilt
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False

from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic

# Nordic UART Service UUIDs
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

# Bandpass filter for EMG
def bandpass_filter(x, fs, low=20.0, high=450.0, order=4):
    if not SCIPY_AVAILABLE or len(x) < order * 3:
        return x  # fallback: no filtering
    nyq = fs / 2.0
    low = low / nyq
    high = high / nyq
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, x)

# Envelope filters
class ButterLPF:
    def __init__(self, fs: float, cutoff: float = 3.6):
        self.fs = max(1e-3, float(fs))
        self.cutoff = float(cutoff)
        self._use_scipy = SCIPY_AVAILABLE
        self._init_filter()
        self._zi = None
        if self._use_scipy:
            self._zi = lfilter_zi(self._b, self._a) * 0.0
        self._y_prev = 0.0

    def _init_filter(self):
        if self._use_scipy:
            Wn = self.cutoff / (self.fs / 2.0)
            self._b, self._a = butter(2, Wn, btype='low')
        else:
            self._alpha = np.exp(-2.0 * np.pi * self.cutoff / self.fs)

    def set_fs(self, fs: float):
        fs = max(1e-3, float(fs))
        if abs(fs - self.fs) / self.fs > 0.05:
            self.fs = fs
            self._init_filter()
            self._y_prev = 0.0
            if self._use_scipy:
                self._zi = lfilter_zi(self._b, self._a) * 0.0

    def update(self, x: float) -> float:
        if self._use_scipy:
            y, self._zi = lfilter(self._b, self._a, [x], zi=self._zi)
            return float(max(y[0], 0.0))
        y = (1.0 - self._alpha) * x + self._alpha * self._y_prev
        self._y_prev = y
        return float(max(y, 0.0))

class RMSWindow:
    def __init__(self, fs: float, win_ms: float = 50.0):
        self.fs = max(1e-3, float(fs))
        self.win_ms = float(win_ms)
        self._N = max(1, int(round(self.fs * self.win_ms / 1000.0)))
        self._buf = deque(maxlen=self._N)
        self._sumsq = 0.0

    def set_fs(self, fs: float):
        fs = max(1e-3, float(fs))
        if abs(fs - self.fs) / self.fs > 0.05:
            self.fs = fs
            self._N = max(1, int(round(self.fs * self.win_ms / 1000.0)))
            self._buf = deque(maxlen=self._N)
            self._sumsq = 0.0

    def update(self, x: float) -> float:
        if len(self._buf) == self._buf.maxlen:
            old = self._buf[0]
            self._sumsq -= old * old
        self._buf.append(x)
        self._sumsq += x * x
        denom = max(1, len(self._buf))
        return float(np.sqrt(self._sumsq / denom))

# Spectral features
def median_mean_frequency(x: np.ndarray, fs: float) -> tuple:
    n = len(x)
    if n < 16:
        return (np.nan, np.nan)
    if SCIPY_AVAILABLE:
        w = get_window('hamming', n, fftbins=True)
    else:
        w = np.hamming(n)
    X = np.fft.rfft(x * w)
    freqs = np.fft.rfftfreq(n, d=1.0/fs)
    Pxx = (np.abs(X) ** 2) / (np.sum(w**2))
    total = np.sum(Pxx)
    if total <= 0:
        return (np.nan, np.nan)
    cumsum = np.cumsum(Pxx)
    idx = np.searchsorted(cumsum, 0.5 * total)
    mdf = freqs[min(idx, len(freqs)-1)]
    mnf = float(np.sum(freqs * Pxx) / total)
    return (mdf, mnf)

# Line parser
class LineParser:
    def __init__(self):
        self._buf = bytearray()
    def feed(self, data: bytes):
        self._buf.extend(data)
        lines = []
        while True:
            i = self._buf.find(b"\n")
            if i == -1:
                break
            raw = self._buf[:i]
            del self._buf[:i+1]
            if raw.endswith(b"\r"):
                raw = raw[:-1]
            try:
                line = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                line = ''
            if line:
                lines.append(line)
        return lines

# Ring buffer
class Ring:
    def __init__(self, maxlen: int):
        self.maxlen = maxlen
        self.buf = np.zeros(maxlen, dtype=np.float32)
        self.idx = 0
        self.filled = False
    def append_many(self, values):
        vals = np.asarray(values, dtype=np.float32)
        n = len(vals)
        end = self.idx + n
        if end <= self.maxlen:
            self.buf[self.idx:end] = vals
        else:
            first = self.maxlen - self.idx
            self.buf[self.idx:] = vals[:first]
            self.buf[:end % self.maxlen] = vals[first:]
        self.idx = (self.idx + n) % self.maxlen
        if n >= self.maxlen or end >= self.maxlen:
            self.filled = True
    def view_latest(self, n):
        if n >= self.maxlen or not self.filled:
            return self.buf.copy() if self.filled else self.buf[:self.idx].copy()
        start = (self.idx - n) % self.maxlen
        if start + n <= self.maxlen:
            return self.buf[start:start+n].copy()
        part1 = self.buf[start:]
        part2 = self.buf[:(start+n) % self.maxlen]
        return np.concatenate([part1, part2])

async def find_device_by_name(name: str):
    def match(device, adv):
        try:
            if name and device.name == name:
                return True
            if NUS_SERVICE_UUID.lower() in (adv.service_uuids or []):
                return True
        except Exception:
            pass
        return False
    try:
        device = await BleakScanner.find_device_by_filter(match)
        return device
    except Exception:
        devices = await BleakScanner.discover()
        return next((d for d in devices if d.name == name), None)

async def run_once(args, backoff_s: float):
    target = await find_device_by_name(args.name)
    if not target:
        print("Device not found.")
        await asyncio.sleep(backoff_s)
        return False

    print(f"Connecting to: {target.address}: {target.name}")
    disconnected = asyncio.Event()

    def on_disconnect(_client: BleakClient):
        print("Disconnected.")
        disconnected.set()

    try:
        async with BleakClient(target, disconnected_callback=on_disconnect) as client:
            ch_uuid = args.char if args.char else NUS_TX_UUID
            ch_obj = None
            for svc in client.services:
                for ch in svc.characteristics:
                    if str(ch.uuid).lower() == ch_uuid.lower():
                        ch_obj = ch
                        break
                if ch_obj:
                    break
            if not ch_obj:
                print("TX characteristic not found.")
                return False

            parser = LineParser()
            fs_user = float(args.fs) if args.fs > 0 else 0.0
            fs_est = float(fs_user or 100.0)
            ts_hist = deque(maxlen=200)
            env_butter = ButterLPF(fs=fs_est, cutoff=args.cutoff)
            env_rms = RMSWindow(fs=fs_est, win_ms=args.win)

            raw_ring = Ring(maxlen=int(max(200, fs_est * 10)))
            env_shield_ring = Ring(maxlen=int(max(200, fs_est * 10)))
            env_pc_ring = Ring(maxlen=int(max(200, fs_est * 10)))
            env_rms_ring = Ring(maxlen=int(max(200, fs_est * 10)))

            window_sec = 1.0
            window_samples = int(max(16, fs_est * window_sec))
            last_feature = time.time()

            csv_writer = None
            csv_file = None
            if args.save:
                existed = os.path.exists(args.save) and os.path.getsize(args.save) > 0
                csv_file = open(args.save, 'a', newline='', encoding='utf-8')
                csv_writer = csv.writer(csv_file)
                if not existed:
                    csv_writer.writerow(['ts_ms','env_shield','raw','env_pc_butter','env_pc_rms','mdf','mnf'])

            def handle(_: BleakGATTCharacteristic, data: bytearray):
                nonlocal fs_est, window_samples, last_feature
                now = time.time()
                for line in parser.feed(bytes(data)):
                    parts = line.split(',')
                    if len(parts) < 2:
                        continue
                    try:
                        env_sh = float(parts[0])  # ENV first
                        raw = float(parts[1])     # RAW second
                    except Exception:
                        continue

                    ts_hist.append(now)
                    if fs_user <= 0 and len(ts_hist) >= 5:
                        dts = [t2 - t1 for t1, t2 in zip(ts_hist, list(ts_hist)[1:])]
                        avg_dt = float(np.mean(dts)) if dts else 0.01
                        new_fs = float(1.0 / max(1e-6, avg_dt))
                        if abs(new_fs - fs_est) / fs_est > 0.05:
                            fs_est = new_fs
                            env_butter.set_fs(fs_est)
                            env_rms.set_fs(fs_est)
                            window_samples = int(max(16, fs_est * window_sec))

                    rect = abs(raw)
                    env_bt = env_butter.update(rect)
                    env_rm = env_rms.update(rect)

                    mdf, mnf = (np.nan, np.nan)
                    if time.time() - last_feature > 0.2:
                        seg = raw_ring.view_latest(window_samples)
                        if len(seg) >= window_samples:
                            seg_bp = bandpass_filter(seg - np.mean(seg), fs_est)
                            mdf, mnf = median_mean_frequency(seg_bp, fs=fs_est)
                        last_feature = time.time()

                    raw_ring.append_many([raw])
                    env_shield_ring.append_many([env_sh])
                    env_pc_ring.append_many([env_bt])
                    env_rms_ring.append_many([env_rm])

                    if args.out != 'none':
                        ts_ms = int(now * 1000)
                        if args.out == 'csv':
                            cols = [str(ts_ms), f"{env_sh}", f"{raw}", f"{env_bt}", f"{env_rm}", f"{mdf}", f"{mnf}"]
                            print(','.join(cols))
                        elif args.out == 'plain':
                            print(f"[{ts_ms}] env_shield={env_sh} raw={raw} env_bt={env_bt} env_rms={env_rm} MDF={mdf} MNF={mnf}")
                        sys.stdout.flush()

                    if csv_writer:
                        csv_writer.writerow([int(now*1000), env_sh, raw, env_bt, env_rm, mdf, mnf])
                        if (int(now*10) % 10) == 0:
                            csv_file.flush()

            await client.start_notify(ch_obj, handle)
            print("Subscribed to NUS TX; streaming...")

            if args.no_plot:
                try:
                    while not disconnected.is_set():
                        await asyncio.sleep(0.5)
                except KeyboardInterrupt:
                    pass
                finally:
                    await client.stop_notify(ch_obj)
                    if csv_file:
                        csv_file.flush(); csv_file.close()
                return True

            # Plotting
            plt.style.use('seaborn-v0_8')
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
            raw_line, = ax1.plot([], [], color='steelblue', lw=0.9, label='RAW')
            env_bt_line, = ax1.plot([], [], color='crimson', lw=1.6, label='Envelope (PC Butter)')
            env_rms_line, = ax1.plot([], [], color='teal', lw=1.6, label='Envelope (PC RMS)')
            env_sh_line, = ax1.plot([], [], color='gray', lw=1.2, ls='--', label='Envelope (Shield)')
            ax1.set_title('EMG RAW & Envelopes')
            ax1.set_ylabel('Amplitude')
            ax1.legend()

            mdf_line, = ax2.plot([], [], color='green', lw=1.2, label='MDF')
            mnf_line, = ax2.plot([], [], color='orange', lw=1.2, label='MNF')
            ax2.set_title('Spectral Features')
            ax2.set_ylabel('Frequency (Hz)')
            ax2.set_xlabel('Time')
            ax2.legend()

            mdf_trend = deque(maxlen=600)
            mnf_trend = deque(maxlen=600)

            def update(_frame):
                fs = fs_user if fs_user > 0 else fs_est
                disp_sec = 5.0
                n_disp = int(max(10, fs * disp_sec))
                raw_data = raw_ring.view_latest(n_disp)
                env_pc_data = env_pc_ring.view_latest(n_disp)
                env_rms_data = env_rms_ring.view_latest(n_disp)
                env_sh_data = env_shield_ring.view_latest(n_disp)
                t = np.arange(len(raw_data)) / max(1e-3, fs)
                if len(raw_data) > 0:
                    raw_line.set_data(t, raw_data)
                    env_bt_line.set_data(t, env_pc_data)
                    env_rms_line.set_data(t, env_rms_data)
                    env_sh_line.set_data(t, env_sh_data)
                    ax1.set_xlim(max(0, t[-1]-disp_sec), t[-1])
                    ax1.set_ylim(0, max(1.0, float(np.max(raw_data))*1.1))
                if len(mdf_trend) > 0:
                    mdf_line.set_data(np.arange(len(mdf_trend)), np.asarray(mdf_trend))
                if len(mnf_trend) > 0:
                    mnf_line.set_data(np.arange(len(mnf_trend)), np.asarray(mnf_trend))
                return raw_line, env_bt_line, env_rms_line, env_sh_line, mdf_line, mnf_line

            ani = FuncAnimation(fig, update, interval=100)
            plt.tight_layout()
            plt.show()

            await client.stop_notify(ch_obj)
            if csv_file:
                csv_file.flush(); csv_file.close()
            return True
    except Exception as e:
        print(f"Session error: {e}")
        await asyncio.sleep(backoff_s)
        return False

async def run(args):
    base = 1.0
    max_backoff = 30.0
    backoff = base
    try:
        while True:
            ok = await run_once(args, backoff)
            if ok:
                backoff = base
            else:
                backoff = min(max_backoff, backoff * 2.0)
    except KeyboardInterrupt:
        print("Interrupted.")

def main():
    parser = argparse.ArgumentParser(description="MyoWare BLE Central with fatigue analysis and bandpass filtering")
    parser.add_argument('--name', type=str, required=True, help='BLE device name')
    parser.add_argument('--char', type=str, default='', help='Characteristic UUID to subscribe to (default: NUS TX)')
    parser.add_argument('--fs', type=float, default=0.0, help='Sampling rate (Hz), 0=estimate')
    parser.add_argument('--cutoff', type=float, default=3.6, help='Butterworth cutoff (Hz)')
    parser.add_argument('--win', type=float, default=50.0, help='RMS window (ms)')
    parser.add_argument('--out', type=str, choices=['none','plain','csv'], default='none', help='Console output format')
    parser.add_argument('--timestamp', action='store_true', help='Include timestamp')
    parser.add_argument('--no-plot', action='store_true', help='Disable plotting')
    parser.add_argument('--save', type=str, default='', help='CSV file path')
    args = parser.parse_args()

    asyncio.run(run(args))

if __name__ == '__main__':
    main()
