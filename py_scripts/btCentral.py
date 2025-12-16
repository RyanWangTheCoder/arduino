#!/usr/bin/env python3
# MyoWare BLE UART (Nordic UART Service) central — CSV logging, WinRT-safe reconnect, PC envelope
# ----------------------------------------------------------------------------------------------
# - Subscribes to Nordic UART Service TX (6E400003-... Notify) and parses ASCII lines: "ENV,RAW[,REF]\n".
# - Prints and/or plots RAW vs. PC-computed envelope (Butterworth or RMS) with shield ENV as reference.
# - Saves to CSV with --save data.csv (ts_ms, raw, env_shield, env_pc_butter, env_pc_rms, ref).
# - Windows-safe Bleak usage (characteristic object) + reconnect loop with exponential backoff.
#
# Examples:
#   python btCentral_NUS.py --name MyoW1                         # plots, no CSV
#   python btCentral_NUS.py --name MyoW1 --save emg.csv          # plots + CSV logging
#   python btCentral_NUS.py --name MyoW1 --out csv --no-plot     # console CSV only
#   python btCentral_NUS.py --name MyoW1 --fs 500 --env butter   # force fs & use Butterworth for PC envelope

import asyncio
import argparse
import time
from collections import deque
import sys
import json
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

try:
    from scipy.signal import butter, lfilter, filtfilt, get_window
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False

from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic

# Nordic UART Service UUIDs
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # central writes
NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # central subscribes (notify)

# -------------------------------------------------
# Verbose util
# -------------------------------------------------
def vprint(enabled: bool, *args, **kwargs):
    if enabled:
        print(*args, **kwargs)

# -------------------------------------------------
# Envelope estimators (streaming)
# -------------------------------------------------
class ButterLPF:
    """Streaming low-pass on rectified signal.
    If SciPy is available: use 2nd-order Butterworth IIR via lfilter with state.
    Else: use one-pole IIR approximation y[n] = (1-a)*x[n] + a*y[n-1].
    """
    def __init__(self, fs: float, cutoff: float = 3.6):
        self.fs = max(1e-3, float(fs))
        self.cutoff = float(cutoff)
        self._use_scipy = SCIPY_AVAILABLE
        self._init_filter()
        self._zi = None
        if self._use_scipy:
            # Initialize filter state for lfilter
            from scipy.signal import lfilter_zi
            self._zi = lfilter_zi(self._b, self._a) * 0.0
        self._y_prev = 0.0

    def _init_filter(self):
        if self._use_scipy:
            from scipy.signal import butter
            Wn = self.cutoff / (self.fs / 2.0)
            self._b, self._a = butter(2, Wn, btype='low')
        else:
            # one-pole IIR approximation
            self._alpha = np.exp(-2.0 * np.pi * self.cutoff / self.fs)

    def set_fs(self, fs: float):
        fs = max(1e-3, float(fs))
        if abs(fs - self.fs) / self.fs > 0.05:  # reconfigure if change >5%
            self.fs = fs
            self._init_filter()
            self._y_prev = 0.0
            if self._use_scipy:
                from scipy.signal import lfilter_zi
                self._zi = lfilter_zi(self._b, self._a) * 0.0

    def update(self, x: float) -> float:
        if self._use_scipy:
            y, self._zi = lfilter(self._b, self._a, [x], zi=self._zi)
            return float(max(y[0], 0.0))
        # one-pole IIR
        y = (1.0 - self._alpha) * x + self._alpha * self._y_prev
        self._y_prev = y
        return float(max(y, 0.0))

class RMSWindow:
    """Streaming RMS over a sliding window on rectified signal."""
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
        sq = x * x
        if len(self._buf) == self._buf.maxlen:
            old = self._buf[0]
            self._sumsq -= old * old
        self._buf.append(x)
        self._sumsq += x * x
        denom = max(1, len(self._buf))
        return float(np.sqrt(self._sumsq / denom))

# -------------------------------------------------
# Spectral features (unchanged)
# -------------------------------------------------
def median_mean_frequency(x: np.ndarray, fs: float, verbose=False) -> tuple:
    n = len(x)
    if n < 16:
        vprint(verbose, f"[spec] segment too short (n={n})")
        return (np.nan, np.nan)
    if SCIPY_AVAILABLE:
        from scipy.signal import get_window
        w = get_window('hamming', n, fftbins=True)
    else:
        w = np.hamming(n)
    X = np.fft.rfft(x * w)
    freqs = np.fft.rfftfreq(n, d=1.0/fs)
    Pxx = (np.abs(X) ** 2) / (np.sum(w**2))
    total = np.sum(Pxx)
    if total <= 0:
        vprint(verbose, "[spec] zero total power")
        return (np.nan, np.nan)
    cumsum = np.cumsum(Pxx)
    idx = np.searchsorted(cumsum, 0.5 * total)
    mdf = freqs[min(idx, len(freqs)-1)]
    mnf = float(np.sum(freqs * Pxx) / total)
    vprint(verbose, f"[spec] MDF={mdf:.1f} Hz, MNF={mnf:.1f} Hz")
    return (mdf, mnf)

# -------------------------------------------------
# Ring buffer
# -------------------------------------------------
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

# -------------------------------------------------
# Parsing helpers for NUS ASCII lines
# -------------------------------------------------
class LineParser:
    """Accumulate bytes until newline, then yield decoded text lines."""
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

# -------------------------------------------------
# Device discovery helpers
# -------------------------------------------------
async def find_device_by_name(name: str):
    # Prefer advertisement that includes NUS service UUID, else fallback to name
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
        # Fallback full scan
        devices = await BleakScanner.discover()
        return next((d for d in devices if d.name == name), None)

# -------------------------------------------------
# Main session logic with reconnect/backoff
# -------------------------------------------------
async def run_once(args, backoff_s: float):
    verbose = args.verbose
    vprint(verbose, f"[scan] discovering (backoff={backoff_s:.1f}s)...")
    target = await find_device_by_name(args.name)
    if not target:
        print("Device not found.")
        await asyncio.sleep(backoff_s)
        return False  # retry

    print(f"Connecting to: {target.address}: {target.name}")

    disconnected = asyncio.Event()

    def on_disconnect(_client: BleakClient):
        print("Disconnected.")
        disconnected.set()

    # WinRT-safe: resolve characteristic object from services
    try:
        async with BleakClient(target, disconnected_callback=on_disconnect) as client:
            vprint(verbose, f"[conn] connected={client.is_connected}")
            # Resolve characteristic object by UUID (prefer TX notify characteristic)
            tx_uuid = args.char if args.char else NUS_TX_UUID
            ch_obj = None  # type: BleakGATTCharacteristic | None
            for svc in client.services:
                for ch in svc.characteristics:
                    if str(ch.uuid).lower() == tx_uuid.lower():
                        ch_obj = ch
                        break
                if ch_obj:
                    break
            if not ch_obj:
                print("TX characteristic UUID not found in resolved services.")
                return False

            # State & buffers
            parser = LineParser()
            fs_user = float(args.fs) if args.fs > 0 else 0.0
            fs_est = float(fs_user or 100.0)
            ts_hist = deque(maxlen=200)
            # Envelope estimators (PC)
            env_butter = ButterLPF(fs=fs_est, cutoff=args.cutoff)
            env_rms    = RMSWindow(fs=fs_est, win_ms=args.win)

            # Plot buffers
            raw_ring       = Ring(maxlen=int(max(200, fs_est * 10)))   # ~10 s
            env_shield_ring= Ring(maxlen=int(max(200, fs_est * 10)))
            env_pc_ring    = Ring(maxlen=int(max(200, fs_est * 10)))   # chosen method
            window_sec = 1.0
            window_samples = int(max(50, fs_est * window_sec))
            last_feature = time.time()

            # CSV file setup
            csv_writer = None
            csv_file = None
            if args.save:
                # Open in append mode, write header if file is new/empty
                existed = os.path.exists(args.save) and os.path.getsize(args.save) > 0
                csv_file = open(args.save, 'a', newline='', encoding='utf-8')
                csv_writer = csv.writer(csv_file)
                if not existed:
                    csv_writer.writerow(['ts_ms','raw','env_shield','env_pc_butter','env_pc_rms','ref'])

            # Printing helpers
            def print_row(ts_ms: int, env_sh: float, raw: float, ref: float | None, env_bt: float, env_rm: float):
                fmt = args.out
                if fmt == 'csv':
                    cols = []
                    if args.timestamp:
                        cols.append(str(ts_ms))
                    cols.extend([f"{raw}", f"{env_sh}", f"{env_bt}", f"{env_rm}"])
                    if ref is not None:
                        cols.append(f"{ref}")
                    print(','.join(cols))
                elif fmt == 'json':
                    obj = {
                        "ts": ts_ms if args.timestamp else None,
                        "raw": raw,
                        "env_shield": env_sh,
                        "env_butter": env_bt,
                        "env_rms": env_rm,
                        "ref": ref,
                    }
                    print(json.dumps(obj, separators=(',', ':'), ensure_ascii=False))
                elif fmt == 'plain':
                    ts_part = f"[{ts_ms}] " if args.timestamp else ""
                    ref_part = f" ref={ref}" if ref is not None else ""
                    print(f"{ts_part}raw={raw} env_shield={env_sh} env_butter={env_bt} env_rms={env_rm}{ref_part}")
                sys.stdout.flush()

            # Notification handler
            def handle(_: BleakGATTCharacteristic, data: bytearray):
                nonlocal fs_est, window_samples, last_feature
                now = time.time()
                for line in parser.feed(bytes(data)):
                    parts = line.split(',')
                    if len(parts) < 2:
                        continue
                    try:
                        env_sh = float(parts[0])
                        raw = float(parts[1])
                        ref = float(parts[2]) if len(parts) >= 3 else None
                    except Exception:
                        continue

                    # Update FS estimate if not user-provided
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

                    # Compute PC envelopes from RAW (rectified)
                    rect = abs(raw)
                    env_bt = env_butter.update(rect)
                    env_rm = env_rms.update(rect)

                    # Console print
                    if args.out != 'none':
                        ts_ms = int(now * 1000)
                        print_row(ts_ms, env_sh, raw, ref, env_bt, env_rm)

                    # CSV log
                    if csv_writer:
                        csv_writer.writerow([int(now*1000), raw, env_sh, env_bt, env_rm, ref if ref is not None else ''])
                        # Flush periodically to reduce data loss on disconnect
                        if (int(now*10) % 10) == 0:
                            csv_file.flush()

                    # Plot buffers
                    if args.out == 'none' and not args.no_plot:
                        raw_ring.append_many([raw])
                        env_shield_ring.append_many([env_sh])
                        # choose PC envelope method per args.env
                        env_pc = env_bt if args.env == 'butter' else (env_rm if args.env == 'rms' else env_bt)
                        env_pc_ring.append_many([env_pc])

            # Start notifications (WinRT-safe: pass characteristic object)
            await client.start_notify(ch_obj, handle)
            print("Subscribed to NUS TX;", "printing..." if args.out != 'none' else ("starting plots..." if not args.no_plot else "plots disabled."))

            # PRINT-ONLY PATH
            if args.out != 'none' or args.no_plot:
                try:
                    while not disconnected.is_set():
                        await asyncio.sleep(0.5)
                except KeyboardInterrupt:
                    pass
                finally:
                    try:
                        await client.stop_notify(ch_obj)
                    except Exception:
                        pass
                    if csv_file:
                        csv_file.flush(); csv_file.close()
                return True  # finished session

            # PLOTTING PATH
            plt.style.use('seaborn-v0_8')
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=False)
            try:
                fig.canvas.manager.set_window_title('MyoWare RAW + PC Envelope + Shield ENV (NUS)')
            except Exception:
                pass

            # Top plot: RAW, PC envelope (selected), shield ENV (reference)
            raw_line,      = ax1.plot([], [], color='steelblue', lw=0.9, label='RAW')
            env_pc_line,   = ax1.plot([], [], color='crimson',   lw=1.6, label=f"Envelope (PC: {args.env})")
            env_ref_line,  = ax1.plot([], [], color='gray',      lw=1.0, ls='--', label='Envelope (shield)')
            ax1.set_title('EMG RAW & Envelopes (last 5 s)')
            ax1.set_ylabel('Amplitude')
            ax1.legend(loc='upper right')

            # Bottom plot: spectral features on RAW
            mdf_line, = ax2.plot([], [], color='green',  lw=1.2, label='Median Freq (MDF)')
            mnf_line, = ax2.plot([], [], color='orange', lw=1.2, label='Mean Freq (MNF)')
            ax2.set_title('Running Spectral Features (1 s windows)')
            ax2.set_ylabel('Frequency (Hz)')
            ax2.set_xlabel('Time (updates)')
            ax2.legend(loc='upper right')
            ax2.set_ylim(0, 500)

            mdf_trend = deque(maxlen=600)
            mnf_trend = deque(maxlen=600)

            def update(_frame):
                fs = fs_user if fs_user > 0 else fs_est
                disp_sec = 5.0
                n_disp = int(max(10, fs * disp_sec))
                raw = raw_ring.view_latest(n_disp)
                env_ref = env_shield_ring.view_latest(n_disp)
                env_pc = env_pc_ring.view_latest(n_disp)
                t = np.arange(len(raw)) / max(1e-3, fs)
                if len(raw) > 0:
                    raw_line.set_data(t, raw)
                    env_pc_line.set_data(t, env_pc)
                    env_ref_line.set_data(t, env_ref)
                    ax1.set_xlim(max(0, (t[-1] - disp_sec)), t[-1] if len(t) > 0 else disp_sec)
                    y_max = max(1.0, float(np.max(raw)) * 1.1)
                    ax1.set_ylim(0, y_max)

                    # Feature update every ~0.2 s using last 1 s of RAW
                    nonlocal last_feature, window_samples
                    now_t = time.time()
                    if now_t - last_feature > 0.2:
                        seg = raw_ring.view_latest(window_samples)
                        if len(seg) >= window_samples:
                            mdf, mnf = median_mean_frequency(seg - np.mean(seg), fs=fs, verbose=args.verbose)
                            mdf_trend.append(mdf)
                            mnf_trend.append(mnf)
                        x_feat = np.arange(len(mdf_trend))
                        if len(mdf_trend) > 0:
                            mdf_line.set_data(x_feat, np.asarray(mdf_trend))
                        if len(mnf_trend) > 0:
                            mnf_line.set_data(x_feat, np.asarray(mnf_trend))
                        ax2.set_xlim(max(0, len(mdf_trend) - 600), max(len(mdf_trend), 50))
                        last_feature = now_t
                return raw_line, env_pc_line, env_ref_line, mdf_line, mnf_line

            ani = FuncAnimation(fig, update, interval=100, blit=False)
            plt.tight_layout()

            # Run until disconnect or window closed
            try:
                plt.show()
            except KeyboardInterrupt:
                pass
            finally:
                try:
                    await client.stop_notify(ch_obj)
                except Exception:
                    pass
                if csv_file:
                    csv_file.flush(); csv_file.close()
            return True
    except Exception as e:
        print(f"Session error: {e}")
        await asyncio.sleep(backoff_s)
        return False

async def run(args):
    # Reconnect loop with exponential backoff
    base = 1.0
    max_backoff = 30.0
    backoff = base
    try:
        while True:
            ok = await run_once(args, backoff)
            if ok:
                # Reset backoff after a successful session
                backoff = base
            else:
                backoff = min(max_backoff, backoff * 2.0)
    except KeyboardInterrupt:
        print("Interrupted by user.")

# -------------------------------------------------
# CLI
# -------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="BLE central for MyoWare ENV/RAW ASCII over Nordic UART Service with CSV logging & reconnect")
    parser.add_argument('--name', type=str, required=True, help='BLE local name (e.g., MyoW1)')
    parser.add_argument('--char', type=str, default='', help='Characteristic UUID to subscribe to (default: NUS TX)')
    parser.add_argument('--fs', type=float, default=0.0, help='Sampling rate on peripheral (Hz). 0=estimate from timing')
    parser.add_argument('--env', type=str, choices=['butter','rms'], default='butter', help='PC envelope method for plotting (both are logged)')
    parser.add_argument('--cutoff', type=float, default=3.6, help='LPF cutoff for Butterworth envelope (Hz)')
    parser.add_argument('--win', type=float, default=50.0, help='RMS window length (ms)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose debug prints')
    # Printing & control
    parser.add_argument('--out', type=str, choices=['none','plain','csv','json'], default='none', help='Console output format (none=disabled)')
    parser.add_argument('--timestamp', action='store_true', help='Include epoch ms timestamp in output lines')
    parser.add_argument('--no-plot', action='store_true', help='Disable plots (useful with --out)')
    parser.add_argument('--save', type=str, default='', help='CSV file path to append samples (e.g., data.csv)')
    args = parser.parse_args()
    asyncio.run(run(args))

if __name__ == '__main__':
    main()
