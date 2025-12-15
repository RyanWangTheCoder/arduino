#!/usr/bin/env python3
"""
MyoWare RAW EMG BLE central (Bleak)
- Subscribes to 20-byte RAW packets: [seq (1)] + [9x uint16 LE (18)] + [flags (1)].
- Builds linear envelope (rectify + Butterworth LPF ~3.6 Hz) or moving RMS.
- Computes Median/Mean Frequency over 1-s windows.
- Live plots: RAW+envelope (top) and MDF/MNF (bottom).

Usage:
  python btCentral.py --name MyoWareSensor1 --char <UUID> --fs 1000 --env butter
  python btCentral.py --name MyoWareSensor1 --char <UUID> --fs 1000 --env rms --win 50
"""

import asyncio
import argparse
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation  # import FuncAnimation correctly

try:
    from scipy.signal import butter, filtfilt, get_window
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False

from bleak import BleakScanner, BleakClient

# -----------------------------
# Envelope
# -----------------------------

def envelope_butter(rect: np.ndarray, fs: float, cutoff_hz: float = 3.6) -> np.ndarray:
    if SCIPY_AVAILABLE:
        from scipy.signal import butter, filtfilt
        Wn = cutoff_hz / (fs / 2.0)
        b, a = butter(2, Wn, btype='low')
        env = filtfilt(b, a, rect)
        return np.maximum(env, 0.0)
    # Fallback: simple moving average ~250 ms
    win = int(max(1, round(0.25 * fs)))
    kernel = np.ones(win, dtype=float) / win
    return np.convolve(rect, kernel, mode='same')


def envelope_rms(rect: np.ndarray, fs: float, win_ms: float = 50.0) -> np.ndarray:
    win = int(max(1, round(win_ms * fs / 1000.0)))
    power = rect ** 2
    kernel = np.ones(win, dtype=float) / win
    return np.sqrt(np.convolve(power, kernel, mode='same'))

# -----------------------------
# Spectral features
# -----------------------------

def median_mean_frequency(x: np.ndarray, fs: float) -> tuple:
    n = len(x)
    if n < 16:
        return (np.nan, np.nan)
    if SCIPY_AVAILABLE:
        from scipy.signal import get_window
        w = get_window('hamming', n, fftbins=True)
    else:
        w = np.hamming(n)
    X = np.fft.rfft(x * w)
    freqs = np.fft.rfftfreq(n, d=1.0/fs)
    Pxx = (np.abs(X) ** 2) / (np.sum(w**2))
    cumsum = np.cumsum(Pxx)
    total = cumsum[-1]
    if total <= 0:
        return (np.nan, np.nan)
    idx = np.searchsorted(cumsum, 0.5 * total)
    mdf = freqs[min(idx, len(freqs)-1)]
    mnf = float(np.sum(freqs * Pxx) / total)
    return (mdf, mnf)

# -----------------------------
# Packet decode
# -----------------------------

def decode_packet(payload: bytes):
    if len(payload) != 20:
        return None
    seq = payload[0]
    flags = payload[19]
    samples = list(np.frombuffer(payload[1:19], dtype='<u2'))
    return seq, samples, flags

# -----------------------------
# Ring buffer
# -----------------------------

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

# -----------------------------
# Main
# -----------------------------

async def run(args):
    print(f"Scanning for device named '{args.name}' ...")
    devices = await BleakScanner.discover()
    target = next((d for d in devices if d.name == args.name), None)
    if not target:
        print("Device not found. Available devices:")
        for d in devices:
            print(" -", d)
        return

    print("Connecting to:", target)
    async with BleakClient(target) as client:
        print("Connected. Subscribing to characteristic:", args.char)
        seq_expected = None
        dropped = 0

        samples_per_sec = args.fs
        raw_ring = Ring(maxlen=int(samples_per_sec * 10))
        env_ring = Ring(maxlen=int(samples_per_sec * 2))
        mdf_trend = deque(maxlen=600)
        mnf_trend = deque(maxlen=600)

        window_sec = 1.0
        window_samples = int(samples_per_sec * window_sec)
        last_feature = time.time()

        def handle(_, data: bytearray):
            nonlocal seq_expected, dropped
            decoded = decode_packet(data)
            if not decoded:
                return
            seq, samples, flags = decoded
            if seq_expected is None:
                seq_expected = (seq + 1) % 256
            else:
                if seq != seq_expected:
                    delta = (seq - seq_expected) % 256
                    if delta != 0:
                        dropped += delta
                seq_expected = (seq + 1) % 256
            raw_ring.append_many(samples)
            env_ring.append_many(samples)

        await client.start_notify(args.char, handle)
        print("Subscribed; starting plots...")

        plt.style.use('seaborn-v0_8')
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=False)
        try:
            fig.canvas.manager.set_window_title('MyoWare RAW + Envelope + Spectral Features')
        except Exception:
            pass

        raw_line, = ax1.plot([], [], color='steelblue', lw=0.8, label='RAW (ADC units)')
        env_line, = ax1.plot([], [], color='crimson', lw=1.2, label='Envelope')
        ax1.set_title('EMG RAW & Envelope (last 3 s)')
        ax1.set_ylabel('Amplitude')
        ax1.legend(loc='upper right')

        mdf_line, = ax2.plot([], [], color='green', lw=1.2, label='Median Freq (MDF)')
        mnf_line, = ax2.plot([], [], color='orange', lw=1.2, label='Mean Freq (MNF)')
        ax2.set_title('Running Spectral Features (1 s windows)')
        ax2.set_ylabel('Frequency (Hz)')
        ax2.set_xlabel('Time (updates)')
        ax2.legend(loc='upper right')
        ax2.set_ylim(0, 500)

        def update(_frame):
            disp_sec = 3.0
            n_disp = int(samples_per_sec * disp_sec)
            raw = raw_ring.view_latest(n_disp)
            t = np.arange(len(raw)) / samples_per_sec

            if len(raw) > 0:
                rect = np.abs(raw)
                if args.env == 'rms':
                    env = envelope_rms(rect, fs=samples_per_sec, win_ms=args.win)
                else:
                    env = envelope_butter(rect, fs=samples_per_sec, cutoff_hz=args.cutoff)

                raw_line.set_data(t, raw)
                env_line.set_data(t, env[-len(raw):])
                ax1.set_xlim(max(0, t[-1] - disp_sec), t[-1] if len(t) > 0 else disp_sec)
                y_max = max(1.0, float(np.max(raw)) * 1.1)
                ax1.set_ylim(0, y_max)

            now = time.time()
            if now - last_feature > 0.1:
                seg = raw_ring.view_latest(window_samples)
                if len(seg) >= window_samples:
                    mdf, mnf = median_mean_frequency(seg - np.mean(seg), fs=samples_per_sec)
                    mdf_trend.append(mdf)
                    mnf_trend.append(mnf)
                x_feat = np.arange(len(mdf_trend))
                if len(mdf_trend) > 0:
                    mdf_line.set_data(x_feat, np.asarray(mdf_trend))
                if len(mnf_trend) > 0:
                    mnf_line.set_data(x_feat, np.asarray(mnf_trend))
                ax2.set_xlim(max(0, len(mdf_trend) - 600), max(len(mdf_trend), 50))

            fig.suptitle(f"Seq drops: {dropped}")
            return raw_line, env_line, mdf_line, mnf_line

        ani = FuncAnimation(fig, update, interval=100, blit=False)
        plt.tight_layout()
        plt.show()

        await client.stop_notify(args.char)
        print("Stopped notifications.")


def main():
    parser = argparse.ArgumentParser(description="BLE central for MyoWare RAW EMG packets with processing & plots")
    parser.add_argument('--name', type=str, required=True, help='BLE local name (e.g., MyoWareSensor1)')
    parser.add_argument('--char', type=str, required=True, help='Characteristic UUID to subscribe to')
    parser.add_argument('--fs', type=int, default=1000, help='Sampling rate on peripheral (Hz)')
    parser.add_argument('--env', type=str, choices=['butter','rms'], default='butter', help='Envelope method')
    parser.add_argument('--cutoff', type=float, default=3.6, help='LPF cutoff for butter envelope (Hz)')
    parser.add_argument('--win', type=float, default=50.0, help='RMS window length (ms)')
    args = parser.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("Interrupted by user.")


if __name__ == '__main__':
    main()
