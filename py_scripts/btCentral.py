#!/usr/bin/env python3
"""
MyoWare RAW EMG BLE central (Bleak) — verbose + print-integrated (WinRT-safe)
- Subscribes to 20-byte RAW packets: [seq (1)] + [9x uint16 LE (18)] + [flags (1)].
- Optional console printing of raw integers (plain/csv/json) with --out.
- Optional plots: RAW+envelope (top) and MDF/MNF (bottom) unless --no-plot.
- Uses BleakGATTCharacteristic object for start/stop_notify (safer on Windows).
- Avoids double stop_notify to prevent WinRT KeyError.
"""

import asyncio
import argparse
import time
from collections import deque
import sys
import json

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

try:
    from scipy.signal import butter, filtfilt, get_window
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False

from bleak import BleakScanner, BleakClient

# -----------------------------
# Verbose util
# -----------------------------

def vprint(enabled: bool, *args, **kwargs):
    if enabled:
        print(*args, **kwargs)

# -----------------------------
# Envelope
# -----------------------------

def envelope_butter(rect: np.ndarray, fs: float, cutoff_hz: float = 3.6, verbose=False) -> np.ndarray:
    vprint(verbose, f"[env] Butterworth LPF cutoff={cutoff_hz} Hz, fs={fs} Hz, scipy={SCIPY_AVAILABLE}")
    if SCIPY_AVAILABLE:
        from scipy.signal import butter, filtfilt
        Wn = cutoff_hz / (fs / 2.0)
        b, a = butter(2, Wn, btype='low')
        env = filtfilt(b, a, rect)
        return np.maximum(env, 0.0)
    # Fallback: moving average ~250 ms
    win = int(max(1, round(0.25 * fs)))
    vprint(verbose, f"[env] Fallback moving-average window={win} samples (~{win/fs:.3f}s)")
    kernel = np.ones(win, dtype=float) / win
    return np.convolve(rect, kernel, mode='same')


def envelope_rms(rect: np.ndarray, fs: float, win_ms: float = 50.0, verbose=False) -> np.ndarray:
    win = int(max(1, round(win_ms * fs / 1000.0)))
    vprint(verbose, f"[env] RMS window={win} samples ({win_ms} ms)")
    power = rect ** 2
    kernel = np.ones(win, dtype=float) / win
    return np.sqrt(np.convolve(power, kernel, mode='same'))

# -----------------------------
# Spectral features
# -----------------------------

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
    cumsum = np.cumsum(Pxx)
    total = cumsum[-1]
    if total <= 0:
        vprint(verbose, "[spec] zero total power")
        return (np.nan, np.nan)
    idx = np.searchsorted(cumsum, 0.5 * total)
    mdf = freqs[min(idx, len(freqs)-1)]
    mnf = float(np.sum(freqs * Pxx) / total)
    vprint(verbose, f"[spec] MDF={mdf:.1f} Hz, MNF={mnf:.1f} Hz")
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
    verbose = args.verbose
    vprint(verbose, f"[cfg] name={args.name}, char={args.char}, fs={args.fs}, env={args.env}, cutoff={args.cutoff}, win={args.win}, out={args.out}, timestamp={args.timestamp}, max={args.max}, no_plot={args.no_plot}")

    # Discovery
    vprint(verbose, "[scan] starting discovery...")
    devices = await BleakScanner.discover()
    target = next((d for d in devices if d.name == args.name), None)
    if not target:
        print("Device not found. Available devices:")
        for d in devices:
            print(" -", d)
        return

    vprint(verbose, f"[scan] found {len(devices)} devices; connecting to {target}")
    print(f"Connecting to: {target.address}: {target.name}")
    async with BleakClient(target) as client:
        vprint(verbose, f"[conn] connected={client.is_connected}")
        vprint(verbose, f"[conn] client services ready? {'yes' if client.services else 'no'}")
        print("Connected. Subscribing to characteristic:", args.char)

        # Resolve characteristic object by UUID (safer than UUID-only on Windows)
        ch_obj = None
        for svc in client.services:
            for ch in svc.characteristics:
                if str(ch.uuid).lower() == args.char.lower():
                    ch_obj = ch
                    break
            if ch_obj:
                break
        if not ch_obj:
            print("Characteristic UUID not found in resolved services.")
            return

        # State
        seq_expected = None
        dropped = 0
        packets = 0
        last_stats_t = time.time()
        stop_event = asyncio.Event()

        # Buffers for plotting path
        samples_per_sec = args.fs
        raw_ring = Ring(maxlen=int(samples_per_sec * 10))
        env_ring = Ring(maxlen=int(samples_per_sec * 2))
        mdf_trend = deque(maxlen=600)
        mnf_trend = deque(maxlen=600)
        window_sec = 1.0
        window_samples = int(samples_per_sec * window_sec)
        last_feature = time.time()

        # Printing helpers
        def print_packet(ts_ms: int, seq: int, flags: int, samples):
            fmt = args.out
            if fmt == 'csv':
                row = ([str(ts_ms)] if args.timestamp else []) + [str(seq), str(flags)] + [str(int(s)) for s in samples]
                print(','.join(row))
            elif fmt == 'json':
                obj = {
                    "ts": ts_ms if args.timestamp else None,
                    "seq": seq,
                    "flags": flags,
                    "samples": [int(s) for s in samples],
                }
                print(json.dumps(obj, separators=(',', ':'), ensure_ascii=False))
            elif fmt == 'plain':
                ts_part = f"[{ts_ms}] " if args.timestamp else ""
                print(f"{ts_part}seq={seq} flags={flags} samples: " + ' '.join(str(int(s)) for s in samples))
            sys.stdout.flush()

        # Notification handler (shared for both modes)
        def handle(_, data: bytearray):
            nonlocal seq_expected, dropped, packets, last_stats_t
            decoded = decode_packet(data)
            if not decoded:
                return
            seq, samples, flags = decoded
            packets += 1

            # Sequence loss detection
            if seq_expected is None:
                seq_expected = (seq + 1) % 256
            else:
                if seq != seq_expected:
                    delta = (seq - seq_expected) % 256
                    if delta != 0:
                        dropped += delta
                seq_expected = (seq + 1) % 256

            # Print mode
            if args.out != 'none':
                ts_ms = int(time.time() * 1000)
                print_packet(ts_ms, seq, flags, samples)
                # Stop after max packets if requested: signal only; do NOT stop_notify here
                if args.max and packets >= args.max:
                    stop_event.set()
                    return

            # Plot mode (only if not printing and not explicitly disabled)
            if args.out == 'none' and not args.no_plot:
                raw_ring.append_many(samples)
                env_ring.append_many(samples)

            # Periodic debug stats (~2 s)
            now = time.time()
            if verbose and (now - last_stats_t) > 2.0:
                interval = now - last_stats_t
                est_sps = (packets * 9) / interval  # 9 samples per packet
                print(f"[rx] packets={packets} dropped_seq={dropped} est_samples_per_sec={est_sps:.1f}")
                packets = 0
                last_stats_t = now

        await client.start_notify(ch_obj, handle)
        print("Subscribed;", "printing raw integers..." if args.out != 'none' else ("starting plots..." if not args.no_plot else "plots disabled."))

        # -------- Execution paths --------
        if args.out != 'none' or args.no_plot:
            # PRINT-ONLY PATH (no plots)
            try:
                # Wait until stop_event set (max packets) or Ctrl-C
                while not stop_event.is_set():
                    await asyncio.sleep(0.5)
            except KeyboardInterrupt:
                pass
            finally:
                try:
                    await client.stop_notify(ch_obj)
                except KeyError:
                    # Already stopped or callback missing; ignore on Windows
                    pass
                except Exception:
                    pass
                print(f"Done. dropped_seq={dropped}")
                return

        # PLOTTING PATH
        plt.style.use('seaborn-v0_8')
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=False)
        try:
            fig.canvas.manager.set_window_title('MyoWare RAW + Envelope + Spectral Features')
        except Exception:
            pass

        # Top plot
        raw_line, = ax1.plot([], [], color='steelblue', lw=0.8, label='RAW (ADC units)')
        env_line, = ax1.plot([], [], color='crimson', lw=1.2, label='Envelope')
        ax1.set_title('EMG RAW & Envelope (last 3 s)')
        ax1.set_ylabel('Amplitude')
        ax1.legend(loc='upper right')

        # Bottom plot
        mdf_line, = ax2.plot([], [], color='green', lw=1.2, label='Median Freq (MDF)')
        mnf_line, = ax2.plot([], [], color='orange', lw=1.2, label='Mean Freq (MNF)')
        ax2.set_title('Running Spectral Features (1 s windows)')
        ax2.set_ylabel('Frequency (Hz)')
        ax2.set_xlabel('Time (updates)')
        ax2.legend(loc='upper right')
        ax2.set_ylim(0, 500)

        def update(_frame):
            # RAW window for display
            disp_sec = 3.0
            n_disp = int(samples_per_sec * disp_sec)
            raw = raw_ring.view_latest(n_disp)
            t = np.arange(len(raw)) / samples_per_sec

            if len(raw) > 0:
                rect = np.abs(raw)
                # Envelope
                if args.env == 'rms':
                    env = envelope_rms(rect, fs=samples_per_sec, win_ms=args.win, verbose=verbose)
                else:
                    env = envelope_butter(rect, fs=samples_per_sec, cutoff_hz=args.cutoff, verbose=verbose)

                raw_line.set_data(t, raw)
                env_line.set_data(t, env[-len(raw):])
                ax1.set_xlim(max(0, t[-1] - disp_sec), t[-1] if len(t) > 0 else disp_sec)
                y_max = max(1.0, float(np.max(raw)) * 1.1)
                ax1.set_ylim(0, y_max)

            # Feature update every ~0.1 s using last 1 s raw
            now = time.time()
            if now - last_feature > 0.1:
                seg = raw_ring.view_latest(window_samples)
                if len(seg) >= window_samples:
                    mdf, mnf = median_mean_frequency(seg - np.mean(seg), fs=samples_per_sec, verbose=verbose)
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

        try:
            await client.stop_notify(ch_obj)
        except KeyError:
            pass
        except Exception:
            pass
        print("Stopped notifications.")


def main():
    parser = argparse.ArgumentParser(description="BLE central for MyoWare RAW EMG packets with processing/plots/printing")
    parser.add_argument('--name', type=str, required=True, help='BLE local name (e.g., MyoWareSensor1)')
    parser.add_argument('--char', type=str, required=True, help='Characteristic UUID to subscribe to')
    parser.add_argument('--fs', type=int, default=1000, help='Sampling rate on peripheral (Hz)')
    parser.add_argument('--env', type=str, choices=['butter','rms'], default='butter', help='Envelope method')
    parser.add_argument('--cutoff', type=float, default=3.6, help='LPF cutoff for butter envelope (Hz)')
    parser.add_argument('--win', type=float, default=50.0, help='RMS window length (ms)')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose debug prints')
    # Printing & control
    parser.add_argument('--out', type=str, choices=['none','plain','csv','json'], default='none',
                        help='Console output format for raw samples (none=disabled)')
    parser.add_argument('--timestamp', action='store_true', help='Include epoch ms timestamp in output lines')
    parser.add_argument('--max', type=int, default=0, help='Stop after N packets (0=unlimited)')
    parser.add_argument('--no-plot', action='store_true', help='Disable plots (useful with --out)')

    args = parser.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("Interrupted by user.")


if __name__ == '__main__':
    main()
