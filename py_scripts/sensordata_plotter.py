#!/usr/bin/env python3
"""sensordata_plotter_fixed.py — Realtime UDP plotter (device-time aligned) + offline viewer.

Improvements vs previous device-time plotter:
  1) Works even if started AFTER btCentral: if emg_meta is missed, it infers sr_hz from dev_us deltas.
  2) Still uses emg_meta when available and will reinitialize LPF if sr_hz changes.

UDP messages expected (JSON):
  EMG meta (periodic or on change):
    {"k":"emg_meta", "sr_hz":<int>, "ch":<int>, "adc_bits":<int>, "ymax":<int>, "seg":<int>}
  EMG samples:
    {"k":"raw", "dev_us":<int>, "raw":<int>, "seg":<int>, "t":<host_ms optional>}
    {"k":"env", "dev_us":<int>, "env":<int>, "seg":<int>, "t":<host_ms optional>}
  IMU samples:
    {"k":"imu", "dev_us":<int>, "amag":<float>, "seg":<int>, "t":<host_ms optional>}

Plots (live):
  - EMG subplot: RAW (absolute), ENV_device, ENV_lpf(abs(RAW))
  - IMU subplot: accel magnitude (g)

Time axis (live):
  - Uses dev_us for BOTH EMG and IMU for true alignment.
"""

import argparse
import csv
import json
import socket
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description='Realtime UDP plotter (device-time aligned, robust meta)')
    ap.add_argument('--mode', choices=['live', 'offline'], default='live')
    ap.add_argument('--listen-host', default='127.0.0.1')
    ap.add_argument('--listen-port', type=int, default=9999)

    ap.add_argument('--emg-window-sec', type=float, default=2.0)
    ap.add_argument('--imu-window-sec', type=float, default=None)
    ap.add_argument('--fps', type=float, default=30.0)

    ap.add_argument('--emg-ymax', type=float, default=4095.0)
    ap.add_argument('--cutoff-hz', type=float, default=3.6)

    # Meta inference
    ap.add_argument('--infer-sr', action='store_true', help='Infer sr_hz from dev_us if emg_meta missed')
    ap.add_argument('--infer-min-hz', type=float, default=100.0)
    ap.add_argument('--infer-max-hz', type=float, default=4000.0)
    ap.add_argument('--infer-min-samples', type=int, default=30)

    # Offline
    ap.add_argument('--emg-file', default=None)
    ap.add_argument('--max-points', type=int, default=400000)
    ap.add_argument('--sr-hz', type=float, default=2000.0)
    return ap.parse_args()


class ButterLPF:
    def __init__(self, fs_hz: float, cutoff_hz: float = 3.6):
        from scipy.signal import butter, lfilter_zi
        self.fs_hz = float(fs_hz)
        self.cutoff_hz = float(cutoff_hz)
        self.b, self.a = butter(2, self.cutoff_hz / (self.fs_hz / 2.0), btype='low')
        self.zi = lfilter_zi(self.b, self.a) * 0.0

    def update(self, x: float) -> float:
        from scipy.signal import lfilter
        y, self.zi = lfilter(self.b, self.a, [float(x)], zi=self.zi)
        return float(max(y[0], 0.0))


def infer_sr_from_dev_us(dev_us_list, min_hz: float, max_hz: float) -> float | None:
    if len(dev_us_list) < 3:
        return None
    diffs = np.diff(np.array(dev_us_list, dtype=np.int64))
    diffs = diffs[diffs > 0]
    if diffs.size < 3:
        return None
    dt_us = float(np.median(diffs))
    if dt_us <= 0:
        return None
    sr = 1_000_000.0 / dt_us
    if sr < min_hz or sr > max_hz:
        return None
    # Round to a sensible integer (common rates)
    return float(int(round(sr)))


def run_live(args: argparse.Namespace) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen_host, int(args.listen_port)))
    sock.setblocking(False)

    raw_buf = deque()
    env_dev_buf = deque()
    env_lpf_buf = deque()
    imu_buf = deque()

    # Track recent raw dev_us to infer sr_hz if needed
    recent_raw_dev_us = deque(maxlen=200)

    t0_dev_us = None
    emg_sr_hz = None
    emg_ch = None
    env_filter = None

    emg_win = float(args.emg_window_sec)
    imu_win = float(args.imu_window_sec) if args.imu_window_sec is not None else emg_win

    fig, (ax_emg, ax_imu) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    (ln_raw,) = ax_emg.plot([], [], lw=0.8, label='RAW')
    (ln_env_dev,) = ax_emg.plot([], [], lw=1.2, label='ENV_device')
    (ln_env_lpf,) = ax_emg.plot([], [], lw=1.4, label='ENV_lpf(abs(RAW))')

    ax_emg.set_title('MyoWare Realtime (UDP, dev_us timebase)')
    ax_emg.set_ylabel('ADC (counts)')
    ax_emg.set_ylim(0, float(args.emg_ymax))
    ax_emg.grid(True, alpha=0.3)
    ax_emg.legend(loc='upper right')

    (ln_imu,) = ax_imu.plot([], [], lw=1.2, label='acc_mag (g)')
    ax_imu.set_ylabel('g')
    ax_imu.set_xlabel('Time (s) (device dev_us)')
    ax_imu.grid(True, alpha=0.3)
    ax_imu.legend(loc='upper right')

    status = ax_emg.text(0.01, 0.98, '', transform=ax_emg.transAxes, va='top')

    def prune(now_t: float) -> None:
        while raw_buf and (now_t - raw_buf[0][0]) > emg_win:
            raw_buf.popleft()
        while env_dev_buf and (now_t - env_dev_buf[0][0]) > emg_win:
            env_dev_buf.popleft()
        while env_lpf_buf and (now_t - env_lpf_buf[0][0]) > emg_win:
            env_lpf_buf.popleft()
        while imu_buf and (now_t - imu_buf[0][0]) > imu_win:
            imu_buf.popleft()

    def update(_):
        nonlocal t0_dev_us, emg_sr_hz, emg_ch, env_filter
        last_seg = None
        rx_count = 0

        while True:
            try:
                data, _addr = sock.recvfrom(8192)
            except BlockingIOError:
                break
            except Exception:
                break

            try:
                msg = json.loads(data.decode('utf-8'))
            except Exception:
                continue

            k = msg.get('k')
            seg = msg.get('seg')
            if seg is not None:
                last_seg = seg

            if k == 'emg_meta':
                sr = msg.get('sr_hz')
                ch = msg.get('ch')
                if sr is not None:
                    sr = float(sr)
                    if (emg_sr_hz is None) or (float(emg_sr_hz) != sr):
                        emg_sr_hz = sr
                        emg_ch = ch
                        env_filter = ButterLPF(fs_hz=emg_sr_hz, cutoff_hz=float(args.cutoff_hz))
                        print(f'[Plotter] EMG meta: sr_hz={emg_sr_hz:.1f} ch={emg_ch}')
                continue

            dev_us = msg.get('dev_us')
            if dev_us is None:
                continue
            dev_us = int(dev_us)
            if t0_dev_us is None:
                t0_dev_us = dev_us
            t_sec = (dev_us - t0_dev_us) / 1_000_000.0

            if k == 'raw':
                r = msg.get('raw')
                if r is None:
                    continue
                r = float(r)
                raw_buf.append((t_sec, r))
                recent_raw_dev_us.append(dev_us)

                # Infer sr_hz if meta was missed
                if env_filter is None and args.infer_sr and len(recent_raw_dev_us) >= int(args.infer_min_samples):
                    sr_inf = infer_sr_from_dev_us(list(recent_raw_dev_us), float(args.infer_min_hz), float(args.infer_max_hz))
                    if sr_inf is not None:
                        emg_sr_hz = sr_inf
                        env_filter = ButterLPF(fs_hz=emg_sr_hz, cutoff_hz=float(args.cutoff_hz))
                        print(f'[Plotter] Inferred EMG sr_hz={emg_sr_hz:.0f} (no meta yet)')

                if env_filter is not None:
                    env_lpf_buf.append((t_sec, env_filter.update(abs(r))))

            elif k == 'env':
                e = msg.get('env')
                if e is None:
                    continue
                env_dev_buf.append((t_sec, float(e)))

            elif k == 'imu':
                a = msg.get('amag')
                if a is None:
                    continue
                imu_buf.append((t_sec, float(a)))

            rx_count += 1

        now_t = 0.0
        for b in (raw_buf, env_dev_buf, env_lpf_buf, imu_buf):
            if b:
                now_t = max(now_t, b[-1][0])
        prune(now_t)

        if raw_buf:
            xs = np.array([p[0] for p in raw_buf], dtype=float)
            ys = np.array([p[1] for p in raw_buf], dtype=float)
            ln_raw.set_data(xs, ys)
        if env_dev_buf:
            xs = np.array([p[0] for p in env_dev_buf], dtype=float)
            ys = np.array([p[1] for p in env_dev_buf], dtype=float)
            ln_env_dev.set_data(xs, ys)
        if env_lpf_buf:
            xs = np.array([p[0] for p in env_lpf_buf], dtype=float)
            ys = np.array([p[1] for p in env_lpf_buf], dtype=float)
            ln_env_lpf.set_data(xs, ys)

        if imu_buf:
            xs = np.array([p[0] for p in imu_buf], dtype=float)
            ys = np.array([p[1] for p in imu_buf], dtype=float)
            ln_imu.set_data(xs, ys)
            lo, hi = float(np.min(ys)), float(np.max(ys))
            if hi <= lo:
                hi = lo + 0.1
            pad = 0.1 * (hi - lo)
            ax_imu.set_ylim(lo - pad, hi + pad)

        ax_imu.set_xlim(max(0.0, now_t - max(emg_win, imu_win)), now_t)

        seg_str = f'seg={last_seg}' if last_seg is not None else 'seg=?'
        sr_str = f'sr={emg_sr_hz:.0f}Hz' if emg_sr_hz else 'sr=?'
        status.set_text(f'UDP {args.listen_host}:{args.listen_port}  {seg_str}  {sr_str}  rx={rx_count}')
        return ln_raw, ln_env_dev, ln_env_lpf, ln_imu, status

    interval_ms = max(10, int(1000.0 / float(args.fps)))
    _ani = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


def run_offline(args: argparse.Namespace) -> None:
    if not args.emg_file:
        raise SystemExit('Offline mode requires --emg-file')

    t_ms, raw, env = [], [], []
    with open(args.emg_file, 'r', encoding='utf-8') as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                t = int(row.get('host_rx_ms', '0'))
                rw = float(row.get('raw', 'nan'))
                ev = float(row.get('env', 'nan'))
            except Exception:
                continue
            t_ms.append(t)
            raw.append(rw)
            env.append(ev)
            if len(t_ms) >= int(args.max_points):
                break

    if not t_ms:
        raise SystemExit('No data read from EMG file')

    t0 = t_ms[0]
    xs = np.array([(t - t0) / 1000.0 for t in t_ms], dtype=float)
    raw = np.array(raw, dtype=float)
    env = np.array(env, dtype=float)

    env_filter = ButterLPF(fs_hz=float(args.sr_hz), cutoff_hz=float(args.cutoff_hz))
    env_lpf = np.array([env_filter.update(abs(x)) for x in raw], dtype=float)

    fig, ax = plt.subplots(1, 1, figsize=(11, 5))
    ax.plot(xs, raw, lw=0.8, label='RAW')
    ax.plot(xs, env, lw=1.0, label='ENV_device')
    ax.plot(xs, env_lpf, lw=1.2, label='ENV_lpf(abs(RAW))')
    ax.set_title(f'Offline EMG: {args.emg_file}')
    ax.set_xlabel('Time (s) (host_rx_ms)')
    ax.set_ylabel('ADC (counts)')
    ax.set_ylim(0, float(args.emg_ymax))
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.show()


def main() -> None:
    args = parse_args()
    if args.mode == 'live':
        run_live(args)
    else:
        run_offline(args)


if __name__ == '__main__':
    main()
