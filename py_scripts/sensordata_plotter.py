#!/usr/bin/env python3
"""sensordata_plotter.py — Realtime UDP plotter (device-time aligned) + offline viewer + EMG fatigue.

Improvements vs previous device-time plotter:
  1) Works even if started AFTER btCentral: if emg_meta is missed, it infers sr_hz from dev_us deltas.
  2) Still uses emg_meta when available and will reinitialize LPF if sr_hz changes.
  3) Optional real-time EMG fatigue monitoring via FatigueMonitor (--fatigue flag).

UDP messages expected (JSON):
  EMG meta:  {"k":"emg_meta", "sr_hz":<int>, "ch":<int>, "adc_bits":<int>, "ymax":<int>, "seg":<int>}
  EMG raw:   {"k":"raw",  "dev_us":<int>, "raw":<int>,   "seg":<int>, "t":<host_ms optional>}
  EMG env:   {"k":"env",  "dev_us":<int>, "env":<int>,   "seg":<int>, "t":<host_ms optional>}
  IMU:       {"k":"imu",  "dev_us":<int>, "amag":<float>, "gmag":<float>, "seg":<int>}
               amag = accel magnitude (g), gmag = gyro magnitude (dps) -- added by btCentral.py
  Vitals:    {"k":"vitals", "pct":<int>, "v":<float>, "spo2":<int>, "hr":<float>, "seg":<int>}

Fatigue monitoring (--fatigue):
  - Maintains a 2600-sample ring buffer of raw EMG at full rate (1300ms @ 2000Hz, +/-300ms MDF padding)
  - Triggers on IMU gyro magnitude >= --imu-thr-dps (default 150 dps) with 300ms debounce
  - Calls FatigueMonitor.push_swing() in a background thread (same class as Android)
  - Displays: text annotation on EMG plot + 3rd MDF trend subplot
  - FatigueMonitor is imported from fatigue_monitor.py (--emg-util-path points to emg_util/ dir)

Usage:
  python sensordata_plotter.py --mode live --fatigue --imu-thr-dps 150 --fatigue-warmup-min 0
"""

import argparse
import csv
import json
import os
import queue
import socket
import sys
import threading
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ---------------------------------------------------------------------------
# FatigueMonitor import (optional -- graceful fallback if not found)
# ---------------------------------------------------------------------------
_FatigueMonitor = None  # set by _try_import_fatigue_monitor()


def _try_import_fatigue_monitor(path: str) -> bool:
    """Try to import FatigueMonitor from fatigue_monitor.py at the given directory path.

    fatigue_monitor.py is the canonical source (emg_util/fatigue_monitor.py).
    It is self-contained: zero pandas/matplotlib dependency.
    """
    global _FatigueMonitor
    resolved = os.path.normpath(path) if path else ''
    if resolved and os.path.isdir(resolved):
        if resolved not in sys.path:
            sys.path.insert(0, resolved)
    try:
        from fatigue_monitor import FatigueMonitor as _FM
        _FatigueMonitor = _FM
        print(f'[Fatigue] Loaded FatigueMonitor from fatigue_monitor at {resolved}')
        return True
    except ImportError as e:
        print(f'[Fatigue] WARNING: Could not import FatigueMonitor from fatigue_monitor at {resolved!r}: {e}')
        return False


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

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
    ap.add_argument('--infer-sr', action='store_true',
                    help='Infer sr_hz from dev_us deltas if emg_meta missed')
    ap.add_argument('--infer-min-hz', type=float, default=100.0)
    ap.add_argument('--infer-max-hz', type=float, default=4000.0)
    ap.add_argument('--infer-min-samples', type=int, default=30)

    # Offline
    ap.add_argument('--emg-file', default=None)
    ap.add_argument('--max-points', type=int, default=400000)
    ap.add_argument('--sr-hz', type=float, default=2000.0)

    # Fatigue monitoring
    ap.add_argument('--fatigue', action='store_true',
                    help='Enable real-time EMG fatigue monitoring')
    ap.add_argument('--emg-util-path', default=None,
                    help='Path to directory containing emg_util.py '
                         '(default: ../../emg_util relative to this script)')
    ap.add_argument('--imu-thr-dps', type=float, default=150.0,
                    help='Gyro magnitude trigger threshold in dps (default 150)')
    ap.add_argument('--fatigue-warmup-min', type=float, default=0.0,
                    help='Warmup skip before baseline collection in minutes '
                         '(default 0 = start immediately; use 5.0 for real sessions)')
    ap.add_argument('--fatigue-baseline-swings', type=int, default=20,
                    help='Number of swings to collect for baseline (default 20)')

    return ap.parse_args()


# ---------------------------------------------------------------------------
# Butterworth LPF (causal, sample-by-sample)
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def infer_sr_from_dev_us(dev_us_list, min_hz: float, max_hz: float):
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
    return float(int(round(sr)))


# ---------------------------------------------------------------------------
# Live mode
# ---------------------------------------------------------------------------

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
    last_vitals = {}

    emg_win = float(args.emg_window_sec)
    imu_win = float(args.imu_window_sec) if args.imu_window_sec is not None else emg_win

    # -----------------------------------------------------------------------
    # Fatigue monitoring setup
    # -----------------------------------------------------------------------
    # EMG ring buffer: 2600 samples = 1300ms at 2000Hz (includes +/-300ms MDF padding)
    # Matches Android emgFatigueBuf exactly.
    EMG_FATIGUE_BUF_SIZE = 2600
    fatigue_enabled = bool(args.fatigue)
    monitor = None
    fatigue_q = None
    fatigue_lock = threading.Lock()
    # Shared result dict updated by fatigue_worker, read by animation update()
    fatigue_result = {
        'swing_id': -1,
        'mdf': None,
        'mdf_drop_pct': None,
        'fatigue_level': 'warming_up',
        'warning': False,
        'baseline_ready': False,
        'baseline_swings_collected': 0,
        'baseline_mdf': None,
        'history': [],   # list of {t_peak_ms, mdf, mdf_drop_pct, fatigue_level}
    }
    emg_fatigue_buf = deque(maxlen=EMG_FATIGUE_BUF_SIZE)
    
    # Peak detection state (matches Android implementation)
    last_imu_trigger_t = [-999.0]   # mutable container (avoids nonlocal float)
    imu_thr_dps = float(args.imu_thr_dps)
    in_peak = [False]           # mutable: currently tracking a peak
    peak_gyro_mag = [0.0]       # mutable: highest gyro value in current peak
    peak_time_sec = [0.0]       # mutable: time of peak
    swing_count = [0]           # mutable: total swings detected this session

    if fatigue_enabled:
        util_path = args.emg_util_path
        if util_path is None:
            # Default: ../../emg_util relative to this script (C:\wangx\ftc\emg_util)
            util_path = os.path.normpath(os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                '..', '..', 'emg_util',
            ))
        if _try_import_fatigue_monitor(util_path):
            monitor = _FatigueMonitor(
                fs=2000.0,   # updated when emg_meta arrives with actual sr_hz
                warmup_skip_min=float(args.fatigue_warmup_min),
                baseline_swings=int(args.fatigue_baseline_swings),
            )
            fatigue_q = queue.Queue(maxsize=1)
            print(f'[Fatigue] Enabled | thr={imu_thr_dps:.0f} dps | '
                  f'warmup={args.fatigue_warmup_min} min | '
                  f'baseline={args.fatigue_baseline_swings} swings')
        else:
            fatigue_enabled = False

    def fatigue_worker():
        """Background thread: consume EMG snapshots, call FatigueMonitor, update shared result."""
        while True:
            try:
                item = fatigue_q.get(timeout=1.0)
            except queue.Empty:
                continue
            if item is None:   # sentinel to stop
                break
            emg_snap, t_peak_ms, imu_peak_mag = item
            try:
                result = monitor.push_swing(emg_snap, t_peak_ms, imu_peak_mag)
                with fatigue_lock:
                    fatigue_result.update(result)
                    fatigue_result['history'].append({
                        't_peak_ms': result['t_peak_ms'],
                        'mdf': result['mdf'],
                        'mdf_drop_pct': result['mdf_drop_pct'],
                        'fatigue_level': result['fatigue_level'],
                    })
                level = result['fatigue_level']
                mdf = result.get('mdf')
                drop = result.get('mdf_drop_pct')
                sid = result.get('swing_id', '?')
                print(f'[Fatigue] Swing #{sid} | MDF={mdf} Hz | Drop={drop}% | {level.upper()}')
            except Exception as exc:
                print(f'[Fatigue] Error in push_swing: {exc}')
            finally:
                fatigue_q.task_done()

    if fatigue_enabled and fatigue_q is not None:
        fat_thread = threading.Thread(target=fatigue_worker, daemon=True, name='fatigue_worker')
        fat_thread.start()

    # -----------------------------------------------------------------------
    # Figure layout: 2 subplots normally, 3 with fatigue
    # ax_emg and ax_imu share x-axis (rolling time window)
    # ax_fat has independent x-axis (swing index)
    # -----------------------------------------------------------------------
    if fatigue_enabled:
        fig = plt.figure(figsize=(12, 11))
        ax_emg = fig.add_subplot(3, 1, 1)
        ax_imu = fig.add_subplot(3, 1, 2, sharex=ax_emg)
        ax_fat = fig.add_subplot(3, 1, 3)   # independent x-axis: swing #
    else:
        fig, (ax_emg, ax_imu) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        ax_fat = None

    # EMG subplot
    (ln_raw,)     = ax_emg.plot([], [], lw=0.8, label='RAW')
    (ln_env_dev,) = ax_emg.plot([], [], lw=1.2, label='ENV_device')
    (ln_env_lpf,) = ax_emg.plot([], [], lw=1.4, label='ENV_lpf(abs(RAW))')

    title_text = ax_emg.set_title('Realtime monitoring (dev_us timebase)')
    ax_emg.set_ylabel('ADC (counts)')
    ax_emg.set_ylim(0, float(args.emg_ymax))
    ax_emg.grid(True, alpha=0.3)
    ax_emg.legend(loc='upper right', fontsize=8)

    # Fatigue text annotation (bottom-left of EMG plot)
    if fatigue_enabled:
        fatigue_ann = ax_emg.text(
            0.01, 0.04,
            'Fatigue: waiting for first swing...',
            transform=ax_emg.transAxes,
            va='bottom', fontsize=9, fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow',
                      alpha=0.88, edgecolor='gray'),
        )
    else:
        fatigue_ann = None

    # IMU subplot
    (ln_imu,) = ax_imu.plot([], [], lw=1.2, label='acc_mag (g)')
    ax_imu.set_ylabel('g')
    if not fatigue_enabled:
        ax_imu.set_xlabel('Time (s) (device dev_us)')
    ax_imu.grid(True, alpha=0.3)
    ax_imu.legend(loc='upper right', fontsize=8)

    # MDF trend subplot (fatigue only)
    ln_mdf_scatter = ln_mdf_trend = None
    fat_baseline_line = [None]   # mutable container for the axhline reference

    if fatigue_enabled and ax_fat is not None:
        (ln_mdf_scatter,) = ax_fat.plot(
            [], [], 'o', ms=5, alpha=0.7, color='steelblue', label='MDF per swing')
        (ln_mdf_trend,) = ax_fat.plot(
            [], [], '-', lw=2.0, alpha=0.85, color='steelblue',
            label='MDF trend (rolling median)')
        ax_fat.set_ylabel('MDF (Hz)')
        ax_fat.set_xlabel('Swing #')
        ax_fat.set_title('Fatigue trend -- MDF per swing')
        ax_fat.grid(True, alpha=0.3)
        ax_fat.legend(loc='upper right', fontsize=8)
        ax_fat.set_xlim(-0.5, 10)
        ax_fat.set_ylim(50, 500)

    # Status text (top-left of EMG plot)
    status = ax_emg.text(0.01, 0.98, '', transform=ax_emg.transAxes, va='top', fontsize=8)

    # -----------------------------------------------------------------------
    # Prune old samples from rolling buffers
    # -----------------------------------------------------------------------
    def prune(now_t: float) -> None:
        while raw_buf     and (now_t - raw_buf[0][0])     > emg_win:
            raw_buf.popleft()
        while env_dev_buf and (now_t - env_dev_buf[0][0]) > emg_win:
            env_dev_buf.popleft()
        while env_lpf_buf and (now_t - env_lpf_buf[0][0]) > emg_win:
            env_lpf_buf.popleft()
        while imu_buf     and (now_t - imu_buf[0][0])     > imu_win:
            imu_buf.popleft()

    # -----------------------------------------------------------------------
    # Animation update callback
    # -----------------------------------------------------------------------
    def update(_frame):
        nonlocal t0_dev_us, emg_sr_hz, emg_ch, env_filter, last_vitals
        last_seg = None
        rx_count = 0

        # Drain all pending UDP packets
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
            if k not in ('raw', 'env', 'imu'):
                print(f'[Plotter DEBUG] Received packet: {msg}')
            seg = msg.get('seg')
            if seg is not None:
                last_seg = seg

            # --- Metadata packets (no dev_us) ---

            if k == 'emg_meta':
                sr = msg.get('sr_hz')
                ch = msg.get('ch')
                meta_seg = msg.get('seg')
                if sr is not None:
                    sr = float(sr)
                    if (emg_sr_hz is None) or (float(emg_sr_hz) != sr):
                        emg_sr_hz = sr
                        emg_ch = ch
                        env_filter = ButterLPF(fs_hz=emg_sr_hz, cutoff_hz=float(args.cutoff_hz))
                        # Keep FatigueMonitor fs in sync via public setter
                        if fatigue_enabled and monitor is not None:
                            monitor.set_fs(sr)
                            print(f'[Fatigue] Updated fs={sr:.0f} Hz')
                        print(f'[Plotter] EMG meta: sr_hz={emg_sr_hz:.1f} ch={emg_ch}')
                # Reset fatigue monitor on new BLE session (seg=0 after data already seen)
                # seg=0 means btCentral just started a new BLE connection / CSV rotation 0.
                if (fatigue_enabled and monitor is not None
                        and meta_seg == 0 and t0_dev_us is not None):
                    monitor.reset()
                    emg_fatigue_buf.clear()
                    last_imu_trigger_t[0] = -999.0
                    print('[Fatigue] New BLE session (seg=0) — monitor reset')
                continue

            if k == 'vitals':
                last_vitals.update(msg)
                continue

            # --- Data packets (all have dev_us) ---

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

                # Push to fatigue ring buffer at FULL rate (before any decimation)
                if fatigue_enabled:
                    emg_fatigue_buf.append(r)

                # Infer sr_hz from timing if emg_meta was missed
                if env_filter is None and args.infer_sr and \
                        len(recent_raw_dev_us) >= int(args.infer_min_samples):
                    sr_inf = infer_sr_from_dev_us(
                        list(recent_raw_dev_us),
                        float(args.infer_min_hz), float(args.infer_max_hz))
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

                # Peak detection: only trigger on the HIGHEST point of the swing
                # This prevents double-counting backswing + forward swing
                if fatigue_enabled and monitor is not None and fatigue_q is not None:
                    gmag = msg.get('gmag')
                    if gmag is not None:
                        gmag = float(gmag)
                        
                        if gmag >= imu_thr_dps:
                            if not in_peak[0]:
                                # Start of new peak
                                in_peak[0] = True
                                peak_gyro_mag[0] = gmag
                                peak_time_sec[0] = t_sec
                            else:
                                # Already in peak - update if this is higher
                                if gmag > peak_gyro_mag[0]:
                                    peak_gyro_mag[0] = gmag
                                    peak_time_sec[0] = t_sec
                        elif in_peak[0]:
                            # Dropped below threshold - end of peak
                            # Trigger detection at the peak point (if debounce allows)
                            if (peak_time_sec[0] - last_imu_trigger_t[0]) >= 1.0:
                                last_imu_trigger_t[0] = peak_time_sec[0]
                                swing_count[0] += 1  # Increment swing counter
                                snap = list(emg_fatigue_buf)   # snapshot (copy)
                                if len(snap) >= 32:
                                    t_peak_ms = peak_time_sec[0] * 1000.0
                                    try:
                                        fatigue_q.put_nowait((snap, t_peak_ms, peak_gyro_mag[0]))
                                    except queue.Full:
                                        pass   # drop if fatigue thread is still busy
                            in_peak[0] = False
                            peak_gyro_mag[0] = 0.0

            elif k == 'battery':
                pct = msg.get('pct')
                v   = msg.get('v')
                if pct is not None:
                    print(f'[Plotter] Battery: {int(pct)}%')
                if v is not None:
                    print(f'[Plotter] Voltage: {float(v):.2f}V')

            rx_count += 1

        # --- Compute current time and prune old data ---
        now_t = 0.0
        for b in (raw_buf, env_dev_buf, env_lpf_buf, imu_buf):
            if b:
                now_t = max(now_t, b[-1][0])
        prune(now_t)

        # --- Update EMG lines ---
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

        # --- Update IMU line ---
        if imu_buf:
            xs = np.array([p[0] for p in imu_buf], dtype=float)
            ys = np.array([p[1] for p in imu_buf], dtype=float)
            ln_imu.set_data(xs, ys)
            lo, hi = float(np.min(ys)), float(np.max(ys))
            if hi <= lo:
                hi = lo + 0.1
            pad = 0.1 * (hi - lo)
            ax_imu.set_ylim(lo - pad, hi + pad)

        # Shared x-axis for EMG + IMU (rolling window)
        xlim_lo = max(0.0, now_t - max(emg_win, imu_win))
        ax_imu.set_xlim(xlim_lo, now_t)   # propagates to ax_emg via sharex

        # --- Title: vitals + swing count ---
        if last_vitals:
            parts = []
            if last_vitals.get('pct') is not None:
                batt_str = f"BATTERY: {last_vitals['pct']}%"
                if last_vitals.get('v') is not None:
                    batt_str += f" ({last_vitals['v']:.2f}V)"
                parts.append(batt_str)
            if last_vitals.get('spo2') is not None:
                parts.append(f"SpO2: {last_vitals['spo2']}%")
            if last_vitals.get('hr') is not None:
                parts.append(f"HR: {last_vitals['hr']:.1f} bpm")
            # Add swing count
            if fatigue_enabled and swing_count[0] > 0:
                parts.append(f"SWINGS: {swing_count[0]}")
            title_text.set_text('Realtime Monitoring | ' + ' | '.join(parts))
            batt_pct = last_vitals.get('pct', 100)
            batt_color = 'red' if batt_pct < 20 else ('orange' if batt_pct < 50 else 'green')
            title_text.set_color(batt_color)
            title_text.set_fontweight('bold')
        else:
            # Show swing count even without vitals
            swing_str = f" | SWINGS: {swing_count[0]}" if (fatigue_enabled and swing_count[0] > 0) else ""
            title_text.set_text(f'Realtime Monitoring [Waiting for Vitals...]{swing_str}')
            title_text.set_color('black')
            title_text.set_fontweight('normal')

        seg_str = f'seg={last_seg}' if last_seg is not None else 'seg=?'
        sr_str  = f'sr={emg_sr_hz:.0f}Hz' if emg_sr_hz else 'sr=?'
        status.set_text(
            f'UDP {args.listen_host}:{args.listen_port}  {seg_str}  {sr_str}  rx={rx_count}')

        # -----------------------------------------------------------------------
        # Fatigue display update (annotation + MDF trend subplot)
        # -----------------------------------------------------------------------
        if fatigue_enabled and fatigue_ann is not None:
            with fatigue_lock:
                fr = dict(fatigue_result)
                history = list(fr.get('history', []))

            swing_id       = fr.get('swing_id', -1)
            level          = fr.get('fatigue_level', 'warming_up')
            mdf            = fr.get('mdf')
            drop           = fr.get('mdf_drop_pct')
            baseline_ready = fr.get('baseline_ready', False)
            n_baseline     = fr.get('baseline_swings_collected', 0)
            warning        = fr.get('warning', False)
            baseline_mdf   = fr.get('baseline_mdf')

            # Text annotation
            if swing_id < 0:
                fat_str    = 'Fatigue: waiting for first swing...'
                fat_color  = 'gray'
                bbox_color = 'lightyellow'
            elif not baseline_ready:
                fat_str    = (f'Swing #{swing_id} | Warming up '
                              f'({n_baseline}/{args.fatigue_baseline_swings} baseline swings)')
                fat_color  = 'navy'
                bbox_color = 'lightcyan'
            else:
                mdf_str  = f'{mdf:.1f} Hz' if mdf is not None else 'N/A'
                drop_str = f'{drop:+.1f}%'  if drop is not None else 'N/A'
                warn_pfx = '[!] ' if warning else ''
                fat_str  = (f'{warn_pfx}Swing #{swing_id} | MDF={mdf_str} | '
                            f'Drop={drop_str} | {level.upper()}')
                fat_color  = {'fresh': 'darkgreen', 'moderate': 'darkorange',
                              'high': 'red'}.get(level, 'gray')
                bbox_color = {'fresh': 'honeydew', 'moderate': 'lightyellow',
                              'high': 'mistyrose'}.get(level, 'lightyellow')

            fatigue_ann.set_text(fat_str)
            fatigue_ann.set_color(fat_color)
            fatigue_ann.get_bbox_patch().set_facecolor(bbox_color)

            # MDF trend subplot
            if ax_fat is not None and ln_mdf_scatter is not None and len(history) >= 1:
                valid = [(i, h['mdf']) for i, h in enumerate(history) if h['mdf'] is not None]
                if valid:
                    xs_h = [v[0] for v in valid]
                    ys_h = [v[1] for v in valid]
                    ln_mdf_scatter.set_data(xs_h, ys_h)

                    # Rolling median trend line (window = min(10, n//3), min 3)
                    n_h = len(ys_h)
                    win = max(3, min(10, n_h // 3)) if n_h >= 3 else n_h
                    if n_h >= win:
                        trend = [
                            float(np.median(ys_h[max(0, i - win + 1):i + 1]))
                            for i in range(n_h)
                        ]
                        ln_mdf_trend.set_data(xs_h, trend)

                    # Baseline reference horizontal line (created once, updated after)
                    if baseline_mdf is not None:
                        if fat_baseline_line[0] is None:
                            fat_baseline_line[0] = ax_fat.axhline(
                                float(baseline_mdf), color='k', ls='--',
                                lw=1.5, alpha=0.6, label='baseline MDF')
                            ax_fat.legend(loc='upper right', fontsize=8)
                        else:
                            fat_baseline_line[0].set_ydata(
                                [float(baseline_mdf), float(baseline_mdf)])

                    ax_fat.set_xlim(-0.5, max(len(history) - 0.5, 5))
                    lo = min(ys_h) * 0.85
                    hi = max(ys_h) * 1.15
                    if hi <= lo:
                        hi = lo + 10.0
                    ax_fat.set_ylim(lo, hi)

        # Collect artists for return (blit=False so not strictly needed, but good practice)
        artists = [ln_raw, ln_env_dev, ln_env_lpf, ln_imu, status]
        if fatigue_ann is not None:
            artists.append(fatigue_ann)
        if ln_mdf_scatter is not None:
            artists.extend([ln_mdf_scatter, ln_mdf_trend])
        return artists

    interval_ms = max(10, int(1000.0 / float(args.fps)))
    _ani = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Offline mode (unchanged)
# ---------------------------------------------------------------------------

def run_offline(args: argparse.Namespace) -> None:
    if not args.emg_file:
        raise SystemExit('Offline mode requires --emg-file')

    t_ms, raw, env = [], [], []
    with open(args.emg_file, 'r', encoding='utf-8') as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                t  = int(row.get('host_rx_ms', '0'))
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
    xs  = np.array([(t - t0) / 1000.0 for t in t_ms], dtype=float)
    raw = np.array(raw, dtype=float)
    env = np.array(env, dtype=float)

    env_filter = ButterLPF(fs_hz=float(args.sr_hz), cutoff_hz=float(args.cutoff_hz))
    env_lpf = np.array([env_filter.update(abs(x)) for x in raw], dtype=float)

    fig, ax = plt.subplots(1, 1, figsize=(11, 5))
    ax.plot(xs, raw,     lw=0.8, label='RAW')
    ax.plot(xs, env,     lw=1.0, label='ENV_device')
    ax.plot(xs, env_lpf, lw=1.2, label='ENV_lpf(abs(RAW))')
    ax.set_title(f'Offline EMG: {args.emg_file}')
    ax.set_xlabel('Time (s) (host_rx_ms)')
    ax.set_ylabel('ADC (counts)')
    ax.set_ylim(0, float(args.emg_ymax))
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    args = parse_args()
    if args.mode == 'live':
        run_live(args)
    else:
        run_offline(args)


if __name__ == '__main__':
    main()
