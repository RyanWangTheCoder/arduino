# Changelog (host scripts)

All notable changes to host-side scripts are documented here.
Format: newest session at the top.

---

## 2026-02-25 — `btCentral.py`: IMU FSR fallback defaults raised (gyro 250→1000 dps, acc 2→4 g)

### Changed (`btCentral.py`)
- `cur_acc_fsr_g = 2` → `4` (line 493)
- `cur_gyro_fsr_dps = 250` → `1000` (line 494)

These are the **fallback defaults** used when a capture has no FSR metadata in the BLE frame
(old firmware or missing `FLAG_IMU_META_V2`). For new captures the FSR is read from the frame
and overrides these values automatically — no change needed to the decoder logic.

### Note: `sensordata_plotter.py` — no change needed
`--imu-thr-dps` (default 150 dps) is a physical threshold in dps. `gmag` is already in
physical units (computed by `btCentral.py` using per-frame FSR scaling). The threshold is
FSR-independent and remains valid at 150 dps.

---

## 2026-02-25 — `sensordata_plotter.py`: Direct `fatigue_monitor` import

### Changed (`sensordata_plotter.py`)
- `_try_import_fatigue_monitor()`: changed import from `from emg_util import FatigueMonitor`
  to `from fatigue_monitor import FatigueMonitor`.
- Module docstring updated to reflect the new import source.

### Motivation
`FatigueMonitor` was extracted from `emg_util.py` into a standalone `fatigue_monitor.py`
(see `emg_util/CHANGELOG.md`). Importing directly from `fatigue_monitor` avoids loading
all of `emg_util.py` (pandas, matplotlib, scipy) just to get `FatigueMonitor`.

`sensordata_plotter.py` only needs `FatigueMonitor` — it has no use for the rest of
`emg_util`. The `--emg-util-path` argument still points to the `emg_util/` directory,
which is where `fatigue_monitor.py` lives. No file copy is needed: `_try_import_fatigue_monitor()`
adds `emg_util/` to `sys.path` at runtime, so `fatigue_monitor.py` is found directly.

Note: the Android copy (`btLeApp/app/src/main/python/fatigue_monitor.py`) is still required
because Chaquopy can only import Python files physically packaged inside the APK — there is
no `sys.path` trick available on Android. That copy must be refreshed before each APK build.

---

## 2025 — Session 2: Reconnect Hygiene + Timestamp Sync Fix (btCentral.py)

### Root Cause Fix: `DeviceClockExtender` Not Reset on Reconnect

**Root cause location:** `btCentral.py` — host-side bug.

**Problem:**
After a BLE reconnect (or device watchdog reboot), `DeviceClockExtender` was not reset.
The device's `micros()` counter restarts from 0 on a watchdog reboot (`ESP.restart()`).
`DeviceClockExtender` uses a wrap-detection heuristic: if `t_us_u32` jumps backward by
more than 2³¹ µs, it increments `wraps` and adds 2³² µs (~71 minutes) to all subsequent
timestamps.

On a device reboot, `t_us_u32` jumps from some large value (e.g., `0xC0000000`) back to
near 0. `DeviceClockExtender` would detect this as a 32-bit wrap and add 2³² µs to all
subsequent `dev_us` timestamps — making EMG and IMU timestamps wrong by ~71 minutes.

```
Before fix (device reboots after 30s watchdog timeout):
  last_u32 = 0xC0000000 (device was running ~53 min)
  new  t   = 0x00001000 (device just rebooted)
  → (last_u32 - t) > 0x40000000 → wraps++ → WRONG
  → dev_us = (1 << 32) + 0x00001000 ≈ 71 min + 4ms  ← incorrect

After fix (DeviceClockExtender reset on reconnect):
  wraps = 0, last_u32 = None
  new  t   = 0x00001000
  → dev_us = 0x00001000 = 4ms  ← correct
```

**Fix:**
```python
# In the reconnect exception handler:
st.parser = FrameParser()          # clear stale partial frame bytes
st.clock = DeviceClockExtender()   # reset extended clock — handles device reboot case
st.last_frame_seq = None           # don't count firmware seq reset as a gap
```

**Why `FrameParser` also needs reset:**
If the previous connection ended mid-frame, `FrameParser.buf` contains stale bytes.
After reconnect, new data is appended to the stale bytes. The parser may attempt to
parse a garbage frame, which will fail CRC and be counted as a `crc_errors` increment.
Resetting the parser ensures clean parsing from the first byte of the new connection.

**Why `last_frame_seq` needs reset:**
The firmware resets `frameSeq = 0` on each new connection. Without resetting
`last_frame_seq`, the first frame of the new connection (seq=0) would be counted as
a gap (expected `last_seq+1`, got 0), polluting the `frame_gaps` counter.

**Note on normal reconnect (no reboot):**
On a normal BLE reconnect without device reboot, `micros()` continues monotonically —
only `frameSeq/emgSeq/imuSeq` reset to 0. Resetting `DeviceClockExtender` in this case
is harmless: `wraps` would be 0 anyway (no wrap has occurred yet in a typical session).

**Residual limitation (by design):**
After `DeviceClockExtender` reset, if the device has been running for more than 71 minutes
(2³² µs) without rebooting, the extended `dev_us` values will be offset from the true
device uptime by one wrap period. This only affects absolute timestamp values — relative
synchronization between EMG and IMU within a session is unaffected (both use the same
`micros()` counter). The `host_rx_ms` column provides wall-clock anchoring.

### Additional Changes in This Session

- **Cooperative scheduling in `consumer_task` (asyncio fairness)**
  - **Change:** Periodically yields to the event loop (`await asyncio.sleep(0)`) after processing a number of frames and also inside long EMG/IMU per-sample loops.
  - **Rationale:** Prevents `consumer_task` from monopolizing the event loop during heavy parsing/processing so `writer_task` (CSV flush) and `health_task` stay responsive; helps reduce queue buildup/drops.

- **UDP send failure backoff (best-effort + CPU protection)**
  - **Change:** `udp_send()` remains best-effort (exceptions swallowed) but now applies exponential backoff when sends fail, skipping attempts until the backoff window expires.
  - **Rationale:** If the plotter/receiver isn't running or UDP sends error, repeatedly calling `sendto()` at high rates wastes CPU. Backoff reduces needless work while keeping recording non-blocking.

- **CSV rotation filename collision avoidance**
  - **Change:** `TimeRotatingCsv.open()` now includes milliseconds in the file `open_id`.
  - **Rationale:** Avoids rare filename collisions/overwrites when a rotation happens within the same second.

- **BLE queue overflow warnings**
  - **Change:** When the BLE queue is full, dropped-chunk warnings are emitted periodically (drop policy unchanged: drop newest chunk).
  - **Rationale:** Makes data loss visible instead of silent.

---

## 2025 — Session 1: Initial btCentral.py (BLE Recorder + UDP Telemetry)

- Initial implementation of robust BLE recorder with:
  - Nordic UART Service (NUS) BLE connection to ESP32 firmware
  - Binary protocol parser (TLV format, CRC16-CCITT, `FrameParser`)
  - `DeviceClockExtender` for 32-bit `micros()` wrap-around handling
  - Time-rotating CSV logging (EMG, IMU, vitals) with `.meta.json` sidecar files
  - Host watchdog (PING/ACK keepalive, stall detection, reconnect with exponential backoff)
  - UDP telemetry for real-time plotting (decimated RAW/ENV/IMU streams)
  - Battery monitoring and low-battery warnings from vitals frames
