# Changelog (host scripts)

All notable changes to host-side scripts are documented here.
Format: newest session at the top.

---

## 2026-03-02 — Root Cause Identified: BLE STALLs were caused by wrong hardware pad (I2C SDA bus corruption)

### Root Cause Analysis (Retrospective)

The frequent BLE reconnects and `state=STALL` events observed in early March 2026 were ultimately traced to a **hardware wiring error**, not a software bug.

**The wrong pad:**
The jumper wire bypassing the broken pogo pin was soldered to the pad labeled **"A4"** on the SparkFun Wireless Shield. Due to Arduino Uno pin-compatibility routing, this pad is connected to **GPIO 21 (ESP32 I2C SDA)**, not to an ADC pin. The firmware was reading GPIO 32 (ADC1 channel 4) for RAW EMG data, so the ADC was floating and the wire was injecting the MyoWare RAW analog signal directly onto the **I2C bus**.

**Why this caused BLE STALLs:**
The MyoWare RAW output is a low-impedance analog signal that swings between 0V and 3.3V at muscle frequencies (20–500 Hz). I2C SDA is an open-drain bus with a pull-up resistor (~4.7kΩ). When the RAW signal drove SDA **LOW** during a muscle flex, the ESP32 I2C peripheral saw a **stuck-low SDA** — a classic I2C bus lockup. The cascade:

1. RAW EMG signal drives I2C SDA low during muscle flex
2. ESP32 I2C peripheral hangs waiting for SDA to go HIGH (never does while muscle is active)
3. IMU read task blocks indefinitely on the stuck I2C bus
4. No new IMU data → no new BLE notification packets
5. Health monitor sees `fps=0` → `state=STALL` → triggers reconnect

This explains the health log pattern exactly:
```
fps=102  ← normal, muscle at rest, RAW near 1.65V DC (not driving SDA low hard enough)
fps=99
fps=6    ← user starts flexing, RAW starts swinging, I2C getting corrupted
fps=0    ← I2C bus locked up, IMU task hung
state=STALL → reconnect
```
The reconnect "fixed" it temporarily because the ESP32 re-initialized, clearing the I2C lockup. But as soon as the muscle flexed again, the same thing happened.

**The actual fix:**
Moved the solder wire from the "A4" pad (GPIO 21, I2C SDA) to the pad explicitly labeled **`32`** on the back of the Wireless Shield (GPIO 32, ADC1 channel 4). This is the pin the firmware was already reading. See `myoware_imu_v2/WIRING.md` for the full debug process (finger-touch test, ADC crosstalk ghost signal analysis, and final verification).

**Note on the btCentral.py software changes (same date):**
The reconnect logic improvements (removing resubscribe, adding `disconnected_callback`, wrapping `stop_notify` in a timeout) were made before the hardware root cause was identified. They remain valid improvements — they make the software more resilient to any future hardware or BLE stack issues — but they were treating the symptom, not the cause.

---

## 2026-03-02 — `btCentral.py`: Fix excessive reconnects and handle ESP32 brown-outs

### Changed (`btCentral.py`)
- Increased PING interval (`--ping-sec`) default from 1.0s to 5.0s to reduce unnecessary BLE traffic.
- Removed the `resub_attempted` logic from `watchdog_task`. It now immediately triggers a reconnect when the 5-second stall timeout is reached.
- Added a `disconnected_callback` (`handle_disconnect`) to the `BleakClient` initialization to trigger an immediate reconnect when the OS detects a dropped connection.
- Wrapped `client.stop_notify()` inside the `finally` cleanup block with `asyncio.wait_for(..., timeout=2.0)` to prevent the script from hanging indefinitely when tearing down a dead connection.

### Motivation & Root Cause Analysis
Users observed frequent reconnects with the reason `reconnect_after_resub_9605ms` when running the ESP32 on battery power, but the connection was stable when plugged into USB-C. Furthermore, the ESP32 LED was observed turning off and back on during these events.

**Root Cause:**
The ESP32 was experiencing **Brown-Out Resets (BOD)**. During BLE transmission spikes, the ESP32 can draw 300-500mA. On battery power (if the battery is weak or wiring has high resistance), this current spike caused the voltage to momentarily dip below ~2.8V, triggering a hardware reset. 

**Why it caused 9-second hangs in Python:**
When the ESP32 suddenly lost power and reset, the Windows BLE stack did not immediately fire a disconnect event. The Python `watchdog_task` correctly detected the silence after 5 seconds (`stall_reconnect_ms`). However, the script then attempted to `stop_notify` and `start_notify` to "resubscribe". Because the physical connection was dead, these WinRT BLE operations hung and eventually timed out 4 seconds later (totaling 9+ seconds of silence). 

**The Fix:**
By adding a proper `disconnected_callback` and simplifying the watchdog to force an immediate reconnect (skipping the doomed resubscribe attempts), the script now recovers almost instantly when the ESP32 reboots. Additionally, wrapping `stop_notify` in a timeout ensures the cleanup phase doesn't hang. While the true fix for the brown-out is hardware-based (better battery, shorter wires, or decoupling capacitors), the software is now highly resilient and recovers from device resets in ~1-2 seconds.

---

## 2026-03-01 — `myoware_imu_v2.ino`: Fix frequent STALL reconnect loops

### Changed (`myoware_imu_v2.ino`)
- Relaxed Watchdog host heartbeat timeout policies by 10x:
  - `HOST_PING_EXPECT_MS` raised from 2000 to 20000.
  - `HOST_SILENCE_PAUSE_MS` raised from 5000 to 50000.
  - `HOST_SILENCE_REBOOT_MS` raised from 30000 to 300000.

### Motivation
During high-speed data capture (2000 Hz EMG + 200 Hz IMU), the Windows Bluetooth stack frequently enters a "stall" state where it silently drops or indefinitely queues the outbound `PING` packets from `btCentral.py`. 

Because the ESP32 was not receiving these PINGs, its internal watchdog (previously set to 5 seconds) would trigger, causing it to intentionally pause all data sampling to save power. This sudden stop in data caused the Python host script to register a `stall_no_notify` error and forcefully initiate a reconnect loop. 

A previous attempt (2026-02-28) tried to fix this by changing `response=False` to `response=True` in Python, but this did not solve the OS-level packet drops. The correct fix is to acknowledge that Windows will occasionally drop outbound BLE writes under heavy load, and the ESP32 must be resilient to this. Relaxing the ESP32 timeout to 50 seconds completely prevents the stuttering `fps=0` issue while still maintaining a functional watchdog in the event of a true disconnection.

---

## 2026-03-01 — `btCentral.py`: Fix `asyncio.exceptions.CancelledError` crash during Watchdog reconnects

### Changed (`btCentral.py`)
- Explicitly catch `asyncio.CancelledError` inside the main event loop's task-cleanup `finally` blocks (lines 923-927 and 971-975).

### Motivation
When the BLE connection stalls (e.g., `stall_no_notify_5490ms`) and triggers a forced reconnect, the main loop calls `.cancel()` on the running background tasks (`ping_task`, `watchdog_task`, `writer_task`, etc.). 
In Python 3.8+, `asyncio.CancelledError` inherits from `BaseException`, not `Exception`. Because the cleanup code previously only used `except Exception: pass`, the cancellation exception escaped the handler and crashed the entire Python script. 
Adding `except asyncio.CancelledError: pass` ensures tasks can be safely torn down and restarted without crashing the script.

## 2026-02-28 — BLE Watchdog & Connection Stall Recovery (End-to-End Analysis)

### Part 1: Python PING `response=False` → `response=True` (Best Practice Fix)

**Initial hypothesis:**
The Arduino watchdog wasn't detecting the host because Python was sending PING with `response=False`, which might signal "no one is listening."

**Fix applied:**
Changed `response=False` to `response=True` in **2 locations** in `btCentral.py`:

1. **Initial connection PING** (line ~1047):
```python
await client.write_gatt_char(NUS_RX_UUID, b"PING", response=True)  # was False
```

2. **Periodic ping_task** (line ~1001):
```python
await client.write_gatt_char(NUS_RX_UUID, b"PING", response=True)  # was False
```

**Why this is correct BLE practice:**
- `response=True` makes Python wait for Arduino's ACK, providing better error detection
- If the write truly fails, Python will get an exception (incrementing `ping_fail`)
- This is the proper way to implement keepalive protocols in BLE

### Part 2: Root Cause Analysis — BLE Connection Stall (Not a Code Bug)

**Observed symptoms after applying the fix:**
```
[Health] fps=0 notify/s=0 rx_kB/s=0.0  ← Arduino not streaming data
         ping_fail=0                    ← Python thinks PINGs are succeeding
         ping_age=30481ms               ← Arduino hasn't received PING for 30s
         conn_sub=1                     ← BLE reports "connected"
```

**What's actually happening:**
This is a **BLE connection stall** — a known issue where the BLE stack enters a state where:
1. ✅ Python's BLE stack reports "connected"
2. ✅ Python's `write_gatt_char()` succeeds without exceptions
3. ❌ **Data never reaches Arduino's `onWrite` callback**
4. ✅ Arduino's watchdog correctly detects no PING for 30s
5. ✅ Arduino pauses streaming to save battery (correct behavior)

**This is NOT a bug in either Python or Arduino code** — it's a BLE stack issue that can occur due to:
- Timing issues in the BLE protocol stack
- Buffer overflows in the BLE controller
- Radio interference or packet loss
- Firmware bugs in the BLE chipset

### Part 3: Arduino Watchdog — The Correct Recovery Mechanism

The Arduino firmware **already has the correct recovery mechanism** built in:

```cpp
// myoware_imu_v2.ino, loop() function
if (hp != 0 && hostAge >= HOST_SILENCE_REBOOT_MS) {  // 30 seconds
  Serial.printf("[WD] Host silent %lu ms. Rebooting device for recovery...\n", hostAge);
  delay(50);
  ESP.restart();
}
```

**How the watchdog works (end-to-end):**

1. **Python side (`btCentral.py`):**
   - Sends PING every 1 second via `ping_task`
   - Tracks `last_ping_ok_ms` (when write succeeded)
   - Reports `ping_age` in health logs

2. **Arduino side (`myoware_imu_v2.ino`):**
   - `RxCallbacks::onWrite` receives PING and calls `setLastHostPingNow()`
   - `loop()` checks `hostAge = millis() - lastHostPingMs`
   - If `hostAge > 5000ms`: **pause sampling/streaming** (save battery)
   - If `hostAge > 30000ms`: **reboot device** (recover from BLE stall)

3. **Recovery sequence:**
   - Arduino reboots after 30s of silence
   - BLE connection is torn down and rebuilt fresh
   - Python's watchdog detects disconnect and reconnects
   - Fresh connection works normally

### Part 4: Why Both Changes Were Necessary

**The `response=True` fix:**
- ✅ Improves error detection (Python will see exceptions if writes truly fail)
- ✅ Follows BLE best practices for keepalive protocols
- ✅ May prevent other types of connection issues
- ❌ Doesn't solve BLE stack stalls (those require reboot recovery)

**The Arduino watchdog (already present):**
- ✅ Detects when host is absent (no PING received)
- ✅ Pauses sampling to save battery when host is gone
- ✅ Reboots after 30s to recover from BLE stalls
- ✅ Works correctly regardless of `response=True` vs `response=False`

### Design Principles

1. **Keepalive protocols should use `response=True`:**
   - Provides better error detection
   - Signals to the peripheral that the host is actively listening
   - Follows BLE best practices

2. **Watchdogs must be bidirectional:**
   - Python tracks `ping_age` (time since last successful write)
   - Arduino tracks `hostAge` (time since last PING received)
   - Both sides can detect and recover from connection issues

3. **BLE stalls require reboot recovery:**
   - No amount of code changes can prevent BLE stack stalls
   - The only reliable recovery is to reboot the peripheral
   - 30-second timeout balances recovery speed vs false positives

4. **Graceful degradation:**
   - When host is absent, pause sampling (don't waste battery)
   - When connection stalls, reboot automatically (don't require manual intervention)
   - Log all state transitions for debugging

### Conclusion

The system is **working as designed**. The `response=True` fix improves BLE practices, and the Arduino watchdog correctly detects and recovers from connection stalls. No further code changes are needed.
>>>>>>> b78c01a92494cb09d3e3281a1968f42c1eeccbae

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
