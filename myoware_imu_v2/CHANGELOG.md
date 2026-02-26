# MyoWare IMU v2 — Changelog

All notable changes to `myoware_imu_v2.ino` are documented here.  
Format: newest session at the top.

---

## 2025 — Session 9: `printBatteryStatus()` — Remove Live I2C Read

### Problem
`printBatteryStatus()` called `readBattery()` unconditionally on every call, which is
invoked from the `[Health]` debug report every 1 second. `readBattery()` performs
4 live I2C transactions (`getSOC`, `getVoltage`, `getChangeRate`, `isLow`) — ~400 µs
of I2C bus time added to `loop()` every second, and an unnecessary contention point
on `wireMutex` shared with `imuTask` and `spo2Task`.

The cached values `lastBatteryPct` and `lastBatteryVoltage` are already maintained
by `send_vitals_frame()` (refreshed every `BATTERY_READ_INTERVAL_MS` = 10 s) and
by `setup()` (initial read). They are always available for a serial debug print.

### Fix
Rewritten `printBatteryStatus()` to use `lastBatteryPct` / `lastBatteryVoltage` directly.
No I2C. No `wireMutex`. Added `age=Ns` field showing seconds since last real read,
so the output remains honest about data freshness.

```
// Before (every 1 s):
[Battery] 87.3% (3.92V) +0.1%/hr
  └─ 4 live I2C reads, ~400 µs, wireMutex contention

// After (every 1 s):
[Battery] 87.3% (3.92V) age=3s
  └─ 2 RAM reads, ~0 µs, no I2C
```

`readBattery()` is now called **only** from `send_vitals_frame()` on the 10-second interval.

---



### Concern Raised
> If `emg_adc_init()` is called while `emg_adc_deinit()` hasn't set `adcHandle = nullptr` yet,
> `adc_continuous_new_handle()` could be called while the old handle is still being deinitialized.

### Analysis: Already Covered by `adcMutex` (Session 2)

Both `emg_adc_init()` and `emg_adc_deinit()` take `adcMutex` with `portMAX_DELAY` as their
first operation. The mutex serialises them completely:

```
emg_adc_deinit() (Core 0)        emg_adc_init() (Core 0)
─────────────────────────        ───────────────────────
xSemaphoreTake(adcMutex) LOCKED
adc_continuous_stop()
adc_continuous_deinit()
adcHandle = nullptr
xSemaphoreGive(adcMutex) UNLOCKED
                                 xSemaphoreTake(adcMutex)  ← now acquires
                                 adcHandle == nullptr ✓
                                 adc_continuous_new_handle()
                                 xSemaphoreGive(adcMutex)
```

`emg_adc_init()` cannot observe a partially-deinitialized handle because it cannot
acquire the mutex until `emg_adc_deinit()` has completed **and** set `adcHandle = nullptr`.

| Scenario | Result |
|---|---|
| `init` called while `deinit` holds mutex | Blocks at `xSemaphoreTake` until `deinit` finishes |
| `init` called twice concurrently | Second caller blocks, then sees `adcHandle != nullptr` and returns |
| `emgDrainTask` reading while `deinit` runs | 10 ms timeout — skips cycle if mutex unavailable |

### Conclusion
No code change required. The concern was valid before Session 2. `adcMutex` is the fix.

---



### Problem
The DMA delivers ADC results in round-robin order: ENV, RAW, ENV, RAW, ...
If the ADC arbiter skips a conversion (e.g. due to BLE radio interference on ESP32),
two consecutive same-channel results can arrive: ENV, ENV, RAW, RAW.

The previous code silently overwrote `envVal` / `rawVal` with no record of the event.
The resulting pair used the second ENV with the next RAW — a one-sample mismatch.

### Fix
Added `static volatile uint32_t emgChOrderViolations` counter:
- **Incremented** inside `emgMux` spinlock when a same-channel overwrite is detected
  (ENV arriving while `envValid` is already set, or RAW while `rawValid` is already set)
- **Reset** to 0 under `emgMux` in `onConnect()` alongside `emgDropped` / `emgSeq`
- **Printed** in the `[Health]` report every second as `EMGchViol=N`

The pairing behaviour is unchanged — overwrite-and-continue is still correct
(the newer value is more recent and more useful). The counter provides diagnostic
visibility to detect ADC arbiter issues in the field.

### Health Report Format Change
```
// Before:
[Health] ... EMGdrop=N IMUdrop=N Frames/s=N ...
// After:
[Health] ... EMGdrop=N IMUdrop=N EMGchViol=N Frames/s=N ...
```

---



### Problem
`imuTask` and `spo2Task` were both priority 1 on Core 1. `spo2Task` collects 100 samples
over ~4 seconds, calling `maxSensor.check()` in a loop with `vTaskDelay(1)` between polls.
If `maxSensor.check()` (I2C) takes longer than expected (e.g. bus contention, timeout),
`spo2Task` holds the CPU and `imuTask` misses its 200 Hz timer ticks — dropping IMU samples.

`spo2Task` has no real-time requirement: it computes SpO2/HR once every ~30 seconds.
`imuTask` is real-time: it must respond to 200 Hz `esp_timer` ticks within 5 ms.

### Fix
Adjusted task priorities on Core 1:

| Task | Before | After | Reason |
|---|---|---|---|
| `emgDrainTask` | 2 | 2 | Unchanged — real-time DMA drain at 2 kHz |
| `imuTask` | 1 | **2** | Raised — real-time 200 Hz, must not be starved |
| `spo2Task` | 1 | **0** | Lowered — background, no real-time requirement |

`emgDrainTask` and `imuTask` are now equal priority 2. The FreeRTOS scheduler
time-slices between equal-priority tasks, so neither starves the other.
`spo2Task` at priority 0 only runs when both priority-2 tasks are blocked
(waiting on DMA data or `vTaskDelay`).

---



### Problem
`emgSeq`, `imuSeq`, `emgDropped`, `imuDropped` are `volatile uint32_t` accessed from two cores
without synchronisation:

| Variable | Reset (Core 0) | Increment (Core 1) |
|---|---|---|
| `emgSeq` | `onConnect()` line 498 | `emgDrainTask` — `emgSeq++` outside `emgMux` |
| `imuSeq` | `onConnect()` line 499 | `imuTask` — `imuSeq++` outside `imuMux` |
| `emgDropped` | `onConnect()` line 496 | `emg_push()` inside `emgMux` |
| `imuDropped` | `onConnect()` line 497 | `imu_push()` inside `imuMux` |

Two distinct races:
1. `onConnect()` resets `emgSeq = 0` while `emgDrainTask` is mid `emgSeq++` — torn write.
2. `emgSeq++` in `emgDrainTask` was **outside** `emgMux`, so it was not protected by the
   same lock used to reset it in `onConnect()`.

### Fix

**1. Move `emgSeq++` / `imuSeq++` inside the existing critical sections in `emg_push` / `imu_push`:**
```cpp
// emg_push: seq now assigned atomically with the ring buffer write
else { emgBuf[emgHead] = s; emgBuf[emgHead].seq = emgSeq++; emgHead = next; }
```
The call sites set `s.seq = 0` as a placeholder — the real value is assigned under the lock.

**2. Reset all four counters under their respective spinlocks in `onConnect()`:**
```cpp
portENTER_CRITICAL(&emgMux);
emgDropped = 0;  emgSeq = 0;
portEXIT_CRITICAL(&emgMux);

portENTER_CRITICAL(&imuMux);
imuDropped = 0;  imuSeq = 0;
portEXIT_CRITICAL(&imuMux);
```

### Why `portMUX` (spinlock) and not FreeRTOS mutex here
`emg_push()` is called from `emgDrainTask` (a FreeRTOS task) but `emgMux` is a `portMUX_TYPE`
spinlock, not a FreeRTOS mutex. `portENTER_CRITICAL` / `portEXIT_CRITICAL` disable interrupts
on the calling core and are safe to use from task context for very short critical sections.
`onConnect()` runs on Core 0 in the BLE task context — also safe to use `portENTER_CRITICAL`.

---



### Problem 1: EMG Sample Timestamps Late by Up to One DMA Frame (~16 ms)
`s.t_us = micros()` was called when a pair was formed in `emgDrainTask`, not when the ADC
actually captured the signal. The DMA frame is 256 bytes = 64 results = 32 ENV+RAW pairs.
At 2000 pairs/s, one frame covers 16 ms. `adc_continuous_read()` blocks up to 10 ms.
So `s.t_us` could be up to ~16 ms later than the actual sample time — a systematic offset
that would misalign EMG and IMU signals during swing detection.

### Fix 1: Back-Calculate Actual ADC Capture Time
Capture `frameEndUs = micros()` immediately after `adc_continuous_read()` returns.
For each result slot `i` in the frame, the actual capture time is:

```
actual_t_us = frameEndUs - (numResults - 1 - i) * (1 000 000 / ADC_CONV_FREQ_HZ)
```

At 4000 conv/s, `US_PER_CONV = 250 µs`. The oldest result in a 64-result frame was
captured 63 × 250 = 15 750 µs before the newest. This corrects the systematic offset
down to the residual jitter of `micros()` itself (~1 µs).

| | Before | After |
|---|---|---|
| Timestamp source | `micros()` at pair-emit time | Back-calculated from frame-end time |
| Max timestamp error | ~16 ms (one full frame) | ~1 µs (micros() resolution) |
| IMU-EMG alignment | Systematic 16 ms offset | Accurate |

### Problem 2: Regression — `setSamplingPaused(true)` + `emg_adc_deinit()` Missing from `loop()` Pause Branches
During Session 3 refactoring, the disconnect and host-absent branches in `loop()` lost
their `setSamplingPaused(true)` and `emg_adc_deinit()` calls. The DMA would keep running
after the host disconnected or went silent.

### Fix 2: Restore Missing Calls
Both pause branches in `loop()` now correctly call:
```cpp
setSamplingPaused(true);
emg_adc_deinit();
```
before `rb_flush_all()` and `imu_ticks = 0`, matching the original intended behaviour.

---



### Problem 1: I2C Bus Shared Without Mutex
Three execution contexts used the `Wire` I2C bus concurrently without synchronisation:
- `imuTask` (Core 1): `myICM.dataReady()` + `myICM.getAGMT()`
- `spo2Task` (Core 1): `maxSensor.check()` + `maxSensor.getRed()` + `maxSensor.getIR()`
- `loop()` / `readBattery()` (Core 0): `lipo.getSOC()` + `lipo.getVoltage()` (every 1 s health report + every 30 s vitals frame)

Arduino `Wire` on ESP32 is **not thread-safe**. Concurrent transactions corrupt I2C bus state,
causing garbled sensor readings or SDA/SCL stuck-low bus lockups.

### Fix 1
Added `static SemaphoreHandle_t wireMutex` protecting every I2C call site:

| Call site | Context | Protection added |
|---|---|---|
| `lipo.getSOC/getVoltage/getChangeRate/isLow` | `readBattery()` → Core 0 | `wireMutex` take/give |
| `myICM.dataReady()` + `myICM.getAGMT()` | `imuTask` → Core 1 | `wireMutex` take/give |
| `maxSensor.check()` + `available()` | `spo2Task` → Core 1 | `wireMutex` take/give |
| `maxSensor.getRed/getIR/nextSample()` | `spo2Task` → Core 1 | `wireMutex` take/give |
| `configureMax3010x()` (brightness adjust) | `spo2Task` → Core 1 | `wireMutex` take/give |

Note: `Wire.begin()`, `myICM.begin()`, `maxSensor.begin()`, `lipo.begin()` in `setup()` are
**not** wrapped — they run before any tasks start, so no contention is possible.

### Problem 2: `samplingPaused` Cross-Core Visibility
`samplingPaused` is a `volatile bool` written on Core 0 and read on Core 1.
On ESP32 Xtensa LX6, `volatile` prevents compiler reordering but does **not** insert a
hardware memory barrier. The L1 data cache is not coherent between cores for DRAM without
explicit barriers — reads on Core 1 may see stale values under high load.

### Fix 2
Added atomic accessor functions using GCC built-ins:
```cpp
static inline void setSamplingPaused(bool v) {
  __atomic_store_n(&samplingPaused, v, __ATOMIC_RELEASE);
}
static inline bool getSamplingPaused() {
  return __atomic_load_n(&samplingPaused, __ATOMIC_ACQUIRE);
}
```
All read/write sites replaced with `getSamplingPaused()` / `setSamplingPaused()`.
Two exceptions use direct assign with comments: `setup()` end and `onConnect()` —
both run before tasks start, so no cross-core visibility issue exists there.

---



### Problem Identified
`adcHandle` was accessed from two cores without synchronisation:
- **Core 0** (`loop()`): calls `emg_adc_deinit()` → frees handle, sets `adcHandle = nullptr`
- **Core 1** (`emgDrainTask`): checks `adcHandle == nullptr`, then calls `adc_continuous_read(adcHandle, ...)`

The check and the read were not atomic, leaving a race window:

```
Core 1 (emgDrainTask)           Core 0 (loop)
─────────────────────           ─────────────
check: adcHandle != nullptr ✓
                                samplingPaused = true
                                emg_adc_deinit()
                                  → adc_continuous_deinit()
                                  → adcHandle = nullptr   ← freed!
adc_continuous_read(adcHandle)  ← called with freed handle → CRASH
```

`samplingPaused = true` being set first does not help because there is no memory barrier
guaranteeing Core 1 sees the updated value before it proceeds past its check.

### Fix
Added a **FreeRTOS mutex** (`SemaphoreHandle_t adcMutex`) protecting all access to `adcHandle`.

- `portMUX_TYPE` spinlock was **not** used — it is ISR-safe but not suitable for blocking
  across cores in a task context (would busy-spin and starve the scheduler).
- A FreeRTOS mutex is correct here because both callers (`loop()` and `emgDrainTask`) are tasks.

### Changes

| Location | Change |
|---|---|
| Global | Added `static SemaphoreHandle_t adcMutex = nullptr;` |
| `setup()` | `adcMutex = xSemaphoreCreateMutex()` + `configASSERT` before tasks start |
| `emg_adc_init()` | Takes mutex for entire init sequence; releases after `adc_continuous_start()` |
| `emg_adc_deinit()` | Takes mutex for entire deinit sequence; releases after `adcHandle = nullptr` |
| `emgDrainTask` | Takes mutex with 10 ms timeout before using `adcHandle`; re-checks `adcHandle == nullptr` after acquiring; releases immediately after `adc_continuous_read()` |

### Mutex Timeout in `emgDrainTask`
A 10 ms timeout (not `portMAX_DELAY`) is used so the drain task never blocks indefinitely
if `emg_adc_deinit()` is slow. On timeout, the task skips the cycle and retries next iteration.

---

## 2025 — Session 1: ADC DMA Migration + Bug Fixes

### Problem Identified
`analogRead()` was being called inside an `esp_timer` ISR at 2000 Hz (every 500 µs).  
On ESP32, `analogRead()` is **not ISR-safe** — it uses the ADC arbiter which can conflict  
with the BLE/WiFi radio. Each call blocks for ~50–100 µs, consuming 20–40% of a CPU core.

### Solution
Replaced software-timer + `analogRead()` with **ESP32 ADC Continuous (DMA) mode**  
(`esp_adc/adc_continuous.h`, IDF 5.x / Arduino core 3.3.5).  
DMA fills the buffer autonomously at hardware-clocked 4000 conv/s (2000 Hz per channel).  
A new `emgDrainTask` (FreeRTOS, Core 1, priority 2) drains the DMA buffer and pushes  
paired `EmgSample` entries into the existing ring buffer.

### Fixes Applied

| # | Category | Issue | Fix |
|---|---|---|---|
| 1 | 🔴 ISR safety | `analogRead()` called in `esp_timer` ISR at 2 kHz | Replaced with ADC DMA + `emgDrainTask` |
| 2 | 🔴 Crash | `adc_continuous_read()` called before `adcHandle == nullptr` guard | Guard moved **before** the read call |
| 3 | 🔴 Wrong API | `ADC_DIGI_OUTPUT_FORMAT_TYPE2` is S2/S3 only | Corrected to `ADC_DIGI_OUTPUT_FORMAT_TYPE1` for original ESP32 |
| 4 | 🟡 Compile | `ADC_CONV_FREQ_HZ` used `EMG_RATE_HZ` before it was defined | Moved ADC config block **after** `EMG_RATE_HZ` definition |
| 5 | 🟡 Compile | `{ .flush_pool = 0 }` nested designated initializer warning in C++ | Replaced with zero-init `= {}` |
| 6 | 🟡 Compile | `SOC_ADC_DIGI_RESULT_BYTES` removed in IDF 5.x | Replaced with `sizeof(adc_digi_output_data_t)` |
| 7 | 🟡 Compile | `r->type2.channel` / `r->type2.data` union removed in IDF 5.x | Replaced with `r->val` bit extraction (`>> 12` / `& 0x0FFF`) |
| 8 | 🟡 Dead code | `emgTimer` handle created but never started; `emg_timer_cb` was empty no-op | Both removed entirely |
| 9 | 🟡 Performance | `readBattery()` (I2C, ~100 µs) called on every `loop()` iteration (~1 kHz) for LED check | Replaced with cached `lastBatteryPct` (pure RAM read); I2C refresh unchanged at 10 s via `send_vitals_frame()` |

### Architecture After Changes

```
Before:
  esp_timer ISR (500 µs) → analogRead() x2 (blocking, ~100 µs each) → emg_push()

After:
  ADC DMA hardware (4000 conv/s, autonomous)
       ↓
  emgDrainTask (Core 1, priority 2)
       → drains DMA frame
       → demux ENV (ch3) / RAW (ch4) by channel tag
       → pairs into EmgSample
       → emg_push()
```

### ADC DMA Configuration

| Parameter | Value | Notes |
|---|---|---|
| Channels | ADC1_CH3 (PIN 39), ADC1_CH4 (PIN 32) | ENV and RAW |
| Attenuation | `ADC_ATTEN_DB_12` | ~0–3.3 V range |
| Bit width | 12-bit | Same as before |
| Format | `ADC_DIGI_OUTPUT_FORMAT_TYPE1` | Only valid format on original ESP32 |
| Total conv rate | 4000 conv/s | Round-robin: 2000 Hz per channel |
| DMA frame size | 256 bytes | 64 results per frame |
| Internal buffer | 1024 bytes (4× frame) | |

### ADC DMA Lifecycle

| Event | Action |
|---|---|
| Host heartbeat received (resume) | `emg_adc_init()` — start DMA |
| Host heartbeat lost (pause) | `emg_adc_deinit()` — stop DMA |
| BLE disconnect | `emg_adc_deinit()` — stop DMA |
| BLE connect | DMA stays stopped until first host PING |

### What Was NOT Changed
- Ring buffers, `EmgSample` / `ImuSample` structs
- All BLE, IMU, SpO2, battery, watchdog logic
- Protocol framing (`send_emg_frame`, `send_imu_frame`, `send_vitals_frame`)
- LED state machine patterns
- `samplingPaused` gate behaviour

### Target Platform
- Board: ESP32 WROOM-32E  
- Arduino ESP32 core: **3.3.5** (ESP-IDF 5.3.x)

---

*Add new sessions above this line, newest first.*
