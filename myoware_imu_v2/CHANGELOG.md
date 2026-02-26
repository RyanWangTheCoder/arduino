# MyoWare IMU v2 — Changelog

All notable changes to `myoware_imu_v2.ino` are documented here.  
Format: newest session at the top.

---

## High-Level Design Summary

### System Architecture

**Hardware:**
- ESP32 WROOM-32E (dual-core Xtensa LX6, 240 MHz)
- ICM-20948 9-axis IMU (I2C, 200 Hz)
- MyoWare EMG sensors: ENV (envelope) + RAW (rectified) channels (ADC DMA, 2000 Hz)
- MAX30105 pulse oximeter (I2C, 25 Hz)
- MAX17048 battery fuel gauge (I2C)
- BLE Nordic UART Service (NUS) for wireless data streaming

**Software Stack:**
- Arduino ESP32 core 3.3.5 (ESP-IDF 5.3.x)
- FreeRTOS (dual-core scheduler)
- ESP32 ADC Continuous (DMA) driver
- Custom binary protocol with CRC16 for reliable streaming

### Task Architecture (Dual-Core)

**Core 0 (Protocol CPU):**
- `loop()` — BLE TX, host watchdog, LED state machine (~1 kHz)
- BLE stack (callbacks: `onConnect`, `onDisconnect`, `onWrite`)
- `esp_timer` task (priority 22) — runs `imu_timer_cb` at 200 Hz

**Core 1 (Sensor CPU):**
- `emgDrainTask` (priority 2) — drains ADC DMA at 2 kHz, pairs ENV+RAW samples
- `imuTask` (priority 2) — reads IMU at 200 Hz (timer-driven)
- `spo2Task` (priority 0) — computes SpO2/HR every ~30 seconds (background)

**Priority Rationale:**
- Priority 2: Real-time sensor acquisition (EMG, IMU) — must not drop samples
- Priority 0: Background computation (SpO2) — no real-time requirement
- FreeRTOS priority inheritance prevents priority inversion on `wireMutex`

### Data Flow

```
Sensors (Core 1)          Ring Buffers (shared)     BLE TX (Core 0)
────────────────          ─────────────────────     ───────────────
ADC DMA (2 kHz)    ──→    emgBuf[2048]       ──→   send_emg_frame()
  ↓ emgDrainTask                                      (batched, CRC16)
  pairs ENV+RAW

ICM-20948 (200 Hz) ──→    imuBuf[512]        ──→   send_imu_frame()
  ↓ imuTask                                           (batched, CRC16)
  reads I2C

MAX30105 (25 Hz)   ──→    vitals (atomic)    ──→   send_vitals_frame()
  ↓ spo2Task                                          (every 30s)
  computes SpO2/HR
```

### Synchronization Primitives

**FreeRTOS Mutexes (with priority inheritance):**
- `adcMutex` — protects `adcHandle` (ADC DMA lifecycle) across Core 0 ↔ Core 1
- `wireMutex` — protects I2C bus (`Wire`) across `imuTask`, `spo2Task`, `loop()`

**Spinlocks (`portMUX_TYPE`):**
- `emgMux` — protects EMG ring buffer (`emgBuf`, `emgHead`, `emgTail`, `emgSeq`, `emgDropped`)
- `imuMux` — protects IMU ring buffer (`imuBuf`, `imuHead`, `imuTail`, `imuSeq`, `imuDropped`)
- `vitalsMux` — protects vitals state (SpO2, HR, battery)
- `hostMux` — protects host heartbeat timestamp (`lastHostPingMs`)

**Atomic Variables (GCC built-ins):**
- `samplingPaused` — cross-core flag (Core 0 writes, Core 1 reads)
  - Uses `__atomic_store_n` / `__atomic_load_n` with `__ATOMIC_RELEASE` / `__ATOMIC_ACQUIRE`
  - Ensures memory barrier for cross-core visibility on Xtensa LX6

**Volatile (single-writer, atomic hardware):**
- `imu_ticks` — 32-bit counter (Core 0 `esp_timer` task writes, Core 1 `imuTask` reads/decrements)
  - Lost ticks acceptable (just skips one IMU sample)
- `ledBrightness` — 8-bit value (Core 1 `spo2Task` writes, Core 0 `loop` reads)
  - Non-critical display value, 8-bit writes are atomic

### Critical Timing Constraints

| Task | Rate | Period | Budget | Worst-Case Blocking | Margin |
|---|---|---|---|---|---|
| `emgDrainTask` | 2 kHz | 500 µs | ~16 ms (DMA frame) | 10 ms (`adcMutex`) | ✅ Acceptable |
| `imuTask` | 200 Hz | 5 ms | 5 ms | ~400 µs (`wireMutex`) | ✅ 92% margin |
| `spo2Task` | 25 Hz | 40 ms | No deadline | N/A | ✅ Background |

**Key Design Decisions:**
1. **ADC DMA** — Hardware-clocked sampling eliminates ISR overhead and BLE radio conflicts
2. **Timestamp back-calculation** — Corrects ~16 ms DMA latency to ~1 µs accuracy
3. **Priority inheritance** — Prevents `spo2Task` (pri 0) from blocking `imuTask` (pri 2) indefinitely
4. **Host watchdog** — Pauses sampling when host is absent (prevents buffer overflow)
5. **Atomic accessors** — Ensures cross-core visibility without expensive mutexes

### Tricky Parts & Gotchas

**1. ADC DMA Channel-Order Violations (Session 7)**
- **Problem:** BLE radio interference can cause ADC arbiter to skip conversions
- **Symptom:** Two consecutive ENV or RAW results (ENV, ENV, RAW, RAW instead of ENV, RAW, ENV, RAW)
- **Solution:** Overwrite stale value, count violations in `emgChOrderViolations` for diagnostics
- **Why it's tricky:** Pairing logic must handle out-of-order results without dropping samples

**2. Cross-Core `samplingPaused` Visibility (Session 3)**
- **Problem:** `volatile` alone doesn't guarantee memory barrier on Xtensa LX6
- **Symptom:** Core 1 sees stale `samplingPaused` value, continues sampling after pause
- **Solution:** Use `__atomic_store_n` / `__atomic_load_n` with `RELEASE` / `ACQUIRE` ordering
- **Why it's tricky:** ESP32 L1 cache is not coherent between cores for DRAM

**3. `adcMutex` Held for 10 ms (Session 13)**
- **Problem:** `emgDrainTask` holds `adcMutex` during `adc_continuous_read()` (10 ms timeout)
- **Concern:** Does this block `emg_adc_deinit()` too long?
- **Analysis:** All deinit call sites are non-real-time (disconnect, host timeout)
- **Why it's tricky:** Must hold mutex across entire read to prevent use-after-free

**4. Priority Inheritance on `wireMutex` (Session 13)**
- **Problem:** `spo2Task` (pri 0) holds `wireMutex`, `imuTask` (pri 2) blocks
- **Solution:** `xSemaphoreCreateMutex()` provides automatic priority inheritance
- **Result:** `spo2Task` temporarily raised to pri 2 while holding mutex
- **Why it's tricky:** Without inheritance, `emgDrainTask` (pri 2) would preempt `spo2Task`, causing unbounded delay

**5. EMG Timestamp Back-Calculation (Session 4)**
- **Problem:** `micros()` called at pair-emit time, not ADC capture time
- **Error:** Up to 16 ms late (one full DMA frame)
- **Solution:** Capture `frameEndUs` after `adc_continuous_read()`, back-calculate each sample's age
- **Formula:** `actual_t_us = frameEndUs - (numResults - 1 - i) * 250 µs`
- **Why it's tricky:** DMA delivers results in batches; must reconstruct original capture time

**6. `esp_timer` Callbacks Are Tasks, Not ISRs (Session 14)**
- **Misconception:** `esp_timer` callbacks run in hardware ISR context
- **Reality:** `ESP_TIMER_TASK` dispatch runs callbacks in FreeRTOS task (pri 22)
- **Implication:** Safe to use `__atomic_load_n`, `xSemaphoreTake`, etc. in `imu_timer_cb`
- **Why it's tricky:** ESP-IDF documentation is subtle about this distinction

**7. Sequence Number Assignment Under Lock (Session 5)**
- **Problem:** `emgSeq++` was outside `emgMux`, causing race with `onConnect()` reset
- **Solution:** Assign `seq` inside `emg_push()` under `emgMux` spinlock
- **Pattern:** Caller sets `s.seq = 0` (placeholder), `emg_push()` overwrites with `emgSeq++`
- **Why it's tricky:** Sequence numbers must be monotonic and gap-free for protocol correctness

**8. Battery Read Caching (Session 9)**
- **Problem:** `printBatteryStatus()` called `readBattery()` every 1 second (4 I2C transactions)
- **Impact:** ~400 µs I2C bus time, `wireMutex` contention with real-time tasks
- **Solution:** Cache `lastBatteryPct` / `lastBatteryVoltage`, refresh every 10 seconds
- **Why it's tricky:** Debug prints must not impact real-time performance

### Protocol Design

**Frame Structure:**
```
[Header: 14 bytes]
  magic0, magic1 (0xA5, 0x5A)
  proto_ver (0x02)
  flags (CRC16 | IMU_META_V2)
  frame_seq (uint32_t)
  frame_t_us (uint32_t)
  payload_len (uint16_t)

[Payload: TLV format]
  TLV_EMG_BATCH (0x01):
    seq, t_offset, rate, channels, count
    [env, raw] × count
  
  TLV_IMU_BATCH (0x02):
    seq, t_offset, rate, count, axes_mask
    acc_fsr_code, gyro_fsr_code
    [ax, ay, az, gx, gy, gz] × count
  
  TLV_VITALS (0x03):
    spo2, hr_x10, age_ms, flags
    brightness, battery_pct, battery_mv

[CRC16: 2 bytes]
  CRC-CCITT over header + payload
```

**Key Features:**
- **Batching:** Multiple samples per frame (reduces BLE overhead)
- **Timestamp offset:** Relative to frame timestamp (saves 4 bytes per sample)
- **FSR metadata:** Logged in IMU frames for host-side scaling
- **CRC16:** Detects corruption over BLE (Nordic UART is not reliable)

### Host Watchdog

**Policy:**
- Host sends `PING` every ~1 second
- Device expects PING within 2 seconds (warn)
- Device pauses sampling after 5 seconds silence (prevents buffer overflow)
- Device reboots after 30 seconds silence (recovers stuck BLE state)

**State Machine:**
```
Disconnected ──connect──→ Wait Host ──PING──→ Streaming
     ↑                         ↓                    ↓
     └────────────────────5s timeout────────────────┘
                              ↓
                         30s timeout
                              ↓
                           Reboot
```

### LED Patterns

| Mode | Pattern | Meaning |
|---|---|---|
| `LED_DISCONNECTED` | Slow blink (1 Hz) | No BLE connection |
| `LED_WAIT_HOST` | Double-blink (every 2s) | Connected, waiting for host PING |
| `LED_STREAMING` | Solid ON | Streaming data to host |
| `LED_BACKLOG` | Fast blink (16 Hz) | Ring buffer backlog high (MTU too low) |
| `LED_LOW_BATTERY` | Very fast blink (10 Hz) | Battery < 20% |

---

## 2025 — Session 15: Fix — Inconsistent `samplingPaused` Write in `onConnect()`

### Problem
The comment on line 489 in `onConnect()` was incorrect:
```cpp
samplingPaused = true;  // direct assign OK here — BLE callback runs before tasks start
```

**The comment is wrong because:**
- `onConnect()` is called when a BLE client connects **AFTER** `setup()` completes
- Tasks (`emgDrainTask`, `imuTask`, `spo2Task`) are already running at this point
- `emgDrainTask` on Core 1 is actively reading `samplingPaused` via `getSamplingPaused()`

**The inconsistency:**
- `onDisconnect()` line 498 uses `setSamplingPaused(true)` (atomic with memory barrier)
- `onConnect()` line 489 uses direct assign `samplingPaused = true` (no memory barrier)

**Why it hasn't caused a visible bug:**
- `onConnect()` sets `samplingPaused = true` (pausing), which is the safe direction
- The ADC DMA hasn't been started yet (it starts only after the first PING in `loop()`)
- `emgDrainTask` checks `getSamplingPaused()` first thing in its loop, and if true, it just sleeps for 5ms
- So even if Core 1 sees a stale `false` value briefly, it will just do one extra iteration before seeing the updated `true`

**But it's still wrong** — it violates the documented pattern and could theoretically cause a race.

### Fix
Changed `onConnect()` line 489 to use `setSamplingPaused(true)` (atomic write with memory barrier),
matching the pattern used in `onDisconnect()` and all other cross-core writes to `samplingPaused`.

```cpp
// Before:
samplingPaused = true;  // direct assign OK here — BLE callback runs before tasks start

// After:
setSamplingPaused(true);  // atomic write — emgDrainTask on Core 1 reads this
```

This ensures consistent use of atomic accessors for all cross-core access to `samplingPaused`.

### Verification of Other Cross-Core Variables

All other cross-core accesses were reviewed and confirmed correct:

| Variable | Writer | Reader | Protection | Status |
|---|---|---|---|---|
| `samplingPaused` | Core 0 (`loop`, BLE callbacks) | Core 1 (`emgDrainTask`, `imuTask`) | `setSamplingPaused()` / `getSamplingPaused()` (atomic) | ✅ Fixed |
| `imu_ticks` | Core 0 (`imu_timer_cb` task) | Core 1 (`imuTask`) | `volatile` + 32-bit atomic writes | ✅ Acceptable (lost ticks OK) |
| `ledBrightness` | Core 1 (`spo2Task`) | Core 0 (`send_vitals_frame`) | 8-bit atomic reads/writes | ✅ Acceptable (non-critical display value) |
| `lastBatteryPct` / `lastBatteryVoltage` | Core 0 (`readBattery`) | Core 0 (`printBatteryStatus`, LED check) | Same core | ✅ No issue |
| `lastBatteryReadMs` / `batteryInitialReadDone` | Core 0 (`send_vitals_frame`) | Core 0 (`printBatteryStatus`) | Same core | ✅ No issue |
| `isConnected` | Core 0 (BLE callbacks) | Core 0 (`loop`) | Same core | ✅ No issue |
| `negotiatedMTU` / `maxNotifyPayload` | Core 0 (`onConnect`) | Core 0 (`loop`) | Same core | ✅ No issue |
| `frameSeq` | Core 0 (`loop`) | Core 0 (`loop`) | Same core | ✅ No issue |
| `emgSeq` / `imuSeq` / `emgDropped` / `imuDropped` | Core 0 (`onConnect`) / Core 1 (tasks) | Both cores | `emgMux` / `imuMux` spinlocks | ✅ Correct (Session 5) |
| `emgChOrderViolations` | Core 1 (`emgDrainTask`) / Core 0 (`onConnect`) | Core 0 (`loop`) | `emgMux` spinlock | ✅ Correct (Session 7) |

---

## 2025 — Session 14: Analysis — `getSamplingPaused()` in `imu_timer_cb` (Safe from ESP32 `esp_timer` Task Context)

### Concern Raised
> `imu_timer_cb` is an `esp_timer` callback. Is it safe to call `getSamplingPaused()`
> (which uses `__atomic_load_n`) from this context?

### Analysis: Safe — `esp_timer` Callbacks Run in Task Context, Not Hardware ISR

The relevant code:
```cpp
// Line 447-450
static void imu_timer_cb(void* /*arg*/) {
  if (getSamplingPaused()) return;
  imu_ticks++;
}

static inline bool getSamplingPaused() {
  return __atomic_load_n(&samplingPaused, __ATOMIC_ACQUIRE);
}
```

**Key fact:** On ESP32, `esp_timer` callbacks are **not hardware ISRs**. They run in a
dedicated FreeRTOS task (`esp_timer` task) at priority 22 (configurable). This is
explicitly configured in `setup()` line 1210:

```cpp
esp_timer_create_args_t imuArgs = {
  .callback = &imu_timer_cb,
  .arg = nullptr,
  .dispatch_method = ESP_TIMER_TASK,  // ← Runs in task context
  .name = "imu_timer"
};
```

The `ESP_TIMER_TASK` dispatch method guarantees the callback runs in the `esp_timer` task,
not in a hardware interrupt. This means:

1. **`__atomic_load_n` is safe** — GCC atomic built-ins are safe from any context (task or ISR)
2. **`__ATOMIC_ACQUIRE` memory ordering is correct** — ensures cross-core visibility of
   `samplingPaused` writes from Core 0 to reads on Core 1
3. **`imu_ticks++` is safe** — 32-bit aligned writes are atomic on Xtensa LX6; `volatile`
   prevents compiler reordering

### What Would Be Unsafe (If This Were a Hardware ISR)

If `esp_timer` callbacks were hardware ISRs (they're not), the following would be unsafe:

| Operation | Safe in hardware ISR? | Safe in `esp_timer` task? |
|---|---|---|
| `xSemaphoreTake()` | ❌ No — would crash | ✅ Yes |
| `portENTER_CRITICAL()` | ⚠️ Only with `_ISR` variant | ✅ Yes |
| `__atomic_load_n()` | ✅ Yes | ✅ Yes |
| `vTaskDelay()` | ❌ No — would crash | ✅ Yes |

But since `esp_timer` callbacks run in task context, all FreeRTOS APIs are safe to use
(within reason — long-running operations would delay other timer callbacks).

### Conclusion
No code change required. `getSamplingPaused()` is safe to call from `imu_timer_cb` because:

1. `esp_timer` callbacks run in the `esp_timer` FreeRTOS task, not a hardware ISR
2. `__atomic_load_n` is safe from any context (task or ISR)
3. The `__ATOMIC_ACQUIRE` memory ordering ensures cross-core visibility of `samplingPaused`

The current implementation is correct.

---

## 2025 — Session 13: Analysis — `adcMutex` Held for 10 ms During `adc_continuous_read()` (Not a Bug)

### Concern Raised
> `emgDrainTask` holds `adcMutex` for the full duration of `adc_continuous_read()`,
> which has a 10 ms timeout. Does this block `emg_adc_deinit()` for too long?

### Analysis: 10 ms Hold Is Bounded, Intentional, and Acceptable

The relevant code:
```cpp
// emgDrainTask (Core 1)
esp_err_t ret = adc_continuous_read(adcHandle, dmaBuf, ADC_FRAME_SIZE, &bytesRead, 10);
xSemaphoreGive(adcMutex);  // release immediately after read

// emg_adc_deinit() (Core 0)
xSemaphoreTake(adcMutex, portMAX_DELAY);  // waits up to 10 ms
adc_continuous_stop(adcHandle);
adc_continuous_deinit(adcHandle);
adcHandle = nullptr;
xSemaphoreGive(adcMutex);
```

**Maximum blocking time for `emg_adc_deinit()`:** 10 ms (one `adc_continuous_read` timeout).

**All three `emg_adc_deinit()` call sites:**

| Call site | Context | 10 ms delay acceptable? |
|---|---|---|
| `onDisconnect()` line 509 | BLE callback, Core 0 | ✅ Yes — disconnect is not time-critical |
| `loop()` disconnect branch line 1264 | Core 0 task | ✅ Yes — `loop()` runs at ~1 kHz, 10 ms is one iteration |
| `loop()` host-absent branch line 1274 | Core 0 task | ✅ Yes — triggered after 5 s silence, 10 ms is negligible |

None of the deinit call sites have a hard real-time deadline. A 10 ms wait is
indistinguishable from normal `loop()` jitter in all three cases.

### Why `portMAX_DELAY` in `emg_adc_deinit()` Is Correct
Using `portMAX_DELAY` in `deinit` (rather than a timeout) is intentional:
- `deinit` **must** complete — a partial deinit leaving `adcHandle` non-null would
  cause `emg_adc_init()` to skip re-initialisation on the next resume.
- The maximum wait is bounded at 10 ms by the `adc_continuous_read` timeout in
  `emgDrainTask`, so `portMAX_DELAY` never actually waits indefinitely.

### Why the Mutex Is Held Across the Full `adc_continuous_read()` Call
Releasing the mutex before `adc_continuous_read()` returns would create a window
where `emg_adc_deinit()` could free `adcHandle` while `adc_continuous_read()` is
still using it — the exact crash scenario Session 2 was designed to prevent.
The 10 ms hold is the minimum necessary to close that window.

### Conclusion
No code change required. The 10 ms mutex hold is bounded, intentional, and
acceptable at all call sites. `portMAX_DELAY` in `deinit` is safe because the
maximum wait is bounded by the `adc_continuous_read` timeout.

---



### Concern Raised
> `spo2Task` (priority 0) holds `wireMutex` during I2C reads. `imuTask` (priority 2)
> blocks waiting for it. Classic priority inversion — is this handled?

### Analysis: FreeRTOS Priority Inheritance Covers This

`wireMutex` is created with `xSemaphoreCreateMutex()` (line 1210), which is the
FreeRTOS **mutex with priority inheritance** — not `xSemaphoreCreateBinary()` (no
inheritance) or `xSemaphoreCreateRecursiveMutex()` (recursive, different semantics).

When `imuTask` (priority 2) blocks on `wireMutex` held by `spo2Task` (priority 0),
FreeRTOS automatically raises `spo2Task`'s effective priority to 2 for the duration
of the lock hold. This ensures `spo2Task` is scheduled promptly to release the mutex,
preventing `imuTask` from being indefinitely delayed by any lower-priority task.

```
Normal case (no contention):
  spo2Task (pri 0) holds wireMutex → runs at pri 0
  imuTask  (pri 2) not blocked     → preempts freely

Priority inversion window:
  spo2Task (pri 0) holds wireMutex
  imuTask  (pri 2) blocks on wireMutex
  FreeRTOS: spo2Task temporarily raised to pri 2
  spo2Task completes I2C, releases wireMutex
  FreeRTOS: spo2Task restored to pri 0
  imuTask  (pri 2) acquires wireMutex, proceeds
```

### Worst-Case Blocking Time for `imuTask`
`spo2Task` holds `wireMutex` for one I2C transaction at a time:
- `maxSensor.check()` — one I2C read, ~100–400 µs at 400 kHz
- `maxSensor.getRed/getIR/nextSample()` — one batch, ~200–500 µs
- `configureMax3010x()` — multi-register write, ~500–800 µs (rare)

`imuTask` budget per tick: 5 ms (200 Hz). Worst-case I2C hold ~800 µs.
Blocking time is well within the 5 ms budget even without priority inheritance.

### Why `adcMutex` Does Not Have This Issue
`emgDrainTask` (priority 2) holds `adcMutex` only during `adc_continuous_read()`,
which is called with a 10 ms timeout. `loop()` (Core 0) calls `emg_adc_deinit()`
which takes `adcMutex` — but `loop()` and `emgDrainTask` are on **different cores**,
so this is cross-core mutex contention, not priority inversion (different scheduler
domains on ESP32 dual-core).

### Conclusion
No code change required. `xSemaphoreCreateMutex()` provides priority inheritance.
Worst-case blocking time for `imuTask` is ~800 µs, well within the 5 ms tick budget.

---



### Concern Raised
> `readBattery()` uses `if (wireMutex) xSemaphoreTake(...)` — is the conditional guard
> necessary, and could it mask a real bug?

### Analysis: Guard Is Defensive Dead-Code, Correct As-Is

`readBattery()` is called from exactly **one** site:

| Call site | Context | `wireMutex` state |
|---|---|---|
| `send_vitals_frame()` line 952 | `loop()` task, Core 0 | Always non-null (created in `setup()` before tasks start) |

The two other historical call sites no longer exist:
- `printBatteryStatus()` — rewritten in Session 9 to use cached values; no longer calls `readBattery()`
- `setup()` initial read — calls `lipo.getSOC()` / `lipo.getVoltage()` **directly**, not through `readBattery()`

The `setup()` direct calls are correct: they run before `wireMutex` is created and before
any tasks start, so there is no contention and no mutex is needed.

`readBattery()` itself is only reachable after `setup()` completes (via `loop()`), at which
point `wireMutex` is guaranteed non-null. The `if (wireMutex)` guard is therefore
defensive dead-code — it can never be false in practice.

### Why Keep the Guard
It costs nothing (one pointer comparison). It prevents a hard fault if `readBattery()` is
ever called from a new code path before `setup()` completes — e.g. during future
refactoring. It is consistent with the comment on `emg_adc_init()` which has the same
pattern (`if (!adcMutex) return`).

### Conclusion
No code change required. The guard is correct and intentional.

---



### Concern Raised
> `wireMutex` is created after `BLEDevice::startAdvertising()`. Could a BLE callback
> fire before `wireMutex` exists and try to take it?

### Analysis: Order Is Correct

The `setup()` sequence is:
```
Wire.begin() + sensor inits     ← single-threaded, no mutex needed
BLEDevice::init / startAdvertising  ← BLE stack starts here
adcMutex  = xSemaphoreCreateMutex() ← created
wireMutex = xSemaphoreCreateMutex() ← created
configASSERT(both)              ← verified non-null
esp_timer_create/start          ← imu_timer_cb only touches imu_ticks, no mutex
xTaskCreatePinnedToCore × 3    ← tasks start AFTER mutexes exist
```

`BLEDevice::startAdvertising()` does start the BLE stack before the mutexes are created.
However, the BLE callbacks that use `wireMutex` are:
- `onConnect()` — uses `emgMux` / `imuMux` spinlocks only, **not** `wireMutex`
- `onDisconnect()` — calls `emg_adc_deinit()`, uses `adcMutex` only
- `onWrite()` (RxCallbacks) — uses `hostMux` spinlock only

`wireMutex` is only taken in:
- `readBattery()` — called from `send_vitals_frame()` in `loop()` (Core 0 task)
- `imuTask` — FreeRTOS task
- `spo2Task` — FreeRTOS task

All three are FreeRTOS tasks created **after** `wireMutex` exists. No BLE callback
ever calls `xSemaphoreTake(wireMutex, ...)`, so there is no window where a BLE
callback could race against `wireMutex` being null.

`readBattery()` has an explicit null guard: `if (wireMutex) xSemaphoreTake(...)` —
this would safely skip the lock if somehow called before init, but that path
does not exist in practice.

### Conclusion
No code change required. Mutex creation order is correct and safe.

---



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
