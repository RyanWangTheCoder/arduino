# Battery Monitoring Integration Patch

## Part 1: Arduino Firmware Modifications (myoware_imu_v2.ino)

### Changes Summary:
1. Add MAX17048 library and global object
2. Add battery state tracking variables
3. Add battery initialization in setup() with graceful degradation
4. Add battery reading function with safety checks
5. Add battery to Serial debug prints
6. Add LOW_BATTERY LED mode
7. Modify VitalsSample struct to include battery fields
8. Update send_vitals_frame() to include battery data

---

### Step 1: Add Library Include (after line 11)

```cpp
#include "MAX30105.h"
#include "spo2_algorithm.h"

// NEW: Add MAX17048 fuel gauge library
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
```

---

### Step 2: Update LED Enum (line 15)

```cpp
// OLD:
enum LedMode : uint8_t { LED_DISCONNECTED, LED_WAIT_HOST, LED_STREAMING, LED_BACKLOG };

// NEW:
enum LedMode : uint8_t { LED_DISCONNECTED, LED_WAIT_HOST, LED_STREAMING, LED_BACKLOG, LED_LOW_BATTERY };
```

---

### Step 3: Add Battery Objects and Variables (after line 127, after MAX30105 declaration)

```cpp
// MAX3010x
MAX30105 maxSensor;
static constexpr uint8_t MAX3010X_ADDR = 0x57;

// NEW: MAX17048 Battery Fuel Gauge
SFE_MAX1704X lipo(MAX1704X_MAX17048);
static bool batteryAvailable = false;
static float lastBatteryPct = 100.0f;
static float lastBatteryVoltage = 4.2f;
static uint32_t lastBatteryReadMs = 0;
static constexpr uint32_t BATTERY_READ_INTERVAL_MS = 10000;  // Read every 10 seconds
```

---

### Step 4: Add Battery State Struct and Reading Function (after line 160, before VitalsState)

```cpp
// NEW: Battery state structure
struct BatteryState {
    float pct;          // 0-100%, or NaN if unavailable
    float voltage;      // Volts, or NaN if unavailable
    float changeRate;   // %/hr, or NaN if unavailable
    bool isLow;         // Low battery alert
    bool isValid;       // True if data is valid
};

BatteryState readBattery() {
    BatteryState state;
    state.pct = NAN;
    state.voltage = NAN;
    state.changeRate = NAN;
    state.isLow = false;
    state.isValid = false;
    
    // Check if battery monitor is available
    if (!batteryAvailable) {
        return state;  // Return invalid state
    }
    
    // Try to read (SparkFun library has built-in I2C timeout)
    state.pct = lipo.getSOC();
    state.voltage = lipo.getVoltage();
    state.changeRate = lipo.getChangeRate();
    state.isLow = lipo.isLow();
    
    // Validate readings (sanity check)
    if (state.pct < 0.0f || state.pct > 100.0f) {
        state.pct = NAN;
    }
    if (state.voltage < 2.5f || state.voltage > 4.5f) {
        state.voltage = NAN;
    }
    
    // Mark as valid if at least one field is good
    state.isValid = !isnan(state.pct) || !isnan(state.voltage);
    
    // Cache for fallback
    if (!isnan(state.pct)) lastBatteryPct = state.pct;
    if (!isnan(state.voltage)) lastBatteryVoltage = state.voltage;
    
    return state;
}

void printBatteryStatus() {
    if (!batteryAvailable) {
        Serial.println("[Battery] N/A (MAX17048 not detected)");
        return;
    }
    
    BatteryState batt = readBattery();
    
    if (!batt.isValid) {
        Serial.println("[Battery] ERROR (failed to read)");
        return;
    }
    
    Serial.print("[Battery] ");
    
    if (!isnan(batt.pct)) {
        Serial.print(batt.pct, 1);
        Serial.print("%");
    } else {
        Serial.print("?%");
    }
    
    Serial.print(" (");
    if (!isnan(batt.voltage)) {
        Serial.print(batt.voltage, 2);
        Serial.print("V");
    } else {
        Serial.print("?.??V");
    }
    Serial.print(")");
    
    if (!isnan(batt.changeRate)) {
        if (batt.changeRate > 0) {
            Serial.print(" ↑+");
        } else {
            Serial.print(" ↓");
        }
        Serial.print(batt.changeRate, 1);
        Serial.print("%/hr");
    }
    
    if (batt.isLow) {
        Serial.print(" [LOW!]");
    }
    
    Serial.println();
}

// Shared vitals state (written by task, read by BLE loop)
struct VitalsState {
  uint16_t spo2_u16;       // 0..100 or 0xFFFF invalid
  uint16_t hr_x10_u16;     // bpm*10 or 0xFFFF invalid
  uint32_t computed_ms;    // millis() when computed
  uint8_t  flags;          // bit0 spo2Valid, bit1 hrValid
  byte     brightness;
  uint8_t  battery_pct;    // NEW: 0-100 or 255 if unknown
  uint16_t battery_mv;     // NEW: millivolts or 0 if unknown
};
static VitalsState vitals = {0xFFFF, 0xFFFF, 0, 0, ledBrightness, 255, 0};
```

---

### Step 5: Add Battery Initialization in setup() (after MAX3010x init, around line 577)

```cpp
  // ---- MAX3010x init (non-fatal if missing) ----
  Serial.println("Initializing MAX3010x...");
  if (maxSensor.begin(Wire, I2C_SPEED_FAST, MAX3010X_ADDR)) {
    max3010xPresent = true;
    configureMax3010x();
    Serial.println("MAX3010x OK.");
  } else {
    max3010xPresent = false;
    Serial.println("[WARN] MAX3010x not found; continuing without SpO2/HR.");
  }

  // NEW: ---- MAX17048 Battery init (non-fatal if missing) ----
  Serial.print("[Battery] Checking for MAX17048... ");
  if (lipo.begin() == false) {
    Serial.println("NOT FOUND. Battery monitoring disabled.");
    batteryAvailable = false;
  } else {
    Serial.println("OK.");
    batteryAvailable = true;
    
    // Configure alerts
    lipo.setThreshold(20);  // Alert at 20% battery
    lipo.enableSOCAlert();  // Enable 1% change alerts
    
    // Optional: Set voltage thresholds
    lipo.setVALRTMax(4.1);  // High voltage alert (4.1V)
    lipo.setVALRTMin(3.3);  // Low voltage alert (3.3V)
    
    // Read initial state
    float pct = lipo.getSOC();
    float volt = lipo.getVoltage();
    Serial.print("[Battery] Initial: ");
    Serial.print(pct, 1);
    Serial.print("% (");
    Serial.print(volt, 2);
    Serial.println("V)");
    
    lastBatteryPct = pct;
    lastBatteryVoltage = volt;
  }
```

---

### Step 6: Update send_vitals_frame() (around line 450)

Find the function `send_vitals_frame()` and modify it:

```cpp
bool send_vitals_frame(uint16_t maxPayload) {
  const uint16_t headerBytes = 14;
  const uint16_t tlvHdrBytes = 3;
  const uint16_t vitalsBytes = 11;  // CHANGED: was 8, now 11 (added 3 bytes)
  const uint16_t crcBytes = 2;

  if (maxPayload < (headerBytes + tlvHdrBytes + vitalsBytes + crcBytes)) return false;

  VitalsState v;
  portENTER_CRITICAL(&vitalsMux);
  v = vitals;
  portEXIT_CRITICAL(&vitalsMux);

  // NEW: Read battery periodically
  uint32_t nowMs = millis();
  if ((nowMs - lastBatteryReadMs) >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryReadMs = nowMs;
    
    if (batteryAvailable) {
      BatteryState batt = readBattery();
      
      if (batt.isValid && !isnan(batt.pct)) {
        v.battery_pct = (uint8_t)constrain((int)batt.pct, 0, 100);
      } else {
        v.battery_pct = 255;  // Unknown
      }
      
      if (batt.isValid && !isnan(batt.voltage)) {
        v.battery_mv = (uint16_t)(batt.voltage * 1000.0f);
      } else {
        v.battery_mv = 0;  // Unknown
      }
      
      // Update cached vitals
      portENTER_CRITICAL(&vitalsMux);
      vitals.battery_pct = v.battery_pct;
      vitals.battery_mv = v.battery_mv;
      portEXIT_CRITICAL(&vitalsMux);
    } else {
      v.battery_pct = 255;  // MAX17048 not available
      v.battery_mv = 0;
    }
  }

  uint16_t age_ms = 0;
  uint32_t now = millis();
  if (v.computed_ms != 0) {
    uint32_t age = (now >= v.computed_ms) ? (now - v.computed_ms) : 0;
    age_ms = (age > 65535) ? 65535 : (uint16_t)age;
  } else {
    age_ms = 65535;
  }

  uint8_t buf[64];
  uint8_t* p = buf;

  uint32_t now_us = micros();

  // Header
  p[0] = MAGIC0; p[1] = MAGIC1; p[2] = PROTO_VER; p[3] = FLAG_CRC16;
  p += 4;
  wr_u32(p, frameSeq++);
  wr_u32(p, now_us);
  wr_u16(p, 0);

  uint8_t* payload_start = buf + headerBytes;
  p = payload_start;

  *p++ = TLV_VITALS;
  wr_u16(p, vitalsBytes);

  wr_u16(p, v.spo2_u16);
  wr_u16(p, v.hr_x10_u16);
  wr_u16(p, age_ms);
  *p++ = v.flags;
  *p++ = (uint8_t)v.brightness;
  *p++ = v.battery_pct;    // NEW
  wr_u16(p, v.battery_mv); // NEW

  uint16_t payloadLen = (uint16_t)(p - payload_start);
  buf[12] = payloadLen & 0xFF;
  buf[13] = payloadLen >> 8;

  size_t preCrcLen = headerBytes + payloadLen;
  uint16_t crc = crc16_ccitt(buf, preCrcLen);
  wr_u16(p, crc);

  size_t totalLen = preCrcLen + 2;
  if (totalLen > maxPayload) return false;

  pTXCharacteristic->setValue(buf, totalLen);
  pTXCharacteristic->notify();
  return true;
}
```

---

### Step 7: Update LED State Machine (around line 480)

```cpp
static void led_update(LedMode mode) {
  uint32_t now = millis();

  if (mode != ledMode) {
    ledMode = mode;
    ledLastMs = now;
    ledPhase = 0;
    led_set(false);
  }

  switch (ledMode) {
    case LED_DISCONNECTED: {
      if (now - ledLastMs >= 500) {
        ledLastMs = now;
        led_set(!digitalRead(PIN_LED));
      }
    } break;

    case LED_WAIT_HOST: {
      const uint32_t t = now - ledLastMs;
      bool on = false;
      if (t < 120) on = true;
      else if (t < 240) on = false;
      else if (t < 360) on = true;
      else on = false;
      led_set(on);
      if (t >= 2000) {
        ledLastMs = now;
      }
    } break;

    case LED_STREAMING: {
      led_set(true);
    } break;

    case LED_BACKLOG: {
      if (now - ledLastMs >= 62) {
        ledLastMs = now;
        led_set(!digitalRead(PIN_LED));
      }
    } break;

    // NEW: Low battery warning (fast blink)
    case LED_LOW_BATTERY: {
      if (now - ledLastMs >= 100) {  // Very fast blink (10 Hz)
        ledLastMs = now;
        led_set(!digitalRead(PIN_LED));
      }
    } break;
  }
}
```

---

### Step 8: Update loop() LED Mode Decision (around line 650)

```cpp
  // ---- Decide LED mode ----
  LedMode mode = LED_DISCONNECTED;
  
  // NEW: Priority 1 - Check for low battery first
  if (batteryAvailable) {
    BatteryState batt = readBattery();
    if (batt.isValid && !isnan(batt.pct) && batt.pct < 20.0f) {
      mode = LED_LOW_BATTERY;
    }
  }
  
  // Priority 2 - Other states (only if not low battery)
  if (mode != LED_LOW_BATTERY) {
    if (!isConnected) {
      mode = LED_DISCONNECTED;
    } else if (samplingPaused) {
      mode = LED_WAIT_HOST;
    } else {
      if (emg_available() > EMG_BACKLOG_PAUSE_THRESH || imu_available() > IMU_BACKLOG_PAUSE_THRESH) {
        mode = LED_BACKLOG;
      } else {
        mode = LED_STREAMING;
      }
    }
  }
  
  led_update(mode);
```

---

### Step 9: Add Battery to Debug Report (around line 690)

```cpp
  // ---- Debug report every second ----
  if (nowMs - lastDebugMs >= HEALTH_REPORT_MS) {
    lastDebugMs = nowMs;
    uint32_t fps = framesSent - lastFramesSent;
    lastFramesSent = framesSent;

    VitalsState v;
    portENTER_CRITICAL(&vitalsMux);
    v = vitals;
    portEXIT_CRITICAL(&vitalsMux);

    String sSpo2 = (v.flags & 0x01) ? String(v.spo2_u16) : "NA";
    String sHr   = (v.flags & 0x02) ? String(v.hr_x10_u16 / 10.0f, 1) : "NA";

    // NEW: Print battery status
    printBatteryStatus();

    Serial.printf("[Health] MTU=%u EMGavail=%u IMUavail=%u EMGdrop=%lu IMUdrop=%lu Frames/s=%lu | HostAge=%lu ms Paused=%u | SpO2=%s HR=%s\n",
                  negotiatedMTU, emg_available(), imu_available(),
                  (unsigned long)emgDropped, (unsigned long)imuDropped, (unsigned long)fps,
                  (unsigned long)hostAge, samplingPaused ? 1 : 0,
                  sSpo2.c_str(), sHr.c_str());
  }
```

---

## Part 2: Python btCentral.py Modifications

### Changes Summary:
1. Update decode_vitals() to parse battery fields
2. Update vitals CSV header
3. Add battery to health report
4. Add low battery warnings

---

### Step 1: Update decode_vitals() Function

Find the `decode_vitals()` function and replace it with:

```python
def decode_vitals(val: bytes) -> Dict[str, Optional[float]]:
    """Decode Vitals TLV payload (with battery data, backward compatible)."""
    if len(val) < 14:
        # Old format (no battery) - backward compatible
        if len(val) >= 8:
            spo2_u16, hr_x10_u16, age_ms, flags, bright = struct.unpack_from("<HHHBB", val, 0)
            return {
                "spo2": float(spo2_u16) / 100.0 if spo2_u16 < 10000 else None,
                "hr": float(hr_x10_u16) / 10.0 if hr_x10_u16 < 3000 else None,
                "age_ms": int(age_ms),
                "flags": int(flags),
                "brightness": int(bright),
                "battery_pct": None,  # Not available in old format
                "battery_v": None,
            }
        return {}
    
    # New format (with battery)
    spo2_u16, hr_x10_u16, age_ms, flags, bright, batt_pct, batt_mv = struct.unpack_from("<HHHBBBH", val, 0)
    
    return {
        "spo2": float(spo2_u16) / 100.0 if spo2_u16 < 10000 else None,
        "hr": float(hr_x10_u16) / 10.0 if hr_x10_u16 < 3000 else None,
        "age_ms": int(age_ms),
        "flags": int(flags),
        "brightness": int(bright),
        "battery_pct": int(batt_pct) if batt_pct < 255 else None,  # 255 = unknown
        "battery_v": float(batt_mv) / 1000.0 if batt_mv > 0 else None,  # 0 = unknown
    }
```

---

### Step 2: Update Vitals CSV Header

Find where `vitals_csv` is created in `RecorderState.__init__()` and update:

```python
self.vitals_csv = CsvWriter(
    os.path.join(args.outdir, 'vitals_log.csv'),
    ['dev_us', 'spo2', 'hr', 'age_ms', 'flags', 'brightness', 'battery_pct', 'battery_v', 'host_rx_ms']
)
```

---

### Step 3: Update Vitals Row Appending

Find where vitals rows are appended (in `consumer_task`) and update:

```python
vit = decode_vitals(payload)
st.vitals_rows.append([
    dev_us,
    vit.get('spo2'),
    vit.get('hr'),
    vit.get('age_ms'),
    vit.get('flags'),
    vit.get('brightness'),
    vit.get('battery_pct'),  # NEW
    vit.get('battery_v'),    # NEW
    host_rx
])
```

---

### Step 4: Add Battery to Health Report

Find the `health_task()` function and add battery display:

```python
async def health_task(st: RecorderState) -> None:
    # ... existing code ...
    
    while True:
        await asyncio.sleep(1.0)
        
        # ... existing stats calculation ...
        
        # NEW: Extract battery data
        last_battery_pct = None
        last_battery_v = None
        
        if st.vitals_rows:
            last_row = st.vitals_rows[-1]
            if len(last_row) >= 9:  # Check if battery fields exist
                last_battery_pct = last_row[6]  # battery_pct column
                last_battery_v = last_row[7]    # battery_v column
        
        # NEW: Format battery string
        if last_battery_pct is not None:
            batt_str = f"batt={last_battery_pct}%"
            if last_battery_v is not None:
                batt_str += f" ({last_battery_v:.2f}V)"
        else:
            batt_str = "batt=N/A"
        
        print(
            f"[Health] seg={st.seg_id} off={elapsed_str} state={state_str} "
            f"conn_sub={int(st.connected)} fps={fps:.0f} notify/s={notify_rate:.0f} "
            f"rx_kB/s={rx_kbps:.1f} q={qsz} crc={st.stats.crc_errors} "
            f"q_drop={st.stats.queue_drops} imu_sat={st.stats.imu_sat} "
            f"{batt_str}  "  # NEW
            f"last_notify={last_notify_ms}ms last_good={last_good_ms}ms "
            f"ping_age={ping_age_ms}ms ping_fail={st.stats.ping_fail} "
            f"gaps/s={gaps_per_sec:.0f} recon={st.stats.reconnects} reason={st.reason}"
        )
        
        # NEW: Low battery warning
        if last_battery_pct is not None:
            if last_battery_pct < 20:
                print(f"[WARN] Battery low: {last_battery_pct}% - Consider stopping recording soon.")
            if last_battery_pct < 10:
                print(f"[CRITICAL] Battery critical: {last_battery_pct}% - Recording may stop soon!")
```

---

## Testing Checklist

### Test 1: MAX17048 Connected (Normal Operation)
- [ ] Upload firmware
- [ ] Check Serial Monitor shows: `[Battery] Checking for MAX17048... OK.`
- [ ] Check Serial Monitor shows: `[Battery] Initial: XX.X% (X.XXV)`
- [ ] Check Serial Monitor prints battery every second
- [ ] Start Python recorder
- [ ] Check health report shows: `batt=XX% (X.XXV)`
- [ ] Check `vitals_log.csv` has battery columns

### Test 2: MAX17048 Disconnected (Graceful Degradation)
- [ ] Disconnect MAX17048 from I2C
- [ ] Upload firmware
- [ ] Check Serial Monitor shows: `[Battery] Checking for MAX17048... NOT FOUND.`
- [ ] Verify no crashes or hangs
- [ ] Verify IMU and SpO2 still work
- [ ] Start Python recorder
- [ ] Check health report shows: `batt=N/A`

### Test 3: Low Battery Warning
- [ ] Modify threshold temporarily: `if (batt.pct < 95.0f)` in loop()
- [ ] Verify LED blinks very fast (10 Hz)
- [ ] Verify Serial shows `[LOW!]`
- [ ] Verify Python shows warning

---

## Expected Output Examples

### Arduino Serial Monitor (Normal):
```
[Battery] Checking for MAX17048... OK.
[Battery] Initial: 98.1% (4.16V)
[Battery] 98.1% (4.16V) ↓-3.1%/hr
[Health] MTU=247 EMGavail=0 IMUavail=0 ...
```

### Arduino Serial Monitor (Not Connected):
```
[Battery] Checking for MAX17048... NOT FOUND. Battery monitoring disabled.
[Battery] N/A (MAX17048 not detected)
[Health] MTU=247 EMGavail=0 IMUavail=0 ...
```

### Python Health Report (Normal):
```
[Health] seg=0 off=12:34 ... batt=98% (4.16V) ...
```

### Python Health Report (Low Battery):
```
[Health] seg=0 off=12:34 ... batt=18% (3.45V) ...
[WARN] Battery low: 18% - Consider stopping recording soon.
```

---

## Installation Instructions

1. Install MAX17048 library in Arduino IDE:
   - Go to: Sketch → Include Library → Manage Libraries
   - Search: "SparkFun MAX1704x"
   - Install: "SparkFun MAX1704x Fuel Gauge Arduino Library"

2. Apply firmware changes to `myoware_imu_v2.ino`

3. Upload to ESP32

4. Apply Python changes to `btCentral.py`

5. Test with both MAX17048 connected and disconnected

---

## Notes

- Battery is read every 10 seconds (configurable via `BATTERY_READ_INTERVAL_MS`)
- Battery data is sent with Vitals frame (every 30 seconds)
- LED flashes fast when battery < 20% (overrides all other LED states)
- System works perfectly even if MAX17048 is not connected
- Backward compatible with old firmware (Python handles both formats)
