#include <Arduino.h>
#include <Wire.h>
#include <esp_adc/adc_continuous.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <MyoWare.h>
#include "ICM_20948.h"

#include "MAX30105.h"
#include "spo2_algorithm.h"

// Battery fuel gauge
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

// ==================== LED State Machine Types (MUST be above prototypes) ====================
enum LedMode : uint8_t { LED_DISCONNECTED, LED_WAIT_HOST, LED_STREAMING, LED_BACKLOG, LED_LOW_BATTERY };

// Forward declarations (avoid Arduino auto-prototype issues)
static void led_set(bool on);
static void led_update(LedMode mode);

// ==================== BLE UUIDs (Nordic UART style) ====================
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

static const char* DEVICE_NAME = "MyoW1";

// ==================== LED / Watchdog ====================
static constexpr int PIN_LED = 13;  // MyoWare Wireless Shield "User LED (13)" (verified)

// Host heartbeat policy
static constexpr uint32_t HOST_PING_EXPECT_MS      = 2000;  // host sends PING ~1s; warn if >2s
static constexpr uint32_t HOST_SILENCE_PAUSE_MS    = 5000;  // pause streaming/sampling if >5s
static constexpr uint32_t HOST_SILENCE_REBOOT_MS   = 30000; // reboot if >30s

// TX backlog policy
static constexpr uint16_t EMG_BACKLOG_PAUSE_THRESH = 1800; // indicate backlog if EMG backlog too high (approx)
static constexpr uint16_t IMU_BACKLOG_PAUSE_THRESH = 450;

// ==================== EMG Pins ====================
static constexpr int PIN_ENV = 39; // A3 (ADC1)
static constexpr int PIN_RAW = 32; // A4 (ADC1)
static constexpr int PIN_REF = 33; // A5 (ADC1)
static constexpr bool REF_CONNECTED = false;

// ==================== Rates ====================
static constexpr uint32_t EMG_RATE_HZ = 2000;
static constexpr uint32_t IMU_RATE_HZ = 200;
static constexpr uint32_t EMG_PERIOD_US = 1000000UL / EMG_RATE_HZ;
static constexpr uint32_t IMU_PERIOD_US = 1000000UL / IMU_RATE_HZ;

// ==================== Ring buffers ====================
static constexpr uint16_t EMG_BUF_SIZE = 2048;
static constexpr uint16_t IMU_BUF_SIZE = 512;

// Debug print
static constexpr uint32_t HEALTH_REPORT_MS = 1000;

// ==================== ADC Continuous (DMA) config ====================
// Placed after EMG_RATE_HZ so the constexpr multiply is valid.
// PIN_ENV=39 → ADC1 channel 3, PIN_RAW=32 → ADC1 channel 4
static constexpr adc_channel_t ADC_CH_ENV  = ADC_CHANNEL_3;
static constexpr adc_channel_t ADC_CH_RAW  = ADC_CHANNEL_4;
// Round-robin over 2 channels: total conv rate = EMG_RATE_HZ * 2
static constexpr uint32_t ADC_CONV_FREQ_HZ = EMG_RATE_HZ * 2;  // 4000 conv/s
// 256 bytes / sizeof(adc_digi_output_data_t=4) = 64 results per DMA frame
static constexpr uint32_t ADC_FRAME_SIZE   = 256;
static adc_continuous_handle_t adcHandle   = nullptr;
// Mutex protecting adcHandle across Core 0 (loop/deinit) and Core 1 (emgDrainTask/read).
// Must be a FreeRTOS mutex (not portMUX spinlock) because both sides are tasks, not ISRs.
static SemaphoreHandle_t adcMutex  = nullptr;
// Mutex protecting the shared Wire/I2C bus across imuTask (Core 1),
// spo2Task (Core 1), and loop()/readBattery() (Core 0).
// Arduino Wire is NOT thread-safe on ESP32.
static SemaphoreHandle_t wireMutex = nullptr;

// ==================== Protocol ====================
static constexpr uint8_t MAGIC0 = 0xA5;
static constexpr uint8_t MAGIC1 = 0x5A;

// PROTO_VER bumped to v2 to indicate IMU meta includes FSR fields
static constexpr uint8_t PROTO_VER = 0x02;

static constexpr uint8_t FLAG_CRC16       = 0x01;
static constexpr uint8_t FLAG_IMU_META_V2 = 0x02; // IMU meta includes FSR bytes

static constexpr uint8_t TLV_EMG_BATCH = 0x01;
static constexpr uint8_t TLV_IMU_BATCH = 0x02;
static constexpr uint8_t TLV_VITALS    = 0x03;

// ==================== IMU scaling config (LOGGED IN META) ====================
static constexpr uint8_t  IMU_ACC_FSR_G     = 2;    // one of {2,4,8,16}
static constexpr uint16_t IMU_GYRO_FSR_DPS  = 250;  // one of {250,500,1000,2000}

struct EmgSample { uint16_t env, raw; uint32_t t_us, seq; };
struct ImuSample { int16_t ax, ay, az, gx, gy, gz; uint32_t t_us, seq; };

static EmgSample emgBuf[EMG_BUF_SIZE];
static ImuSample imuBuf[IMU_BUF_SIZE];
static volatile uint16_t emgHead = 0, emgTail = 0;
static volatile uint16_t imuHead = 0, imuTail = 0;
static volatile uint32_t emgSeq = 0, imuSeq = 0;
static volatile uint32_t emgDropped = 0, imuDropped = 0;
// Counts consecutive same-channel ADC results (ENV+ENV or RAW+RAW) —
// indicates DMA arbiter skipped a conversion, causing a channel-order violation.
// The overwritten value is discarded; the counter provides diagnostic visibility.
static volatile uint32_t emgChOrderViolations = 0;

// Battery state structure
struct BatteryState {
    float pct;          // 0-100%, or NaN if unavailable
    float voltage;      // Volts, or NaN if unavailable
    float changeRate;   // %/hr, or NaN if unavailable
    bool isLow;         // Low battery alert
    bool isValid;       // True if data is valid
};

static portMUX_TYPE emgMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;

// Encode FSRs into compact codes (1 byte each)
static inline uint8_t accFsrToCode(uint8_t g) {
  switch (g) {
    case 2:  return 0;
    case 4:  return 1;
    case 8:  return 2;
    case 16: return 3;
    default: return 0;
  }
}
static inline uint8_t gyroFsrToCode(uint16_t dps) {
  switch (dps) {
    case 250:  return 0;
    case 500:  return 1;
    case 1000: return 2;
    case 2000: return 3;
    default:   return 0;
  }
}

static inline uint16_t rb_next(uint16_t idx, uint16_t size) { return (idx + 1) % size; }

static bool emg_push(const EmgSample& s) {
  bool ok = true;
  portENTER_CRITICAL(&emgMux);
  uint16_t next = rb_next(emgHead, EMG_BUF_SIZE);
  if (next == emgTail) { emgDropped++; ok = false; }
  else { emgBuf[emgHead] = s; emgBuf[emgHead].seq = emgSeq++; emgHead = next; }
  portEXIT_CRITICAL(&emgMux);
  return ok;
}

static bool imu_push(const ImuSample& s) {
  bool ok = true;
  portENTER_CRITICAL(&imuMux);
  uint16_t next = rb_next(imuHead, IMU_BUF_SIZE);
  if (next == imuTail) { imuDropped++; ok = false; }
  else { imuBuf[imuHead] = s; imuBuf[imuHead].seq = imuSeq++; imuHead = next; }
  portEXIT_CRITICAL(&imuMux);
  return ok;
}

static uint16_t emg_available() {
  portENTER_CRITICAL(&emgMux);
  uint16_t h = emgHead, t = emgTail;
  portEXIT_CRITICAL(&emgMux);
  return (h >= t) ? (h - t) : (EMG_BUF_SIZE - (t - h));
}

static uint16_t imu_available() {
  portENTER_CRITICAL(&imuMux);
  uint16_t h = imuHead, t = imuTail;
  portEXIT_CRITICAL(&imuMux);
  return (h >= t) ? (h - t) : (IMU_BUF_SIZE - (t - h));
}

static uint16_t emg_pop_batch(EmgSample* out, uint16_t maxN) {
  uint16_t n = 0;
  portENTER_CRITICAL(&emgMux);
  while (n < maxN && emgTail != emgHead) {
    out[n++] = emgBuf[emgTail];
    emgTail = rb_next(emgTail, EMG_BUF_SIZE);
  }
  portEXIT_CRITICAL(&emgMux);
  return n;
}

static uint16_t imu_pop_batch(ImuSample* out, uint16_t maxN) {
  uint16_t n = 0;
  portENTER_CRITICAL(&imuMux);
  while (n < maxN && imuTail != imuHead) {
    out[n++] = imuBuf[imuTail];
    imuTail = rb_next(imuTail, IMU_BUF_SIZE);
  }
  portEXIT_CRITICAL(&imuMux);
  return n;
}

static void rb_flush_all() {
  // Drop buffered samples (used when host is absent to prevent stale backlog)
  portENTER_CRITICAL(&emgMux);
  emgTail = emgHead;
  portEXIT_CRITICAL(&emgMux);

  portENTER_CRITICAL(&imuMux);
  imuTail = imuHead;
  portEXIT_CRITICAL(&imuMux);
}

// ==================== Sensors ====================
MyoWare myoware;
ICM_20948_I2C myICM;
static constexpr uint8_t AD0_VAL = 1;

// MAX3010x
MAX30105 maxSensor;
static constexpr uint8_t MAX3010X_ADDR = 0x57;

// MAX17048 Battery Fuel Gauge
SFE_MAX1704X lipo(MAX1704X_MAX17048);
static bool batteryAvailable = false;
static float lastBatteryPct = 100.0f;
static float lastBatteryVoltage = 4.2f;
static uint32_t lastBatteryReadMs = 0;
static constexpr uint32_t BATTERY_READ_INTERVAL_MS = 10000;  // Read every 10 seconds
static bool batteryInitialReadDone = false;  // Track if we've done the first read

// SpO2 algorithm buffers
static constexpr int SPO2_BUFFER_SIZE = 100; // 4 sec @ 25 Hz
static uint32_t irBuffer[SPO2_BUFFER_SIZE];
static uint32_t redBuffer[SPO2_BUFFER_SIZE];

// Brightness control
static byte ledBrightness = 0x1F;
static constexpr byte BRIGHTNESS_MIN = 0x0F;
static constexpr byte BRIGHTNESS_MAX = 0x7F;
static constexpr uint32_t SATURATION_HIGH = 200000;
static constexpr uint32_t SATURATION_LOW  = 10000;



BatteryState readBattery() {
    BatteryState state;
    state.pct = NAN;
    state.voltage = NAN;
    state.changeRate = NAN;
    state.isLow = false;
    state.isValid = false;

    if (!batteryAvailable) return state;

    // Guard I2C bus — shared with imuTask and spo2Task
    if (wireMutex) xSemaphoreTake(wireMutex, portMAX_DELAY);
    state.pct        = lipo.getSOC();
    state.voltage    = lipo.getVoltage();
    state.changeRate = lipo.getChangeRate();
    state.isLow      = lipo.isLow();
    if (wireMutex) xSemaphoreGive(wireMutex);
    
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

// Print battery status using cached values — no I2C.
// Cache is refreshed every BATTERY_READ_INTERVAL_MS (10 s) by send_vitals_frame().
// Calling readBattery() here would do 4 live I2C reads every second (~400 us/call).
void printBatteryStatus() {
    if (!batteryAvailable) {
        Serial.println("[Battery] N/A (MAX17048 not detected)");
        return;
    }
    if (!batteryInitialReadDone) {
        Serial.println("[Battery] pending first read");
        return;
    }

    uint32_t ageMs = millis() - lastBatteryReadMs;

    Serial.print("[Battery] ");
    Serial.print(lastBatteryPct, 1);
    Serial.print("% (");
    Serial.print(lastBatteryVoltage, 2);
    Serial.print("V) age=");
    Serial.print(ageMs / 1000);
    Serial.print("s");

    if (lastBatteryPct < 20.0f) {
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
  uint8_t  battery_pct;    // Battery percentage (0-100 or 255 if unknown)
  uint16_t battery_mv;     // Battery voltage in millivolts (0 if unknown)
};
static VitalsState vitals = {0xFFFF, 0xFFFF, 0, 0, ledBrightness, 255, 0};
static portMUX_TYPE vitalsMux = portMUX_INITIALIZER_UNLOCKED;
static bool max3010xPresent = false;

// ==================== BLE ====================
BLEServer* pServer = nullptr;
BLECharacteristic* pTXCharacteristic = nullptr;
BLECharacteristic* pRXCharacteristic = nullptr;
volatile bool isConnected = false;
bool previousConnected = false;
uint16_t negotiatedMTU = 23;
uint16_t maxNotifyPayload = 20;

// Host heartbeat tracking (application-level watchdog)
static volatile uint32_t lastHostPingMs = 0;
static volatile uint32_t lastHostAckSeq = 0;
static portMUX_TYPE hostMux = portMUX_INITIALIZER_UNLOCKED;

// ==================== LED State vars ====================
static LedMode ledMode = LED_DISCONNECTED;
static uint32_t ledLastMs = 0;
static uint8_t ledPhase = 0;

static inline uint32_t getLastHostPingMs() {
  portENTER_CRITICAL(&hostMux);
  uint32_t v = lastHostPingMs;
  portEXIT_CRITICAL(&hostMux);
  return v;
}

static inline void setLastHostPingNow() {
  portENTER_CRITICAL(&hostMux);
  lastHostPingMs = millis();
  portEXIT_CRITICAL(&hostMux);
}

static inline void clearHostPing() {
  portENTER_CRITICAL(&hostMux);
  lastHostPingMs = 0;
  portEXIT_CRITICAL(&hostMux);
}

static inline void setLastHostAck(uint32_t ack) {
  portENTER_CRITICAL(&hostMux);
  lastHostAckSeq = ack;
  lastHostPingMs = millis(); // treat ACK as heartbeat too
  portEXIT_CRITICAL(&hostMux);
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    // ESP32 core 3.x BLE returns Arduino String here in your build
    String v = c->getValue();

    if (v.length() >= 4 && v.startsWith("PING")) {
      setLastHostPingNow();
      return;
    }
    if (v.startsWith("ACK:")) {
      uint32_t ack = (uint32_t)strtoul(v.c_str() + 4, nullptr, 10);
      setLastHostAck(ack);
      return;
    }
    // Future: parse other control commands (e.g., SETRATE, LED modes, etc.)
  }
};

// ==================== Timers ====================
// emgTimer removed — EMG sampling is driven by ADC DMA, not a software timer.
static esp_timer_handle_t imuTimer = nullptr;

// Use counter instead of bool to avoid losing ticks
static volatile uint32_t imu_ticks = 0;

// IMPORTANT: start paused so we don't sample/drop when no host is consuming
static volatile bool samplingPaused = true;

// Atomic accessors for samplingPaused — written on Core 0, read on Core 1.
// volatile alone does not guarantee cross-core visibility on Xtensa LX6
// (no hardware memory barrier). __atomic builtins insert the required barriers.
static inline void setSamplingPaused(bool v) {
  __atomic_store_n(&samplingPaused, v, __ATOMIC_RELEASE);
}
static inline bool getSamplingPaused() {
  return __atomic_load_n(&samplingPaused, __ATOMIC_ACQUIRE);
}

// ==================== ADC Continuous (DMA) helpers ====================

static void emg_adc_init() {
  if (!adcMutex) return;  // safety: mutex must exist before init
  xSemaphoreTake(adcMutex, portMAX_DELAY);
  if (adcHandle != nullptr) { xSemaphoreGive(adcMutex); return; } // already running

    // Zero-init first to avoid nested designated-initializer warnings in C++
  adc_continuous_handle_cfg_t handleCfg = {};
  handleCfg.max_store_buf_size = ADC_FRAME_SIZE * 4;  // internal ring buffer
  handleCfg.conv_frame_size    = ADC_FRAME_SIZE;
  // handleCfg.flags.flush_pool = 0 is already 0 from zero-init above
  ESP_ERROR_CHECK(adc_continuous_new_handle(&handleCfg, &adcHandle));

  // Two channels: ENV and RAW, both on ADC1, 12-bit, 11dB attenuation
  adc_digi_pattern_config_t patterns[2];
  patterns[0].atten     = ADC_ATTEN_DB_12;  // ~0-3.3V range (replaces ADC_11db)
  patterns[0].channel   = ADC_CH_ENV;
  patterns[0].unit      = ADC_UNIT_1;
  patterns[0].bit_width = ADC_BITWIDTH_12;

  patterns[1].atten     = ADC_ATTEN_DB_12;
  patterns[1].channel   = ADC_CH_RAW;
  patterns[1].unit      = ADC_UNIT_1;
  patterns[1].bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t adcCfg = {
    .pattern_num    = 2,
    .adc_pattern    = patterns,
    .sample_freq_hz = ADC_CONV_FREQ_HZ,
    .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
    // TYPE1 is the only format supported on original ESP32 (TYPE2 is for S2/S3 only).
    // TYPE1 layout: bits[11:0]=12-bit data, bits[15:12]=channel index
    .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
  };
    ESP_ERROR_CHECK(adc_continuous_config(adcHandle, &adcCfg));
  ESP_ERROR_CHECK(adc_continuous_start(adcHandle));
  xSemaphoreGive(adcMutex);
  Serial.println("[ADC] Continuous DMA started.");
}

static void emg_adc_deinit() {
  if (!adcMutex) return;
  xSemaphoreTake(adcMutex, portMAX_DELAY);
  if (adcHandle == nullptr) { xSemaphoreGive(adcMutex); return; }
  adc_continuous_stop(adcHandle);
  adc_continuous_deinit(adcHandle);
  adcHandle = nullptr;
  xSemaphoreGive(adcMutex);
  Serial.println("[ADC] Continuous DMA stopped.");
}

// ==================== IMU timer ====================
static void imu_timer_cb(void* /*arg*/) {
  if (getSamplingPaused()) return;
  imu_ticks++;
}

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* /*pServer*/) override {
    isConnected = true;
    negotiatedMTU = BLEDevice::getMTU();
    maxNotifyPayload = (negotiatedMTU > 3) ? (negotiatedMTU - 3) : 20;
    Serial.printf("[BLE] Connected. MTU=%u, maxNotifyPayload=%u\n", negotiatedMTU, maxNotifyPayload);
    if (negotiatedMTU < 100) {
      Serial.println("[WARN] MTU too low for 2kHz EMG; expect drops.");
    }

    // Require FIRST host PING before sampling/streaming.
    clearHostPing();

        // Reset per-session stats under their respective spinlocks —
    // emgDrainTask (Core 1) increments emgSeq/emgDropped inside emgMux,
    // imuTask (Core 1) increments imuSeq/imuDropped inside imuMux.
        portENTER_CRITICAL(&emgMux);
    emgDropped          = 0;
    emgSeq              = 0;
    emgChOrderViolations = 0;
    portEXIT_CRITICAL(&emgMux);

    portENTER_CRITICAL(&imuMux);
    imuDropped = 0;
    imuSeq     = 0;
    portEXIT_CRITICAL(&imuMux);

    // Reset frame sequence used in BLE packets
    // (declared below; forward-declared here via extern)
    // We'll reset it in loop once connected and before first send.

        rb_flush_all();
    imu_ticks = 0;
    setSamplingPaused(true);  // atomic write — emgDrainTask on Core 1 reads this
    // ADC DMA will be started by loop() when host heartbeat arrives
  }

  void onDisconnect(BLEServer* /*pServer*/) override {
    isConnected = false;
    // Immediately pause sampling to avoid filling buffers while disconnected
        setSamplingPaused(true);
    emg_adc_deinit();  // stop DMA while disconnected to save power
    clearHostPing();
    rb_flush_all();
    imu_ticks = 0;
  }
};

// ==================== EMG Drain Task (ADC continuous DMA) ====================
// Runs on Core 1 alongside imuTask. Drains the DMA buffer, demultiplexes
// ENV and RAW channels, pairs them into EmgSample, and pushes to ring buffer.
void emgDrainTask(void* /*param*/) {
  // Scratch buffer sized to one DMA frame
  static uint8_t dmaBuf[ADC_FRAME_SIZE];

  // Partial sample accumulator: hold one channel until the other arrives
  bool     envValid  = false;
  bool     rawValid  = false;
  uint16_t envVal    = 0;
  uint16_t rawVal    = 0;

    for (;;) {
        // Guard FIRST: if paused, don't touch the driver
        if (getSamplingPaused()) {
      envValid = false;
      rawValid = false;
      vTaskDelay(5 / portTICK_PERIOD_MS);
      continue;
    }

    // Take mutex to safely use adcHandle — prevents race with emg_adc_deinit() on Core 0
    if (xSemaphoreTake(adcMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      // Couldn't get mutex in time — deinit may be in progress, skip this cycle
      vTaskDelay(1);
      continue;
    }

    if (adcHandle == nullptr) {
      // Handle was freed between our samplingPaused check and mutex acquisition
      xSemaphoreGive(adcMutex);
      envValid = false;
      rawValid = false;
      vTaskDelay(5 / portTICK_PERIOD_MS);
      continue;
    }

    // Block until ADC DMA has data (10 ms timeout keeps task responsive)
    uint32_t bytesRead = 0;
    esp_err_t ret = adc_continuous_read(adcHandle, dmaBuf, ADC_FRAME_SIZE, &bytesRead, 10);
    xSemaphoreGive(adcMutex);  // release immediately after read

    if (ret != ESP_OK || bytesRead == 0) {
      vTaskDelay(1);
      continue;
    }

            // Capture frame-end timestamp BEFORE processing results.
    // Back-calculate each sample's actual ADC capture time:
    //   sample[i] was captured (numResults-1-i) * (1e6/ADC_CONV_FREQ_HZ) µs before frame end.
    // This corrects for the up-to-16 ms latency between ADC capture and emgDrainTask wakeup.
    uint32_t frameEndUs = micros();
    // Inter-conversion period in µs (250 µs at 4000 conv/s)
    static constexpr float US_PER_CONV = 1000000.0f / ADC_CONV_FREQ_HZ;

    uint32_t numResults = bytesRead / sizeof(adc_digi_output_data_t);
    adc_digi_output_data_t* results = (adc_digi_output_data_t*)dmaBuf;

    for (uint32_t i = 0; i < numResults; i++) {
      adc_digi_output_data_t* r = &results[i];
            // TYPE1 layout on original ESP32 (IDF 5.x): bits[11:0]=data, bits[15:12]=channel
      // Using .val directly avoids the removed .type1/.type2 union members in IDF 5.x
      uint8_t  ch  = (uint8_t)((r->val >> 12) & 0x0F);
      uint16_t val = (uint16_t)(r->val & 0x0FFF);

            if (ch == ADC_CH_ENV) {
        if (envValid) {
          // ENV arrived while previous ENV not yet paired — channel-order violation.
          // Discard the stale ENV value; use the newer one. Count for diagnostics.
          portENTER_CRITICAL(&emgMux);
          emgChOrderViolations++;
          portEXIT_CRITICAL(&emgMux);
        }
        envVal   = val;
        envValid = true;
      } else if (ch == ADC_CH_RAW) {
        if (rawValid) {
          // RAW arrived while previous RAW not yet paired — channel-order violation.
          portENTER_CRITICAL(&emgMux);
          emgChOrderViolations++;
          portEXIT_CRITICAL(&emgMux);
        }
        rawVal   = val;
        rawValid = true;
      }
      // else: unexpected channel — ignore

            // Emit a paired sample once both channels have been seen
            if (envValid && rawValid) {
        EmgSample s;
        uint32_t age_us = (uint32_t)((numResults - 1u - i) * US_PER_CONV);
        s.t_us = frameEndUs - age_us;
        // seq assigned inside emg_push() under emgMux — see emg_push_next_seq()
        s.seq  = 0;  // placeholder; overwritten atomically in emg_push
        s.env  = envVal;
        s.raw  = rawVal;
        emg_push(s);
        envValid = false;
        rawValid = false;
      }
    }
  }
}

// ==================== IMU Task ====================
void imuTask(void* /*param*/) {
  for (;;) {
    if (imu_ticks > 0) {
      imu_ticks--;
      xSemaphoreTake(wireMutex, portMAX_DELAY);
      bool ready = myICM.dataReady();
      if (ready) myICM.getAGMT();
      xSemaphoreGive(wireMutex);
            if (ready) {
        ImuSample s;
        s.t_us = micros();
        s.seq  = 0;  // placeholder; overwritten atomically in imu_push
        s.ax   = myICM.agmt.acc.axes.x;
        s.ay   = myICM.agmt.acc.axes.y;
        s.az   = myICM.agmt.acc.axes.z;
        s.gx   = myICM.agmt.gyr.axes.x;
        s.gy   = myICM.agmt.gyr.axes.y;
        s.gz   = myICM.agmt.gyr.axes.z;
        imu_push(s);
      }
    }
    vTaskDelay(1);
  }
}

// ==================== MAX3010x Helpers ====================
static void configureMax3010x() {
  byte sampleAverage = 4;
  byte ledMode       = 2;    // Red + IR
  int  sampleRate    = 25;   // Hz
  int  pulseWidth    = 411;  // wide
  int  adcRange      = 4096;

  maxSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void spo2Task(void* /*param*/) {
  float hrAvg = 0;

  for (;;) {
    if (!max3010xPresent) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

        // Collect 100 samples (~4 sec at 25 Hz)
    for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
      bool avail = false;
      while (!avail) {
        xSemaphoreTake(wireMutex, portMAX_DELAY);
        maxSensor.check();
        avail = maxSensor.available();
        xSemaphoreGive(wireMutex);
        if (!avail) vTaskDelay(1);
      }
      xSemaphoreTake(wireMutex, portMAX_DELAY);
      redBuffer[i] = maxSensor.getRed();
      irBuffer[i]  = maxSensor.getIR();
      maxSensor.nextSample();
      xSemaphoreGive(wireMutex);
    }

    uint32_t lastIR  = irBuffer[SPO2_BUFFER_SIZE - 1];
    uint32_t lastRED = redBuffer[SPO2_BUFFER_SIZE - 1];

    // Auto brightness adjust (avoid max<int>/min<int> templates)
        if (lastIR > SATURATION_HIGH && ledBrightness > BRIGHTNESS_MIN) {
      int newB = max((int)BRIGHTNESS_MIN, (int)ledBrightness - 0x0F);
      ledBrightness = (byte)newB;
      xSemaphoreTake(wireMutex, portMAX_DELAY);
      configureMax3010x();
      xSemaphoreGive(wireMutex);
    } else if (lastIR < SATURATION_LOW && ledBrightness < BRIGHTNESS_MAX) {
      int newB = min((int)BRIGHTNESS_MAX, (int)ledBrightness + 0x0F);
      ledBrightness = (byte)newB;
      xSemaphoreTake(wireMutex, portMAX_DELAY);
      configureMax3010x();
      xSemaphoreGive(wireMutex);
    }

    // Compute SpO2 and HR
    int32_t spo2;
    int8_t spo2Valid;
    int32_t heartRate;
    int8_t hrValid;

    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, SPO2_BUFFER_SIZE,
      redBuffer,
      &spo2, &spo2Valid,
      &heartRate, &hrValid
    );

    uint16_t spo2_u16 = 0xFFFF;
    uint16_t hr_x10   = 0xFFFF;
    uint8_t flags     = 0;

    if (spo2Valid) {
      spo2 = constrain(spo2, 0, 100);
      spo2_u16 = (uint16_t)spo2;
      flags |= 0x01;
    }
    if (hrValid && heartRate > 0 && heartRate < 250) {
      hrAvg = (hrAvg == 0) ? (float)heartRate : (0.8f * hrAvg + 0.2f * (float)heartRate);
      hr_x10 = (uint16_t)constrain((int)(hrAvg * 10.0f + 0.5f), 0, 5000);
      flags |= 0x02;
    }

    // Publish vitals atomically
    portENTER_CRITICAL(&vitalsMux);
    vitals.spo2_u16 = spo2_u16;
    vitals.hr_x10_u16 = hr_x10;
    vitals.flags = flags;
    vitals.computed_ms = millis();
    vitals.brightness = ledBrightness;
    portEXIT_CRITICAL(&vitalsMux);

    // Optional debug
    String sSpo2 = (flags & 0x01) ? String(spo2_u16) : "NA";
    String sHr   = (flags & 0x02) ? String(hr_x10 / 10.0f, 1) : "NA";
    Serial.printf("[Vitals] SpO2=%s HR=%s Bright=0x%02X IR=%lu RED=%lu\n",
                  sSpo2.c_str(), sHr.c_str(),
                  ledBrightness, (unsigned long)lastIR, (unsigned long)lastRED);

    vTaskDelay(30000 / portTICK_PERIOD_MS);
  }
}

// ==================== Packet Writer Helpers ====================
static uint32_t frameSeq = 0;

static inline void wr_u16(uint8_t*& p, uint16_t v) { *p++ = v & 0xFF; *p++ = v >> 8; }
static inline void wr_i16(uint8_t*& p, int16_t v)  { wr_u16(p, (uint16_t)v); }
static inline void wr_u32(uint8_t*& p, uint32_t v) {
  *p++ = v & 0xFF;
  *p++ = (v >> 8) & 0xFF;
  *p++ = (v >> 16) & 0xFF;
  *p++ = (v >> 24) & 0xFF;
}

static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// ==================== Send Frames ====================
bool send_emg_frame(uint16_t maxPayload) {
  const uint16_t headerBytes = 14;
  const uint16_t tlvHdrBytes = 3;
  const uint16_t emgMetaBytes = 13;
  const uint16_t crcBytes = 2;

    if (maxPayload < (headerBytes + tlvHdrBytes + emgMetaBytes + crcBytes + 4)) return false;

  uint16_t maxSamples = (maxPayload - headerBytes - tlvHdrBytes - emgMetaBytes - crcBytes) / 4;
  if (maxSamples == 0) return false;

  uint16_t have = emg_available();
  if (have < 5) return false;

  uint16_t N = (have < maxSamples) ? have : maxSamples;
  static EmgSample batch[256];
  if (N > 256) N = 256;
  N = emg_pop_batch(batch, N);
  if (N == 0) return false;

  uint8_t buf[512];
  uint8_t* p = buf;

  uint32_t now_us = micros();

  // Header
  p[0] = MAGIC0; p[1] = MAGIC1; p[2] = PROTO_VER; p[3] = FLAG_CRC16;
  p += 4;
  wr_u32(p, frameSeq++);
  wr_u32(p, now_us);
  wr_u16(p, 0); // payload_len placeholder

  uint8_t* payload_start = buf + headerBytes;
  p = payload_start;

  // TLV header
  *p++ = TLV_EMG_BATCH;
  wr_u16(p, 0);
  uint8_t* tlvLenPos = p - 2;

  // EMG meta
  wr_u32(p, batch[0].seq);
  int32_t off = (int32_t)batch[0].t_us - (int32_t)now_us;
  if (off > 32767) off = 32767; if (off < -32768) off = -32768;
  wr_i16(p, (int16_t)off);
  wr_u16(p, EMG_RATE_HZ);
    *p++ = 2; // channels
  *p++ = 0x03;
  *p++ = 0;
  wr_u16(p, N);

    for (uint16_t i = 0; i < N; i++) {
    wr_u16(p, batch[i].env);
    wr_u16(p, batch[i].raw);
  }

  uint16_t tlvLen = emgMetaBytes + 4 * N;
  tlvLenPos[0] = tlvLen & 0xFF;
  tlvLenPos[1] = tlvLen >> 8;

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

bool send_imu_frame(uint16_t maxPayload) {
  const uint16_t headerBytes = 14;
  const uint16_t tlvHdrBytes = 3;
  const uint16_t imuMetaBytes = 14; // v2
  const uint16_t crcBytes = 2;
  const uint16_t bytesPerSample = 12;

  if (maxPayload < (headerBytes + tlvHdrBytes + imuMetaBytes + crcBytes + bytesPerSample)) return false;

  uint16_t maxSamples = (maxPayload - headerBytes - tlvHdrBytes - imuMetaBytes - crcBytes) / bytesPerSample;
  if (maxSamples == 0) return false;

  uint16_t have = imu_available();
  if (have < 1) return false;

  uint16_t N = (have < maxSamples) ? have : maxSamples;
  if (N > 12) N = 12;

  static ImuSample batch[64];
  N = imu_pop_batch(batch, N);
  if (N == 0) return false;

  uint8_t buf[512];
  uint8_t* p = buf;

  uint32_t now_us = micros();

  // Header: include FLAG_IMU_META_V2
  p[0] = MAGIC0; p[1] = MAGIC1; p[2] = PROTO_VER; p[3] = (uint8_t)(FLAG_CRC16 | FLAG_IMU_META_V2);
  p += 4;
  wr_u32(p, frameSeq++);
  wr_u32(p, now_us);
  wr_u16(p, 0);

  uint8_t* payload_start = buf + headerBytes;
  p = payload_start;

  *p++ = TLV_IMU_BATCH;
  wr_u16(p, 0);
  uint8_t* tlvLenPos = p - 2;

  wr_u32(p, batch[0].seq);
  int32_t off = (int32_t)batch[0].t_us - (int32_t)now_us;
  if (off > 32767) off = 32767; if (off < -32768) off = -32768;
  wr_i16(p, (int16_t)off);
  wr_u16(p, IMU_RATE_HZ);
  *p++ = (uint8_t)N;
  *p++ = 0;
  wr_u16(p, 0x003F);

  *p++ = accFsrToCode(IMU_ACC_FSR_G);
  *p++ = gyroFsrToCode(IMU_GYRO_FSR_DPS);

  for (uint16_t i = 0; i < N; i++) {
    wr_i16(p, batch[i].ax);
    wr_i16(p, batch[i].ay);
    wr_i16(p, batch[i].az);
    wr_i16(p, batch[i].gx);
    wr_i16(p, batch[i].gy);
    wr_i16(p, batch[i].gz);
  }

  uint16_t tlvLen = imuMetaBytes + bytesPerSample * N;
  tlvLenPos[0] = tlvLen & 0xFF;
  tlvLenPos[1] = tlvLen >> 8;

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

// Vitals frame (SpO2/HR) once per 30 seconds
bool send_vitals_frame(uint16_t maxPayload) {
  const uint16_t headerBytes = 14;
  const uint16_t tlvHdrBytes = 3;
  const uint16_t vitalsBytes = 11;  // spo2(2) hr_x10(2) age_ms(2) flags(1) bright(1) batt_pct(1) batt_mv(2)
  const uint16_t crcBytes = 2;

  if (maxPayload < (headerBytes + tlvHdrBytes + vitalsBytes + crcBytes)) return false;

    VitalsState v;
  portENTER_CRITICAL(&vitalsMux);
  v = vitals;
  portEXIT_CRITICAL(&vitalsMux);

      // Read battery periodically (or immediately on first call)
  uint32_t nowMs = millis();
  if (!batteryInitialReadDone || (nowMs - lastBatteryReadMs) >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryReadMs = nowMs;
    batteryInitialReadDone = true;
    
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
  *p++ = v.battery_pct;    // Battery percentage
  wr_u16(p, v.battery_mv); // Battery voltage (mV)

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

// ==================== LED State Machine ====================
// Patterns:
// - DISCONNECTED: slow blink 1 Hz
// - CONNECTED waiting for host: double-blink every 2s
// - STREAMING OK: solid ON
// - BACKLOG HIGH: fast blink

static void led_set(bool on) {
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

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

    case LED_LOW_BATTERY: {
      if (now - ledLastMs >= 100) {  // Very fast blink (10 Hz)
        ledLastMs = now;
        led_set(!digitalRead(PIN_LED));
      }
    } break;
  }
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("MyoWare + IMU + SpO2 BLE starting... (host-gated sampling + watchdog)");

  pinMode(PIN_LED, OUTPUT);
  led_set(false);

    // ADC pins are now managed by the continuous DMA driver (emg_adc_init).
  // analogRead / analogSetPinAttenuation are NOT used for EMG channels.
  // PIN_REF is still available for manual reads if REF_CONNECTED is set.
  if (REF_CONNECTED) {
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_REF, ADC_11db);
  }

  Wire.begin();
  Wire.setClock(400000);

  // ---- IMU init ----
  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok) initialized = true;
    else { Serial.println("IMU init retry..."); delay(500); }
  }

  // Explicit full-scale ranges (matches logged FSR)
  {
    ICM_20948_fss_t fss;
    fss.a = (IMU_ACC_FSR_G == 2)  ? gpm2  :
            (IMU_ACC_FSR_G == 4)  ? gpm4  :
            (IMU_ACC_FSR_G == 8)  ? gpm8  :
                                   gpm16;
    fss.g = (IMU_GYRO_FSR_DPS == 250)  ? dps250  :
            (IMU_GYRO_FSR_DPS == 500)  ? dps500  :
            (IMU_GYRO_FSR_DPS == 1000) ? dps1000 :
                                         dps2000;

    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
    Serial.printf("[IMU] Set FSR: acc=+/- %ug, gyro=+/- %udps\n", IMU_ACC_FSR_G, IMU_GYRO_FSR_DPS);
  }

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

    // ---- MAX17048 Battery init (non-fatal if missing) ----
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
    lipo.setVALRTMax((float)4.1);  // High voltage alert (4.1V)
    lipo.setVALRTMin((float)3.3);  // Low voltage alert (3.3V)
    
    // Read initial state and update vitals
    float pct = lipo.getSOC();
    float volt = lipo.getVoltage();
    Serial.print("[Battery] Initial: ");
    Serial.print(pct, 1);
    Serial.print("% (");
    Serial.print(volt, 2);
    Serial.println("V)");
    
    lastBatteryPct = pct;
    lastBatteryVoltage = volt;
    
    // Initialize vitals struct with battery data
    portENTER_CRITICAL(&vitalsMux);
    vitals.battery_pct = (uint8_t)constrain((int)pct, 0, 100);
    vitals.battery_mv = (uint16_t)(volt * 1000.0f);
    portEXIT_CRITICAL(&vitalsMux);
  }

  // ---- BLE init ----
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(247);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(NUS_SERVICE_UUID);

  pTXCharacteristic = pService->createCharacteristic(
    NUS_TX_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTXCharacteristic->addDescriptor(new BLE2902());

  pRXCharacteristic = pService->createCharacteristic(
    NUS_RX_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRXCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
  BLEDevice::startAdvertising();

    // Create mutexes before any task or init that uses the protected resources
  adcMutex  = xSemaphoreCreateMutex();
  wireMutex = xSemaphoreCreateMutex();
  configASSERT(adcMutex);   // halt if heap allocation failed
  configASSERT(wireMutex);

  // ---- Timers ----
  // EMG is now driven by ADC DMA — no EMG timer needed.
  esp_timer_create_args_t imuArgs = { .callback = &imu_timer_cb, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK, .name = "imu_timer" };
  esp_timer_create(&imuArgs, &imuTimer);
  esp_timer_start_periodic(imuTimer, IMU_PERIOD_US);

  // ---- Tasks ----
    // Task priorities on Core 1:
  //   2 = emgDrainTask  — real-time, must drain DMA at 2kHz
  //   2 = imuTask       — real-time, 200 Hz timer-driven; raised from 1 to prevent SpO2 starvation
  //   0 = spo2Task      — background, computes every ~30s; lowered from 1, no real-time requirement
  xTaskCreatePinnedToCore(emgDrainTask, "emgDrain", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(imuTask,      "imuTask",  4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(spo2Task,     "spo2Task", 6144, NULL, 0, NULL, 1);

  Serial.printf("ADC DMA: %lu Hz/ch x2 ch. IMU timer: %lu Hz\n", (unsigned long)EMG_RATE_HZ, (unsigned long)IMU_RATE_HZ);

  // Start paused until first host heartbeat after a BLE connection
    samplingPaused = true;  // direct assign OK — tasks not started yet
  rb_flush_all();
}

// ==================== Loop ====================
void loop() {
  static uint32_t lastDebugMs = 0;
  static uint32_t framesSent = 0;
  static uint32_t lastFramesSent = 0;
  static uint32_t lastVitalsSendMs = 0;
  static bool frameSeqResetDone = false;

  uint32_t nowMs = millis();

  // Reset frameSeq once per connection (after connect, before first send)
  if (!isConnected) {
    frameSeqResetDone = false;
  } else if (isConnected && !frameSeqResetDone) {
    frameSeq = 0;
    frameSeqResetDone = true;
  }

  // ---- Host heartbeat / sampling gate ----
  uint32_t hp = getLastHostPingMs();
  uint32_t hostAge = (hp == 0) ? 0xFFFFFFFF : (nowMs >= hp ? (nowMs - hp) : 0);

  bool hostAlive = (hp != 0) && (hostAge <= HOST_SILENCE_PAUSE_MS);

    // IMPORTANT: never sample when not connected
  if (!isConnected) {
    if (!getSamplingPaused()) {
      setSamplingPaused(true);
      emg_adc_deinit();  // stop DMA on disconnect
      rb_flush_all();
      imu_ticks = 0;
      Serial.println("[WD] Disconnected. Pausing sampling.");
    }
  } else {
    // Connected: pause if host not alive; resume only when hostAlive
    if (!hostAlive) {
      if (!getSamplingPaused()) {
        setSamplingPaused(true);
        emg_adc_deinit();  // stop DMA while host is absent
        rb_flush_all();
        imu_ticks = 0;
        Serial.printf("[WD] Host heartbeat missing (%lu ms). Pausing sampling/streaming.\n", (unsigned long)hostAge);
      }
    } else {
            if (getSamplingPaused()) {
                setSamplingPaused(false);
        rb_flush_all();
        imu_ticks = 0;
        emg_adc_init();  // start DMA ADC now that host is alive
        Serial.println("[WD] Host heartbeat present. Resuming sampling/streaming.");
      }
    }

    // If host silent too long, reboot to recover stuck BLE states
    if (hp != 0 && hostAge >= HOST_SILENCE_REBOOT_MS) {
      Serial.printf("[WD] Host silent %lu ms. Rebooting device for recovery...\n", (unsigned long)hostAge);
      delay(50);
      ESP.restart();
    }
  }

    // ---- Decide LED mode ----
  LedMode mode = LED_DISCONNECTED;
  
    // Priority 1: Check for low battery first.
  // Use cached lastBatteryPct — updated every BATTERY_READ_INTERVAL_MS by send_vitals_frame().
  // Do NOT call readBattery() here: that does I2C at ~1kHz loop rate, wasting ~100µs/call.
  if (batteryAvailable && batteryInitialReadDone && lastBatteryPct < 20.0f) {
    mode = LED_LOW_BATTERY;
  }
  
  // Priority 2: Other states (only if not low battery)
  if (mode != LED_LOW_BATTERY) {
    if (!isConnected) {
      mode = LED_DISCONNECTED;
    } else if (getSamplingPaused()) {
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

  // ---- TX loop ----
  if (isConnected && pTXCharacteristic && !getSamplingPaused()) {
    bool sent = false;

    sent |= send_emg_frame(maxNotifyPayload);

    static uint32_t lastImuSendMs = 0;
    if (nowMs - lastImuSendMs >= 20 && emg_available() < EMG_BUF_SIZE * 3 / 4) {
      lastImuSendMs = nowMs;
      sent |= send_imu_frame(maxNotifyPayload);
    }

                if (nowMs - lastVitalsSendMs >= 30000) {  // Send Vitals every 30 seconds
        lastVitalsSendMs = nowMs;
        sent |= send_vitals_frame(maxNotifyPayload);
      }

    if (sent) framesSent++;
  }

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

    // Print battery status
    printBatteryStatus();

        Serial.printf("[Health] MTU=%u EMGavail=%u IMUavail=%u EMGdrop=%lu IMUdrop=%lu EMGchViol=%lu Frames/s=%lu | HostAge=%lu ms Paused=%u | SpO2=%s HR=%s\n",
                  negotiatedMTU, emg_available(), imu_available(),
                  (unsigned long)emgDropped, (unsigned long)imuDropped,
                  (unsigned long)emgChOrderViolations, (unsigned long)fps,
                  (unsigned long)hostAge, getSamplingPaused() ? 1 : 0,
                  sSpo2.c_str(), sHr.c_str());
  }

  // ---- Handle disconnect/reconnect ----
  if (!isConnected && previousConnected) {
    Serial.println("Disconnected. Restarting advertising...");
    delay(200);
    pServer->startAdvertising();
    previousConnected = isConnected;
  } else if (isConnected && !previousConnected) {
    previousConnected = isConnected;
  }

  delay(1);
}
