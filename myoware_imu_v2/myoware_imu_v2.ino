#include <Arduino.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <MyoWare.h>
#include "ICM_20948.h"

#include "MAX30105.h"
#include "spo2_algorithm.h"

// ==================== LED State Machine Types (MUST be above prototypes) ====================
enum LedMode : uint8_t { LED_DISCONNECTED, LED_WAIT_HOST, LED_STREAMING, LED_BACKLOG };

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

// ADC attenuation (IMPORTANT for avoiding saturation & improving stability)
// Adjust based on your actual MyoWare output swing.
static constexpr adc_attenuation_t ADC_ATTEN = ADC_11db;

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
  else { emgBuf[emgHead] = s; emgHead = next; }
  portEXIT_CRITICAL(&emgMux);
  return ok;
}

static bool imu_push(const ImuSample& s) {
  bool ok = true;
  portENTER_CRITICAL(&imuMux);
  uint16_t next = rb_next(imuHead, IMU_BUF_SIZE);
  if (next == imuTail) { imuDropped++; ok = false; }
  else { imuBuf[imuHead] = s; imuHead = next; }
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

// Shared vitals state (written by task, read by BLE loop)
struct VitalsState {
  uint16_t spo2_u16;       // 0..100 or 0xFFFF invalid
  uint16_t hr_x10_u16;     // bpm*10 or 0xFFFF invalid
  uint32_t computed_ms;    // millis() when computed
  uint8_t  flags;          // bit0 spo2Valid, bit1 hrValid
  byte     brightness;
};
static VitalsState vitals = {0xFFFF, 0xFFFF, 0, 0, ledBrightness};
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
static esp_timer_handle_t emgTimer = nullptr;
static esp_timer_handle_t imuTimer = nullptr;

// Use counter instead of bool to avoid losing ticks
static volatile uint32_t imu_ticks = 0;

// IMPORTANT: start paused so we don't sample/drop when no host is consuming
static volatile bool samplingPaused = true;

// ==================== ADC/EMG sampling ====================
// NOTE: For maximum stability at 2kHz, moving analogRead out of esp_timer callback is best.
// This version keeps your structure but gates sampling based on host heartbeat.

static void emg_timer_cb(void* /*arg*/) {
  if (samplingPaused) return;
  EmgSample s;
  s.t_us = micros();
  s.seq = emgSeq++;
  s.env = analogRead(PIN_ENV);
  s.raw = analogRead(PIN_RAW);
  emg_push(s);
}

static void imu_timer_cb(void* /*arg*/) {
  if (samplingPaused) return;
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

    // Reset per-session stats so drop counters are meaningful.
    emgDropped = 0;
    imuDropped = 0;
    emgSeq = 0;
    imuSeq = 0;

    // Reset frame sequence used in BLE packets
    // (declared below; forward-declared here via extern)
    // We'll reset it in loop once connected and before first send.

    rb_flush_all();
    imu_ticks = 0;
    samplingPaused = true;
  }

  void onDisconnect(BLEServer* /*pServer*/) override {
    isConnected = false;
    // Immediately pause sampling to avoid filling buffers while disconnected
    samplingPaused = true;
    clearHostPing();
    rb_flush_all();
    imu_ticks = 0;
  }
};

// ==================== IMU Task ====================
void imuTask(void* /*param*/) {
  for (;;) {
    if (imu_ticks > 0) {
      // consume one tick
      imu_ticks--;
      if (myICM.dataReady()) {
        myICM.getAGMT();
        ImuSample s;
        s.t_us = micros();
        s.seq = imuSeq++;
        s.ax = myICM.agmt.acc.axes.x;
        s.ay = myICM.agmt.acc.axes.y;
        s.az = myICM.agmt.acc.axes.z;
        s.gx = myICM.agmt.gyr.axes.x;
        s.gy = myICM.agmt.gyr.axes.y;
        s.gz = myICM.agmt.gyr.axes.z;
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
      while (!maxSensor.available()) {
        maxSensor.check();
        vTaskDelay(1);
      }
      redBuffer[i] = maxSensor.getRed();
      irBuffer[i]  = maxSensor.getIR();
      maxSensor.nextSample();
    }

    uint32_t lastIR  = irBuffer[SPO2_BUFFER_SIZE - 1];
    uint32_t lastRED = redBuffer[SPO2_BUFFER_SIZE - 1];

    // Auto brightness adjust (avoid max<int>/min<int> templates)
    if (lastIR > SATURATION_HIGH && ledBrightness > BRIGHTNESS_MIN) {
      int newB = max((int)BRIGHTNESS_MIN, (int)ledBrightness - 0x0F);
      ledBrightness = (byte)newB;
      configureMax3010x();
    } else if (lastIR < SATURATION_LOW && ledBrightness < BRIGHTNESS_MAX) {
      int newB = min((int)BRIGHTNESS_MAX, (int)ledBrightness + 0x0F);
      ledBrightness = (byte)newB;
      configureMax3010x();
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
  const uint16_t vitalsBytes = 8;  // spo2(2) hr_x10(2) age_ms(2) flags(1) bright(1)
  const uint16_t crcBytes = 2;

  if (maxPayload < (headerBytes + tlvHdrBytes + vitalsBytes + crcBytes)) return false;

  VitalsState v;
  portENTER_CRITICAL(&vitalsMux);
  v = vitals;
  portEXIT_CRITICAL(&vitalsMux);

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
  }
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("MyoWare + IMU + SpO2 BLE starting... (host-gated sampling + watchdog)");

  pinMode(PIN_LED, OUTPUT);
  led_set(false);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_RAW, ADC_ATTEN);
  analogSetPinAttenuation(PIN_ENV, ADC_ATTEN);
  if (REF_CONNECTED) analogSetPinAttenuation(PIN_REF, ADC_ATTEN);

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

  // ---- Timers ----
  esp_timer_create_args_t emgArgs = { .callback = &emg_timer_cb, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK, .name = "emg_timer" };
  esp_timer_create(&emgArgs, &emgTimer);
  esp_timer_start_periodic(emgTimer, EMG_PERIOD_US);

  esp_timer_create_args_t imuArgs = { .callback = &imu_timer_cb, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK, .name = "imu_timer" };
  esp_timer_create(&imuArgs, &imuTimer);
  esp_timer_start_periodic(imuTimer, IMU_PERIOD_US);

  // ---- Tasks ----
  xTaskCreatePinnedToCore(imuTask,  "imuTask",  4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(spo2Task, "spo2Task", 6144, NULL, 1, NULL, 1);

  Serial.printf("Timers started: EMG=%lu Hz, IMU=%lu Hz\n", (unsigned long)EMG_RATE_HZ, (unsigned long)IMU_RATE_HZ);

  // Start paused until first host heartbeat after a BLE connection
  samplingPaused = true;
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
    if (!samplingPaused) {
      samplingPaused = true;
      rb_flush_all();
      imu_ticks = 0;
      Serial.println("[WD] Disconnected. Pausing sampling.");
    }
  } else {
    // Connected: pause if host not alive; resume only when hostAlive
    if (!hostAlive) {
      if (!samplingPaused) {
        samplingPaused = true;
        rb_flush_all();
        imu_ticks = 0;
        Serial.printf("[WD] Host heartbeat missing (%lu ms). Pausing sampling/streaming.\n", (unsigned long)hostAge);
      }
    } else {
      if (samplingPaused) {
        samplingPaused = false;
        rb_flush_all();
        imu_ticks = 0;
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
  led_update(mode);

  // ---- TX loop ----
  if (isConnected && pTXCharacteristic && !samplingPaused) {
    bool sent = false;

    sent |= send_emg_frame(maxNotifyPayload);

    static uint32_t lastImuSendMs = 0;
    if (nowMs - lastImuSendMs >= 20 && emg_available() < EMG_BUF_SIZE * 3 / 4) {
      lastImuSendMs = nowMs;
      sent |= send_imu_frame(maxNotifyPayload);
    }

    if (nowMs - lastVitalsSendMs >= 30000) {
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

    Serial.printf("[Health] MTU=%u EMGavail=%u IMUavail=%u EMGdrop=%lu IMUdrop=%lu Frames/s=%lu | HostAge=%lu ms Paused=%u | SpO2=%s HR=%s\n",
                  negotiatedMTU, emg_available(), imu_available(),
                  (unsigned long)emgDropped, (unsigned long)imuDropped, (unsigned long)fps,
                  (unsigned long)hostAge, samplingPaused ? 1 : 0,
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
