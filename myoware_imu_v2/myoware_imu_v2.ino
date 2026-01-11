
#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MyoWare.h>
#include "ICM_20948.h"

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

static const char* DEVICE_NAME = "MyoW1";

static constexpr int PIN_ENV = 39; // A3
static constexpr int PIN_RAW = 32; // A4
static constexpr int PIN_REF = 33; // A5
static constexpr bool REF_CONNECTED = false;

static constexpr uint32_t EMG_RATE_HZ = 2000;
static constexpr uint32_t IMU_RATE_HZ = 200;
static constexpr uint32_t EMG_PERIOD_US = 1000000UL / EMG_RATE_HZ;
static constexpr uint32_t IMU_PERIOD_US = 1000000UL / IMU_RATE_HZ;

static constexpr uint16_t EMG_BUF_SIZE = 2048;
static constexpr uint16_t IMU_BUF_SIZE = 512;

static constexpr uint32_t HEALTH_REPORT_MS = 1000;

static constexpr uint8_t MAGIC0 = 0xA5;
static constexpr uint8_t MAGIC1 = 0x5A;
static constexpr uint8_t PROTO_VER = 0x01;
static constexpr uint8_t FLAG_CRC16 = 0x01;

static constexpr uint8_t TLV_EMG_BATCH = 0x01;
static constexpr uint8_t TLV_IMU_BATCH = 0x02;

struct EmgSample { uint16_t env, raw, ref; uint32_t t_us, seq; };
struct ImuSample { int16_t ax, ay, az, gx, gy, gz; uint32_t t_us, seq; };

static EmgSample emgBuf[EMG_BUF_SIZE];
static ImuSample imuBuf[IMU_BUF_SIZE];
static volatile uint16_t emgHead = 0, emgTail = 0;
static volatile uint16_t imuHead = 0, imuTail = 0;
static volatile uint32_t emgSeq = 0, imuSeq = 0;
static volatile uint32_t emgDropped = 0, imuDropped = 0;

static portMUX_TYPE emgMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;

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

/******************** Sensors ********************/
MyoWare myoware;
ICM_20948_I2C myICM;
static constexpr uint8_t AD0_VAL = 1;

/******************** BLE ********************/
BLEServer* pServer = nullptr;
BLECharacteristic* pTXCharacteristic = nullptr;
volatile bool isConnected = false;
bool previousConnected = false;
uint16_t negotiatedMTU = 23;
uint16_t maxNotifyPayload = 20;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* /*pServer*/) override {
    isConnected = true;
    negotiatedMTU = BLEDevice::getMTU();
    maxNotifyPayload = (negotiatedMTU > 3) ? (negotiatedMTU - 3) : 20;
    Serial.printf("[BLE] Connected. MTU=%u, maxNotifyPayload=%u\n", negotiatedMTU, maxNotifyPayload);
    if (negotiatedMTU < 100) {
      Serial.println("[WARN] MTU too low for 2kHz EMG; expect drops.");
    }
  }
  void onDisconnect(BLEServer* /*pServer*/) override {
    isConnected = false;
  }
};

/******************** Timers ********************/
static esp_timer_handle_t emgTimer = nullptr;
static esp_timer_handle_t imuTimer = nullptr;
static volatile bool imu_tick = false;

static void emg_timer_cb(void* /*arg*/) {
  EmgSample s;
  s.t_us = micros();
  s.seq = emgSeq++;
  s.env = analogRead(PIN_ENV);
  s.raw = analogRead(PIN_RAW);
  s.ref = REF_CONNECTED ? analogRead(PIN_REF) : 0;
  emg_push(s);
}

static void imu_timer_cb(void* /*arg*/) {
  imu_tick = true;
}

/******************** IMU Task ********************/
void imuTask(void* /*param*/) {
  for (;;) {
    if (imu_tick) {
      imu_tick = false;
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

bool send_emg_frame(uint16_t maxPayload) {
  const uint16_t headerBytes = 14;
  const uint16_t tlvHdrBytes = 3;
  const uint16_t emgMetaBytes = 13;
  const uint16_t crcBytes = 2;

  if (maxPayload < (headerBytes + tlvHdrBytes + emgMetaBytes + crcBytes + 6)) return false;

  uint16_t maxSamples = (maxPayload - headerBytes - tlvHdrBytes - emgMetaBytes - crcBytes) / 6;
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

  // Header
  p[0] = MAGIC0; p[1] = MAGIC1; p[2] = PROTO_VER; p[3] = FLAG_CRC16;
  wr_u32(p += 4, frameSeq++);
  wr_u32(p, micros());
  wr_u16(p, 0); // payload_len placeholder
  uint8_t* payload_start = buf + headerBytes;
  p = payload_start;

  // TLV header
  *p++ = TLV_EMG_BATCH;
  wr_u16(p, 0);
  uint8_t* tlvLenPos = p - 2;

  // EMG meta
  wr_u32(p, batch[0].seq);
  int32_t off = (int32_t)batch[0].t_us - (int32_t)micros();
  if (off > 32767) off = 32767; if (off < -32768) off = -32768;
  wr_i16(p, (int16_t)off);
  wr_u16(p, EMG_RATE_HZ);
  *p++ = 3;
  *p++ = REF_CONNECTED ? 0x07 : 0x03;
  *p++ = 0;
  wr_u16(p, N);

  for (uint16_t i = 0; i < N; i++) {
    wr_u16(p, batch[i].env);
    wr_u16(p, batch[i].raw);
    wr_u16(p, batch[i].ref);
  }

  uint16_t tlvLen = emgMetaBytes + 6 * N;
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
  const uint16_t imuMetaBytes = 12;
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

  p[0] = MAGIC0; p[1] = MAGIC1; p[2] = PROTO_VER; p[3] = FLAG_CRC16;
  wr_u32(p += 4, frameSeq++);
  wr_u32(p, micros());
  wr_u16(p, 0);
  uint8_t* payload_start = buf + headerBytes;
  p = payload_start;

  *p++ = TLV_IMU_BATCH;
  wr_u16(p, 0);
  uint8_t* tlvLenPos = p - 2;

  wr_u32(p, batch[0].seq);
  int32_t off = (int32_t)batch[0].t_us - (int32_t)micros();
  if (off > 32767) off = 32767; if (off < -32768) off = -32768;
  wr_i16(p, (int16_t)off);
  wr_u16(p, IMU_RATE_HZ);
  *p++ = N;
  *p++ = 0;
  wr_u16(p, 0x003F);

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

/******************** Setup ********************/
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("MyoWare BLE starting...");

  analogReadResolution(12);
  Wire.begin();
  Wire.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok) initialized = true;
    else { Serial.println("IMU init retry..."); delay(500); }
  }

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(247);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* pService = pServer->createService(NUS_SERVICE_UUID);
  pTXCharacteristic = pService->createCharacteristic(NUS_TX_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTXCharacteristic->addDescriptor(new BLE2902());
  pService->createCharacteristic(NUS_RX_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
  BLEDevice::startAdvertising();

  esp_timer_create_args_t emgArgs = { .callback = &emg_timer_cb, .name = "emg_timer" };
  esp_timer_create(&emgArgs, &emgTimer);
  esp_timer_start_periodic(emgTimer, EMG_PERIOD_US);

  esp_timer_create_args_t imuArgs = { .callback = &imu_timer_cb, .name = "imu_timer" };
  esp_timer_create(&imuArgs, &imuTimer);
  esp_timer_start_periodic(imuTimer, IMU_PERIOD_US);

  xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, NULL, 1, NULL, 1);

  Serial.printf("Timers started: EMG=%lu Hz, IMU=%lu Hz\n", EMG_RATE_HZ, IMU_RATE_HZ);
}

void loop() {
  static uint32_t lastHealthMs = 0;
  static uint32_t framesSent = 0;
  static uint32_t lastFramesSent = 0;

  if (isConnected && pTXCharacteristic) {
    bool sent = false;

    // Send EMG aggressively
    sent |= send_emg_frame(maxNotifyPayload);

    // Send IMU every ~20ms (but skip if EMG backlog is high)
    static uint32_t lastImuSendMs = 0;
    if (millis() - lastImuSendMs >= 20 && emg_available() < EMG_BUF_SIZE * 3 / 4) {
      lastImuSendMs = millis();
      sent |= send_imu_frame(maxNotifyPayload);
    }

    if (sent) framesSent++;
  }

  // Health report every second
  if (millis() - lastHealthMs >= HEALTH_REPORT_MS) {
    lastHealthMs = millis();
    uint32_t fps = framesSent - lastFramesSent;
    lastFramesSent = framesSent;
    Serial.printf("[Health] MTU=%u EMGavail=%u IMUavail=%u EMGdrop=%lu IMUdrop=%lu Frames/s=%lu\n",
                  negotiatedMTU, emg_available(), imu_available(),
                  (unsigned long)emgDropped, (unsigned long)imuDropped, (unsigned long)fps);
  }

  // Handle disconnect/reconnect
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