\
/*
  MyoWare 2.0 Wireless Shield — Dual BLE Streaming (Bluefruit + Binary)
  - Nordic UART Service (NUS) for Bluefruit Connect app (CSV "ENV,RAW" lines)
  - Existing 20-byte RAW packet notifications for high-rate logging

  UART (NUS):
    Service:     MyoWareBLE::uuidUARTService
    TX (notify): MyoWareBLE::uuidUARTTXCharacteristic
    RX (write):  MyoWareBLE::uuidUARTRXCharacteristic

  Binary Packet:
    Service:     MyoWareBLE::uuidMyoWareService
    Characteristic (notify|read): MyoWareBLE::uuidMyoWareCharacteristic
    Payload (20 bytes):
      [0]   : seq (uint8_t)
      [1-18]: 9 x RAW samples (uint16_t LE)
      [19]  : flags (reserved)

  Commands over UART RX (ASCII):
    START / STOP
    MODE=CSV | MODE=PKT           (select streaming mode)
    RATE_CSV=<Hz>                 (e.g., RATE_CSV=200)
    RATE_PKT=<Hz>                 (e.g., RATE_PKT=1000)
    MV=0 | MV=1                   (disable/enable mV conversion in MyoWare lib)
    DC=0 | DC=1                   (disable/enable RAW-REF DC offset removal)
    PING                          (responds PONG)

  Defaults:
    kFsPktHz         = 1000 Hz
    kSamplesPerPacket= 9
    kFsCsvHz         = 200 Hz

  Notes:
  - Designed for ArduinoBLE + MyoWare library. Ensure both are installed.
  - Advertises UART service for Bluefruit Connect discovery; binary service is
    available after connection.
*/

#include <ArduinoBLE.h>
#include <MyoWare.h>

// ---------- User-configurable ----------
const char*   kLocalName          = "MyoWareSensor1";   // make unique per shield
const uint32_t kFsPktHz           = 1000;               // binary packet sampling rate (Hz)
const uint8_t  kSamplesPerPacket  = 9;                  // fits 20-byte ATT payload
const uint32_t kFsCsvHz           = 200;                // UART CSV rate (Hz)
// --------------------------------------

// MyoWare object
MyoWare myoware;

// Status LED (from MyoWare library)
const int statusLED = myoware.getStatusLEDPin();

// --- UART (Nordic UART Service) ---
BLEService uartService(MyoWareBLE::uuidUARTService.c_str());

// TX characteristic (Notify). Allocate enough bytes for a CSV line + header.
BLECharacteristic uartTX(MyoWareBLE::uuidUARTTXCharacteristic.c_str(),
                         BLENotify, 64);

// RX characteristic (Write/Write Without Response). For incoming commands.
BLECharacteristic uartRX(MyoWareBLE::uuidUARTRXCharacteristic.c_str(),
                         BLEWrite | BLEWriteWithoutResponse, 64);

// --- Binary packet service/characteristic (kept from user's design) ---
BLEService myoWareService(MyoWareBLE::uuidMyoWareService.c_str());
BLECharacteristic emgPacketChar(MyoWareBLE::uuidMyoWareCharacteristic.c_str(),
                                BLERead | BLENotify, 20);

// Streaming state
volatile uint8_t seq = 0;
uint16_t         packetBuf[kSamplesPerPacket];
uint8_t          bufCount = 0;

// Timing
uint32_t samplePktIntervalUs = 0;     // computed from kFsPktHz
uint32_t nextPktDueUs        = 0;

uint32_t csvPeriodMs         = 0;     // computed from kFsCsvHz
uint32_t nextCsvDueMs        = 0;

// Mode & options
bool streamEnabled = true;            // start streaming on connect
enum Mode { MODE_CSV, MODE_PKT };
Mode currentMode   = MODE_CSV;        // default to UART CSV for Bluefruit
bool convertToMv   = false;           // myoware.setConvertOutput()
bool dcRemove      = false;           // subtract REF from RAW

// Utility: send one UART line (notify)
static void uart_send_line(const char* s) {
  size_t len = strlen(s);
  if (len > 63) len = 63; // keep within allocated size
  uartTX.writeValue((const uint8_t*)s, len);
}

// Parse ASCII commands from RX characteristic
static void handle_uart_command(const uint8_t* data, size_t len) {
  // Copy to a local, null-terminated buffer
  char buf[65];
  if (len > 64) len = 64;
  memcpy(buf, data, len);
  buf[len] = '\0';

  // Trim trailing \r\n
  while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n')) {
    buf[--len] = '\0';
  }

  // Simple dispatch
  if (strcmp(buf, "START") == 0) {
    streamEnabled = true;
    uart_send_line("OK START\n");
  } else if (strcmp(buf, "STOP") == 0) {
    streamEnabled = false;
    uart_send_line("OK STOP\n");
  } else if (strcmp(buf, "MODE=CSV") == 0) {
    currentMode = MODE_CSV;
    uart_send_line("OK MODE CSV\n");
  } else if (strcmp(buf, "MODE=PKT") == 0) {
    currentMode = MODE_PKT;
    uart_send_line("OK MODE PKT\n");
  } else if (strncmp(buf, "RATE_CSV=", 9) == 0) {
    int hz = atoi(buf + 9);
    if (hz >= 10 && hz <= 500) {
      csvPeriodMs  = (uint32_t)(1000UL / (uint32_t)hz);
      uart_send_line("OK RATE_CSV\n");
    } else {
      uart_send_line("ERR RATE_CSV_RANGE\n");
    }
  } else if (strncmp(buf, "RATE_PKT=", 9) == 0) {
    int hz = atoi(buf + 9);
    if (hz >= 100 && hz <= 2000) {
      samplePktIntervalUs = (uint32_t)(1000000UL / (uint32_t)hz);
      uart_send_line("OK RATE_PKT\n");
    } else {
      uart_send_line("ERR RATE_PKT_RANGE\n");
    }
  } else if (strcmp(buf, "MV=1") == 0) {
    convertToMv = true;
    myoware.setConvertOutput(true);
    uart_send_line("OK MV=1\n");
  } else if (strcmp(buf, "MV=0") == 0) {
    convertToMv = false;
    myoware.setConvertOutput(false);
    uart_send_line("OK MV=0\n");
  } else if (strcmp(buf, "DC=1") == 0) {
    dcRemove = true;
    uart_send_line("OK DC=1\n");
  } else if (strcmp(buf, "DC=0") == 0) {
    dcRemove = false;
    uart_send_line("OK DC=0\n");
  } else if (strcmp(buf, "PING") == 0) {
    uart_send_line("PONG\n");
  } else {
    uart_send_line("ERR UNKNOWN_CMD\n");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // MyoWare configuration (pins per Wireless Shield)
  myoware.setConvertOutput(convertToMv);
  myoware.setGainPotentiometer(50.0f);  // update this to your measured kΩ
  myoware.setENVPin(A3);
  myoware.setRAWPin(A4);
  myoware.setREFPin(A5);

  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, HIGH); // LED on during BLE init

  // BLE init
  if (!BLE.begin()) {
    Serial.println("FAILED - BLE Initialization!");
    while (1) { delay(1000); }
  }

  BLE.setLocalName(kLocalName);

  // --- UART service ---
  BLE.addService(uartService);
  uartService.addCharacteristic(uartTX);
  uartService.addCharacteristic(uartRX);
  // TX initial value (empty)
  uartTX.writeValue((const uint8_t*)"", 0);

  // --- Binary packet service ---
  BLE.addService(myoWareService);
  myoWareService.addCharacteristic(emgPacketChar);
  // Init 20-byte to zeros
  {
    uint8_t zero[20] = {0};
    emgPacketChar.writeValue(zero, sizeof(zero));
  }

  // Advertise UART service for Bluefruit
  BLE.setAdvertisedService(uartService);
  BLE.advertise();

  digitalWrite(statusLED, LOW);

  // Timing setup
  samplePktIntervalUs = (uint32_t)(1000000UL / kFsPktHz);
  nextPktDueUs        = micros() + samplePktIntervalUs;

  csvPeriodMs         = (uint32_t)(1000UL / kFsCsvHz);
  nextCsvDueMs        = millis() + csvPeriodMs;

  Serial.println("MyoWare dual BLE streaming ready.");
  Serial.print("LocalName: "); Serial.println(kLocalName);
  Serial.print("UART Service UUID: "); Serial.println(uartService.uuid());
  Serial.print("UART TX UUID: "); Serial.println(uartTX.uuid());
  Serial.print("UART RX UUID: "); Serial.println(uartRX.uuid());
  Serial.print("Packet Service UUID: "); Serial.println(myoWareService.uuid());
  Serial.print("Packet Char UUID: "); Serial.println(emgPacketChar.uuid());
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: "); Serial.println(central.address());
    digitalWrite(statusLED, HIGH);

    // Handshake banner + header for Bluefruit Plotter
    uart_send_line("HELLO MyoWare EMG v1.0\n");
    uart_send_line("MODE CSV\n");
    uart_send_line("ENV,RAW\n"); // header line

    // Connected loop
    while (central.connected()) {
      // Service BLE (RX commands)
      BLE.poll();

      if (uartRX.written()) {
        // Read the command and handle
        int len = uartRX.valueLength();
        uint8_t cmd[64];
        if (len > 64) len = 64;
        uartRX.readValue(cmd, len);
        handle_uart_command(cmd, len);
      }

      if (!streamEnabled) {
        // Idle while connected (still process commands)
        continue;
      }

      // --- CSV streaming for Bluefruit UART ---
      if (currentMode == MODE_CSV) {
        uint32_t nowMs = millis();
        if ((int32_t)(nowMs - nextCsvDueMs) >= 0) {
          // Read ENV and RAW
          double env = myoware.readSensorOutput(MyoWare::ENVELOPE);
          double raw = myoware.readSensorOutput(MyoWare::RAW);
          if (dcRemove) {
            // If library supports REFERENCE enumeration; otherwise replace with analogRead(A5)
            double ref = myoware.readSensorOutput(MyoWare::REFERENCE);
            raw -= ref;
          }

          // Compose CSV "env,raw\n"
          char line[64];
          snprintf(line, sizeof(line), "%.6f,%.6f\n", env, raw);
          uart_send_line(line);

          nextCsvDueMs += csvPeriodMs;
        }
      }

      // --- Binary packet streaming (RAW only) ---
      if (currentMode == MODE_PKT) {
        uint32_t nowUs = micros();
        if ((int32_t)(nowUs - nextPktDueUs) >= 0) {
          // Sample RAW
          double raw_d = myoware.readSensorOutput(MyoWare::RAW);
          if (dcRemove) {
            double ref = myoware.readSensorOutput(MyoWare::REFERENCE);
            raw_d -= ref;
          }
          // Transport as uint16_t (clip to 0..65535 if needed)
          uint32_t raw_u32 = (raw_d < 0) ? 0 : (raw_d > 65535.0 ? 65535 : (uint32_t)raw_d);
          uint16_t raw_u16 = (uint16_t)raw_u32;

          if (bufCount < kSamplesPerPacket) {
            packetBuf[bufCount++] = raw_u16;
          }

          if (bufCount == kSamplesPerPacket) {
            uint8_t payload[20];
            payload[0] = seq++;
            uint8_t w = 1;
            for (uint8_t i = 0; i < kSamplesPerPacket; ++i) {
              uint16_t s = packetBuf[i];
              payload[w++] = (uint8_t)(s & 0xFF);
              payload[w++] = (uint8_t)(s >> 8);
            }
            payload[19] = 0x00; // flags/reserved
            emgPacketChar.writeValue(payload, sizeof(payload));
            bufCount = 0;
          }

          nextPktDueUs += samplePktIntervalUs;
        }
      }
    }

    // Disconnected
    digitalWrite(statusLED, LOW);
    Serial.print("Disconnected from central: "); Serial.println(central.address());
    bufCount      = 0;
    streamEnabled = true;      // auto-enable on next connect
    currentMode   = MODE_CSV;  // default back to CSV for Bluefruit
    nextPktDueUs  = micros() + samplePktIntervalUs;
    nextCsvDueMs  = millis() + csvPeriodMs;
  } else {
    // Idle animation + BLE service
    myoware.blinkStatusLED();
    BLE.poll();
  }
}
\
