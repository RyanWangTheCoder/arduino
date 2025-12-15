
/*
  MyoWare 2.0 Wireless Shield — RAW EMG Packet Streaming (BLE)
  - Streams RAW EMG as 20-byte notifications:
      byte 0   : sequence counter (uint8_t, wraps at 255)
      bytes 1-18: 9 samples of RAW EMG (uint16_t, little-endian)
      byte 19  : flags/reserved (0 for now)

  Notes:
  - Default sample rate: 1000 Hz
  - Default samples per packet: 9 (fits 20-byte ATT payload at MTU=23)
  - RAW EMG on MyoWare is band-pass filtered ~20–498 Hz and amplified.
    You can compute RECT (abs) and ENV (LPF ~3–5 Hz) on the central.

  Based on MyoWare Arduino library examples (Advancer Technologies).
*/

#include <ArduinoBLE.h>
#include <MyoWare.h>

// ---------- User-configurable ----------
const char* kLocalName = "MyoWareSensor1";      // keep same or make unique
const uint32_t kFs_Hz = 1000;                   // sampling rate (Hz)
const uint8_t  kSamplesPerPacket = 9;           // 9 * 2 bytes + 1 seq + 1 flags = 20
// --------------------------------------

// MyoWare class object and I/O config
MyoWare myoware;

// BLE Service/Characteristic (reuse MyoWare UUIDs for compatibility)
BLEService myoWareService(MyoWareBLE::uuidMyoWareService.c_str());

// Binary characteristic (20-byte payload)
BLECharacteristic emgPacketChar(
  MyoWareBLE::uuidMyoWareCharacteristic.c_str(),
  BLERead | BLENotify, 20
);

// Status LED on shield (managed by MyoWare lib)
const int statusLED = myoware.getStatusLEDPin();

// Sampling state
volatile uint8_t  seq = 0;
uint16_t          packetBuf[kSamplesPerPacket];
uint8_t           bufCount = 0;

// Timing
uint32_t sampleIntervalUs = (uint32_t)(1000000UL / kFs_Hz);
uint32_t nextSampleDueUs  = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // MyoWare configuration
  myoware.setConvertOutput(false); // return ADC-scale values (not mV) for RAW
  myoware.setGainPotentiometer(50.0f);
  myoware.setENVPin(A3);  // default pins for Wireless Shield
  myoware.setRAWPin(A4);
  myoware.setREFPin(A5);

  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, HIGH);   // LED on while waiting for BLE init

  // BLE init
  if (!BLE.begin()) {
    Serial.println("FAILED - BLE Initialization!");
    while (1) { delay(1000); }
  }

  BLE.setLocalName(kLocalName);
  BLE.setAdvertisedService(myoWareService);
  myoWareService.addCharacteristic(emgPacketChar);
  BLE.addService(myoWareService);

  // Initialize characteristic value (20 bytes of zeros)
  uint8_t zero[20] = {0};
  emgPacketChar.writeValue(zero, sizeof(zero));

  BLE.advertise();
  digitalWrite(statusLED, LOW);

  // Prepare sampling schedule
  nextSampleDueUs = micros() + sampleIntervalUs;

  Serial.println("MyoWare RAW EMG streaming ready (BLE notify).");
  Serial.print("LocalName: "); Serial.println(kLocalName);
  Serial.print("Service UUID: "); Serial.println(myoWareService.uuid());
  Serial.print("Char UUID: "); Serial.println(emgPacketChar.uuid());
  Serial.print("Fs: "); Serial.print(kFs_Hz); Serial.println(" Hz");
  Serial.print("Samples/packet: "); Serial.println(kSamplesPerPacket);
}

void loop() {
  // Wait for central
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: "); Serial.println(central.address());
    digitalWrite(statusLED, HIGH);

    // Sampling + packetization while connected
    while (central.connected()) {
      uint32_t nowUs = micros();

      // Timed sampling (busy-wait to keep jitter low)
      if ((int32_t)(nowUs - nextSampleDueUs) >= 0) {
        // Read RAW EMG via MyoWare lib
        // Returns a number proportional to ADC value; cast to uint16_t for transport.
        uint16_t raw = (uint16_t)myoware.readSensorOutput(MyoWare::RAW);

        // Buffer the sample
        if (bufCount < kSamplesPerPacket) {
          packetBuf[bufCount++] = raw;
        }

        // Packet ready? Build and notify
        if (bufCount == kSamplesPerPacket) {
          uint8_t payload[20];
          payload[0] = seq++;            // sequence counter
          // Pack 9 samples, little-endian
          uint8_t w = 1;
          for (uint8_t i = 0; i < kSamplesPerPacket; ++i) {
            uint16_t s = packetBuf[i];
            payload[w++] = (uint8_t)(s & 0xFF);
            payload[w++] = (uint8_t)(s >> 8);
          }
          payload[19] = 0x00;            // flags/reserved

          // Notify central
          emgPacketChar.writeValue(payload, sizeof(payload));

          // Reset buffer
          bufCount = 0;
        }

        // Schedule next sample
        nextSampleDueUs += sampleIntervalUs;
      }

      // Keep BLE stack serviced
      BLE.poll();
    }

    // Central disconnected
    digitalWrite(statusLED, LOW);
    Serial.print("Disconnected from central: "); Serial.println(central.address());
    bufCount = 0; // clear any partial packet
  } else {
       // Idle animation
    myoware.blinkStatusLED();
    BLE.poll();
  }
}
