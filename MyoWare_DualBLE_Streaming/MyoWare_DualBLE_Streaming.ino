
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MyoWare.h>

// BLE UUIDs for Nordic UART Service
#define NUS_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID        "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// Device name
const char* DEVICE_NAME = "MyoW1";

// MyoWare object
MyoWare myoware;

// BLE objects
BLEServer* pServer = nullptr;
BLECharacteristic* pTXCharacteristic = nullptr;
bool isConnected = false;
bool previousConnected = false; // Added for reconnect logic

// Sampling parameters
const uint16_t SAMPLE_RATE_HZ = 100;       // Target sampling rate
const uint32_t SAMPLE_INTERVAL_US = 1000000UL / SAMPLE_RATE_HZ; // 5000 µs for 200 Hz
uint32_t lastSampleMicros = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { isConnected = true; }
  void onDisconnect(BLEServer* pServer) { isConnected = false; }
};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("A3=%d A4=%d A5=%d\n", A3, A4, A5);
  // Configure MyoWare pins
  myoware.setConvertOutput(false); // Set to true to convert ADC output to the amplitude of
                                      // of the muscle activity as it appears at the electrodes
                                      // in millivolts
  myoware.setGainPotentiometer(50.);  // Gain potentiometer resistance in kOhms.
                                      // adjust the potentiometer setting such that the
                                      // max muscle reading is below 3.3V then update this
                                      // parameter to the measured value of the potentiometer
  myoware.setENVPin(A3);
  myoware.setRAWPin(A4);
  myoware.setREFPin(A5);

  pinMode(myoware.getStatusLEDPin(), OUTPUT);
  digitalWrite(myoware.getStatusLEDPin(), HIGH);

  // BLE setup
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(NUS_SERVICE_UUID);

  pTXCharacteristic = pService->createCharacteristic(
    NUS_TX_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTXCharacteristic->addDescriptor(new BLE2902());

  // RX characteristic (not used now, but created for completeness)
  pService->createCharacteristic(
    NUS_RX_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
  //BLEDevice::setMTU(247); // Larger MTU for better throughput
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started. Waiting for connection...");
}

void loop() {
  uint32_t nowMicros = micros();
  if (nowMicros - lastSampleMicros >= SAMPLE_INTERVAL_US) {
    lastSampleMicros = nowMicros;

    // Read sensor values
    int env = analogRead(A3);
    int raw = analogRead(A4);
    int ref = analogRead(A5);

    // Debug prints
    Serial.printf("analogRead ENV=%d RAW=%d REF=%d\n", env, raw, ref);
    Serial.printf("A3(GPIO39)=%d  A4(GPIO32)=%d  A5(GPIO33)=%d\n",
                  analogRead(39), analogRead(32), analogRead(33));

    uint32_t env_mv = analogReadMilliVolts(39);
    uint32_t raw_mv = analogReadMilliVolts(32);
    uint32_t ref_mv = analogReadMilliVolts(33);

    Serial.printf("GPIO ENV=%d (%lumV)  RAW=%d (%lumV)  REF=%d (%lumV)\n",
                  env, env_mv, raw, raw_mv, ref, ref_mv);
    // end of debug prints

    // Send data over BLE
    if (isConnected && pTXCharacteristic) {
      char buffer[32];
      snprintf(buffer, sizeof(buffer), "%d,%d,%d\r\n", env, raw, ref);
      pTXCharacteristic->setValue(buffer);
      pTXCharacteristic->notify();
      delay(5); // Helps BLE stack breathe
    }

    // Blink LED if not connected
    if (!isConnected) {
      myoware.blinkStatusLED();
    } else {
      digitalWrite(myoware.getStatusLEDPin(), HIGH);
    }
  }

  // Restart advertising after disconnect
  if (!isConnected && previousConnected) {
    Serial.println("Disconnected. Restarting advertising...");
    delay(500);
    pServer->startAdvertising();
    previousConnected = isConnected;
  } else if (isConnected && !previousConnected) {
    Serial.println("Connected to central.");
    previousConnected = isConnected;
  }
}
