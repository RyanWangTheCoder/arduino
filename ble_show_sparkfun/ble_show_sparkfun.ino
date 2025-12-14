
/*
 * Arduino Nano 33 BLE — Print Bluetooth Address (Minimal)
 * Library: ArduinoBLE (Library Manager → "ArduinoBLE")
 */

#include <ArduinoBLE.h>

void setup() {
  Serial.begin(115200);
  // Wait up to 3 seconds for Serial (avoids hanging forever if USB isn't open)
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000)) { /* wait briefly */ }

  if (!BLE.begin()) {
    Serial.println("ERROR: Starting BLE failed!");
       while (1) { delay(1000); }  // halt if BLE can't start
  }

  // Optional: give the device a friendly name and start advertising
  BLE.setLocalName("Nano33BLE");
  BLE.advertise();

  Serial.println("=== Nano 33 BLE ===");
  Serial.print("BLE address: ");
  Serial.println(BLE.address());   // e.g., "F1:23:45:67:89:AB"
  Serial.println("Advertising started...");
}

void loop() {
  // Keep the BLE stack responsive
  BLE.poll();

  // Reprint the address every 5 seconds (handy for monitoring)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 5000) {
    lastPrint = millis();
    Serial.print("BLE address: ");
    Serial.println(BLE.address());
  }
}
