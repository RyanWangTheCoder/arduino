
#include <Wire.h>
#include "MAX30105.h"

MAX30105 sensor;

// IMPORTANT: use the SAME SDA/SCL pins you used for the I2C scanner.
// If you used defaults in the scanner, keep these. Otherwise change them.
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // start slow; you can try 400k later

  Serial.println("Starting MAX3010x...");

  // begin(wirePort, i2cSpeed, i2cAddr)
  if (!sensor.begin(Wire, I2C_SPEED_STANDARD, 0x57)) { // 0x57 is default in library
    Serial.println("MAX3010x not found by library (even though I2C scan sees 0x57).");
    Serial.println("Next step: read Part ID register (see section 2).");
    while (true) { delay(1000); } // delay => no WDT reset
  }

  Serial.println("Sensor found. Calling setup()...");
  sensor.setup();  // default configuration
}

void loop() {
  sensor.check();

  while (sensor.available()) {
    uint32_t ir  = sensor.getIR();
    uint32_t red = sensor.getRed();

    Serial.print("IR=");
    Serial.print(ir);
    Serial.print("  RED=");
    Serial.println(red);

    sensor.nextSample();
  }

  delay(10);
}
