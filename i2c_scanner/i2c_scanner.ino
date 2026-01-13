
#include <Wire.h>

#ifndef SDA_PIN
#define SDA_PIN 21   // change if your Qwiic/I2C pins are different
#endif
#ifndef SCL_PIN
#define SCL_PIN 22   // change if your Qwiic/I2C pins are different
#endif

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("I2C scan...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("Found 0x");
      Serial.println(addr, HEX);
    }
    delay(2);
  }
}

void loop() {}
