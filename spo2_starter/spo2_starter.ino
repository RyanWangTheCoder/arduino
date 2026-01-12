
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"  // SparkFun library includes this

MAX30105 sensor;

// ESP32 I²C pins (Qwiic defaults)
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

static const uint8_t I2C_ADDR = 0x57;

// Buffer for SpO₂ algorithm
const int SPO2_BUFFER_SIZE = 100; // ~4 sec at 25 Hz
uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];

// HR smoothing
float hrAvg = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("Initializing MAX30102...");

  if (!sensor.begin(Wire, I2C_SPEED_STANDARD, I2C_ADDR)) {
    Serial.println("Sensor not found. Check wiring.");
    while (true) delay(1000);
  }

  // Configure sensor for SpO₂
  byte ledBrightness = 0x1F; // Lower brightness to avoid saturation
  byte sampleAverage = 4;
  byte ledMode       = 2;    // Red + IR
  int sampleRate     = 25;   // Lower rate for SpO₂
  int pulseWidth     = 411;  // Wide pulse for better SNR
  int adcRange       = 4096;

  sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  Serial.println("Place finger firmly, block ambient light, hold still...");
}

void loop() {
  // Collect SPO2_BUFFER_SIZE samples (~4 sec at 25 Hz)
  for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
    while (!sensor.available()) {
      sensor.check();
      delay(1);
    }

    redBuffer[i] = sensor.getRed();
    irBuffer[i]  = sensor.getIR();
    sensor.nextSample();
  }

  // Compute SpO₂ and HR
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

  // Print results
  Serial.print("SpO₂=");
  if (spo2Valid) Serial.print(spo2);
  else Serial.print("NA");

  Serial.print("%  HR=");
  if (hrValid) {
    hrAvg = (hrAvg == 0) ? heartRate : (0.8 * hrAvg + 0.2 * heartRate);
    Serial.print(hrAvg, 1);
  } else {
    Serial.print("NA");
  }
  Serial.println(" bpm");

  // Debug: last raw values
  Serial.print("IR=");
  Serial.print(irBuffer[SPO2_BUFFER_SIZE - 1]);
  Serial.print("  RED=");
  Serial.println(redBuffer[SPO2_BUFFER_SIZE - 1]);

  // Wait 30 seconds before next measurement
  delay(10000);
}
