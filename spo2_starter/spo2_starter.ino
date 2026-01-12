
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

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

// LED brightness range
byte ledBrightness = 0x1F; // Start low
const byte BRIGHTNESS_MIN = 0x0F;
const byte BRIGHTNESS_MAX = 0x7F;

// Saturation thresholds
const uint32_t SATURATION_HIGH = 200000; // ADC near max
const uint32_t SATURATION_LOW  = 10000;  // Very weak signal

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

  configureSensor();
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

  // Auto brightness adjustment based on last IR value
  uint32_t lastIR = irBuffer[SPO2_BUFFER_SIZE - 1];
  if (lastIR > SATURATION_HIGH && ledBrightness > BRIGHTNESS_MIN) {
    ledBrightness -= 0x0F; // decrease brightness
    if (ledBrightness < BRIGHTNESS_MIN) ledBrightness = BRIGHTNESS_MIN;
    Serial.print("Reducing brightness to: ");
    Serial.println(ledBrightness);
    configureSensor();
  } else if (lastIR < SATURATION_LOW && ledBrightness < BRIGHTNESS_MAX) {
    ledBrightness += 0x0F; // increase brightness
    if (ledBrightness > BRIGHTNESS_MAX) ledBrightness = BRIGHTNESS_MAX;
    Serial.print("Increasing brightness to: ");
    Serial.println(ledBrightness);
    configureSensor();
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
  Serial.print(lastIR);
  Serial.print("  RED=");
  Serial.println(redBuffer[SPO2_BUFFER_SIZE - 1]);

  // Wait 30 seconds before next measurement
  delay(30000);
}

void configureSensor() {
  byte sampleAverage = 4;
  byte ledMode       = 2;    // Red + IR
  int sampleRate     = 25;   // Lower rate for SpO₂
  int pulseWidth     = 411;  // Wide pulse for better SNR
  int adcRange       = 4096;

  sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}
