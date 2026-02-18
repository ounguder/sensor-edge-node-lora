#include <Arduino.h>

// Feed a known stable voltage to GPIO7 (ADC input), then read serial output.
// Default known input is 2.000 V.

constexpr uint8_t kAdcPin = 7;
constexpr uint16_t kAdcMax = 4095;
constexpr uint8_t kAdcResolutionBits = 12;
constexpr uint8_t kDiscardSamples = 8;
constexpr uint8_t kAvgSamples = 64;
constexpr float kAssumedVref = 3.300f;
constexpr float kKnownInputV = 2.000f;  // Change if you feed a different calibration voltage.

uint16_t readAdcAveraged(uint8_t pin) {
  for (uint8_t i = 0; i < kDiscardSamples; ++i) {
    (void)analogRead(pin);
    delayMicroseconds(250);
  }

  uint32_t sum = 0;
  for (uint8_t i = 0; i < kAvgSamples; ++i) {
    sum += analogRead(pin);
    delayMicroseconds(250);
  }
  return static_cast<uint16_t>(sum / kAvgSamples);
}

void setup() {
  Serial.begin(115200);
  delay(800);

  analogReadResolution(kAdcResolutionBits);
  analogSetAttenuation(ADC_11db);
  pinMode(kAdcPin, INPUT);

  Serial.println();
  Serial.println("ADC GPIO7 calibration mode");
  Serial.println("Make sure input <= 3.3V and GND is common.");
  Serial.print("Known input voltage: ");
  Serial.print(kKnownInputV, 3);
  Serial.println(" V");
}

void loop() {
  const uint16_t raw = readAdcAveraged(kAdcPin);
  const float vPinAssumed = (static_cast<float>(raw) / kAdcMax) * kAssumedVref;
  const float vrefEffective = (raw > 0) ? (kKnownInputV * kAdcMax / static_cast<float>(raw)) : NAN;

  Serial.print("RAW=");
  Serial.print(raw);
  Serial.print("  VPIN@3.3=");
  Serial.print(vPinAssumed, 4);
  Serial.print(" V  VREF_EFFECTIVE=");
  Serial.print(vrefEffective, 4);
  Serial.println(" V");

  delay(1000);
}

