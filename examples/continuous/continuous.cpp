#include <Arduino.h>
#include <Wire.h>

#include "LPS22.h"

// Pins (adjust according to hardware)
constexpr int8_t PIN_SDA = 8;
constexpr int8_t PIN_SCL = 9;

LPS22::Driver sensor(Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  Wire.begin(PIN_SDA, PIN_SCL);

  // Configuration set before begin() is applied by begin()
  sensor.setDataRate(LPS22::DataRate::Hz25);
  sensor.setLowPassFilter(LPS22::LowPassFilter::OdrDiv20); // lowest noise

  if (!sensor.begin()) {
    Serial.println("[LPS22] begin() failed");
    while (true)
      delay(1000);
  }

  Serial.printf("[LPS22] Continuous mode at %s\n", LPS22::toString(sensor.getDataRate()));
}

void loop() {
  // Non-blocking: only read when the chip has a new sample
  if (!sensor.dataReady()) return;

  LPS22::Measurement m;
  if (sensor.read(m)) Serial.printf("%.3f hPa  -  %.2f °C\n", m.pressure, m.temperature);
}
