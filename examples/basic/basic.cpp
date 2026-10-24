#include <Arduino.h>
#include <Wire.h>

#include "LPS22.h"

// Pins (adjust according to hardware)
constexpr int8_t PIN_SDA = 8;
constexpr int8_t PIN_SCL = 9;

// SA0 pin to GND -> ADDRESS_LOW (0x5C), to VDD -> ADDRESS_HIGH (0x5D)
LPS22::Driver sensor(Wire, LPS22::ADDRESS_LOW);

// SPI alternative (4-wire): LPS22::Driver sensor(SPI, PIN_CS);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  // I2C bus must be initialized externally
  Wire.begin(PIN_SDA, PIN_SCL);

  if (!sensor.begin()) {
    Serial.println("[LPS22] begin() failed");
    while (true)
      delay(1000);
  }

  // begin() defaults to one-shot mode: each read triggers a conversion and waits for it
  Serial.println("[LPS22] Initialized");
}

void loop() {
  LPS22::Measurement m;

  if (sensor.read(m)) {
    Serial.printf("Pressure: %.2f hPa (%.3f psi)  -  Temperature: %.2f °C\n",
      m.pressure,
      LPS22::convertPressure(m.pressure, LPS22::Unit::Psi),
      m.temperature);
  } else {
    Serial.println("[LPS22] read failed");
  }

  delay(1000);
}
