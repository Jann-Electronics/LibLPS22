#include <Arduino.h>
#include <Wire.h>

#include "LPS22.h"

// Pins (adjust according to hardware)
constexpr int8_t PIN_SDA = 46;
constexpr int8_t PIN_SCL = 3;

// Known altitude of the place where the sketch starts (m). Used to compute the local sea-level
// pressure (QNH), so that later altitude readings are relative to real sea level
constexpr float START_ALTITUDE_M = 0.0f;

// Optional: pressure from a calibrated barometer (hPa) for one-point calibration. NAN to skip
constexpr float REFERENCE_PRESSURE_HPA = NAN;

LPS22::Driver sensor(Wire, LPS22::ADDRESS_HIGH);

float sea_level_hpa = LPS22::SEA_LEVEL_PRESSURE_HPA;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  Wire.begin(PIN_SDA, PIN_SCL);

  if (!sensor.begin()) {
    Serial.println("[LPS22] begin() failed");
    while (true)
      delay(1000);
  }

  if (!isnan(REFERENCE_PRESSURE_HPA)) {
    // The offset is volatile: store it and pass it to setPressureOffset() on the next boot
    if (sensor.calibrate(REFERENCE_PRESSURE_HPA))
      Serial.printf("[LPS22] Pressure offset: %.4f hPa\n", sensor.getPressureOffset());
    else
      Serial.println("[LPS22] calibrate() failed");
  }

  const float pressure = sensor.readPressure();
  sea_level_hpa        = LPS22::seaLevelPressure(pressure, START_ALTITUDE_M);
  Serial.printf("[LPS22] Sea-level pressure: %.2f hPa\n", sea_level_hpa);
}

void loop() {
  const float pressure = sensor.readPressure();
  const float altitude = LPS22::pressureToAltitude(pressure, sea_level_hpa);

  Serial.printf("Pressure: %.2f hPa  -  Altitude: %.2f m (standard atmosphere: %.2f m)\n",
    pressure,
    altitude,
    LPS22::pressureToAltitude(pressure));

  delay(1000);
}
