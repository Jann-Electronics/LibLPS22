#include <Arduino.h>
#include <Wire.h>

#include "LPS22.h"

// Pins (adjust according to hardware)
constexpr int8_t PIN_SDA = 8;
constexpr int8_t PIN_SCL = 9;
constexpr int8_t PIN_INT = 4; // INT_DRDY

// Pressure change (hPa) that triggers an event, relative to the pressure captured at startup.
// ~0.12 hPa is roughly 1 m of altitude near sea level
constexpr float THRESHOLD_HPA = 0.5f;

LPS22::Driver sensor(Wire);

volatile bool interrupt_flag = false;

void IRAM_ATTR onInterrupt() { interrupt_flag = true; }

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  Wire.begin(PIN_SDA, PIN_SCL);

  // Threshold events compare (pressure - reference). AutoRifp captures the reference with the first
  // conversion and keeps readings absolute
  sensor.setDataRate(LPS22::DataRate::Hz10);
  sensor.setReferenceMode(LPS22::ReferenceMode::AutoRifp);
  sensor.setPressureThreshold(THRESHOLD_HPA);
  sensor.setPressureEvents(LPS22::PressureEvent::Both);
  sensor.setInterruptLatched(true); // the pin stays active until readInterruptSource()

  // Route both events to the INT_DRDY pin
  sensor.setPinSource(LPS22::PinSource::PressureHighOrLow);
  sensor.setPinPolarity(LPS22::PinPolarity::ActiveHigh);
  sensor.setPinDrive(LPS22::PinDrive::PushPull);

  if (!sensor.begin()) {
    Serial.println("[LPS22] begin() failed");
    while (true)
      delay(1000);
  }

  pinMode(PIN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), onInterrupt, RISING);

  delay(200); // let the first conversions capture the reference
  Serial.printf("[LPS22] Reference: %.2f hPa, threshold: ±%.2f hPa\n",
    sensor.readReferencePressure(),
    sensor.getPressureThreshold());
}

void loop() {
  // The pin level also counts, in case an event latched before attachInterrupt()
  if (!interrupt_flag && digitalRead(PIN_INT) == LOW) return;
  interrupt_flag = false;

  // Reading INT_SOURCE releases the latched interrupt
  const LPS22::PressureEvent event = sensor.readInterruptSource();
  const float pressure             = sensor.readPressure();

  if (event == LPS22::PressureEvent::High)
    Serial.printf("Pressure rose above the threshold: %.2f hPa\n", pressure);
  else if (event == LPS22::PressureEvent::Low)
    Serial.printf("Pressure fell below the threshold: %.2f hPa\n", pressure);
  else if (LPS22::any(event))
    Serial.printf("High and low events: %.2f hPa\n", pressure);
}
