#include <Arduino.h>
#include <Wire.h>

#include "LPS22.h"

// Pins (adjust according to hardware)
constexpr int8_t PIN_SDA = 8;
constexpr int8_t PIN_SCL = 9;
constexpr int8_t PIN_INT = 4; // INT_DRDY

// Samples collected before the host wakes up to read them (1 to 31)
constexpr uint8_t WATERMARK = 25;

LPS22::Driver sensor(Wire);

volatile bool watermark_flag = false;

void IRAM_ATTR onWatermark() { watermark_flag = true; }

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  Wire.begin(PIN_SDA, PIN_SCL);

  // The chip buffers samples at 10 Hz: the host only needs to read every ~2.5 s
  sensor.setDataRate(LPS22::DataRate::Hz10);
  sensor.setFifoMode(LPS22::FifoMode::DynamicStream);
  sensor.setFifoWatermark(WATERMARK);

  // FIFO watermark flag on the INT_DRDY pin
  sensor.setPinSource(LPS22::PinSource::DataSignals);
  sensor.setDataSignals(LPS22::DataSignal::FifoWatermark);

  if (!sensor.begin()) {
    Serial.println("[LPS22] begin() failed");
    while (true)
      delay(1000);
  }

  pinMode(PIN_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), onWatermark, RISING);

  Serial.printf("[LPS22] FIFO in %s mode, watermark %u\n",
    LPS22::toString(sensor.getFifoMode()),
    WATERMARK);
}

void loop() {
  // The pin level also counts, in case the watermark was reached before attachInterrupt()
  if (!watermark_flag && digitalRead(PIN_INT) == LOW) return;
  watermark_flag = false;

  const LPS22::FifoStatus status = sensor.readFifoStatus();
  if (status.overrun) Serial.println("[LPS22] FIFO overrun: samples were lost");

  LPS22::Measurement samples[32];
  const uint8_t n = sensor.readFifo(samples, 32);

  float sum = 0.0f;
  for (uint8_t i = 0; i < n; ++i)
    sum += samples[i].pressure;

  if (n > 0)
    Serial.printf("%u samples  -  mean %.3f hPa  -  last %.2f °C\n",
      n,
      sum / n,
      samples[n - 1].temperature);
}
