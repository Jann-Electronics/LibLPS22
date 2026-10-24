#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

namespace LPS22 {

/* ---------------------------------------------------------------------------------------------- */
/*                                            Constants                                           */
/* ---------------------------------------------------------------------------------------------- */
constexpr uint8_t ADDRESS_LOW  = 0x5C; // SA0 pin = LOW
constexpr uint8_t ADDRESS_HIGH = 0x5D; // SA0 pin = HIGH

constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25f; // standard atmosphere

/* ---------------------------------------------------------------------------------------------- */
/*                                              Enums                                             */
/* ---------------------------------------------------------------------------------------------- */

// Output data rate (CTRL_REG1 ODR bits). OneShot keeps the chip in power-down between reads
enum class DataRate : uint8_t {
  OneShot = 0, // each read triggers a conversion and waits for it
  Hz1     = 1,
  Hz10    = 2,
  Hz25    = 3,
  Hz50    = 4,
  Hz75    = 5,
};

// Extra low-pass filter on pressure data, only active at continuous data rates
enum class LowPassFilter : uint8_t {
  Off      = 0, // bandwidth ODR/2
  OdrDiv9  = 2, // bandwidth ODR/9
  OdrDiv20 = 3, // bandwidth ODR/20 (lowest noise)
};

// Reference used for differential pressure (INTERRUPT_CFG AUTOZERO / AUTORIFP). In both modes the
// reference (REF_P) is captured by the next conversion after the mode is selected
enum class ReferenceMode : uint8_t {
  Absolute, // no reference: readings are absolute pressure
  AutoRifp, // readings stay absolute; threshold interrupts compare (pressure - reference)
  AutoZero, // readings and threshold interrupts are (pressure - reference), can be negative
};

// Pressure threshold events (INTERRUPT_CFG PHE/PLE and INT_SOURCE PH/PL share this layout)
enum class PressureEvent : uint8_t {
  None = 0,
  High = 1, // differential pressure above +threshold
  Low  = 2, // differential pressure below -threshold
  Both = 3,
};

// Signal routed to the INT_DRDY pin (CTRL_REG3 INT_S bits)
enum class PinSource : uint8_t {
  DataSignals       = 0, // data-ready and FIFO flags selected with setDataSignals()
  PressureHigh      = 1,
  PressureLow       = 2,
  PressureHighOrLow = 3,
};

// Data signals on the INT_DRDY pin when PinSource::DataSignals is selected (CTRL_REG3 bits 5:2).
// Multiple flags can be combined with |
enum class DataSignal : uint8_t {
  None          = 0x00,
  DataReady     = 0x04,
  FifoOverrun   = 0x08,
  FifoWatermark = 0x10,
  FifoFull      = 0x20,
};

enum class PinPolarity : uint8_t {
  ActiveHigh = 0,
  ActiveLow  = 1,
};

enum class PinDrive : uint8_t {
  PushPull  = 0,
  OpenDrain = 1,
};

// FIFO mode (FIFO_CTRL F_MODE bits). The trigger-based modes switch on INT_SOURCE IA, so they need
// a threshold interrupt configured (latched interrupts recommended)
enum class FifoMode : uint8_t {
  Bypass         = 0, // FIFO disabled
  Fifo           = 1, // fills up and stops when full (or at the watermark with stop-on-watermark)
  Stream         = 2, // keeps the newest 32 samples
  StreamToFifo   = 3, // Stream until an interrupt occurs, then FIFO
  BypassToStream = 4, // Bypass until an interrupt occurs, then Stream
  DynamicStream  = 6, // like Stream, but the level always counts new samples only
  BypassToFifo   = 7, // Bypass until an interrupt occurs, then FIFO
};

enum class Unit : uint8_t {
  Hectopascal,
  Millibar,
  Kilopascal,
  Pascal,
  Psi,
  Atmosphere,
  MmHg,
  InHg,
};

/* ---------------------------------------------------------------------------------------------- */
/*                                 Bitmask operators for DataSignal                               */
/* ---------------------------------------------------------------------------------------------- */
constexpr DataSignal operator|(DataSignal a, DataSignal b) {
  return static_cast<DataSignal>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

constexpr DataSignal operator&(DataSignal a, DataSignal b) {
  return static_cast<DataSignal>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

inline DataSignal& operator|=(DataSignal& a, DataSignal b) { return a = a | b; }
inline DataSignal& operator&=(DataSignal& a, DataSignal b) { return a = a & b; }

constexpr bool any(DataSignal s) { return s != DataSignal::None; }
constexpr bool has(DataSignal s, DataSignal flag) { return (s & flag) == flag; }

constexpr bool any(PressureEvent e) { return e != PressureEvent::None; }

/* ---------------------------------------------------------------------------------------------- */
/*                                         Free helpers                                           */
/* ---------------------------------------------------------------------------------------------- */

// Converts a pressure in hPa to the given unit
float convertPressure(float pressure_hpa, Unit unit);

// Altitude in meters from the international barometric formula. sea_level_hpa is the local
// pressure reduced to sea level (QNH); the default gives the standard-atmosphere altitude
float pressureToAltitude(float pressure_hpa, float sea_level_hpa = SEA_LEVEL_PRESSURE_HPA);

// Sea-level pressure (QNH) in hPa from a pressure measured at a known altitude in meters
float seaLevelPressure(float pressure_hpa, float altitude_m);

const char* toString(DataRate rate);
const char* toString(FifoMode mode);

/* ---------------------------------------------------------------------------------------------- */
/*                                            Structs                                             */
/* ---------------------------------------------------------------------------------------------- */

// Results of a single conversion, as returned by Driver::read()
struct Measurement {
  float pressure;    // hPa (relative to the reference in ReferenceMode::AutoZero)
  float temperature; // °C
};

struct FifoStatus {
  uint8_t level;  // unread samples (0 to 32)
  bool watermark; // level >= watermark (always false with a watermark of 0)
  bool overrun;   // FIFO full and at least one sample overwritten
};

/* ---------------------------------------------------------------------------------------------- */
/*                                             Driver                                             */
/* ---------------------------------------------------------------------------------------------- */

// Configuration setters can be called before begin(): the values are stored and begin() applies
// them all at once. After begin(), each setter is applied to the chip immediately. Calling begin()
// again resets the chip and re-applies the whole configuration (e.g. after it lost power).
//
// Read behaviour depends on the data rate:
//   - OneShot:    triggers a conversion and waits for it (a few ms). If triggerOneShot() was called
//                 before, waits for that conversion instead of starting a new one.
//   - Continuous: returns the latest sample. After begin() or a configuration change, the first
//                 read waits for the first sample (up to one period at the selected rate).
// With the FIFO enabled, every read pops the oldest sample from the FIFO.
// Reads return false / NAN on bus errors or timeouts.
class Driver {
  public:
  // I2C constructor. The bus must be initialized externally (Wire.begin()).
  explicit Driver(TwoWire& wire = Wire, uint8_t address = ADDRESS_LOW);

  // SPI constructor (4-wire). The bus must be initialized externally (SPI.begin()).
  // spi_freq_hz: SPI clock frequency in Hz (default 1 MHz, clamped to the 10 MHz maximum)
  Driver(SPIClass& spi, int8_t cs_pin, uint32_t spi_freq_hz = 1000000);

  // Checks WHO_AM_I, resets the chip and applies the configuration. Defaults:
  //   - OneShot data rate, low-pass filter off, low-noise mode (low-current mode off)
  //   - Absolute readings, no pressure offset, FIFO in Bypass mode
  //   - Threshold interrupts off, INT_DRDY pin push-pull, active high, no signals routed
  // Returns false if the chip does not respond or is not an LPS22HB
  bool begin();

  // Puts the chip in power-down. Call begin() to use it again
  void end();

  bool isInitialized() const { return _initialized; }

  // ----- Configuration -----

  void setDataRate(DataRate rate);
  DataRate getDataRate() const;

  // Only effective at continuous data rates
  void setLowPassFilter(LowPassFilter filter);
  LowPassFilter getLowPassFilter() const;

  // Low-current mode: about 3 µA instead of 12 µA at 1 Hz, with higher noise
  void setLowCurrentMode(bool enabled);
  bool isLowCurrentMode() const;

  // ----- Calibration -----

  // One-point calibration offset in hPa (±2048 hPa, 1/16 hPa resolution), subtracted by the chip
  // from every absolute reading. Volatile: begin() writes it again. Returns false if NAN
  bool setPressureOffset(float offset_hpa);
  float getPressureOffset() const;

  // Measures the current pressure and adjusts the offset so that it reads reference_hpa (e.g. from
  // a calibrated barometer). Requires begin() and ReferenceMode::Absolute. Returns false on error
  bool calibrate(float reference_hpa);

  // ----- Reference pressure -----

  void setReferenceMode(ReferenceMode mode);
  ReferenceMode getReferenceMode() const;

  // Captures a new reference with the next conversion (no effect in ReferenceMode::Absolute)
  void captureReference();

  // Reference pressure in hPa currently stored in the chip (NAN on error or before begin())
  float readReferencePressure();

  // ----- Threshold interrupts -----

  // Differential pressure threshold in hPa (0 to 4095.9375, 1/16 hPa resolution). Events compare
  // (pressure - reference), so a reference mode other than Absolute is needed. Returns false if
  // the value is NAN or out of range
  bool setPressureThreshold(float threshold_hpa);
  float getPressureThreshold() const;

  // Selects which events are generated. None disables threshold interrupts
  void setPressureEvents(PressureEvent events);
  PressureEvent getPressureEvents() const;

  // Latched: events stay in INT_SOURCE (and on the pin) until readInterruptSource() is called
  void setInterruptLatched(bool latched);
  bool isInterruptLatched() const;

  // Reads INT_SOURCE: the events that occurred. Clears latched events
  PressureEvent readInterruptSource();

  // ----- INT_DRDY pin -----

  void setPinSource(PinSource source);
  PinSource getPinSource() const;

  void setDataSignals(DataSignal signals);
  DataSignal getDataSignals() const;

  void setPinPolarity(PinPolarity polarity);
  PinPolarity getPinPolarity() const;

  void setPinDrive(PinDrive drive);
  PinDrive getPinDrive() const;

  // ----- FIFO -----

  // Selecting a mode empties the FIFO. The datasheet recommends discarding the first sample after
  // switching into or out of a FIFO mode
  void setFifoMode(FifoMode mode);
  FifoMode getFifoMode() const;

  // Watermark level (0 to 31, clamped). With stop-on-watermark, the FIFO depth is limited to
  // watermark + 1 samples
  void setFifoWatermark(uint8_t level, bool stop_on_watermark = false);
  uint8_t getFifoWatermark() const;
  bool isFifoStopOnWatermark() const;

  // Empties the FIFO and restarts the current FIFO mode
  void resetFifo();

  FifoStatus readFifoStatus();

  // Reads up to max_samples samples from the FIFO, oldest first. Returns the number of samples read
  uint8_t readFifo(Measurement* buffer, uint8_t max_samples);

  // ----- Readings -----

  // Pressure and temperature from the same conversion. Returns false (fields set to NAN) on error
  bool read(Measurement& measurement);

  float readPressure(Unit unit = Unit::Hectopascal);

  // Temperature in °C
  float readTemperature();

  // Altitude in meters for the given sea-level pressure. NAN in ReferenceMode::AutoZero
  float readAltitude(float sea_level_hpa = SEA_LEVEL_PRESSURE_HPA);

  // ----- Manual one-shot conversion -----

  // Starts a one-shot conversion. Returns false at continuous data rates or on error
  bool triggerOneShot();

  // True when a new sample is available (both pressure and temperature)
  bool dataReady();

  private:
  // ----- Bus -----
  TwoWire* _wire;
  SPIClass* _spi;
  SPISettings _spi_settings;
  uint8_t _address;
  int8_t _cs_pin;

  // ----- Configuration (applied by begin()) -----
  DataRate _data_rate;
  LowPassFilter _low_pass;
  bool _low_current;
  int16_t _offset_raw;
  ReferenceMode _reference_mode;
  uint16_t _threshold_raw;
  PressureEvent _events;
  bool _latched;
  PinSource _pin_source;
  DataSignal _data_signals;
  PinPolarity _pin_polarity;
  PinDrive _pin_drive;
  FifoMode _fifo_mode;
  uint8_t _fifo_watermark;
  bool _fifo_stop_on_watermark;

  // ----- State -----
  bool _initialized;
  bool _has_result;       // a sample arrived since the last (re)configuration (continuous rates)
  bool _one_shot_pending; // a one-shot conversion was triggered and not read yet

  // ----- Register values -----
  uint8_t _ctrlReg1() const;
  uint8_t _ctrlReg2() const;
  uint8_t _ctrlReg3() const;
  uint8_t _fifoCtrl() const;
  uint8_t _interruptCfg() const;
  bool _burstReads() const;

  // ----- Configuration helpers -----
  bool _softReset();
  void _applyDataRate();
  void _applyFifo();
  void _applyReference();
  void _writeInterruptCfg();
  void _writeOffset();
  void _writeThreshold();

  // ----- Conversion helpers -----
  void _resetSampleState();
  bool _waitOneShot();
  bool _waitFirstSample();
  bool _acquire();
  bool _readSample(Measurement& measurement);

  // ----- Register access -----
  bool _readRegister(uint8_t reg, uint8_t& value);
  bool _writeRegister(uint8_t reg, uint8_t value);
  // Consecutive registers, one byte per transfer (works whatever the IF_ADD_INC setting)
  bool _readRegisters(uint8_t reg, uint8_t* buffer, uint8_t n);
  bool _writeRegisters(uint8_t reg, const uint8_t* data, uint8_t n);
  // Multi-byte transfer, requires IF_ADD_INC = 1
  bool _readBurst(uint8_t reg, uint8_t* buffer, uint8_t n);
};

} // namespace LPS22
