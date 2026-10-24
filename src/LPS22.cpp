#include "LPS22.h"

#include <math.h>

namespace LPS22 {

/* ---------------------------------------------------------------------------------------------- */
/*                                       Register addresses                                       */
/* ---------------------------------------------------------------------------------------------- */
namespace Reg {
constexpr uint8_t INTERRUPT_CFG = 0x0B;
constexpr uint8_t THS_P_L       = 0x0C; // threshold, 16-bit unsigned, 16 LSB/hPa
constexpr uint8_t WHO_AM_I      = 0x0F;
constexpr uint8_t CTRL_REG1     = 0x10;
constexpr uint8_t CTRL_REG2     = 0x11;
constexpr uint8_t CTRL_REG3     = 0x12;
constexpr uint8_t FIFO_CTRL     = 0x14;
constexpr uint8_t REF_P_XL      = 0x15; // reference pressure, 24-bit two's complement
constexpr uint8_t RPDS_L        = 0x18; // pressure offset, 16-bit two's complement
constexpr uint8_t RES_CONF      = 0x1A;
constexpr uint8_t INT_SOURCE    = 0x25;
constexpr uint8_t FIFO_STATUS   = 0x26;
constexpr uint8_t STATUS        = 0x27;
constexpr uint8_t PRESS_OUT_XL  = 0x28;
constexpr uint8_t PRESS_OUT_L   = 0x29;
constexpr uint8_t PRESS_OUT_H   = 0x2A;
constexpr uint8_t TEMP_OUT_L    = 0x2B;
constexpr uint8_t TEMP_OUT_H    = 0x2C;
constexpr uint8_t LPFP_RES      = 0x33; // reading it resets the low-pass filter
} // namespace Reg

namespace InterruptCfgBit {
constexpr uint8_t AUTORIFP  = 0x80; // self-clears once the reference is captured
constexpr uint8_t RESET_ARP = 0x40;
constexpr uint8_t AUTOZERO  = 0x20; // self-clears once the reference is captured
constexpr uint8_t RESET_AZ  = 0x10;
constexpr uint8_t DIFF_EN   = 0x08;
constexpr uint8_t LIR       = 0x04;
} // namespace InterruptCfgBit

namespace CtrlReg1Bit {
constexpr uint8_t ODR_SHIFT = 4;
constexpr uint8_t ODR_MASK  = 0x70;
constexpr uint8_t LPF_SHIFT = 2; // EN_LPFP and LPFP_CFG, bits 3:2
constexpr uint8_t BDU       = 0x02;
} // namespace CtrlReg1Bit

namespace CtrlReg2Bit {
constexpr uint8_t FIFO_EN     = 0x40;
constexpr uint8_t STOP_ON_FTH = 0x20;
constexpr uint8_t IF_ADD_INC  = 0x10;
constexpr uint8_t I2C_DIS     = 0x08;
constexpr uint8_t SWRESET     = 0x04; // self-clears
constexpr uint8_t ONE_SHOT    = 0x01; // self-clears when the conversion is done
} // namespace CtrlReg2Bit

namespace CtrlReg3Bit {
constexpr uint8_t INT_H_L = 0x80;
constexpr uint8_t PP_OD   = 0x40;
} // namespace CtrlReg3Bit

namespace ResConfBit {
constexpr uint8_t RESERVED = 0x02; // must keep its factory value
constexpr uint8_t LC_EN    = 0x01; // only writable in power-down
} // namespace ResConfBit

namespace StatusBit {
constexpr uint8_t T_DA = 0x02;
constexpr uint8_t P_DA = 0x01;
} // namespace StatusBit

namespace FifoStatusBit {
constexpr uint8_t FTH   = 0x80;
constexpr uint8_t OVR   = 0x40;
constexpr uint8_t LEVEL = 0x3F;
} // namespace FifoStatusBit

namespace {
constexpr uint8_t WHO_AM_I_VALUE   = 0xB1;
constexpr uint8_t SPI_READ         = 0x80;
constexpr uint32_t SPI_MAX_FREQ_HZ = 10000000;

constexpr float PRESSURE_LSB_PER_HPA  = 4096.0f;
constexpr float TEMPERATURE_LSB_PER_C = 100.0f;
constexpr float OFFSET_LSB_PER_HPA    = 16.0f; // PRESS_OUT = measured - 256 x RPDS
constexpr float THRESHOLD_LSB_PER_HPA = 16.0f;

constexpr uint8_t FIFO_WATERMARK_MAX  = 31;
constexpr uint8_t SAMPLE_SIZE         = 5; // PRESS_OUT_XL to TEMP_OUT_H
constexpr uint8_t CALIBRATION_SAMPLES = 8;

constexpr uint32_t POLL_INTERVAL_MS    = 1;
constexpr uint32_t RESET_TIMEOUT_MS    = 10;
constexpr uint32_t ONE_SHOT_TIMEOUT_MS = 100;
constexpr uint32_t SAMPLE_MARGIN_MS    = 50;

// Sample period by DataRate (ms), 0 for OneShot
constexpr uint16_t PERIOD_MS[] = {0, 1000, 100, 40, 20, 14};

// Rounds value * scale to the nearest integer, clamped to [min, max]
int32_t toFixed(float value, float scale, int32_t min, int32_t max) {
  const float scaled = roundf(value * scale);
  if (scaled <= static_cast<float>(min)) return min;
  if (scaled >= static_cast<float>(max)) return max;
  return static_cast<int32_t>(scaled);
}

// 24-bit two's complement value, little-endian (XL, L, H)
int32_t decode24(const uint8_t* b) {
  int32_t raw = static_cast<int32_t>(b[0]) | (static_cast<int32_t>(b[1]) << 8) |
                (static_cast<int32_t>(b[2]) << 16);
  if (raw & 0x800000) raw -= 0x1000000;
  return raw;
}

// 16-bit two's complement value, little-endian (L, H)
int16_t decode16(const uint8_t* b) {
  return static_cast<int16_t>(static_cast<uint16_t>(b[0]) | (static_cast<uint16_t>(b[1]) << 8));
}
} // namespace

/* ---------------------------------------------------------------------------------------------- */
/*                                          Free helpers                                          */
/* ---------------------------------------------------------------------------------------------- */
float convertPressure(float pressure_hpa, Unit unit) {
  switch (unit) {
    case Unit::Hectopascal:
    case Unit::Millibar: return pressure_hpa;
    case Unit::Kilopascal: return pressure_hpa * 0.1f;
    case Unit::Pascal: return pressure_hpa * 100.0f;
    case Unit::Psi: return pressure_hpa * 0.0145037738f;
    case Unit::Atmosphere: return pressure_hpa / SEA_LEVEL_PRESSURE_HPA;
    case Unit::MmHg: return pressure_hpa * 0.750061683f;
    case Unit::InHg: return pressure_hpa * 0.0295299831f;
    default: return NAN;
  }
}

float pressureToAltitude(float pressure_hpa, float sea_level_hpa) {
  if (!(pressure_hpa > 0.0f) || !(sea_level_hpa > 0.0f)) return NAN;
  return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa, 1.0f / 5.255f));
}

float seaLevelPressure(float pressure_hpa, float altitude_m) {
  const float ratio = 1.0f - altitude_m / 44330.0f;
  if (!(pressure_hpa > 0.0f) || !(ratio > 0.0f)) return NAN;
  return pressure_hpa / powf(ratio, 5.255f);
}

const char* toString(DataRate rate) {
  switch (rate) {
    case DataRate::OneShot: return "OneShot";
    case DataRate::Hz1: return "1 Hz";
    case DataRate::Hz10: return "10 Hz";
    case DataRate::Hz25: return "25 Hz";
    case DataRate::Hz50: return "50 Hz";
    case DataRate::Hz75: return "75 Hz";
    default: return "Unknown";
  }
}

const char* toString(FifoMode mode) {
  switch (mode) {
    case FifoMode::Bypass: return "Bypass";
    case FifoMode::Fifo: return "FIFO";
    case FifoMode::Stream: return "Stream";
    case FifoMode::StreamToFifo: return "Stream-to-FIFO";
    case FifoMode::BypassToStream: return "Bypass-to-Stream";
    case FifoMode::DynamicStream: return "Dynamic-Stream";
    case FifoMode::BypassToFifo: return "Bypass-to-FIFO";
    default: return "Unknown";
  }
}

/* ---------------------------------------------------------------------------------------------- */
/*                                             Driver                                             */
/* ---------------------------------------------------------------------------------------------- */
Driver::Driver(TwoWire& wire, uint8_t address)
    : _wire(&wire)
    , _spi(nullptr)
    , _spi_settings()
    , _address(address)
    , _cs_pin(-1)
    , _data_rate(DataRate::OneShot)
    , _low_pass(LowPassFilter::Off)
    , _low_current(false)
    , _offset_raw(0)
    , _reference_mode(ReferenceMode::Absolute)
    , _threshold_raw(0)
    , _events(PressureEvent::None)
    , _latched(false)
    , _pin_source(PinSource::DataSignals)
    , _data_signals(DataSignal::None)
    , _pin_polarity(PinPolarity::ActiveHigh)
    , _pin_drive(PinDrive::PushPull)
    , _fifo_mode(FifoMode::Bypass)
    , _fifo_watermark(0)
    , _fifo_stop_on_watermark(false)
    , _initialized(false)
    , _has_result(false)
    , _one_shot_pending(false) {}

Driver::Driver(SPIClass& spi, int8_t cs_pin, uint32_t spi_freq_hz)
    : Driver() {
  _wire = nullptr;
  _spi  = &spi;
  _spi_settings =
    SPISettings(spi_freq_hz > SPI_MAX_FREQ_HZ ? SPI_MAX_FREQ_HZ : spi_freq_hz, MSBFIRST, SPI_MODE3);
  _cs_pin = cs_pin;
}

bool Driver::begin() {
  _initialized = false;

  if (_spi) {
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);
  }

  uint8_t who_am_i = 0;
  if (!_readRegister(Reg::WHO_AM_I, who_am_i) || who_am_i != WHO_AM_I_VALUE) return false;
  if (!_softReset()) return false;

  _initialized = true;
  _writeRegister(Reg::CTRL_REG2, _ctrlReg2());
  _writeRegister(Reg::CTRL_REG3, _ctrlReg3());
  _writeOffset();
  _writeThreshold();
  _applyFifo();
  _applyReference();
  _applyDataRate();
  return true;
}

void Driver::end() {
  if (_initialized) _writeRegister(Reg::CTRL_REG1, _ctrlReg1() & ~CtrlReg1Bit::ODR_MASK);
  _initialized      = false;
  _one_shot_pending = false;
}

/* ----------------------------------------- Configuration -------------------------------------- */
void Driver::setDataRate(DataRate rate) {
  _data_rate = rate;
  if (_initialized) _applyDataRate();
}

DataRate Driver::getDataRate() const { return _data_rate; }

void Driver::setLowPassFilter(LowPassFilter filter) {
  _low_pass = filter;
  if (_initialized) _applyDataRate();
}

LowPassFilter Driver::getLowPassFilter() const { return _low_pass; }

void Driver::setLowCurrentMode(bool enabled) {
  _low_current = enabled;
  if (_initialized) _applyDataRate();
}

bool Driver::isLowCurrentMode() const { return _low_current; }

/* ------------------------------------------ Calibration --------------------------------------- */
bool Driver::setPressureOffset(float offset_hpa) {
  if (isnan(offset_hpa)) return false;

  _offset_raw = static_cast<int16_t>(toFixed(offset_hpa, OFFSET_LSB_PER_HPA, INT16_MIN, INT16_MAX));

  if (_initialized) {
    _writeOffset();
    _resetSampleState();
  }
  return true;
}

float Driver::getPressureOffset() const { return _offset_raw / OFFSET_LSB_PER_HPA; }

bool Driver::calibrate(float reference_hpa) {
  if (!_initialized || isnan(reference_hpa)) return false;
  if (_reference_mode != ReferenceMode::Absolute || _fifo_mode != FifoMode::Bypass) return false;

  float sum = 0.0f;
  for (uint8_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
    Measurement m;
    if (!read(m)) return false;
    sum += m.pressure;
    _has_result = false; // continuous rates: wait for a new sample on the next read
  }

  // Readings already have the current offset subtracted
  const float measured = sum / CALIBRATION_SAMPLES;
  return setPressureOffset(getPressureOffset() + measured - reference_hpa);
}

/* --------------------------------------- Reference pressure ----------------------------------- */
void Driver::setReferenceMode(ReferenceMode mode) {
  _reference_mode = mode;
  if (_initialized) _applyReference();
}

ReferenceMode Driver::getReferenceMode() const { return _reference_mode; }

void Driver::captureReference() {
  if (_initialized && _reference_mode != ReferenceMode::Absolute) _applyReference();
}

float Driver::readReferencePressure() {
  uint8_t buf[3];
  if (!_initialized || !_readRegisters(Reg::REF_P_XL, buf, sizeof(buf))) return NAN;
  return decode24(buf) / PRESSURE_LSB_PER_HPA;
}

/* -------------------------------------- Threshold interrupts ---------------------------------- */
bool Driver::setPressureThreshold(float threshold_hpa) {
  if (isnan(threshold_hpa) || threshold_hpa < 0.0f) return false;
  if (threshold_hpa * THRESHOLD_LSB_PER_HPA > UINT16_MAX) return false;

  _threshold_raw =
    static_cast<uint16_t>(toFixed(threshold_hpa, THRESHOLD_LSB_PER_HPA, 0, UINT16_MAX));

  if (_initialized) _writeThreshold();
  return true;
}

float Driver::getPressureThreshold() const { return _threshold_raw / THRESHOLD_LSB_PER_HPA; }

void Driver::setPressureEvents(PressureEvent events) {
  _events = events;
  if (_initialized) _writeInterruptCfg();
}

PressureEvent Driver::getPressureEvents() const { return _events; }

void Driver::setInterruptLatched(bool latched) {
  _latched = latched;
  if (_initialized) _writeInterruptCfg();
}

bool Driver::isInterruptLatched() const { return _latched; }

PressureEvent Driver::readInterruptSource() {
  uint8_t source = 0;
  if (!_initialized || !_readRegister(Reg::INT_SOURCE, source)) return PressureEvent::None;
  return static_cast<PressureEvent>(source & 0x03);
}

/* ------------------------------------------- INT_DRDY pin ------------------------------------- */
void Driver::setPinSource(PinSource source) {
  _pin_source = source;
  if (_initialized) _writeRegister(Reg::CTRL_REG3, _ctrlReg3());
}

PinSource Driver::getPinSource() const { return _pin_source; }

void Driver::setDataSignals(DataSignal signals) {
  _data_signals = signals;
  if (_initialized) _writeRegister(Reg::CTRL_REG3, _ctrlReg3());
}

DataSignal Driver::getDataSignals() const { return _data_signals; }

void Driver::setPinPolarity(PinPolarity polarity) {
  _pin_polarity = polarity;
  if (_initialized) _writeRegister(Reg::CTRL_REG3, _ctrlReg3());
}

PinPolarity Driver::getPinPolarity() const { return _pin_polarity; }

void Driver::setPinDrive(PinDrive drive) {
  _pin_drive = drive;
  if (_initialized) _writeRegister(Reg::CTRL_REG3, _ctrlReg3());
}

PinDrive Driver::getPinDrive() const { return _pin_drive; }

/* ---------------------------------------------- FIFO ------------------------------------------ */
void Driver::setFifoMode(FifoMode mode) {
  _fifo_mode = mode;
  if (_initialized) _applyFifo();
}

FifoMode Driver::getFifoMode() const { return _fifo_mode; }

void Driver::setFifoWatermark(uint8_t level, bool stop_on_watermark) {
  _fifo_watermark         = level > FIFO_WATERMARK_MAX ? FIFO_WATERMARK_MAX : level;
  _fifo_stop_on_watermark = stop_on_watermark;

  if (_initialized) {
    _writeRegister(Reg::CTRL_REG2, _ctrlReg2());
    _writeRegister(Reg::FIFO_CTRL, _fifoCtrl());
  }
}

uint8_t Driver::getFifoWatermark() const { return _fifo_watermark; }

bool Driver::isFifoStopOnWatermark() const { return _fifo_stop_on_watermark; }

void Driver::resetFifo() {
  if (_initialized) _applyFifo();
}

FifoStatus Driver::readFifoStatus() {
  uint8_t status = 0;
  if (!_initialized || !_readRegister(Reg::FIFO_STATUS, status)) return FifoStatus{0, false, false};

  return FifoStatus{static_cast<uint8_t>(status & FifoStatusBit::LEVEL),
    (status & FifoStatusBit::FTH) != 0,
    (status & FifoStatusBit::OVR) != 0};
}

uint8_t Driver::readFifo(Measurement* buffer, uint8_t max_samples) {
  if (buffer == nullptr || max_samples == 0 || _fifo_mode == FifoMode::Bypass) return 0;

  const uint8_t level = readFifoStatus().level;
  const uint8_t n     = level < max_samples ? level : max_samples;

  uint8_t count = 0;
  while (count < n && _readSample(buffer[count]))
    ++count;
  return count;
}

/* ------------------------------------------- Readings ----------------------------------------- */
bool Driver::read(Measurement& measurement) {
  measurement = Measurement{NAN, NAN};
  if (!_acquire()) return false;
  if (_readSample(measurement)) return true;

  measurement = Measurement{NAN, NAN};
  return false;
}

float Driver::readPressure(Unit unit) {
  Measurement m;
  if (!read(m)) return NAN;
  return convertPressure(m.pressure, unit);
}

float Driver::readTemperature() {
  Measurement m;
  if (!read(m)) return NAN;
  return m.temperature;
}

float Driver::readAltitude(float sea_level_hpa) {
  if (_reference_mode == ReferenceMode::AutoZero) return NAN;
  return pressureToAltitude(readPressure(), sea_level_hpa);
}

/* ---------------------------------------- One-shot control ------------------------------------ */
bool Driver::triggerOneShot() {
  if (!_initialized || _data_rate != DataRate::OneShot) return false;

  uint8_t ctrl2 = 0;
  if (!_readRegister(Reg::CTRL_REG2, ctrl2)) return false;

  // Already running: the pending conversion is the one to wait for
  if (!(ctrl2 & CtrlReg2Bit::ONE_SHOT))
    if (!_writeRegister(Reg::CTRL_REG2, _ctrlReg2() | CtrlReg2Bit::ONE_SHOT)) return false;

  _one_shot_pending = true;
  return true;
}

bool Driver::dataReady() {
  if (!_initialized) return false;
  if (_fifo_mode != FifoMode::Bypass) return readFifoStatus().level > 0;

  uint8_t status = 0;
  if (!_readRegister(Reg::STATUS, status)) return false;

  constexpr uint8_t both = StatusBit::P_DA | StatusBit::T_DA;
  return (status & both) == both;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                        Register values                                         */
/* ---------------------------------------------------------------------------------------------- */

// The datasheet forbids BDU together with multi-byte I2C reads. Outside FIFO modes the driver reads
// byte by byte with BDU on; with the FIFO enabled it reads each sample in one burst with BDU off
bool Driver::_burstReads() const { return _fifo_mode != FifoMode::Bypass; }

uint8_t Driver::_ctrlReg1() const {
  uint8_t reg = (static_cast<uint8_t>(_data_rate) << CtrlReg1Bit::ODR_SHIFT) |
                (static_cast<uint8_t>(_low_pass) << CtrlReg1Bit::LPF_SHIFT);
  if (!_burstReads()) reg |= CtrlReg1Bit::BDU;
  return reg;
}

uint8_t Driver::_ctrlReg2() const {
  uint8_t reg = 0;
  if (_fifo_mode != FifoMode::Bypass) reg |= CtrlReg2Bit::FIFO_EN;
  if (_fifo_stop_on_watermark) reg |= CtrlReg2Bit::STOP_ON_FTH;
  if (_burstReads()) reg |= CtrlReg2Bit::IF_ADD_INC;
  if (_spi) reg |= CtrlReg2Bit::I2C_DIS; // keeps SPI traffic from being decoded as I2C
  return reg;
}

uint8_t Driver::_ctrlReg3() const {
  uint8_t reg = static_cast<uint8_t>(_data_signals) | static_cast<uint8_t>(_pin_source);
  if (_pin_polarity == PinPolarity::ActiveLow) reg |= CtrlReg3Bit::INT_H_L;
  if (_pin_drive == PinDrive::OpenDrain) reg |= CtrlReg3Bit::PP_OD;
  return reg;
}

uint8_t Driver::_fifoCtrl() const {
  return (static_cast<uint8_t>(_fifo_mode) << 5) | _fifo_watermark;
}

// INTERRUPT_CFG without the reference bits
uint8_t Driver::_interruptCfg() const {
  uint8_t reg = static_cast<uint8_t>(_events); // PHE and PLE
  if (any(_events)) reg |= InterruptCfgBit::DIFF_EN;
  if (_latched) reg |= InterruptCfgBit::LIR;
  return reg;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                     Configuration helpers                                      */
/* ---------------------------------------------------------------------------------------------- */

// Restores the default register values. RPDS and RES_CONF are not affected and are written later
bool Driver::_softReset() {
  uint8_t reset = CtrlReg2Bit::IF_ADD_INC | CtrlReg2Bit::SWRESET;
  if (_spi) reset |= CtrlReg2Bit::I2C_DIS;
  if (!_writeRegister(Reg::CTRL_REG2, reset)) return false;

  // Bus errors while the reset runs are retried until the timeout
  const uint32_t start = millis();
  do {
    delay(POLL_INTERVAL_MS);
    uint8_t ctrl2 = 0;
    if (_readRegister(Reg::CTRL_REG2, ctrl2) && !(ctrl2 & CtrlReg2Bit::SWRESET)) return true;
  } while (millis() - start <= RESET_TIMEOUT_MS);
  return false;
}

// Writes the data rate, filter and low-current mode. LC_EN may only change in power-down, so the
// chip is stopped first
void Driver::_applyDataRate() {
  if (_one_shot_pending) _waitOneShot();

  _writeRegister(Reg::CTRL_REG1, _ctrlReg1() & ~CtrlReg1Bit::ODR_MASK);

  uint8_t res_conf = 0;
  if (_readRegister(Reg::RES_CONF, res_conf)) {
    res_conf &= ResConfBit::RESERVED;
    if (_low_current) res_conf |= ResConfBit::LC_EN;
    _writeRegister(Reg::RES_CONF, res_conf);
  }

  _writeRegister(Reg::CTRL_REG1, _ctrlReg1());

  // Skips the filter transient caused by the previous configuration
  uint8_t dummy;
  if (_low_pass != LowPassFilter::Off) _readRegister(Reg::LPFP_RES, dummy);

  _resetSampleState();
}

// Writes the FIFO configuration. Passing through Bypass mode empties the FIFO
void Driver::_applyFifo() {
  _writeRegister(Reg::FIFO_CTRL, _fifo_watermark);
  _writeRegister(Reg::CTRL_REG2, _ctrlReg2());
  _writeRegister(Reg::CTRL_REG1, _ctrlReg1()); // BDU depends on the FIFO mode
  if (_fifo_mode != FifoMode::Bypass) _writeRegister(Reg::FIFO_CTRL, _fifoCtrl());

  _resetSampleState();
}

// Leaves any previous reference mode (clearing REF_P), then requests a new capture
void Driver::_applyReference() {
  const uint8_t cfg = _interruptCfg();
  _writeRegister(Reg::INTERRUPT_CFG, cfg | InterruptCfgBit::RESET_ARP | InterruptCfgBit::RESET_AZ);

  switch (_reference_mode) {
    case ReferenceMode::AutoRifp:
      _writeRegister(Reg::INTERRUPT_CFG, cfg | InterruptCfgBit::AUTORIFP);
      break;
    case ReferenceMode::AutoZero:
      _writeRegister(Reg::INTERRUPT_CFG, cfg | InterruptCfgBit::AUTOZERO);
      break;
    default: _writeRegister(Reg::INTERRUPT_CFG, cfg); break;
  }

  _resetSampleState();
}

// Updates the event bits only. A reference capture that has not happened yet keeps its AUTORIFP /
// AUTOZERO bit; once captured the chip clears the bit, so rewriting it would capture again
void Driver::_writeInterruptCfg() {
  constexpr uint8_t capture_bits = InterruptCfgBit::AUTORIFP | InterruptCfgBit::AUTOZERO;

  uint8_t current = 0;
  _readRegister(Reg::INTERRUPT_CFG, current);
  _writeRegister(Reg::INTERRUPT_CFG, (current & capture_bits) | _interruptCfg());
}

void Driver::_writeOffset() {
  const uint16_t raw  = static_cast<uint16_t>(_offset_raw);
  const uint8_t buf[] = {static_cast<uint8_t>(raw), static_cast<uint8_t>(raw >> 8)};
  _writeRegisters(Reg::RPDS_L, buf, sizeof(buf));
}

void Driver::_writeThreshold() {
  const uint8_t buf[] = {static_cast<uint8_t>(_threshold_raw),
    static_cast<uint8_t>(_threshold_raw >> 8)};
  _writeRegisters(Reg::THS_P_L, buf, sizeof(buf));
}

/* ---------------------------------------------------------------------------------------------- */
/*                                       Conversion helpers                                       */
/* ---------------------------------------------------------------------------------------------- */

// After a configuration change, makes the next read wait for a sample taken with the new settings.
// Reading the output registers clears the data-available flags and releases BDU (skipped with the
// FIFO enabled, where it would pop a sample)
void Driver::_resetSampleState() {
  _has_result       = false;
  _one_shot_pending = false;

  if (!_burstReads()) {
    Measurement dummy;
    _readSample(dummy);
  }
}

bool Driver::_waitOneShot() {
  _one_shot_pending = false;

  const uint32_t start = millis();
  uint8_t ctrl2        = 0;
  while (true) {
    if (!_readRegister(Reg::CTRL_REG2, ctrl2)) return false;
    if (!(ctrl2 & CtrlReg2Bit::ONE_SHOT)) return true;
    if (millis() - start > ONE_SHOT_TIMEOUT_MS) return false;
    delay(POLL_INTERVAL_MS);
  }
}

bool Driver::_waitFirstSample() {
  if (_has_result) return true;

  const uint32_t timeout_ms = 2u * PERIOD_MS[static_cast<uint8_t>(_data_rate)] + SAMPLE_MARGIN_MS;
  const uint32_t start      = millis();
  constexpr uint8_t both    = StatusBit::P_DA | StatusBit::T_DA;

  uint8_t status = 0;
  while (true) {
    if (!_readRegister(Reg::STATUS, status)) return false;
    if ((status & both) == both) break;
    if (millis() - start > timeout_ms) return false;
    delay(POLL_INTERVAL_MS);
  }

  _has_result = true;
  return true;
}

// Makes sure the output registers hold a sample that can be read
bool Driver::_acquire() {
  if (!_initialized) return false;

  if (_fifo_mode != FifoMode::Bypass) return readFifoStatus().level > 0;

  if (_data_rate == DataRate::OneShot) {
    if (!_one_shot_pending && !triggerOneShot()) return false;
    return _waitOneShot();
  }

  return _waitFirstSample();
}

bool Driver::_readSample(Measurement& measurement) {
  uint8_t buf[SAMPLE_SIZE]; // PRESS_OUT_XL, L, H, TEMP_OUT_L, H

  if (_burstReads()) {
    if (!_readBurst(Reg::PRESS_OUT_XL, buf, SAMPLE_SIZE)) return false;
  } else {
    // Byte by byte, with PRESS_OUT_H last: with BDU on, reading it releases the output registers
    if (!_readRegisters(Reg::TEMP_OUT_L, &buf[3], 2) || !_readRegisters(Reg::PRESS_OUT_XL, buf, 3))
      return false;
  }

  measurement.pressure    = decode24(&buf[0]) / PRESSURE_LSB_PER_HPA;
  measurement.temperature = decode16(&buf[3]) / TEMPERATURE_LSB_PER_C;
  return true;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                        Register access                                         */
/* ---------------------------------------------------------------------------------------------- */
bool Driver::_readRegister(uint8_t reg, uint8_t& value) { return _readBurst(reg, &value, 1); }

bool Driver::_writeRegister(uint8_t reg, uint8_t value) {
  if (_spi) {
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_cs_pin, LOW);
    _spi->transfer(reg & ~SPI_READ);
    _spi->transfer(value);
    digitalWrite(_cs_pin, HIGH);
    _spi->endTransaction();
    return true;
  }

  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  return _wire->endTransmission() == 0;
}

bool Driver::_readRegisters(uint8_t reg, uint8_t* buffer, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i)
    if (!_readRegister(reg + i, buffer[i])) return false;
  return true;
}

bool Driver::_writeRegisters(uint8_t reg, const uint8_t* data, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i)
    if (!_writeRegister(reg + i, data[i])) return false;
  return true;
}

bool Driver::_readBurst(uint8_t reg, uint8_t* buffer, uint8_t n) {
  if (_spi) {
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_cs_pin, LOW);
    _spi->transfer(reg | SPI_READ);
    for (uint8_t i = 0; i < n; ++i)
      buffer[i] = _spi->transfer(0x00);
    digitalWrite(_cs_pin, HIGH);
    _spi->endTransaction();
    return true;
  }

  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;
  if (_wire->requestFrom(_address, n) != n) return false;

  for (uint8_t i = 0; i < n; ++i)
    buffer[i] = static_cast<uint8_t>(_wire->read());
  return true;
}

} // namespace LPS22
