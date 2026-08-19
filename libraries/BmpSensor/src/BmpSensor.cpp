#include "BmpSensor.h"

#include <Adafruit_BMP280.h>
// Adafruit_BMP280::MODE_SLEEP clashes with the MODE_SLEEP macro in the
// MCP2515/CAN libraries. Drop the macro here so translation units that pull in
// both still compile, regardless of include order.
#undef MODE_SLEEP

// BMP280 datasheet operating range, used to sanity-check readings.
// takeForcedMeasurement() returns true whenever the chip is in forced mode -
// it does not verify that the I2C transfer worked. A disconnected sensor reads
// back as 0xFF registers, which decodes to values far outside these bounds.
static const double MinTemperatureK = 233.15;  // -40 C
static const double MaxTemperatureK = 358.15;  // +85 C
static const double MinPressurePa = 30000.0;   // 300 hPa
static const double MaxPressurePa = 110000.0;  // 1100 hPa

BmpSensor::BmpSensor(uint32_t updateIntervalMs, uint32_t staleTimeoutMs)
    : _bmp(NULL),
      _updateIntervalMs(updateIntervalMs),
      _staleTimeoutMs(staleTimeoutMs),
      _lastAttemptMs(0),
      _lastGoodMs(0),
      _hasGoodReading(false),
      _temperatureK(NAN),
      _pressurePa(NAN),
      _address(0),
      _present(false),
      _failures(0),
      _goodReadings(0) {}

bool BmpSensor::setup() {
  if (setup(BMP280_ADDRESS)) return true;       // 0x77
  return setup(BMP280_ADDRESS_ALT);             // 0x76
}

bool BmpSensor::setup(uint8_t i2cAddress) {
  if (!begin(i2cAddress)) return false;

  // Prime the state so isValid() is meaningful straight after setup().
  update();
  return true;
}

bool BmpSensor::begin(uint8_t i2cAddress) {
  if (_bmp == NULL) {
    _bmp = new Adafruit_BMP280();
    if (_bmp == NULL) return false;
  }

  _present = _bmp->begin(i2cAddress);
  if (!_present) return false;

  _address = i2cAddress;
  _failures = 0;
  applySampling();
  return true;
}

void BmpSensor::applySampling() {
  // Forced mode: the chip sleeps between our reads, which keeps current draw
  // low at a 2000 ms cadence. FILTER_OFF because the IIR filter needs ~16
  // consecutive samples to settle - with one sample every 2 s that would make
  // the readings lag reality by half a minute.
  _bmp->setSampling(Adafruit_BMP280::MODE_FORCED,
                    Adafruit_BMP280::SAMPLING_X2,   // temperature
                    Adafruit_BMP280::SAMPLING_X16,  // pressure
                    Adafruit_BMP280::FILTER_OFF,
                    Adafruit_BMP280::STANDBY_MS_1);  // unused in forced mode
}

void BmpSensor::loop() {
  // No attempt yet (setup() not called, or called and it bailed early):
  // read straight away so the first loop() populates or fails visibly.
  if (_goodReadings == 0 && _failures == 0) {
    update();
    return;
  }

  if ((uint32_t)(millis() - _lastAttemptMs) < _updateIntervalMs) return;
  update();
}

bool BmpSensor::recordFailure() {
  if (_failures < 255) _failures++;

  // Only worth re-initialising if we know which address to talk to.
  if (_failures >= ReinitAfterFailures && _address != 0) begin(_address);
  return false;
}

bool BmpSensor::update() {
  _lastAttemptMs = millis();

  if (_bmp == NULL || !_present) return recordFailure();

  // Blocks for roughly 40 ms at SAMPLING_X16 pressure oversampling while the
  // conversion completes. Happens once per update interval.
  if (!_bmp->takeForcedMeasurement()) return recordFailure();

  const double temperatureK = (double)_bmp->readTemperature() + 273.15;
  const double pressurePa = (double)_bmp->readPressure();

  const bool plausible = !isnan(temperatureK) && !isnan(pressurePa) &&
                         temperatureK >= MinTemperatureK &&
                         temperatureK <= MaxTemperatureK &&
                         pressurePa >= MinPressurePa &&
                         pressurePa <= MaxPressurePa;

  if (!plausible) return recordFailure();

  _temperatureK = temperatureK;
  _pressurePa = pressurePa;
  _lastGoodMs = _lastAttemptMs;
  _hasGoodReading = true;
  _failures = 0;
  if (_goodReadings < 0xFFFFFFFFUL) _goodReadings++;
  return true;
}

bool BmpSensor::isValid() const {
  if (!_hasGoodReading) return false;
  return (uint32_t)(millis() - _lastGoodMs) <= _staleTimeoutMs;
}

double BmpSensor::temperature() const { return isValid() ? _temperatureK : NAN; }

double BmpSensor::pressure() const { return isValid() ? _pressurePa : NAN; }

double BmpSensor::temperatureCelsius() const {
  return isValid() ? _temperatureK - 273.15 : NAN;
}

uint32_t BmpSensor::ageMs() const {
  if (!_hasGoodReading) return UINT32_MAX;
  return (uint32_t)(millis() - _lastGoodMs);
}
