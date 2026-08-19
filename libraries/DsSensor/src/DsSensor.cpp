#include "DsSensor.h"

#include <OneWire.h>
#include <DallasTemperature.h>

// DS18B20 datasheet operating range, used to sanity-check readings.
// getTempCByIndex() returns DEVICE_DISCONNECTED_C (-127) on a failed read,
// which already falls well outside this range, so the same plausibility
// check that catches a genuinely out-of-range reading catches a
// disconnected sensor too - no separate sentinel check needed.
static const double MinTemperatureK = 218.15;  // -55 C
static const double MaxTemperatureK = 398.15;  // +125 C

// 12-bit resolution: 0.0625 C steps, ~750 ms conversion time.
static const uint8_t Resolution = 12;

DsSensor::DsSensor(uint8_t pin, uint32_t updateIntervalMs, uint32_t staleTimeoutMs)
    : _oneWire(NULL),
      _sensors(NULL),
      _pin(pin),
      _updateIntervalMs(updateIntervalMs),
      _staleTimeoutMs(staleTimeoutMs),
      _conversionDelayMs(0),
      _lastAttemptMs(0),
      _conversionStartMs(0),
      _converting(false),
      _lastGoodMs(0),
      _hasGoodReading(false),
      _temperatureK(NAN),
      _present(false),
      _failures(0),
      _goodReadings(0) {}

bool DsSensor::setup() {
  if (!begin()) return false;

  // Prime the state so isValid() is meaningful straight after setup().
  startConversion();
  delay(_conversionDelayMs);
  finishConversion();
  return true;
}

bool DsSensor::begin() {
  if (_oneWire == NULL) {
    _oneWire = new OneWire(_pin);
    if (_oneWire == NULL) return false;
  }
  if (_sensors == NULL) {
    _sensors = new DallasTemperature(_oneWire);
    if (_sensors == NULL) return false;
  }

  _sensors->begin();
  _present = _sensors->getDeviceCount() > 0;
  if (!_present) return false;

  _failures = 0;
  _converting = false;
  applyConfiguration();
  return true;
}

void DsSensor::applyConfiguration() {
  // Non-blocking: loop() times the conversion itself rather than having
  // requestTemperatures() block for it.
  _sensors->setWaitForConversion(false);
  _sensors->setResolution(Resolution);
  _conversionDelayMs = DallasTemperature::millisToWaitForConversion(Resolution);
}

void DsSensor::loop() {
  if (_converting) {
    if ((uint32_t)(millis() - _conversionStartMs) < _conversionDelayMs) return;
    finishConversion();
    return;
  }

  // No attempt yet (setup() not called, or called and it bailed early):
  // start straight away so the first loop() populates or fails visibly.
  if (_goodReadings == 0 && _failures == 0) {
    startConversion();
    return;
  }

  if ((uint32_t)(millis() - _lastAttemptMs) < _updateIntervalMs) return;
  startConversion();
}

void DsSensor::startConversion() {
  _lastAttemptMs = millis();

  if (_sensors == NULL || !_present) {
    recordFailure();
    return;
  }

  _sensors->requestTemperatures();
  _conversionStartMs = _lastAttemptMs;
  _converting = true;
}

bool DsSensor::finishConversion() {
  _converting = false;

  const double temperatureK = (double)_sensors->getTempCByIndex(0) + 273.15;

  const bool plausible = !isnan(temperatureK) &&
                         temperatureK >= MinTemperatureK &&
                         temperatureK <= MaxTemperatureK;

  if (!plausible) return recordFailure();

  _temperatureK = temperatureK;
  _lastGoodMs = millis();
  _hasGoodReading = true;
  _failures = 0;
  if (_goodReadings < 0xFFFFFFFFUL) _goodReadings++;
  return true;
}

bool DsSensor::recordFailure() {
  if (_failures < 255) _failures++;
  if (_failures >= ReinitAfterFailures) begin();
  return false;
}

bool DsSensor::isValid() const {
  if (!_hasGoodReading) return false;
  return (uint32_t)(millis() - _lastGoodMs) <= _staleTimeoutMs;
}

double DsSensor::temperature() const { return isValid() ? _temperatureK : NAN; }

double DsSensor::temperatureCelsius() const {
  return isValid() ? _temperatureK - 273.15 : NAN;
}

uint32_t DsSensor::ageMs() const {
  if (!_hasGoodReading) return UINT32_MAX;
  return (uint32_t)(millis() - _lastGoodMs);
}
