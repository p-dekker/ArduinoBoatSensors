#include "VictronBleSensor.h"

VictronBleSensor* VictronBleSensor::s_instance = NULL;

VictronBleSensor::VictronBleSensor(const char* mac, const char* hexKey, const char* name,
                                    uint32_t staleTimeoutMs)
    : _victron(),
      _mac(mac),
      _hexKey(hexKey),
      _name(name),
      _staleTimeoutMs(staleTimeoutMs),
      _lastGoodMs(0),
      _hasGoodReading(false),
      _batteryVoltageV(NAN),
      _goodReadings(0) {}

bool VictronBleSensor::setup() {
  s_instance = this;

  if (!_victron.begin()) return false;

  _victron.setCallback(&VictronBleSensor::onData);
  if (!_victron.addDevice(_name, _mac, _hexKey, DEVICE_TYPE_BATTERY_MONITOR)) return false;

  return true;
}

void VictronBleSensor::loop() { _victron.loop(); }

bool VictronBleSensor::isValid() const {
  if (!_hasGoodReading) return false;
  return (uint32_t)(millis() - _lastGoodMs) <= _staleTimeoutMs;
}

double VictronBleSensor::batteryVoltage() const { return isValid() ? _batteryVoltageV : NAN; }

void VictronBleSensor::onData(const VictronDevice* device) {
  if (s_instance == NULL || device == NULL) return;
  if (device->deviceType != DEVICE_TYPE_BATTERY_MONITOR) return;

  s_instance->_batteryVoltageV = device->battery.voltage;
  s_instance->_lastGoodMs = millis();
  s_instance->_hasGoodReading = true;
  if (s_instance->_goodReadings < 0xFFFFFFFFUL) s_instance->_goodReadings++;
}
