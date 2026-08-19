// DsSensor.h - self-updating DS18B20 outside-temperature state.
//
// Usage:
//
//   #include <DsSensor.h>
//
//   DsSensor sensor(2);  // data pin, with a 4.7k pull-up to the sensor's VDD
//
//   void setup() {
//     if (!sensor.setup()) {
//       // no DS18x20 found on the bus
//     }
//   }
//
//   void loop() {
//     sensor.loop();                      // starts/polls a conversion every 2000 ms
//     if (sensor.isValid()) {
//       double k = sensor.temperature();  // Kelvin
//     }
//   }
//
// Note on includes: OneWire.h and DallasTemperature.h are deliberately NOT
// included here, only in DsSensor.cpp, so callers of this header don't need
// to know the 1-Wire library exists - the same isolation BmpSensor.h uses
// for Adafruit_BMP280.
//
// Assumes a single DS18x20-family device on the bus (reads by index 0).

#ifndef DS_SENSOR_H
#define DS_SENSOR_H

#include <Arduino.h>
#include <stdint.h>

#include <Sensor.h>

class OneWire;
class DallasTemperature;

class DsSensor : public Sensor {
 public:
  /* How often loop() starts a new conversion. Must stay comfortably above
     the ~750 ms a 12-bit conversion takes. */
  static const uint32_t DefaultUpdateIntervalMs = 2000;

  /* isValid() turns false once the last good reading is older than this.
     Slightly over three update intervals, so a single missed read does not
     invalidate the state. */
  static const uint32_t DefaultStaleTimeoutMs = 6500;

  /* Re-run begin() after this many consecutive failed reads, to recover
     from a sensor that was briefly unpowered or a wedged 1-Wire bus. */
  static const uint8_t ReinitAfterFailures = 3;

  DsSensor(uint8_t pin, uint32_t updateIntervalMs = DefaultUpdateIntervalMs,
           uint32_t staleTimeoutMs = DefaultStaleTimeoutMs);

  /* Scan the 1-Wire bus on the configured pin for a DS18x20-family device.
     Returns true if one answered. Blocks for one full conversion (~750 ms)
     so isValid() is already meaningful when setup() returns - the only
     place this class blocks. */
  bool setup() override;

  /* Call every pass through the sketch's loop(). Cheap and non-blocking:
     starts a conversion every updateIntervalMs and polls for completion on
     later calls, so it never stalls the caller for the conversion time. */
  void loop() override;

  /* True if a plausible reading was taken within the stale timeout. */
  bool isValid() const;

  /* Kelvin. NAN when !isValid(). */
  double temperature() const;

  /* Degrees Celsius, for logging. NAN when !isValid(). */
  double temperatureCelsius() const;

  /* Age of the most recent good reading, in ms. Returns UINT32_MAX if
     there has never been one. */
  uint32_t ageMs() const;

  /* Diagnostics. */
  bool isPresent() const { return _present; }
  uint8_t consecutiveFailures() const { return _failures; }
  uint32_t goodReadings() const { return _goodReadings; }

  void setUpdateInterval(uint32_t ms) { _updateIntervalMs = ms; }
  void setStaleTimeout(uint32_t ms) { _staleTimeoutMs = ms; }

 private:
  bool begin();
  void applyConfiguration();
  void startConversion();
  bool finishConversion();
  bool recordFailure();  // counts a failed read, re-inits when needed; returns false

  OneWire *_oneWire;
  DallasTemperature *_sensors;

  uint8_t _pin;
  uint32_t _updateIntervalMs;
  uint32_t _staleTimeoutMs;
  uint32_t _conversionDelayMs;

  uint32_t _lastAttemptMs;
  uint32_t _conversionStartMs;
  bool _converting;

  uint32_t _lastGoodMs;
  bool _hasGoodReading;

  double _temperatureK;

  bool _present;
  uint8_t _failures;
  uint32_t _goodReadings;
};

#endif  // DS_SENSOR_H
