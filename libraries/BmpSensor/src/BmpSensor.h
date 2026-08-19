// BmpSensor.h - self-updating BMP280 temperature/pressure state.
//
// Usage:
//
//   #include <BmpSensor.h>
//
//   BmpSensor sensor;
//
//   void setup() {
//     if (!sensor.setup()) {
//       // sensor not found on 0x77 or 0x76
//     }
//   }
//
//   void loop() {
//     sensor.loop();                       // refreshes state every 2000 ms
//     if (sensor.isValid()) {
//       double k  = sensor.temperature();  // Kelvin
//       double pa = sensor.pressure();     // Pascal
//     }
//   }
//
// Note on includes: Adafruit_BMP280.h is deliberately NOT included here.
// Its sensor_mode enum declares MODE_SLEEP, which collides with the
// MODE_SLEEP macro in the MCP2515/CAN libraries. Keeping the BMP280 header
// inside BmpSensor.cpp means the sketch can include this header in any
// order relative to NMEA2000_CAN.h without needing an #undef.

#ifndef BMP_SENSOR_H
#define BMP_SENSOR_H

#include <Arduino.h>
#include <stdint.h>

#include <Sensor.h>

class Adafruit_BMP280;  // forward declaration only - see note above

class BmpSensor : public Sensor {
 public:
  /* How often loop() refreshes the internal state. */
  static const uint32_t DefaultUpdateIntervalMs = 2000;

  /* isValid() turns false once the last good reading is older than this.
     Slightly over three update intervals, so a single missed read does not
     invalidate the state. */
  static const uint32_t DefaultStaleTimeoutMs = 6500;

  /* Re-run begin() after this many consecutive failed reads, to recover
     from a sensor that was briefly unpowered or a wedged I2C bus. */
  static const uint8_t ReinitAfterFailures = 3;

  BmpSensor(uint32_t updateIntervalMs = DefaultUpdateIntervalMs,
            uint32_t staleTimeoutMs = DefaultStaleTimeoutMs);

  /* Probe the default address (0x77) and then the alternate (0x76).
     Returns true if a BMP280 answered. Takes one reading on success, so
     isValid() is already true when setup() returns. */
  bool setup() override;

  /* Same, but only tries the given I2C address. */
  bool setup(uint8_t i2cAddress);

  /* Call every pass through the sketch's loop(). Cheap and non-blocking
     except on the pass where it actually reads the sensor. */
  void loop() override;

  /* Read immediately, ignoring the update interval. Returns true if the
     reading passed the plausibility checks. */
  bool update();

  /* True if a plausible reading was taken within the stale timeout. */
  bool isValid() const;

  /* Kelvin. NAN when !isValid(). */
  double temperature() const;

  /* Pascal. NAN when !isValid(). */
  double pressure() const;

  /* Degrees Celsius, for logging. NAN when !isValid(). */
  double temperatureCelsius() const;

  /* Age of the most recent good reading, in ms. Returns UINT32_MAX if
     there has never been one. */
  uint32_t ageMs() const;

  /* Diagnostics. */
  uint8_t i2cAddress() const { return _address; }
  bool isPresent() const { return _present; }
  uint8_t consecutiveFailures() const { return _failures; }
  uint32_t goodReadings() const { return _goodReadings; }

  void setUpdateInterval(uint32_t ms) { _updateIntervalMs = ms; }
  void setStaleTimeout(uint32_t ms) { _staleTimeoutMs = ms; }

 private:
  bool begin(uint8_t i2cAddress);
  void applySampling();
  bool recordFailure();  // counts a failed read, re-inits when needed; returns false

  Adafruit_BMP280 *_bmp;

  uint32_t _updateIntervalMs;
  uint32_t _staleTimeoutMs;

  uint32_t _lastAttemptMs;
  uint32_t _lastGoodMs;
  bool _hasGoodReading;

  double _temperatureK;
  double _pressurePa;

  uint8_t _address;
  bool _present;
  uint8_t _failures;
  uint32_t _goodReadings;
};

#endif  // BMP_SENSOR_H
