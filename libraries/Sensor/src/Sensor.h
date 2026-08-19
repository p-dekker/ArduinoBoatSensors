// Sensor.h - minimal common contract so heterogeneous sensor wrappers
// (BmpSensor, DsSensor, and future ones - e.g. a Victron BLE sensor) can be
// driven generically from a Sensor* array in the sketch's setup()/loop().
//
// Deliberately just setup()/loop(): that's the only part every sensor
// wrapper does identically. What a sensor measures, how it reports
// staleness, how many arguments its constructor takes, and any
// sketch-level bookkeeping tied to a reading (NMEA SID, LED flashes) stay
// specific to the concrete class and are accessed by name, not through
// this interface.

#ifndef SENSOR_H
#define SENSOR_H

class Sensor {
 public:
  virtual ~Sensor() {}

  /* Take the sensor from cold to ready. Returns true on success. */
  virtual bool setup() = 0;

  /* Call every pass through the sketch's loop(). Must never block longer
     than the caller can tolerate - each implementation documents its own
     worst case. */
  virtual void loop() = 0;
};

#endif  // SENSOR_H
