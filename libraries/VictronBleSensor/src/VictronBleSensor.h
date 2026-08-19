// VictronBleSensor.h - self-updating wrapper around a single Victron BLE
// battery monitor (e.g. a SmartShunt), keeping battery voltage as internal
// state.
//
// Usage:
//
//   #include <VictronBleSensor.h>
//
//   VictronBleSensor sensor("aabbccddeeff", "000102030405060708090a0b0c0d0e0f");
//
//   void setup() {
//     if (!sensor.setup()) {
//       // BLE radio failed to start, or the mac/hexKey was rejected
//     }
//   }
//
//   void loop() {
//     sensor.loop();
//     if (sensor.isValid()) {
//       double v = sensor.batteryVoltage();  // Volts
//     }
//   }
//
// Unlike BmpSensor/DsSensor, there is no on-demand read: the underlying
// VictronBLE library only decodes a reading when the monitored device's own
// BLE advertisement arrives (a SmartShunt broadcasts roughly once a second),
// so isValid() stays false until the first one shows up after setup() -
// there is nothing to force or block on to prime it early.
//
// Only one VictronBleSensor may exist at a time: VictronBLE's callback is a
// plain function pointer with no user-data argument, so this class routes
// it through a single static instance pointer.

#ifndef VICTRON_BLE_SENSOR_H
#define VICTRON_BLE_SENSOR_H

#include <Arduino.h>
#include <stdint.h>

#include <Sensor.h>
#include <VictronBLE.h>

class VictronBleSensor : public Sensor {
 public:
  /* isValid() turns false once the last good reading is older than this. */
  static const uint32_t DefaultStaleTimeoutMs = 6500;

  /* mac: 12 hex chars, colons optional. hexKey: 32 hex chars. Both come from
     the VictronConnect app (Settings > Product info > Instant readout via
     Bluetooth > Encryption key). */
  VictronBleSensor(const char* mac, const char* hexKey, const char* name = "Victron",
                    uint32_t staleTimeoutMs = DefaultStaleTimeoutMs);

  /* Starts the BLE radio and registers the device. Returns false if the
     radio failed to start or mac/hexKey was rejected (wrong length). */
  bool setup() override;

  /* Call every pass through the sketch's loop(). Non-blocking: just pumps
     VictronBLE's scan/advertisement handling. */
  void loop() override;

  /* True if a reading arrived within the stale timeout. */
  bool isValid() const;

  /* Volts. NAN when !isValid(). */
  double batteryVoltage() const;

  /* Diagnostics. */
  uint32_t goodReadings() const { return _goodReadings; }

 private:
  static void onData(const VictronDevice* device);

  VictronBLE _victron;
  const char* _mac;
  const char* _hexKey;
  const char* _name;
  uint32_t _staleTimeoutMs;

  uint32_t _lastGoodMs;
  bool _hasGoodReading;
  double _batteryVoltageV;
  uint32_t _goodReadings;

  static VictronBleSensor* s_instance;
};

#endif  // VICTRON_BLE_SENSOR_H
