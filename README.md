# ArduinoBoatSensors

An Arduino MKR WiFi 1010 sketch that reads a BMP280, a DS18B20, and a
Victron BLE battery monitor (e.g. a SmartShunt), and publishes cabin
temperature, atmospheric pressure, outside temperature, and battery voltage
onto a boat's NMEA 2000 (CAN) bus via an MCP2515 shield.

This is an Arduino **sketchbook**: `arduino-cli` resolves libraries straight
from `libraries/` in this repo, so everything needed to build is vendored
in-tree - no separate library install step.

## Hardware

- Arduino MKR WiFi 1010
- MCP2515 CAN bus shield/module, wired to the NMEA 2000 backbone
- BMP280 (I2C) - cabin temperature and atmospheric pressure
- DS18B20 (1-Wire, data pin 2 with a 4.7k pull-up) - outside temperature
- A Victron device with Instant Readout via Bluetooth enabled (e.g. a
  SmartShunt) - battery voltage, read over BLE via the MKR's onboard
  u-blox NINA-W102 module

## Layout

```
ArduinoMKRSensors/ArduinoMKRSensors.ino   the sketch
libraries/Sensor/                         first-party: abstract setup()/loop() contract
libraries/BmpSensor/                      first-party: BMP280 wrapper, : public Sensor
libraries/DsSensor/                       first-party: DS18B20 wrapper, same shape as BmpSensor
libraries/VictronBleSensor/               first-party: Victron BLE battery monitor wrapper, same shape
libraries/{NMEA2000,NMEA2000_mcp,CAN_BUS_Shield-master}/          vendored
libraries/{Adafruit_BMP280_Library,Adafruit_BusIO,Adafruit_Unified_Sensor}/  vendored
libraries/{OneWire,DallasTemperature}/    vendored
libraries/VictronBLE/                     vendored (from a fork adding an Arduino MKR backend)
libraries/{ArduinoBLE,Arduino_SpiNINA}/   vendored
```

Every sensor implements a common `Sensor` interface (`setup()`/`loop()`
only) and is driven generically from an array in the sketch, so adding a
new sensor never requires touching the sketch's `setup()`/`loop()`. See
`CLAUDE.md` for the full design rationale, known gotchas, and what's still
open.

## Build & upload

```bash
# Compile (run from the repo root)
arduino-cli compile --fqbn arduino:samd:mkrwifi1010 \
  --libraries ./libraries ArduinoMKRSensors

# List attached boards, then upload
arduino-cli board list
arduino-cli upload -p /dev/cu.usbmodemXXXX \
  --fqbn arduino:samd:mkrwifi1010 ArduinoMKRSensors

# Watch it run - the sketch also prints every value it sends to Serial,
# so this works without a live NMEA 2000 bus connected
arduino-cli monitor -p /dev/cu.usbmodemXXXX --config baudrate=115200
```

## Status

- The Victron BLE MAC/encryption key in the sketch are placeholders - fill
  them in from the VictronConnect app before the battery reading will be
  anything but `N/A`. See `libraries/VictronBleSensor/examples/ReadSensor`
  for a MAC-discovery walkthrough.
- The NMEA 2000 node address isn't persisted across reboots (SAMD has no
  EEPROM). Accepted as-is for this boat's bus layout - see `CLAUDE.md` for
  details.

## License

This repo is licensed under Apache-2.0. The vendored libraries under
`libraries/` carry their own upstream licenses (mostly MIT/LGPL) - check
each library's own `LICENSE` file before reusing it outside this project.
