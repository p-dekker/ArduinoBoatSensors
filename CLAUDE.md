# Arduino sketchbook — NMEA 2000 sensor node

An Arduino MKR board reads a BMP280 and a DS18B20, and listens for a Victron
BLE battery monitor, and publishes cabin temperature, atmospheric pressure,
outside temperature, and battery voltage onto a boat's NMEA 2000 (CAN) bus
via an MCP2515 shield.

## Layout

This is an Arduino **sketchbook**, not a normal repo. `libraries/` holds vendored
third-party libraries that `arduino-cli` resolves from here — do not treat them
as project source, and do not edit them.

```
ArduinoMKRSensors/ArduinoMKRSensors.ino   the sketch (only first-party sketch)
libraries/Sensor/                         first-party: abstract setup()/loop() contract
libraries/BmpSensor/                      first-party: BMP280 state wrapper, : public Sensor
libraries/DsSensor/                       first-party: DS18B20 state wrapper, same shape as BmpSensor
libraries/VictronBleSensor/                first-party: Victron BLE battery monitor wrapper, same shape as BmpSensor
libraries/{NMEA2000,NMEA2000_mcp,CAN_BUS_Shield-master}/   vendored, do not edit
libraries/{Adafruit_BMP280_Library,Adafruit_BusIO,Adafruit_Unified_Sensor}/  vendored, do not edit
libraries/{OneWire,DallasTemperature}/    vendored, do not edit
libraries/VictronBLE/                     vendored, do not edit - see note below
libraries/{ArduinoBLE,Arduino_SpiNINA}/    vendored, do not edit
```

`libraries/VictronBLE` is vendored from `https://github.com/p-dekker/VictronBLE`
(a fork of `SH3D/VictronBLE`), specifically from the `samd-mkrwifi1010-backend`
branch (draft PR #1 on that fork, not yet merged to its `main`) — that branch
is what adds the `ArduinoBLE`-based backend this board needs, since the
MKR WiFi 1010's SAMD21 has no BLE radio of its own. Re-vendor from that repo
(`git archive <branch> | tar -x -C libraries/VictronBLE`) if it gets updated,
not by hand-editing the copy here.

### The `Sensor` interface

`libraries/Sensor/src/Sensor.h` declares just `virtual bool setup() = 0;` and
`virtual void loop() = 0;` — deliberately nothing else. The sketch holds a
`Sensor *Sensors[] = { &bmp, &outsideTemp, &victronBattery };` array and drives
every sensor's `setup()`/`loop()` generically from it, so adding a sensor never
requires touching `setup()`/`loop()` in the sketch: make the new class
`: public Sensor`, add it to the array, done. `VictronBleSensor` is the first
sensor built after this interface was written specifically to accommodate it,
and it slotted in without any change to `setup()`/`loop()`, confirming the
design works as intended.

What's deliberately **not** in the interface: value getters (`temperature()`,
`pressure()`, `batteryVoltage()`, ...), staleness (`isValid()`), and anything
sketch-level tied to a reading (NMEA `SID` advancing, LED flashes). Those stay
specific to each concrete sensor and are accessed by name, both because they
genuinely differ per sensor and because folding NMEA-specific bookkeeping into
the sensor libraries would leak an unrelated concern into classes that
otherwise know nothing about NMEA 2000. SID advancing in particular is
generalized in the sketch as a small templated `SidTracker` class (one
instance per sensor: `BmpSid`, `OutsideSid`, `VictronSid`) rather than a
per-sensor bookkeeping function, since every sensor's SID logic is identical
- "if `goodReadings()` changed, advance a wrapping SID."

## Build & upload

`FQBN` below is a placeholder — replace with the actual board before relying on it.

```bash
# Compile the sketch (run from the sketchbook root)
arduino-cli compile --fqbn arduino:samd:mkrwifi1010 \
  --libraries ./libraries ArduinoMKRSensors

# Compile the library examples too — they catch API breakage the sketch misses
arduino-cli compile --fqbn arduino:samd:mkrwifi1010 \
  --libraries ./libraries libraries/BmpSensor/examples/ReadSensor
arduino-cli compile --fqbn arduino:samd:mkrwifi1010 \
  --libraries ./libraries libraries/DsSensor/examples/ReadSensor
arduino-cli compile --fqbn arduino:samd:mkrwifi1010 \
  --libraries ./libraries libraries/VictronBleSensor/examples/ReadSensor

# Show all warnings while iterating
arduino-cli compile --warnings all --fqbn arduino:samd:mkrwifi1010 \
  --libraries ./libraries ArduinoMKRSensors

# List attached boards, then upload
arduino-cli board list
arduino-cli upload -p /dev/cu.usbmodemXXXX \
  --fqbn arduino:samd:mkrwifi1010 ArduinoMKRSensors

arduino-cli monitor -p /dev/cu.usbmodemXXXX --config baudrate=115200
```

Always compile after changing `libraries/Sensor`, `libraries/BmpSensor`,
`libraries/DsSensor`, or `libraries/VictronBleSensor` — there is no test
harness on the target, so the compiler is the only automated check. A change
to `Sensor.h` in particular can break all three concrete sensors at once via
the `override` mismatch it would cause, so recompile all three example
sketches, not just one.

## Gotchas that will bite you

**`MODE_SLEEP` collides.** `Adafruit_BMP280::MODE_SLEEP` (an enum) clashes with a
`MODE_SLEEP` *macro* in the MCP2515/CAN libraries, so include order used to
matter. `BmpSensor.h` deliberately forward-declares `Adafruit_BMP280` and
includes the real header only in `BmpSensor.cpp`, which makes the sketch immune.
Do not "tidy up" by moving that include into the header.

**`takeForcedMeasurement()` lies.** It returns `true` whenever the chip is in
forced mode; it never checks that the I2C transfer worked. A disconnected sensor
reads back `0xFF` registers that decode to plausible-looking floats. `BmpSensor`
therefore range-checks every reading against the datasheet limits (-40…+85 °C,
300…1100 hPa) and treats out-of-range as a failure.

**Never transmit stale sensor data.** `BmpSensor::isValid()` goes false 6500 ms
after the last good reading. Callers must substitute `N2kDoubleNA` rather than
sending a retained or zero value — on the bus, "unknown" and "0 K" mean very
different things to listeners.

**No IIR filtering in forced mode.** Sampling is `FILTER_OFF` on purpose: the
filter needs ~16 consecutive samples to settle, and at one sample per 2 s that
would lag reality by roughly half a minute.

**DS18B20 conversion must never block `loop()`.** A 12-bit conversion takes
~750 ms — long enough to stall `NMEA2000.ParseMessages()` if done the way
`BmpSensor` reads the BMP280 (one blocking call). `DsSensor` therefore calls
`setWaitForConversion(false)` and splits the read into two phases inside
`loop()`: start the conversion, then poll `millis()` against the datasheet
conversion time before reading back. The only place `DsSensor` blocks is once,
synchronously, inside `setup()`, to prime `isValid()` before the sketch's
`loop()` starts.

**`getTempCByIndex()` also lies, differently than the BMP280 does.** It
returns `DEVICE_DISCONNECTED_C` (-127 °C) on a failed read instead of throwing
or blocking. That value is already outside the DS18B20 datasheet range
(-55…+125 °C), so `DsSensor`'s plausibility check catches it as a side effect
— no separate sentinel comparison needed.

## Conventions

- The existing sketch style is upstream NMEA2000-example style (2-space indent,
  `//` comments, `tN2kMsg` locals). Match it rather than reformatting.
- Library code uses `_leadingUnderscore` members and initialises every member in
  the constructor init list, in declaration order.
- Getters return SI units that NMEA 2000 wants directly (Kelvin, Pascal) so the
  sketch needs no conversion, and `NAN` when the value is not valid.

## Won't fix (accepted)

1. The node address (22) is not persisted after address claim, so a claim
   conflict would be re-fought on every boot. SAMD has no EEPROM, so fixing
   this means vendoring FlashStorage (or similar) and wiring it to
   `NMEA2000.h`'s existing hooks (`ReadResetAddressChanged()`,
   `GetN2kSource()`, `SetN2kSource()`). Accepted as-is: on this boat, address
   22 is free and the bus configuration isn't expected to change, so there's
   no conflict to re-fight in practice. Revisit if another device ever claims
   22 first or the bus layout changes.

## Fixed

The following were fixed in the sketch (no vendored library touched):

- `TransmitMessages` now declares 130316/130311/130314, matching what's
  actually sent (`SetN2kTemperatureExt`/`SetN2kEnvironmentalParameters`/
  `SetN2kPressure`) instead of the stale 130310/130312 mismatch.
- Switched from deprecated `SetN2kTemperature` (130312) to
  `SetN2kTemperatureExt` (130316, 0.001 K resolution).
- Renamed `OutsideEnvironmentalScheduler` to `PressureScheduler` and changed
  its period from 500 ms to the PGN 130314 default of 2000 ms.
- `SetN2kEnvironmentalParameters` now passes the live `pressure` reading as
  its `AtmosphericPressure` argument instead of leaving it N/A.
- Replaced the hardcoded `SID` of `1` with a shared `uint8_t SID` that
  increments (wrapping at 253) once per BMP280 measurement cycle, so all
  messages built from the same reading carry the same SID.
- `ErrorLoop()` now blinks a short pattern and calls `NVIC_SystemReset()`
  instead of hanging forever — rebooting beats going dark for an unattended
  sensor on a boat.

## Outside temperature (DS18B20)

Added `libraries/DsSensor`, structured the same way as `BmpSensor`
(`setup()`/`loop()`/`isValid()`/staleness timeout), wrapping the newly
vendored `OneWire` + `DallasTemperature` libraries. Wired into the sketch as:

- Data line on pin 2 (`OutsideTempPin`), with a 4.7k pull-up to the sensor's
  VDD — no other pin was specified, so confirm this matches the actual wiring
  before relying on it.
- Sent via `SetN2kTemperatureExt` (130316, same PGN as cabin temperature) with
  `N2kts_OutsideTemperature` and instance `2` (cabin is instance `1`), on its
  own `OutsideTemperatureScheduler` (2000 ms period) and its own `OutsideSID`
  counter — it must not share the BMP280's `SID`, since it correlates a
  different underlying reading.

## Victron battery voltage (BLE SmartShunt)

Added `libraries/VictronBleSensor`, structured the same way as
`BmpSensor`/`DsSensor` (`setup()`/`loop()`/`isValid()`/staleness timeout),
wrapping the newly vendored `VictronBLE` library. Unlike the other two
sensors, there is nothing to poll: `VictronBleSensor` only gets a fresh
reading when the SmartShunt's own BLE advertisement arrives and decodes
successfully (roughly once a second), so `isValid()` stays false until the
first one shows up after `setup()` — there is no blocking-in-`setup()` trick
available here the way `DsSensor` uses one.

- `VictronMac`/`VictronHexKey` in the sketch are **placeholders** — get the
  real values from the VictronConnect app (Settings > Product info >
  Instant readout via Bluetooth > Encryption key) before relying on this.
  `libraries/VictronBleSensor/examples/ReadSensor` doubles as a MAC-discovery
  tool: run it with the placeholder MAC first and watch Serial for
  `[VictronBLE] Unmonitored Victron: <mac>` to find the SmartShunt's real one.
- Sent via `SetN2kDCBatStatus` (PGN 127508, 1500 ms default period, its own
  `BatteryVoltageScheduler` and `VictronSid`), battery instance `0` — a
  separate instance namespace from the `N2kts_*` temperature source
  instances used elsewhere in the sketch, so it doesn't collide with them.
  Current and temperature are left as `N2kDoubleNA` for now; `VictronBLE`
  already decodes both for a battery monitor if a future change wants them.
- Only one `VictronBleSensor` may exist at a time: `VictronBLE`'s callback is
  a plain function pointer with no user-data argument, so the wrapper routes
  it through a single static instance pointer. Monitoring a second Victron
  device would need `VictronBleSensor` to grow support for `VictronBLE`'s
  existing multi-device `addDevice()` (up to 8), not a second instance of
  this class.
