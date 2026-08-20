
#include <Arduino.h>
#include <stdint.h>

#include <Sensor.h>          // common setup()/loop() contract, see the Sensors[] array below
#include <BmpSensor.h>       // safe to include before the CAN headers - see BmpSensor.h
#include <DsSensor.h>        // outside-temperature DS18B20, same isolation as BmpSensor.h
#include <VictronBleSensor.h>  // Victron BLE battery monitor (SmartShunt), same shape as BmpSensor.h

#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>

// Set to 1 to print every message's values to Serial right after they're
// sent on the bus - lets you sanity-check readings without a live NMEA 2000
// bus connected. Set to 0 for silent operation; the prints below compile
// out entirely rather than becoming no-ops.
#define DEBUG_SERIAL_PRINT 1

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = { 130316L, 130311L, 130314L, 127508L, 0 };

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler TemperatureScheduler(false, 2000, 500);
tN2kSyncScheduler EnvironmentalScheduler(false, 500, 510);
tN2kSyncScheduler PressureScheduler(false, 2000, 520);
tN2kSyncScheduler OutsideTemperatureScheduler(false, 2000, 530);
tN2kSyncScheduler BatteryVoltageScheduler(false, 1500, 540);  // 1500 ms: PGN 127508 default

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen() {
  // Start schedulers now.
  TemperatureScheduler.UpdateNextTime();
  EnvironmentalScheduler.UpdateNextTime();
  PressureScheduler.UpdateNextTime();
  OutsideTemperatureScheduler.UpdateNextTime();
  BatteryVoltageScheduler.UpdateNextTime();
}

// SID bookkeeping is NMEA 2000-specific (not part of the Sensor interface,
// see CLAUDE.md), so it stays here in the sketch rather than in the sensor
// libraries. goodReadings() isn't declared on Sensor either - it's exposed
// identically by BmpSensor and DsSensor, so this is templated rather than
// typed against a common base. Advances a wrapping SID (0-252; 253-255 are
// reserved) once per fresh reading, so every message built from that same
// reading carries the same SID, per NMEA 2000 convention.
class SidTracker {
 public:
  template <typename SensorT>
  void update(const SensorT &sensor) {
    const uint32_t goodReadings = sensor.goodReadings();
    if (goodReadings == _lastGoodReadings) return;
    _lastGoodReadings = goodReadings;
    _sid = (_sid + 1) % 253;
  }

  uint8_t sid() const { return _sid; }

 private:
  uint32_t _lastGoodReadings = 0;
  uint8_t _sid = 0;
};

// ********************** BMP280 **************************
// All sensor handling lives in the BmpSensor library. It refreshes its internal
// state every 2000 ms and reports staleness through isValid().
BmpSensor bmp;
SidTracker BmpSid;

// ********************** DS18B20 (outside temperature) **********************
// Data line on pin 2, with a 4.7k pull-up to the sensor's VDD. All sensor
// handling lives in the DsSensor library, structured the same way as
// BmpSensor above: it refreshes its internal state every 2000 ms and
// reports staleness through isValid().
const uint8_t OutsideTempPin = 2;
DsSensor outsideTemp(OutsideTempPin);

// Own tracker, separate from the BMP280's - it correlates a different
// underlying reading, so it must not share the BMP280's SID.
SidTracker OutsideSid;

// ********************** Victron BLE battery monitor (SmartShunt) **********
// All sensor handling lives in the VictronBleSensor library, same shape as
// BmpSensor/DsSensor. Unlike those, a reading arrives asynchronously
// whenever the SmartShunt's own BLE advertisement decodes successfully,
// rather than on a fixed poll interval.
//
// TODO: placeholders - get the real values from the VictronConnect app
// (Settings > Product info > Instant readout via Bluetooth > Encryption
// key) before relying on this. See libraries/VictronBleSensor's
// ReadSensor example for a discovery walkthrough.
const char *VictronMac = "aabbccddeeff";
const char *VictronHexKey = "000102030405060708090a0b0c0d0e0f";
VictronBleSensor victronBattery(VictronMac, VictronHexKey, "SmartShunt");

SidTracker VictronSid;

// All sensors driven generically from setup()/loop() below. Add a new
// sensor by making it `: public Sensor` and adding it to this array - no
// other change to setup()/loop() needed.
Sensor *Sensors[] = { &bmp, &outsideTemp, &victronBattery };
const uint8_t SensorCount = sizeof(Sensors) / sizeof(Sensors[0]);

// Names matching Sensors[] above, index-for-index. Temporary - only for the
// setup() failure print below, to see which sensor triggers ErrorLoop() over
// serial before the reset wipes the cause.
const char *SensorNames[] = { "BMP280", "DS18B20 (outside temp)", "Victron BLE" };

// ********************** Status LED **************************
// Heartbeat while running normally: 1 s on, 1 s off. Non-blocking (millis()
// based) so it never stalls NMEA2000.ParseMessages(). ErrorLoop() below
// takes over the LED with a fast blink right before it reboots.
const uint16_t LedHeartbeatPeriod = 1000;
uint32_t lastLedToggle = 0;
bool ledOn = false;

void updateHeartbeatLed() {
  const uint32_t now = millis();
  if (now - lastLedToggle >= LedHeartbeatPeriod) {
    lastLedToggle = now;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
  }
}

void nmeaInit() {
  // Set Product information
  NMEA2000.SetProductInformation("00000001",             // Manufacturer's Model serial code
                                 100,                    // Manufacturer's product code
                                 "Simple temp monitor",  // Manufacturer's Model ID
                                 "1.0.0 (2026-07-20)",   // Manufacturer's Software version code
                                 "1.0.0 (2026-07-20)"    // Manufacturer's Model version
  );
  // Set device information
  NMEA2000.SetDeviceInformation(112233,  // Unique number. Use e.g. Serial number.
                                130,     // Device function=Temperature. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75,      // Device class=Sensor Communication Interface. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040     // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );
  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader
  // NMEA2000.SetForwardStream(&Serial);
  // // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(false);  // Disable all msg forwarding to USB (=Serial)
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();
}


// *****************************************************************************
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) { 
    delay(100);
  } //wait until ready
  Serial.println("Arduino MKR Boat Sensors");
  
  #if DEBUG_SERIAL_PRINT
  Serial.println("DEBUG ON");
  #endif 

  for (uint8_t i = 0; i < SensorCount; i++) {
    if (!Sensors[i]->setup()) {
      
      #if DEBUG_SERIAL_PRINT
      Serial.print("[ErrorLoop] setup() failed for sensor: ");
      Serial.println(SensorNames[i]);
      #endif

      ErrorLoop();
    }
  }

  nmeaInit();
}



// *****************************************************************************
void loop() {
  updateHeartbeatLed();
  for (uint8_t i = 0; i < SensorCount; i++) {
    Sensors[i]->loop();  //each sensor refreshes its own internal state.
  }
  BmpSid.update(bmp);
  OutsideSid.update(outsideTemp);
  VictronSid.update(victronBattery);
  SendN2kMessages();  //send it NMEA bus various message on different schedule.
  NMEA2000.ParseMessages();
}

#if DEBUG_SERIAL_PRINT
// N2kDoubleNA is a sentinel (-1e9), not NaN, so print "N/A" instead of the
// raw sentinel value.
void printN2kValue(double value, const char *unit) {
  if (N2kIsNA(value)) {
    Serial.print("N/A");
  } else {
    Serial.print(value);
    Serial.print(unit);
  }
}
#endif

// *****************************************************************************
void SendN2kMessages() {

  tN2kMsg N2kMsg;

  // Send N2kDoubleNA rather than a stale value when the sensor state is not
  // fresh, so listeners can tell "unknown" from "actually zero".
  const double cabinTemp = bmp.isValid() ? bmp.temperature() : N2kDoubleNA;
  const double pressure = bmp.isValid() ? bmp.pressure() : N2kDoubleNA;
  const double outsideTempK = outsideTemp.isValid() ? outsideTemp.temperature() : N2kDoubleNA;
  const double batteryVoltage = victronBattery.isValid() ? victronBattery.batteryVoltage() : N2kDoubleNA;

  if (TemperatureScheduler.IsTime()) {
    TemperatureScheduler.UpdateNextTime();
    SetN2kTemperatureExt(N2kMsg, BmpSid.sid(), 1, N2kts_MainCabinTemperature, cabinTemp);
    NMEA2000.SendMsg(N2kMsg);

#if DEBUG_SERIAL_PRINT
    Serial.print("[130316 MainCabinTemperature] SID=");
    Serial.print(BmpSid.sid());
    Serial.print(" ");
    printN2kValue(cabinTemp, " K");
    Serial.println();
#endif
  }

  if (EnvironmentalScheduler.IsTime()) {
    EnvironmentalScheduler.UpdateNextTime();
    SetN2kEnvironmentalParameters(N2kMsg, BmpSid.sid(), N2kts_MainCabinTemperature, cabinTemp,
                                   N2khs_Undef, N2kDoubleNA, pressure);
    NMEA2000.SendMsg(N2kMsg);
#if DEBUG_SERIAL_PRINT
    Serial.print("[130311 EnvironmentalParameters] SID=");
    Serial.print(BmpSid.sid());
    Serial.print(" temp=");
    printN2kValue(cabinTemp, " K");
    Serial.print(" pressure=");
    printN2kValue(pressure, " Pa");
    Serial.println();
#endif
  }

  if (PressureScheduler.IsTime()) {
    PressureScheduler.UpdateNextTime();
    SetN2kPressure(N2kMsg, BmpSid.sid(), 1, N2kps_Atmospheric, pressure);
    NMEA2000.SendMsg(N2kMsg);
#if DEBUG_SERIAL_PRINT
    Serial.print("[130314 Pressure] SID=");
    Serial.print(BmpSid.sid());
    Serial.print(" ");
    printN2kValue(pressure, " Pa");
    Serial.println();
#endif
  }

  if (OutsideTemperatureScheduler.IsTime()) {
    OutsideTemperatureScheduler.UpdateNextTime();
    SetN2kTemperatureExt(N2kMsg, OutsideSid.sid(), 2, N2kts_OutsideTemperature, outsideTempK);
    NMEA2000.SendMsg(N2kMsg);
#if DEBUG_SERIAL_PRINT
    Serial.print("[130316 OutsideTemperature] SID=");
    Serial.print(OutsideSid.sid());
    Serial.print(" ");
    printN2kValue(outsideTempK, " K");
    Serial.println();
#endif
  }

  if (BatteryVoltageScheduler.IsTime()) {
    BatteryVoltageScheduler.UpdateNextTime();
    // Battery instance 0 - a separate instance namespace from the N2kts_*
    // temperature source instances used above, so it doesn't collide with them.
    SetN2kDCBatStatus(N2kMsg, 0, batteryVoltage, N2kDoubleNA, N2kDoubleNA, VictronSid.sid());
    NMEA2000.SendMsg(N2kMsg);
#if DEBUG_SERIAL_PRINT
    Serial.print("[127508 DCBatStatus] SID=");
    Serial.print(VictronSid.sid());
    Serial.print(" ");
    printN2kValue(batteryVoltage, " V");
    Serial.println();
#endif
  }
}

void ErrorLoop() {
  #if DEBUG_SERIAL_PRINT
  Serial.println("Error in setup. reboot in 20 seconds");
  #endif
  // Fast blink (100 ms on/off) then reboot rather than hang forever - for an
  // unattended sensor on a boat, rebooting beats going dark. This is the one
  // place the LED blocks: the device is about to reset, so there is no
  // NMEA2000.ParseMessages() left to stall.
  for (uint8_t i = 0; i < 100; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  NVIC_SystemReset();
}
