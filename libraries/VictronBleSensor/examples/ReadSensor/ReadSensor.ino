// VictronBleSensor example - prints battery voltage once per second.
//
// Fill in MAC and HEX_KEY from the VictronConnect app (Settings > Product
// info > Instant readout via Bluetooth > Encryption key) before this
// reports real data. Requires an Arduino MKR WiFi 1010 (or another board
// VictronBLE has a backend for).

#include <VictronBleSensor.h>

static const char* MAC = "aabbccddeeff";
static const char* HEX_KEY = "000102030405060708090a0b0c0d0e0f";

VictronBleSensor sensor(MAC, HEX_KEY, "SmartShunt");

uint32_t lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    ;  // wait briefly for the USB serial monitor, then carry on regardless
  }

  if (!sensor.setup()) {
    Serial.println("VictronBleSensor setup failed - check the BLE radio and MAC/key.");
  }
}

void loop() {
  sensor.loop();

  if ((uint32_t)(millis() - lastPrint) >= 1000) {
    lastPrint = millis();

    if (sensor.isValid()) {
      Serial.print(sensor.batteryVoltage(), 2);
      Serial.println(" V");
    } else {
      Serial.println("no valid reading yet");
    }
  }
}
