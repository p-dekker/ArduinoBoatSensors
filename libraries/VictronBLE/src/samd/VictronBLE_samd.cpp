/**
 * VictronBLE - Arduino MKR WiFi 1010 BLE scanning backend
 *
 * The MKR WiFi 1010's SAMD21 has no BLE radio of its own; the u-blox
 * NINA-W102 co-processor is reachable from sketch code only through
 * ArduinoBLE over SPI. Extracts the manufacturer data, MAC and RSSI from
 * each scan result and hands them to the platform-independent
 * VictronBLE::onAdvertisement(), same as the ESP32 and nRF52 backends.
 *
 * Tested target: Arduino MKR WiFi 1010.
 *
 * Copyright (c) 2025 Scott Penrose
 * License: MIT
 */
#include "../VictronBLE.h"

#if defined(VICTRON_BACKEND_SAMD_BLE)

// ArduinoBLE has been observed to stop delivering results after a
// long-running scan on this backend; restarting periodically is cheap
// insurance against that.
static constexpr uint32_t RESCAN_INTERVAL_MS = 30000;

bool VictronBLE::begin(uint32_t scanDuration) {
    if (initialized) return true;
    this->scanDuration = scanDuration;  // unused: ArduinoBLE scanning is continuous, not timed

    if (!BLE.begin()) return false;

    initialized = true;
    if (debugEnabled) Serial.println("[VictronBLE] Initialized (SAMD/ArduinoBLE backend)");
    return true;
}

void VictronBLE::loop() {
    if (!initialized) return;

    if (!scanning || (millis() - lastScanStart) > RESCAN_INTERVAL_MS) {
        BLE.stopScan();
        // withDuplicates=true: Victron's payload changes every advertisement,
        // so the default duplicate filter would deliver only the first one.
        scanning = BLE.scan(true) != 0;
        lastScanStart = millis();
    }

    BLEDevice peripheral = BLE.available();
    while (peripheral) {
        processPeripheral(peripheral);
        peripheral = BLE.available();
    }
}

void VictronBLE::processPeripheral(BLEDevice& peripheral) {
    if (debugEnabled) {
        VBLE_PRINTF("[VictronBLE] MAC=%-17s  RSSI=%-4d  ManData=%s\n",
            peripheral.address().c_str(), peripheral.rssi(),
            peripheral.hasManufacturerData() ? "yes" : "no");
    }

    if (!peripheral.hasManufacturerData()) return;

    int len = peripheral.manufacturerDataLength();
    if (len <= 0 || len > 31) return;  // 31: legacy BLE advertisement payload limit

    uint8_t mfg[31];
    peripheral.manufacturerData(mfg, len);

    onAdvertisement(mfg, len, peripheral.address().c_str(), (int8_t)peripheral.rssi());
}

#endif  // VICTRON_BACKEND_SAMD_BLE
