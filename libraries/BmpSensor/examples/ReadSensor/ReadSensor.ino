// BmpSensor example - prints temperature and pressure once per second.
//
// The sensor state itself refreshes every 2000 ms inside sensor.loop(), so
// half of these lines report the same reading with a growing age.

#include <BmpSensor.h>

BmpSensor sensor;

uint32_t lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    ;  // wait briefly for the USB serial monitor, then carry on regardless
  }

  if (sensor.setup()) {
    Serial.print("BMP280 found at 0x");
    Serial.println(sensor.i2cAddress(), HEX);
  } else {
    Serial.println("No BMP280 on 0x77 or 0x76 - check wiring.");
  }
}

void loop() {
  sensor.loop();

  if ((uint32_t)(millis() - lastPrint) >= 1000) {
    lastPrint = millis();

    if (sensor.isValid()) {
      Serial.print(sensor.temperatureCelsius(), 2);
      Serial.print(" C  ");
      Serial.print(sensor.pressure() / 100.0, 2);
      Serial.print(" hPa  (age ");
      Serial.print(sensor.ageMs());
      Serial.println(" ms)");
    } else {
      Serial.print("no valid reading - consecutive failures: ");
      Serial.println(sensor.consecutiveFailures());
    }
  }
}
