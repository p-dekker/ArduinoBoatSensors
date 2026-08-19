// DsSensor example - prints outside temperature once per second.
//
// The sensor state itself refreshes every 2000 ms inside sensor.loop(), so
// most of these lines report the same reading with a growing age.

#include <DsSensor.h>

DsSensor sensor(2);  // data pin, with a 4.7k pull-up to the sensor's VDD

uint32_t lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    ;  // wait briefly for the USB serial monitor, then carry on regardless
  }

  if (sensor.setup()) {
    Serial.println("DS18x20 found on the 1-Wire bus.");
  } else {
    Serial.println("No DS18x20 found - check wiring and pull-up resistor.");
  }
}

void loop() {
  sensor.loop();

  if ((uint32_t)(millis() - lastPrint) >= 1000) {
    lastPrint = millis();

    if (sensor.isValid()) {
      Serial.print(sensor.temperatureCelsius(), 2);
      Serial.print(" C  (age ");
      Serial.print(sensor.ageMs());
      Serial.println(" ms)");
    } else {
      Serial.print("no valid reading - consecutive failures: ");
      Serial.println(sensor.consecutiveFailures());
    }
  }
}
