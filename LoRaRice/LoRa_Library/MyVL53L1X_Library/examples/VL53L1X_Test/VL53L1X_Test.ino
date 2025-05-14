#include "MyVL53L1X.h"

MyVL53L1X sensor(30, 28);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!sensor.begin()) {
    Serial.println("Sensor init failed!");
    while (1) delay(100);
  }
  Serial.println("Sensor ready!");
}

void loop() {
  int d = sensor.readDistance();
  if (d > 0) {
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" mm");
  }
  delay(300);
}
