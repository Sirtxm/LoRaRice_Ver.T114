#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MyVL53L1X.h"

MyVL53L1X sensor(30, 28);
TwoWire *wi = &Wire;
Adafruit_BME280 bme; 

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // ตั้งขา I2C เป็น GPIO30 = SDA, GPIO28 = SCL
  wi->setPins(30, 28);
  wi->begin();

  Serial.println("Initializing BME280...");
  if (!sensor.begin()) {
    Serial.println("Sensor init failed!");
    while (1) delay(100);
  }
  // เริ่มต้นเซนเซอร์โดยส่ง TwoWire pointer
  if (!bme.begin(0x76, wi)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1) delay(100);
  }

  Serial.println("BME280 initialized successfully.");
}

void loop() {
  // อ่านค่าเซนเซอร์
  float temperature = bme.readTemperature(); // °C
  float humidity = bme.readHumidity();       // %
  float pressure = bme.readPressure() / 100.0F; // hPa

  // แสดงผล
  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");
    int d = sensor.readDistance();
  if (d > 0) {
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" mm");
  }
  delay(300);

  delay(2000);
}
