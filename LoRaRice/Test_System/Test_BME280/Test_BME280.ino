#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  while (!Serial);  

  Serial.println(F("BME280 Test"));

  if (!bme.begin(0x76)) {
    Serial.println("ไม่พบ BME280 โปรดตรวจสอบการเชื่อมต่อหรือเปลี่ยนเป็น address 0x77");
    while (1); 
  }
}

void loop() {
  float temperature = bme.readTemperature();       
  float humidity = bme.readHumidity();             


  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" % ");
 

  delay(2000); // รอ 2 วินาทีก่อนอ่านค่าใหม่
}
