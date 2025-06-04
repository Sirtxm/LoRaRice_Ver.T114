#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_SHT4x.h"

// ===== Global Instances =====
TwoWire *wi = &Wire;
Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31(wi);
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// ===== Sensor Selection =====
enum {
  BME = 0,
  SHT = 1,
  SHT4 = 2
};
int sensor = BME;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Starting Temperature/Humidity Measurement...");

  // Init I2C Bus
  wi->setPins(30, 28);
  wi->begin();
  Serial.println("I2C started on SDA: 30, SCL: 28");

  // Init BME280
  if (!bme.begin(0x76, wi)) {
    Serial.println("[ERROR] BME280 not found at 0x76.");
    while (1) delay(100);
  }
  Serial.println("BME280 initialized successfully.");

  // Init SHT31
  if (!sht31.begin(0x44)) {
    Serial.println("SHT31 sensor not found");
    while (1) delay(1);
  }
  Serial.println("SHT31 sensor ready!");

  // Init SHT4x
  if (!sht4.begin()) {
    Serial.println("SHT4x sensor not found");
    while (1) delay(10);
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
  Serial.println("SHT4x initialized successfully.");
}

void loop() {
  if (Serial.available()) {
    char key = Serial.read();

    if (key == '0') {
      sensor = BME;
      Serial.println("[Keyboard] Sensor switched to: BME280");
    } else if (key == '1') {
      sensor = SHT;
      Serial.println("[Keyboard] Sensor switched to: SHT31");
    } else if (key == '2') {
      sensor = SHT4;
      Serial.println("[Keyboard] Sensor switched to: SHT4x");
    }
  }

  Serial.println("Reading sensor...");
  if (sensor == BME) {
    float temperatureBME = bme.readTemperature();
    float humidityBME = bme.readHumidity();
    Serial.printf("BME280 -> Temp: %.2f °C, Humidity: %.2f %%\n", temperatureBME, humidityBME);
  } else if (sensor == SHT) {
    float tempSHT = sht31.readTemperature();
    float humSHT = sht31.readHumidity();
    Serial.printf("SHT31 -> Temp: %.2f °C, Humidity: %.2f %%\n", tempSHT, humSHT);
  } else if (sensor == SHT4) {
    sensors_event_t tempSHT4, humSHT4;
    sht4.getEvent(&tempSHT4, &humSHT4);
    Serial.printf("SHT4x -> Temp: %.2f °C, Humidity: %.2f %%\n", tempSHT4.temperature, humSHT4.relative_humidity);
  }

  delay(2000); // รอ 2 วินาทีแล้วอ่านใหม่
}
