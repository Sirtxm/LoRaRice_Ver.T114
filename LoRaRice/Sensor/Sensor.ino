#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <BatteryMonitor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include "MyVL53L1X.h"

// ===== Pin Definitions =====
#define PIN_BAT_ADC      4    // GPIO4
#define PIN_BAT_ADC_CTL  6    // GPIO6
#define MY_BAT_AMPLIFY   2.0

// ===== Global Instances =====
TwoWire *wi = &Wire;
Adafruit_BME280 bme;
MyVL53L1X sensor(30, 28);
BatteryMonitor battery(PIN_BAT_ADC, PIN_BAT_ADC_CTL, MY_BAT_AMPLIFY);

// ===== State Machine =====
enum {
  READ_SENSOR = 0,
  LORA_SEND_SENSOR = 1,
  SLEEP_MODE = 2
};
int state = READ_SENSOR;

// ===== Timing Configurations =====
unsigned long lastSensorTime = 0;
unsigned long sensorInterval = 2000; // ms

unsigned long lastLoraTime = 0;
unsigned long loraInterval = 1000;

unsigned long lastSleepTime = 0;
unsigned long sleepInterval = 1000;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Booting Mesh Node T114...");

  // ===== Init I2C Bus =====
  Serial.println("Setting up I2C...");
  wi->setPins(30, 28);
  wi->begin();
  Serial.println("I2C started on SDA: 30, SCL: 28");

  // ===== Init BME280 =====
  Serial.println("Initializing BME280...");
  if (!bme.begin(0x76, wi)) {
    Serial.println("[ERROR] BME280 not found at 0x76. Check wiring or address jumper!");
    while (1) delay(100);
  }
  Serial.println("BME280 initialized successfully.");

  // ===== Init VL53L1X =====
  Serial.println("Initializing VL53L1X...");
  if (!sensor.begin()) {
    Serial.println("[ERROR] VL53L1X initialization failed!");
    while (1) delay(100);
  }
  Serial.println("VL53L1X ready.");

  // ===== Init Battery Monitor =====
  Serial.println("Initializing battery monitor...");
  battery.begin();
  Serial.println("Battery monitor initialized.");

  Serial.println("System initialization complete.\n");
}

void loop() {
  unsigned long now = millis();

  if (state == READ_SENSOR) {
    if (now - lastSensorTime >= sensorInterval) {
      lastSensorTime = now;
      Serial.println("Reading sensors...");

      int distance = sensor.readDistance();
      if (distance > 0) {
        Serial.printf("Distance: %d mm\n", distance);
      } else {
        Serial.println("[WARN] Distance read failed or too close.");
      }

      float temperature = bme.readTemperature();
      float humidity = bme.readHumidity();
      Serial.printf("Temp: %.2f °C, Humidity: %.2f %%\n", temperature, humidity);

      uint16_t mv = battery.readMillivolts();
      Serial.printf("Battery Voltage: %d mV\n", mv);

      Serial.print("Sensor reading complete.\n");
      state = LORA_SEND_SENSOR;
    }

  } else if (state == LORA_SEND_SENSOR) {
    if (now - lastLoraTime >= loraInterval) {
      lastLoraTime = now;
      Serial.println("Preparing LoRa payload.");
      state = SLEEP_MODE;
    }

  } else if (state == SLEEP_MODE) {
    if (now - lastSleepTime >= sleepInterval) {
      lastSleepTime = now;
      Serial.println("Entering sleep mode...");
      state = READ_SENSOR;
    }
  }
}
