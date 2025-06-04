#include <Arduino.h>
#include <Adafruit_BME280>
#include <Wire.h>
#include <BatteryMonitor.h>

// ===== Pin Definitions =====
#define PIN_BAT_ADC     4
#define PIN_BAT_ADC_CTL 6
#define MY_BAT_AMPLIFY  4.9

// ===== Global Instances =====
TwoWire *wi = &Wire;
Adafruit_BME280 bme;
BatteryMonitor battery(PIN_BAT_ADC, PIN_BAT_ADC_CTL, MY_BAT_AMPLIFY);

// ===== State Machine =====
enum {
  READ_SENSOR = 0;
  LORA_MODE   = 1;
  AUTO_MODE   = 2;
  MANUAL_MODE = 3;
}

int state = READ_SENSOR;
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

    // ===== Init Battery Monitor =====
  Serial.println("Initializing battery monitor...");
  battery.begin();
  Serial.println("Battery monitor initialized.");

  Serial.println("System initialization complete.\n");
}

void loop() {
  if (state == READ_SENSOR) {
    // ==== BME ====
    float temperatureBME = bme.readTemperature();
    float humidityBME = bme.readHumidity();
    Serial.println("BME reading...");
    Serial.printf("Temp: %.2f °C, Humidity: %.2f %%\n", temperatureBME, humidityBME);
    
    // ==== Battery ====
    uint16_t mv = battery.readMillivolts();
    Serial.printf("Battery Voltage: %d mV\n", mv);
    Serial.print("Sensor reading complete.\n");
    state = LORA_MODE;
  }
  else if (state == LORA_MODE){
    
  }
  else if (state == AUTO_MODE){
    
    state = READ_SENSOR;
  }
  else if (state == MANUAL_MODE){
    state = READ_SENSOR;
  }
}
