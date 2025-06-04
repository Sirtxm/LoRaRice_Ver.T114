#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <BatteryMonitor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include "heltec_nrf_lorawan.h"

// ===== LoRaWAN OTAA Credentials =====
uint8_t devEui[] = { 0x22, 0x32, 0x33, 0x00, 0x00, 0x99, 0xaa, 0x04 };
uint8_t appEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xF0, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };
uint8_t appSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };
uint32_t devAddr = (uint32_t)0x007e6ae4;

// ===== LoRaWAN Settings =====
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_AS923;  
DeviceClass_t loraWanClass = CLASS_A;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 1;
uint32_t appTxDutyCycle = 15000;

uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

#define APP_TX_DUTYCYCLE_RND 1000 

// ===== Define Value =====
double adjustedDistance = 0;
double distance = 0;

// ===== Pin Definitions =====
#define PIN_BAT_ADC      4    // GPIO4
#define PIN_BAT_ADC_CTL  6    // GPIO6
#define MY_BAT_AMPLIFY   4.9

// Global variables
float temperatureBME = 0;
float humidityBME = 0;
float distanceVL = 0;
uint16_t batteryVoltage = 0;

// ===== Global Instances =====
TwoWire *wi = &Wire;
Adafruit_BME280 bme;
BatteryMonitor battery(PIN_BAT_ADC, PIN_BAT_ADC_CTL, MY_BAT_AMPLIFY);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();


void prepareTxFrame(uint8_t port) {
  appDataSize = 8;

  int16_t temp = temperatureBME * 100;
  uint16_t humi = humidityBME * 100;
  uint16_t dist = distanceVL * 10;
  uint16_t batt = batteryVoltage;

  appData[0] = (temp >> 8) & 0xFF;
  appData[1] = temp & 0xFF;
  appData[2] = (humi >> 8) & 0xFF;
  appData[3] = humi & 0xFF;
  appData[4] = (dist >> 8) & 0xFF;
  appData[5] = dist & 0xFF;
  appData[6] = (batt >> 8) & 0xFF;
  appData[7] = batt & 0xFF;
  
  Serial.println("Payload prepared for LoRaWAN:");
  Serial.printf("Temp: %d, Humi: %d, Dist: %d, Batt: %d\n", temp, humi, dist, batt);
}

void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("Downlink received | Port: %d | Size: %d\n", mcpsIndication->Port, mcpsIndication->BufferSize);

  Serial.print("Payload: ");
  for(uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    Serial.printf("%02X ", mcpsIndication->Buffer[i]);
  }
  Serial.println();

  if(mcpsIndication->BufferSize >= 1) {
    uint8_t cmd = mcpsIndication->Buffer[0];
  switch (cmd) {
        case 0x00:
          Serial.println("Command: Enter Debug Mode");
          break;
        case 0x01:
          Serial.println("Command: Enter Normal Mode");
          break;
        default:
          Serial.printf("Unknown command: 0x%02X\n", cmd);
          break;
      }
  }
}


void readSensors() {
  Serial.println("Reading sensors...");

  // VL53L1X
  unsigned long startTime = millis();
  while (!vl53.dataReady()) {
    if (millis() - startTime > 500) {
      Serial.println("VL53L1X Timeout.");
      return;
    }
  }

  distance = vl53.distance() / 10.0;
  if (distance == -1) {
    Serial.print(F("Couldn't get distance: "));
    Serial.println(vl53.vl_status);
    return;
  }

  adjustedDistance = 50.0 - distance;
  distanceVL = adjustedDistance;

  // BME280
  temperatureBME = bme.readTemperature();
  humidityBME = bme.readHumidity();
  Serial.printf("Temp: %.2f °C, Humidity: %.2f %%\n", temperatureBME, humidityBME);

  // Battery
  batteryVoltage = battery.readMillivolts();
  Serial.printf("Battery Voltage: %d mV\n", batteryVoltage);

  Serial.println("Sensor reading complete.");
}

void setup() {
  boardInit(LORA_DEBUG_ENABLE, LORA_DEBUG_SERIAL_NUM, 115200);
  debug_printf("Booting Mesh Node T114...\n");

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

   if (!vl53.begin(0x29, wi)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

   if (vl53.VL53L1X_SetROI(5,5) == 0) {   //(ROIcenter,ROIsize)
    Serial.println(F("ROI successed"));
  } else {
    Serial.println(F("ROI Failure"));
  }
  Serial.println("VL53L1X ready.");

  // ===== Init Battery Monitor =====
  Serial.println("Initializing battery monitor...");
  battery.begin();
  Serial.println("Battery monitor initialized.");

  Serial.println("System initialization complete.\n");
  deviceState = DEVICE_STATE_INIT;
}

void loop() {
  static unsigned long lastTxTime = 0;
  unsigned long now = millis();

  switch (deviceState) {
    case DEVICE_STATE_INIT:
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      deviceState = DEVICE_STATE_JOIN;
      break;

    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      break;

    case DEVICE_STATE_SEND:
      readSensors();
      prepareTxFrame(appPort);
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;

    case DEVICE_STATE_CYCLE:
      lastTxTime = now;
      txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;

    case DEVICE_STATE_SLEEP:
      LoRaWAN.sleep(loraWanClass);
      break;

    default:
      deviceState = DEVICE_STATE_INIT;
      break;
  }
}