#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <BatteryMonitor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include "Adafruit_VL53L1X.h"
#include "heltec_nrf_lorawan.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// ===== LoRaWAN OTAA Credentials =====
uint8_t devEui[] = {0x70, 0xB3, 0xD5, 0x7E, 0xD8, 0x00, 0x4F, 0x3F};
uint8_t appEui[] = {0xAA, 0xBB, 0xCC, 0x11, 0x11, 0x11, 0x11, 0x11};
uint8_t appKey[] = {0xCF, 0x51, 0x8D, 0xB1, 0x01, 0xB4, 0x9D, 0x44, 0xF4, 0x76, 0x72, 0xD9, 0xC7, 0xFC, 0x76, 0x3F};

/* ABP para*/
uint8_t nwkSKey[] = {0xE0, 0x70, 0x80, 0x08, 0x70, 0xE5, 0x31, 0x94, 0x29, 0x75, 0xCA, 0xFB, 0x6E, 0x27, 0x95, 0xA9};
uint8_t appSKey[] = {0xD1, 0x1D, 0x6D, 0xF4, 0x7A, 0x99, 0x51, 0xA9, 0xC0, 0xCB, 0xB5, 0x43, 0x37, 0xD2, 0x85, 0x63};
uint32_t devAddr = (uint32_t)0x27FC8281;

// ===== LoRaWAN Settings =====
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_AS923;  
DeviceClass_t loraWanClass = CLASS_A;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 8;
uint32_t appTxDutyCycle = 15 * 60 * 1000;
// uint32_t appTxDutyCycle = 15000;
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

#define APP_TX_DUTYCYCLE_RND 1000 

// ===== Define Value =====
double adjustedDistance = 0;
double distance = 0;

// ===== Pin Definitions =====
#define PIN_VEXT_CTRL    21   // power
#define PIN_BAT_ADC      4    // GPIO4
#define PIN_BAT_ADC_CTL  6    // GPIO6
#define MY_BAT_AMPLIFY   4.9

#define GPS_RX 9   
#define GPS_TX 10  

// Global variables
float temperatureBME = 0;
float humidityBME = 0;
float distanceVL = 0;
uint16_t batteryVoltage = 0;
double latitude = 0;
double longitude = 0;

// ===== Global Instances =====
TwoWire *wi = &Wire;
Adafruit_BME280 bme;
BatteryMonitor battery(PIN_BAT_ADC, PIN_BAT_ADC_CTL, MY_BAT_AMPLIFY);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;

void prepareTxFrame(uint8_t port) {
  appDataSize = 16;

  Serial.println("Reading sensors...");

  // VL53L1X
  unsigned long startTime = millis();
    while (!vl53.dataReady()) {
    if (millis() - startTime > 1000) {
      Serial.println("VL53L1X Timeout.");
      break;
    }
  }
  if (vl53.dataReady()) {
    distance = vl53.distance();
    vl53.clearInterrupt();
  } else {
    distance = -1;
  }

  if (distance < 0) {
    Serial.print(F("Couldn't get distance: "));
    Serial.println(vl53.vl_status);

    distanceVL = -99;
  } else {
    adjustedDistance = 500.0 - distance;
    distanceVL = adjustedDistance;
  }
  // BME280
  temperatureBME = bme.readTemperature();
  humidityBME = bme.readHumidity();
  Serial.printf("Temp: %.2f °C, Humidity: %.2f %%\n", temperatureBME, humidityBME);


  // Battery
  batteryVoltage = battery.readMillivolts();
  Serial.printf("Battery Voltage: %d mV\n", batteryVoltage);

  // GPS 
  unsigned long gpsStart = millis();
  bool gpsValid = false;
  while (millis() - gpsStart < 60000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated() && gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      gpsValid = true;
      break;
    }
  }

  if (!gpsValid) {
    Serial.println("GPS not valid, skipping lat/lon encode.");
    latitude = 0;
    longitude = 0;
  }
  Serial.println("Sensor reading complete.");

  // ===== Move encoding AFTER sensor update =====
  int16_t temp = temperatureBME * 100;
  uint16_t humi = humidityBME * 100;
  int16_t dist = distanceVL;
  uint16_t batt = batteryVoltage / 10;
  int32_t lat = latitude * 1e6;
  int32_t lon = longitude * 1e6;

  // temp
  appData[0] = (temp >> 8) & 0xFF;
  appData[1] = temp & 0xFF;

  //hump
  appData[2] = (humi >> 8) & 0xFF;
  appData[3] = humi & 0xFF;

  //dist
  appData[4] = (dist >> 8) & 0xFF;
  appData[5] = dist & 0xFF;

  //batt
  appData[6] = (batt >> 8) & 0xFF;
  appData[7] = batt & 0xFF;

  //lat
  appData[8]  = (lat >> 24) & 0xFF;
  appData[9]  = (lat >> 16) & 0xFF;
  appData[10] = (lat >> 8) & 0xFF;
  appData[11] = lat & 0xFF;

  // lon
  appData[12] = (lon >> 24) & 0xFF;
  appData[13] = (lon >> 16) & 0xFF;
  appData[14] = (lon >> 8) & 0xFF;
  appData[15] = lon & 0xFF;
  Serial.println("Payload prepared for LoRaWAN:");
  Serial.printf("Temp: %d, Humi: %d, Dist: %d, Batt: %d, Lat: %.6f, Lon: %.6f\n",
              temp, humi, dist, batt,
              latitude, longitude);
}

void setupSensor(){
  digitalWrite(PIN_VEXT_CTRL, HIGH);
  delay(100);
  //BME
  bme.begin(0x76, wi);
  //VL53L1X
  vl53.begin(0x29, wi);
  vl53.VL53L1X_SetROI(5,5);
  vl53.startRanging();
}

void setup() {
  boardInit(LORA_DEBUG_ENABLE, LORA_DEBUG_SERIAL_NUM, 115200);
  debug_printf("Booting Mesh Node T114...\n");

  //setup vext_ctrl
  pinMode(PIN_VEXT_CTRL, OUTPUT);
  digitalWrite(PIN_VEXT_CTRL, HIGH);

  gpsSerial.begin(9600);
  Serial.println("GPS Module Ready");

  // ===== Init I2C Bus =====
  Serial.println("Setting up I2C... SCL31 SDA 29");
  wi->setPins(29, 31);
  wi->begin();

  // ===== Init BME280 =====
  bme.begin(0x76, wi);

  // ===== Init VL53L1X =====
  vl53.begin(0x29, wi);
  vl53.VL53L1X_SetROI(5,5);
  vl53.startRanging();

  // ===== Init Battery Monitor =====
  Serial.println("Initializing battery monitor...");
  battery.begin();
  Serial.println("Battery monitor initialized.");

  Serial.println("System initialization complete.\n");
  deviceState = DEVICE_STATE_INIT;
}

void loop() {
  
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  
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
      setupSensor();
      prepareTxFrame(appPort);
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;

    case DEVICE_STATE_CYCLE:
      txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      digitalWrite(PIN_VEXT_CTRL, LOW);
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