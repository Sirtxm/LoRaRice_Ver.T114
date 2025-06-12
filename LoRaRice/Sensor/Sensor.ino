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
uint8_t devEui[] = {0x70, 0xB3, 0xD5, 0x7E, 0xD8, 0x00, 0x41, 0x3D};
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
uint8_t appKey[] = {0xDE, 0x67, 0x90, 0x7C, 0x63, 0x5C, 0x79, 0x33, 0xA4, 0xD3, 0xC4, 0x4F, 0x12, 0x56, 0x0C, 0x50};

/* ABP para*/
uint8_t nwkSKey[] = {0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85};
uint8_t appSKey[] = {0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67};
uint32_t devAddr = (uint32_t)0x007e6ae1;

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

#define GPS_RX 9   // GPS TX ต่อเข้าขานี้
#define GPS_TX 10  // ไม่ได้ใช้ก็ได้

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
      return;
    }
  }

  distance = vl53.distance();
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

  // GPS 
  unsigned long gpsStart = millis();
  bool gpsValid = false;
  while (millis() - gpsStart < 3000) {
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
  uint16_t dist = distanceVL * 10;
  uint16_t batt = batteryVoltage;
  int32_t lat = latitude * 1e6;
  int32_t lon = longitude * 1e6;

  appData[0] = (temp >> 8) & 0xFF;
  appData[1] = temp & 0xFF;
  appData[2] = (humi >> 8) & 0xFF;
  appData[3] = humi & 0xFF;
  appData[4] = (dist >> 8) & 0xFF;
  appData[5] = dist & 0xFF;
  appData[6] = (batt >> 8) & 0xFF;
  appData[7] = batt & 0xFF;

  appData[8]  = (lat >> 24) & 0xFF;
  appData[9]  = (lat >> 16) & 0xFF;
  appData[10] = (lat >> 8) & 0xFF;
  appData[11] = lat & 0xFF;

  appData[12] = (lon >> 24) & 0xFF;
  appData[13] = (lon >> 16) & 0xFF;
  appData[14] = (lon >> 8) & 0xFF;
  appData[15] = lon & 0xFF;
  Serial.println("Payload prepared for LoRaWAN:");
  Serial.printf("Temp: %d, Humi: %d, Dist: %d, Batt: %d, Lat: %.6f, Lon: %.6f\n",
              temp, humi, dist, batt,
              latitude, longitude);


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


void setup() {
  boardInit(LORA_DEBUG_ENABLE, LORA_DEBUG_SERIAL_NUM, 115200);
  debug_printf("Booting Mesh Node T114...\n");

  gpsSerial.begin(9600);
  Serial.println("GPS Module Ready");

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
  
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

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