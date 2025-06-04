#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <BatteryMonitor.h>
#include "heltec_nrf_lorawan.h"

// ===== LoRaWAN OTAA Credentials =====
uint8_t devEui[] = { 0x22, 0x32, 0x33, 0x00, 0x00, 0x99, 0xaa, 0x04 };
uint8_t appEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xF0, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };

// ===== LoRaWAN Settings =====
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_AS923;
DeviceClass_t loraWanClass = CLASS_A;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 1;
uint32_t appTxDutyCycle = 30000;
#define APP_TX_DUTYCYCLE_RND 500

// ===== Pin Definitions =====
#define PIN_BAT_ADC     4
#define PIN_BAT_ADC_CTL 6
#define MY_BAT_AMPLIFY  4.9
#define LED_AUTO_ON     8
#define LED_AUTO_OFF    7
#define RELAY_TRICK_ON  44
#define RELAY_TRICK_OFF 46

// ===== Global Instances =====
TwoWire *wi = &Wire;
Adafruit_BME280 bme;
BatteryMonitor battery(PIN_BAT_ADC, PIN_BAT_ADC_CTL, MY_BAT_AMPLIFY);

// ===== Pump Control Variables =====
bool pumpOnCommand = false;   // รับค่าจาก Downlink

// ===== LoraWAN Payload =====
void prepareTxFrame(uint8_t port) {
  appDataSize = 6;

  int16_t temp = bme.readTemperature() * 100;
  uint16_t humi = bme.readHumidity() * 100;
  uint16_t batt = battery.readMillivolts();

  appData[0] = (temp >> 8) & 0xFF;
  appData[1] = temp & 0xFF;
  appData[2] = (humi >> 8) & 0xFF;
  appData[3] = humi & 0xFF;
  appData[4] = (batt >> 8) & 0xFF;
  appData[5] = batt & 0xFF;
}

// ===== Downlink Control =====
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  if(mcpsIndication->BufferSize >= 1) {
    uint8_t cmd = mcpsIndication->Buffer[0];
    switch(cmd) {
      case 0x00:
        Serial.println("Command: Turn PUMP OFF");
        pumpOnCommand = false;
        break;
      case 0x01:
        Serial.println("Command: Turn PUMP ON");
        pumpOnCommand = true;
        break;
      default:
        Serial.printf("Unknown downlink command: 0x%02X\n", cmd);
        break;
    }
  }
}

// ===== Pump Control Logic =====
void controlPump() {
  if (pumpOnCommand) {
    digitalWrite(LED_AUTO_ON, HIGH);
    digitalWrite(RELAY_TRICK_ON, HIGH);
    digitalWrite(LED_AUTO_OFF, LOW);
    digitalWrite(RELAY_TRICK_OFF, LOW);
  } else {
    digitalWrite(LED_AUTO_OFF, HIGH);
    digitalWrite(RELAY_TRICK_OFF, HIGH);
    digitalWrite(LED_AUTO_ON, LOW);
    digitalWrite(RELAY_TRICK_ON, LOW);
  }
}

void setup() {
  boardInit(LORA_DEBUG_ENABLE, LORA_DEBUG_SERIAL_NUM, 115200);
  debug_printf("Booting Mesh Node Pump Controller...\n");

  pinMode(LED_AUTO_ON, OUTPUT);
  pinMode(LED_AUTO_OFF, OUTPUT);
  pinMode(RELAY_TRICK_ON, OUTPUT);
  pinMode(RELAY_TRICK_OFF, OUTPUT);

  wi->setPins(30, 28);
  wi->begin();
  bme.begin(0x76, wi);
  battery.begin();

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
      prepareTxFrame(appPort);
      LoRaWAN.send();
      controlPump();  // สั่งปั๊มทุกครั้งหลังส่ง
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
