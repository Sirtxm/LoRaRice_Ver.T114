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
enum PumpState{
  STATE_PUMP_AUTO = 0,
  STATE_PUMP_MANUAL = 1
};
PumpState pumpState;
uint8_t pumpCmd;

// ===== TIME VARIABLES =====
unsigned long relayActionStartTime = 0;
bool relayActionPending = false;

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

// ===== Pump Control Logic =====
void controlPump(PumpState state, uint8_t cmd) {
  if (!relayActionPending) {
    if (state == STATE_PUMP_AUTO) {
      if (cmd == 0x01) {
        digitalWrite(LED_AUTO_ON, LOW);
        digitalWrite(RELAY_TRICK_ON, LOW);
      } else {
        digitalWrite(LED_AUTO_OFF, LOW);
        digitalWrite(RELAY_TRICK_OFF, LOW);
      }
    } else if (state == STATE_PUMP_MANUAL) {
      if (cmd == 0x01) {
        digitalWrite(RELAY_TRICK_ON, LOW);
      } else {
        digitalWrite(RELAY_TRICK_OFF, LOW);
      }
    }
    relayActionStartTime = millis();
    relayActionPending = true;
  }

  if (relayActionPending && (millis() - relayActionStartTime >= 1000)) {
    if (state == STATE_PUMP_AUTO) {
      if (cmd == 0x01) {
        digitalWrite(LED_AUTO_ON, HIGH);
        digitalWrite(RELAY_TRICK_ON, HIGH);
      } else {
        digitalWrite(LED_AUTO_OFF, HIGH);
        digitalWrite(RELAY_TRICK_OFF, HIGH);
      }
    } else if (state == STATE_PUMP_MANUAL) {
      if (cmd == 0x01) {
        digitalWrite(RELAY_TRICK_ON, HIGH);
      } else {
        digitalWrite(RELAY_TRICK_OFF, HIGH);
      }
    }
    relayActionPending = false;
  }
}

// ===== Downlink Control =====
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  if(mcpsIndication->BufferSize >= 2) {
    uint8_t mode = mcpsIndication->Buffer[0];
    uint8_t cmd = mcpsIndication->Buffer[1];
    switch(mode) {
      case 0x00:
        Serial.println("Mode: Auto Level Trigger");
        pumpState = STATE_PUMP_AUTO;
        pumpCmd = cmd;
        controlPump(pumpState, pumpCmd);
        break;
      case 0x01:
        Serial.println("Mode: Remote manual");
        pumpState = STATE_PUMP_MANUAL;
        pumpCmd = cmd;
        controlPump(pumpState, pumpCmd);
        break;
      default:
        Serial.printf("Unknown downlink command: 0x%02X\n", cmd);
        break;
    }
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
      controlPump(pumpState, pumpCmd);  
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
