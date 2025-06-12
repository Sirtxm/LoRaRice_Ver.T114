#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <BatteryMonitor.h>
#include "heltec_nrf_lorawan.h"

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
bool isTxConfirmed = true;
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
  STATE_IDLE = 0,
  STATE_PUMP_AUTO = 1,
  STATE_PUMP_MANUAL = 2
};
PumpState pumpState;
uint8_t pumpCmd;

// ===== TIME VARIABLES =====
unsigned long relayActionStartTime = 0;
bool relayActionPending = false;

// ===== LoraWAN Payload =====
static void prepareTxFrame(uint8_t port) {
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

  Serial.println("Payload prepared for LoRaWAN:");
  Serial.printf("Temp: %.2f °C, Humi: %.2f %%RH,  Batt: %.3f V\n",
              temp / 100.0, humi / 100.0, batt / 1000.0);
}

// ===== Pump Control Logic =====
void controlPump(PumpState state, uint8_t cmd) {

    if (state == STATE_PUMP_AUTO) {
      Serial.println("[controlPump] Mode: AUTO");

      if (cmd == 0x01) {
        Serial.println("[controlPump] -> Pump ON (AUTO)");
        digitalWrite(LED_AUTO_ON, LOW);
        digitalWrite(RELAY_TRICK_ON, LOW);
        delay(1000);
        digitalWrite(LED_AUTO_ON, HIGH);
        digitalWrite(RELAY_TRICK_ON, HIGH);
      } else {
        Serial.println("[controlPump] -> Pump OFF (AUTO)");
        digitalWrite(LED_AUTO_OFF, LOW);
        digitalWrite(RELAY_TRICK_OFF, LOW);
        delay(1000);
        digitalWrite(LED_AUTO_OFF, HIGH);
        digitalWrite(RELAY_TRICK_OFF, HIGH);
      }

    } else if (state == STATE_PUMP_MANUAL) {
      Serial.println("[controlPump] Mode: MANUAL");

      if (cmd == 0x01) {
        Serial.println("[controlPump] -> Pump ON (MANUAL)");
        digitalWrite(RELAY_TRICK_ON, LOW);
        delay(1000);
         digitalWrite(RELAY_TRICK_ON, HIGH);
      } else {
        Serial.println("[controlPump] -> Pump OFF (MANUAL)");
        digitalWrite(RELAY_TRICK_OFF, LOW);
        delay(1000);
        digitalWrite(RELAY_TRICK_OFF, HIGH);
      }
    }
}


// ===== Downlink Control =====
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{

  if(mcpsIndication->BufferSize >= 2) {
    uint8_t mode = mcpsIndication->Buffer[0];
    uint8_t cmd = mcpsIndication->Buffer[1];
    
    Serial.print(">>> Downlink Received: ");
    Serial.print("mode=0x");
    if (mode < 0x10) Serial.print("0");
    Serial.print(mode, HEX);
    Serial.print(", cmd=0x");
    if (cmd < 0x10) Serial.print("0");
    Serial.println(cmd, HEX);

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

   // <<< INITIAL STATES TO AVOID UNINTENDED LOWS >>>
  digitalWrite(LED_AUTO_ON, HIGH); 
  digitalWrite(LED_AUTO_OFF, HIGH);
  digitalWrite(RELAY_TRICK_ON, HIGH);  
  digitalWrite(RELAY_TRICK_OFF, HIGH);

  pumpState = STATE_IDLE;
  pumpCmd = 0x00;

  wi->setPins(30, 28);
  wi->begin();
  Serial.println("I2C started on SDA: 30, SCL: 28");
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
