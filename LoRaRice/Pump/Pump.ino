#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <BatteryMonitor.h>
#include "heltec_nrf_lorawan.h"

// ===== LoRaWAN OTAA Credentials =====
uint8_t devEui[] = {0x70, 0xB3, 0xD5, 0x7E, 0xD8, 0x00, 0x4E, 0xD7};
uint8_t appEui[] = {0xAA, 0xAA, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
uint8_t appKey[] = {0x7F, 0xCD, 0x8A, 0x66, 0x37, 0xC3, 0xDE, 0x76, 0xF0, 0x73, 0xFA, 0x60, 0xF9, 0xAC, 0x88, 0xCB};

/* ABP para*/
uint8_t nwkSKey[] = {0xD2, 0xCE, 0x33, 0xCD, 0xA0, 0x14, 0x63, 0x9C, 0x29, 0x9D, 0x8F, 0xD4, 0x9B, 0x7A, 0xBB, 0x3A};
uint8_t appSKey[] = {0x73, 0xA4, 0xB3, 0xB8, 0xA1, 0x36, 0x6F, 0x4A, 0x61, 0x16, 0x79, 0x64, 0x49, 0x9C, 0xA7, 0xCD};
uint32_t devAddr = (uint32_t)0x27FC828A;

// ===== LoRaWAN Settings =====
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_AS923;  
DeviceClass_t loraWanClass = CLASS_C;
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

#define ACTIVE_PUMP  8
#define LED_AUTO     13
#define RELAY_ON     16
#define RELAY_OFF    33

// ===== Global Instances =====
TwoWire *wi = &Wire;
Adafruit_BME280 bme;
BatteryMonitor battery(PIN_BAT_ADC, PIN_BAT_ADC_CTL, MY_BAT_AMPLIFY);
int active ;
// ===== Pump Control Variables =====
enum PumpState{
  STATE_IDLE = 0,
  STATE_PUMP_AUTO = 1,
  STATE_PUMP_MANUAL = 2
};
PumpState pumpState;
PumpState lastPumpState;

uint8_t pumpCmd = STATE_IDLE;
uint8_t lastPumpCmd = 0xFF;

int stateLed = 0;

// ===== LoraWAN Payload =====
static void prepareTxFrame(uint8_t port) {
  appDataSize = 7;

  active = digitalRead(ACTIVE_PUMP);

  int16_t temp = bme.readTemperature() * 100;
  uint16_t humi = bme.readHumidity() * 100;
  uint16_t batt = battery.readMillivolts();

  appData[0] = (temp >> 8) & 0xFF;
  appData[1] = temp & 0xFF;
  appData[2] = (humi >> 8) & 0xFF;
  appData[3] = humi & 0xFF;
  appData[4] = (batt >> 8) & 0xFF;
  appData[5] = batt & 0xFF;
  appData[6] = active & 0x01;

  Serial.println("Payload prepared for LoRaWAN:");
  Serial.printf("Temp: %d, Humi: %d, Batt: %d, Active: %d\n",
                temp, humi, batt, active);
}

// ===== Pump Control Logic =====
void controlPump(PumpState state, uint8_t cmd) {

    if (state == STATE_PUMP_AUTO) {
      Serial.println("[controlPump] Mode: AUTO");

      if (cmd == 0x01) {
        Serial.println("[controlPump] -> Pump ON (AUTO)");
        digitalWrite(RELAY_ON, LOW);    //Relay on
        delay(1000);
        digitalWrite(RELAY_ON, HIGH); //Relay on(close)
      } else if (cmd == 0x00){
        Serial.println("[controlPump] -> Pump OFF (AUTO)");
        digitalWrite(RELAY_OFF, LOW);
        delay(1000);
        digitalWrite(RELAY_OFF, HIGH);
      } 

    } else if (state == STATE_PUMP_MANUAL) {
      Serial.println("[controlPump] Mode: MANUAL");
      digitalWrite(LED_AUTO, HIGH);   //Led Auto off

      if (cmd == 0x01) {
        Serial.println("[controlPump] -> Pump ON (MANUAL)");
        digitalWrite(RELAY_ON, LOW);
        delay(1000);
        digitalWrite(RELAY_ON, HIGH);
      } else if(cmd == 0x00){
        Serial.println("[controlPump] -> Pump OFF (MANUAL)");
        digitalWrite(RELAY_OFF, LOW);
        delay(1000);
        digitalWrite(RELAY_OFF, HIGH);
      }
    }
}


// ===== Downlink Control =====
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  if (mcpsIndication->BufferSize >= 2) {
    uint8_t mode = mcpsIndication->Buffer[0];
    uint8_t cmd = mcpsIndication->Buffer[1];
    
    Serial.print(">>> Downlink Received: ");
    PumpState newPumpState;
    switch(mode) {
      case 0x00:
        Serial.println("Mode: Auto Level Trigger");
        digitalWrite(LED_AUTO, LOW); 
        newPumpState = STATE_PUMP_AUTO;
        break;
      case 0x01:
        Serial.println("Mode: Remote manual");
        newPumpState = STATE_PUMP_MANUAL;
        break;
      default:
        Serial.printf("Unknown downlink command: 0x%02X\n", cmd);
        return;  
    }
    controlPump(newPumpState, cmd);
  // detect same state
  //   if (newPumpState != lastPumpState || cmd != lastPumpCmd) {
  //     lastPumpState = newPumpState;
  //     lastPumpCmd = cmd;

  //     pumpState = newPumpState;
  //     pumpCmd = cmd;
  //     controlPump(pumpState, pumpCmd);
  //   } else {
  //     Serial.println(">>> Duplicate command detected — ignored.");
  //   }
  // }
  }
}

void setup() {
  boardInit(LORA_DEBUG_ENABLE, LORA_DEBUG_SERIAL_NUM, 115200);
  debug_printf("Booting Mesh Node Pump Controller...\n");

  pinMode(ACTIVE_PUMP, INPUT);

  pinMode(LED_AUTO, OUTPUT);
  pinMode(RELAY_ON, OUTPUT);
  pinMode(RELAY_OFF, OUTPUT);

// =========== Open Drain ==============
    nrf_gpio_cfg(
    digitalPinToPinName(LED_AUTO),
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_S0D1, // S0D1 = Standard 0, Disconnect 1 (Open Drain)
    NRF_GPIO_PIN_NOSENSE
  );

    nrf_gpio_cfg(
    digitalPinToPinName(RELAY_ON),
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_S0D1, // S0D1 = Standard 0, Disconnect 1 (Open Drain)
    NRF_GPIO_PIN_NOSENSE
  );

    nrf_gpio_cfg(
    digitalPinToPinName(RELAY_OFF),
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_S0D1, // S0D1 = Standard 0, Disconnect 1 (Open Drain)
    NRF_GPIO_PIN_NOSENSE
  );
  Serial.println("Pin configured as Open Drain (S0D1)");

  pumpState = STATE_IDLE;
  pumpCmd = 0x00;

  // ===== Init I2C Bus =====
  wi->setPins(29, 31);
  wi->begin();
  Serial.println("Setting up I2C... SCL31 SDA 29");

  // ===== Init BME280 =====
  bme.begin(0x76, wi));
   
  // ======= Init Batt ========
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
