  #include <Arduino.h>
  #include <Wire.h>
  #include <SPI.h>
  #include <Adafruit_TinyUSB.h> 
  #include "MyVL53L1X.h"
  #include <BatteryMonitor.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  #include <TinyGPSPlus.h>
  #include "heltec_nrf_lorawan.h"

  #define PIN_BAT_ADC     4
  #define PIN_BAT_ADC_CTL 6
  #define MY_BAT_AMPLIFY     2.0

  /* OTAA para*/
  uint8_t devEui[] = { 0x22, 0x32, 0x33, 0x00, 0x00, 0x99, 0xaa, 0x04 };
  uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint8_t appKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };

  /* ABP para*/
  uint8_t nwkSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };
  uint8_t appSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88 };
  uint32_t devAddr = (uint32_t)0x007e6ae4;

  /*LoraWan channelsmask, default channels 0-7*/
  uint16_t userChannelsMask[6] = { 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

  /*LoraWan region, select in arduino IDE tools*/
  LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_CN470;

  /*LoraWan Class, Class A and Class C are supported*/
  DeviceClass_t loraWanClass = CLASS_A;

  /*the application data transmission duty cycle.  value in [ms].*/
  uint32_t appTxDutyCycle = 15000;

  /*OTAA or ABP*/
  bool overTheAirActivation = 1;

  /*ADR enable*/
  bool loraWanAdr = true;

  /* Indicates if the node is sending confirmed or unconfirmed messages */
  bool isTxConfirmed = true;

  /* Application port */
  uint8_t appPort = 2;

  uint8_t confirmedNbTrials = 1;

  const int READ_SENSOR = 0;
  const int LORA_SEND_SENSOR = 1;
  const int SLEEP_MODE = 2; 
  int state ;

  TinyGPSPlus gps;
  #define SerialGPS Serial1

  float gpsLat = 0.0;
  float gpsLng = 0.0;
  bool gpsHasFix = false;

  unsigned long lastSensorReadTime = 0;
  const unsigned long sensorReadInterval = 15000;
  uint32_t receivedSleepDuration = 15000;

  MyVL53L1X sensor(30, 28);
  BatteryMonitor battery(PIN_BAT_ADC, PIN_BAT_ADC_CTL, MY_BAT_AMPLIFY);
  Adafruit_BME280 bme;

  static void prepareTxFrame(uint8_t port) {
  int distance = sensor.readDistance();
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();

  int32_t lat_enc = gpsLat * 1e6;  // encode to int32
  int32_t lng_enc = gpsLng * 1e6;

  appDataSize = 14;
  appData[0] = (distance >> 8) & 0xFF;
  appData[1] = distance & 0xFF;

  appData[2] = (int)temperature;
  appData[3] = ((int)(temperature * 100)) % 100;
  appData[4] = (int)humidity;
  appData[5] = ((int)(humidity * 100)) % 100;

  appData[6] = (lat_enc >> 24) & 0xFF;
  appData[7] = (lat_enc >> 16) & 0xFF;
  appData[8] = (lat_enc >> 8) & 0xFF;
  appData[9] = lat_enc & 0xFF;

  appData[10] = (lng_enc >> 24) & 0xFF;
  appData[11] = (lng_enc >> 16) & 0xFF;
  appData[12] = (lng_enc >> 8) & 0xFF;
  appData[13] = lng_enc & 0xFF;
}

  void setupRtcWakeup(uint32_t ms)
  {
    NRF_RTC2->TASKS_STOP = 1;
    NRF_RTC2->TASKS_CLEAR = 1;

    NRF_RTC2->PRESCALER = 32;     // 1 tick = 1 ms (32.768kHz / (32+1))
    NRF_RTC2->CC[0] = ms;
    NRF_RTC2->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
    NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    NVIC_EnableIRQ(RTC2_IRQn);
    NRF_RTC2->TASKS_START = 1;
  }

  void enterDeepSleep() {
    setupRtcWakeup(receivedSleepDuration);
    Serial.printf("💤 Entering deep sleep for %lu ms...\n", receivedSleepDuration);
    delay(100);  
    sd_power_system_off(); 
  }

 void downLinkDataHandle(McpsIndication_t *mcpsIndication)
  {
    Serial.println("📥 Downlink received!");

    if (mcpsIndication->BufferSize >= 2) {
      receivedSleepDuration = ((uint16_t)mcpsIndication->Buffer[0] << 8) | mcpsIndication->Buffer[1];
      Serial.printf("⏱️ New sleep duration from TTN: %lu ms\n", receivedSleepDuration);
    }
  }

  void setup() {
    Serial.begin(115200);
    SerialGPS.begin(9600);
    Wire.begin();
    if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
    } else {
    Serial.println("BME280 sensor OK!");

    if (!sensor.begin()) {
      Serial.println("Sensor init failed!");
      while (1) delay(100);
    }
    Serial.println("Sensor ready!");
    battery.begin();
    state = READ_SENSOR;
    }
    LoRaWAN.init(loraWanClass, loraWanRegion);
    LoRaWAN.join();
  }

  void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastSensorReadTime >= sensorReadInterval) {
        lastSensorReadTime = currentTime;

      if(state == READ_SENSOR){
        while (SerialGPS.available()) {
        gps.encode(SerialGPS.read());
      }

      if (gps.location.isUpdated()) {
        gpsLat = gps.location.lat();
        gpsLng = gps.location.lng();
        gpsHasFix = true;

        Serial.print("📍 Lat: ");
        Serial.println(gpsLat, 6);
        Serial.print("📍 Lng: ");
        Serial.println(gpsLng, 6);
      } else {
        gpsHasFix = false;
        Serial.println("⚠️ No GPS fix yet.");
      }

        int distance = sensor.readDistance();
        if (distance > 0){
          Serial.print("Distance: ");
          Serial.print(distance);
          Serial.println(" mm");
        }

        uint16_t batteryVoltage = battery.readMillivolts();
        Serial.printf("Battery voltage: %d mV\n", batteryVoltage);

        float temperature = bme.readTemperature();     // °C
        float humidity = bme.readHumidity();           // %

        Serial.print("Temp: ");
        Serial.print(temperature);
        Serial.print(" °C, Hum: ");
        Serial.println(humidity);

        state = LORA_SEND_SENSOR;
      }
    }
    else if(state == LORA_SEND_SENSOR){
      prepareTxFrame(appPort);  
      LoRaWAN.send();           
      state = SLEEP_MODE;

    }
    else if(state== SLEEP_MODE){
      enterDeepSleep();
      state = READ_SENSOR;
    }

  }
