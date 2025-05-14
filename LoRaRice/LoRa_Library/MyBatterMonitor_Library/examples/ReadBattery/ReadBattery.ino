#include <BatteryMonitor.h>
#include <Adafruit_TinyUSB.h> 

#define PIN_BAT_ADC      4
#define PIN_BAT_ADC_CTL  6
#define MY_BATTERY_AMPLIFY 2.0

BatteryMonitor battery(PIN_BAT_ADC, PIN_BAT_ADC_CTL, MY_BATTERY_AMPLIFY);

void setup() {
  // อย่าใส่ Adafruit_TinyUSB
  Serial.begin(115200);
  delay(2000); // ให้ Serial เสถียร (สำคัญกับบางบอร์ด)
  Serial.println("Setup OK");
  battery.begin();
}

void loop() {
  uint16_t mv = battery.readMillivolts();
  Serial.printf("Battery voltage: %d mV\n", mv);
  delay(1000);
}