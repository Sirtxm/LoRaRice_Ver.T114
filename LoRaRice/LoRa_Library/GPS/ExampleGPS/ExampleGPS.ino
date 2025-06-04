#define BAT_ADC_PIN A2          // P0.04 หรือ AIN2
#define ADC_REF_VOLTAGE 3.7     // กำหนดอ้างอิงแรงดัน ADC ที่ใช้
#define ADC_RESOLUTION 1023.0   // สำหรับ 10-bit ADC
#define VOLTAGE_DIVIDER_RATIO 4.9  // จาก R32 และ R33

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);
}

void loop() {
  int raw_adc = analogRead(BAT_ADC_PIN);
  float adc_voltage = (raw_adc / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  float battery_voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO;

  Serial.print("Battery Voltage: ");
  Serial.print(battery_voltage, 2);
  Serial.println(" V");

  delay(2000);
}
