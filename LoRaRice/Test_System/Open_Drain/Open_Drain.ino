#include <Arduino.h>
// เพิ่มบรรทัดนี้เพื่อช่วยให้ Linker หา Serial เจอในบางกรณี
#include <Adafruit_TinyUSB.h> 

#define TEST_PIN 13

void setup() {
  // เริ่มต้น Serial ปกติ
  Serial.begin(115200);
  
  // รอจนกว่า Serial Port จะพร้อม (สำหรับ USB Serial)
  while (!Serial) delay(10); 
  
  Serial.println("nRF52840 Open Drain Test Started...");

  pinMode(TEST_PIN, OUTPUT);

  // ใช้ gpiote configuration โดยตรงจาก nRF SDK
  // digitalPinToPinName จะแปลงเลขขา Arduino เป็น Physical Pin ของ nRF
  nrf_gpio_cfg(
    digitalPinToPinName(TEST_PIN),
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_S0D1, // S0D1 = Standard 0, Disconnect 1 (Open Drain)
    NRF_GPIO_PIN_NOSENSE
  );

  Serial.println("Pin configured as Open Drain (S0D1)");
}

void loop() {

  Serial.println("Status: HIGH (Floating)");
  digitalWrite(TEST_PIN, HIGH);
  delay(2000);
}