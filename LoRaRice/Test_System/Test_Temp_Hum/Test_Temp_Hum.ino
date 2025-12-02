#include <Wire.h>

TwoWire* wi = &Wire;  // ใช้ pointer ไปยัง Wire

void setup() {
  wi->setPins(29,31);   // ตั้งขา SDA = 30, SCL = 28
  wi->begin();

  Serial.begin(115200);
  delay(1000);
  Serial.println("Scanning...");

  for (byte addr = 1; addr < 127; addr++) {
    wi->beginTransmission(addr);
    if (wi->endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(addr, HEX);
    }
  }
}

void loop() {}
