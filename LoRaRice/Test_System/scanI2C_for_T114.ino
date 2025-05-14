#include <Wire.h>

TwoWire *wi = &Wire;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  wi->setPins(30, 28);  // Set GPIO for SDA/SCL
  wi->begin();          // Begin I2C

  Serial.println("Scanning address from 0 to 127");
}

void loop() {
  for (int addr = 1; addr < 128; addr++) {
    wi->beginTransmission(addr);
    if (wi->endTransmission() == 0) {
      Serial.print("Found: 0x");
      Serial.println(addr, HEX);
    }
  }

  delay(5000);
}
