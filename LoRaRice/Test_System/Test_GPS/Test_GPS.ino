#include <TinyGPS++.h>
#include "Adafruit_TinyUSB.h"


TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);      // Serial Monitor
  Serial1.begin(9600);       // UART1 สำหรับ GPS (ใช้ขาปกติ GPIO9/10)

  Serial.println("เริ่มอ่านพิกัดจาก GPS ด้วย TinyGPS++");
}

void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("ละติจูด: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("ลองจิจูด: ");
    Serial.println(gps.location.lng(), 6);
    Serial.println("------------------------");
  }
}
