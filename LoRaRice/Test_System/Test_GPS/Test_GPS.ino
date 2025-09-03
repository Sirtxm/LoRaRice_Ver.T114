#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#define GPS_RX 9   // GPS TX ต่อเข้าขานี้
#define GPS_TX 10  // ไม่ได้ใช้ก็ได้

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);      // สำหรับดูข้อมูลบน Serial Monitor
  gpsSerial.begin(9600);     // GPS Module ใช้ความเร็ว 9600 baud
  delay(1000);
  Serial.println("GPS Module Ready");
}

void loop() {
  while (gpsSerial.available()) {
    // char c = gpsSerial.read();
    // Serial.write(c);
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  }
}