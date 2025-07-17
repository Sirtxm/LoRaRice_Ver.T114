#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// ===== Pin Definitions =====
#define GPS_RX 9   // ขา RX ของ Arduino (ต่อกับ TX ของ GPS)
#define GPS_TX 10  // ขา TX ของ Arduino (ต่อกับ RX ของ GPS)

// ===== Global Variables =====
double latitude = 0;
double longitude = 0;

// ===== GPS Instances =====
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  Serial.println("GPS Module Ready");
}

void loop() {
  // อ่านข้อมูลจาก GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // ตรวจสอบและแสดงข้อมูลพิกัด
  if (gps.location.isUpdated() && gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
  }
}
