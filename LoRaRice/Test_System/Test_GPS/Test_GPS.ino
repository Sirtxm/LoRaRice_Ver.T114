

TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;

#define VEXT_CTRL 21

void setup() {
  // เปิดไฟเลี้ยงให้โมดูล GPS
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, LOW);  // LOW = เปิด Vext (ดูจาก schematic ของ Heltec)

  Serial.begin(115200);
  while (!Serial);  // รอให้ USB Serial พร้อมใช้งาน
  Serial.println("เริ่มอ่าน GPS");

  Serial1.begin(GPSBaud);  // ใช้ UART1 เชื่อมกับ GPS module
}

void loop() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", Lng: ");
    Serial.println(gps.location.lng(), 6);
  }
}
