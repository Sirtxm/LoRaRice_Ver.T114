void setup() {
  Serial.begin(115200);  // เริ่มต้น Serial ที่ baudrate 115200
}

void loop() {
  // พิมพ์ซ้ำทุกวินาที
  Serial.println("Hello, World!");
  delay(1000);
}

