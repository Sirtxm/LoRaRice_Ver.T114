#ifndef MY_VL53L1X_H
#define MY_VL53L1X_H

#include <Wire.h>
#include <Adafruit_VL53L1X.h>

class MyVL53L1X {
  public:
    MyVL53L1X(uint8_t sda, uint8_t scl);
    bool begin();
    int readDistance();

  private:
    uint8_t _sda, _scl;
    TwoWire *_i2c;
    Adafruit_VL53L1X _sensor;
};

#endif
