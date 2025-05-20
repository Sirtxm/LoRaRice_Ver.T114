#include "MyVL53L1X.h"

MyVL53L1X::MyVL53L1X(uint8_t sda, uint8_t scl) {
  _sda = sda;
  _scl = scl;
  _i2c = &Wire;
}

bool MyVL53L1X::begin() {
  _i2c->setPins(_sda, _scl);
  _i2c->begin();

  if (!_sensor.begin(0x29, _i2c)) {
    return false;
  }

  if (!_sensor.startRanging()) {
    return false;
  }

  _sensor.setTimingBudget(50);
  return true;
}

int MyVL53L1X::readDistance() {
  if (_sensor.dataReady()) {
    int16_t distance = _sensor.distance();
    if (distance > 0) {
      _sensor.clearInterrupt();
      return distance;
    }
  }
  return -1;
}
