#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

class BatteryMonitor {
public:
    BatteryMonitor(uint8_t adcPin, uint8_t controlPin, float amplify = 2.0, float refVoltage = 3000.0);
    void begin();
    uint16_t readMillivolts();

private:
    uint8_t _adcPin;
    uint8_t _controlPin;
    float _amplify;
    float _mvPerLsb;
};

#endif