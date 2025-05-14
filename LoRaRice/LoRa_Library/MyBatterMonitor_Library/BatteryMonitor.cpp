#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor(uint8_t adcPin, uint8_t controlPin, float amplify, float refVoltage)
    : _adcPin(adcPin), _controlPin(controlPin), _amplify(amplify) {
        _mvPerLsb = refVoltage / 4096.0F;  
    }

void BatteryMonitor::begin() {
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);
    pinMode(_controlPin, OUTPUT);
}

uint16_t BatteryMonitor::readMillivolts() {
    digitalWrite(_controlPin, HIGH);
    delay(10);
    int adcValue = analogRead(_adcPin);
    digitalWrite(_controlPin, LOW);
    return (uint16_t)((float)adcValue * _mvPerLsb * _amplify);
}
