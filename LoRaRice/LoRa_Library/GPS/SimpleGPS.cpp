#include "MeshtasticGPS.h"

MeshtasticGPS::MeshtasticGPS() {}

void MeshtasticGPS::begin(HardwareSerial &serialPort, uint32_t baudRate, int8_t enPin, int8_t standbyPin) {
    serial = &serialPort;
    gpsEnPin = enPin;
    this->standbyPin = standbyPin;

    if (gpsEnPin != -1) {
        pinMode(gpsEnPin, OUTPUT);
        digitalWrite(gpsEnPin, HIGH);  // Enable power by default
    }

    if (standbyPin != -1) {
        pinMode(standbyPin, OUTPUT);
        digitalWrite(standbyPin, LOW); // Awake by default
    }

    serial->begin(baudRate);
    powerState = GPS_ACTIVE;
}

void MeshtasticGPS::update() {
    while (serial->available() > 0) {
        gps.encode(serial->read());
    }
}

bool MeshtasticGPS::hasFix() {
    return gps.location.isValid() && gps.location.age() < 5000;
}

double MeshtasticGPS::latitude() {
    return gps.location.lat();
}

double MeshtasticGPS::longitude() {
    return gps.location.lng();
}

double MeshtasticGPS::altitude() {
    return gps.altitude.meters();
}

uint8_t MeshtasticGPS::satellites() {
    return gps.satellites.value();
}

void MeshtasticGPS::setPowerState(GPSPowerState state) {
    powerState = state;

    switch (state) {
        case GPS_ACTIVE:
            writePinEN(true);
            writePinStandby(false);  // Awake
            break;
        case GPS_SOFTSLEEP:
            writePinStandby(true);  // Sleep mode (standby pin)
            break;
        case GPS_HARDSLEEP:
        case GPS_OFF:
            writePinEN(false);
            break;
        case GPS_IDLE:
        default:
            // No specific action
            break;
    }
}

void MeshtasticGPS::powerOn() {
    setPowerState(GPS_ACTIVE);
}

void MeshtasticGPS::powerOff() {
    setPowerState(GPS_OFF);
}

void MeshtasticGPS::writePinEN(bool on) {
    if (gpsEnPin != -1) {
        digitalWrite(gpsEnPin, on ? HIGH : LOW);
    }
}

void MeshtasticGPS::writePinStandby(bool standby) {
    if (standbyPin != -1) {
        digitalWrite(standbyPin, standby ? HIGH : LOW);
    }
}
