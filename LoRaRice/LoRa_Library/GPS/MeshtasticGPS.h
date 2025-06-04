#ifndef MESHTASTIC_GPS_H
#define MESHTASTIC_GPS_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

enum GPSPowerState : uint8_t {
    GPS_ACTIVE,
    GPS_IDLE,
    GPS_SOFTSLEEP,
    GPS_HARDSLEEP,
    GPS_OFF
};

class MeshtasticGPS {
public:
    MeshtasticGPS();

    // Initialize GPS on specific hardware serial pins
    void begin(HardwareSerial &serialPort, uint32_t baudRate = 9600, int8_t enPin = -1, int8_t standbyPin = -1);

    // Read serial data and decode NMEA
    void update();

    // Return true if GPS has a valid fix
    bool hasFix();

    // Return latitude & longitude (in degrees)
    double latitude();
    double longitude();

    // Return altitude in meters
    double altitude();

    // Number of satellites
    uint8_t satellites();

    // Set GPS power state
    void setPowerState(GPSPowerState state);

    // Enable or disable GPS module
    void powerOn();
    void powerOff();

private:
    TinyGPSPlus gps;
    HardwareSerial *serial;
    int8_t gpsEnPin = -1;
    int8_t standbyPin = -1;
    GPSPowerState powerState = GPS_OFF;

    void writePinEN(bool on);
    void writePinStandby(bool standby);
};

#endif  // MESHTASTIC_GPS_H
