#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// GPS Module – NEO-6M NMEA parser
//
// Responsibilities:
//  • Initialise the NEO-6M over UART2.
//  • Continuously parse incoming NMEA sentences via TinyGPSPlus.
//  • Provide the latest valid latitude, longitude, and fix validity flag.
// ─────────────────────────────────────────────────────────────────────────────

class GpsModule {
public:
    // Initialise UART2 and TinyGPS++ parser.
    void begin();

    // Feed incoming serial bytes into the NMEA parser.
    // Call this every loop iteration.
    void update();

    // Returns true when a valid position fix is available.
    bool hasFix() const;

    // Returns the most recent valid latitude in decimal degrees.
    double getLatitude() const;

    // Returns the most recent valid longitude in decimal degrees.
    double getLongitude() const;

    // Returns the age of the last valid fix in milliseconds (0 = very fresh).
    uint32_t getFixAgeMs() const;

private:
    HardwareSerial _serial{2}; // UART2
    TinyGPSPlus    _gps;
};
