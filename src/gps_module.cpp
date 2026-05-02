#include "gps_module.h"

// ─────────────────────────────────────────────────────────────────────────────
// GpsModule implementation
// ─────────────────────────────────────────────────────────────────────────────

void GpsModule::begin() {
    _serial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.printf("[GPS] NEO-6M initialised on UART2 (RX=%d, TX=%d) @ %ld baud\n",
                  GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);
}

void GpsModule::update() {
    while (_serial.available() > 0) {
        char c = _serial.read();
        _gps.encode(c);
    }
}

bool GpsModule::hasFix() const {
    return _gps.location.isValid() && _gps.location.age() < 5000UL;
}

double GpsModule::getLatitude() const {
    return _gps.location.isValid() ? _gps.location.lat() : 0.0;
}

double GpsModule::getLongitude() const {
    return _gps.location.isValid() ? _gps.location.lng() : 0.0;
}

uint32_t GpsModule::getFixAgeMs() const {
    return _gps.location.isValid() ? _gps.location.age() : UINT32_MAX;
}
