#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// GSM Module – SIM800L AT-command driver
//
// Responsibilities:
//  • Initialise the SIM800L over UART1.
//  • Send an SMS alert (including optional GPS coordinates) to the configured
//    emergency contact numbers.
//  • Expose a simple, non-blocking status so the main loop stays responsive.
// ─────────────────────────────────────────────────────────────────────────────

class GsmModule {
public:
    // Initialise UART and verify communication with the SIM800L.
    // Returns true if the module responds to AT.
    bool begin();

    // Returns true when the module is ready to send.
    bool isReady() const { return _ready; }

    // Send an SMS with the given message body to both emergency contacts.
    // If lat/lng are non-zero, a Google Maps link is appended automatically.
    // Returns true if both messages were accepted by the module.
    bool sendAlert(const char* message, double lat = 0.0, double lng = 0.0);

private:
    HardwareSerial _serial{1}; // UART1
    bool _ready = false;

    // Send an AT command and wait for the expected response within timeoutMs.
    // Returns true if the response was received.
    bool _sendCmd(const char* cmd, const char* expected,
                  unsigned long timeoutMs = 5000UL);

    // Read the raw response of the last AT command.
    String _readResponse(unsigned long timeoutMs = 3000UL);

    // Send an SMS to a single number.
    bool _sendSms(const char* number, const char* body);
};
