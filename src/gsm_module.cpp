#include "gsm_module.h"

// ─────────────────────────────────────────────────────────────────────────────
// GsmModule implementation
// ─────────────────────────────────────────────────────────────────────────────

bool GsmModule::begin() {
    _serial.begin(GSM_BAUD_RATE, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    delay(3000); // Allow the SIM800L to complete its power-on sequence.

    // Basic communication test.
    if (!_sendCmd("AT", "OK", 3000)) {
        Serial.println("[GSM] No response from SIM800L – check wiring.");
        return false;
    }

    // Turn off echo so responses are easier to parse.
    _sendCmd("ATE0", "OK");

    // Verify that a SIM card is inserted.
    if (!_sendCmd("AT+CIMI", "OK", 5000)) {
        Serial.println("[GSM] SIM card not detected.");
        return false;
    }

    // Wait for network registration (up to 30 s).
    unsigned long deadline = millis() + 30000UL;
    bool registered = false;
    while (millis() < deadline) {
        _serial.println("AT+CREG?");
        String resp = _readResponse(3000);
        // +CREG: 0,1 → registered home  / +CREG: 0,5 → registered roaming
        if (resp.indexOf(",1") != -1 || resp.indexOf(",5") != -1) {
            registered = true;
            break;
        }
        delay(2000);
    }

    if (!registered) {
        Serial.println("[GSM] Network registration failed.");
        return false;
    }

    // Set SMS text mode.
    _sendCmd("AT+CMGF=1", "OK");

    _ready = true;
    Serial.println("[GSM] SIM800L ready.");
    return true;
}

bool GsmModule::sendAlert(const char* message, double lat, double lng) {
    if (!_ready) {
        Serial.println("[GSM] Module not ready – alert not sent.");
        return false;
    }

    // Build the message body, optionally appending a maps link.
    char body[SMS_BODY_MAX_LENGTH];
    if (lat != 0.0 && lng != 0.0) {
        snprintf(body, sizeof(body),
                 "%s\nLocation: https://maps.google.com/?q=%.6f,%.6f",
                 message, lat, lng);
    } else {
        snprintf(body, sizeof(body), "%s\nLocation: GPS fix unavailable", message);
    }

    bool ok1 = _sendSms(EMERGENCY_CONTACT_1, body);
    bool ok2 = _sendSms(EMERGENCY_CONTACT_2, body);

    return ok1 && ok2;
}

bool GsmModule::_sendSms(const char* number, const char* body) {
    // Issue AT+CMGS command.
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", number);
    _serial.println(cmd);

    // The module responds with a '>' prompt when ready to receive the body.
    String prompt = _readResponse(5000);
    if (prompt.indexOf('>') == -1) {
        Serial.printf("[GSM] No prompt for number %s\n", number);
        return false;
    }

    // Send the message body followed by Ctrl-Z (0x1A) to transmit.
    _serial.print(body);
    _serial.write(0x1A);

    String resp = _readResponse(10000);
    if (resp.indexOf("+CMGS:") != -1 || resp.indexOf("OK") != -1) {
        Serial.printf("[GSM] SMS sent to %s\n", number);
        return true;
    }

    Serial.printf("[GSM] SMS failed for %s: %s\n", number, resp.c_str());
    return false;
}

bool GsmModule::_sendCmd(const char* cmd, const char* expected,
                         unsigned long timeoutMs) {
    _serial.println(cmd);
    String resp = _readResponse(timeoutMs);
    return resp.indexOf(expected) != -1;
}

String GsmModule::_readResponse(unsigned long timeoutMs) {
    String resp;
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        while (_serial.available()) {
            char c = _serial.read();
            resp += c;
        }
        // Exit early if a terminal token is present.
        if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1 ||
            resp.indexOf(">") != -1  || resp.indexOf("+CMGS:") != -1) {
            break;
        }
        delay(10);
    }
    return resp;
}
