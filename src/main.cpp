/*
 * EMG Protective Shield – Main Firmware
 * ======================================
 * Hardware : ESP32-S3 DevKitC-1
 * Sensor   : EMG Bio-Amp (analog output)
 * GSM      : SIM800L  (UART1)
 * GPS      : NEO-6M   (UART2)
 * Actuator : Servo motor (PWM)
 * Power    : 3.7 V Li-ion + MT3608 boost converter → 5 V / 3.3 V rails
 *
 * State machine
 * ─────────────
 *  IDLE     → Normal monitoring. EMG, GPS updated every loop.
 *  ALERT    → Abnormal EMG detected. Servo deploys, SMS sent, LED blinks.
 *  COOLDOWN → Post-alert lockout (ALERT_COOLDOWN_MS) before returning to IDLE.
 */

#include <Arduino.h>
#include "config.h"
#include "emg_sensor.h"
#include "gsm_module.h"
#include "gps_module.h"
#include "servo_control.h"

// ── Module instances ──────────────────────────────────────────────────────────
static EmgSensor    emg;
static GsmModule    gsm;
static GpsModule    gps;
static ServoControl servo;

// ── State machine ─────────────────────────────────────────────────────────────
enum class SystemState { IDLE, ALERT, COOLDOWN };
static SystemState state = SystemState::IDLE;

// Timestamp used for cooldown expiry and GPS fix wait.
static unsigned long stateEnteredAt = 0;

// ── Helpers ───────────────────────────────────────────────────────────────────

// Blink the status LED a given number of times.
static void blinkLed(int times, unsigned long onMs = 200, unsigned long offMs = 200) {
    for (int i = 0; i < times; ++i) {
        digitalWrite(LED_PIN, HIGH);
        delay(onMs);
        digitalWrite(LED_PIN, LOW);
        delay(offMs);
    }
}

// Wait up to GPS_FIX_TIMEOUT_MS for a valid GPS fix while feeding the parser.
// Returns true if a fix was obtained within the timeout.
static bool waitForGpsFix() {
    unsigned long deadline = millis() + GPS_FIX_TIMEOUT_MS;
    while (millis() < deadline) {
        gps.update();
        if (gps.hasFix()) {
            return true;
        }
        delay(50);
    }
    return false;
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[SYS] EMG Protective Shield – booting...");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialise all subsystems.
    emg.begin();
    gps.begin();
    servo.begin();

    // GSM initialisation is blocking; indicate progress on the LED.
    Serial.println("[SYS] Initialising GSM module...");
    blinkLed(3, 100, 100);
    if (!gsm.begin()) {
        Serial.println("[SYS] WARNING – GSM initialisation failed. "
                       "Alerts will not be sent.");
    }

    state = SystemState::IDLE;
    stateEnteredAt = millis();
    Serial.println("[SYS] Boot complete. Entering IDLE state.");
    blinkLed(2, 300, 150);
}

void loop() {
    // Always keep the GPS parser fed.
    gps.update();

    switch (state) {

    // ── IDLE ──────────────────────────────────────────────────────────────────
    case SystemState::IDLE: {
        // Update the EMG sensor; it returns true once a new RMS window is ready.
        if (emg.update() && emg.isAlertActive()) {
            Serial.printf("[SYS] EMG alert detected (RMS=%u). Transitioning to ALERT.\n",
                          emg.getRms());

            // Immediate physical response: deploy the servo.
            servo.deploy();
            digitalWrite(LED_PIN, HIGH);

            // Attempt to obtain a GPS fix before composing the SMS.
            bool hasFix = waitForGpsFix();
            double lat  = hasFix ? gps.getLatitude()  : 0.0;
            double lng  = hasFix ? gps.getLongitude() : 0.0;

            // Build alert message. Buffer sized to accommodate the full body
            // assembled in sendAlert (message portion + Google Maps URL).
            char msg[256];
            snprintf(msg, sizeof(msg),
                     "EMERGENCY ALERT: Abnormal muscle activity detected. "
                     "EMG RMS=%u. Immediate assistance required!", emg.getRms());

            // Send SMS to emergency contacts.
            if (gsm.isReady()) {
                gsm.sendAlert(msg, lat, lng);
            } else {
                Serial.println("[SYS] GSM not ready – SMS skipped.");
            }

            // Clear the EMG alert latch.
            emg.clearAlert();

            // Transition to ALERT state.
            state = SystemState::ALERT;
            stateEnteredAt = millis();
        }
        break;
    }

    // ── ALERT ─────────────────────────────────────────────────────────────────
    case SystemState::ALERT: {
        // Blink LED rapidly to provide a visible alarm indication.
        blinkLed(1, 100, 100);

        // After the hold period, retract the servo and enter cooldown.
        if (millis() - stateEnteredAt >= ALERT_HOLD_PERIOD_MS) {
            servo.retract();
            digitalWrite(LED_PIN, LOW);
            state = SystemState::COOLDOWN;
            stateEnteredAt = millis();
            Serial.println("[SYS] Alert hold complete. Entering COOLDOWN.");
        }
        break;
    }

    // ── COOLDOWN ──────────────────────────────────────────────────────────────
    case SystemState::COOLDOWN: {
        // Slow LED heartbeat to indicate the device is alive but in cooldown.
        blinkLed(1, 50, 950);

        if (millis() - stateEnteredAt >= ALERT_COOLDOWN_MS) {
            state = SystemState::IDLE;
            stateEnteredAt = millis();
            Serial.println("[SYS] Cooldown complete. Returning to IDLE.");
        }
        break;
    }

    } // end switch
}
