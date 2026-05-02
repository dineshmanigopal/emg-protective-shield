#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// EMG Protective Shield – Hardware Configuration
// Target MCU : ESP32-S3 (Xtensa LX7 dual-core, 240 MHz)
// ─────────────────────────────────────────────────────────────────────────────

// ── EMG Sensor (Bio-Amp) ─────────────────────────────────────────────────────
// Analog output of the EMG bio-amp connected to an ADC-capable GPIO.
static constexpr int EMG_PIN              = 4;   // GPIO4 / ADC1_CH3
static constexpr int EMG_SAMPLE_RATE_HZ   = 500; // samples per second
static constexpr int EMG_WINDOW_SIZE      = 50;  // RMS window (100 ms @ 500 Hz)

// Threshold above which sustained activity is flagged as an emergency.
// Value is expressed as a 12-bit ADC count (0–4095).
// Tune this value to match the bio-amp gain and electrode placement.
static constexpr int EMG_ALERT_THRESHOLD  = 2800;

// Number of consecutive RMS samples that must exceed the threshold before an
// alert is triggered (debounce / false-positive filter).
static constexpr int EMG_CONSEC_OVER      = 5;

// ── GSM Module – SIM800L ─────────────────────────────────────────────────────
// The SIM800L is driven over a dedicated hardware UART (UART1).
static constexpr int GSM_TX_PIN           = 17; // ESP32-S3 TX → SIM800L RX
static constexpr int GSM_RX_PIN           = 18; // SIM800L TX → ESP32-S3 RX
static constexpr long GSM_BAUD_RATE       = 9600;

// Emergency contact numbers (E.164 format recommended).
// ⚠️  IMPORTANT: Replace these placeholders with real phone numbers before
//     flashing to hardware. The numbers MUST NOT start with "+91XXXXXXX".
static constexpr const char* EMERGENCY_CONTACT_1 = "+91XXXXXXXXXX"; // primary
static constexpr const char* EMERGENCY_CONTACT_2 = "+91XXXXXXXXXX"; // secondary

// Compile-time reminder: the build will fail if this flag is set, reminding
// the developer to update the contacts before a production build.
// Uncomment the line below once real numbers have been entered:
// #define EMG_CONTACTS_CONFIGURED
#ifndef EMG_CONTACTS_CONFIGURED
#pragma message("WARNING: Emergency contact numbers are still set to placeholders. " \
                "Update EMERGENCY_CONTACT_1 / EMERGENCY_CONTACT_2 in config.h " \
                "and define EMG_CONTACTS_CONFIGURED before deploying to hardware.")
#endif

// ── GPS Module – NEO-6M ──────────────────────────────────────────────────────
// NEO-6M is driven over a second hardware UART (UART2).
static constexpr int GPS_TX_PIN           = 15; // ESP32-S3 TX → NEO-6M RX
static constexpr int GPS_RX_PIN           = 16; // NEO-6M TX → ESP32-S3 RX
static constexpr long GPS_BAUD_RATE       = 9600;

// Timeout (ms) for waiting for a valid GPS fix before sending SMS without coords.
static constexpr unsigned long GPS_FIX_TIMEOUT_MS = 10000UL;

// ── Servo Motor ──────────────────────────────────────────────────────────────
static constexpr int SERVO_PIN            = 13; // PWM-capable GPIO
// Positions (degrees): 0° = rest / retracted, 90° = deployed / protective
static constexpr int SERVO_REST_ANGLE     = 0;
static constexpr int SERVO_DEPLOY_ANGLE   = 90;

// ── Status LED ───────────────────────────────────────────────────────────────
static constexpr int LED_PIN              = 2;  // on-board LED (active HIGH)

// ── Timing ───────────────────────────────────────────────────────────────────
// Minimum interval between successive alerts (ms) to avoid SMS flooding.
static constexpr unsigned long ALERT_COOLDOWN_MS   = 60000UL; // 60 s
// Duration (ms) the servo stays deployed and the LED blinks after an alert.
static constexpr unsigned long ALERT_HOLD_PERIOD_MS = 5000UL;  //  5 s

// ── SMS ──────────────────────────────────────────────────────────────────────
// Maximum length (bytes) of the full SMS body built in GsmModule::sendAlert.
// Accounts for the message text + Google Maps URL (~80 chars).
static constexpr int SMS_BODY_MAX_LENGTH = 320;
