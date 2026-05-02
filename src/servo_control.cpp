#include "servo_control.h"

// ─────────────────────────────────────────────────────────────────────────────
// ServoControl implementation
// ─────────────────────────────────────────────────────────────────────────────

void ServoControl::begin() {
    // Allocate a free hardware timer channel for the servo PWM signal.
    // ESP32Servo manages channel allocation internally.
    _servo.setPeriodHertz(50); // standard 50 Hz servo signal
    _servo.attach(SERVO_PIN, 500, 2400); // min/max pulse widths in µs

    // Start in the rest (retracted) position.
    _servo.write(SERVO_REST_ANGLE);
    _deployed = false;

    Serial.printf("[SERVO] Attached to GPIO%d, rest=%d°, deploy=%d°\n",
                  SERVO_PIN, SERVO_REST_ANGLE, SERVO_DEPLOY_ANGLE);
}

void ServoControl::deploy() {
    if (!_deployed) {
        _servo.write(SERVO_DEPLOY_ANGLE);
        _deployed = true;
        Serial.printf("[SERVO] Deployed to %d°\n", SERVO_DEPLOY_ANGLE);
    }
}

void ServoControl::retract() {
    if (_deployed) {
        _servo.write(SERVO_REST_ANGLE);
        _deployed = false;
        Serial.printf("[SERVO] Retracted to %d°\n", SERVO_REST_ANGLE);
    }
}
