#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// Servo Control – protective / alerting action mechanism
//
// The servo is used to physically deploy a protective arm or trigger a visible
// alarm indicator when an emergency condition is detected.
//
//  • Rest position  (SERVO_REST_ANGLE)   : normal / standby state.
//  • Deploy position (SERVO_DEPLOY_ANGLE): emergency / alert state.
// ─────────────────────────────────────────────────────────────────────────────

class ServoControl {
public:
    // Attach the servo to the configured PWM pin and move to the rest position.
    void begin();

    // Move to the deploy (emergency) position.
    void deploy();

    // Move back to the rest (standby) position.
    void retract();

    // Returns true while the servo is in the deployed position.
    bool isDeployed() const { return _deployed; }

private:
    Servo _servo;
    bool  _deployed = false;
};
