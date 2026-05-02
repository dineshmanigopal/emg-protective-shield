#pragma once

#include <Arduino.h>
#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// EMG Sensor – signal acquisition and abnormal-activity detection
//
// Algorithm overview
// ──────────────────
//  1. Raw ADC samples are collected at EMG_SAMPLE_RATE_HZ.
//  2. Each sample is full-wave rectified (centred around Vcc/2 bias).
//  3. An RMS value is computed over a sliding window of EMG_WINDOW_SIZE
//     samples (≈ 100 ms at 500 Hz).
//  4. If the RMS exceeds EMG_ALERT_THRESHOLD for EMG_CONSEC_OVER consecutive
//     windows, an alert condition is raised.
// ─────────────────────────────────────────────────────────────────────────────

class EmgSensor {
public:
    // Initialise ADC and internal state.
    void begin();

    // Call this in the main loop as fast as possible.
    // Returns true when a new RMS value has been computed (every sample once
    // the buffer is fully populated – true sliding-window RMS).
    bool update();

    // Returns the most recently computed RMS value (12-bit ADC counts).
    uint16_t getRms() const { return _rms; }

    // Returns true if the alert condition is currently active.
    bool isAlertActive() const { return _alertActive; }

    // Clears the alert latch so detection can restart.
    void clearAlert() { _alertActive = false; _consecCount = 0; }

private:
    // Circular sample buffer used for the sliding RMS window.
    uint32_t _samples[EMG_WINDOW_SIZE] = {};
    int      _sampleIdx    = 0;
    int      _sampleCount  = 0;

    unsigned long _lastSampleUs = 0;          // µs timestamp of last sample
    unsigned long _samplePeriodUs = 0;        // target period in µs

    uint16_t _rms         = 0;
    int      _consecCount = 0;
    bool     _alertActive = false;

    // Running sum of squared rectified samples for O(1) sliding-window RMS.
    uint64_t _sumSq = 0;

    // ADC mid-point bias (half of 3.3 V reference → ~1.65 V → ~2048 counts).
    // Adjust if the bio-amp uses a different common-mode voltage.
    static constexpr uint16_t ADC_BIAS = 2048;
};
