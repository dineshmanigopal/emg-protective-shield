#include "emg_sensor.h"
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// EmgSensor implementation
// ─────────────────────────────────────────────────────────────────────────────

void EmgSensor::begin() {
    // Configure ADC resolution to 12 bits (0–4095).
    analogReadResolution(12);
    // Use the internal 3.3 V reference; attenuation set for 0–3.3 V range.
    analogSetPinAttenuation(EMG_PIN, ADC_11db);

    _samplePeriodUs = 1000000UL / EMG_SAMPLE_RATE_HZ;
    _lastSampleUs   = micros();
    _sampleIdx      = 0;
    _sampleCount    = 0;
    _sumSq          = 0;
    _rms            = 0;
    _consecCount    = 0;
    _alertActive    = false;

    Serial.printf("[EMG] Initialised on GPIO%d, %d Hz, window %d samples\n",
                  EMG_PIN, EMG_SAMPLE_RATE_HZ, EMG_WINDOW_SIZE);
}

bool EmgSensor::update() {
    unsigned long now = micros();

    // Enforce the sampling rate using a busy-wait free approach.
    if ((now - _lastSampleUs) < _samplePeriodUs) {
        return false;
    }
    _lastSampleUs = now;

    // Acquire raw ADC value and apply full-wave rectification around the bias.
    int raw = analogRead(EMG_PIN);
    int biased = raw - static_cast<int>(ADC_BIAS);
    uint32_t rectified = static_cast<uint32_t>(abs(biased));

    // Subtract the outgoing sample's squared value from the running sum only
    // after the circular buffer has been fully populated (avoids underflow
    // during the initial fill where slots beyond _sampleCount are still 0).
    if (_sampleCount >= EMG_WINDOW_SIZE) {
        uint64_t outgoing = static_cast<uint64_t>(_samples[_sampleIdx]) * _samples[_sampleIdx];
        _sumSq -= outgoing;
    }
    _sumSq += static_cast<uint64_t>(rectified) * rectified;

    // Insert new sample into the circular buffer.
    _samples[_sampleIdx] = rectified;
    _sampleIdx = (_sampleIdx + 1) % EMG_WINDOW_SIZE;

    if (_sampleCount < EMG_WINDOW_SIZE) {
        ++_sampleCount;
        // Not enough samples yet to produce a valid RMS.
        return false;
    }

    // Compute sliding-window RMS from the maintained running sum.
    double mean = static_cast<double>(_sumSq) / EMG_WINDOW_SIZE;
    _rms = static_cast<uint16_t>(sqrt(mean));

    if (_rms >= EMG_ALERT_THRESHOLD) {
        ++_consecCount;
        if (_consecCount >= EMG_CONSEC_OVER && !_alertActive) {
            _alertActive = true;
            Serial.printf("[EMG] ALERT triggered – RMS=%u (threshold=%d)\n",
                          _rms, EMG_ALERT_THRESHOLD);
        }
    } else {
        // Reset consecutive counter if signal drops below threshold.
        _consecCount = 0;
    }
    return true;
}
