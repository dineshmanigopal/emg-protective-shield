# EMG-Based Smart Protective Shield with GSM Alerts

> An intelligent wearable safety system that detects emergency muscle activity and triggers real-time alerts with location tracking.

## Overview

This project uses EMG signals to detect abnormal muscle activity and automatically deploy a protective shield while sending a GPS-enabled SMS alert. The firmware targets an **ESP32-WROOM-32** and integrates GSM, GPS, and a servo actuator.

## Key Features

- Automatic EMG-based trigger (no manual input required)
- GSM SMS alerts to a predefined contact
- Live or cached GPS location in the alert
- Servo-driven shield deployment with auto-retract
- Manual trigger button for testing/backup
- Calibration step on boot for reliable thresholds

## System Architecture

```
EMG Sensor → ESP32-WROOM-32 → Decision Logic →
   ├── GSM Module (Alert SMS)
   ├── GPS Module (Location)
   └── Servo Motor (Shield Actuation)
```

## Hardware Components

| Component                 | Description                                   |
| ------------------------- | --------------------------------------------- |
| ESP32-WROOM-32            | Main microcontroller for processing           |
| Muscle Sensor v3 (EMG)    | Detects muscle activity signals               |
| SIM800L GSM               | Sends emergency SMS alerts                    |
| NEO-6M GPS                | Provides real-time location                   |
| SG90 Servo Motor          | Activates protective shield                   |
| Dual 9V supply (±9V)      | Required for the EMG sensor                   |
| Separate 5V servo supply  | Required for SG90 (shared ground only)        |
| Push Button               | Manual trigger (wired to GND with pull-up)    |

## Critical Wiring & Power Notes

- **EMG sensor requires ±9V dual supply** (two 9V batteries).
- **SIG output swings 0–9V** → must be scaled to 3.3V for ESP32 ADC.
- Use a voltage divider: **47kΩ / 18kΩ** to GPIO **34** (9V → ~2.49V).
- **Do not power the SG90 from the ESP32 3.3V/5V pin**. Use a separate 5V source.
- **Share ground** between ESP32 and the servo power supply.

## Pin Map (Firmware Defaults)

| Signal     | ESP32 Pin | Notes                              |
| ---------- | --------- | ---------------------------------- |
| EMG ADC    | GPIO 34   | Input-only ADC                     |
| Servo PWM  | GPIO 13   | SG90 signal wire                   |
| Button     | GPIO 25   | INPUT_PULLUP → button to GND       |
| GPS RX     | GPIO 16   | UART1 RX                           |
| GPS TX     | GPIO 17   | UART1 TX                           |
| GSM RX     | GPIO 26   | UART2 RX                           |
| GSM TX     | GPIO 27   | UART2 TX                           |

## Working Principle

1. On boot, the firmware calibrates the EMG baseline.
2. EMG samples are averaged and compared against a threshold.
3. A sustained contraction triggers:
   - Servo deployment to **90°**
   - GSM SMS with GPS coordinates (live or cached)
4. After **AUTO_RETRACT_MS**, the shield retracts.
5. A short cooldown re-arms the system for the next event.

## Configuration (in `emg_shield_ESP32.ino`)

Update these constants to match your hardware and preferences:

- `EMERGENCY_NUMBER` (required)
- `THRESHOLD_OFFSET` and `PEAK_COUNT_REQUIRED`
- `AUTO_RETRACT_MS` and `REARM_DELAY_MS`
- Servo settings: `SERVO_MIN_US`, `SERVO_MAX_US`, `SERVO_DEPLOYED_ANGLE`
- `DEBUG_MODE` for continuous ADC output
- `TEST_MODE` to run the built-in firmware test suite

## Software & Libraries

- Arduino IDE + ESP32 core
- TinyGPS++ (Mikal Hart)
- ESP32Servo (Kevin Harrington)

## Test Mode

Set `TEST_MODE` to `true` in the `.ino`, flash the board, and open the Serial Monitor at 115200 to run the built-in tests. The suite checks logic like servo pulse mapping, sweep step behavior, EMG peak gating, state-machine transitions, GPS fallback, and deploy timing (it does not validate external GSM/GPS hardware). Set it back to `false` for normal operation.

## Limitations

- EMG signal noise varies by placement and user
- Threshold tuning is user-specific
- GSM coverage affects alert delivery
- Intense activity can still cause false triggers

## Future Improvements

- ML-based signal classification
- Mobile app + cloud alerts
- Improved power management
- Advanced filtering for EMG noise
