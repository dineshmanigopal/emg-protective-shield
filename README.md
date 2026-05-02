# EMG Protective Shield

In recent years, personal safety has become a major concern, especially for
individuals in vulnerable situations. This project presents an **EMG-based
wearable safety device** designed to detect abnormal muscle activity and
automatically trigger emergency responses.

The system utilises an EMG bio-amp sensor to monitor muscle signals from the
user's body. These signals are processed by an **ESP32-S3** microcontroller to
identify emergency conditions. Upon detecting abnormal or sudden muscle-activity
patterns the system:

1. Activates a **GSM module (SIM800L)** to send SMS alerts to pre-defined
   emergency contacts.
2. Reads real-time location data from a **GPS module (NEO-6M)** and appends a
   Google Maps link to the alert message.
3. Deploys a **servo motor** to trigger a physical protective or alerting
   action.

The device is powered by a compact **3.7 V Li-ion battery** with an **MT3608
boost converter** to ensure a stable voltage supply across all subsystems.

**Keywords:** Electromyography (EMG), Wearable Safety Device, ESP32-S3, GSM
Module (SIM800L), GPS Tracking (NEO-6M), Emergency Alert System.

---

## Repository structure

```
emg-protective-shield/
├── platformio.ini          # PlatformIO build configuration
└── src/
    ├── config.h            # Pin assignments, thresholds, contact numbers
    ├── emg_sensor.h/.cpp   # EMG signal acquisition & abnormal-activity detection
    ├── gsm_module.h/.cpp   # SIM800L AT-command driver (SMS alerts)
    ├── gps_module.h/.cpp   # NEO-6M NMEA parser via TinyGPSPlus
    ├── servo_control.h/.cpp# Servo motor protective-action control
    └── main.cpp            # Top-level state machine integrating all modules
```

---

## Hardware

| Component | Part | Notes |
|-----------|------|-------|
| Microcontroller | ESP32-S3 DevKitC-1 (N8R8) | Dual-core Xtensa LX7, 240 MHz |
| EMG sensor | Muscle BioAmp Shield / DIY bio-amp | Analog output to GPIO 4 |
| GSM module | SIM800L | UART1: TX→GPIO17, RX→GPIO18 |
| GPS module | NEO-6M | UART2: TX→GPIO15, RX→GPIO16 |
| Servo motor | SG90 / MG90S | PWM on GPIO 13 |
| Battery | 3.7 V Li-ion (18650) | |
| Boost converter | MT3608 | Outputs 5 V for SIM800L / servo |
| Status LED | On-board LED | GPIO 2 |

### Wiring diagram (summary)

```
EMG Bio-Amp OUT ──────────────────────► GPIO 4  (ADC1_CH3)

SIM800L RXD ──────── GPIO 17 (UART1 TX)
SIM800L TXD ──────── GPIO 18 (UART1 RX)
SIM800L VCC ──────── MT3608 VOUT (5 V, ensure 2 A capable supply)
SIM800L GND ──────── GND

NEO-6M  RXD ──────── GPIO 15 (UART2 TX)
NEO-6M  TXD ──────── GPIO 16 (UART2 RX)
NEO-6M  VCC ──────── 3.3 V
NEO-6M  GND ──────── GND

Servo   Signal ───── GPIO 13 (PWM)
Servo   VCC ──────── MT3608 VOUT (5 V)
Servo   GND ──────── GND
```

---

## Firmware overview

### Signal processing pipeline

1. The EMG bio-amp output is sampled at **500 Hz** via the ESP32-S3 ADC
   (12-bit, 0–4095 counts).
2. Each sample is **full-wave rectified** around the ADC mid-point bias
   (~2048 counts / 1.65 V).
3. An **RMS** value is computed over a sliding window of 50 samples (~100 ms).
4. If the RMS exceeds `EMG_ALERT_THRESHOLD` for `EMG_CONSEC_OVER` consecutive
   windows the alert condition is latched.

### State machine

```
 ┌──────────────────────────────────────────────────────────────┐
 │  IDLE  ──(EMG alert)──►  ALERT  ──(5 s hold)──►  COOLDOWN  │
 │    ▲                                                  │      │
 │    └─────────────── (60 s cooldown expired) ──────────┘      │
 └──────────────────────────────────────────────────────────────┘
```

| State | Behaviour |
|-------|-----------|
| **IDLE** | Continuously samples EMG and feeds GPS parser. |
| **ALERT** | Servo deployed, LED on solid, SMS sent with GPS link. LED blinks rapidly for 5 s. |
| **COOLDOWN** | Servo retracted, slow LED heartbeat for 60 s before returning to IDLE. |

### Alert SMS format

```
EMERGENCY ALERT: Abnormal muscle activity detected. EMG RMS=<value>.
Immediate assistance required!
Location: https://maps.google.com/?q=<lat>,<lng>
```

If a GPS fix cannot be obtained within 10 s the location line reads
`Location: GPS fix unavailable`.

---

## Build & flash

### Prerequisites

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html)
  (CLI) **or** the PlatformIO IDE extension for VS Code.

### Steps

```bash
# 1. Clone the repository
git clone https://github.com/dineshmanigopal/emg-protective-shield.git
cd emg-protective-shield

# 2. Edit emergency contact numbers
#    Open src/config.h and update EMERGENCY_CONTACT_1 / EMERGENCY_CONTACT_2

# 3. Build
pio run

# 4. Flash (connect the ESP32-S3 via USB)
pio run --target upload

# 5. Open the serial monitor
pio device monitor
```

### Adjusting detection sensitivity

All tuning parameters are in `src/config.h`:

| Constant | Default | Description |
|----------|---------|-------------|
| `EMG_ALERT_THRESHOLD` | `2800` | RMS count to exceed for an alert |
| `EMG_CONSEC_OVER` | `5` | Consecutive windows above threshold before alert fires |
| `EMG_SAMPLE_RATE_HZ` | `500` | ADC sampling rate |
| `EMG_WINDOW_SIZE` | `50` | Samples per RMS window |
| `ALERT_COOLDOWN_MS` | `60000` | Post-alert lockout period (ms) |
| `ALERT_HOLD_PERIOD_MS` | `5000` | Duration the servo stays deployed and LED blinks (ms) |
| `GPS_FIX_TIMEOUT_MS` | `10000` | Max wait for GPS fix before sending without coords (ms) |

---

## License

See [LICENSE](LICENSE).

