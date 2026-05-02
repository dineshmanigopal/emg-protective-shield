# EMG-Based Smart Protective Shield with GSM Alerts

> An intelligent wearable safety system that automatically detects emergency situations using muscle signals and triggers real-time alerts with location tracking.

## Overview

This project presents an EMG-based wearable safety device that enhances personal safety through automatic emergency detection and response. Unlike traditional systems that rely on manual triggers, it uses electromyography (EMG) signals to detect abnormal muscle activity. The system automatically initiates emergency actions and is based on a final year B.Tech project report.

## Key Features

- Automatic emergency detection without manual input
- GSM-based SMS alerts to predefined contacts
- Real-time GPS location sharing
- Servo-based protective shield activation
- Portable, battery-powered wearable design
- Manual reset using a push button

## System Architecture

```
EMG Sensor → ESP32-S3 → Decision Logic →
   ├── GSM Module (Alert SMS)
   ├── GPS Module (Location)
   └── Servo Motor (Shield Actuation)
```

## Hardware Components

| Component               | Description                         |
| ----------------------- | ----------------------------------- |
| ESP32-S3                | Main microcontroller for processing |
| EMG Sensor              | Detects muscle activity signals     |
| SIM800L GSM             | Sends emergency SMS alerts          |
| NEO-6M GPS              | Provides real-time location         |
| SG90 Servo Motor        | Activates protective shield         |
| Li-ion Battery + MT3608 | Power supply unit                   |
| Push Button             | Manual reset                        |

## Working Principle

1. EMG sensor captures muscle activity signals.
2. ESP32 processes signals using ADC.
3. Signal is compared with a predefined threshold.
4. If an abnormal condition is detected:
   - Servo motor activates the protective shield.
   - GSM sends an emergency SMS.
   - GPS location is attached to the alert.
5. System remains in emergency mode until reset.

## Algorithm Flow

- Initialize all modules.
- Continuously monitor EMG signals.
- Compare with threshold.
- Trigger emergency actions if exceeded.
- Maintain state until manual reset.

## Limitations

- Signal noise from improper sensor placement
- Variable thresholds across different users
- GSM network dependency
- False triggers during intense physical activity

## Future Improvements

- Machine learning for better EMG classification
- Cloud and mobile app integration
- Improved power efficiency
- Advanced signal filtering techniques
- Miniaturized wearable design

## Software and Tools

- Arduino IDE
- Embedded C
- ESP32 Libraries
- TinyGPS++ Library
- ESP32Servo Library

## Authors

- Pathivada Suneel Kumar
- Dinesh Mani Gopal
- Kandipilli Vinay Kumar
- Pisini Bhanu Prasad

## Institution

Vignan’s Institute of Information Technology (Autonomous)  
Department of Electronics and Communication Engineering

## License

This project is developed for academic purposes. You may reuse with proper credit.
