# 🚀 EMG-Based Smart Protective Shield with GSM Alerts

> **An intelligent wearable safety system that automatically detects emergency situations using muscle signals and triggers real-time alerts with location tracking.**

---

## 📌 Overview

This project presents an **EMG-based wearable safety device** designed to enhance personal safety through **automatic emergency detection and response**.

Unlike traditional systems that rely on manual triggers (panic buttons or apps), this system uses **electromyography (EMG) signals** to detect abnormal muscle activity and **automatically initiate emergency actions**.

📄 Based on final year B.Tech project report

---

## 🎯 Key Features

* ⚡ **Automatic Emergency Detection** (No manual input required)
* 📡 **GSM-Based SMS Alerts** to predefined contacts
* 📍 **Real-Time GPS Location Sharing**
* 🤖 **Servo-Based Protective Shield Activation**
* 🔋 **Portable Battery-Powered Wearable Design**
* 🔁 **Manual Reset using Push Button**

---

## 🧠 System Architecture

```
EMG Sensor → ESP32-S3 → Decision Logic → 
   ├── GSM Module (Alert SMS)
   ├── GPS Module (Location)
   └── Servo Motor (Shield Actuation)
```

---

## 🛠️ Hardware Components

| Component               | Description                         |
| ----------------------- | ----------------------------------- |
| ESP32-S3                | Main microcontroller for processing |
| EMG Sensor              | Detects muscle activity signals     |
| SIM800L GSM             | Sends emergency SMS alerts          |
| NEO-6M GPS              | Provides real-time location         |
| SG90 Servo Motor        | Activates protective shield         |
| Li-ion Battery + MT3608 | Power supply unit                   |
| Push Button             | Manual reset                        |

---

## ⚙️ Working Principle

1. EMG sensor captures muscle activity signals
2. ESP32 processes signals using ADC
3. Signal compared with predefined threshold
4. If abnormal condition detected:

   * 🚨 Servo motor activates protective shield
   * 📩 GSM sends emergency SMS
   * 📍 GPS location is attached to alert
5. System stays in emergency mode until reset

---

## 🔄 Algorithm Flow

* Initialize all modules
* Continuously monitor EMG signals
* Compare with threshold
* Trigger emergency actions if exceeded
* Maintain state until manual reset

---

## 📊 Results & Performance

* ✔️ Accurate detection of abnormal muscle activity
* ⚡ Fast response time
* 📡 Reliable GSM communication
* 📍 Accurate GPS tracking
* 🔁 Stable continuous monitoring system

---

## 🚧 Limitations

* Signal noise due to improper sensor placement
* Threshold varies for different users
* GSM network dependency
* Possible false triggers during intense physical activity

---

## 🔮 Future Improvements

* 🤖 Machine Learning for better EMG classification
* ☁️ Cloud & Mobile App Integration
* 🔋 Improved power efficiency
* 📉 Advanced signal filtering techniques
* 📦 Miniaturized wearable design

---

## 💻 Software & Tools

* Arduino IDE
* Embedded C
* ESP32 Libraries
* TinyGPS++ Library
* ESP32Servo Library

---

## 📂 Repository Structure (Suggested)

```
📁 EMG-Safety-System
 ┣ 📁 Code
 ┃ ┗ main.ino
 ┣ 📁 Circuit
 ┃ ┗ circuit_diagram.png
 ┣ 📁 Images
 ┃ ┗ prototype.jpg
 ┣ 📄 README.md
 ┗ 📄 Report.pdf
```

---

## 👨‍💻 Authors

* Pathivada Suneel Kumar
* **Dinesh Mani Gopal**
* Kandipilli Vinay Kumar
* Pisini Bhanu Prasad

---

## 🏫 Institution

**Vignan’s Institute of Information Technology (Autonomous)**
Department of Electronics and Communication Engineering

---

## 📜 License

This project is developed for academic purposes.
You may reuse with proper credit.

---

## ⭐ Contribution & Support

If you found this project useful:

* ⭐ Star this repo
* 🍴 Fork and improve
* 📢 Share with others

---

If you want, I can also:

* Add **badges (GitHub stats, tech stack icons)**
* Create **premium README design with banners**
* Add **demo GIFs / circuit diagrams**
* Optimize it specifically for **placements or recruiters**

Just tell me 👍
