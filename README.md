# 🔥 Smart Fire Extinguisher & Sonar System (IoT Based)

![Project](https://img.shields.io/badge/Project-IoT%20Fire%20System-red)
![Platform](https://img.shields.io/badge/Platform-ESP8266-blue)
![Status](https://img.shields.io/badge/Status-Completed-brightgreen)

---

## 📌 Overview

Proyek ini merupakan sistem **pemadam kebakaran otomatis berbasis IoT** yang mampu mendeteksi api, menentukan arah sumber api, dan memadamkannya secara otomatis menggunakan air.

Sistem dikendalikan oleh **ESP8266** dan dapat dimonitor secara real-time melalui aplikasi IoT.

---

## 🚀 Features

* 🔥 Deteksi api otomatis (Flame Sensor KY-026)
* 📏 Pengukuran jarak (Ultrasonic HC-SR04)
* 🔄 Servo scanning (mencari arah api)
* 🎯 Servo tracking (mengarah ke api)
* 💧 Pompa air otomatis
* 🔔 Alarm buzzer
* 📟 LCD display (status sistem)
* 📱 Monitoring IoT (Blynk)

---

## 🧠 System Workflow

1. Sensor flame mendeteksi api (IR)
2. Servo scan menyapu area
3. Ultrasonic membaca jarak
4. Sistem menentukan arah api
5. Jika api terdeteksi:

   * Servo mengarah ke api
   * Pompa aktif
   * Buzzer menyala
6. Data ditampilkan di LCD & Blynk

---

## 🧩 Components

* ESP8266 / NodeMCU
* Flame Sensor (KY-026)
* Ultrasonic Sensor (HC-SR04)
* Servo SG90 (2x)
* Relay Module
* Water Pump
* LCD 16x2 I2C
* Buzzer
* Power Supply 5V

---

## 🔌 Wiring

| Komponen     | Pin |
| ------------ | --- |
| Flame Sensor | A0  |
| HC-SR04 TRIG | D7  |
| HC-SR04 ECHO | D8  |
| Servo Scan   | D5  |
| Servo Pump   | D6  |
| Relay        | D1  |
| Buzzer       | D2  |
| LCD SDA      | D3  |
| LCD SCL      | D4  |

---

## 📱 IoT (Blynk)

| Virtual Pin | Fungsi           |
| ----------- | ---------------- |
| V0          | Sudut servo scan |
| V1          | Status pompa     |
| V2          | Status api       |
| V3          | Jarak            |
| V4          | Sudut servo pump |
| V7          | Mode             |
| V8          | Sensitivitas     |

---

## 🖥️ LCD Output

```
Fire7:Yes AUTO
SCAN:090 DIST:25
```

---

## 📁 Project Structure

```
.
├── data/       # Data & hasil pengujian
├── docs/       # Laporan & dokumentasi
├── images/     # Gambar & hasil percobaan
├── src/        # Source code Arduino
└── README.md
```

---

## 🖼️ Preview

Tambahkan gambar di folder `/images` lalu tampilkan di sini:

```md
![Image](https://github.com/user-attachments/assets/252faf27-288d-4396-a185-a06925704149)
```

---

## ⚙️ How to Run

1. Upload kode ke ESP8266
2. Hubungkan komponen sesuai wiring
3. Set WiFi & Blynk token
4. Jalankan sistem
5. Monitor via LCD atau aplikasi

---

## 📊 Result

* Deteksi api akurat
* Servo stabil (15°–165°)
* Pompa aktif otomatis
* Monitoring real-time berhasil

---

## 👨‍💻 Authors

* Hemart (Hema Tata Nugraha)
* Barron Aswin Saffir
* Zulfi Azmi

---

## 💡 Future Development

* AI Fire Detection (Camera)
* Notifikasi Telegram/WhatsApp
* Mobile Robot Firefighter

---

## ⭐ Support

Jika project ini membantu, jangan lupa ⭐ repo ini!

---
