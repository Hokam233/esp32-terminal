# ESP32 Access Terminal with Matrix Keypad and BLE Control

##  Project Overview

This project implements a secure access terminal using an ESP32 microcontroller. The system features:

- **Matrix keypad** (manual GPIO reading, no libraries allowed)
- **Two LED indicators** for status (green = open, red = closed)
- **Bluetooth Low Energy (BLE)** communication with a **WebBluetooth-based mobile-friendly UI**
- **Non-volatile configuration storage (NVS)** for password and unlock duration

Designed to meet the specifications of an academic embedded systems project.

---

##  Features

- Manual scanning of a 4x3 matrix keypad via GPIO interrupts.
- User-defined 4-digit password and configurable open duration (in seconds).
- Visual feedback using PWM-driven LEDs.
- Secure password change process through keypad input.
- BLE interface for remote control and configuration.
- Data persistence across reboots via NVS.
- Fully coded in C using ESP-IDF (no Arduino).

---

## Hardware Requirements

- ESP32 development board
- 4x3 matrix keypad
- 2 LEDs (Red and Green)
- Suitable resistors and wiring
- Power supply (USB or battery)

---


