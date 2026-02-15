# Datasheets – Hardware Component References

This folder contains official manufacturer datasheets and technical documentation for all major hardware components used in the Metallic-Madness WRO 2026 vehicle.

These documents were used during:

- Electrical design
- Firmware development
- Sensor integration
- Motor control calibration
- System architecture planning

---

## Components Included

### NVIDIA Jetson Orin Nano
High-performance SBC responsible for AI and decision-making.

File:
- Jetson-Orin-Nano-Series-Modules-Datasheet_DS-11105-001_v1.5.pdf

---

### STM32F411CE Microcontroller
Main real-time controller handling sensors and motor driver.

File:
- stm32f411ce.pdf

---

### Cytron MD10C Motor Driver
10A DC motor driver used for propulsion control.

File:
- cython Motor driver diagram MD10C datasheet.pdf

---

### 12V Double Shaft DC Gear Motor with Hall Encoder
Main drive motor with integrated encoder feedback.

File:
- Double Shaft DC Gear Motor Encoder 12V Hall.pdf

---

### VL53L7CH (8×8 Time-of-Flight Sensor)
Front multi-zone ranging sensor for obstacle perception.

File:
- vl53l7ch datasheet.pdf

---

### VL53L4CD (Single-Zone ToF Sensor)
Dual side distance sensors (Left and Right configuration).

File:
- vl53l4cd.pdf

---

### Pololu Maestro Servo Controller
Steering control module for precise servo positioning.

File:
- Pololu Maestro Servo Controller User guide.pdf

---

## Purpose of This Folder

These datasheets ensure:

- Correct voltage levels
- Safe operating conditions
- Accurate timing configuration
- Reliable communication setup
- Proper motor and encoder interfacing

All hardware integration decisions in this project are based on these official documents.