| Supported Target | Jetson Orin Nano Dev Kit / STM32F411 |
| ----------------- | ------------------------------------ |

# WRO-MetallicMadness  
## Metallic-Madness Team – WRO 2026 Future Engineers

---

## Introduction

Welcome to the official repository of the **Metallic-Madness Team** for the **WRO 2026 – Future Engineers** category.

This project focuses on the development of a fully autonomous self-driving vehicle built on a **dual-architecture system**:

- ?? **NVIDIA Jetson Orin Nano (SBC)** – High-level processing  
- ?? **STM32F411 (SBM)** – Real-time low-level control  

The system is designed with a clear architectural separation:

### ?? Jetson Orin Nano (High-Level Intelligence)
Responsible for:
- Computer vision (YOLO-based object detection)
- Lane and obstacle analysis
- Decision-making and path planning
- Sending structured motion commands to STM32 using a custom communication protocol (**DD Protocol**)

### ?? STM32F411 (Real-Time Control Layer)
Responsible for:
- Motor control (MD10C driver with PWM and encoder feedback)
- Reading distance sensors (VL53L7CH – 8×8 ToF, VL53L4CD – dual ToF)
- Reading IMU data
- Executing motion commands received from the Jetson
- Sending sensor feedback through UART

This distributed architecture ensures:
- Deterministic motor control
- High-performance AI processing
- Reliable real-time communication between systems

Detailed technical information is available in the Technical Specification Document.

---

## Repository Structure

This repository follows the official WRO structure guidelines:

* `t-photos`  
  Contains two team photos:
  - Official team photo  
  - Fun/creative team photo  

* `v-photos`  
  Contains vehicle photos:
  - Front, back, left, right  
  - Top  
  - Bottom  

* `video`  
  Contains `video.md` with the link to the driving demonstration video.

* `schemes`  
  Contains schematic diagrams (JPEG/PNG/PDF) illustrating:
  - STM32 connections
  - Motor driver wiring
  - Sensor connections (VL53L7CH, VL53L4CD, IMU)
  - Power distribution
  - SBC ? STM32 communication

* `src`  
  Contains all source code for the system:
  
  - **stm32-firmware**  
    Firmware for STM32F411 including:
    - Motor driver module
    - Encoder handling
    - VL53 sensor drivers
    - IMU driver
    - DD Protocol UART communication
  
  - **jetson-ai**  
    Jetson Orin Nano code including:
    - Computer vision pipeline
    - YOLO detection
    - Decision logic
    - Command transmission to STM32

* `models`  
  Contains CAD / 3D printing files used for:
  - Sensor mounts
  - Structural components
  - Custom mechanical parts

* `other`  
  Additional documentation such as:
  - Communication protocol description
  - Hardware setup instructions
  - System architecture diagrams
  - Technical specification document
  - Calibration procedures

---

## System Architecture Overview
