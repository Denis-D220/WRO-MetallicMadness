# Flowchart and Communication Protocol

This folder contains documentation describing the communication protocol used between:

- Jetson Orin Nano (High-Level Controller)
- STM32F411CE (Low-Level Real-Time Controller)

The communication is implemented over UART using a custom binary protocol called:

DD Protocol

---

## Protocol File

- UART Protocol with Length and Checksum.pdf

---

## Protocol Overview

The protocol ensures structured, reliable, and validated communication between the SBC and the microcontroller.

Frame structure:

| Field        | Size     | Description |
|-------------|----------|-------------|
| Start Byte  | 1 byte   | Frame identifier |
| Length      | 2 bytes  | Total frame length |
| Command ID  | 2 bytes  | Action to execute |
| Data        | N bytes  | Optional payload |
| Checksum    | 1 byte   | XOR validation |
| End Byte    | 1 byte   | Frame termination |

---

## Purpose of the Protocol

The protocol is designed to provide:

- Frame integrity verification
- Length validation
- Command structure consistency
- Error detection using XOR checksum
- Expandable command system

---

## System Communication Flow

1. Jetson processes camera input
2. AI decides vehicle action
3. Command is packed into DD Protocol frame
4. Frame is sent via UART
5. STM32 validates frame
6. STM32 executes command
7. STM32 optionally returns sensor data

This protocol guarantees deterministic control and prevents corrupted command execution during competition.