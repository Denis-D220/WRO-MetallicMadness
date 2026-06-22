# Protocol

This folder contains communication protocol documentation for the **MetallicMadness / ARBIBOT** WRO Future Engineers self-driving car project.

The main purpose of this folder is to document how the robot’s high-level controller and low-level controller communicate. In ARBIBOT, the NVIDIA Jetson Orin Nano runs AI, camera processing, YOLO inference, and navigation logic, while the STM32F411 handles motor control, sensor acquisition, encoder reading, and low-level I/O.

The protocol documentation in this folder makes the communication layer easier to understand, debug, reproduce, and extend.

## Folder contents

| File | Description |
|---|---|
| `dd-uart-protocol.md` | Main DD-UART protocol documentation. It explains the frame format, length field, checksum calculation, command IDs, payload formats, response examples, validation rules, Python reference code, STM32 validation logic, serial ownership rules, and test checklist. |
| `UART Protocol with Length and Checksum.pdf` | Original reference document for the UART frame structure, length field, checksum method, command examples, and validation approach. Keep this file if it is part of the project history or protocol design reference. |

## DD-UART protocol summary

ARBIBOT uses a framed binary UART protocol between the Jetson and the STM32.

Frame format:

```text
[START][LENGTH][COMMAND][DATA][CHECKSUM][END]
```

Expanded view:

```text
$  LEN_H  LEN_L  CMD_H  CMD_L  DATA...  CHK  \n
```

Field summary:

| Field | Size | Description |
|---|---:|---|
| `START` | 1 byte | ASCII `$`, marks the beginning of the frame |
| `LENGTH` | 2 bytes | Total frame length, big-endian |
| `COMMAND` | 2 bytes | Command ID, big-endian |
| `DATA` | 0 or more bytes | Optional command payload |
| `CHECKSUM` | 1 byte | XOR checksum over `LENGTH + COMMAND + DATA` |
| `END` | 1 byte | ASCII newline `\n`, marks the end of the frame |

Minimum frame length:

```text
7 bytes
```

## Current command groups

The DD-UART protocol is used for:

- reading IMU gyro data,
- reading IMU accelerometer data,
- reading the front VL53L8CH / VL53L7CX-style matrix sensor,
- reading side VL53L4CD distance sensors,
- commanding the motor forward,
- commanding the motor reverse,
- stopping the motor,
- setting motor speed,
- moving forward by encoder degrees,
- moving reverse by encoder degrees.

## Active command IDs

| Command ID | Name | Purpose |
|---:|---|---|
| `0x0001` | `READ_GYRO` | Read IMU gyro values |
| `0x0002` | `READ_ACCEL` | Read IMU accelerometer values |
| `0x0003` | `READ_FRONT` | Read front ToF matrix sensor |
| `0x0004` | `FORWARD_CONTINUOUS` | Start continuous forward motor motion |
| `0x0005` | `REVERSE_CONTINUOUS` | Start continuous reverse motor motion |
| `0x0101` | `FORWARD_DEGREES` | Move forward by encoder degrees |
| `0x0102` | `REVERSE_DEGREES` | Move reverse by encoder degrees |
| `0x0103` | `STOP` | Stop the motor |
| `0x0104` | `SET_SPEED` | Set motor speed percentage |
| `0x0105` | `READ_SIDES` | Read right-front, right-rear, and left VL53L4CD sensors |

## Example frames

### STOP

```text
24 00 07 01 03 05 0A
```

Meaning:

```text
START    = 24
LENGTH   = 00 07
COMMAND  = 01 03
CHECKSUM = 05
END      = 0A
```

### SET_SPEED 80%

```text
24 00 08 01 04 50 5D 0A
```

Meaning:

```text
COMMAND = 0x0104
DATA    = 0x50 = 80 decimal
```

### FORWARD continuous

```text
24 00 07 00 04 03 0A
```

Meaning:

```text
COMMAND = 0x0004
DATA    = none
```

## Related robot software

This protocol is used by both Jetson Python code and STM32 firmware.

Relevant areas:

```text
src/motor_driver.py
src/sensor_distance_v3.py
src/main_challenge_01_v4.py
src/main_challenge_02.py
STM32 firmware serial protocol files
STM32 motor driver files
STM32 sensor driver files
```

## Important implementation rules

1. **Only one process or thread should own serial reads.**  
   Multiple readers can steal each other’s response frames.

2. **Critical commands should be repeated.**  
   Commands such as `STOP` should be pulsed several times because a single frame can be dropped or delayed during heavy serial traffic.

3. **The camera loop should not be blocked by motor communication.**  
   Blocking motor command waits can reduce YOLO detection timing and cause missed pillar detections.

4. **All frames must be validated.**  
   The parser should reject invalid start bytes, invalid end bytes, length mismatch, and checksum mismatch.

5. **Protocol changes must be documented.**  
   If a command ID, payload format, or response format changes in firmware, update `dd-uart-protocol.md`.

## Recommended future files

The following files can be added later if needed:

| File | Purpose |
|---|---|
| `dd-uart-python-reference.py` | Small standalone Python script for creating, sending, and validating DD-UART frames |
| `dd-uart-frame-examples.md` | Extra frame examples and checksum calculations |
| `dd-uart-test-log.md` | Manual command test results |
| `stm32-command-map.md` | Firmware-side command dispatch map |
| `serial-debug-notes.md` | Debug notes for UART timeout, checksum, and parser issues |

## Documentation links

Related project documents:

```text
docs/03-software-architecture.md
docs/06-testing-and-tuning.md
docs/07-calibration-procedures.md
docs/10-risk-register.md
schemes/full-wiring-diagram.png
```

Example link from the root `README.md`:

```markdown
[DD-UART Protocol](other/protocol/dd-uart-protocol.md)
```

Example link from a file inside `docs/`:

```markdown
[DD-UART Protocol](../other/protocol/dd-uart-protocol.md)
```

## Update policy

When the protocol changes:

1. Update the STM32 firmware.
2. Update the Jetson Python command code.
3. Update `dd-uart-protocol.md`.
4. Update this README if new files are added.
5. Add or update command examples.
6. Run the manual command test checklist.
7. Commit the code and documentation together when possible.

## Current status

The folder currently documents the DD-UART protocol used by ARBIBOT for Jetson-to-STM32 communication. The active protocol uses binary frames with a start byte, total length, 16-bit command ID, optional payload, XOR checksum, and newline end byte.
