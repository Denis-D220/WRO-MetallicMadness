# DD-UART Protocol

**Project:** MetallicMadness / ARBIBOT  
**Location:** `other/protocol/dd-uart-protocol.md`  
**Purpose:** Document the binary UART protocol used between the NVIDIA Jetson and the STM32F411 controller.

This document describes the **DD-UART** frame format, checksum, command IDs, payload formats, response examples, and recommended parser behavior.

The protocol is based on the team's original **UART Protocol with Length and Checksum** design and updated to match the current ARBIBOT STM32/Jetson command set.

---

## 1. Protocol Role in ARBIBOT

ARBIBOT uses a two-controller architecture:

```text
NVIDIA Jetson Orin Nano  ->  high-level AI, camera, YOLO, navigation
STM32F411 Black Pill     ->  low-level sensor polling, motor control, encoder reading
```

The Jetson sends DD-UART command frames to the STM32 to:

- read IMU values,
- read front ToF matrix data,
- read side VL53L4CD distance sensors,
- command the drive motor forward or reverse,
- stop the drive motor,
- set motor speed,
- move the motor by encoder degrees.

The STM32 validates each frame, dispatches the command, and replies using the same frame format.

---

## 2. Transport Layer

| Parameter | Value |
|---|---|
| Physical link | USB serial / UART |
| Default baud rate | `115200` |
| Data bits | `8` |
| Parity | None |
| Stop bits | `1` |
| Flow control | None |
| Encoding | Binary frame with optional ASCII payload |
| Main users | Jetson Python code and STM32 firmware |

Recommended serial setting:

```text
115200 8N1
```

---

## 3. Frame Format

Every DD-UART message uses this structure:

```text
[START][LENGTH][COMMAND][DATA][CHECKSUM][END]
```

Expanded view:

```text
$  LEN_H  LEN_L  CMD_H  CMD_L  DATA...  CHK  \n
```

Visual format:

```text
$  LEN(2, big-endian)  CMD(2, big-endian)  DATA(0..n)  CHK(1)  \n
└START                                                      └XOR   └END
```

---

## 4. Field Definition

| Field | Size | Value / format | Description |
|---|---:|---|---|
| `START` | 1 byte | ASCII `$` = `0x24` | Marks the beginning of the frame |
| `LENGTH` | 2 bytes | Unsigned integer, big-endian | Total frame length in bytes, including all fields |
| `COMMAND` | 2 bytes | Unsigned integer, big-endian | Command ID |
| `DATA` | 0 to n bytes | Command-specific payload | Optional command parameters or response payload |
| `CHECKSUM` | 1 byte | XOR checksum | XOR of bytes from `LENGTH` through `DATA` |
| `END` | 1 byte | ASCII newline `\n` = `0x0A` | Marks the end of the frame |

Minimum frame:

```text
START + LENGTH + COMMAND + CHECKSUM + END
1     + 2      + 2       + 1        + 1 = 7 bytes
```

Minimum no-payload frame:

```text
24 00 07 CC CC XX 0A
```

Where:

```text
24       = '$'
00 07    = length = 7 bytes
CC CC    = command
XX       = checksum
0A       = '\n'
```

---

## 5. Length Field

The `LENGTH` field is the total number of bytes in the full frame.

```text
length = 1 + 2 + 2 + len(DATA) + 1 + 1
length = 7 + len(DATA)
```

Examples:

| Payload size | Total length | LENGTH bytes |
|---:|---:|---|
| 0 bytes | 7 | `00 07` |
| 1 byte | 8 | `00 08` |
| 2 bytes | 9 | `00 09` |
| 7 bytes | 14 | `00 0E` |

### Original design maximum

The original DD-UART draft limited the `DATA` field to `0-93 bytes`, making the maximum frame size `100 bytes`.

```text
1 + 2 + 2 + 93 + 1 + 1 = 100 bytes
```

### Current ARBIBOT implementation note

The current ARBIBOT sensor responses can include text payloads, especially front matrix data. If front matrix payloads exceed 93 bytes, the final STM32 and Jetson buffers must support a larger maximum frame size.

Recommended final check:

```text
Verify MAX_FRAME_LEN in Jetson Python code.
Verify UART RX/TX buffer size in STM32 firmware.
Verify maximum READ_FRONT response size.
```

---

## 6. Byte Order

| Field | Byte order |
|---|---|
| `LENGTH` | Big-endian |
| `COMMAND` | Big-endian |
| `DATA` | Command-specific |
| Degree payload for `0x0101` / `0x0102` | Little-endian `uint16` |
| Speed payload for `0x0104` | Single byte `uint8` |

Example command encoding:

```text
Command 0x0103 -> CMD_H = 0x01, CMD_L = 0x03
```

Example degree payload:

```text
80 degrees decimal = 0x0050
uint16 little-endian payload = 50 00
```

---

## 7. Checksum

The checksum is a 1-byte XOR checksum.

It is calculated over every byte **after START** and **before CHECKSUM**.

Included in checksum:

```text
LENGTH + COMMAND + DATA
```

Excluded from checksum:

```text
START
CHECKSUM
END
```

Formula:

```text
checksum = LEN_H ^ LEN_L ^ CMD_H ^ CMD_L ^ DATA_0 ^ DATA_1 ^ ... ^ DATA_N
```

### Python checksum function

```python
def calculate_checksum(payload: bytes) -> int:
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return checksum
```

For a full frame being created:

```python
checksum = calculate_checksum(length_bytes + command_bytes + data)
```

For a received full message:

```python
checksum = calculate_checksum(message[1:-2])
```

Where:

```text
message[1:-2] = LENGTH + COMMAND + DATA
message[-2]   = received CHECKSUM
```

---

## 8. Frame Validation Rules

A received frame is valid only if all checks pass.

Validation order:

1. Check minimum length.
2. Check `START == 0x24`.
3. Check `END == 0x0A`.
4. Decode `LENGTH`.
5. Verify `LENGTH == len(frame)`.
6. Calculate checksum over `message[1:-2]`.
7. Verify calculated checksum equals `message[-2]`.
8. Decode command.
9. Dispatch command.

### Validation pseudocode

```text
if len(frame) < 7:
    reject

if frame[0] != '$':
    reject

if frame[-1] != '\n':
    reject

received_length = (frame[1] << 8) | frame[2]

if received_length != len(frame):
    reject

calculated_checksum = XOR(frame[1:-2])

if calculated_checksum != frame[-2]:
    reject

command = (frame[3] << 8) | frame[4]
data = frame[5:-2]
dispatch(command, data)
```

---

## 9. Current ARBIBOT Command List

| Command ID | Name | Direction | Payload | Response |
|---:|---|---|---|---|
| `0x0001` | `READ_GYRO` | Jetson -> STM32 | None | IMU gyro values |
| `0x0002` | `READ_ACCEL` | Jetson -> STM32 | None | IMU accelerometer values |
| `0x0003` | `READ_FRONT` | Jetson -> STM32 | None | Front VL53L7CX/L8 matrix text |
| `0x0004` | `FORWARD_CONTINUOUS` | Jetson -> STM32 | None | `"Forward"` |
| `0x0005` | `REVERSE_CONTINUOUS` | Jetson -> STM32 | None | `"Reverse"` |
| `0x0101` | `FORWARD_DEGREES` | Jetson -> STM32 | `uint16 LE degrees` | Ack / movement result |
| `0x0102` | `REVERSE_DEGREES` | Jetson -> STM32 | `uint16 LE degrees` | Ack / movement result |
| `0x0103` | `STOP` | Jetson -> STM32 | None | `"Stopped"` |
| `0x0104` | `SET_SPEED` | Jetson -> STM32 | `uint8 percent`, `0-100` | `"Speed updated"` |
| `0x0105` | `READ_SIDES` | Jetson -> STM32 | None | Side VL53L4CD text payload |

---

## 10. Legacy Command Mapping

The original protocol draft used a simpler command list:

| Legacy command | Original description |
|---:|---|
| `00` | Reset peripheral sensors |
| `01` | Read IMU gyro |
| `02` | Read IMU accelerometer |
| `03` | Read distance sensor |
| `04` | Read continuous distance sensor |
| `05` | Motor forward |
| `06` | Motor reverse |
| `07` | Motor stop |
| `08` | Motor speed |

ARBIBOT currently uses the expanded 16-bit command IDs listed above. The current table should be treated as the active project command map.

---

## 11. Payload Formats

### 11.1 No-payload commands

These commands use no `DATA` bytes:

```text
0x0001 READ_GYRO
0x0002 READ_ACCEL
0x0003 READ_FRONT
0x0004 FORWARD_CONTINUOUS
0x0005 REVERSE_CONTINUOUS
0x0103 STOP
0x0105 READ_SIDES
```

No-payload frame length:

```text
7 bytes
```

### 11.2 `SET_SPEED` payload

Command:

```text
0x0104 SET_SPEED
```

Payload:

```text
uint8 speed_percent
```

Range:

```text
0 to 100
```

Example:

```text
80% speed = 0x50
```

Frame length:

```text
8 bytes
```

### 11.3 `FORWARD_DEGREES` payload

Command:

```text
0x0101 FORWARD_DEGREES
```

Payload:

```text
uint16 degrees, little-endian
```

Example:

```text
80 degrees = 0x0050 = 50 00
```

Frame length:

```text
9 bytes
```

### 11.4 `REVERSE_DEGREES` payload

Command:

```text
0x0102 REVERSE_DEGREES
```

Payload:

```text
uint16 degrees, little-endian
```

Example:

```text
80 degrees = 0x0050 = 50 00
```

Frame length:

```text
9 bytes
```

---

## 12. Command Frame Examples

### 12.1 `STOP` command

Command:

```text
0x0103 STOP
```

Frame fields:

| Field | Bytes |
|---|---|
| START | `24` |
| LENGTH | `00 07` |
| COMMAND | `01 03` |
| DATA | none |
| CHECKSUM | `05` |
| END | `0A` |

Full frame:

```text
24 00 07 01 03 05 0A
```

Calculation:

```text
00 ^ 07 ^ 01 ^ 03 = 05
```

---

### 12.2 `SET_SPEED 80%` command

Command:

```text
0x0104 SET_SPEED
```

Payload:

```text
80 decimal = 0x50
```

Frame fields:

| Field | Bytes |
|---|---|
| START | `24` |
| LENGTH | `00 08` |
| COMMAND | `01 04` |
| DATA | `50` |
| CHECKSUM | `5D` |
| END | `0A` |

Full frame:

```text
24 00 08 01 04 50 5D 0A
```

Calculation:

```text
00 ^ 08 ^ 01 ^ 04 ^ 50 = 5D
```

---

### 12.3 `FORWARD_CONTINUOUS` command

Command:

```text
0x0004 FORWARD_CONTINUOUS
```

Frame:

```text
24 00 07 00 04 03 0A
```

Calculation:

```text
00 ^ 07 ^ 00 ^ 04 = 03
```

---

### 12.4 `REVERSE_CONTINUOUS` command

Command:

```text
0x0005 REVERSE_CONTINUOUS
```

Frame:

```text
24 00 07 00 05 02 0A
```

Calculation:

```text
00 ^ 07 ^ 00 ^ 05 = 02
```

---

### 12.5 `READ_FRONT` command

Command:

```text
0x0003 READ_FRONT
```

Frame:

```text
24 00 07 00 03 04 0A
```

Calculation:

```text
00 ^ 07 ^ 00 ^ 03 = 04
```

Expected response:

```text
Command: 0x0003
Payload: ASCII text containing front matrix distances and statuses
```

Example response payload format:

```text
Distances:
123 124 126 130
118 119 121 125
...
Statuses:
5 5 5 5
5 5 5 5
...
```

Final response format depends on STM32 firmware.

---

### 12.6 `READ_SIDES` command

Command:

```text
0x0105 READ_SIDES
```

Frame:

```text
24 00 07 01 05 03 0A
```

Calculation:

```text
00 ^ 07 ^ 01 ^ 05 = 03
```

Expected response payload:

```text
RIGHT_FRONT: a mm | RIGHT_REAR: b mm | LEFT: c mm
```

Example:

```text
RIGHT_FRONT: 320 mm | RIGHT_REAR: 335 mm | LEFT: 410 mm
```

---

### 12.7 `FORWARD_DEGREES 80` command

Command:

```text
0x0101 FORWARD_DEGREES
```

Payload:

```text
80 degrees = 0x0050 = 50 00
```

Frame fields:

| Field | Bytes |
|---|---|
| START | `24` |
| LENGTH | `00 09` |
| COMMAND | `01 01` |
| DATA | `50 00` |
| CHECKSUM | `59` |
| END | `0A` |

Full frame:

```text
24 00 09 01 01 50 00 59 0A
```

Calculation:

```text
00 ^ 09 ^ 01 ^ 01 ^ 50 ^ 00 = 59
```

---

### 12.8 `REVERSE_DEGREES 80` command

Command:

```text
0x0102 REVERSE_DEGREES
```

Payload:

```text
80 degrees = 0x0050 = 50 00
```

Frame fields:

| Field | Bytes |
|---|---|
| START | `24` |
| LENGTH | `00 09` |
| COMMAND | `01 02` |
| DATA | `50 00` |
| CHECKSUM | `5A` |
| END | `0A` |

Full frame:

```text
24 00 09 01 02 50 00 5A 0A
```

Calculation:

```text
00 ^ 09 ^ 01 ^ 02 ^ 50 ^ 00 = 5A
```

---

## 13. Response Frame Format

STM32 responses use the same frame structure:

```text
[START][LENGTH][COMMAND][DATA][CHECKSUM][END]
```

Where:

| Field | Meaning |
|---|---|
| `COMMAND` | Usually echoes the command ID |
| `DATA` | ASCII response text or binary payload |
| `CHECKSUM` | XOR of `LENGTH + COMMAND + DATA` |

Example response payloads:

| Command | Example payload |
|---:|---|
| `0x0001` | `GYRO: x,y,z` |
| `0x0002` | `ACCEL: x,y,z` |
| `0x0003` | `Distances: ... Statuses: ...` |
| `0x0004` | `Forward` |
| `0x0005` | `Reverse` |
| `0x0103` | `Stopped` |
| `0x0104` | `Speed updated` |
| `0x0105` | `RIGHT_FRONT: a mm | RIGHT_REAR: b mm | LEFT: c mm` |

---

## 14. Python Reference Implementation

```python
from __future__ import annotations

START_BYTE = b"$"
END_BYTE = b"\n"

def calculate_checksum(data: bytes) -> int:
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def create_frame(command: int, data: bytes = b"") -> bytes:
    if not (0 <= command <= 0xFFFF):
        raise ValueError("command must fit in uint16")

    if data is None:
        data = b""

    length = 7 + len(data)

    length_bytes = length.to_bytes(2, "big")
    command_bytes = command.to_bytes(2, "big")

    checksum = calculate_checksum(length_bytes + command_bytes + data)

    return (
        START_BYTE
        + length_bytes
        + command_bytes
        + data
        + bytes([checksum])
        + END_BYTE
    )

def validate_frame(frame: bytes) -> bool:
    if len(frame) < 7:
        return False

    if frame[0:1] != START_BYTE:
        return False

    if frame[-1:] != END_BYTE:
        return False

    length = int.from_bytes(frame[1:3], "big")

    if length != len(frame):
        return False

    expected_checksum = calculate_checksum(frame[1:-2])
    received_checksum = frame[-2]

    return expected_checksum == received_checksum

def parse_frame(frame: bytes) -> tuple[int, bytes]:
    if not validate_frame(frame):
        raise ValueError("Invalid DD-UART frame")

    command = int.from_bytes(frame[3:5], "big")
    data = frame[5:-2]

    return command, data

def payload_u8(value: int) -> bytes:
    if not (0 <= value <= 255):
        raise ValueError("value must fit in uint8")
    return bytes([value])

def payload_u16_le(value: int) -> bytes:
    if not (0 <= value <= 0xFFFF):
        raise ValueError("value must fit in uint16")
    return value.to_bytes(2, "little")

# Examples
STOP_FRAME = create_frame(0x0103)
SET_SPEED_80_FRAME = create_frame(0x0104, payload_u8(80))
FORWARD_80_DEG_FRAME = create_frame(0x0101, payload_u16_le(80))

print(STOP_FRAME.hex(" "))
print(SET_SPEED_80_FRAME.hex(" "))
print(FORWARD_80_DEG_FRAME.hex(" "))
```

Expected output:

```text
24 00 07 01 03 05 0a
24 00 08 01 04 50 5d 0a
24 00 09 01 01 50 00 59 0a
```

---

## 15. STM32 Validation Logic

Recommended STM32 validation steps:

```c
#define START_BYTE '$'
#define END_BYTE '\n'

int validate_message(uint8_t *message, uint16_t length)
{
    if (length < 7) {
        return 0;
    }

    if (message[0] != START_BYTE || message[length - 1] != END_BYTE) {
        return 0;
    }

    uint16_t received_length = ((uint16_t)message[1] << 8) | message[2];

    if (received_length != length) {
        return 0;
    }

    uint8_t checksum = 0;

    for (uint16_t i = 1; i < length - 2; i++) {
        checksum ^= message[i];
    }

    if (checksum != message[length - 2]) {
        return 0;
    }

    return 1;
}
```

Command decode:

```c
uint16_t command = ((uint16_t)message[3] << 8) | message[4];
uint8_t *data = &message[5];
uint16_t data_len = length - 7;
```

---

## 16. Recommended Parser Behavior

The parser should be robust to partial frames and stale bytes.

Recommended behavior:

1. Read until `START_BYTE`.
2. Read the two length bytes.
3. Reject if length is below 7 or above `MAX_FRAME_LEN`.
4. Read the remaining bytes.
5. Validate `END_BYTE`.
6. Validate checksum.
7. Dispatch command.
8. If validation fails, discard bytes until the next `$`.

### Why this matters

During robot operation:

- sensor requests can be frequent,
- front matrix responses can be long,
- motor commands may be sent while sensor polling is active,
- USB serial can delay bytes,
- a single dropped STOP command can cause a crash.

---

## 17. Threading and Serial Ownership Rule

The Jetson must avoid having multiple threads read from the same serial port at the same time.

Recommended rule:

```text
Only one thread owns serial reads.
```

In the current ARBIBOT architecture:

- sensor manager owns serial reads,
- sensor data is cached,
- motor commands are often sent fire-and-forget,
- critical commands are pulsed,
- the camera/YOLO loop should not block on motor command responses.

This prevents motor commands from blocking camera inference and prevents two parts of the program from stealing each other's response frames.

---

## 18. Critical Command Policy

Some commands are safety-critical.

Safety-critical commands:

```text
0x0103 STOP
0x0004 FORWARD_CONTINUOUS
0x0104 SET_SPEED
```

Recommended behavior:

```text
Send critical commands more than once, with a short delay.
```

Example stop policy:

```text
STOP -> wait 20 ms -> STOP -> wait 20 ms -> STOP
```

Reason:

```text
A single frame can be dropped or delayed while the serial bus is busy.
```

---

## 19. Common Failure Modes

| Failure | Likely cause | Mitigation |
|---|---|---|
| Invalid checksum | Corrupted frame or parser offset | Discard frame and resync at next `$` |
| Length mismatch | Partial frame or stale bytes | Flush/resync |
| Unknown command | Version mismatch between Jetson and STM32 | Check command table |
| Motor does not stop | STOP frame dropped or command parser busy | Pulse STOP command |
| Camera loop slows down | Blocking motor response wait | Use fire-and-forget motor commands where safe |
| Sensor response parsed by motor function | Multiple readers on same serial port | Single serial-read owner |
| Front matrix response truncated | Buffer too small | Increase `MAX_FRAME_LEN` and firmware TX buffer |
| Degree command moves wrong amount | Payload endian or encoder constants wrong | Use `uint16 LE` and verify encoder ticks |

---

## 20. Command Test Checklist

Before running autonomous code, test commands manually.

| Test | Command | Expected result | Pass |
|---|---:|---|---|
| T-001 | `0x0103 STOP` | Motor stopped | [TODO] |
| T-002 | `0x0104 SET_SPEED 50` | Speed updates | [TODO] |
| T-003 | `0x0004 FORWARD_CONTINUOUS` | Motor moves forward | [TODO] |
| T-004 | `0x0103 STOP` | Motor stops | [TODO] |
| T-005 | `0x0005 REVERSE_CONTINUOUS` | Motor moves reverse | [TODO] |
| T-006 | `0x0101 FORWARD_DEGREES 80` | Motor moves forward by command amount | [TODO] |
| T-007 | `0x0102 REVERSE_DEGREES 80` | Motor moves reverse by command amount | [TODO] |
| T-008 | `0x0105 READ_SIDES` | Returns right-front, right-rear, left distances | [TODO] |
| T-009 | `0x0003 READ_FRONT` | Returns front matrix data | [TODO] |

---

## 21. Recommended Repository Location

This document should be stored here:

```text
other/protocol/dd-uart-protocol.md
```

Optional related files:

```text
other/protocol/dd-uart-frame-examples.md
other/protocol/dd-uart-test-log.md
other/protocol/dd-uart-python-reference.py
```

---

## 22. Change Log

| Date | Change | Notes |
|---|---|---|
| [TODO] | Initial DD-UART protocol document created | Based on original length/checksum protocol and updated ARBIBOT command IDs |
| [TODO] | Verify final STM32 buffer size | Needed for front matrix response |
| [TODO] | Add final response payload samples | Capture from real STM32 firmware |
| [TODO] | Add final serial test log | Use command checklist |

---

## 23. Conclusion

The DD-UART protocol gives ARBIBOT a compact and reliable command format between the Jetson and STM32.

The important protocol rules are:

- frames start with `$` and end with `\n`,
- `LENGTH` and `COMMAND` are big-endian,
- checksum is XOR over `LENGTH + COMMAND + DATA`,
- no-payload commands are 7 bytes,
- motor degree payloads use `uint16 little-endian`,
- speed uses one byte from `0` to `100`,
- responses use the same frame format,
- serial reads should have a single owner,
- critical commands should be repeated.

This protocol is simple enough to debug with hex logs but structured enough to protect the robot from corrupted or incomplete serial messages.
