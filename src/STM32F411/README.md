# WRO 2026 – Future Engineers (STM32F411 Firmware)

Firmware for the WRO Future Engineers 2026 self-driving car platform (Metallic-Madness Team).  
The STM32F411 is the **low-level “vehicle controller”**: it drives the motor, reads sensors, and exchanges commands/telemetry with the main SBC (NVIDIA Jetson Orin Nano) over UART using the **DD Protocol**.

---

## What the STM32 Handles (Scope)

1. **Motor control** (MD10C driver)
   - PWM speed control
   - Direction control
   - Optional “move by degrees” using an encoder

2. **Sensors**
   - **VL53L7** (8×8 ToF grid) for front perception (distance/status matrix)
   - **VL53L4CD** (2× single-zone ToF) left/right ranging (dual sensors via XSHUT + address remap)
   - **IMU** (LSM6DS3TR-C) accel/gyro (currently present but reads are commented in `main.c`)

3. **Serial communication**
   - UART protocol parser in IRQ
   - Command execution in the main loop
   - Response messages sent back to the Jetson

---

## Firmware Architecture (High Level)

- **Interrupt context (UART RX)**:
  - Bytes are collected until an `END_BYTE` is detected.
  - The full message is validated (start byte, length, checksum, end byte).
  - A decoded command is stored in `received_message` and `new_message_ready` is raised.

- **Main loop**:
  - Watches `new_message_ready`
  - Executes the matching command (motor/sensors)
  - Sends back a response using the same DD Protocol framing

This keeps time-critical RX parsing in IRQ and leaves the heavy work (sensor reads, motor actions) in the main loop.

---

## Project Files (Reviewed)

### `main.c`
**Role:** Application entry point, peripheral init, sensor bring-up, command dispatcher.

Key responsibilities:
- Initializes HAL + clock + GPIO + I2C + UART + timers
- Starts:
  - Motor PWM (`motor_init`)
  - VL53L7 ranging (`sensor_vl53l7ch_init`, `sensor_vl53l7ch_start_ranging`)
  - Dual VL53L4CD sensors (XSHUT sequencing, I2C address remap, ranging start)
- Implements the **command switch-case** driven by `received_message.command`
- Provides `_write()` to redirect `printf()` to UART1 (useful for debugging)

Notes:
- IMU init and reads are present but commented out.
- Dual VL53L4CD setup uses XSHUT pins:
  - LEFT: `PA5`
  - RIGHT: `PA7`
- Dual VL53L4CD addresses are remapped to:
  - LEFT: `0x54`
  - RIGHT: `0x56`

---

### `motor_drv.c`
**Role:** Motor driver abstraction (MD10C control + encoder utilities).

What it provides:
- `motor_init()` ? binds timer/channel for PWM + direction GPIO, starts PWM
- `motor_set_speed(0..100)` ? PWM duty update
- `motor_forward(speed)` / `motor_reverse(speed)`
- `motor_forward_degrees(deg, speed)` / `motor_reverse_degrees(deg, speed)`
  - Converts degrees into encoder ticks (uses `PULSES_PER_REV`)
  - Busy-waits until the target ticks are reached, then stops
- `motor_encoder_update(chA, chB)` ? software quadrature decode into `encoder_count`
- `motor_get_rpm()` ? estimates RPM from delta encoder count over time

Important notes (worth fixing soon):
- `motor_get_rpm()` uses **PPR = 11** in the formula, while the file defines `PULSES_PER_REV = 1595`.
  - That mismatch will produce garbage RPM unless intentionally different sensors.
- The “degrees move” functions **busy-wait in a loop** with `printf()` inside.
  - Great for debugging, terrible for real-time behavior. Consider:
    - remove the prints
    - add a timeout
    - convert to non-blocking state machine

---

### `serial_dd_protocol.c`
**Role:** DD Protocol implementation: validate, decode, respond, and UART RX ISR.

Core pieces:
- `validate_message()`:
  - checks start/end bytes
  - checks embedded length matches received length
  - checksum = XOR of bytes `[1 .. length-3]`
- `process_message()`:
  - decodes into `DD_Uart_Message`
  - sets `received_message` + `new_message_ready`
- `send_response()` and `send_response_len()`:
  - build framed response messages using the same protocol
- `DD_USART1_IRQHandler()`:
  - reads bytes while RXNE flag set
  - accumulates until `END_BYTE`, then calls `process_message()`

Important notes:
- RX buffer is **100 bytes** in the IRQ handler.
  - But `send_response_len()` can transmit up to **1024 bytes**.
  - If the Jetson ever sends long frames, STM32 RX will truncate/break.
  - Recommendation: increase RX buffer and/or enforce max payload size in the protocol.

---

### `sensor_vl53l7ch.c`
**Role:** Wrapper around ST VL53L7CX ULD API (8×8 ranging).

What it does:
- Checks sensor alive (`vl53l7cx_is_alive`)
- Initializes (`vl53l7cx_init`)
- Configures:
  - autonomous mode
  - **8×8 resolution**
  - **10 Hz** ranging frequency
  - closest target order
  - prints current integration time
- Provides convenience wrappers:
  - start/stop ranging
  - check data ready
  - read results
  - debug print per-zone

---

### `sensor_vl53l4cd.c`
**Role:** Wrapper around ST VL53L4CD API (single sensor).

Provides:
- init + ID check
- start/stop ranging
- check ready
- get result
- clear interrupt
- debug print helper

Note:
- In `main.c`, the dual-sensor approach is implemented directly (XSHUT + address change),
  so this wrapper is currently more “single-sensor utility” than the main path.

---

### `platform2.c`
**Role:** ST “platform layer” for VL53L4CD API: I2C read/write primitives + delay.

Implements:
- `VL53L4CD_RdByte/RdWord/RdDWord`
- `VL53L4CD_WrByte/WrWord/WrDWord`
- `VL53L4CD_RdMulti`
- `VL53L4CD_WaitMs`

Uses:
- `hi2c2` (declared `extern`) as the I2C bus for VL53L4CD operations.

---

### `sensor_imu.c`
**Role:** IMU driver for LSM6DS3TR-C using I2C register reads/writes.

Provides:
- `sensor_imu_init()`:
  - checks `WHO_AM_I` (expects `0x6A`)
  - config accel + gyro
- `sensor_imu_read_accel()`
- `sensor_imu_read_gyro()`

Note:
- Calls are currently commented out in `main.c`, but the driver looks ready.

---

### `stm32f4xx_hal_msp.c`
**Role:** STM32CubeMX-generated MSP init for clocks/pins/peripheral low-level setup.

Highlights from the current config:
- **I2C1**
  - PB6 SCL, PB7 SDA (AF4)
- **I2C2**
  - PB10 SCL (AF4), PB9 SDA (AF9)
- **TIM3 PWM**
  - PA6 TIM3_CH1 (motor PWM)
- **TIM9 input capture pins**
  - PA2 TIM9_CH1, PA3 TIM9_CH2 (encoder inputs)
- **USART1**
  - PA9 TX, PA10 RX + IRQ enabled

---

### `stm32f4xx_it.c`
**Role:** Interrupt vector handlers.

Key customization:
- `USART1_IRQHandler()` calls `DD_USART1_IRQHandler()` instead of `HAL_UART_IRQHandler()`,
  so **your protocol RX runs directly in the interrupt**.

---

### `system_stm32f4xx.c`
**Role:** System startup / clock definitions (Cube-generated).

Typically contains:
- core clock config variables
- vector table relocation options
- `SystemInit()` / `SystemCoreClockUpdate()`

---

## DD Protocol (UART Frame Format)

Implemented in `serial_dd_protocol.c` (constants in `serial_dd_protocol.h`).

Frame layout:

| Field | Size | Notes |
|------|------|------|
| START | 1 byte | Must match `START_BYTE` |
| LENGTH | 2 bytes | Big-endian, **total frame length in bytes** |
| COMMAND | 2 bytes | Big-endian command ID |
| DATA | N bytes | `N = LENGTH - 7` |
| CHECKSUM | 1 byte | XOR of bytes `[1 .. LENGTH-3]` |
| END | 1 byte | Must match `END_BYTE` |

---

## Supported Commands (Current `main.c`)

> Command IDs shown in hex.

| Command | Meaning | Payload | Response |
|--------|---------|---------|----------|
| `0x0001` | Read gyro | (planned) | (currently commented) |
| `0x0002` | Read accel | (planned) | (currently commented) |
| `0x0003` | Read VL53L7 (8×8) | none | Returns formatted distances + statuses |
| `0x0004` | Motor forward | none | “Forward” |
| `0x0005` | Motor reverse | none | “Reverse” |
| `0x0101` | Forward by degrees | 2 bytes: `deg` (LE in current code) | “Moved Forward by degrees” |
| `0x0102` | Reverse by degrees | 2 bytes: `deg` (LE in current code) | “Moved Reverse by degrees” |
| `0x0103` | Stop motor | none | “Stopped” |
| `0x0104` | Set speed | 1 byte: `0..100` | “Speed updated” |
| `0x0105` | Read VL53L4CD L/R | none | “LEFT: X mm \| RIGHT: Y mm” |

?? Note on degrees payload:
- Code reads degrees as:  
  `deg = data[0] | (data[1] << 8)`  
  That is **little-endian**. Your frame’s command/length are big-endian, but degrees are currently treated as little-endian. Keep it consistent on the Jetson side (or adjust firmware).

---

## Build & Flash

Typical STM32CubeIDE flow:
1. Open the project in **STM32CubeIDE**
2. Build (Debug/Release)
3. Flash via ST-LINK

Serial debug:
- `printf()` is redirected to **USART1** via `_write()` in `main.c`.

---

## Practical TODOs (Recommended Next Fixes)

If you want this to behave like a race car and not like a debugging seminar:

1. **Make “move by degrees” non-blocking**
   - remove busy-wait loops
   - implement a motion state machine + periodic encoder checks

2. **Fix encoder constants**
   - unify `PULSES_PER_REV` vs RPM PPR constant

3. **RX buffer sizing**
   - increase RX buffer beyond 100 bytes or enforce a strict max payload size

4. **IMU enablement**
   - uncomment IMU init + reads and define response payload format

---

## Ownership / Credits

- Firmware: Metallic-Madness Team (WRO Future Engineers 2026)
- Sensor APIs: STMicroelectronics VL53 ULD/ULD API layers
- MCU HAL: STM32Cube HAL (ST)

---