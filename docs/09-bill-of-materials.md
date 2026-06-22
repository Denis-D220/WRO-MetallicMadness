# 09 - Bill of Materials

**Project:** MetallicMadness / ARBIBOT  
**Category:** WRO 2026 Future Engineers - Self-Driving Cars  
**Document purpose:** Document the main components used in ARBIBOT, including component function, voltage, estimated current, communication interface, and subsystem role.

> This Bill of Materials is written for engineering documentation and reproducibility. Current values are based on the current ARBIBOT design notes and should be verified against final datasheets and measured current values before final submission.

---

## 1. Purpose of the Bill of Materials

This document lists the main mechanical, electrical, electronic, sensor, compute, and power components used in ARBIBOT.

The goal is to make the robot easier to:

- understand,
- reproduce,
- debug,
- evaluate,
- repair,
- and improve.

The Bill of Materials also helps explain why each component exists in the system and how it connects to the rest of the robot.

---

## 2. System Overview

ARBIBOT is divided into the following hardware subsystems:

| Subsystem | Main components |
|---|---|
| Compute and vision | NVIDIA Jetson Orin Nano, IMX477 camera |
| Low-level control | STM32F411 Black Pill |
| Motor drive | Cytron MD10C, JGY-370B motor, encoder |
| Steering | MG996R servo, Pololu servo controller |
| Distance sensing | VL53L4CD sensors, VL53L8CH front matrix sensor |
| Power | 3S 18650 motor battery, 3S BMS, Waveshare UPS module |
| Communication | CP2102 USB-TTL, USB serial links, I2C bus |
| Mechanical | 3D-printed chassis, wheels, steering linkage, bumper |

---

## 3. Main Component Summary

| # | Component | Qty | Function | Voltage | Current | Interface |
|---:|---|---:|---|---|---|---|
| 1 | NVIDIA Jetson Orin Nano | 1 | AI compute, camera processing, YOLO inference, navigation decisions | 12V input through UPS/power path | Up to ~25W typical / up to ~40W high load estimate | USB, CSI camera, serial |
| 2 | IMX477 camera | 1 | Vision input for pillars, lines, and corner features | Powered from Jetson | [TODO: verify] | MIPI CSI-2 |
| 3 | STM32F411CEU6 Black Pill | 1 | Low-level control, sensor polling, motor control, encoder reading | 5V input / 3.3V logic | ~10 mA estimate at 100 MHz, verify final | USB/UART, I2C, GPIO, PWM, EXTI |
| 4 | Cytron MD10C motor driver | 1 | Drives rear DC gear motor | Motor battery voltage, nominal 11.1V / full 12.6V | Motor dependent; driver rated higher than motor current | PWM + DIR from STM32 |
| 5 | JGY-370B 12V worm gear motor with encoder | 1 | Rear-wheel drive motor | 12V nominal | 0.06-0.09A no-load, 0.2-0.3A rated load, 1.3-2.0A stall estimate | DC motor wires + quadrature encoder |
| 6 | MG996R servo | 1 | Front steering actuator | 5V | ~10mA idle, 120-170mA no-load, 500-900mA normal, 1.5-2.5A stall estimate | Servo PWM via Pololu controller |
| 7 | Pololu Micro Maestro servo controller | 1 | Dedicated servo pulse controller | 5V logic / servo power input | [TODO: verify] plus servo load | USB serial / TTL serial, servo PWM |
| 8 | VL53L4CD ToF distance sensor | 3 | Side/front distance sensing and wall following | 3.3V logic / module dependent | ~15-25mA active estimate each | I2C + XSHUT |
| 9 | VL53L8CH front matrix ToF sensor | 1 | Front barrier and corner matrix sensing | 3.3V logic / module dependent | [TODO: verify] | I2C + optional XSHUT |
| 10 | CP2102 USB to TTL module | 1 | USB serial bridge for STM32 / debugging | 5V USB, 3.3V/5V TTL depending config | [TODO: verify] | USB, UART TX/RX |
| 11 | Waveshare UPS module | 1 | Power supply for Jetson/electronics domain | 3x 18650 input, 12.6V charge, 5V/12V outputs depending module | Output up to 5V 5A; verify final use | Power + optional I2C |
| 12 | 3S 18650 motor battery pack | 1 | Main motor power source | 11.1V nominal, 12.6V full | Depends on cells and load | Power |
| 13 | 3S 20A BMS | 1 | Motor battery protection | 3S Li-ion, 12.6V full | 20A module rating | Power |
| 14 | I2C interface expansion board | 1 | Cleaner I2C and power wiring for sensors | 3.3V / 5V depending wiring | Low current | I2C bus |
| 15 | 3D-printed chassis | 1 | Mechanical structure | N/A | N/A | Mechanical |
| 16 | Front bumper / sensor mount | 1 | Holds front/side sensors and protects front | N/A | N/A | Mechanical |
| 17 | Rubber wheels | 4 | Contact with WRO field | N/A | N/A | Mechanical |
| 18 | Steering linkage / tie rod / pushrod | 1 set | Transfers servo motion to front wheels | N/A | N/A | Mechanical |
| 19 | Wires, connectors, screws, spacers | As needed | Electrical and mechanical assembly | Depends | Depends | Power, signal, mechanical |

---

## 4. Detailed Component Records

---

## 4.1 NVIDIA Jetson Orin Nano

| Field | Value |
|---|---|
| Component | NVIDIA Jetson Orin Nano |
| Quantity | 1 |
| Subsystem | Compute and vision |
| Function | Runs camera capture, YOLO inference, image processing, challenge strategy, high-level navigation, and serial command output |
| Input voltage | 12V power path through robot UPS/power system |
| Estimated power | Up to ~25W typical workload, up to ~40W high load estimate |
| Interface | MIPI CSI-2 camera, USB serial, USB peripherals, network/SSH for development |
| Connected to | Camera, STM32 serial link, servo controller if USB-connected, Jetson UPS |
| Main software | Python, OpenCV, Ultralytics YOLO, challenge scripts |
| Documentation links | `docs/03-software-architecture.md`, `docs/02-power-and-sensor-architecture.md` |

### Reason for use

The Jetson was selected because ARBIBOT uses AI-based vision. YOLO inference and camera processing require more compute power than a small microcontroller can provide.

### Risk notes

| Risk | Mitigation |
|---|---|
| High power draw | Use dedicated UPS/power path |
| Camera or YOLO process failure | Test camera before each run |
| Blocking serial communication slows vision | Use non-blocking motor command strategy |
| Thermal or power instability | Verify runtime under full load |

---

## 4.2 IMX477 Camera

| Field | Value |
|---|---|
| Component | IMX477 camera |
| Quantity | 1 |
| Subsystem | Vision |
| Function | Provides front visual input for pillar detection, line/corner detection, and navigation |
| Voltage | Powered directly from Jetson camera interface |
| Current | [TODO: verify datasheet/module value] |
| Interface | MIPI CSI-2, 22-pin 0.5mm pitch cable |
| Connected to | NVIDIA Jetson Orin Nano |
| Used by | YOLO models, OpenCV frame processing |
| Documentation links | `docs/07-calibration-procedures.md`, `docs/03-software-architecture.md` |

### Reason for use

The camera gives semantic information that distance sensors cannot provide, especially pillar color and visual corner/line information.

### Calibration notes

- Camera angle must be fixed before YOLO confidence and zone calibration.
- Lens should be cleaned before testing.
- Lighting and glare should be tested before competition.

---

## 4.3 STM32F411CEU6 Black Pill

| Field | Value |
|---|---|
| Component | STM32F411CEU6 Black Pill |
| Quantity | 1 |
| Subsystem | Low-level embedded control |
| Function | Reads sensors, manages I2C, controls motor driver, reads encoder, handles UART protocol |
| Input voltage | 5V from USB/TTL power path |
| Logic voltage | 3.3V |
| Estimated current | ~10mA estimate at 100MHz with peripherals disabled; final value should be measured |
| Interface | USB/UART, I2C, GPIO, PWM, EXTI |
| Connected to | VL53 sensors, MD10C motor driver, motor encoder, Jetson serial link |
| Firmware role | Command parser, sensor acquisition, motor control, response frames |
| Documentation links | `docs/03-software-architecture.md`, `docs/02-power-and-sensor-architecture.md` |

### Reason for use

The STM32 handles low-level real-time tasks more deterministically than Linux on the Jetson. It is responsible for sensor timing, motor PWM, encoder interrupts, and command execution.

### Important pins and signals

| Signal | Purpose | Notes |
|---|---|---|
| I2C SCL/SDA | VL53 distance sensors | Shared bus |
| XSHUT PA5 | Left VL53L4CD | Used for address initialization |
| XSHUT PA7 | Right-front VL53L4CD | Used for address initialization |
| XSHUT PB14 | Right-rear VL53L4CD | Used for address initialization |
| PWM output | MD10C speed control | Exact pin in firmware |
| DIR output | MD10C direction control | Exact pin in firmware |
| Encoder A/B inputs | Motor encoder feedback | EXTI / interrupt input |
| UART/USB | Jetson communication | DD-UART protocol |

---

## 4.4 Cytron MD10C Motor Driver

| Field | Value |
|---|---|
| Component | Cytron MD10C |
| Quantity | 1 |
| Subsystem | Motor drive |
| Function | Drives the rear DC gear motor using PWM and direction control |
| Input voltage | Motor battery path, nominal 11.1V / full 12.6V |
| Current | Must handle motor load and stall current; verify final margin |
| Interface | PWM + DIR from STM32 |
| Connected to | 3S BMS, motor, STM32 |
| Documentation links | `schemes/full-wiring-diagram.png`, `docs/02-power-and-sensor-architecture.md` |

### Reason for use

The MD10C is a robust motor driver module suitable for controlling the rear DC gear motor. It separates low-power STM32 logic from the high-current motor path.

### Risk notes

| Risk | Mitigation |
|---|---|
| Incorrect motor direction | Verify FORWARD/REVERSE before field run |
| Motor noise affects logic | Use separated power paths and careful wiring |
| Motor stall | Avoid blocked wheels and excessive load |

---

## 4.5 JGY-370B 12V Worm Gear Motor with Encoder

| Field | Value |
|---|---|
| Component | JGY-370B 12V mini worm gear motor with encoder |
| Quantity | 1 |
| Subsystem | Drivetrain |
| Function | Rear-wheel drive motor |
| Voltage | 12V nominal |
| Speed | Approximately 150 RPM model |
| Estimated current | 0.06-0.09A no-load, 0.2-0.3A rated load, 1.3-2.0A stall estimate |
| Interface | DC motor power wires + quadrature encoder |
| Connected to | MD10C motor output, STM32 encoder inputs |
| Documentation links | `docs/01-mechanical-design.md`, `docs/07-calibration-procedures.md` |

### Wiring reference

| Wire color | Function |
|---|---|
| Red | Motor DC power positive |
| White | Motor DC power negative |
| Blue | Encoder V+ |
| Black | Encoder GND |
| Yellow | Encoder signal A or B |
| Green | Encoder signal B or A |

### Reason for use

This motor provides a balance of torque, manageable speed, metal gearing, encoder feedback, and compact size.

### Calibration needed

| Calibration | Purpose |
|---|---|
| Motor direction | Confirm FORWARD and REVERSE |
| Minimum moving speed | Find lowest reliable movement speed |
| Safe competition speed | Balance lap time and control |
| Encoder ticks per wheel revolution | Support movement feedback |

---

## 4.6 MG996R Steering Servo

| Field | Value |
|---|---|
| Component | MG996R servo |
| Quantity | 1 |
| Subsystem | Steering |
| Function | Actuates front steering linkage |
| Voltage | 5V |
| Torque | 9.4 kg·cm at 4.8V stated in project notes |
| Estimated current | ~10mA idle, 120-170mA no-load, 500-900mA normal load, 1.5-2.5A stall |
| Interface | PWM servo pulse |
| Connected to | Pololu servo controller, steering linkage |
| Documentation links | `docs/01-mechanical-design.md`, `docs/07-calibration-procedures.md` |

### Reason for use

The MG996R provides enough torque for the front steering mechanism and is commonly available.

### Calibration needed

| Calibration | Purpose |
|---|---|
| Servo center | Straight front wheels |
| Left limit | Avoid linkage binding |
| Right limit | Avoid tire/bumper interference |
| Steering clamp | Prevent aggressive obstacle steering |

---

## 4.7 Pololu Micro Maestro Servo Controller

| Field | Value |
|---|---|
| Component | Pololu Micro Maestro servo controller |
| Quantity | 1 |
| Subsystem | Steering control |
| Function | Generates stable servo PWM signals |
| Voltage | 5V logic / servo power input depending wiring |
| Current | Controller current low; servo current passes through servo power path |
| Interface | USB serial or TTL serial |
| Connected to | Jetson or control computer, MG996R servo |
| Documentation links | `schemes/full-wiring-diagram.png`, `docs/03-software-architecture.md` |

### Reason for use

A dedicated servo controller reduces timing complexity and provides reliable steering pulse output.

### Risk notes

| Risk | Mitigation |
|---|---|
| USB port changes | Verify port before run |
| Servo power dip | Ensure adequate servo power |
| Wrong servo channel | Label and document final channel |

---

## 4.8 VL53L4CD Distance Sensors

| Field | Value |
|---|---|
| Component | VL53L4CD ToF distance sensor |
| Quantity | 3 |
| Subsystem | Distance sensing |
| Function | Side/front distance sensing, wall following, heading correction |
| Voltage | 3.3V logic/module dependent |
| Estimated current | ~15-25mA active per sensor |
| Interface | I2C + XSHUT |
| Connected to | STM32F411 |
| Documentation links | `docs/02-power-and-sensor-architecture.md`, `docs/07-calibration-procedures.md` |

### Placement

| Sensor | Position | Function | Address | XSHUT |
|---|---|---|---:|---|
| Left VL53L4CD | Front/left bumper area | Left/front distance sensing | `0x52` | PA5 |
| Right-front VL53L4CD | Front/right bumper area | Right wall distance | `0x54` | PA7 |
| Right-rear VL53L4CD | Middle/right side between wheels | Right heading correction | `0x56` | PB14 |

> Address map should be verified against final STM32 firmware.

### Reason for use

VL53L4CD sensors provide more precise short-range distance measurements than ultrasonic sensors for the compact WRO field geometry.

---

## 4.9 VL53L8CH / VL53L7CX-Style Front Matrix Sensor

| Field | Value |
|---|---|
| Component | VL53L8CH front matrix ToF sensor |
| Quantity | 1 |
| Subsystem | Front distance sensing |
| Function | Front matrix distance sensing for front wall, obstacles, and corner trigger support |
| Voltage | 3.3V logic/module dependent |
| Current | [TODO: verify final module value] |
| Interface | I2C |
| Connected to | STM32F411 |
| Documentation links | `docs/02-power-and-sensor-architecture.md`, `docs/07-calibration-procedures.md` |

### Reason for use

The front matrix sensor provides multiple distance zones instead of a single front value. This helps detect front barriers and avoid false corner triggers when filtered correctly.

### Calibration needed

| Parameter | Purpose |
|---|---|
| Matrix row used | Select useful cells |
| Valid distance band | Reject floor/invalid values |
| Required valid cells | Prevent one-cell false trigger |
| Trigger distance | Decide when to turn |

---

## 4.10 CP2102 USB to TTL Serial Module

| Field | Value |
|---|---|
| Component | CP2102 USB 2.0 to TTL module |
| Quantity | 1 |
| Subsystem | Communication / debugging |
| Function | USB-to-serial bridge for STM32 communication or testing |
| Voltage | USB 5V input, TTL output depending module configuration |
| Current | [TODO: verify] |
| Interface | USB, UART TX/RX |
| Connected to | Development computer, STM32, or Jetson depending test setup |
| Documentation links | `schemes/full-wiring-diagram.png`, `docs/03-software-architecture.md` |

### Reason for use

The CP2102 module provides a practical way to test UART communication and interact with STM32 serial commands during development.

---

## 4.11 Waveshare UPS Module

| Field | Value |
|---|---|
| Component | Waveshare UPS module |
| Quantity | 1 |
| Subsystem | Power |
| Function | Provides power for Jetson/electronics domain |
| Battery input | 3 x 18650 cells |
| Charging input | 12.6V / 2A charger according to project notes |
| Outputs | 5V 5A, 3.3V 300mA, and battery/12V path depending module configuration |
| Interface | Power + optional I2C monitoring |
| Connected to | Jetson Orin Nano, servo controller/power distribution depending final wiring |
| Documentation links | `docs/02-power-and-sensor-architecture.md`, `schemes/power-distribution-diagram.png` |

### Reason for use

The UPS module gives the Jetson and electronics a more stable power path separated from motor current spikes.

### Risk notes

| Risk | Mitigation |
|---|---|
| Insufficient output current | Test full load with YOLO, camera, servo, and sensors |
| Low battery | Measure voltage before run |
| Connector movement | Secure power cables |

---

## 4.12 3S 18650 Motor Battery Pack

| Field | Value |
|---|---|
| Component | 3S 18650 lithium-ion battery pack |
| Quantity | 1 |
| Subsystem | Motor power |
| Function | Supplies motor power through BMS and motor driver |
| Nominal voltage | 11.1V |
| Full voltage | 12.6V |
| Capacity | 1200mAh cells mentioned in project notes; final pack capacity should be verified |
| Current | Depends on cell capability and motor load |
| Interface | Power |
| Connected to | 3S 20A BMS |
| Documentation links | `docs/02-power-and-sensor-architecture.md` |

### Reason for use

A 3S pack provides voltage suitable for the 12V gear motor and motor driver path.

---

## 4.13 3S 20A BMS

| Field | Value |
|---|---|
| Component | 3S 20A BMS |
| Quantity | 1 |
| Subsystem | Motor battery protection |
| Function | Protects the 3S Li-ion motor battery pack |
| Voltage | 3S Li-ion, 12.6V full |
| Current rating | 20A module rating |
| Interface | Power |
| Connected to | Motor battery, Cytron MD10C |
| Documentation links | `schemes/power-distribution-diagram.png` |

### Reason for use

The BMS protects the motor battery pack from unsafe charge/discharge conditions and provides a controlled motor power path.

---

## 4.14 I2C Interface Expansion Board

| Field | Value |
|---|---|
| Component | I2C interface expansion board |
| Quantity | 1 |
| Subsystem | Sensor wiring |
| Function | Organizes I2C and power wiring for sensors |
| Voltage | 3.3V / 5V depending final wiring |
| Current | Low current; sensor dependent |
| Interface | I2C bus |
| Connected to | STM32 and VL53 sensors |
| Documentation links | `docs/02-power-and-sensor-architecture.md`, `schemes/full-wiring-diagram.png` |

### Reason for use

The I2C expansion board helps keep wiring cleaner and makes sensor connections easier to manage.

> Note: This board is used as a wiring/interface expansion board, not as an intelligent I2C multiplexer unless final hardware proves otherwise.

---

## 4.15 3D-Printed Chassis

| Field | Value |
|---|---|
| Component | Custom 3D-printed chassis |
| Quantity | 1 |
| Subsystem | Mechanical |
| Function | Main robot structure |
| Voltage | N/A |
| Current | N/A |
| Interface | Mechanical mounting |
| Connected to | All mechanical and electronic subsystems |
| Documentation links | `docs/01-mechanical-design.md`, `v-photos/` |

### Known dimensions

| Measurement | Value |
|---|---:|
| Length | 25 cm |
| Width | 15.6 cm |
| Height | 18 cm |
| Weight | 1.35 kg |
| Wheelbase | 13.9 cm |
| Track width | [TODO: measure final] |

### Reason for use

3D printing allowed custom mounting for sensors, electronics, steering, bumper geometry, and rapid mechanical iteration.

---

## 4.16 Front Bumper and Sensor Mount

| Field | Value |
|---|---|
| Component | Front bumper / sensor mount |
| Quantity | 1 |
| Subsystem | Mechanical / sensors |
| Function | Holds front and side sensors and protects front area |
| Voltage | N/A |
| Current | N/A |
| Interface | Mechanical |
| Connected to | Chassis, VL53 sensors |
| Documentation links | `v-photos/front-bumper.jpeg`, `schemes/sensor-placement-diagram.png` |

### Reason for use

The front bumper gives a defined location for the front distance sensor and the left/right sensors. It also protects the front of the robot.

### Iteration notes

Earlier bumper versions caused tire rubbing and poor sensor placement. The bumper was redesigned to improve clearance and sensor alignment.

---

## 4.17 Wheels

| Field | Value |
|---|---|
| Component | Rubber wheels |
| Quantity | 4 |
| Subsystem | Mechanical drivetrain |
| Function | Provide contact with WRO field and support car-like movement |
| Voltage | N/A |
| Current | N/A |
| Interface | Mechanical |
| Connected to | Rear drive axle, front steering hubs |
| Documentation links | `docs/01-mechanical-design.md` |

### Reason for use

Wider rubber wheels were selected after thinner wheels slipped on the track. Better traction improved repeatability.

---

## 4.18 Steering Linkage

| Field | Value |
|---|---|
| Component | Pushrod/tie-rod steering linkage |
| Quantity | 1 set |
| Subsystem | Mechanical steering |
| Function | Transfers servo motion to front wheels |
| Voltage | N/A |
| Current | N/A |
| Interface | Mechanical |
| Connected to | MG996R servo, front steering hubs |
| Documentation links | `docs/01-mechanical-design.md`, `v-photos/` |

### Reason for use

The linkage creates a simple car-like steering mechanism and allows mechanical adjustment of wheel alignment.

---

## 4.19 Wires, Connectors, Screws, Spacers, Cable Management

| Field | Value |
|---|---|
| Component | Wiring and hardware accessories |
| Quantity | As needed |
| Subsystem | Electrical and mechanical integration |
| Function | Power, signal, mounting, strain relief |
| Voltage | Depends on circuit |
| Current | Depends on circuit |
| Interface | Power, UART, I2C, GPIO, mechanical |
| Connected to | All subsystems |

### Notes

Important wiring practices:

- keep motor wires away from sensitive I2C/UART lines where possible,
- secure wires so they cannot touch wheels,
- keep grounds connected where needed for signal reference,
- label sensor and motor wires,
- use strain relief for moving sections,
- verify connectors after crashes.

---

## 5. Power Domain Summary

ARBIBOT uses separated power domains to reduce instability.

### 5.1 Motor Power Domain

```text
3S 18650 motor battery -> 3S 20A BMS -> Cytron MD10C -> JGY-370B motor
```

| Component | Voltage | Current notes |
|---|---:|---|
| 3S battery | 11.1V nominal / 12.6V full | Depends on cell capacity and discharge rating |
| 3S BMS | 12.6V full | 20A module rating |
| Cytron MD10C | Motor battery voltage | Must handle motor load |
| JGY-370B motor | 12V nominal | 0.06-0.09A no-load, 0.2-0.3A normal, 1.3-2.0A stall estimate |

### 5.2 Electronics / Compute Power Domain

```text
Waveshare UPS -> Jetson Orin Nano / camera / servo controller / electronics
```

| Component | Voltage | Current notes |
|---|---:|---|
| Waveshare UPS | 3x 18650 input / regulated outputs | Verify final current margin |
| Jetson Orin Nano | 12V path in current design | Up to ~25W typical / ~40W high load estimate |
| Camera | From Jetson CSI | Low current, verify module |
| Pololu servo controller | 5V | Low controller current plus servo power |
| MG996R servo | 5V | Peak current can be high |
| STM32 | 5V USB / 3.3V logic | Low current |
| VL53 sensors | 3.3V/module dependent | ~15-25mA each for VL53L4CD estimate |

---

## 6. Communication and Interface Summary

| Link | From | To | Interface | Purpose |
|---|---|---|---|---|
| Camera link | IMX477 camera | Jetson | MIPI CSI-2 | Vision frames |
| Main command link | Jetson | STM32 | USB serial / UART | Motor/sensor commands |
| Servo link | Jetson or controller host | Pololu Micro Maestro | USB serial / TTL serial | Steering commands |
| Sensor bus | STM32 | VL53 sensors | I2C | Distance readings |
| XSHUT lines | STM32 | VL53L4CD sensors | GPIO | Address initialization |
| Motor control | STM32 | MD10C | PWM + DIR | Speed and direction |
| Encoder feedback | Motor encoder | STM32 | Quadrature A/B | Wheel feedback |
| Power path | Battery/BMS/UPS | Subsystems | Power wiring | Energy distribution |

---

## 7. Estimated Current Budget

These values should be treated as estimates until measured on the final robot.

| Subsystem | Quantity | Typical current / power | Peak current / power | Notes |
|---|---:|---:|---:|---|
| Jetson Orin Nano | 1 | Up to ~25W estimate | Up to ~40W estimate | Depends on power mode and YOLO load |
| IMX477 camera | 1 | [TODO] | [TODO] | Powered from Jetson |
| STM32F411 | 1 | ~10mA estimate | [TODO] | Verify with peripherals active |
| MG996R servo | 1 | 500-900mA normal load | 1.5-2.5A stall estimate | Major peak-current risk |
| JGY-370B motor | 1 | 0.2-0.3A rated load estimate | 1.3-2.0A stall estimate | Motor power domain |
| VL53L4CD | 3 | 15-25mA each estimate | [TODO] | Active ranging |
| VL53L8CH | 1 | [TODO] | [TODO] | Verify final module |
| Pololu controller | 1 | [TODO] | [TODO] | Excluding servo load |
| CP2102 | 1 | [TODO] | [TODO] | USB serial module |
| I2C expansion board | 1 | negligible | negligible | Passive/low-current wiring board |

---

## 8. Component Selection Reasoning

| Component choice | Reason |
|---|---|
| Jetson Orin Nano | Needed for real-time camera processing and YOLO inference |
| STM32F411 | Reliable low-level control, I2C, PWM, encoder interrupts |
| VL53 ToF sensors | More precise than ultrasonic sensors for compact WRO geometry |
| VL53L8CH front matrix | Provides multi-zone front distance information |
| JGY-370B worm gear motor | Good torque, manageable speed, encoder feedback |
| MG996R servo | Enough torque for steering linkage |
| Cytron MD10C | Robust motor driver for DC gear motor |
| Pololu controller | Stable servo pulse generation |
| Separate power domains | Reduces motor noise and voltage-drop problems |
| 3D-printed chassis | Allows rapid iteration and custom mounts |

---

## 9. Alternatives Rejected

| Component / approach | Reason rejected |
|---|---|
| Ultrasonic sensors | Less precise and less suitable for near-wall WRO geometry |
| Motor without encoder | No reliable feedback for movement validation |
| Thin wheels | Slipped on the field |
| Shared power path for everything | Higher risk of Jetson brownout and sensor instability |
| Camera-only navigation | Too dependent on lighting and visual features |
| Single side distance sensor | Could not estimate robot heading relative to wall |
| Direct Jetson low-level motor control | Less deterministic than STM32 for embedded control |
| L298N-style motor driver | Lower efficiency and not preferred for motor current path |

---

## 10. Spare Parts Recommended

Recommended spare parts for testing and competition:

| Spare part | Reason |
|---|---|
| Extra 18650 cells | Battery replacement and runtime reliability |
| Extra VL53L4CD sensor | Sensor damage or address/debug replacement |
| Extra servo horn | Steering linkage repairs |
| Extra MG996R servo | Servo gear or motor failure |
| Extra CP2102 module | Serial debugging backup |
| Extra wires/connectors | Field repairs |
| Extra screws/nuts/spacers | Mechanical repairs |
| Extra wheels | Traction or damage replacement |
| Extra 3D-printed bumper | Crash replacement |
| Extra camera cable | CSI cable damage replacement |

---

## 11. Final BOM Table for Competition

Complete this table before final submission.

| Item | Qty final | Verified voltage | Measured current | Verified interface | Final status |
|---|---:|---:|---:|---|---|
| Jetson Orin Nano | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| IMX477 camera | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| STM32F411 | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| Cytron MD10C | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| JGY-370B motor | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| MG996R servo | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| Pololu servo controller | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| VL53L4CD | 3 | [TODO] | [TODO] | [TODO] | [TODO] |
| VL53L8CH | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| CP2102 | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| Waveshare UPS | 1 | [TODO] | [TODO] | [TODO] | [TODO] |
| 3S battery pack | 1 | [TODO] | [TODO] | Power | [TODO] |
| 3S BMS | 1 | [TODO] | [TODO] | Power | [TODO] |
| I2C expansion board | 1 | [TODO] | [TODO] | I2C | [TODO] |

---

## 12. Documentation Links

Related documents:

| Document | Relationship |
|---|---|
| `docs/01-mechanical-design.md` | Mechanical components, chassis, steering, motor mounting |
| `docs/02-power-and-sensor-architecture.md` | Power paths, sensors, wiring, voltage domains |
| `docs/03-software-architecture.md` | Interfaces, command protocol, Jetson/STM32 split |
| `docs/05-systems-thinking-decisions.md` | Why components were selected |
| `docs/07-calibration-procedures.md` | Calibration of camera, sensors, motor, servo |
| `docs/10-risk-register.md` | Risks related to components and subsystems |
| `schemes/full-wiring-diagram.png` | Wiring reference |
| `schemes/power-distribution-diagram.png` | Power reference |
| `schemes/sensor-placement-diagram.png` | Sensor placement reference |
| `v-photos/` | Real robot photos |

---

## 13. Change Log

| Date | Change | Notes |
|---|---|---|
| [TODO] | Initial BOM created | Based on current ARBIBOT hardware |
| [TODO] | Add measured currents | Needed after multimeter/current testing |
| [TODO] | Add final vendor links or part numbers | Optional but useful for reproducibility |
| [TODO] | Verify final voltage rails | Needed before final submission |

---

## 14. Conclusion

The ARBIBOT Bill of Materials shows that the robot is built around a two-level control system: Jetson for AI/vision/navigation and STM32 for low-level real-time control. The design combines a 12V rear drive motor, servo steering, VL53 time-of-flight sensors, a front matrix distance sensor, a Jetson camera, separated power domains, and a 3D-printed mechanical chassis.

The most important final work for this BOM is to replace estimates with measured values:

- measured current draw,
- verified voltage rails,
- final sensor address map,
- final connector/interface details,
- final battery runtime,
- and final part quantities.

Once those values are filled, this document will serve as a complete hardware reference for reproducing ARBIBOT.
