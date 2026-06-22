# 02 - Power and Sensor Architecture

**Project:** MetallicMadness / ARBIBOT  
**Category:** WRO 2026 Future Engineers - Self-Driving Cars  
**Document purpose:** Explain the electrical power architecture, battery systems, regulators, current draw, wiring organization, sensor placement, I2C design, and calibration process.

> This document is part of the ARBIBOT technical documentation package. It focuses on the power and sensor subsystem. Mechanical design and software architecture are documented separately.

---

## 1. Design Goals

The power and sensor architecture of ARBIBOT was designed around one main principle: **separate noisy high-current loads from sensitive logic and sensing electronics**.

ARBIBOT uses two main electronic domains:

1. **Low-level control domain**  
   This includes the STM32F411, distance sensors, motor driver interface, encoder feedback, and sensor acquisition.

2. **High-level AI and vision domain**  
   This includes the NVIDIA Jetson Orin Nano, CSI camera, YOLO inference, navigation decisions, and steering servo controller.

The vehicle uses independent power paths for the motor system and the Jetson system. This reduces the probability that motor current spikes will reset the Jetson, disturb sensor readings, or corrupt UART communication. Motors are electrically noisy little gremlins; the design keeps them away from the parts that need clean signals.

---

## 2. High-Level Electrical Architecture

ARBIBOT uses a distributed architecture:

```text
NVIDIA Jetson Orin Nano
    |
    | USB / UART serial command link
    v
STM32F411 Black Pill
    |
    | PWM + direction
    v
Cytron MD10C Motor Driver
    |
    v
12V DC Gear Motor with Encoder

STM32F411
    |
    | I2C bus + XSHUT address control
    v
VL53L4CD and VL53L8CH Distance Sensors

Jetson Orin Nano
    |
    | CSI camera
    v
IMX477 Camera

Jetson / Pololu
    |
    | Servo signal
    v
MG996R Steering Servo
```

![STM32 wiring diagram](../engineering-journal/images/stm32_wiring_diagram.jpg)

The STM32 handles real-time hardware interaction, while the Jetson handles the computationally expensive tasks such as image capture, YOLO inference, object classification, and navigation decision logic.

---

## 3. Power Architecture Overview

ARBIBOT uses **two main battery/power paths**:

| Power path | Main load | Purpose |
|---|---|---|
| Motor power path | Cytron MD10C + DC drive motor | High-current propulsion |
| Jetson/electronics power path | Jetson Orin Nano, camera, Pololu servo controller | AI, vision, steering control, high-level processing |
| STM32 logic power | STM32F411 via USB/TTL 5V | Low-level command and sensor controller |

The power paths are separated to improve reliability. The DC motor and servo can draw high peak currents, especially during acceleration, steering, or stall conditions. These current spikes can cause voltage drops. If all electronics were powered from one weak shared supply, the Jetson or STM32 could reset during motion, which would be fatal during a WRO run.

---

## 4. Motor Power Path

The drive motor is powered through a **3S 20A BMS system** and a **Cytron MD10C motor driver**.

```text
3S Li-ion battery pack
    |
    v
BMS 3S 20A
    |
    v
Cytron MD10C motor driver
    |
    v
JGY-370B 12V DC gear motor with encoder
```

![BMS 3S 20A](../engineering-journal/images/bms3s_20a.jpg)

### 4.1 Motor Battery

| Parameter | Value |
|---|---|
| Cell type | 18650 lithium-ion |
| Cell nominal voltage | 3.7 V |
| Cell capacity | 1200 mAh |
| Cell chemistry | Li-ion |
| Series configuration | 3S |
| Pack nominal voltage | 11.1 V |
| Pack fully charged voltage | 12.6 V |
| Approximate pack capacity | 1200 mAh, if cells are connected only in series |
| Approximate energy | 11.1 V × 1.2 Ah = 13.32 Wh |
| Protection | BMS 3S 20A |

The nominal motor pack voltage is close to the 12V requirement of the selected DC gear motor. The BMS protects the lithium-ion cells and provides the motor power path to the motor driver.

### 4.2 Cytron MD10C Motor Driver

The robot uses a **Cytron MD10C DC motor driver** to control the brushed DC gear motor.

![Cytron MD10C motor driver](../engineering-journal/images/cytron_md10c.jpg)

| MD10C parameter | Value |
|---|---|
| Motor voltage input | 5 V to 30 V DC |
| Control logic | 3.3 V to 5.0 V compatible |
| Control mode | PWM + direction |
| PWM frequency | Up to 20 kHz |
| Rated continuous current | 13 A |
| Peak current | 30 A |
| STM32 PWM pin | PA6 / TIM3_CH1 |
| STM32 direction pin | PA1 |
| Common ground | Required between STM32 and MD10C |

The STM32 sends a PWM signal and direction signal to the MD10C. The MD10C then switches the motor power from the battery/BMS path.

### 4.3 Motor Current Estimate

The JGY-370B motor current depends heavily on load. The expected values are:

| Motor condition | Estimated current |
|---|---:|
| No-load | 0.06 A to 0.09 A |
| Normal driving load | 0.2 A to 0.3 A |
| Heavy acceleration / high friction | [TODO: measure] |
| Stall / peak condition | 1.3 A to 2.0 A |

The MD10C has far more current capacity than the motor normally requires. This margin is useful because motor start-up and stall current are always higher than steady-state running current.

---

## 5. Jetson Power Path

The Jetson Orin Nano is powered independently from the motor system using a **Waveshare UPS module**.

![Waveshare UPS module](../engineering-journal/images/waveshare_ups.jpg)

| UPS parameter | Value |
|---|---|
| Battery support | 3 × 18650 lithium batteries |
| Output voltage | Batteries series voltage; 5 V 5 A; 3.3 V 300 mA |
| Control interface | I2C |
| Charger | 12.6 V 2 A |
| Dimensions | 60 × 93 mm |
| Mounting hole size | 3.0 mm |
| Jetson supply voltage used | 12 V / battery series voltage path |

The Jetson system was separated from the motor battery because the Jetson is sensitive to voltage drops. When YOLO inference is running, the Jetson can draw significant power. Sharing the same supply with the drive motor could cause resets or unstable behavior.

### 5.1 Jetson Current and Power Estimate

The Jetson Orin Nano power consumption depends on workload:

| Jetson condition | Estimated power |
|---|---:|
| Idle / low load | [TODO: measure] |
| Camera capture only | [TODO: measure] |
| YOLO inference running | Up to approximately 25 W for standard workloads |
| Higher performance load | Up to approximately 40 W |
| Competition workload estimate | [TODO: measure on robot] |

The Jetson power budget should be measured during a real run because camera capture, model inference, USB serial communication, and navigation logic can change the actual current draw.

### 5.2 Camera Power

The camera is powered directly from the Jetson through the CSI camera connector.

| Camera parameter | Value |
|---|---|
| Camera model | Mini 12.3MP HQ Camera compatible with NVIDIA Jetson |
| Image sensor | Sony IMX477 |
| Sensor size | 1/2.3 inch |
| Lens mount | M12 |
| Interface | MIPI CSI-2 |
| Connector | 22-pin, 0.5 mm pitch |
| Power source | Jetson CSI camera interface |

The camera is part of the Jetson domain because it is used by OpenCV and the YOLO detection pipeline.

---

## 6. STM32 Power Path

The STM32F411 Black Pill is powered through the USB/TTL 5V interface during operation.

![STM32F411 Black Pill](../engineering-journal/images/stm32f411_blackpill.jpg)

| STM32 parameter | Value |
|---|---|
| Controller | STM32F411 Black Pill |
| Main role | Sensor acquisition, motor command execution, encoder reading |
| Power source | USB/TTL 5V |
| Logic level | 3.3 V |
| Estimated current at 100 MHz | Approximately 10 mA with peripherals disabled |
| Main interfaces | UART, I2C, PWM, GPIO, encoder inputs |

The STM32 is responsible for low-level control. It receives commands from the Jetson and executes actions such as reading sensors, setting motor speed, moving by degrees, stopping the motor, and returning sensor data.

---

## 7. Servo Power and Steering Control

The robot uses an **MG996R servo** for steering. The steering signal is managed by a Pololu servo controller.

![Pololu servo controller](../engineering-journal/images/pololu_servo_controller.jpg)

| Steering power/control parameter | Value |
|---|---|
| Steering servo | MG996R |
| Servo voltage | 5 V |
| Rated torque | 9.4 kg·cm at 4.8 V |
| Control board | Pololu servo controller |
| Servo controller power | Waveshare UPS |
| Steering mechanism | Servo-actuated pushrod and tie-rod linkage |

### 7.1 Servo Current Estimate

| Servo condition | Estimated current |
|---|---:|
| Idle | ~10 mA |
| No-load movement | 120 mA to 170 mA |
| Normal load | 500 mA to 900 mA |
| Stall / peak | 1.5 A to 2.5 A |

The servo can draw high peak current when the front wheels are under load or when the steering linkage is blocked. For this reason, the steering mechanism must be mechanically smooth. Mechanical friction becomes electrical current. That is physics being annoying but honest.

---

## 8. Power Budget

The following table summarizes the estimated current and power consumption. Some values are estimates and must be replaced with measured values before final submission.

### 8.1 Estimated Power Budget

| Subsystem | Voltage | Typical current | Peak current | Typical power | Peak power | Notes |
|---|---:|---:|---:|---:|---:|---|
| Jetson Orin Nano | 12 V | ~2.1 A | ~3.3 A | ~25 W | ~40 W | AI/vision workload estimate |
| STM32F411 | 5 V input / 3.3 V logic | ~10 mA | [TODO] | ~0.05 W | [TODO] | MCU only estimate |
| MG996R steering servo | 5 V | 0.5 A to 0.9 A | 1.5 A to 2.5 A | 2.5 W to 4.5 W | 7.5 W to 12.5 W | Steering load varies |
| JGY-370B drive motor | 12 V | 0.2 A to 0.3 A | 1.3 A to 2.0 A | 2.4 W to 3.6 W | 15.6 W to 24 W | Through MD10C |
| VL53L4CD ×3 | 3.3 V | 45 mA to 75 mA total | [TODO] | 0.15 W to 0.25 W | [TODO] | 15-25 mA each |
| VL53L8CH ×1 | 3.3 V | [TODO] | [TODO] | [TODO] | [TODO] | Needs measured/spec value |
| IMU | 3.3 V | [TODO] | [TODO] | [TODO] | [TODO] | Sensor polling |
| Camera IMX477 | Jetson CSI | [TODO] | [TODO] | [TODO] | [TODO] | Powered by Jetson |
| CP2102 USB-TTL | 5 V USB | [TODO] | [TODO] | [TODO] | [TODO] | Serial bridge |
| Total electronics excluding drive motor | Mixed | [TODO] | [TODO] | [TODO] | [TODO] | To be measured |
| Total full-system peak estimate | Mixed | — | — | — | ~76 W worst-case estimate | Jetson peak + servo stall + motor stall, unlikely continuous |

The **worst-case peak estimate** combines Jetson high-performance load, servo stall, and motor stall. This is not expected to occur continuously, but it is useful for checking power-path margin.

### 8.2 Runtime Estimate

The expected runtime is approximately **20 to 30 minutes**, while each WRO challenge round is **3 minutes**. This gives enough margin for one official attempt, but practice sessions require recharging or battery replacement planning.

| Runtime parameter | Value |
|---|---:|
| Expected runtime | 20 to 30 minutes |
| WRO round duration | 3 minutes |
| Runtime margin for one run | Good |
| Recommended practice strategy | Charge between sessions and monitor voltage |

### 8.3 Power Measurement Plan

Before final submission, the team should measure:

| Test case | Measurement needed |
|---|---|
| Robot idle, powered on | Baseline current |
| Jetson booted, no model | Jetson idle consumption |
| Camera only | Vision capture consumption |
| YOLO inference active | AI workload consumption |
| Servo sweeping left/right | Steering current |
| Motor forward continuous | Drive current |
| Motor acceleration from stop | Peak motor current |
| All sensors ranging | Sensor current |
| Full lap test | Real competition current |
| Battery voltage before/after 3 minutes | Voltage drop and remaining margin |

These measurements will make the power budget stronger and support the WRO documentation criterion for power reasoning.

---

## 9. Grounding and Noise Considerations

All control signals require a common reference. The STM32, MD10C, encoder, sensors, and serial interface must share the proper ground reference for communication and control signals.

Important grounding rules:

1. The STM32 GND must connect to the MD10C GND.
2. The motor battery negative and logic ground must be referenced correctly where required by the driver/control signal path.
3. Encoder GND must connect to STM32 GND.
4. Sensor GND must connect to STM32 GND.
5. UART GND must connect between Jetson/USB-TTL and STM32.
6. High-current motor wires should be routed away from I2C and UART signal wires when possible.

Motor current can introduce electrical noise. This can affect:

- I2C distance sensor readings,
- UART communication,
- encoder pulse counting,
- and servo stability.

For this reason, ARBIBOT separates motor power from AI/electronics power and keeps the STM32 as the real-time hardware controller.

---

## 10. Communication and Wiring Overview

### 10.1 Jetson to STM32 Communication

The Jetson communicates with the STM32 over a custom serial protocol through USB/TTL.

![CP2102 USB-TTL adapter](../engineering-journal/images/cp2102_usb_ttl.jpg)

| Signal | Source | Destination |
|---|---|---|
| TXD | CP2102 / Jetson serial side | STM32 PA10 / USART1_RX |
| RXD | CP2102 / Jetson serial side | STM32 PA9 / USART1_TX |
| GND | CP2102 | STM32 GND |
| 5V | CP2102 | STM32 5V |

The STM32 receives binary command frames from the Jetson and returns response frames. The DD-UART protocol uses start and end bytes, length, command ID, payload, and XOR checksum.

### 10.2 STM32 to Motor Driver

| STM32 pin | MD10C input | Function |
|---|---|---|
| PA6 / TIM3_CH1 | PWM | Motor speed control |
| PA1 | DIR | Motor direction |
| GND | GND | Common ground |

### 10.3 STM32 to Encoder

| Encoder signal | STM32 pin | Function |
|---|---|---|
| Phase A | PB12 | Encoder pulse channel A |
| Phase B | PB13 | Encoder pulse channel B |
| VCC | 3.3 V | Encoder logic power |
| GND | GND | Encoder ground |

The encoder provides wheel odometry and motor RPM feedback.

---

## 11. Sensor Architecture Overview

ARBIBOT uses time-of-flight distance sensors for wall following, obstacle distance estimation, parking support, and side correction.

| Sensor type | Quantity | Main purpose |
|---|---:|---|
| VL53L4CD | 3 | Left/right side distance, wall following, side correction |
| VL53L8CH | 1 | Front distance matrix, obstacle/front-wall detection |
| IMU | 1 | Gyroscope/accelerometer data |
| Camera IMX477 | 1 | YOLO pillar detection and corner/line detection |

The distance sensors are connected to the STM32 because they require real-time polling and stable low-level communication. The camera is connected to the Jetson because image processing and YOLO inference require more computing power.

---

## 12. VL53L4CD Sensor Placement

The robot uses **three VL53L4CD sensors**.

![VL53L4CD sensor](../engineering-journal/images/vl53l4cd.jpg)

| Sensor | Placement | I2C address | XSHUT pin | Main use |
|---|---|---:|---|---|
| Left VL53L4CD | Front bumper, left side | 0x52 | PA5 | Left distance / obstacle and parking support |
| Right-front VL53L4CD | Front bumper, right side | 0x54 | PA7 | Right front wall distance |
| Right-rear VL53L4CD | Middle of car, between both wheels | 0x56 | PB14 | Right rear wall distance / heading correction |

The right-front and right-rear sensors are especially important for lane correction. By comparing their readings, the robot can estimate whether it is parallel to the wall.

### 12.1 Two-Right-Sensor Heading Correction

The two right-side sensors allow the robot to estimate heading relative to the wall:

```text
heading_error = right_front_distance - right_rear_distance
```

Interpretation:

| Condition | Meaning | Correction |
|---|---|---|
| Right-front ≈ right-rear | Robot is approximately parallel to the right wall | Maintain course |
| Right-front > right-rear | Front is farther from the wall than rear | Adjust steering toward wall |
| Right-front < right-rear | Front is closer to the wall than rear | Adjust steering away from wall |

This is better than using only one side sensor because a single distance value cannot tell whether the robot is angled relative to the wall. One sensor sees distance; two sensors reveal attitude. Tiny geometry upgrade, big control improvement.

---

## 13. VL53L8CH Front Matrix Sensor

The robot uses one **ST VL53L8CH** front sensor.

![VL53L8CH sensor](../engineering-journal/images/vl53l8ch.jpg)

| Parameter | Value |
|---|---|
| Sensor | ST VL53L8CH |
| Placement | Front bumper, center/front |
| Interface | I2C |
| Main use | Front obstacle and wall distance matrix |
| Software use | Front barrier detection and corner trigger support |
| I2C address | 0x52 listed in current notes; verify against final firmware/address map |

The VL53L8CH provides a distance matrix instead of a single point distance. In the software, the front matrix is reduced to a usable front-barrier distance. The front matrix is especially useful when detecting the approach to a wall or corner.

### 13.1 Matrix Sensor Lessons Learned

During testing, the front matrix initially caused turn timing issues. Some matrix cells detected the floor or outer-wall corner instead of the intended front barrier. This produced false or unstable distance readings.

The fix was to:

1. exclude floor-band cells,
2. use the correct matrix row for front barrier detection,
3. require at least two in-band cells before triggering a turn,
4. and avoid acting on a single isolated cell.

This improved corner-trigger reliability.

---

## 14. I2C Expansion Board

The robot uses an I2C expansion board to distribute the sensor bus and power wiring cleanly.

![I2C expansion board](../engineering-journal/images/i2c_expansion_board.jpg)

| Purpose | Explanation |
|---|---|
| Shared I2C bus | Allows multiple sensors to connect to the same SCL/SDA lines |
| Cleaner wiring | Reduces loose point-to-point wiring |
| Shared power distribution | Provides organized VCC and GND connections |
| Easier debugging | Sensors can be connected/disconnected more cleanly |

The I2C expansion board is not used as an intelligent multiplexer. It is mainly used as a wiring and distribution board. The individual VL53L4CD sensors require XSHUT control so the STM32 can assign or manage unique addresses.

---

## 15. XSHUT Address Control

The VL53L4CD sensors use XSHUT pins so the STM32 can control sensor startup and manage different I2C addresses.

| Sensor | XSHUT pin |
|---|---|
| Left VL53L4CD | PA5 |
| Right-front VL53L4CD | PA7 |
| Right-rear VL53L4CD | PB14 |

This is necessary because identical sensors often boot with the same default I2C address. By controlling their XSHUT pins, the STM32 can initialize sensors in a controlled order and avoid bus address conflicts.

---

## 16. Sensor Calibration

The distance sensors were calibrated using fixed reference distances.

Calibration distances:

```text
120 cm, 110 cm, 100 cm, 90 cm, 80 cm, 70 cm,
60 cm, 50 cm, 40 cm, 30 cm, 20 cm, 10 cm
```

A Python test program was used to read all sensors each time the car was moved to a known distance. This allowed the team to compare measured distance against real distance and identify offset, instability, and invalid readings.

### 16.1 Calibration Table Template

| Real distance | Left VL53L4CD | Right-front VL53L4CD | Right-rear VL53L4CD | Front VL53L8CH | Notes |
|---:|---:|---:|---:|---:|---|
| 120 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 110 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 100 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 90 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 80 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 70 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 60 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 50 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 40 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 30 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 20 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |
| 10 cm | [TODO] | [TODO] | [TODO] | [TODO] |  |

### 16.2 Calibration Goals

The calibration process is used to determine:

1. whether each sensor reads consistently,
2. whether any sensor has a fixed offset,
3. whether readings become unstable at close range,
4. whether sensor height and angle are correct,
5. whether the front matrix sees the wall or accidentally sees the floor,
6. and whether filtering is needed in software.

---

## 17. Sensor Use by Driving Function

| Function | Sensors used | Reason |
|---|---|---|
| Wall following | Right-front and right-rear VL53L4CD | Maintain distance and heading relative to right wall |
| Side correction | Right-front and right-rear VL53L4CD | Correct angle using RF-RR difference |
| Obstacle distance | VL53L4CD and VL53L8CH | Detect nearby objects and front barriers |
| Parking support | Side/front ToF sensors | Estimate position relative to wall/parking area |
| Corner detection | Camera + VL53L8CH front matrix | Front matrix helps detect front wall/corner approach |
| Pillar color classification | Jetson camera + YOLO | Detect red and green traffic signs |

The current notes indicate that side sensors are not the primary corner detection system. Corner detection is mainly handled by camera and front matrix logic.

---

## 18. Failure Modes and Mitigation

| Failure mode | Cause | Mitigation |
|---|---|---|
| Jetson reset during driving | Motor current spike or shared weak supply | Separate Jetson UPS from motor BMS path |
| STM32 reset | Unstable 5V supply or wiring issue | Use stable USB/TTL 5V and verify ground |
| UART errors | Shared bus timing, dropped frames, noise | Use checksum and repeated critical commands |
| I2C address conflict | Multiple identical VL53L4CD sensors | Use XSHUT pins for controlled initialization |
| Invalid distance readings | Sensor angle/height or reflective surface | Calibrate at fixed distances and adjust mounts |
| False front-wall detection | Matrix cell sees floor or corner edge | Exclude bad cells and require multiple valid cells |
| Steering servo voltage drop | Servo peak current | Separate power planning and avoid mechanical binding |
| Motor noise affecting sensors | High-current switching | Separate power paths, common ground, careful routing |
| Sensor thread starvation | Jetson YOLO load affects serial reading | Keep STM32 responsible for sensor polling and avoid blocking motor waits |

---

## 19. Wiring Documentation Checklist

The following wiring diagrams should be included in the repository:

| Diagram | Recommended path |
|---|---|
| Full wiring diagram | `schemes/full-wiring-diagram.pdf` |
| Power distribution diagram | `schemes/power-distribution-diagram.png` |
| Sensor placement diagram | `schemes/sensor-placement-diagram.png` |
| I2C bus diagram | `schemes/i2c-bus-diagram.png` |
| UART protocol wiring | `schemes/jetson-stm32-uart-wiring.png` |
| Motor driver wiring | `schemes/md10c-motor-driver-wiring.png` |
| Encoder wiring | `schemes/motor-encoder-wiring.png` |
| Servo power/control wiring | `schemes/servo-controller-wiring.png` |

The current technical documentation already includes several supporting images. Before final WRO submission, the team should also add real photos of the final wiring installed on the robot.

---

## 20. Bill of Materials - Power and Sensors

| Component | Quantity | Function |
|---|---:|---|
| NVIDIA Jetson Orin Nano | 1 | High-level AI/vision controller |
| STM32F411 Black Pill | 1 | Low-level motor/sensor controller |
| Cytron MD10C | 1 | DC motor driver |
| JGY-370B 12V gear motor with encoder | 1 | Rear-wheel drive motor |
| MG996R servo | 1 | Front steering actuator |
| Pololu servo controller | 1 | Servo signal/control interface |
| VL53L4CD | 3 | Side/front time-of-flight sensors |
| VL53L8CH | 1 | Front matrix time-of-flight sensor |
| IMX477 camera | 1 | Image capture for YOLO detection |
| I2C expansion board | 1 | Sensor bus/power distribution |
| CP2102 USB-TTL | 1 | Serial communication / STM32 power interface |
| BMS 3S 20A | 1 | Motor battery protection |
| Waveshare UPS module | 1 | Jetson power system |
| 18650 Li-ion cells | 3 for motor pack + 3 for UPS | Battery power |
| Wires/connectors | As required | Power and signal routing |

---

## 21. Pending Items Before Final Submission

The following information should be added when measured or confirmed:

| Pending item | Why it matters |
|---|---|
| Exact BMS model | Improves reproducibility |
| Exact Waveshare UPS model | Improves reproducibility |
| Actual Jetson current during YOLO inference | Validates power budget |
| Actual motor current during acceleration | Validates motor/BMS/driver margin |
| Actual servo current while steering on track | Confirms UPS/regulator capacity |
| VL53L8CH current draw | Completes sensor budget |
| Camera current draw | Completes Jetson subsystem budget |
| Final I2C address map from firmware | Prevents documentation mismatch |
| Measured runtime under competition workload | Validates battery sizing |
| Real wiring photos | Helps judges verify build quality |
| Sensor calibration data table | Shows engineering testing and iteration |

---

## 22. Conclusion

ARBIBOT uses a separated power architecture to improve reliability during autonomous driving. The motor system is powered through a 3S Li-ion battery pack, BMS 3S 20A, and Cytron MD10C driver. The Jetson Orin Nano uses an independent Waveshare UPS module, reducing the risk of AI/vision resets caused by motor current spikes. The STM32F411 handles low-level sensing and motor control, while the Jetson handles vision and navigation.

The sensor architecture combines three VL53L4CD sensors, one VL53L8CH matrix sensor, an IMX477 camera, and an IMU. The right-front and right-rear ToF sensors provide wall-following and heading correction. The front matrix sensor supports front-barrier and corner-trigger detection. The camera and YOLO model provide red/green pillar classification and line/corner detection.

The strongest design decisions in this subsystem are:

- separating motor and Jetson power paths,
- using the STM32 for real-time sensor acquisition,
- using XSHUT pins to manage multiple VL53L4CD sensors,
- using two right-side sensors for heading correction,
- using a front matrix sensor instead of only a single front distance point,
- and calibrating sensors at fixed distances from 10 cm to 120 cm.

The remaining work is to replace estimated values with measured current draw and runtime data. Once measured values are added, this document will provide a strong power and sensor architecture explanation for WRO engineering evaluation.
