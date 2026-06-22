# MetallicMadness / ARBIBOT — WRO Future Engineers 2026

![ARBIBOT on the WRO Future Engineers field](engineering-journal/images/promo01.png)

**MetallicMadness** is a WRO Future Engineers self-driving car project developed for the **WRO 2026 Future Engineers — Self-Driving Cars** challenge. The robot is named **ARBIBOT**. It is an autonomous four-wheel vehicle that combines computer vision, time-of-flight distance sensing, embedded motor control, and a modular software architecture to complete the Open Challenge and Obstacle Challenge tasks.

The project focuses on building a reproducible autonomous vehicle that can drive several laps on the WRO racetrack, follow the correct driving direction, react to red and green traffic signs, avoid moving obstacles, and support parking behavior. The design uses a high-level AI processor for perception and navigation decisions, and a low-level STM32 controller for real-time motor control, sensor acquisition, and encoder feedback.

This repository contains the source code, engineering journal, wiring references, subsystem images, sensor and motor-control documentation, and supporting files required to understand, rebuild, and evaluate the robot.

---

## Team Information

| Item | Information |
|---|---|
| Team name | MetallicMadness |
| Robot name | ARBIBOT |
| Competition | WRO Future Engineers — Self-Driving Cars |
| Institution | Universidad Metropolitana, Caracas |
| Country | Venezuela |
| Team member | Daniel Jose Denis Castillo |
| Team member | Douglas Gabriel Ordaz Yanez |
| Tutor / Coach | Daniel Jose Denis Rodriguez |

### Team Members

**Daniel Jose Denis Castillo** is an Electrical Engineering student at Universidad Metropolitana in Caracas. He has a strong passion for electronics, robotics, Python programming, AI models, and practical problem solving. His work in this project focuses on embedded systems, robot integration, testing, and autonomous behavior.

**Douglas Gabriel Ordaz Yanez** is studying Electrical Engineering and Computer Engineering at Universidad Metropolitana in Caracas. He has a strong interest in software development, game programming, coding logic, and digital systems. His contribution focuses on programming logic, testing, software structure, and implementation support.

**Daniel Jose Denis Rodriguez**, the tutor, is an Electrical Engineer from Universidad Metropolitana with more than 25 years of experience in software engineering, telecommunications, AI, cybersecurity, embedded systems, and technology leadership. His role is to guide the students in project planning, engineering reasoning, documentation, and technical review.

---

## Project Goals

The main goal of ARBIBOT is to create an autonomous self-driving robot that can complete the WRO Future Engineers track using a combination of computer vision, ToF distance sensors, and embedded motor control.

The robot was designed around the following engineering goals:

1. **Autonomous driving:** the vehicle must drive without remote control or wireless assistance during the round.
2. **Stable lane behavior:** the robot must stay aligned with the track walls using distance sensors and heading correction.
3. **Obstacle strategy:** the robot must recognize red and green traffic signs and pass them on the correct side.
4. **Reliable embedded control:** motor, encoder, and sensor operations must be handled by a dedicated STM32 controller.
5. **High-level perception:** the Jetson Orin Nano processes camera input and runs YOLO-based detection models.
6. **Reproducibility:** the repository and engineering journal must allow another team to understand and rebuild the system with reasonable effort.
7. **Iteration and testing:** the project documents failures, design changes, and improvements made during development.

---

## Repository Structure

```text
WRO-MetallicMadness/
├── README.md
├── engineering-journal/
│   ├── README.md
│   ├── engineering-journal.md
│   ├── engineering-journal.pdf
│   └── images/
├── src/
│   ├── NVIDIA/
│   └── STM32F411/
├── schemes/
├── models/
├── t-photos/
├── v-photos/
├── video/
└── other/
```

### Folder Purpose

| Folder | Purpose |
|---|---|
| `engineering-journal/` | Main engineering documentation in Markdown and PDF format, including images and system explanations. |
| `src/NVIDIA/` | Python code running on the NVIDIA Jetson Orin Nano, including vision, navigation, serial communication, and steering control. |
| `src/STM32F411/` | STM32 firmware for motor control, distance sensor acquisition, encoder feedback, IMU reading, and DD-UART command handling. |
| `schemes/` | Wiring diagrams, power diagrams, system block diagrams, and subsystem architecture references. |
| `models/` | CAD, mechanical models, 3D printed parts, AI model notes, or model references used by the project. |
| `t-photos/` | Team photos required for WRO documentation. |
| `v-photos/` | Vehicle photos from all required views: front, rear, left, right, top, and bottom. |
| `video/` | Links or notes for Open Challenge and Obstacle Challenge autonomous driving videos. |
| `other/` | Additional supporting files such as datasheets, testing notes, calibration references, and protocol details. |

---

## Robot Overview

ARBIBOT is a compact 3D-printed autonomous vehicle using rear-wheel drive and front-wheel steering. The mechanical design follows the WRO requirement of a four-wheel vehicle with one driving axle and one steering actuator. The robot uses a custom front steering linkage and a single DC gear motor with an encoder for propulsion.

### Physical Characteristics

| Parameter | Value |
|---|---:|
| Length | 25 cm |
| Width | 15.6 cm |
| Height | 18 cm |
| Weight | 1.35 kg |
| Wheelbase | 13.9 cm |
| Track width | TODO: measure and fill |
| Chassis | 3D printed |
| Drive configuration | Rear-wheel drive |
| Steering | Servo-actuated pushrod steering with tie-rod linkage |

The steering system uses a servo arm to push and pull a linkage connected to the front steering knuckle. Both front wheels are mechanically connected by a tie rod, so the motion of one wheel is transferred to the other. This creates a synchronized front-wheel steering system using only one steering actuator. The geometry is Ackermann-inspired, but it is documented as a custom servo-driven pushrod/tie-rod linkage unless precise steering angle measurements are added later.

The 3D-printed chassis was selected because it allows fast mechanical iteration. Several bumper versions and sensor positions were tested. Early bumper designs created interference with the wheels while turning, so the front structure and sensor mounts were adjusted. The team also tested thinner wheels, but they slipped on the track, so wider wheels with better grip were selected.

---

## Mechanical Design

### Chassis

The robot uses a 3D-printed chassis designed to hold the Jetson Orin Nano, STM32 controller, motor driver, batteries, distance sensors, camera mount, steering system, and wiring. The chassis design prioritizes modularity and easy access to subsystems during testing.

The mechanical structure was developed through iteration. Early versions had issues with sensor height, bumper clearance, and torque transfer. These issues were corrected by modifying the sensor mount height, changing bumper geometry, and improving the drivetrain layout.

### Drive Motor

The drive motor is a **JGY-370B 12V Mini Worm Gear Motor with Encoder**, configured for approximately **150 RPM** no-load speed. It is a double-shaft DC gear motor with Hall encoder feedback.

Key characteristics:

| Item | Value |
|---|---|
| Motor type | DC worm gear motor with encoder |
| Model | JGY-370B |
| Rated voltage | 12 V DC |
| Speed | 150 RPM |
| Encoder type | Hall AB quadrature output |
| Basic pulse count | 11 PPR before gearbox multiplication |
| Encoder voltage | 3.3 V / 5 V |
| Output signal | Square wave AB phase |
| Direction | Reversible CW / CCW |
| Gear material | Metal gears |

The worm gear motor was selected because it provides useful torque for a compact robot and has a self-locking behavior when power is removed. The encoder allows the STM32 to measure wheel movement and RPM. The project originally tested a motor without an encoder, but this limited feedback and made distance-based movement less reliable.

### Steering Actuator

The steering actuator is an **MG996R servo**.

| Parameter | Value |
|---|---|
| Model | MG996R |
| Voltage | 5 V |
| Torque | approximately 9.4 kg·cm at 4.8 V |
| Control | Pololu servo controller |

The steering linkage was tuned by adjusting rod lengths and servo arm geometry. The final adjustments use a screw-based alignment method to modify the distance between linkage points and improve front-wheel alignment. This was necessary to increase turning range and reduce unwanted steering bias.

---

## System Architecture

ARBIBOT uses a two-controller architecture:

1. **NVIDIA Jetson Orin Nano** — high-level perception, vision, navigation logic, and decision making.
2. **STM32F411 Black Pill** — low-level sensor acquisition, motor control, encoder reading, and serial command execution.

This separation allows the robot to keep time-critical motor and sensor operations on the STM32 while the Jetson handles heavier AI and image-processing tasks. This design also improves modularity: the Jetson can send high-level commands, while the STM32 safely executes motor and sensor routines.

```text
Camera + YOLO + Navigation Logic
              │
              ▼
     NVIDIA Jetson Orin Nano
              │  DD-UART over USB/Serial
              ▼
         STM32F411 Black Pill
              │
 ┌────────────┼─────────────┐
 ▼            ▼             ▼
Motor      ToF Sensors     Encoder / IMU
Driver
```

---

## NVIDIA Jetson Orin Nano Subsystem

The Jetson Orin Nano is the high-level processor used for AI inference, camera processing, autonomous decision making, and communication with the STM32. It was selected because it provides enough AI acceleration and computing capacity to run YOLO inference and OpenCV processing in real time while also managing navigation logic.

### Jetson Responsibilities

The Jetson runs:

- Camera capture through a CSI camera pipeline.
- YOLO inference using custom-trained models.
- Red and green pillar detection.
- Blue/orange line and corner detection.
- Lane and wall-following logic.
- Navigation decision logic.
- Command transmission to the STM32 through the custom DD-UART serial protocol.
- Steering commands through a Pololu servo controller.

### Camera

The camera is a **Mini 12.3MP HQ Camera compatible with NVIDIA Jetson boards**, using a **Sony IMX477** sensor and an M12 mount lens. It is connected directly to the Jetson through the MIPI CSI-2 camera connector.

The camera is used for:

- Detecting red and green pillars.
- Detecting blue and orange track lines.
- Estimating the horizontal position of objects.
- Supporting corner detection and driving direction logic.

YOLO is used instead of only color thresholding because the lighting conditions on the field can change. Reflections, shadows, glare, camera exposure, and partial occlusion can make simple HSV thresholds unstable. YOLO provides both object localization and classification, making it more robust for competition environments.

---

## STM32F411 Subsystem

The STM32F411 Black Pill acts as the real-time controller for peripheral devices. Its main purpose is to isolate low-level deterministic tasks from the Jetson so that motor control and sensor reading remain stable even while the Jetson performs heavier vision inference.

### STM32 Responsibilities

The STM32 handles:

- Reading VL53L4CD side distance sensors.
- Reading the front VL53L8CH ToF matrix sensor.
- Managing the I2C sensor bus.
- Using XSHUT pins to initialize multiple VL53L4CD sensors with different addresses.
- Reading motor encoder AB phases.
- Measuring RPM and movement distance.
- Driving the Cytron MD10C motor driver using PWM and direction signals.
- Processing serial commands from the Jetson.
- Returning sensor and motor responses through DD-UART.

The STM32 firmware is organized into modules for protocol handling, motor control, distance sensors, IMU, platform I2C layers, and the main command-dispatch loop.

---

## Sensor Architecture

The robot uses multiple ST time-of-flight distance sensors to measure the distance to walls, obstacles, and front barriers. The sensor system is designed to support wall following, obstacle distance estimation, side correction, and parking behavior.

### Distance Sensors

| Sensor | Quantity | Placement | Purpose |
|---|---:|---|---|
| VL53L4CD | 3 | left front, right front, right rear/middle | Side distance, wall following, side correction, obstacle distance, parking support |
| VL53L8CH | 1 | front bumper / center front | Front matrix distance, barrier detection, corner trigger support |

### Sensor Placement

| Sensor | Location | I2C Address | XSHUT Pin |
|---|---|---|---|
| Left VL53L4CD | Front bumper, left side | 0x52 | PA5 |
| Right-front VL53L4CD | Front bumper, right side | 0x54 | PA7 |
| Right-rear VL53L4CD | Middle of car, between wheels | 0x56 | PB14 |
| Front VL53L8CH | Front bumper / center | 0x52 | Dedicated initialization / bus handling |

The two right-side sensors are used for wall-following and heading correction. The difference between the right-front and right-rear readings indicates whether the robot is parallel to the wall. This helps prevent the robot from drifting across the lane.

The front VL53L8CH is used as a matrix-based distance sensor. It helps detect the front wall and supports corner timing by identifying when the robot is approaching a barrier. The logic uses filtered readings to avoid false triggers from floor cells or isolated noisy readings.

An I2C expansion board is used to simplify wiring, distribute power, and share the I2C bus cleanly between multiple sensors. Multiple VL53L4CD sensors require XSHUT-controlled startup so that the STM32 can assign or activate addresses without bus conflicts.

### Sensor Calibration

The sensors are calibrated using fixed reference distances:

```text
120 cm, 110 cm, 100 cm, 90 cm, 80 cm, 70 cm, 60 cm, 50 cm, 40 cm, 30 cm, 20 cm, 10 cm
```

A Python test script reads all sensors after moving the car to each reference distance. This helps identify offsets, unstable readings, and sensor placement issues. The calibration process is important because small sensor errors can produce steering oscillation or incorrect corner detection.

---

## Motor Driver and Encoder

The robot uses a **Cytron MD10C motor driver** to control the 12 V DC gear motor. The MD10C receives control signals from the STM32 and motor power from the 3S battery system through the BMS.

### MD10C Control

| Signal | Source | Purpose |
|---|---|---|
| PWM | STM32 PA6 / timer channel | Controls motor speed |
| DIR | STM32 GPIO | Controls motor direction |
| GND | STM32 common ground | Shared logic reference |
| VIN | Motor battery / BMS | Motor power input |
| Motor output | DC gear motor | Drives rear axle |

The motor encoder provides AB quadrature feedback to the STM32. The firmware uses encoder pulses to calculate movement distance for move-by-degrees commands and to estimate motor speed in RPM.

---

## Power Architecture

ARBIBOT uses separate power paths for motor power and Jetson/electronics power. This separation reduces the risk that motor noise or high-current motor peaks will reset the Jetson, STM32, sensors, or serial communication.

### Motor Power Path

The motor power path uses **3 × 18650 lithium-ion cells** with a **3S 20A BMS**. The BMS output feeds the Cytron MD10C motor driver directly. The MD10C then powers the 12 V DC gear motor.

| Item | Value |
|---|---|
| Battery type | 18650 lithium-ion cells |
| Cell nominal voltage | 3.7 V |
| Cell capacity | 1200 mAh |
| Series configuration | 3S |
| BMS | 3S 20A BMS |
| Motor driver | Cytron MD10C |
| Motor voltage | 12 V nominal system |

### Jetson and Electronics Power Path

The Jetson uses a separate **Waveshare UPS module** with support for charging and output at the same time. The UPS supports three 18650 lithium batteries and provides the output used to power the Jetson and related electronics.

| Item | Value |
|---|---|
| UPS module | Waveshare UPS module for 3 × 18650 cells |
| Battery support | 3 × 18650 lithium cells |
| Charger | 12.6 V / 2 A |
| Output | Battery series voltage, 5 V 5 A, 3.3 V 300 mA |
| Jetson input | 12 V |
| Camera power | Directly from Jetson CSI camera interface |
| Pololu servo controller power | From the UPS/electronics power path |

### Estimated Power Budget

| Component | Voltage | Typical Current | Peak Current | Notes |
|---|---:|---:|---:|---|
| Jetson Orin Nano | 12 V | workload-dependent | up to high AI workload range | Runs camera, OpenCV, and YOLO inference |
| STM32F411 | 3.3 V / 5 V input | about 10 mA at high CPU speed, depending on peripherals | low | Handles real-time control |
| MG996R steering servo | 5 V | 120–900 mA depending on load | 1.5–2.5 A stall | Steering load depends on wheel friction and linkage geometry |
| JGY-370B drive motor | 12 V | 0.06–0.3 A typical | 1.3–2.0 A stall | Peak occurs during acceleration or stall |
| VL53 sensors | 3.3 V | 15–25 mA each while ranging | sensor-dependent | Multiple ToF sensors active during navigation |
| Camera | Jetson CSI | powered by Jetson | powered by Jetson | Used for YOLO detection |

The expected runtime is approximately **20–30 minutes**, which is higher than the required **3-minute challenge round** duration. The practical design target is not only runtime, but also stable voltage during motor acceleration, steering peaks, and AI inference load.

---

## Software Architecture

The software is divided into high-level Jetson Python modules and low-level STM32 C firmware.

### Jetson Software Modules

| File | Role |
|---|---|
| `main_challenge_01_v4.py` | Open Challenge logic using camera corner detection, heading wall-following, and front ToF matrix corner trigger. |
| `main_challenge_02.py` | Obstacle Challenge logic using pillar YOLO and PDI pass-side steering. |
| `turn_models.py` | Corner detector and line model helper functions. |
| `Pillar_recognition.py` | Converts YOLO results into pillar detections with color, zone, area, and guidance logic. |
| `pillar_counter.py` | Debounced enter/exit counting per pillar color. |
| `pid_line_follower.py` | PID lane controller using difference and single-wall modes. |
| `sensor_distance_v3.py` | Side ToF and front matrix reader/parser. Reduces matrix readings to useful front-barrier distance. |
| `motor_driver.py` | DD-UART motor command wrapper. |
| `servo_controller.py` | Steering servo control through its serial interface. |
| `color_tuning.py` | Per-camera color correction for detection stability. |

### STM32 Firmware Modules

| File | Role |
|---|---|
| `serial_dd_protocol.c/.h` | Parses and validates DD-UART frames, verifies checksum, dispatches commands, and builds responses. |
| `motor_drv.c/.h` | Controls PWM, motor direction, move-by-degrees, encoder counting, and RPM calculation. |
| `sensor_vl53l4cd.c` | Reads side VL53L4CD ToF sensors. |
| `sensor_vl53l7ch.c` | Reads front VL53L7CX/VL53L8 matrix sensor. |
| `sensor_imu.c` | Reads gyro and accelerometer values. |
| `platform.c / platform2.c` | I2C platform layers for the ToF sensors. |
| `main.c` | Main command dispatch and system loop. |

---

## Navigation and Control Strategy

The robot uses a state-machine approach to organize behavior. The main states are:

```text
WAITING_FOR_START
FIRST_SECTOR
STRAIGHT_DRIVE
CORNERING
OBSTACLE_HANDLING
FINISH / STOP
```

### Open Challenge Strategy

In the Open Challenge, the robot drives without red or green pillars. The main navigation goal is to complete three laps while staying inside the lane and correctly detecting corners. The robot uses side ToF sensors to follow the wall and front ToF matrix readings to detect approaching barriers. Camera-based line/corner recognition supports driving direction and corner logic.

### Obstacle Challenge Strategy

In the Obstacle Challenge, the robot must detect red and green traffic signs and pass them on the correct side. The pillar YOLO model returns bounding boxes and classes for **Green_Pillar** and **Red_Pillar**. The robot uses the object class and horizontal location to decide the steering correction.

Rules used by the robot:

- Red pillar: pass on the right side.
- Green pillar: pass on the left side.

The system chooses the nearest relevant pillar using bounding-box area. Larger bounding boxes normally indicate that the pillar is closer. The robot divides the camera image into left, center, and right zones to estimate where the pillar is located relative to the car.

### Control Algorithms

The robot uses several control methods:

| Control method | Purpose |
|---|---|
| PID | Lane and wall-following correction. |
| PDI | Pillar pass-side steering correction. |
| Proportional heading correction | Uses right-front and right-rear ToF sensor difference to keep the robot parallel to the wall. |
| Threshold logic | Detects corner trigger windows and front barrier distance. |
| Sensor fusion | Combines camera, side ToF, and front matrix readings for more reliable navigation. |

---

## DD-UART Serial Protocol

The Jetson communicates with the STM32 using a custom binary protocol called **DD-UART**. This protocol is used to request sensor data, command motor actions, set speed, and stop the robot.

### Frame Format

```text
$ LEN(2 bytes, big-endian) CMD(2 bytes, big-endian) DATA(0..n) CHK(1 byte) \n
```

- `$` is the start byte.
- `LEN` is the total frame length.
- `CMD` is the command ID.
- `DATA` is the optional payload.
- `CHK` is an XOR checksum over the frame content between the start byte and checksum.
- `\n` is the end byte.

### Main Commands

| Command | Name | Payload | Purpose |
|---:|---|---|---|
| `0x0001` | READ_GYRO | none | Read IMU gyro data. |
| `0x0002` | READ_ACCEL | none | Read IMU acceleration data. |
| `0x0003` | READ_FRONT | none | Read front VL53L8/VL53L7 matrix. |
| `0x0004` | FORWARD continuous | none | Drive forward continuously. |
| `0x0005` | REVERSE continuous | none | Drive reverse continuously. |
| `0x0101` | FORWARD by degrees | uint16 degrees | Move forward by encoder-derived distance. |
| `0x0102` | REVERSE by degrees | uint16 degrees | Move reverse by encoder-derived distance. |
| `0x0103` | STOP | none | Stop drive motor. |
| `0x0104` | SET SPEED | uint8 percent | Set motor speed from 0 to 100%. |
| `0x0105` | READ SIDES | none | Read VL53L4CD side sensors. |

Because sensors and motor commands share one serial link, the Jetson avoids blocking motor commands while the sensor thread owns serial reads. Critical commands such as start and stop are sent multiple times to reduce the chance of a dropped frame affecting the run.

---

## Vision and YOLO Models

ARBIBOT uses Ultralytics YOLO with custom-trained weights. The vision system currently uses two model roles:

1. **Pillar model** for the Obstacle Challenge.
2. **Line/corner model** for the Open Challenge.

### Pillar Detection

The pillar model detects:

- `Green_Pillar`
- `Red_Pillar`

The output used by the navigation system includes:

- Class name.
- Bounding-box center X.
- Bounding-box area.
- Confidence.
- Zone: left, center, or right.

The area is used as an approximate closeness signal. The horizontal position is used to calculate steering error and guide the robot to pass the pillar on the correct side.

### Items Still To Be Completed

The following vision information should be added after measurement or review:

- Exact YOLO version.
- Dataset size per model.
- Annotation tool used.
- Training environment.
- Inference FPS on Jetson.
- Average inference time in milliseconds.
- Final confidence thresholds.
- Accuracy or validation metrics.

---

## Testing and Iteration

The robot was developed through multiple software and hardware iterations. The development process included testing drivetrain options, sensor layouts, bumper geometry, motor feedback, serial protocol behavior, and vision model performance.

### Main Iterations

| Version | Description |
|---|---|
| v1 | Initial camera-based corner detection and open-loop center steering. |
| v2 | Added PID wall-following and sensor/front-ToF corner fallbacks. |
| v3 | Tested two-right-sensor wall following and VL53L8 4x4 matrix logic. This version became complex and unstable. |
| v4 | Current Open Challenge base: improved sensor firmware, front-matrix corner trigger, and RF-RR heading wall-follower. |
| Challenge 02 | Current Obstacle Challenge logic: pillar YOLO and PDI pass-side steering. |

### Problems Found and Fixes Applied

| Problem | Cause | Fix |
|---|---|---|
| Robot turned too late and hit front wall | Front matrix readings included floor cells and produced incorrect barrier distance. | Filtered matrix data and used more reliable front-barrier rows. |
| Robot turned too soon | Single noisy matrix cell was interpreted as a wall. | Required at least two valid in-band cells before firing corner trigger. |
| Robot sometimes started incorrectly or reversed | Motor speed command behavior and dropped frames on shared serial bus. | Sent forward command first, then speed commands, with repeated pulses. |
| Stop command sometimes failed | Stop frame dropped during sensor matrix scan. | Repeated stop command to improve reliability. |
| Side sensors returned invalid values | Parser mismatch and heavy YOLO load affected serial timing. | Updated parser and reduced heavy matrix polling until needed. |
| Lane wandered from wall to wall | Distance-only PID did not account for heading. | Added heading correction using right-front minus right-rear sensor difference. |
| Pillar steering full-locked or lost target | PDI gains too high and reaction too late. | Reduced gains, lowered steering clamp, and reacted earlier. |

### Testing Metrics To Add

The following results should be measured and added as the robot improves:

| Metric | Value |
|---|---|
| Best lap time | TODO |
| Average lap time | TODO |
| Open Challenge success rate | TODO |
| Obstacle Challenge success rate | TODO |
| Parking success rate | TODO |
| YOLO FPS | TODO |
| Front-matrix corner trigger distance | approximately 100 cm observed, confirm with tests |
| Wall-follow target gap error | TODO |

---

## Build and Setup Overview

### Hardware Assembly Summary

1. Print and prepare the chassis and sensor mounts.
2. Install the rear drive motor and connect it mechanically to the driven axle.
3. Install the steering servo and connect the pushrod/tie-rod steering linkage.
4. Mount the Jetson Orin Nano securely on the top platform.
5. Mount the STM32F411 and connect the sensor wiring.
6. Install the Cytron MD10C and connect it to the motor battery path.
7. Install the BMS and motor battery pack.
8. Install the Waveshare UPS and Jetson/electronics battery pack.
9. Mount the VL53L4CD and VL53L8CH sensors in their final positions.
10. Mount the camera and verify the field of view.
11. Connect common grounds where required by the electronics architecture.
12. Verify wiring before powering motors.

### Jetson Setup Summary

The Jetson runs the high-level Python software. The setup includes:

- Jetson Linux / Ubuntu environment.
- Python 3 environment.
- OpenCV.
- Ultralytics YOLO.
- PySerial.
- Camera support through GStreamer / CSI pipeline.
- Project Python modules from `src/NVIDIA/`.

Exact OS version, Python version, and package versions should be added to this README once finalized.

### STM32 Setup Summary

The STM32 firmware is built and flashed using the STM32 development environment used by the team. The firmware includes motor driver control, sensor drivers, protocol handling, and command dispatch.

The STM32 must be connected to the Jetson through USB/serial using the configured USART interface. The sensor bus and motor driver pins must match the firmware pin definitions.

### Start Procedure

The intended competition procedure is:

1. Place the robot in the start zone while switched off.
2. Switch on the robot using the allowed power switch.
3. The robot enters a waiting state.
4. At the judge signal, press the start button.
5. The robot begins autonomous driving.
6. The robot stops automatically after completing the challenge or when commanded by its internal logic.

---

## Documentation

The main engineering documentation is available in:

- [`engineering-journal/engineering-journal.md`](engineering-journal/engineering-journal.md)
- [`engineering-journal/engineering-journal.pdf`](engineering-journal/engineering-journal.pdf)

The Engineering Journal contains more detailed explanations of:

- Mechanical design.
- Power architecture.
- Sensor architecture.
- Jetson and STM32 responsibilities.
- YOLO and navigation strategy.
- DD-UART communication.
- Testing failures and improvements.
- Reproducibility notes.

---

## Media Requirements

WRO documentation requires clear evidence of the final robot and its autonomous behavior. The following files should be added as the project progresses:

### Vehicle Photos

Place these in `v-photos/`:

- Front view.
- Rear view.
- Left side view.
- Right side view.
- Top view.
- Bottom view.
- Steering close-up.
- Sensor mounting close-up.
- Motor and encoder close-up.
- Battery and wiring layout.

### Team Photos

Place team photos in `t-photos/`.

### Videos

Place video links or notes in `video/`:

- Open Challenge autonomous run video.
- Obstacle Challenge autonomous run video.

Each video should clearly show the vehicle driving autonomously for at least 30 seconds.

---

## Current Status

The robot currently has a working hardware and software architecture with the following completed or partially completed subsystems:

- 3D-printed chassis.
- Rear-wheel drive motor and encoder system.
- Servo-actuated front steering.
- Jetson Orin Nano high-level controller.
- STM32F411 low-level controller.
- Cytron MD10C motor control.
- VL53L4CD side sensors.
- VL53L8CH front matrix sensor.
- DD-UART protocol between Jetson and STM32.
- YOLO-based red/green pillar detection.
- Camera-assisted line/corner detection.
- PID/PDI-based navigation logic.
- Engineering Journal first complete draft.

Remaining work includes:

- Fill final physical measurements such as track width and wheel diameter.
- Add official robot photos from all required views.
- Add team photo.
- Add final challenge video links.
- Measure final current consumption.
- Add exact Jetson OS, Python version, and library versions.
- Add YOLO training dataset details and performance metrics.
- Add final Open Challenge and Obstacle Challenge test results.
- Improve parking documentation and final parking test metrics.

---

## Engineering Philosophy

The project is built around a simple principle: the robot should be understandable as a complete system, not just a collection of parts. The Jetson, STM32, sensors, motor driver, batteries, software modules, and mechanical chassis all affect each other. A faster motor changes steering behavior. Sensor placement changes the control loop. Power stability affects serial communication and vision inference. Camera detection affects obstacle strategy. The final design must therefore be evaluated as a full engineering system.

The documentation in this repository is intended to show not only what was built, but why decisions were made. The team tested alternatives, identified failures, and improved the design through iteration. This includes changing motors, changing wheels, adjusting bumper geometry, replacing ultrasonic sensors with laser ToF sensors, tuning steering linkage geometry, filtering front matrix sensor readings, and improving serial command reliability.

---

## License and Use

This repository is prepared for educational and competition documentation purposes. The project is part of the WRO Future Engineers learning process and is intended to help demonstrate the engineering work performed by the MetallicMadness team.

