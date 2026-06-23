# 05 - Systems Thinking and Engineering Decisions

**Project:** MetallicMadness / ARBIBOT  
**Category:** WRO 2026 Future Engineers - Self-Driving Cars  
**Document purpose:** Explain the main engineering decisions behind ARBIBOT, including trade-offs, rejected alternatives, system-level reasoning, failure-driven changes, and how each subsystem affects the others.

> This document focuses on the question: **why was the robot designed this way?**  
> The goal is not only to describe the final design, but to show the thinking process that led to it.

---

## 1. Systems Thinking Overview

ARBIBOT is not a single component robot. It is a complete embedded autonomy system made of mechanical parts, sensors, power electronics, AI vision, low-level control, and competition strategy.

A change in one subsystem affects the others. For example:

- Moving a distance sensor changes the software thresholds.
- Increasing motor speed changes YOLO reaction time.
- Steering linkage geometry changes the required servo values.
- Battery voltage drops can affect motor behavior and sensor stability.
- A blocking motor command can slow the camera loop and cause missed pillar detections.
- A bumper design can make a sensor more accurate, but also cause tire rubbing.

For this reason, the project was developed using system-level thinking instead of isolated component selection.

The main design approach was:

```text
Requirement -> Prototype -> Test -> Failure -> Root cause -> Fix -> Retest -> Document
```

---

## 2. Main System Requirements

The robot was designed around these practical requirements:

| Requirement | Design implication |
|---|---|
| Drive autonomously around the WRO Future Engineers field | Requires stable steering, motor control, distance sensing, and navigation logic |
| Complete the Open Challenge | Requires lane following, corner detection, lap logic, and reliable stop behavior |
| Complete the Obstacle Challenge | Requires red/green pillar detection, pass-side decision, corner handling, and parking behavior |
| Use a real car-like drive base | Requires one driving axle and one steering actuator, not differential steering |
| Fit inside size and weight constraints | Requires compact chassis, compact wiring, and lightweight mounting |
| React fast enough to obstacles | Requires non-blocking software, camera inference, and early pillar response |
| Survive repeated testing | Requires reliable power, mechanical stiffness, and robust serial communication |
| Be reproducible from GitHub | Requires documentation, diagrams, code, photos, videos, and test data |

---

## 3. High-Level Design Philosophy

The final design is based on three engineering principles.

### 3.1 Divide the robot into responsibility domains

ARBIBOT separates responsibilities between two main controllers:

| Controller | Responsibility |
|---|---|
| NVIDIA Jetson Orin Nano | Vision, AI inference, high-level navigation, challenge strategy |
| STM32F411 | Real-time sensor polling, motor control, encoder reading, low-level I/O |

This split avoids forcing one processor to handle everything. The Jetson is strong for AI and camera processing, while the STM32 is better for deterministic embedded control.

### 3.2 Use sensor fusion instead of one “perfect” sensor

No single sensor was trusted to solve the whole challenge. ARBIBOT combines:

- camera and YOLO for pillars and visual features,
- VL53L4CD side distance sensors for wall following,
- VL53L8CX front matrix sensor for front barrier and corner detection,
- motor encoder for movement feedback,
- IMU data for motion information,
- state-machine logic for context.

The robot does not ask “which sensor is perfect?”  
It asks “which sensor is useful for this situation?”

### 3.3 Prefer repeatable behavior over maximum speed

A fast robot that crashes is not useful. The design prioritizes stable lap completion, predictable steering, and recoverable behavior before increasing speed.

---

## 4. System Architecture Decision

### Decision

Use a two-level control architecture:

```text
Jetson Orin Nano -> high-level autonomy
STM32F411 -> low-level embedded control
```

### Reasoning

The Jetson is ideal for:

- camera capture,
- YOLO model inference,
- OpenCV processing,
- obstacle recognition,
- high-level decision logic,
- state-machine control.

The STM32 is ideal for:

- real-time motor commands,
- PWM generation,
- encoder interrupts,
- I2C sensor polling,
- XSHUT sensor address control,
- low-level command response.

### Alternatives considered

| Alternative | Why it was rejected |
|---|---|
| Jetson controls everything directly | More complex wiring, less deterministic timing, harder real-time motor/encoder control |
| STM32 controls everything including vision | Not practical for YOLO/camera inference |
| Raspberry Pi only | Less suitable than Jetson for GPU-based vision inference |
| Arduino-only system | Insufficient for camera AI and full challenge logic |

### System-level benefit

This decision makes the robot easier to debug. If vision fails, the Jetson side is investigated. If motor response fails, the STM32 side is investigated. This separation reduces confusion during testing.

---

## 5. Mechanical Platform Decision

### Decision

Use a 3D-printed chassis with rear-wheel drive and front steering.

### Reasoning

This matches the behavior of a real car-style robot:

- one driving axle,
- front steering,
- clear mechanical direction,
- predictable turning behavior,
- room for sensors and electronics.

The robot dimensions are approximately:

| Dimension | Value |
|---|---:|
| Length | 25 cm |
| Width | 15.6 cm |
| Height | 18 cm |
| Weight | 1.35 kg |
| Wheelbase | 13.9 cm |
| Track width | 13.5 cm center-to-center |

### Alternatives considered

| Alternative | Issue |
|---|---|
| Differential drive | Does not match the intended car-like driving architecture |
| Commercial RC chassis | Less control over sensor placement and custom bumper geometry |
| Fully laser-cut chassis | Less flexible for fast iteration than 3D printing |
| Very low chassis | Poor clearance for electronics and sensor mounting |

### Final reasoning

A 3D-printed chassis allowed rapid iteration. When the bumper rubbed the tires or the sensor height was wrong, the design could be changed and reprinted.

---

## 6. Drive Motor Decision

### Decision

Use a **JGY-370B 12V mini worm gear motor with encoder**, double shaft, approximately 150 RPM.

### Reasoning

The drive motor needed:

- enough torque to move a 1.35 kg robot,
- manageable speed for control,
- encoder feedback,
- reversible motion,
- compact size,
- metal gears,
- reliable low-speed behavior.

A worm gear motor is slower than some DC motors, but it gives better controllability and torque for a WRO vehicle.

### Alternatives tested

| Option | Result |
|---|---|
| Motor without encoder | Movement was possible but feedback was limited |
| Single-shaft gearbox motor with gear transfer to wheels | Torque transfer problem between gearbox gear and wheel gear |
| Faster motor option | Risked reducing control stability and reaction time |
| Final JGY-370B encoder motor | Better balance of speed, torque, and feedback |

### System-level effect

Motor speed affects:

- how much time YOLO has to detect pillars,
- how soon the robot must start steering,
- how reliable corner detection must be,
- how much braking distance is required,
- how much current the drive system draws.

For this reason, speed is tuned as part of the full robot behavior, not only as a motor parameter.

---

## 7. Steering Mechanism Decision

### Decision

Use a **servo-actuated pushrod steering system** with a tie-rod linkage between both front wheels.

### Reasoning

The steering system needed to be:

- simple,
- compact,
- strong enough for the robot weight,
- easy to tune mechanically,
- compatible with a standard servo,
- reproducible with 3D-printed parts and common hardware.

The selected servo is an **MG996R**, powered at 5V, with enough torque for the front steering linkage.

### Alternatives and iterations

| Issue | Change |
|---|---|
| Steering rods caused poor alignment | Rod length and geometry adjusted |
| Servo arm length affected turning response | Servo horn/arm geometry tuned |
| Front wheels needed better alignment | Final screw added to adjust front-wheel alignment |
| Bumper interfered with tire during steering | Bumper geometry redesigned |

### System-level effect

Steering is not only mechanical. It affects:

- PID tuning,
- PDI obstacle steering,
- turning radius,
- corner entry timing,
- speed limit,
- lane recovery after a turn.

If the steering linkage is misaligned, the software will appear unstable even if the algorithm is correct.

---

## 8. Wheel and Traction Decision

### Decision

Use wider rubber wheels instead of thinner wheels.

### Reasoning

Thinner wheels were tested and caused sliding. Sliding makes the robot unpredictable because the software assumes steering commands produce repeatable motion.

### Trade-off

| Wheel type | Advantage | Disadvantage |
|---|---|---|
| Thin wheels | Lower rolling resistance | More sliding, less stable control |
| Wider wheels | Better grip and repeatability | Slightly more rolling resistance |

### Final reasoning

For WRO, repeatable control is more valuable than theoretical maximum speed. The wider wheels produced more stable behavior.

---

## 9. Distance Sensor Decision

### Decision

Use ST VL53 time-of-flight sensors instead of ultrasonic sensors.

### Reasoning

Ultrasonic sensors were tested and rejected because their readings were less suitable for the compact geometry and precision required by the WRO field. Laser time-of-flight sensors gave better distance precision and faster reaction in the robot’s operating range.

### Final sensor set

| Sensor | Quantity | Main use |
|---|---:|---|
| VL53L4CD | 3 | Side/front distance sensing, wall following, correction |
| VL53L8CX / VL53L7CX-style matrix | 1 | Front matrix distance sensing |
| Camera | 1 | Pillar and visual feature detection |
| IMU | 1 | Motion information |
| Motor encoder | 1 | Wheel feedback |

### Alternatives considered

| Option | Issue |
|---|---|
| Ultrasonic sensors | Less precise, wider cone, harder to use near walls and corners |
| Camera-only navigation | Too dependent on lighting and visual detection |
| Side sensors only | Not enough for front barrier/corner confirmation |
| Front sensor only | Not enough for wall-following and heading correction |

### Final reasoning

The final sensor design uses ToF sensors for geometry and the camera for semantic understanding. This is a stronger combination than either method alone.

---

## 10. Side Sensor Placement Decision

### Decision

Use two right-side VL53L4CD sensors: one right-front and one right-rear.

### Reasoning

A single side distance sensor can measure wall distance, but it cannot determine heading. The robot can be at the correct distance while still angled toward the wall.

With two sensors:

```text
heading_error = right_front_distance - right_rear_distance
```

This gives an estimate of whether the robot is angled toward or away from the wall.

### System-level benefit

This improved wall-following stability because the controller can correct both:

- lateral distance error,
- heading error.

### Alternatives considered

| Alternative | Issue |
|---|---|
| One right-side sensor | Distance-only control caused wall-to-wall wandering |
| Camera-only lane following | More sensitive to lighting and visual occlusion |
| IMU-only heading correction | Drift and calibration complexity |
| Two right-side ToF sensors | Simple, direct, useful for wall-following |

### Final reasoning

Two side sensors created a simple and practical heading signal without needing complex localization.

---

## 11. Front Matrix Sensor Decision

### Decision

Use the VL53L8CX front matrix sensor to help detect front barriers and corners.

### Reasoning

A single front distance value can be misleading. A matrix sensor provides multiple zones, making it possible to detect whether the robot is approaching a wall or if only one edge/corner is visible.

### Problem found during testing

The front matrix created two problems:

| Failure | Root cause |
|---|---|
| Turn too late | Some matrix cells saw the floor or irrelevant surfaces |
| Turn too early | A single side cell falsely looked like a front barrier |

### Fix

The logic was improved by:

- selecting the relevant row,
- filtering floor-band cells,
- accepting only values inside a valid distance band,
- requiring at least two valid cells before triggering.

### System-level lesson

More sensor data is not automatically better. Matrix data must be interpreted carefully, otherwise it can create false confidence.

---

## 12. Vision / YOLO Decision

### Decision

Use YOLO object detection for pillar recognition and visual challenge features.

### Reasoning

The Obstacle Challenge requires the robot to identify red and green pillars. Simple color thresholding can work in controlled lighting, but it can fail with shadows, glare, exposure changes, and partial occlusions.

YOLO provides:

- class detection,
- bounding box location,
- confidence value,
- object area estimate,
- better tolerance to lighting changes than simple color masks.

### Alternatives considered

| Alternative | Issue |
|---|---|
| HSV thresholding only | Sensitive to lighting and camera exposure |
| Shape detection only | Pillars can appear different depending on angle and distance |
| Manual color segmentation | Harder to make robust across all lighting |
| YOLO model | Requires training and Jetson compute, but improves detection reliability |

### Final reasoning

YOLO was selected because obstacle color and position are strategic information. The robot needs to know not just that “something” is present, but whether it is red or green and where it is in the frame.

---

## 13. Obstacle Strategy Decision

### Decision

Use pass-side logic based on pillar color:

| Pillar color | Required behavior |
|---|---|
| Red pillar | Keep/pass on the right side |
| Green pillar | Keep/pass on the left side |

### Control approach

The robot uses YOLO output to estimate:

- object color,
- object center,
- object area,
- screen zone,
- nearest/largest obstacle.

Then a PDI-style controller adjusts steering so that the pillar moves toward the correct side of the image before the robot reaches it.

### Why not hard-coded steering?

A fixed steering maneuver is too fragile because pillars can appear at different positions and distances. Using bounding-box feedback allows the robot to adjust based on the current visual situation.

### Problems found

| Problem | Fix |
|---|---|
| Steering went full-lock near pillar | Reduce PDI gains and steering clamp |
| Robot reacted too late | Start response earlier |
| Lost target during aggressive steering | Hold/recover last valid detection briefly |
| Green pillar recognition was affected by motor blocking | Use non-blocking motor command strategy |

### Final reasoning

The obstacle strategy combines visual recognition with feedback steering instead of using a fixed motion pattern.

---

## 14. Power Architecture Decision

### Decision

Use separated power domains:

1. Motor power domain.
2. Electronics / compute power domain.

### Motor power domain

```text
3S 18650 battery pack -> 3S 20A BMS -> Cytron MD10C -> DC gear motor
```

### Electronics / compute power domain

```text
Waveshare UPS module -> Jetson Orin Nano / camera / servo controller / low-current electronics
```

### Reasoning

The motor can create current spikes and electrical noise. The Jetson, camera, sensors, and serial communication are more sensitive to voltage instability.

Separating power paths reduces risk of:

- Jetson brownouts,
- camera drops,
- UART errors,
- I2C instability,
- sensor resets,
- servo jitter,
- STM32 communication problems.

### Trade-off

| Option | Advantage | Disadvantage |
|---|---|---|
| One shared battery for everything | Simpler wiring | Higher brownout/noise risk |
| Separate power domains | More reliable electronics | More wiring and more weight |
| Final design | Better stability for competition | Requires careful grounding |

### System-level note

Even with separated power paths, signal systems still need a valid common ground reference where required.

---

## 15. Motor Driver Decision

### Decision

Use a **Cytron MD10C** motor driver.

### Reasoning

The motor driver must handle:

- DC gear motor power,
- forward/reverse control,
- PWM speed control,
- expected current draw,
- reliable operation during repeated testing.

The MD10C provides a simple and robust interface between STM32 control signals and the motor power path.

### Alternatives considered

| Alternative | Issue |
|---|---|
| Small L298N-style driver | Lower efficiency and not ideal for higher-current motor use |
| Direct MOSFET design | More custom design risk and debugging time |
| Cytron MD10C | Known module, suitable current capacity, easier integration |

### Final reasoning

A robust motor driver reduces risk. The motor path is a high-current subsystem, so reliability is more important than saving a few grams or wires.

---

## 16. Servo Controller Decision

### Decision

Use a Pololu servo controller for steering servo control.

### Reasoning

The Pololu controller gives stable servo pulse generation and separates steering servo command generation from the main Jetson/STM32 timing loads.

### Alternatives considered

| Alternative | Issue |
|---|---|
| Jetson PWM directly | More Linux timing uncertainty |
| STM32 PWM directly | Possible, but STM32 is already managing motor, sensors, encoder, and protocol |
| Pololu controller | Dedicated servo control and easy serial interface |

### Final reasoning

Using a dedicated servo controller simplified steering control and reduced timing complexity.

---

## 17. Communication Protocol Decision

### Decision

Use a custom binary UART protocol between Jetson and STM32.

Frame format:

```text
$ LEN(2 bytes) CMD(2 bytes) DATA(0..n) CHECKSUM(1 byte) \n
```

### Reasoning

A binary frame format is compact and easier to validate than loose text commands. The checksum helps detect corrupted frames.

### Benefits

- consistent command format,
- small message size,
- clear command IDs,
- response frames,
- checksum validation,
- easier debugging with hex traces.

### Alternatives considered

| Alternative | Issue |
|---|---|
| Plain text commands | Easier to type, but harder to parse robustly |
| JSON over serial | Too verbose for low-level command loop |
| ROS-style messaging | Too heavy for this embedded link |
| Custom binary DD-UART | Compact and controllable |

### System-level issue discovered

Motor commands and sensor polling share the serial link. Blocking motor waits slowed the vision loop and affected pillar detection. This led to a software decision:

```text
Sensor thread owns serial reads.
Motor commands are sent fire-and-forget where safe.
Critical commands are pulsed more than once.
```

---

## 18. Software State Machine Decision

### Decision

Use an explicit state machine to organize robot behavior.

Example states:

```text
WAITING_FOR_START
FIRST_SECTOR
STRAIGHT_DRIVE
CORNERING
OBSTACLE_HANDLING
PARKING
FINISH
```

### Reasoning

A state machine makes the robot easier to reason about than one large loop with many unrelated if-statements.

### Benefits

- clearer challenge logic,
- easier debugging,
- safer transitions,
- prevents obstacle logic from fighting corner logic,
- allows cooldowns after turns,
- makes stop behavior easier to manage.

### Alternative considered

| Alternative | Issue |
|---|---|
| One continuous control loop with many flags | Hard to debug and easy to create conflicting actions |
| Fully reactive behavior only | Robot may react incorrectly without context |
| State machine | More structured and easier to document |

### Final reasoning

The field has repeated but contextual events. A state machine gives the robot memory of what it is doing.

---

## 19. Open Challenge Strategy Decision

### Decision

Use wall-following, corner detection, and front-matrix support to complete the Open Challenge.

### Reasoning

The Open Challenge does not require pillar recognition, but it requires stable lap behavior. The main challenge is not object classification; it is consistent navigation.

The current Open Challenge base is:

```text
main_challenge_01_v4.py
```

### Evolution

| Version | Decision |
|---|---|
| v1 | Camera corner detection and open-loop steering |
| v2 | Add PID wall-following and sensor fallback |
| v3 | Try two-right-sensor and front matrix approach, but became unstable |
| v4 | Combine v2 base with improved sensors, front matrix trigger, and RF-RR heading correction |

### Final reasoning

v4 is preferred because it keeps useful improvements but avoids unnecessary complexity from v3.

---

## 20. Failure-Driven Decisions

Many final decisions came from failures, not from first assumptions.

| Failure | Decision created |
|---|---|
| Ultrasonic sensors were not precise enough | Switch to VL53 ToF sensors |
| Thin wheels slipped | Use wider rubber wheels |
| Bumper rubbed tires | Redesign bumper and sensor mounts |
| Side distance-only control wandered | Add right-front/right-rear heading correction |
| Front matrix fired too soon | Require multiple valid cells |
| Front matrix fired too late | Filter floor-band cells |
| Stop command sometimes missed | Pulse critical stop commands |
| Motor command blocked vision | Use non-blocking command strategy |
| Green pillar recognition failed during motion | Protect camera/YOLO loop timing |
| PDI obstacle control oversteered | Reduce gain and clamp steering |

### Lesson

The final robot is the result of iteration. Each failure reduced uncertainty and improved the system.

---

## 21. Engineering Decision Record Summary

| ID | Decision | Main reason | Status |
|---|---|---|---|
| ADR-001 | Use Jetson + STM32 architecture | Separate AI/navigation from real-time control | Accepted |
| ADR-002 | Use rear-wheel drive and front steering | Car-like control and WRO suitability | Accepted |
| ADR-003 | Use 3D-printed chassis | Fast iteration and custom mounting | Accepted |
| ADR-004 | Use JGY-370B encoder motor | Torque, feedback, controllable speed | Accepted |
| ADR-005 | Use MG996R servo steering | Enough torque and simple linkage | Accepted |
| ADR-006 | Use VL53 ToF sensors | Better precision than ultrasonic sensors | Accepted |
| ADR-007 | Use two right-side sensors | Heading correction for wall following | Accepted |
| ADR-008 | Use front matrix sensor | Better front barrier/corner information | Accepted |
| ADR-009 | Use YOLO for pillars | Robust object/color classification | Accepted |
| ADR-010 | Separate motor and electronics power | Reduce brownout/noise risk | Accepted |
| ADR-011 | Use Cytron MD10C | Robust motor control | Accepted |
| ADR-012 | Use Pololu servo controller | Dedicated servo pulse control | Accepted |
| ADR-013 | Use custom binary UART protocol | Compact command and checksum validation | Accepted |
| ADR-014 | Use non-blocking motor command strategy | Prevent vision-loop delays | Accepted |
| ADR-015 | Use state-machine navigation | Clear challenge behavior and transitions | Accepted |

---

## 22. Trade-Off Matrix

| Decision area | Option chosen | Benefit | Cost / risk |
|---|---|---|---|
| Main compute | Jetson Orin Nano | Strong AI vision performance | Higher power draw |
| Low-level controller | STM32F411 | Real-time control and I2C handling | Requires firmware development |
| Drive motor | Encoder worm gear motor | Torque and feedback | Lower maximum speed |
| Steering | Servo pushrod linkage | Simple and adjustable | Requires mechanical tuning |
| Distance sensors | VL53 ToF sensors | Better precision | I2C address/XSHUT complexity |
| Front sensing | VL53L8CX matrix | Multi-zone front detection | More complex filtering |
| Vision | YOLO | Robust red/green recognition | Requires model training and GPU |
| Power | Separate domains | More reliable electronics | More wiring and weight |
| Serial protocol | Binary DD-UART | Compact and reliable | Requires custom parser |
| Control strategy | State machine + PID/PDI | Organized behavior | Needs careful transitions |

---

## 23. Risk Analysis and Mitigation

| Risk | Impact | Mitigation |
|---|---|---|
| Jetson brownout | Robot loses vision/control | Use UPS power path and monitor voltage |
| Motor noise affects sensors | Bad distance readings or serial errors | Separate power paths and route wiring carefully |
| I2C address conflict | Sensor initialization failure | Use XSHUT-controlled startup |
| YOLO misses pillar | Wrong obstacle decision | Tune confidence, lighting, dataset, and reaction timing |
| Camera loop blocked by serial | Delayed obstacle response | Use sensor-thread ownership and non-blocking motor commands |
| Steering overcorrects | Robot hits wall or pillar | Clamp steering and tune gains |
| Matrix false trigger | Turn too early or too late | Filter cells and require multiple valid cells |
| Wheel slip | Control becomes unpredictable | Use wider rubber wheels |
| Mechanical misalignment | Software seems unstable | Add alignment screw and measure servo center |
| Large files in repo | Slow clone/push | Optimize images/videos or use Git LFS if needed |

---

## 24. Reproducibility Decisions

The robot is documented so another team member can understand and rebuild the system.

Important reproducibility choices:

- diagrams are stored in `schemes/`,
- vehicle photos are stored in `v-photos/`,
- team photos are stored in `t-photos/`,
- challenge videos are stored in `video/`,
- source code is stored in `src/`,
- models are stored in `models/`,
- detailed explanation is stored in `docs/`,
- the engineering journal is stored in `engineering-journal/`.

### Why this matters

A working robot without documentation is difficult to evaluate and reproduce. The GitHub repository is treated as part of the engineering product, not only as storage.

---

## 25. Documentation Decisions

### Decision

Separate documentation into focused files instead of putting everything in one long README.

### Reasoning

The root README gives the overview, while the `docs/` folder contains deeper engineering explanation.

Current documentation structure:

| File | Purpose |
|---|---|
| `docs/01-mechanical-design.md` | Mechanical structure, drivetrain, steering, chassis |
| `docs/02-power-and-sensor-architecture.md` | Power paths, sensors, wiring, calibration |
| `docs/03-software-architecture.md` | Jetson/STM32 software and protocol |
| `docs/04-obstacle-strategy.md` | Obstacle Challenge pillar behavior |
| `docs/05-systems-thinking-decisions.md` | Engineering decisions and trade-offs |
| `docs/06-testing-and-tuning.md` | Test runs, failures, tuning data |
| `docs/07-calibration-procedures.md` | Planned calibration procedures |
| `docs/08-build-flash-run-guide.md` | Planned build/run instructions |
| `docs/09-bill-of-materials.md` | Planned parts list |
| `docs/10-risk-register.md` | Planned risk and mitigation register |

### Final reasoning

This structure makes it easier for judges and future developers to find the right information quickly.

---

## 26. Decisions Still Requiring Final Data

Some decisions are accepted but still need final measured evidence.

| Area | Missing data |
|---|---|
| Track width | Final measurement |
| Final servo center | Final PWM/position value |
| Final steering limits | Left/right servo values |
| Final Open Challenge lap time | Best and average |
| Final Obstacle Challenge success rate | Test statistics |
| YOLO model version | YOLOv8/YOLO11/etc. |
| YOLO FPS | Jetson measured value |
| YOLO inference time | Jetson measured value |
| Dataset size | Number of images/classes |
| Final current draw | Motor, servo, Jetson, sensors |
| Final battery runtime | Measured under full load |
| Parking strategy | Final implementation and test results |

These values should be added after final testing.

---

## 27. What We Would Improve Next

If development time allows, the next system-level improvements are:

1. **Better test logging**  
   Save CSV logs for distance sensors, motor commands, YOLO detections, and state transitions.

2. **More measured tuning values**  
   Record future controller-gain changes, speed settings, and timing thresholds when those values are exported from the final source code.

3. **Optimized image/video repository size**  
   Keep high-resolution evidence but add compressed documentation versions.

4. **YOLO dataset documentation**  
   Add training images count, annotation method, training environment, and model version.

5. **Parking behavior evidence**  
   Add parking test videos and final parking strategy results.

6. **Formal calibration procedures**  
   Create repeatable calibration steps for sensors, steering, motor encoder, and camera.

7. **Automated run summary**  
   Generate lap reports from logs instead of writing all test data manually.

---

## 28. Conclusion

ARBIBOT was designed using an iterative systems-engineering process. The final design is not the result of selecting individual components in isolation. It is the result of observing how mechanics, power, sensors, communication, software, and strategy interact during real driving tests.

The most important system-level decisions were:

- splitting AI/navigation and real-time control between Jetson and STM32,
- using ToF sensors instead of ultrasonic sensors,
- using two right-side sensors for heading-aware wall following,
- using a front matrix sensor for corner/front-barrier detection,
- using YOLO for red/green pillar recognition,
- separating motor and electronics power domains,
- using a custom UART protocol,
- protecting the camera loop from blocking serial waits,
- pulsing critical commands,
- and documenting all important trade-offs in GitHub.

The project continues to improve through testing. Each failure is treated as a source of engineering information, and each design change is evaluated by whether it improves real robot behavior on the WRO field.
