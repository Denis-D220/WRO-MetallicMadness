# Docs

This folder contains the main engineering documentation for the **MetallicMadness / ARBIBOT** WRO Future Engineers self-driving car project.

The documents explain the robot design in a structured way: mechanical design, power architecture, sensors, software, obstacle strategy, engineering decisions, testing, calibration, bill of materials, and risk management.

These documents complement the root `README.md`, the Engineering Journal, the diagrams in `schemes/`, the robot photos in `v-photos/`, the trained models in `Models/`, and the challenge videos in `video/`.

---

## Folder Contents

| File | Description | Current status |
|---|---|---|
| `01-mechanical-design.md` | Mechanical design documentation, including chassis, dimensions, drivetrain, steering system, motor selection, wheels, bumper design, mechanical iterations, and known mechanical failure modes. | Updated with current track width and mechanical measurements. |
| `02-power-and-sensor-architecture.md` | Power and sensor architecture, including motor power, Jetson/electronics power, BMS, UPS, current estimates, grounding, VL53 sensor placement, I2C wiring, XSHUT control, and sensor calibration notes. | Updated with current two-battery power design and confirmed sensor address map. |
| `03-software-architecture.md` | Software architecture for Jetson and STM32, including Python modules, STM32 firmware responsibilities, DD-UART command protocol, state machine, sensor processing, motor control, and challenge control flow. | Updated with YOLO11n, current model classes, dataset sizes, and runtime notes. |
| `04-obstacle-strategy.md` | Obstacle Challenge strategy, including red/green pillar behavior, YOLO detection pipeline, nearest-pillar selection, PDI steering control, lane recovery, edge cases, and tuning notes. | Updated with current obstacle result and confirmed model classes. |
| `05-systems-thinking-decisions.md` | Engineering decision record explaining why major design choices were made, what alternatives were rejected, and how failures changed the robot design. | Updated with final mechanical values and current system decisions. |
| `06-testing-and-tuning.md` | Testing and tuning documentation, including test-run logs, lap-time templates, success-rate calculation, failure analysis, motor tests, sensor tests, vision tests, obstacle tests, and tuning parameters. | Updated with current lap times, success rates, YOLO metrics, and known test results. |
| `07-calibration-procedures.md` | Calibration procedures for camera alignment, YOLO confidence, screen zones, ToF thresholds, front matrix trigger, servo center, steering limits, motor speed, encoder validation, and battery logging. | Updated with known calibration values; detailed raw calibration logs remain future measurement records. |
| `09-bill-of-materials.md` | Bill of Materials documenting the main mechanical, electrical, power, compute, sensor, motor, and communication components, including function, voltage, estimated current, and interface. | Updated with current component currents, power domains, and track width. |
| `10-risk-register.md` | Risk register and failure-mode documentation, including glare, bad pillar detection, sensor noise, UART timeout, motor blocking, weak battery, stop-command failure, wheel slip, steering misalignment, and parking risks. | Updated with current sensor address-map mitigation and power-risk status. |

---

## Recommended Reading Order

```text
01-mechanical-design.md
02-power-and-sensor-architecture.md
03-software-architecture.md
04-obstacle-strategy.md
05-systems-thinking-decisions.md
06-testing-and-tuning.md
07-calibration-procedures.md
09-bill-of-materials.md
10-risk-register.md
```

---

## Current Confirmed Project Values

| Topic | Current value |
|---|---|
| Vehicle length | 25.0 cm |
| Vehicle width | Approximately 16.0 cm outside wheel-to-wheel |
| Vehicle height | 18.0 cm |
| Vehicle weight | 1.35 kg |
| Wheelbase | 13.9 cm |
| Track width | 13.5 cm center-to-center |
| Wheel width | 2.5 cm |
| Drive system | Rear-wheel drive using JGY-370B 12 V worm gear motor with encoder |
| Steering | MG996R servo with pushrod/tie-rod front steering linkage |
| Main compute | NVIDIA Jetson Orin Nano |
| Low-level controller | STM32F411 Black Pill |
| Vision models | Ultralytics YOLO11n |
| Pillar classes | `Green_Pillar`, `Red_Pillar` |
| Corner/line classes | `Blue_line`, `Orange_line` |
| Pillar dataset | 100 images |
| Corner/line dataset | 180 images |
| Jetson YOLO runtime | Approximately 60 FPS |
| Best Open Challenge time | 1:05 / 65 seconds |
| Average Open Challenge time | 1:10 / 70 seconds |
| Open Challenge success rate | 80% |
| Obstacle Challenge success rate | 50% |
| Parking success rate | N/A |
| Front matrix address | VL53L8CX on I2C1 at 0x52 8-bit / 0x29 7-bit |
| Side sensor addresses | VL53L4CD sensors on I2C2: LEFT 0x52 PA5, RIGHT_FRONT 0x54 PA7, RIGHT_REAR 0x56 PB14 |

---

## Related Folders

| Folder | Purpose |
|---|---|
| `../schemes/` | Architecture, full wiring, power distribution, and sensor placement diagrams. |
| `../v-photos/` | Real robot photos, including front/back/left/right/top/bottom views, front bumper, motor, and steering images. |
| `../t-photos/` | Team photos. |
| `../video/` | Challenge videos and test-run evidence. |
| `../Models/` | YOLO11n model checkpoints and model documentation. |
| `../other/protocol/` | DD-UART protocol documentation and protocol reference files. |
| `../src/` | Jetson Python code and STM32-related source files. |
| `../engineering-journal/` | Main engineering journal in Markdown/PDF form. |

---

## Important Related Files

| File | Relationship |
|---|---|
| `../README.md` | Main project overview. |
| `../engineering-journal/WRO_Engineering_Journal_MetallicMadness.md` | Main Engineering Journal. |
| `../engineering-journal/WRO_Engineering_Journal_MetallicMadness.pdf` | PDF version of the Engineering Journal. |
| `../schemes/arbibot_2026_electronics_and_control_architecture.png` | High-level electronics and control architecture diagram. |
| `../schemes/full-wiring-diagram.png` | Complete wiring reference. |
| `../schemes/power-distribution-diagram.png` | Power-domain reference. |
| `../schemes/sensor-placement-diagram.png` | Physical sensor layout reference. |
| `../other/protocol/dd-uart-protocol.md` | Detailed serial protocol used between Jetson and STM32. |
| `../video/challenge01.mp4` | Open Challenge video evidence. |

---

## Documentation Status

The documentation set is currently updated with the latest confirmed ARBIBOT information. Some raw calibration/test tables intentionally remain as future measurement records where detailed numeric logs were not captured. These are not blocking documentation items; they are evidence-improvement opportunities.

| Document | Status |
|---|---|
| `01-mechanical-design.md` | Current technical draft updated. |
| `02-power-and-sensor-architecture.md` | Current technical draft updated. |
| `03-software-architecture.md` | Current technical draft updated. |
| `04-obstacle-strategy.md` | Current technical draft updated. |
| `05-systems-thinking-decisions.md` | Current technical draft updated. |
| `06-testing-and-tuning.md` | Current technical draft updated with available metrics. |
| `07-calibration-procedures.md` | Current calibration guide updated with known values. |
| `09-bill-of-materials.md` | Current BOM updated. |
| `10-risk-register.md` | Current risk register updated. |

---

## Update Policy

When the robot changes, update the documentation that is affected.

| Change | Documents to update |
|---|---|
| Steering linkage changed | `01-mechanical-design.md`, `07-calibration-procedures.md`, `10-risk-register.md` |
| Battery or power wiring changed | `02-power-and-sensor-architecture.md`, `09-bill-of-materials.md`, `10-risk-register.md` |
| Sensor position changed | `02-power-and-sensor-architecture.md`, `07-calibration-procedures.md`, `schemes/sensor-placement-diagram.png` |
| YOLO model changed | `03-software-architecture.md`, `04-obstacle-strategy.md`, `07-calibration-procedures.md`, `../Models/README.md` |
| Motor command protocol changed | `03-software-architecture.md`, `../other/protocol/dd-uart-protocol.md`, `10-risk-register.md` |
| New test result recorded | `06-testing-and-tuning.md` |
| New failure discovered | `10-risk-register.md`, `06-testing-and-tuning.md` |

---

## Commit Recommendation

```bash
git add docs/*.md

git commit -m "docs: refresh engineering documentation with current robot data"

git push
```
