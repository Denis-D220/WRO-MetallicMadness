# 06 - Testing and Tuning

**Project:** MetallicMadness / ARBIBOT  
**Category:** WRO 2026 Future Engineers - Self-Driving Cars  
**Document purpose:** Document the testing process used to improve ARBIBOT, including test runs, lap timing, success rate, failure data, tuning parameters, sensor validation, motor validation, obstacle behavior, and lessons learned.

> This document is part of the ARBIBOT engineering documentation package. It is focused on evidence: what was tested, what failed, what changed, and what still needs to be measured.

---

## 1. Purpose of Testing

Testing is the process used to convert ARBIBOT from a working prototype into a reliable competition robot. The robot combines mechanical steering, motor control, distance sensors, camera vision, YOLO inference, UART communication, and state-machine behavior. Because these subsystems interact, most failures only become visible during full driving tests.

The testing process has four main goals:

1. Verify that each subsystem works by itself.
2. Verify that subsystems work together during movement.
3. Measure performance using repeatable metrics.
4. Use failures to improve hardware, software, and tuning.

The most important engineering principle used during testing is:

```text
Do not only record that the robot failed.
Record why it failed, what changed, and whether the change improved the next run.
```

That evidence is what turns a crash into engineering. Without the evidence, it is just an expensive wall handshake.

---

## 2. Testing Categories

ARBIBOT testing is divided into the following categories:

| Category | Purpose |
|---|---|
| Mechanical tests | Check steering range, wheel traction, bumper clearance, drivetrain reliability |
| Power tests | Check runtime, voltage stability, current draw, brownout risk |
| Motor tests | Validate continuous movement, stop, speed, encoder, move-by-degrees |
| Sensor tests | Validate VL53L4CD, VL53L8CH, IMU, sensor addresses, calibration |
| Vision tests | Validate YOLO detection, FPS, confidence thresholds, lighting behavior |
| Open Challenge tests | Validate lane following, corner detection, lap completion |
| Obstacle Challenge tests | Validate red/green pillar detection and pass-side behavior |
| Integration tests | Validate full robot behavior with all systems running together |
| Failure analysis | Record root cause, fix, and retest result |

---

## 3. Main Performance Metrics

The following metrics should be used to evaluate progress.

| Metric | Description | Target / Notes |
|---|---|---|
| Best lap time | Fastest completed lap | [TODO] |
| Average lap time | Average of clean completed laps | [TODO] |
| Open Challenge success rate | Percentage of Open Challenge runs completing required behavior | [TODO] |
| Obstacle Challenge success rate | Percentage of Obstacle Challenge runs obeying red/green signs | [TODO] |
| Parking success rate | Percentage of parking attempts ending correctly | [TODO] |
| Corner detection distance | Distance at which corner turn is triggered | Current target around ~100 cm |
| Wall-following error | Difference from target wall distance | [TODO] |
| Heading error | Difference between right-front and right-rear sensor values | [TODO] |
| YOLO inference time | Time per frame for detection | [TODO] |
| YOLO FPS | Inference frames per second on Jetson | [TODO] |
| Serial command latency | Time for command/response or fire-and-forget execution | [TODO] |
| Stop reliability | Percentage of stop commands executed correctly | [TODO] |
| Battery runtime | Operating time under competition load | Estimated 20-30 min; measure final |

---

## 4. Current Known Progress Summary

The following summary reflects the current development state and should be updated after each major testing session.

| Area | Current status |
|---|---|
| Open Challenge | Single clean laps have been achieved repeatedly. One observed run reached approximately 2.5 of 3 laps before lane-keeping failure. |
| Corner trigger | Front-matrix corner trigger fires reliably at approximately 100 cm after filtering improvements. |
| Wall following | Improved after adding right-front minus right-rear heading correction. |
| Obstacle Challenge | YOLO pillar model detects `Green_Pillar` and `Red_Pillar`; PDI steering logic under tuning. |
| Green pillar issue | Recognition was affected when blocking motor commands slowed the vision loop; non-blocking motor command strategy improved responsiveness. |
| Serial reliability | Critical commands such as start/stop are pulsed several times to reduce dropped-frame risk. |
| Sensor reliability | Side sensor parser was updated for new STM32 firmware; invalid readings are handled more carefully. |
| Mechanical reliability | Bumper and sensor mounts were adjusted to avoid tire rubbing and improve sensor alignment. |

---

## 5. Test Run Log Template

Use this table for every meaningful test run. A run does not need to be successful to be useful. Failed runs are often the best data.

| Run ID | Date | Challenge | Software version | Battery voltage start/end | Speed setting | Result | Laps completed | Time | Main failure / note |
|---|---|---|---|---|---:|---|---:|---:|---|
| RUN-001 | [TODO] | Open | v1 | [TODO] | [TODO] | Failed | [TODO] | [TODO] | [TODO] |
| RUN-002 | [TODO] | Open | v2 | [TODO] | [TODO] | Partial | [TODO] | [TODO] | [TODO] |
| RUN-003 | [TODO] | Open | v4 | [TODO] | [TODO] | Partial | 2.5 | [TODO] | Lane-keeping failure |
| RUN-004 | [TODO] | Obstacle | main_challenge_02 | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |

Recommended result labels:

```text
Success
Partial
Failed
Aborted
Mechanical issue
Sensor issue
Vision issue
Software issue
Power issue
```

---

## 6. Version History and Testing Evolution

| Version | Description | Main test result | Decision |
|---|---|---|---|
| v1 - `main_challenge_01.py` | Camera corner detection + open-loop center steering | Basic motion and corner behavior possible, but not stable enough | Improved with sensor feedback |
| v2 - `main_challenge_01_v2.py` | Added PID wall-following and sensor/front-ToF fallbacks | Better lane behavior, but still required more reliable corner trigger | Continued development |
| v3 - `main_challenge_01_v3.py` | Two-right-sensor follow + new VL53L8 4x4 front matrix | Over-complex / unstable during early tests | Abandoned as main base |
| v4 - `main_challenge_01_v4.py` | v2 base + new firmware sensors + front-matrix corner trigger + RF-RR heading wall-follower | Current Open Challenge base; reached repeated clean laps and partial 3-lap run | Current Open Challenge version |
| `main_challenge_02.py` | Pillar YOLO + PDI pass-side steering on v4 scaffolding | Current Obstacle Challenge base | Under tuning |

---

## 7. Mechanical Testing

Mechanical testing focuses on whether the physical robot can execute the commands produced by the software.

### 7.1 Mechanical Test Checklist

| Test | Method | Expected result | Status |
|---|---|---|---|
| Steering center | Set servo to neutral | Wheels point straight | [TODO] |
| Maximum left steering | Command full left | No mechanical binding | [TODO] |
| Maximum right steering | Command full right | No mechanical binding | [TODO] |
| Bumper clearance | Turn wheels fully both directions | Tires do not rub bumper | Improved after redesign |
| Wheel traction | Drive on WRO mat | No slipping under normal acceleration | Improved after replacing thinner wheels |
| Drive axle torque transfer | Start/stop repeatedly | Rear wheels receive torque reliably | Improved after drivetrain change |
| Chassis rigidity | Push/twist gently by hand | No major flex at mounts | [TODO] |
| Sensor mount stability | Drive over test laps | Sensor angles do not change | [TODO] |

### 7.2 Mechanical Failure Data

| Failure | Observed behavior | Root cause | Fix |
|---|---|---|---|
| Thin wheels slipped on track | Robot did not follow expected path during turns | Low traction/contact area | Switched to wider rubber tires |
| First bumper rubbed tire | Steering movement was restricted | Bumper geometry too close to wheel path | Redesigned bumper and sensor placement |
| Sensor readings unreliable | Distance values did not match expected geometry | Sensors mounted too low | Repositioned sensors |
| Torque transfer issue | Rear drive was inconsistent | Gearbox gear to wheel gear transfer problem | Changed motor/drivetrain approach |
| Steering range/alignment issue | Turning radius and straight tracking inconsistent | Linkage rods and servo arm needed tuning | Adjusted rods, servo arm, and alignment screw |

---

## 8. Motor and Encoder Testing

Motor testing verifies that the STM32, MD10C, motor, and encoder behave correctly before full autonomous driving.

### 8.1 Motor Command Test Sequence

Recommended test sequence:

```text
1. SET SPEED 50%
2. FORWARD continuous for 5 seconds
3. STOP for 2 seconds
4. REVERSE continuous for 5 seconds
5. STOP for 2 seconds
6. FORWARD by degrees
7. STOP
8. REVERSE by degrees
9. STOP
```

### 8.2 Motor Test Log

| Test ID | Command | Expected result | Observed result | Status |
|---|---|---|---|---|
| MOT-01 | `0x0104 SET SPEED` | PWM speed updates | [TODO] | [TODO] |
| MOT-02 | `0x0004 FORWARD` | Motor moves forward continuously | [TODO] | [TODO] |
| MOT-03 | `0x0005 REVERSE` | Motor moves reverse continuously | [TODO] | [TODO] |
| MOT-04 | `0x0103 STOP` | Motor stops immediately | [TODO] | [TODO] |
| MOT-05 | `0x0101 FORWARD DEGREE` | Motor moves requested degrees | [TODO] | [TODO] |
| MOT-06 | `0x0102 REVERSE DEGREE` | Motor reverses requested degrees | [TODO] | [TODO] |
| MOT-07 | Encoder RPM read | RPM changes with motor speed | [TODO] | [TODO] |

### 8.3 Motor Issues Found

| Problem | Root cause | Fix |
|---|---|---|
| Forward by degree initially over-rotated | Degree conversion / command implementation mismatch | Compared Python command format with STM32 implementation and corrected command behavior |
| Motor did not move with non-blocking command in one version | Fire-and-forget command could be dropped or not sequenced correctly | Pulse critical forward/speed commands and send forward before/interleaved with speed |
| Stop sometimes missed | Serial bus busy during matrix scan | Pulse stop command several times |
| Set-speed affected idle direction | Motor driver command sequencing issue | Send forward first, then speed, and repeat critical commands |

---

## 9. Sensor Testing

Sensor testing verifies distance accuracy, address configuration, XSHUT control, and sensor response stability.

### 9.1 Distance Calibration Method

The VL53 sensors are tested at fixed distances:

```text
120 cm, 110 cm, 100 cm, 90 cm, 80 cm, 70 cm,
60 cm, 50 cm, 40 cm, 30 cm, 20 cm, 10 cm
```

For each distance, the robot is moved to the reference location and the Python sensor test program reads all sensors.

### 9.2 Calibration Data Table

| Real distance | Left VL53L4CD | Right-front VL53L4CD | Right-rear VL53L4CD | Front VL53L8CH | Error / notes |
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

### 9.3 Sensor Issues Found

| Problem | Root cause | Fix |
|---|---|---|
| Side sensors returned `-1` | Parser mismatch with new firmware or serial thread starvation | Updated parser and reduced blocking behavior |
| Front matrix showed false close distance | Matrix row included floor cells | Excluded floor-band cells and used a better row selection |
| Turn triggered too early | Lone matrix cell detected corner/wall edge | Required at least two in-band cells before firing |
| Sensor address conflicts possible | Multiple VL53L4CD sensors on one I2C bus | Used XSHUT pins for controlled sensor initialization |
| Sensor values unstable when mounted low | Poor geometry / reading floor or bumper | Repositioned sensor mounts |

---

## 10. Front Matrix Corner Trigger Testing

The front VL53L8CH matrix is used to support corner detection.

### 10.1 Original Problem

The front matrix initially caused two opposite failures:

| Failure | Cause |
|---|---|
| Turned too late | Floor cells in the matrix dragged the calculated median down and hid the true front wall |
| Turned too soon | A single outer-wall corner cell was treated as if it were the front barrier |

### 10.2 Fix

The improved logic:

```text
1. Select the matrix row that best represents the front barrier.
2. Exclude cells likely to see the floor.
3. Accept only values inside a valid distance band.
4. Require at least two valid in-band cells.
5. Fire the corner trigger around the expected distance window.
```

### 10.3 Test Evidence Template

| Test ID | Approach speed | Trigger distance | Cells valid | Result | Notes |
|---|---:|---:|---:|---|---|
| FMT-01 | [TODO] | ~100 cm | [TODO] | Success | [TODO] |
| FMT-02 | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| FMT-03 | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |

---

## 11. Lane-Following Tuning

Lane-following uses the right-front and right-rear VL53L4CD sensors.

### 11.1 Initial Problem

A distance-only controller caused the robot to wander wall-to-wall. The robot knew how far it was from the wall, but it did not know whether it was angled toward or away from the wall.

### 11.2 Improvement

The controller was improved by using the difference between right-front and right-rear readings:

```text
heading_error = right_front_distance - right_rear_distance
```

This heading term helps the robot stay parallel to the wall.

### 11.3 Lane Controller Template

```text
distance_error = target_right_distance - average(right_front, right_rear)
heading_error = right_front - right_rear

steering = Kd * distance_error + Kh * heading_error
```

### 11.4 Tuning Table

| Parameter | Value | Effect |
|---|---:|---|
| Target right distance | [TODO] | Desired distance from right wall |
| Distance gain | [TODO] | Corrects lateral offset |
| Heading gain | [TODO] | Corrects angle relative to wall |
| Steering center | [TODO] | Servo neutral |
| Steering clamp | [TODO] | Maximum correction |
| Straight speed | [TODO] | Motor speed during straight sections |

### 11.5 Lane Test Results

| Test ID | Target distance | Speed | Result | Notes |
|---|---:|---:|---|---|
| LANE-01 | [TODO] | [TODO] | Wandering | Distance-only PID |
| LANE-02 | [TODO] | [TODO] | Improved | Added RF-RR heading |
| LANE-03 | [TODO] | [TODO] | [TODO] | [TODO] |

---

## 12. Vision and YOLO Testing

Vision testing checks whether the Jetson can detect pillars and track lines with enough speed and reliability.

### 12.1 Pillar Detection Test

| Test ID | Object | Lighting | Distance | Confidence | Result |
|---|---|---|---:|---:|---|
| VIS-01 | Red pillar | [TODO] | [TODO] | [TODO] | [TODO] |
| VIS-02 | Green pillar | [TODO] | [TODO] | [TODO] | [TODO] |
| VIS-03 | Multiple pillars | [TODO] | [TODO] | [TODO] | [TODO] |
| VIS-04 | Partial occlusion | [TODO] | [TODO] | [TODO] | [TODO] |
| VIS-05 | Motion blur | [TODO] | [TODO] | [TODO] | [TODO] |

### 12.2 Vision Metrics

| Metric | Current / target value |
|---|---:|
| Pillar confidence threshold | 0.30 |
| Corner line confidence threshold | 0.48 |
| YOLO version | [TODO] |
| Pillar model dataset size | [TODO] |
| Line/corner model dataset size | [TODO] |
| Inference FPS on Jetson | [TODO] |
| Inference time per frame | [TODO] |
| False positive rate | [TODO] |
| False negative rate | [TODO] |

### 12.3 Vision Issues Found

| Problem | Root cause | Fix |
|---|---|---|
| Green pillar recognition failed when motor command waited for response | Vision loop was delayed by blocking serial behavior | Used non-blocking/fire-and-forget motor commands where safe |
| Detection unstable under changing lighting | Camera exposure/color differences | Added per-camera color correction |
| Late pillar reaction caused hard steering | Object was acted on only when too close | React earlier and soften PDI gains |
| Target lost during aggressive steering | Full-lock turn moved pillar out of view | Clamp steering and reduce gains |

---

## 13. Obstacle Strategy Testing

Obstacle testing validates red/green rule behavior.

### 13.1 Obstacle Test Matrix

| Test ID | Scenario | Expected behavior | Result | Notes |
|---|---|---|---|---|
| OBS-01 | Single red pillar centered | Pass red on right | [TODO] |  |
| OBS-02 | Single green pillar centered | Pass green on left | [TODO] |  |
| OBS-03 | Red pillar near left side | Pass right without wall hit | [TODO] |  |
| OBS-04 | Green pillar near right side | Pass left without wall hit | [TODO] |  |
| OBS-05 | Two pillars visible | Choose nearest/largest valid pillar | [TODO] |  |
| OBS-06 | Pillar near corner | Avoid command conflict with cornering | [TODO] |  |
| OBS-07 | Lost detection for one frame | Hold last valid detection briefly | [TODO] |  |
| OBS-08 | Detection flicker | Debounce prevents repeated reactions | [TODO] |  |
| OBS-09 | High steering error | Clamp steering and reduce speed | [TODO] |  |
| OBS-10 | Full Obstacle Challenge run | Complete laps while obeying signs | [TODO] |  |

### 13.2 PDI Tuning Log

| Test ID | Kp | Ki | Kd | Speed | Steering clamp | Result |
|---|---:|---:|---:|---:|---:|---|
| PDI-01 | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | Full-lock / too aggressive |
| PDI-02 | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | Softer response |
| PDI-03 | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |

---

## 14. Open Challenge Testing

Open Challenge testing focuses on three-lap completion without pillars.

### 14.1 Open Challenge Run Log

| Run ID | Software | Speed | Laps completed | Time | Result | Failure / notes |
|---|---|---:|---:|---:|---|---|
| OPEN-01 | v1 | [TODO] | [TODO] | [TODO] | Failed | Early corner logic |
| OPEN-02 | v2 | [TODO] | [TODO] | [TODO] | Partial | PID fallback under tuning |
| OPEN-03 | v3 | [TODO] | [TODO] | [TODO] | Failed/unstable | Over-complex matrix behavior |
| OPEN-04 | v4 | [TODO] | 1.0+ | [TODO] | Partial/successful lap | Single clean laps achieved repeatedly |
| OPEN-05 | v4 | [TODO] | 2.5 | [TODO] | Partial | Lane-keeping failure before full 3 laps |

### 14.2 Open Challenge Failure Modes

| Failure | Cause | Fix / planned fix |
|---|---|---|
| Turn too late | Front barrier detection unreliable | Improved front matrix filtering |
| Turn too early | One-cell false trigger | Require multiple valid matrix cells |
| Wall-to-wall wandering | Distance-only lane controller | Add RF-RR heading correction |
| Missed stop / delayed stop | Serial bus busy | Pulse stop command |
| Overcorrection after turn | Steering returns too sharply | Add cooldown and smoother recovery |

---

## 15. Parking Testing

Parking testing is not yet fully documented, but it should be included because the Obstacle Challenge includes parking after the laps.

### 15.1 Parking Test Plan

| Test ID | Scenario | Expected behavior | Result |
|---|---|---|---|
| PARK-01 | Detect parking section | Robot identifies correct area | [TODO] |
| PARK-02 | Align parallel to wall | Side distances show parallel position | [TODO] |
| PARK-03 | Enter parking lot partially | Robot enters without touching markers | [TODO] |
| PARK-04 | Full parallel park | Robot stops fully inside and parallel | [TODO] |
| PARK-05 | Marker avoidance | Robot avoids magenta parking limits | [TODO] |

### 15.2 Parking Metrics

| Metric | Value |
|---|---:|
| Parking success rate | [TODO] |
| Average parking time | [TODO] |
| Final wall distance front/rear | [TODO] |
| Parallel error | [TODO] |
| Marker contact incidents | [TODO] |

---

## 16. Power and Runtime Testing

Power testing verifies that the robot can run a full challenge without brownouts, resets, or unstable behavior.

### 16.1 Runtime Test

| Test ID | Start voltage | End voltage | Duration | Load condition | Result |
|---|---:|---:|---:|---|---|
| PWR-01 | [TODO] | [TODO] | 3 min | Competition run | [TODO] |
| PWR-02 | [TODO] | [TODO] | 10 min | Continuous driving | [TODO] |
| PWR-03 | [TODO] | [TODO] | 20-30 min | Full electronics active | Estimated target | [TODO] |

### 16.2 Current Draw Measurements

| Subsystem | Idle current | Typical current | Peak current | Notes |
|---|---:|---:|---:|---|
| Jetson Orin Nano | [TODO] | [TODO] | [TODO] | Measure during YOLO |
| STM32F411 | ~10 mA estimate | [TODO] | [TODO] | Measure final |
| MG996R servo | ~10 mA idle | 500-900 mA normal | 1.5-2.5 A stall | Estimate |
| JGY-370B motor | 0.06-0.09 A no-load | 0.2-0.3 A normal | 1.3-2.0 A stall | Estimate |
| VL53L4CD sensors | [TODO] | 15-25 mA each | [TODO] | Estimate |
| VL53L8CH | [TODO] | [TODO] | [TODO] | Measure/spec needed |

### 16.3 Power Issues to Watch

| Issue | Symptom | Mitigation |
|---|---|---|
| Jetson brownout | Jetson reboots or camera drops | Separate UPS power path |
| Servo voltage dip | Steering jitter or reset | Verify UPS/regulator current margin |
| Motor noise | Sensor/UART errors | Separate motor power and route wires carefully |
| Low battery | Reduced speed or unstable behavior | Monitor voltage before competition run |

---

## 17. Failure Analysis Log

The following table records known failures and corrections.

| Failure ID | Failure | Root cause | Fix applied | Retest status |
|---|---|---|---|---|
| FAIL-001 | Turned too late and hit front wall | Front matrix included floor cells; barrier read incorrectly | Excluded floor-band cells and improved row selection | Improved |
| FAIL-002 | Turned too soon | Single matrix cell falsely detected barrier | Required at least two valid cells | Improved |
| FAIL-003 | Car would not start or ran reverse | Command frame dropped / set-speed sequencing issue | Send forward first, interleave speed, pulse commands | Improved |
| FAIL-004 | Stop command missed | Serial busy during matrix scan | Pulse STOP command | Improved |
| FAIL-005 | Side sensors read `-1` | Parser mismatch / serial starvation | Updated parser and reduced blocking | Improved |
| FAIL-006 | Lane wandered wall-to-wall | Distance-only PID lacked heading awareness | Added RF-RR heading term | Improved |
| FAIL-007 | Pillar steering full-lock | PDI gains too aggressive | Reduced gains and steering clamp | Improved |
| FAIL-008 | Green pillar recognition failed during motor command | Vision loop blocked by serial wait | Non-blocking motor command strategy | Improved |
| FAIL-009 | Bumper rubbed tire | Front bumper geometry interfered with steering | Redesigned bumper/sensor mount | Fixed |
| FAIL-010 | Thin wheels slipped | Insufficient traction | Replaced with wider wheels | Fixed |

---

## 18. Tuning Parameter Register

Keep this table updated with final values.

| Parameter | Current value | Test basis | Notes |
|---|---:|---|---|
| Straight speed | [TODO] | Open Challenge runs |  |
| Corner speed | [TODO] | Corner tests |  |
| Obstacle speed | [TODO] | Pillar tests |  |
| Steering center | [TODO] | Servo alignment |  |
| Max left steering | [TODO] | Mechanical range |  |
| Max right steering | [TODO] | Mechanical range |  |
| Target right wall distance | [TODO] | Lane tests |  |
| Distance gain | [TODO] | Lane tests |  |
| Heading gain | [TODO] | Lane tests |  |
| Corner trigger distance | ~100 cm | Front matrix tests | Confirm final |
| Front matrix valid-cell count | >=2 | False trigger tests |  |
| Pillar confidence threshold | 0.30 | Vision tests |  |
| Corner confidence threshold | 0.48 | Vision tests |  |
| PDI Kp | [TODO] | Obstacle tests |  |
| PDI Ki | [TODO] | Obstacle tests |  |
| PDI Kd | [TODO] | Obstacle tests |  |
| Lost target timeout | [TODO] | Obstacle tests |  |
| Debounce frames | [TODO] | Pillar counter tests |  |

---

## 19. Success Rate Calculation

Success rate should be calculated consistently.

```text
success_rate = successful_runs / total_runs * 100%
```

Recommended definition:

| Challenge | Successful run means |
|---|---|
| Open Challenge | Robot completes the target laps without moving walls and stops/end behavior is valid |
| Obstacle Challenge | Robot completes target laps while passing red/green pillars on correct sides and avoiding invalid pillar movement |
| Parking | Robot stops inside or partly inside the parking area without touching limitations, depending on scoring target |

### 19.1 Success Rate Table

| Test group | Total runs | Successful runs | Partial runs | Failed runs | Success rate |
|---|---:|---:|---:|---:|---:|
| Open Challenge | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| Obstacle Challenge | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| Parking | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| Motor command tests | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| Sensor calibration tests | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |

---

## 20. Recommended Test Data Files

The repository should include machine-readable test logs so the team can analyze progress.

Recommended paths:

```text
other/testing/open_challenge_runs.csv
other/testing/obstacle_challenge_runs.csv
other/testing/parking_tests.csv
other/testing/sensor_calibration.csv
other/testing/motor_command_tests.csv
other/testing/vision_inference_tests.csv
other/testing/failure_log.csv
```

### 20.1 Suggested CSV Columns

#### `open_challenge_runs.csv`

```text
run_id,date,software_version,speed,start_voltage,end_voltage,laps_completed,time_seconds,result,failure_note
```

#### `obstacle_challenge_runs.csv`

```text
run_id,date,software_version,speed,pillars_seen,pillars_correct,pillars_missed,laps_completed,time_seconds,result,failure_note
```

#### `sensor_calibration.csv`

```text
sensor_name,real_distance_cm,measured_distance_mm,error_mm,status,notes
```

#### `vision_inference_tests.csv`

```text
test_id,model,frame_resolution,inference_ms,fps,confidence_threshold,true_positive,false_positive,false_negative,notes
```

---

## 21. Evidence to Add Before Final Submission

The following evidence should be added to make the testing documentation stronger:

1. Video of at least one clean Open Challenge lap.
2. Video of Obstacle Challenge red/green pass behavior.
3. Screenshot of YOLO detecting red and green pillars.
4. Plot or table of sensor calibration results.
5. Plot or table of lap time improvement across versions.
6. Final tuning parameter table.
7. Final success rate table.
8. Battery voltage before/after a 3-minute run.
9. Current draw during motor, servo, and YOLO operation.
10. Failure screenshots or photos where useful.

---

## 22. Lessons Learned

The main lessons from testing so far are:

1. **A single side distance sensor is not enough for stable wall following.**  
   The robot needs heading information, so the right-front and right-rear sensor difference is valuable.

2. **Front matrix sensors need careful filtering.**  
   More data is not automatically better. Matrix cells can see the floor, corners, or irrelevant surfaces.

3. **Vision must not be blocked by motor communication.**  
   Blocking serial waits can make the robot miss pillars, especially green pillar detection during movement.

4. **Critical commands should be repeated.**  
   In a moving robot with a shared serial bus, repeated STOP/FORWARD commands are safer than assuming one frame always arrives.

5. **Mechanical problems become software problems.**  
   Tire rubbing, wheel slip, and poor torque transfer all appear as control failures unless they are fixed mechanically.

6. **Tuning must be recorded.**  
   Changing gains without recording results is how teams accidentally rediscover yesterday's mistake tomorrow.

---

## 23. Current Priorities

The next test priorities are:

| Priority | Task | Reason |
|---|---|---|
| 1 | Complete full Open Challenge 3-lap run | Validate v4 strategy |
| 2 | Record final lane-following gains | Make behavior reproducible |
| 3 | Measure YOLO FPS and inference time | Validate Jetson performance |
| 4 | Collect red/green obstacle test data | Validate Obstacle Challenge strategy |
| 5 | Measure current draw and battery voltage drop | Validate power budget |
| 6 | Add parking test data | Complete Obstacle Challenge evidence |
| 7 | Add videos and screenshots | Strengthen documentation for judges |

---

## 24. Conclusion

Testing and tuning are central to ARBIBOT's development. The robot has already improved through several design iterations: the drivetrain was changed after torque-transfer issues, thinner wheels were replaced after slipping, the bumper was redesigned after tire interference, the side sensor parser was updated, front matrix filtering was improved, and wall-following was upgraded with a heading term using right-front and right-rear distance sensors.

The strongest testing-based improvements so far are:

- replacing unreliable drivetrain and wheel choices,
- using ToF sensors instead of ultrasonic sensors,
- filtering the VL53L8CH front matrix to avoid false corner triggers,
- adding right-front/right-rear heading correction,
- reducing blocking serial waits to protect YOLO detection timing,
- pulsing critical motor commands,
- and softening obstacle PDI steering to prevent full-lock behavior.

The remaining work is to convert observed progress into measured data: lap times, success rates, current draw, inference FPS, calibration tables, and final tuning values. Once those values are recorded, this document will provide strong evidence that ARBIBOT was improved through a real engineering test-and-iteration process.
