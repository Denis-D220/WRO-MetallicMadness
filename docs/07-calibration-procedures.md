# 07 - Calibration Procedures

**Project:** MetallicMadness / ARBIBOT  
**Category:** WRO 2026 Future Engineers - Self-Driving Cars  
**Document purpose:** Define repeatable calibration procedures for camera setup, YOLO confidence thresholds, VL53 ToF thresholds, servo center, steering limits, and motor speed.

> This document should be used before major test sessions and before competition runs. Calibration values must be updated whenever the camera, bumper, sensors, steering linkage, wheels, motor, or power system changes.

---

## 1. Calibration Philosophy

ARBIBOT depends on several subsystems working together:

- camera vision,
- YOLO inference,
- ToF distance sensing,
- steering servo control,
- motor speed control,
- encoder feedback,
- Jetson-to-STM32 serial communication,
- and challenge state-machine thresholds.

A small mechanical or electrical change can affect software behavior. For example:

- moving a VL53 sensor changes wall-following thresholds,
- changing camera angle changes pillar detection position,
- changing steering linkage changes servo center and steering limits,
- changing motor speed changes required reaction distance,
- changing battery voltage can change motor response and servo behavior.

The goal of calibration is to produce repeatable behavior, not only one lucky successful run.

---

## 2. Calibration Order

Calibration should be performed in this order:

```text
1. Mechanical inspection
2. Power verification
3. Servo center and steering limits
4. Motor direction and speed
5. Encoder validation
6. ToF sensor initialization and distance calibration
7. Front matrix threshold calibration
8. Camera alignment
9. YOLO confidence threshold tuning
10. Lane-following tuning
11. Obstacle PDI tuning
12. Full challenge validation
```

This order matters. Do not tune YOLO steering behavior before the steering center is correct. Do not tune wall-following thresholds before the sensors are physically stable. Do not tune motor speed before battery voltage is reliable.

---

## 3. Calibration Files and Parameters

The following software files contain values that are affected by calibration.

| File | Calibration role |
|---|---|
| `src/main_challenge_01_v4.py` | Open Challenge state machine, corner behavior, speed settings |
| `src/main_challenge_02.py` | Obstacle Challenge behavior, pillar pass-side logic, PDI tuning |
| `src/sensor_distance_v3.py` | ToF parsing, front matrix processing, distance thresholds |
| `src/Pillar_recognition.py` | Pillar detection handling, confidence threshold, screen zones |
| `src/pillar_counter.py` | Pillar debounce and counting logic |
| `src/pid_line_follower.py` | Lane-following PID/PDI gains |
| `src/motor_driver.py` | Motor command timing, speed commands, fire-and-forget behavior |
| `src/servo_controller.py` | Servo center, steering limits, steering command mapping |
| `src/color_tuning.py` | Camera color correction and image tuning |
| STM32 firmware | Sensor initialization, I2C addresses, XSHUT, motor PWM, encoder counts |

---

## 4. Calibration Master Table

Keep the final calibration values here.

| Parameter | Current value | Final value | Notes |
|---|---:|---:|---|
| Camera resolution | 1280x720 | 1280x720 | Jetson GStreamer/OpenCV capture |
| Camera FPS | Approximately 60 FPS pipeline observation | Approximately 60 FPS | YOLO11n runtime observation on Jetson |
| Camera exposure | Auto | Auto / tune if field lighting requires it | Lock only if lighting instability appears |
| Pillar YOLO confidence | 0.30 | 0.30 | Current working value |
| Corner line confidence | 0.48 | 0.48 | Current working value |
| Front matrix trigger distance | ~100 cm | ~100 cm | Current working trigger distance |
| Front matrix valid-cell requirement | >= 2 cells | >= 2 cells | Used to reduce false triggers |
| Left VL53L4CD offset | Not recorded yet | Not recorded yet | mm correction if needed |
| Right-front VL53L4CD offset | Not recorded yet | Not recorded yet | mm correction if needed |
| Right-rear VL53L4CD offset | Not recorded yet | Not recorded yet | mm correction if needed |
| Target right wall distance | Not recorded yet | Not recorded yet | Used by lane controller |
| Servo center | Not recorded yet | Not recorded yet | Neutral straight wheels |
| Servo max left | Not recorded yet | Not recorded yet | Avoid mechanical binding |
| Servo max right | Not recorded yet | Not recorded yet | Avoid mechanical binding |
| Normal motor speed | Not recorded yet | Not recorded yet | Open Challenge straight speed |
| Corner motor speed | Not recorded yet | Not recorded yet | Slower corner approach if needed |
| Obstacle motor speed | Not recorded yet | Not recorded yet | Safer speed near pillars |
| Motor PWM min moving | Not recorded yet | Not recorded yet | Lowest command that moves reliably |
| Motor PWM max safe | Not recorded yet | Not recorded yet | Maximum stable competition speed |
| Encoder ticks per wheel revolution | Not recorded yet | Not recorded yet | Depends on motor and gearbox |
| Wheel diameter | Not recorded yet | Not recorded yet | Needed for distance calculation |
| Battery voltage start | Not recorded yet | Not recorded yet | Before run |
| Battery voltage end | Not recorded yet | Not recorded yet | After 3-minute run |

---

## 5. Required Tools

Recommended tools for calibration:

| Tool | Use |
|---|---|
| Measuring tape or ruler | ToF distance calibration and robot geometry |
| Printed WRO field / test wall | Wall-following and corner testing |
| Laptop connected to Jetson | Run Python scripts and view logs |
| STM32 serial test script | Motor and sensor command validation |
| Multimeter | Battery voltage and power checks |
| Phone camera or external camera | Record calibration runs |
| Stopwatch | Lap timing and motor movement timing |
| Small object / test pillar | YOLO and obstacle behavior testing |
| Marker tape | Mark sensor distances and trigger points |
| Notebook or CSV log | Record final values |

---

## 6. Safety Before Calibration

Before any calibration where the motor can move:

1. Lift the robot so drive wheels do not touch the floor.
2. Verify motor direction.
3. Test STOP command.
4. Keep physical power disconnect available.
5. Do not place fingers near steering linkage or wheels.
6. Do not run the robot with loose wires.
7. Do not calibrate with low battery.

Minimum safety test:

```text
FORWARD 1 second -> STOP
REVERSE 1 second -> STOP
STEER LEFT -> CENTER -> RIGHT -> CENTER
```

---

## 7. Mechanical Pre-Calibration

Software calibration should not begin until the mechanical system is stable.

### 7.1 Mechanical Checklist

| Check | Expected result | Status |
|---|---|---|
| Front wheels straight at servo center | Wheels point forward | Not recorded yet |
| Steering left | No tire rubbing | Not recorded yet |
| Steering right | No tire rubbing | Not recorded yet |
| Front bumper clearance | Bumper does not touch tire | Not recorded yet |
| Sensor mount angle | Sensor faces intended direction | Not recorded yet |
| Camera mount | Camera fixed, no vibration | Not recorded yet |
| Rear wheels | Wheels fixed and not slipping on shaft | Not recorded yet |
| Wires | No cable touches wheels or steering | Not recorded yet |
| Chassis | No loose part | Not recorded yet |

### 7.2 Mechanical Calibration Notes

Record changes here:

| Date | Mechanical change | Calibration affected |
|---|---|---|
| Not recorded yet | Front bumper changed | ToF thresholds, sensor calibration |
| Not recorded yet | Steering linkage adjusted | Servo center, steering limits |
| Not recorded yet | Camera angle changed | YOLO zones, confidence, line/corner model |
| Not recorded yet | Wheel type changed | Motor speed, lane tuning, encoder distance |

---

## 8. Servo Center Calibration

The steering servo must be calibrated before lane-following or obstacle steering.

### 8.1 Goal

Find the servo value that makes the front wheels physically straight.

This value is called:

```text
SERVO_CENTER
```

### 8.2 Procedure

1. Lift the front of the robot.
2. Power the servo controller.
3. Send the current center value.
4. Place the robot on a flat surface.
5. Look from the front and top.
6. Adjust the servo value until both front wheels point straight.
7. Roll the robot forward by hand and confirm it does not drift strongly.
8. Record the final value.
9. Mark the servo horn position physically if useful.

### 8.3 Servo Center Test Table

| Test | Servo value | Observed behavior | Decision |
|---|---:|---|---|
| SC-01 | Not recorded yet | Wheels slightly left | Increase/decrease value |
| SC-02 | Not recorded yet | Wheels slightly right | Increase/decrease value |
| SC-03 | Not recorded yet | Wheels straight | Candidate center |
| SC-04 | Not recorded yet | Robot rolls straight | Final center |

### 8.4 Final Value

```text
SERVO_CENTER = Not recorded yet
```

### 8.5 Notes

Do not fix steering drift only in software if the mechanical center is wrong. First correct the servo center and linkage. Then tune the lane controller.

---

## 9. Steering Limit Calibration

The servo must not command steering beyond the mechanical safe range.

### 9.1 Goal

Find safe left and right steering limits:

```text
SERVO_LEFT_LIMIT
SERVO_RIGHT_LIMIT
```

### 9.2 Procedure

1. Lift the front wheels.
2. Command center.
3. Slowly increase steering left.
4. Stop before the linkage binds or tire touches bumper/chassis.
5. Record the safe left limit.
6. Return to center.
7. Slowly increase steering right.
8. Stop before the linkage binds or tire touches bumper/chassis.
9. Record the safe right limit.
10. Test both limits on the floor at low speed.

### 9.3 Steering Limit Table

| Test | Direction | Servo value | Result | Safe? |
|---|---|---:|---|---|
| SL-01 | Left | Not recorded yet | No binding | Yes/No |
| SL-02 | Left | Not recorded yet | Tire close to bumper | Yes/No |
| SL-03 | Right | Not recorded yet | No binding | Yes/No |
| SL-04 | Right | Not recorded yet | Tire close to bumper | Yes/No |

### 9.4 Final Values

```text
SERVO_LEFT_LIMIT  = Not recorded yet
SERVO_RIGHT_LIMIT = Not recorded yet
```

### 9.5 Steering Clamp

The software should clamp steering commands to avoid unsafe values:

```text
steering_command = clamp(steering_command, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT)
```

For Obstacle Challenge, use a softer clamp than the mechanical maximum. Mechanical maximum is for safety, not normal driving.

---

## 10. Servo Response Calibration

After center and limits are known, verify how steering responds during motion.

### 10.1 Low-Speed Steering Test

| Test | Speed | Steering command | Expected behavior | Result |
|---|---:|---:|---|---|
| SR-01 | Not recorded yet | Center | Robot drives straight | Not recorded yet |
| SR-02 | Not recorded yet | Small left | Smooth left curve | Not recorded yet |
| SR-03 | Not recorded yet | Small right | Smooth right curve | Not recorded yet |
| SR-04 | Not recorded yet | Medium left | Controlled turn | Not recorded yet |
| SR-05 | Not recorded yet | Medium right | Controlled turn | Not recorded yet |

### 10.2 Steering Deadband

If small steering changes do not move the robot enough, define a steering deadband.

```text
SERVO_DEADBAND = Not recorded yet
```

This prevents the controller from producing tiny corrections that do not affect the wheels.

---

## 11. Motor Direction Calibration

The motor direction must match the software command names.

### 11.1 Motor Command IDs

| Command | ID | Expected behavior |
|---|---:|---|
| FORWARD continuous | `0x0004` | Robot moves forward |
| REVERSE continuous | `0x0005` | Robot moves backward |
| STOP | `0x0103` | Motor stops |
| SET SPEED | `0x0104` | PWM/speed changes |
| FORWARD by degrees | `0x0101` | Motor moves forward by encoder degrees |
| REVERSE by degrees | `0x0102` | Motor moves backward by encoder degrees |

### 11.2 Procedure

1. Lift the robot so wheels can rotate freely.
2. Set speed to a low value.
3. Send FORWARD.
4. Verify wheel direction.
5. Send STOP.
6. Send REVERSE.
7. Verify opposite direction.
8. Send STOP.
9. Place robot on the floor.
10. Repeat for 1 second at low speed.

### 11.3 Direction Test Table

| Test | Command | Expected result | Observed result | Pass? |
|---|---|---|---|---|
| MD-01 | FORWARD | Wheels move robot forward | Not recorded yet | Not recorded yet |
| MD-02 | STOP | Motor stops | Not recorded yet | Not recorded yet |
| MD-03 | REVERSE | Wheels move robot backward | Not recorded yet | Not recorded yet |
| MD-04 | STOP | Motor stops | Not recorded yet | Not recorded yet |

### 11.4 Final Direction Status

```text
MOTOR_FORWARD_IS_CORRECT = Not recorded yet
```

If forward and reverse are inverted, fix the firmware or wiring before continuing.

---

## 12. Motor Speed Calibration

Motor speed calibration finds stable speed values for different driving situations.

### 12.1 Goals

Find:

```text
MOTOR_SPEED_MIN_MOVING
MOTOR_SPEED_OPEN_STRAIGHT
MOTOR_SPEED_CORNER
MOTOR_SPEED_OBSTACLE
MOTOR_SPEED_PARKING
MOTOR_SPEED_MAX_SAFE
```

### 12.2 Minimum Moving Speed

The minimum moving speed is the lowest command that moves the robot reliably on the WRO mat.

#### Procedure

1. Place the robot on the field.
2. Command a very low speed.
3. Increase speed gradually.
4. Record the first value where the robot starts moving consistently.
5. Repeat with fully charged battery and partially used battery.

#### Table

| Test | Speed command | Battery voltage | Movement result |
|---|---:|---:|---|
| MS-01 | 20% | Not recorded yet | Not recorded yet |
| MS-02 | 30% | Not recorded yet | Not recorded yet |
| MS-03 | 40% | Not recorded yet | Not recorded yet |
| MS-04 | 50% | Not recorded yet | Not recorded yet |

Final value:

```text
MOTOR_SPEED_MIN_MOVING = Not recorded yet
```

### 12.3 Open Challenge Straight Speed

The Open Challenge speed should be fast enough to complete laps but slow enough for reliable corner detection.

| Test | Speed | Laps completed | Time | Failure / result |
|---|---:|---:|---:|---|
| OS-01 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| OS-02 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| OS-03 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |

Final value:

```text
MOTOR_SPEED_OPEN_STRAIGHT = Not recorded yet
```

### 12.4 Corner Speed

If the robot turns too late or oversteers at corners, reduce speed before cornering.

| Test | Speed | Corner trigger distance | Turn result |
|---|---:|---:|---|
| CS-01 | Not recorded yet | Not recorded yet | Not recorded yet |
| CS-02 | Not recorded yet | Not recorded yet | Not recorded yet |
| CS-03 | Not recorded yet | Not recorded yet | Not recorded yet |

Final value:

```text
MOTOR_SPEED_CORNER = Not recorded yet
```

### 12.5 Obstacle Speed

Obstacle speed must allow enough time for YOLO detection and steering response.

| Test | Speed | Red pillar result | Green pillar result | Notes |
|---|---:|---|---|---|
| OBMS-01 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| OBMS-02 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| OBMS-03 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |

Final value:

```text
MOTOR_SPEED_OBSTACLE = Not recorded yet
```

### 12.6 Motor Speed Rule

Do not choose speed based only on straight-line movement. Choose speed based on complete system success:

```text
Best speed = highest speed that still allows stable sensing, vision, steering, and stopping.
```

---

## 13. Encoder Calibration

The motor encoder is used to measure rotation and support degree-based movement.

### 13.1 Required Values

| Parameter | Value |
|---|---:|
| Encoder pulses per motor revolution | 11 PPR base |
| Gearbox reduction ratio | Not recorded yet |
| Effective pulses per output revolution | Not recorded yet |
| Wheel diameter | Not recorded yet |
| Wheel circumference | Not recorded yet |
| Encoder ticks per centimeter | Not recorded yet |

### 13.2 Wheel Circumference

Measure the wheel diameter and calculate:

```text
wheel_circumference = pi * wheel_diameter
```

### 13.3 Encoder Test Procedure

1. Lift the robot.
2. Mark one rear wheel with tape.
3. Reset encoder count.
4. Rotate the wheel one full revolution manually or with low motor command.
5. Record encoder ticks.
6. Repeat three times.
7. Compare forward and reverse readings.
8. Update constants if needed.

### 13.4 Encoder Calibration Table

| Test | Direction | Wheel rotations | Encoder ticks | Ticks per rotation |
|---|---|---:|---:|---:|
| ENC-01 | Forward | 1 | Not recorded yet | Not recorded yet |
| ENC-02 | Forward | 1 | Not recorded yet | Not recorded yet |
| ENC-03 | Reverse | 1 | Not recorded yet | Not recorded yet |
| ENC-04 | Reverse | 1 | Not recorded yet | Not recorded yet |

Final value:

```text
ENCODER_TICKS_PER_WHEEL_REV = Not recorded yet
```

---

## 14. ToF Sensor Initialization Calibration

ARBIBOT uses multiple VL53 sensors. Since VL53 sensors can share default I2C addresses, XSHUT-controlled startup is required.

### 14.1 Known XSHUT Pins

| Sensor | XSHUT pin |
|---|---|
| Left VL53L4CD | PA5 |
| Right-front VL53L4CD | PA7 |
| Right-rear VL53L4CD | PB14 |

### 14.2 Known Address Map

| Sensor | Address | Notes |
|---|---:|---|
| Left VL53L4CD | `0x52` | I2C2 / hi2c2, XSHUT PA5 |
| Right-front VL53L4CD | `0x54` | I2C2 / hi2c2, XSHUT PA7 |
| Right-rear VL53L4CD | `0x56` | I2C2 / hi2c2, XSHUT PB14 |
| Front VL53L8CX | `0x52` 8-bit / `0x29` 7-bit | I2C1 / hi2c1; sole device on bus, no conflict |

> Address map confirmed: front matrix is isolated on I2C1 at the default address, while the side VL53L4CD sensors are on I2C2 and initialized with XSHUT sequencing.

### 14.3 Startup Check

| Test | Expected result | Status |
|---|---|---|
| STM32 boots without HardFault | No firmware crash | Not recorded yet |
| Left sensor alive | Valid distance | Not recorded yet |
| Right-front sensor alive | Valid distance | Not recorded yet |
| Right-rear sensor alive | Valid distance | Not recorded yet |
| Front matrix alive | Valid matrix | Not recorded yet |
| Reboot repeats successfully | Same result after reset | Not recorded yet |

---

## 15. VL53L4CD Side Sensor Distance Calibration

The side sensors are calibrated using known distances.

### 15.1 Distances

Use:

```text
120 cm, 110 cm, 100 cm, 90 cm, 80 cm, 70 cm,
60 cm, 50 cm, 40 cm, 30 cm, 20 cm, 10 cm
```

### 15.2 Procedure

1. Place the robot parallel to a flat wall or board.
2. Align the sensor face perpendicular to the wall.
3. Set the first distance using a measuring tape.
4. Read each side sensor several times.
5. Record average, minimum, maximum, and invalid count.
6. Repeat for all distances.
7. Calculate error:

```text
error_mm = measured_mm - real_mm
```

8. Decide whether each sensor needs an offset or filter.

### 15.3 Calibration Table

| Real distance | Real mm | Left avg | RF avg | RR avg | Left error | RF error | RR error | Notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 120 cm | 1200 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 110 cm | 1100 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 100 cm | 1000 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 90 cm | 900 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 80 cm | 800 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 70 cm | 700 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 60 cm | 600 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 50 cm | 500 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 40 cm | 400 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 30 cm | 300 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 20 cm | 200 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |
| 10 cm | 100 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |  |

### 15.4 Final Offsets

```text
LEFT_TOF_OFFSET_MM        = Not recorded yet
RIGHT_FRONT_TOF_OFFSET_MM = Not recorded yet
RIGHT_REAR_TOF_OFFSET_MM  = Not recorded yet
```

---

## 16. ToF Invalid Reading Thresholds

Some distance readings should be ignored.

### 16.1 Invalid Reading Rules

Suggested invalid conditions:

```text
distance_mm <= 0
distance_mm == -1
distance_mm > MAX_VALID_DISTANCE_MM
sensor_status is invalid
sudden jump larger than allowed threshold
```

### 16.2 Threshold Table

| Parameter | Current value | Final value | Purpose |
|---|---:|---:|---|
| `MIN_VALID_DISTANCE_MM` | Not recorded yet | Not recorded yet | Reject too-close/fake values |
| `MAX_VALID_DISTANCE_MM` | Not recorded yet | Not recorded yet | Reject out-of-range values |
| `MAX_JUMP_MM` | Not recorded yet | Not recorded yet | Reject sudden spikes |
| `INVALID_HOLD_TIME_MS` | Not recorded yet | Not recorded yet | Use last valid value briefly |
| `SIDE_SENSOR_TIMEOUT_MS` | Not recorded yet | Not recorded yet | Detect missing sensor update |

### 16.3 Last Valid Value Strategy

If a sensor briefly gives an invalid reading, use the last valid value:

```text
if new_reading_is_valid:
    distance = new_reading
    last_valid = new_reading
else:
    distance = last_valid
```

Do not use stale data forever. Add a timeout.

---

## 17. Right-Side Heading Calibration

ARBIBOT uses two right-side sensors for heading correction.

### 17.1 Heading Error

```text
heading_error = right_front_distance - right_rear_distance
```

Interpretation:

| Condition | Meaning |
|---|---|
| `right_front ≈ right_rear` | Robot is approximately parallel to wall |
| `right_front > right_rear` | Front is farther from wall than rear |
| `right_front < right_rear` | Front is closer to wall than rear |

### 17.2 Procedure

1. Place the robot parallel to the right wall.
2. Measure physical distance from both sensors to the wall.
3. Read right-front and right-rear values.
4. Adjust offsets if one sensor consistently reads higher/lower.
5. Confirm heading error is close to zero when robot is physically parallel.

### 17.3 Calibration Table

| Test | Physical RF distance | Physical RR distance | RF reading | RR reading | Heading error |
|---|---:|---:|---:|---:|---:|
| RH-01 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| RH-02 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| RH-03 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |

Final acceptable heading error when parallel:

```text
MAX_PARALLEL_HEADING_ERROR_MM = Not recorded yet
```

---

## 18. Target Wall Distance Calibration

The lane controller needs a target distance from the wall.

### 18.1 Procedure

1. Place the robot in the preferred driving lane.
2. Measure the right-front and right-rear distances.
3. Average the two values.
4. Drive slowly and observe whether the robot has enough clearance from walls and corners.
5. Tune target distance until it follows the lane without clipping walls or oscillating.

### 18.2 Target Distance Table

| Test | Target distance | Speed | Result |
|---|---:|---:|---|
| TD-01 | Not recorded yet | Not recorded yet | Too close to wall |
| TD-02 | Not recorded yet | Not recorded yet | Too far from wall |
| TD-03 | Not recorded yet | Not recorded yet | Stable |
| TD-04 | Not recorded yet | Not recorded yet | Final candidate |

Final value:

```text
TARGET_RIGHT_WALL_DISTANCE_MM = Not recorded yet
```

---

## 19. Front Matrix Calibration

The front VL53L8CX matrix helps detect walls, corners, and front obstacles.

### 19.1 Goal

Find reliable thresholds for:

```text
FRONT_TRIGGER_DISTANCE_MM
FRONT_MIN_VALID_MM
FRONT_MAX_VALID_MM
FRONT_REQUIRED_VALID_CELLS
FRONT_MATRIX_ROW_USED
```

### 19.2 Current Development Logic

Current development behavior:

```text
front trigger distance ≈ 100 cm
valid cells required >= 2
floor-band cells are ignored
```

### 19.3 Procedure

1. Place the robot facing a flat wall.
2. Start at 120 cm.
3. Move toward the wall in 10 cm steps.
4. Record the front matrix values at each step.
5. Identify which cells consistently see the wall.
6. Identify cells that see the floor or irrelevant objects.
7. Select the best matrix row/cells.
8. Set minimum and maximum valid range.
9. Require at least two valid cells.
10. Test by approaching the corner at real motor speed.

### 19.4 Front Matrix Table

| Real distance | Valid cells | Selected row values | Trigger? | Notes |
|---:|---:|---|---|---|
| 120 cm | Not recorded yet | Not recorded yet | No/Yes |  |
| 110 cm | Not recorded yet | Not recorded yet | No/Yes |  |
| 100 cm | Not recorded yet | Not recorded yet | No/Yes |  |
| 90 cm | Not recorded yet | Not recorded yet | No/Yes |  |
| 80 cm | Not recorded yet | Not recorded yet | No/Yes |  |
| 70 cm | Not recorded yet | Not recorded yet | No/Yes |  |
| 60 cm | Not recorded yet | Not recorded yet | No/Yes |  |

### 19.5 Final Values

```text
FRONT_TRIGGER_DISTANCE_MM   = 1000
FRONT_MIN_VALID_MM          = Not recorded yet
FRONT_MAX_VALID_MM          = Not recorded yet
FRONT_REQUIRED_VALID_CELLS  = 2
FRONT_MATRIX_ROW_USED       = Not recorded yet
```

---

## 20. Camera Physical Alignment

Camera alignment affects YOLO, line/corner detection, and obstacle strategy.

### 20.1 Goal

The camera should see:

- the track ahead,
- red/green pillars early enough,
- useful floor/corner features,
- without too much robot chassis blocking the view.

### 20.2 Procedure

1. Mount camera firmly.
2. Start camera preview.
3. Place robot in normal lane position.
4. Confirm the horizon/field is visible.
5. Confirm front bumper does not cover too much view.
6. Place red and green pillars at expected distances.
7. Confirm both are visible before the robot is too close.
8. Record sample images.
9. Do not change camera angle after YOLO calibration unless necessary.

### 20.3 Camera Alignment Table

| Test | Camera angle | Pillar visible distance | Field visibility | Result |
|---|---|---:|---|---|
| CAM-01 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| CAM-02 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| CAM-03 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |

### 20.4 Final Camera Setup

```text
CAMERA_RESOLUTION = Not recorded yet
CAMERA_FPS        = Not recorded yet
CAMERA_ANGLE      = Not recorded yet
CAMERA_EXPOSURE   = Not recorded yet
CAMERA_GAIN       = Not recorded yet
```

---

## 21. Camera Color Calibration

Camera color correction helps make red/green pillar detection more consistent.

### 21.1 Procedure

1. Run camera preview.
2. Place red and green pillars under normal lighting.
3. Test current color correction from `color_tuning.py`.
4. Compare appearance on screen with real pillar colors.
5. Adjust brightness, contrast, saturation, or color correction only if necessary.
6. Save before/after images.
7. Re-test YOLO after color changes.

### 21.2 Color Tuning Table

| Test | Brightness | Contrast | Saturation | Notes |
|---|---:|---:|---:|---|
| CT-01 | Not recorded yet | Not recorded yet | Not recorded yet | Original |
| CT-02 | Not recorded yet | Not recorded yet | Not recorded yet | Better red |
| CT-03 | Not recorded yet | Not recorded yet | Not recorded yet | Better green |
| CT-04 | Not recorded yet | Not recorded yet | Not recorded yet | Final |

### 21.3 Final Color Values

```text
CAMERA_BRIGHTNESS = Not recorded yet
CAMERA_CONTRAST   = Not recorded yet
CAMERA_SATURATION = Not recorded yet
COLOR_PROFILE     = Not recorded yet
```

---

## 22. YOLO Confidence Calibration

YOLO confidence controls when detections are accepted.

### 22.1 Current Values

Current development values:

```text
PILLAR_CONFIDENCE_THRESHOLD = 0.30
CORNER_LINE_CONFIDENCE_THRESHOLD = 0.48
```

These values should be validated with final robot camera position and final lighting.

### 22.2 Goal

Choose confidence thresholds that minimize both:

- false positives: robot reacts to something that is not a valid target,
- false negatives: robot ignores a real pillar or corner feature.

### 22.3 Pillar Confidence Procedure

1. Place red pillar at different distances.
2. Record confidence values.
3. Repeat with green pillar.
4. Test center, left edge, and right edge positions.
5. Test under normal and glare lighting.
6. Record false positives.
7. Choose threshold that accepts real pillars early while rejecting noise.

### 22.4 Pillar Confidence Table

| Test | Object | Position | Distance | Lighting | Confidence | Accepted? | Correct? |
|---|---|---|---:|---|---:|---|---|
| YC-01 | Red pillar | Center | Not recorded yet | Normal | Not recorded yet | Not recorded yet | Not recorded yet |
| YC-02 | Red pillar | Left | Not recorded yet | Normal | Not recorded yet | Not recorded yet | Not recorded yet |
| YC-03 | Red pillar | Right | Not recorded yet | Glare | Not recorded yet | Not recorded yet | Not recorded yet |
| YC-04 | Green pillar | Center | Not recorded yet | Normal | Not recorded yet | Not recorded yet | Not recorded yet |
| YC-05 | Green pillar | Left | Not recorded yet | Normal | Not recorded yet | Not recorded yet | Not recorded yet |
| YC-06 | Green pillar | Right | Not recorded yet | Glare | Not recorded yet | Not recorded yet | Not recorded yet |
| YC-07 | No pillar | N/A | N/A | Normal | Not recorded yet | Not recorded yet | Not recorded yet |

Final value:

```text
PILLAR_CONFIDENCE_THRESHOLD = Not recorded yet
```

### 22.5 Corner Line Confidence Procedure

1. Place robot near a corner.
2. Run corner/line model.
3. Record confidence values for real corner features.
4. Test different lighting.
5. Test motion blur while moving.
6. Choose confidence value that detects true corners but rejects noise.

### 22.6 Corner Confidence Table

| Test | Scene | Lighting | Speed | Confidence | Accepted? | Correct? |
|---|---|---|---:|---:|---|---|
| CC-01 | Corner line visible | Normal | 0 | Not recorded yet | Not recorded yet | Not recorded yet |
| CC-02 | Corner line visible | Normal | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| CC-03 | No corner | Normal | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| CC-04 | Corner with glare | Glare | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |

Final value:

```text
CORNER_LINE_CONFIDENCE_THRESHOLD = Not recorded yet
```

---

## 23. YOLO Zone Calibration

The obstacle strategy uses screen position to decide how to steer around pillars.

### 23.1 Screen Zones

The image is divided into three horizontal zones:

```text
LEFT | CENTER | RIGHT
```

For a 1280-pixel-wide image:

```text
LEFT   = x < 426
CENTER = 426 <= x < 853
RIGHT  = x >= 853
```

### 23.2 Procedure

1. Show camera frame with vertical zone lines.
2. Place a pillar in the left third.
3. Confirm the software reports `LEFT`.
4. Move pillar to center.
5. Confirm `CENTER`.
6. Move pillar to right third.
7. Confirm `RIGHT`.
8. Confirm bounding box center and area are logged.

### 23.3 Zone Calibration Table

| Test | Pillar color | Physical position | Detected zone | Correct? |
|---|---|---|---|---|
| ZN-01 | Red | Left | Not recorded yet | Not recorded yet |
| ZN-02 | Red | Center | Not recorded yet | Not recorded yet |
| ZN-03 | Red | Right | Not recorded yet | Not recorded yet |
| ZN-04 | Green | Left | Not recorded yet | Not recorded yet |
| ZN-05 | Green | Center | Not recorded yet | Not recorded yet |
| ZN-06 | Green | Right | Not recorded yet | Not recorded yet |

### 23.4 Final Zone Values

```text
FRAME_WIDTH = Not recorded yet
ZONE_LEFT_MAX_X = Not recorded yet
ZONE_CENTER_MAX_X = Not recorded yet
```

---

## 24. YOLO Area / Distance Calibration

Bounding-box area is used as an estimate of closeness. Larger area usually means the pillar is closer.

### 24.1 Procedure

1. Place pillar at known distances.
2. Record bounding-box area.
3. Repeat for red and green.
4. Use area thresholds to decide when the robot should start active avoidance.

### 24.2 Area Table

| Distance | Red area | Green area | Notes |
|---:|---:|---:|---|
| 120 cm | Not recorded yet | Not recorded yet |  |
| 100 cm | Not recorded yet | Not recorded yet |  |
| 80 cm | Not recorded yet | Not recorded yet |  |
| 60 cm | Not recorded yet | Not recorded yet |  |
| 40 cm | Not recorded yet | Not recorded yet |  |
| 30 cm | Not recorded yet | Not recorded yet |  |

Final values:

```text
PILLAR_REACT_AREA_MIN = Not recorded yet
PILLAR_CLOSE_AREA     = Not recorded yet
PILLAR_PASSED_AREA    = Not recorded yet
```

---

## 25. Lane-Following Gain Calibration

Lane-following uses right-side distance and heading error.

### 25.1 Control Terms

```text
distance_error = target_right_distance - average(right_front, right_rear)
heading_error  = right_front - right_rear
steering       = Kd_distance * distance_error + Kh_heading * heading_error
```

### 25.2 Procedure

1. Set servo center and steering limits first.
2. Set a low motor speed.
3. Start with small gains.
4. Drive on a straight section.
5. Increase distance gain until robot corrects wall distance.
6. Add heading gain until robot stays parallel.
7. If oscillation appears, reduce gains or speed.
8. Test after corners.

### 25.3 Gain Table

| Test | Speed | Target distance | Distance gain | Heading gain | Result |
|---|---:|---:|---:|---:|---|
| LF-01 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Wall-to-wall wandering |
| LF-02 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Improved |
| LF-03 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Stable |
| LF-04 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Final candidate |

Final values:

```text
LANE_TARGET_RIGHT_MM = Not recorded yet
LANE_DISTANCE_GAIN   = Not recorded yet
LANE_HEADING_GAIN    = Not recorded yet
```

---

## 26. Obstacle PDI Calibration

The Obstacle Challenge uses PDI-style steering around pillars.

### 26.1 Goal

Tune the robot so it passes:

| Pillar | Required behavior |
|---|---|
| Red | Keep/pass on right side |
| Green | Keep/pass on left side |

### 26.2 Procedure

1. Verify YOLO detection first.
2. Verify servo center and steering clamp.
3. Use low motor speed.
4. Place one red pillar.
5. Tune PDI until the robot passes correctly without full-lock steering.
6. Repeat with green pillar.
7. Add multiple positions: left, center, right.
8. Increase speed only after stable behavior.

### 26.3 PDI Calibration Table

| Test | Pillar | Speed | Kp | Ki | Kd | Clamp | Result |
|---|---|---:|---:|---:|---:|---:|---|
| PDI-01 | Red | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| PDI-02 | Green | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| PDI-03 | Red | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| PDI-04 | Green | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |

Final values:

```text
PILLAR_PDI_KP = Not recorded yet
PILLAR_PDI_KI = Not recorded yet
PILLAR_PDI_KD = Not recorded yet
PILLAR_STEERING_CLAMP = Not recorded yet
```

### 26.4 Tuning Rule

If the robot goes full-lock or loses the pillar target:

```text
reduce gain
reduce steering clamp
start reaction earlier
reduce speed
```

---

## 27. Serial Timing Calibration

Serial timing affects sensors, motor commands, and vision loop timing.

### 27.1 Goal

Ensure motor commands do not block camera/YOLO execution.

### 27.2 Procedure

1. Run camera/YOLO loop without motor commands.
2. Measure FPS.
3. Run with motor commands using response waiting.
4. Measure FPS.
5. Run with fire-and-forget motor commands.
6. Measure FPS.
7. Confirm green and red pillars are detected while moving.
8. Confirm STOP command still works reliably.

### 27.3 Serial Timing Table

| Test | Motor command mode | Camera FPS | YOLO FPS | Serial timeouts | Notes |
|---|---|---:|---:|---:|---|
| ST-01 | No motor command | Not recorded yet | Not recorded yet | Not recorded yet | Baseline |
| ST-02 | Blocking response wait | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| ST-03 | Fire-and-forget | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| ST-04 | Sensor polling active | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |

Final values:

```text
MOTOR_WAIT_RESPONSE_DEFAULT = Not recorded yet
SERIAL_TIMEOUT_MS = Not recorded yet
STOP_PULSE_COUNT = Not recorded yet
STOP_PULSE_DELAY_MS = Not recorded yet
```

---

## 28. Battery Calibration and Voltage Logging

Motor speed and servo behavior can change when battery voltage drops.

### 28.1 Procedure

1. Fully charge batteries.
2. Measure motor battery voltage.
3. Measure Jetson/UPS battery voltage if accessible.
4. Run a 3-minute test.
5. Measure voltage again.
6. Repeat with YOLO, sensors, servo, and motor active.
7. Record whether behavior changes at lower voltage.

### 28.2 Voltage Table

| Test | Motor battery start | Motor battery end | Jetson battery start | Jetson battery end | Result |
|---|---:|---:|---:|---:|---|
| BAT-01 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| BAT-02 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |
| BAT-03 | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet | Not recorded yet |

Final safe range:

```text
MIN_MOTOR_BATTERY_VOLTAGE = Not recorded yet
MIN_JETSON_BATTERY_VOLTAGE = Not recorded yet
```

---

## 29. Full Calibration Checklist

Use this checklist before final challenge tuning.

### Mechanical

- [ ] Chassis fixed.
- [ ] Bumper fixed.
- [ ] Sensor mounts fixed.
- [ ] Camera mount fixed.
- [ ] Wheels clean.
- [ ] Steering alignment checked.

### Servo

- [ ] Servo center measured.
- [ ] Left limit measured.
- [ ] Right limit measured.
- [ ] Steering clamp set.
- [ ] Straight test completed.

### Motor

- [ ] Forward direction verified.
- [ ] Reverse direction verified.
- [ ] STOP command verified.
- [ ] Minimum moving speed measured.
- [ ] Open speed selected.
- [ ] Obstacle speed selected.
- [ ] Encoder ticks measured.

### ToF

- [ ] All sensors initialize.
- [ ] XSHUT sequence verified.
- [ ] Side sensors calibrated at known distances.
- [ ] Invalid reading handling tested.
- [ ] Right-front/right-rear heading error verified.
- [ ] Front matrix threshold tested.

### Camera / YOLO

- [ ] Camera angle final.
- [ ] Camera lens clean.
- [ ] Color profile selected.
- [ ] Red pillar detected.
- [ ] Green pillar detected.
- [ ] Confidence threshold selected.
- [ ] Zone boundaries verified.
- [ ] Bounding-box area thresholds recorded.

### Software

- [ ] Correct serial ports selected.
- [ ] Correct model files selected.
- [ ] Blocking motor behavior avoided.
- [ ] STOP pulse count selected.
- [ ] Lane-following gains recorded.
- [ ] PDI gains recorded.
- [ ] Logs enabled for final tests.

---

## 30. Final Calibration Summary

After final testing, copy the selected values here.

```text
SERVO_CENTER = Not recorded yet
SERVO_LEFT_LIMIT = Not recorded yet
SERVO_RIGHT_LIMIT = Not recorded yet

MOTOR_SPEED_MIN_MOVING = Not recorded yet
MOTOR_SPEED_OPEN_STRAIGHT = Not recorded yet
MOTOR_SPEED_CORNER = Not recorded yet
MOTOR_SPEED_OBSTACLE = Not recorded yet
MOTOR_SPEED_PARKING = Not recorded yet

PILLAR_CONFIDENCE_THRESHOLD = Not recorded yet
CORNER_LINE_CONFIDENCE_THRESHOLD = Not recorded yet

TARGET_RIGHT_WALL_DISTANCE_MM = Not recorded yet
LANE_DISTANCE_GAIN = Not recorded yet
LANE_HEADING_GAIN = Not recorded yet

FRONT_TRIGGER_DISTANCE_MM = Not recorded yet
FRONT_REQUIRED_VALID_CELLS = Not recorded yet

PILLAR_PDI_KP = Not recorded yet
PILLAR_PDI_KI = Not recorded yet
PILLAR_PDI_KD = Not recorded yet
PILLAR_STEERING_CLAMP = Not recorded yet

STOP_PULSE_COUNT = Not recorded yet
SERIAL_TIMEOUT_MS = Not recorded yet
```

---

## 31. Change Log

| Date | Calibration change | Reason | Result |
|---|---|---|---|
| Not recorded yet | Initial calibration procedure created | Documentation | Ready for measured values |
| Not recorded yet | Servo center updated | Mechanical alignment | Not recorded yet |
| Not recorded yet | ToF thresholds updated | Sensor testing | Not recorded yet |
| Not recorded yet | YOLO confidence updated | Vision testing | Not recorded yet |
| Not recorded yet | Motor speed updated | Challenge testing | Not recorded yet |

---

## 32. Conclusion

Calibration is one of the most important parts of ARBIBOT development. The robot can only drive reliably if mechanical alignment, steering values, sensor thresholds, camera setup, YOLO confidence, and motor speed are consistent.

The most important calibration dependencies are:

- **servo center before lane tuning,**
- **sensor placement before ToF thresholds,**
- **camera angle before YOLO tuning,**
- **motor speed before corner trigger distance,**
- **battery voltage before final performance testing,**
- **and non-blocking serial behavior before obstacle tuning.**

Final calibration values should be recorded in this document and updated whenever the physical robot or software strategy changes.
