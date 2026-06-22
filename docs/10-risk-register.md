# 10 - Risk Register

**Project:** MetallicMadness / ARBIBOT  
**Category:** WRO 2026 Future Engineers - Self-Driving Cars  
**Document purpose:** Identify the main failure modes that can affect ARBIBOT during development, testing, and competition, and define mitigation, detection, and recovery strategies.

> This document is a living engineering risk register. It should be updated whenever a new failure appears during testing or when an existing mitigation is improved.

---

## 1. Risk Register Purpose

ARBIBOT combines mechanical steering, motor control, distance sensors, camera vision, YOLO inference, UART communication, batteries, and challenge-specific software. A failure in any one of these subsystems can cause the full robot to fail.

The purpose of this risk register is to:

1. Identify the most important technical risks.
2. Explain how each risk appears during testing.
3. Estimate impact and probability.
4. Define prevention and detection methods.
5. Define recovery behavior where possible.
6. Connect each risk to real test evidence or known robot behavior.

The risk register helps the team make better engineering decisions instead of reacting randomly when the robot fails.

---

## 2. Risk Scoring Method

Each risk is scored using three values:

| Value | Meaning |
|---|---|
| Probability | How likely the risk is to happen |
| Impact | How serious the effect is if it happens |
| Detection difficulty | How hard it is to notice before it causes failure |

Score range:

| Score | Level |
|---:|---|
| 1 | Very low |
| 2 | Low |
| 3 | Medium |
| 4 | High |
| 5 | Very high |

Priority is calculated as:

```text
risk_priority = probability * impact * detection_difficulty
```

The score is not perfect, but it helps the team focus on the most dangerous problems first.

---

## 3. Risk Priority Levels

| Priority score | Level | Meaning |
|---:|---|---|
| 1-15 | Low | Monitor during testing |
| 16-35 | Medium | Add mitigation and test regularly |
| 36-60 | High | Must be addressed before competition |
| 61-125 | Critical | Immediate engineering action required |

---

## 4. Summary Risk Table

| ID | Risk | Subsystem | Probability | Impact | Detection difficulty | Priority | Level |
|---|---|---|---:|---:|---:|---:|---|
| R-001 | Camera glare / lighting changes | Vision | 4 | 4 | 3 | 48 | High |
| R-002 | Bad pillar detection | Vision / Strategy | 4 | 5 | 3 | 60 | High |
| R-003 | Sensor noise or invalid ToF readings | Sensors | 4 | 4 | 3 | 48 | High |
| R-004 | UART timeout or dropped serial frame | Communication | 4 | 4 | 4 | 64 | Critical |
| R-005 | Motor command blocking vision loop | Software / Communication | 4 | 5 | 4 | 80 | Critical |
| R-006 | Weak battery or voltage drop | Power | 3 | 5 | 3 | 45 | High |
| R-007 | Jetson brownout or reboot | Power / Compute | 2 | 5 | 3 | 30 | Medium |
| R-008 | Servo jitter or steering power dip | Power / Steering | 3 | 4 | 3 | 36 | High |
| R-009 | Wheel slip | Mechanical | 3 | 4 | 3 | 36 | High |
| R-010 | Steering misalignment | Mechanical / Control | 3 | 4 | 2 | 24 | Medium |
| R-011 | Bumper or sensor mount interference | Mechanical | 2 | 4 | 2 | 16 | Medium |
| R-012 | I2C address conflict | Sensors / Firmware | 2 | 5 | 3 | 30 | Medium |
| R-013 | Front matrix false corner trigger | Sensors / Strategy | 4 | 4 | 3 | 48 | High |
| R-014 | Corner detected too late | Sensors / Strategy | 3 | 5 | 3 | 45 | High |
| R-015 | Over-aggressive PDI steering | Obstacle strategy | 3 | 4 | 3 | 36 | High |
| R-016 | Robot loses pillar target | Vision / Strategy | 3 | 4 | 3 | 36 | High |
| R-017 | Stop command not executed | Motor / Communication | 3 | 5 | 4 | 60 | High |
| R-018 | Incorrect motor direction | Motor / Firmware | 2 | 5 | 2 | 20 | Medium |
| R-019 | Encoder feedback error | Motor / Odometry | 2 | 3 | 3 | 18 | Medium |
| R-020 | Wiring disconnect during run | Electrical | 2 | 5 | 3 | 30 | Medium |
| R-021 | Repository file too large | Documentation | 3 | 2 | 1 | 6 | Low |
| R-022 | Missing test data | Documentation / Validation | 4 | 3 | 1 | 12 | Low |
| R-023 | Parking behavior incomplete | Strategy | 3 | 4 | 3 | 36 | High |
| R-024 | Model not trained for competition lighting | Vision | 4 | 4 | 3 | 48 | High |
| R-025 | Sensor calibration drift after mechanical changes | Sensors / Mechanical | 3 | 4 | 3 | 36 | High |

---

## 5. Critical Risks

The following risks have the highest priority and should be tested frequently before competition:

| Risk | Reason |
|---|---|
| UART timeout or dropped serial frame | Can cause missed motor stop, stale sensor data, or command failure |
| Motor command blocking vision loop | Can cause missed pillar detection and wrong obstacle behavior |
| Bad pillar detection | Can cause incorrect red/green pass-side decision |
| Weak battery or voltage drop | Can cause resets, slow movement, servo jitter, or sensor instability |
| Sensor noise or invalid ToF readings | Can cause bad wall following, false corners, or late turns |

---

## 6. Detailed Risk Records

---

## R-001 - Camera Glare / Lighting Changes

### Description

Glare, shadows, strong reflections, or uneven lighting can affect the camera image. This can reduce YOLO confidence, change apparent pillar color, or make floor lines harder to detect.

### Subsystem affected

- Jetson camera
- YOLO inference
- line/corner detection
- obstacle detection
- challenge strategy

### Causes

- bright overhead lights,
- reflections from the WRO field,
- camera auto-exposure changes,
- shadows from the robot or people,
- pillar color appearance changing with angle,
- lens contamination or fingerprints.

### Symptoms

- red or green pillar confidence drops,
- pillar is detected late,
- pillar class changes between frames,
- corner line detection becomes unstable,
- false positives appear on floor or wall,
- robot reacts too late.

### Impact

High. The robot may pass a pillar on the wrong side or fail to detect a corner/visual reference.

### Mitigation

- Use YOLO instead of simple HSV thresholding.
- Add color correction in `color_tuning.py`.
- Test under several lighting conditions.
- Avoid relying on one frame only.
- Use confidence thresholds and detection smoothing.
- Record example images from bad lighting and add them to the dataset.
- Clean camera lens before every run.
- Lock or tune camera exposure if possible.

### Detection method

- Monitor YOLO confidence during test runs.
- Save screenshots of missed detections.
- Compare detection results under different light positions.
- Check whether failure appears only under certain lighting.

### Recovery behavior

- If detection is lost briefly, hold last valid detection for a short timeout.
- Fall back to wall following if no valid pillar is visible.
- Avoid sudden full-lock steering based on one low-confidence detection.

### Status

Open. Needs final lighting test under competition-like conditions.

---

## R-002 - Bad Pillar Detection

### Description

The robot may fail to detect a red or green pillar, detect the wrong color, or select the wrong pillar when multiple objects are visible.

### Subsystem affected

- YOLO model
- camera pipeline
- `Pillar_recognition.py`
- `pillar_counter.py`
- obstacle state machine
- steering controller

### Causes

- insufficient training data,
- poor lighting,
- motion blur,
- partial occlusion,
- pillar too far away,
- pillar too close to image edge,
- confidence threshold too high or too low,
- another object incorrectly classified as pillar,
- robot moving too fast for inference timing.

### Symptoms

- robot does not react to pillar,
- robot reacts too late,
- robot passes red/green pillar on wrong side,
- robot oscillates between detections,
- robot chooses a farther pillar instead of nearest pillar,
- detection flickers between red and green.

### Impact

Very high. In the Obstacle Challenge, incorrect pillar detection directly causes strategy failure.

### Mitigation

- Keep using YOLO classes `Green_Pillar` and `Red_Pillar`.
- Select nearest/largest valid pillar using bounding-box area.
- Use screen-zone logic to understand left/center/right position.
- Debounce detections across frames.
- Reduce motor speed near obstacles if needed.
- Add more training data with real field lighting.
- Include examples of close, far, left-edge, right-edge, and partially occluded pillars.
- Tune confidence threshold.
- Log detection class, confidence, center-X, and area during runs.

### Detection method

- Review video with YOLO overlay.
- Log every detection frame.
- Count false positives, false negatives, and wrong-color classifications.
- Test each color separately and then mixed.

### Recovery behavior

- If pillar disappears briefly, keep last valid target for a short time.
- If confidence is too low, prefer safe lane following instead of aggressive steering.
- If two detections appear, choose the closest/largest valid object.
- If classification flickers, require stable color for multiple frames.

### Status

Open. Pillar strategy works in development but requires more full Obstacle Challenge run data.

---

## R-003 - Sensor Noise or Invalid ToF Readings

### Description

VL53 distance sensors may return invalid values, noisy readings, floor reflections, or inconsistent measurements. This can affect wall following, corner detection, and obstacle distance estimation.

### Subsystem affected

- VL53L4CD side sensors
- VL53L8CH front matrix sensor
- I2C bus
- STM32 firmware
- Jetson sensor parser
- lane-following controller

### Causes

- sensor mounted too low,
- sensor sees floor instead of wall,
- angled surface reflection,
- I2C communication issue,
- out-of-range measurement,
- sensor startup/address problem,
- matrix cell sees irrelevant object,
- serial parser mismatch,
- sensor blocked by bumper or cable.

### Symptoms

- `-1` distance value,
- sudden distance spikes,
- robot steers toward wall,
- robot drifts away from wall,
- corner triggers too early,
- corner triggers too late,
- front matrix reports false close obstacle.

### Impact

High. The robot can hit walls, turn too early, or fail to turn.

### Mitigation

- Use last valid reading when new value is invalid.
- Reject out-of-range values.
- Apply median or simple filtering.
- Check sensor status values.
- Use two right-side sensors for heading correction.
- For front matrix, require multiple valid cells.
- Exclude cells known to see floor or irrelevant surfaces.
- Verify sensor mounting height and angle.
- Use XSHUT initialization for address control.

### Detection method

- Run fixed-distance calibration at 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10 cm.
- Log raw and filtered sensor values.
- Print matrix values during corner approach.
- Compare right-front and right-rear readings while robot is parallel to wall.

### Recovery behavior

- If one side sensor is invalid, temporarily rely on the other side sensor or last valid value.
- If front matrix is invalid, avoid triggering a corner based on one bad frame.
- If too many sensors are invalid, slow down or stop.

### Status

Partially mitigated. Parser and front matrix filtering were improved, but final calibration data is still needed.

---

## R-004 - UART Timeout or Dropped Serial Frame

### Description

The Jetson and STM32 communicate through a custom UART protocol. A command frame or response frame can be delayed, corrupted, or missed.

### Subsystem affected

- Jetson serial code
- STM32 serial protocol
- motor commands
- sensor requests
- command response handling
- stop/start behavior

### Causes

- serial bus busy with sensor polling,
- matrix read taking too long,
- blocking wait in wrong thread,
- checksum mismatch,
- incomplete frame,
- USB serial latency,
- buffer overflow,
- command sent while another response is pending,
- electrical noise.

### Symptoms

- motor command ignored,
- stop command missed,
- sensor data does not update,
- timeout exception,
- stale values displayed,
- robot continues moving when it should stop,
- Jetson and STM32 get out of sync.

### Impact

Very high. A missed STOP or delayed sensor response can cause crash or wrong challenge behavior.

### Mitigation

- Use framed protocol with start byte, length, command ID, checksum, and end byte.
- Keep one owner for serial reads.
- Avoid multiple threads reading from the same port.
- Use non-blocking motor commands where safe.
- Pulse critical commands such as STOP and FORWARD.
- Add command timeouts.
- Flush stale bytes if frame parsing fails.
- Log command ID and response text.
- Use checksums to reject bad frames.

### Detection method

- Log every command sent and response received.
- Count timeouts per run.
- Add timestamp around command send/response.
- Test motor and sensor commands separately before full run.
- Reproduce failure while reading front matrix and sending STOP.

### Recovery behavior

- Retry critical command.
- Send STOP multiple times.
- If serial link fails repeatedly, stop the robot or enter safe mode.
- Reset parser state after invalid frame.

### Status

Critical and partially mitigated. Critical commands are pulsed, and blocking serial behavior has been reduced.

---

## R-005 - Motor Command Blocking Vision Loop

### Description

If motor command functions wait for a response while the camera/YOLO loop needs to run, the robot can miss visual detections. This was observed when green pillar recognition improved after changing motor command behavior to avoid blocking.

### Subsystem affected

- Jetson software
- motor driver wrapper
- serial communication
- YOLO loop
- obstacle strategy

### Causes

- `wait_response=True` during active driving,
- motor command uses same serial link as sensor requests,
- serial read conflict between sensor and motor code,
- command timeout pauses main loop,
- motor function blocks while camera frames are skipped.

### Symptoms

- YOLO detection becomes slow or uneven,
- green pillar detection fails during movement,
- robot sees pillar only when stopped,
- motor command works but recognition fails,
- camera display updates at uneven pace.

### Impact

Critical. Obstacle behavior depends on timely vision.

### Mitigation

- Use fire-and-forget motor commands when safe.
- Let sensor thread own serial reads.
- Do not block camera loop waiting for motor response.
- Pulse critical motor commands instead of waiting once.
- Decouple camera inference from motor serial handling.
- Keep motor commands short.
- Use cached sensor values instead of blocking sensor calls inside vision loop.

### Detection method

- Measure camera FPS with and without motor commands.
- Measure YOLO inference interval during driving.
- Log motor command timestamps.
- Check whether detections appear delayed during serial waits.
- Test green pillar detection while motor is moving.

### Recovery behavior

- If camera FPS drops below minimum, reduce motor speed or pause non-critical sensor reads.
- If command response blocks, timeout quickly and continue vision loop.
- If obstacle detection is uncertain, slow down instead of steering aggressively.

### Status

Critical and actively mitigated. Continue testing with full obstacle runs.

---

## R-006 - Weak Battery or Voltage Drop

### Description

A weak or discharged battery can reduce motor speed, cause servo instability, create sensor errors, or reboot compute electronics.

### Subsystem affected

- motor battery pack
- BMS
- UPS module
- Jetson
- servo controller
- STM32
- sensors
- motor driver

### Causes

- low state of charge,
- battery cells not balanced,
- high current draw,
- motor stall,
- servo peak current,
- bad connector,
- insufficient regulator current,
- long wires or voltage drop,
- repeated test runs without recharging.

### Symptoms

- robot slows down over time,
- steering jitter,
- Jetson reboot,
- camera disconnect,
- STM32 reset,
- motor weak or inconsistent,
- sensor readings become unstable,
- stop/start behavior changes with battery level.

### Impact

Very high. Power instability can cause full robot failure.

### Mitigation

- Charge batteries before each test session.
- Measure voltage before and after each run.
- Keep motor and electronics power domains separated.
- Use UPS module for Jetson/electronics.
- Use BMS for motor battery protection.
- Check connectors and wire gauge.
- Avoid motor stall.
- Add battery voltage to test log.
- Replace weak 18650 cells if voltage sag is excessive.

### Detection method

- Measure start and end voltage.
- Observe voltage under acceleration.
- Log failures by battery level.
- Check whether failures happen late in test session.
- Test Jetson runtime with YOLO active.
- Test motor runtime separately.

### Recovery behavior

- Stop testing when voltage falls below safe level.
- Recharge or replace battery pack.
- Reduce speed if voltage sag is detected during testing.
- Do not continue autonomous testing with unstable voltage.

### Status

Open. Estimated runtime is acceptable, but final measured runtime and voltage sag are still needed.

---

## R-007 - Jetson Brownout or Reboot

### Description

The Jetson Orin Nano may reboot if its input voltage/current becomes unstable. Since the Jetson runs AI vision and high-level decisions, this is a severe failure.

### Subsystem affected

- Jetson Orin Nano
- camera
- YOLO model
- navigation logic
- serial control from Jetson

### Causes

- UPS output unable to handle peak load,
- weak battery cells,
- loose power connector,
- motor noise coupling into electronics,
- current draw from camera and peripherals,
- voltage sag during servo movement.

### Symptoms

- camera stream stops,
- SSH connection drops,
- Jetson boot messages appear,
- robot stops making decisions,
- serial commands stop,
- YOLO process exits.

### Impact

Very high.

### Mitigation

- Use stable UPS power path.
- Fully charge Jetson battery pack.
- Keep Jetson power separate from motor power.
- Reduce unnecessary USB loads.
- Monitor Jetson temperature and power mode.
- Use secure power connectors.
- Test full-load runtime before competition.

### Detection method

- Watch system uptime.
- Check `dmesg` after failure.
- Monitor voltage if possible.
- Run YOLO stress test while servo and motor are active.

### Recovery behavior

- If Jetson reboots during testing, stop robot and do not continue run.
- Investigate power and thermal logs.
- Replace or recharge battery pack.

### Status

Medium risk. Needs final full-load validation.

---

## R-008 - Servo Jitter or Steering Power Dip

### Description

The MG996R servo can draw high current, especially when steering under load. This can cause jitter or voltage dips if power is insufficient.

### Subsystem affected

- steering servo
- Pololu servo controller
- UPS/electronics power
- mechanical steering linkage
- lane controller

### Causes

- servo under heavy load,
- steering linkage binding,
- power rail voltage drop,
- weak UPS battery,
- servo trying to hold against mechanical limit,
- aggressive steering commands.

### Symptoms

- front wheels shake,
- steering response inconsistent,
- robot oscillates,
- Pololu controller resets,
- Jetson or STM32 disturbed by voltage dip,
- servo becomes hot.

### Impact

High. Steering instability affects every challenge.

### Mitigation

- Avoid commanding beyond mechanical limits.
- Tune steering clamp.
- Verify linkage does not bind.
- Measure servo current.
- Use adequate power supply.
- Reduce PDI/PID gains if steering oscillates.
- Check servo horn and linkage screw tightness.

### Detection method

- Command left/right steering while robot is lifted.
- Command steering while robot is on the mat.
- Listen for servo strain at endpoints.
- Watch for resets during steering.
- Check if jitter increases when battery is low.

### Recovery behavior

- Reduce steering command range.
- Stop test if servo overheats.
- Recenter steering and restart calibration.

### Status

High risk during aggressive obstacle steering. Partially mitigated by steering clamp and mechanical tuning.

---

## R-009 - Wheel Slip

### Description

Wheel slip reduces control accuracy because the robot does not move according to expected steering and motor commands.

### Subsystem affected

- wheels
- drivetrain
- lane control
- obstacle avoidance
- encoder feedback

### Causes

- thin wheels,
- dusty field,
- high acceleration,
- aggressive turning,
- low weight on drive axle,
- tire material not gripping mat.

### Symptoms

- robot slides outward in turns,
- encoder suggests movement but robot path differs,
- lane controller overcorrects,
- obstacle pass path becomes unpredictable,
- lap time inconsistent.

### Impact

High.

### Mitigation

- Use wider rubber wheels.
- Clean wheels before runs.
- Avoid sudden acceleration.
- Tune speed and steering clamp.
- Ensure rear axle traction is adequate.
- Keep robot weight balanced.

### Detection method

- Video review of turns.
- Compare encoder movement with actual field position.
- Watch for lateral sliding during cornering.

### Recovery behavior

- Reduce speed.
- Use smoother steering.
- Clean or replace wheels.

### Status

Improved after wheel change, but still monitor during fast runs.

---

## R-010 - Steering Misalignment

### Description

If the steering is not centered, the robot drifts even when commanded straight.

### Subsystem affected

- steering linkage
- servo controller
- lane-following software
- mechanical alignment

### Causes

- linkage rods unequal,
- servo horn installed off-center,
- screw alignment not set,
- mechanical play,
- wheel toe angle incorrect,
- servo center value wrong.

### Symptoms

- robot drifts left or right on straight command,
- PID constantly corrects one direction,
- turning radius differs left vs right,
- robot exits corners inconsistently.

### Impact

Medium to high.

### Mitigation

- Mechanically align wheels before software tuning.
- Record final servo center value.
- Use adjustment screw for front-wheel alignment.
- Test straight driving without wall-following correction.
- Recheck alignment after mechanical impacts.

### Detection method

- Drive straight on open section.
- Lift robot and check neutral wheel angle.
- Log steering correction while robot should be centered.

### Recovery behavior

- Recenter servo.
- Adjust linkage and alignment screw.
- Retune PID only after mechanical alignment is fixed.

### Status

Medium. Needs final documented servo center and steering limits.

---

## R-011 - Bumper or Sensor Mount Interference

### Description

The front bumper or sensor mounts can interfere with wheel steering or block sensor view.

### Subsystem affected

- mechanical chassis
- front steering
- distance sensors
- camera view

### Causes

- bumper too close to tires,
- sensor bracket too low,
- cable protruding into wheel path,
- 3D-printed part deformation,
- screws too long.

### Symptoms

- tire rubs bumper at full steering,
- steering servo strains,
- distance sensor reads bumper or floor,
- robot behaves differently left vs right,
- visible mechanical contact.

### Impact

Medium.

### Mitigation

- Check full steering range by hand.
- Test bumper clearance at max left/right.
- Mount sensors high enough and clear from tire path.
- Use cable ties to secure wires.
- Inspect after crashes.

### Detection method

- Visual inspection.
- Full left/right steering test.
- Watch sensor readings while steering.

### Recovery behavior

- Stop run.
- Trim, reprint, or reposition bumper.
- Secure wires.

### Status

Previously observed and improved. Continue inspection.

---

## R-012 - I2C Address Conflict

### Description

Multiple VL53 sensors can have the same default address. If they start with conflicting addresses, the STM32 may fail to initialize them correctly.

### Subsystem affected

- VL53L4CD sensors
- VL53L8CH front sensor
- I2C bus
- XSHUT control
- STM32 firmware

### Causes

- sensors share default address,
- XSHUT sequence incorrect,
- firmware address assignment mismatch,
- sensor not powered during init,
- loose XSHUT wire.

### Symptoms

- one or more sensors not detected,
- distance values stuck or invalid,
- I2C bus error,
- wrong sensor reports wrong position,
- firmware hard fault or init failure.

### Impact

Medium to high.

### Mitigation

- Use XSHUT pins for controlled startup.
- Initialize sensors one at a time.
- Verify final address map in firmware.
- Label sensor cables physically.
- Add startup debug messages.
- Test each sensor individually.

### Detection method

- Print sensor initialization status.
- Run I2C scanner/debug routine.
- Check each sensor position with hand/object test.

### Recovery behavior

- Restart STM32.
- Check XSHUT wiring.
- Test sensors one-by-one.
- Correct firmware address table.

### Status

Medium. Final address map should be verified and documented.

---

## R-013 - Front Matrix False Corner Trigger

### Description

The front matrix may falsely detect a wall/corner too early because one cell sees an edge or irrelevant object.

### Subsystem affected

- VL53L8CH matrix
- corner detection
- Open Challenge logic
- obstacle challenge corner handling

### Causes

- one matrix cell sees side wall,
- floor reflection,
- obstacle edge,
- noisy reading,
- wrong row selected,
- valid-cell threshold too low.

### Symptoms

- robot turns before reaching corner,
- robot cuts corner too early,
- robot leaves correct lane,
- inconsistent trigger distance.

### Impact

High.

### Mitigation

- Require at least two valid in-band cells.
- Ignore known floor-band cells.
- Use median/filtered value.
- Combine with camera corner detection when available.
- Add minimum/maximum trigger distance window.

### Detection method

- Log matrix values approaching corner.
- Record trigger distance.
- Compare false triggers with matrix cell map.
- Review video frame at trigger moment.

### Recovery behavior

- Add cooldown to prevent repeated trigger.
- If trigger happens too soon during test, reduce sensitivity and require more valid cells.

### Status

Improved. Continue validation across all corner orientations.

---

## R-014 - Corner Detected Too Late

### Description

If the robot detects the corner too late, it may hit the wall or miss the turn.

### Subsystem affected

- front matrix sensor
- camera corner model
- lane-following logic
- speed control

### Causes

- front matrix filtering too strict,
- camera model misses corner line,
- robot speed too high,
- sensor mounted poorly,
- invalid matrix readings,
- timeout in sensor polling.

### Symptoms

- robot hits front wall,
- steering begins after correct turn point,
- turn radius not enough,
- repeated failure at same corner.

### Impact

High.

### Mitigation

- Tune corner trigger distance around final speed.
- Use camera + front matrix fusion.
- Reduce approach speed.
- Check front sensor alignment.
- Log trigger distance.
- Ensure matrix polling does not block too long.

### Detection method

- Mark trigger position on video.
- Measure distance to wall at trigger.
- Record speed and matrix value.

### Recovery behavior

- If front wall is closer than safe threshold, emergency turn or stop.
- Reduce speed until trigger is reliable.

### Status

Partially mitigated. Current front trigger is around ~100 cm but requires final measurement.

---

## R-015 - Over-Aggressive PDI Steering

### Description

The obstacle controller may steer too strongly when trying to pass red or green pillars.

### Subsystem affected

- obstacle strategy
- PDI controller
- steering servo
- mechanical steering
- YOLO target selection

### Causes

- high P gain,
- high derivative reaction,
- low steering clamp,
- target detected too late,
- speed too high,
- large error when pillar appears near frame edge.

### Symptoms

- full-lock steering,
- robot loses target,
- robot hits wall after avoiding pillar,
- oscillation around pillar,
- unstable recovery after pass.

### Impact

High.

### Mitigation

- Lower PDI gains.
- Clamp steering.
- Start reaction earlier.
- Reduce speed near pillars.
- Smooth detection error.
- Use target zone instead of exact center if needed.

### Detection method

- Log steering command during pillar approach.
- Review video for full-lock turns.
- Compare different PDI gain sets.

### Recovery behavior

- If steering saturates for too long, reduce command and return to lane following.
- If target lost, hold last valid detection only briefly.

### Status

Improved by softer gains and clamp, but still needs final obstacle run data.

---

## R-016 - Robot Loses Pillar Target

### Description

During avoidance, the pillar can leave the camera frame or detection can disappear for one or more frames.

### Subsystem affected

- YOLO pipeline
- obstacle controller
- target tracker
- steering logic

### Causes

- aggressive steering,
- pillar close to image edge,
- motion blur,
- temporary occlusion,
- confidence drop,
- robot speed too high.

### Symptoms

- steering command suddenly changes,
- robot stops avoiding obstacle,
- robot snaps back to lane following too early,
- robot oscillates.

### Impact

High.

### Mitigation

- Keep last valid detection for a short timeout.
- Use detection debounce.
- Reduce steering aggressiveness.
- React earlier when pillar is farther away.
- Use bounding-box area to estimate closeness.

### Detection method

- Log detection presence per frame.
- Review video with detection overlay.
- Count target-loss events.

### Recovery behavior

- Hold previous steering correction briefly.
- Return gradually to wall following after pass.
- Avoid using stale target for too long.

### Status

Open. Needs obstacle test logs.

---

## R-017 - Stop Command Not Executed

### Description

The robot may continue moving if the STOP command is missed or delayed.

### Subsystem affected

- motor command protocol
- STM32 firmware
- Jetson serial code
- Cytron motor driver
- safety behavior

### Causes

- UART frame dropped,
- serial bus busy,
- checksum failure,
- motor command wait conflict,
- STM32 command parser busy,
- software exception before stop.

### Symptoms

- robot continues moving after stop request,
- button stop delayed,
- end-of-run stop unreliable,
- robot moves during debugging.

### Impact

Very high.

### Mitigation

- Pulse STOP command multiple times.
- Use short delays between stop pulses.
- Keep emergency stop path simple.
- Test stop under sensor polling load.
- Avoid blocking code before stop.
- Consider STM32 timeout failsafe if no valid command is received.

### Detection method

- Run repeated stop tests.
- Trigger stop while front matrix is polling.
- Log stop command count and response.

### Recovery behavior

- Send repeated STOP frames.
- If software stop fails, use physical power cut during testing.
- Investigate serial log.

### Status

High. Pulsed STOP mitigation is already used.

---

## R-018 - Incorrect Motor Direction

### Description

Motor direction may be reversed compared to software expectation.

### Subsystem affected

- motor wiring
- MD10C direction input
- STM32 motor driver
- Jetson motor commands

### Causes

- motor wires reversed,
- firmware direction constant inverted,
- command mapping mismatch,
- test script uses old command mapping.

### Symptoms

- FORWARD command moves robot backward,
- REVERSE command moves robot forward,
- set-speed causes unexpected movement,
- robot leaves start area wrong direction.

### Impact

Medium to high.

### Mitigation

- Test direction before placing robot on field.
- Label motor wires.
- Keep command definitions in documentation.
- Use consistent test script.
- Confirm direction after firmware changes.

### Detection method

- Run motor command test with wheels lifted.
- Send forward/reverse and observe direction.
- Compare with encoder sign.

### Recovery behavior

- Stop immediately.
- Correct wiring or firmware direction mapping.
- Retest before autonomous run.

### Status

Medium. Previously observed start/direction issues were improved by command sequencing.

---

## R-019 - Encoder Feedback Error

### Description

Encoder readings may not match actual wheel movement.

### Subsystem affected

- motor encoder
- STM32 interrupt handling
- odometry logic
- motor degree commands

### Causes

- missed pulses,
- wrong pulses-per-revolution calculation,
- gear ratio not applied,
- A/B channel swapped,
- electrical noise,
- loose encoder wire.

### Symptoms

- degree command moves too far or too little,
- RPM value unrealistic,
- forward and reverse distance differ,
- odometry drift.

### Impact

Medium.

### Mitigation

- Validate encoder with known wheel rotations.
- Confirm quadrature direction.
- Apply correct gearbox reduction ratio.
- Use shielded or short signal wiring if needed.
- Compare commanded degrees with actual movement.

### Detection method

- Mark wheel and count rotations.
- Log encoder ticks for known movement.
- Run forward and reverse degree tests.

### Recovery behavior

- Use time/sensor-based fallback if encoder unreliable.
- Correct constants and retest.

### Status

Medium. Degree command behavior was improved after comparing Python and STM32 command formats.

---

## R-020 - Wiring Disconnect During Run

### Description

A loose cable can disconnect power, sensor, servo, motor, or communication signals.

### Subsystem affected

- all electrical subsystems

### Causes

- vibration,
- crashes,
- weak Dupont connectors,
- cable strain,
- robot motion pulling wire,
- poor cable routing.

### Symptoms

- sensor disappears,
- motor stops,
- servo stops,
- Jetson loses USB device,
- STM32 resets,
- intermittent behavior.

### Impact

Medium to high.

### Mitigation

- Use cable ties.
- Add strain relief.
- Avoid cables near wheels.
- Label connectors.
- Inspect wiring before each run.
- Use secure connectors where possible.

### Detection method

- Wiggle-test wires while system is powered.
- Check logs for intermittent disconnects.
- Inspect after every crash.

### Recovery behavior

- Stop test.
- Reconnect and secure wire.
- Retest subsystem before full run.

### Status

Medium. Continue inspection.

---

## R-021 - Repository File Too Large

### Description

Large images and videos can make the GitHub repository hard to clone or push.

### Subsystem affected

- documentation
- GitHub reproducibility
- collaboration

### Causes

- high-resolution PNG photos,
- large MP4 videos,
- uncompressed images,
- no Git LFS.

### Symptoms

- slow git operations,
- GitHub rejects files over size limit,
- repository becomes heavy,
- collaborators have trouble cloning.

### Impact

Low to medium.

### Mitigation

- Compress large images.
- Use optimized JPG for documentation display.
- Use Git LFS for large videos if needed.
- Upload videos to YouTube and link them.
- Keep original high-resolution images only when useful.

### Detection method

- Check file sizes before commit.
- Run `du -sh`.
- Watch GitHub push warnings.

### Recovery behavior

- Compress file.
- Move large file to Git LFS.
- Remove accidental large files from history if necessary.

### Status

Low. Monitor because current vehicle photos are large.

---

## R-022 - Missing Test Data

### Description

The robot may work, but documentation may lack enough test evidence to prove performance.

### Subsystem affected

- engineering journal
- test documentation
- judging reproducibility

### Causes

- runs not logged,
- videos not linked,
- lap times not recorded,
- tuning values not saved,
- success rates not calculated.

### Symptoms

- docs contain too many `[TODO]` values,
- impossible to compare versions,
- final behavior is not reproducible,
- judges cannot verify engineering process.

### Impact

Medium.

### Mitigation

- Use `docs/06-testing-and-tuning.md`.
- Record run ID, software version, result, lap count, time, and failure.
- Add video evidence.
- Add sensor calibration data.
- Add final tuning parameters.

### Detection method

- Review docs for `[TODO]`.
- Check whether each major claim has evidence.
- Ensure videos and photos are linked.

### Recovery behavior

- Run additional tests and fill missing tables.
- Add short notes even for failed runs.

### Status

Open. More test data should be added before final submission.

---

## R-023 - Parking Behavior Incomplete

### Description

The parking phase may not be fully implemented or tested.

### Subsystem affected

- obstacle challenge state machine
- sensor strategy
- motor control
- steering control
- final scoring

### Causes

- focus on lap navigation first,
- parking logic not yet tuned,
- insufficient parking test data,
- marker detection not finalized,
- final alignment strategy not complete.

### Symptoms

- robot completes laps but fails parking,
- robot stops outside parking area,
- robot touches parking markers,
- robot enters at wrong angle.

### Impact

High for Obstacle Challenge scoring.

### Mitigation

- Create dedicated parking state.
- Use side sensors to align parallel.
- Use speed reduction near parking.
- Add parking test video.
- Log final wall distance and angle.
- Add parking metrics to testing document.

### Detection method

- Run parking-only tests.
- Record final position.
- Measure parallel error.
- Count marker contact incidents.

### Recovery behavior

- If parking alignment is poor, stop safely instead of hitting markers.
- Retune entry angle and distance thresholds.

### Status

Open. Needs final implementation and evidence.

---

## R-024 - Model Not Trained for Competition Lighting

### Description

The YOLO model may perform well during development but fail in the actual competition environment if lighting or camera angle differs.

### Subsystem affected

- YOLO model
- dataset
- camera calibration
- obstacle behavior

### Causes

- small dataset,
- limited lighting variety,
- no examples from final field setup,
- different camera exposure,
- changed camera mount angle,
- model trained only on clean/centered pillars.

### Symptoms

- confidence drops in competition lighting,
- false positives increase,
- green/red confusion,
- late detection,
- bounding boxes unstable.

### Impact

High.

### Mitigation

- Capture training images from final robot camera.
- Include different distances, angles, and light conditions.
- Include partial occlusion and edge-of-frame examples.
- Test with final camera mount.
- Add validation videos with overlay.
- Tune confidence thresholds after final mounting.

### Detection method

- Run model on recorded competition-like footage.
- Compare detections across light conditions.
- Measure true/false positive rate.

### Recovery behavior

- Lower speed if detection uncertainty increases.
- Fall back to conservative wall following if no valid pillar exists.

### Status

Open. Needs final dataset/model documentation.

---

## R-025 - Sensor Calibration Drift After Mechanical Changes

### Description

When sensors are moved or bumper geometry changes, previous calibration values may no longer be valid.

### Subsystem affected

- VL53L4CD sensors
- VL53L8CH front sensor
- wall-following controller
- corner trigger
- obstacle distance logic

### Causes

- bumper redesign,
- sensor remounting,
- sensor angle shift after crash,
- screw loosening,
- new 3D-printed bracket,
- height change.

### Symptoms

- robot follows wall at wrong distance,
- front trigger distance changes,
- left/right readings not symmetric,
- obstacle response starts too late or early,
- previous tuning no longer works.

### Impact

High.

### Mitigation

- Recalibrate after mechanical changes.
- Mark final sensor positions.
- Tighten mounts.
- Use fixed-distance calibration procedure.
- Add calibration values to docs.
- Do not tune software until mechanical position is stable.

### Detection method

- Compare readings at known distances.
- Check before/after mechanical modifications.
- Record calibration table.

### Recovery behavior

- Re-run sensor calibration.
- Update thresholds and documentation.

### Status

High. Continue calibration after each sensor/bumper change.

---

## 7. Risk Mitigation Checklist Before Each Test Session

Use this checklist before running the robot on the field.

### Mechanical

- [ ] Wheels are clean.
- [ ] Steering moves left and right without binding.
- [ ] Servo horn and linkage screws are tight.
- [ ] Front bumper does not touch tires.
- [ ] Sensor mounts are fixed and not loose.
- [ ] No cables touch wheels or steering linkage.

### Power

- [ ] Motor battery charged.
- [ ] Jetson/UPS battery charged.
- [ ] Battery voltage recorded.
- [ ] Power connectors secure.
- [ ] No signs of overheating.
- [ ] Servo power stable.

### Sensors

- [ ] VL53L4CD sensors return valid values.
- [ ] VL53L8CH front matrix returns valid matrix.
- [ ] No side sensor stuck at `-1`.
- [ ] I2C sensors initialize successfully.
- [ ] Sensor readings match approximate real distances.

### Vision

- [ ] Camera starts correctly.
- [ ] YOLO detects red pillar.
- [ ] YOLO detects green pillar.
- [ ] Detection confidence is reasonable.
- [ ] Lighting is similar to expected run condition.
- [ ] Camera lens is clean.

### Communication

- [ ] STM32 serial port opens.
- [ ] Sensor command returns expected data.
- [ ] Motor forward command works.
- [ ] Motor stop command works.
- [ ] STOP command tested multiple times.
- [ ] No serial timeout before run.

### Software

- [ ] Correct challenge script selected.
- [ ] Correct model file selected.
- [ ] Correct serial ports selected.
- [ ] Correct speed setting selected.
- [ ] Logs enabled if testing.
- [ ] Emergency stop method ready.

---

## 8. Competition-Day Risk Checklist

Before the official run:

1. Charge all batteries.
2. Measure voltage.
3. Clean wheels.
4. Clean camera lens.
5. Check front bumper clearance.
6. Check steering center.
7. Check all sensor readings.
8. Check YOLO red/green detection.
9. Run motor forward/stop test.
10. Run steering left/right/center test.
11. Verify correct challenge mode.
12. Verify final speed setting.
13. Verify video/documentation evidence if needed.
14. Keep physical power-off method ready during testing.

---

## 9. Safe-State Strategy

A safe state is the behavior the robot should enter when it cannot trust its inputs or outputs.

### Safe-state triggers

| Trigger | Safe response |
|---|---|
| Repeated UART timeout | Send STOP repeatedly |
| Too many invalid sensors | Slow down or stop |
| Camera lost | Continue wall following only if safe; otherwise stop |
| Jetson exception | Stop motor if possible |
| Battery low | Stop testing |
| Servo stuck | Stop run |
| Unknown state-machine condition | Stop or return to straight-drive safe behavior |

### Minimum safe command

```text
STOP -> STOP -> STOP
```

Critical stop commands should be pulsed because one serial frame may be lost during heavy communication.

---

## 10. Risk Ownership

| Risk area | Owner / responsible focus |
|---|---|
| Mechanical risks | Team mechanical build and inspection |
| Power risks | Wiring, battery, and current validation |
| Sensor risks | STM32 firmware and calibration testing |
| Vision risks | YOLO model and Jetson camera pipeline |
| Communication risks | Jetson serial code and STM32 protocol |
| Strategy risks | Challenge state machine and tuning |
| Documentation risks | GitHub README, docs, journal, videos, photos |

---

## 11. Test Evidence Needed for Final Risk Closure

The following evidence is needed to close or reduce the highest risks:

| Risk | Evidence needed |
|---|---|
| Glare / lighting | Video or screenshots showing YOLO under different lighting |
| Bad pillar detection | Detection statistics for red and green pillars |
| Sensor noise | Calibration table and invalid-reading rate |
| UART timeout | Serial timeout count during full runs |
| Motor blocking | Camera FPS with motor commands active |
| Weak battery | Voltage before/after 3-minute run |
| Stop command failure | Repeated STOP test results |
| Front matrix false trigger | Matrix logs during corner approach |
| Parking incomplete | Parking test video and success table |
| Steering jitter | Servo current or observed stability test |

---

## 12. Change Log

| Date | Change | Notes |
|---|---|---|
| [TODO] | Initial risk register created | Includes glare, sensor noise, UART timeout, motor blocking, weak battery, and bad pillar detection |
| [TODO] | Add final test evidence | Fill after more field testing |
| [TODO] | Update probability/impact after competition-like tests | Replace estimated scores with measured experience |

---

## 13. Conclusion

The most important risks for ARBIBOT are not isolated component failures. They are system interaction failures:

- the motor command can block the camera loop,
- a weak battery can create sensor and steering problems,
- bad lighting can create bad obstacle decisions,
- a noisy sensor can create a wrong turn,
- a dropped UART frame can prevent a stop command,
- aggressive steering can cause the camera to lose the pillar target.

The robot has already improved by addressing several of these risks: ToF sensors replaced ultrasonic sensors, wider wheels replaced thinner slipping wheels, the front matrix logic was filtered, right-front/right-rear heading correction improved wall following, and critical commands are pulsed to reduce serial failure impact.

The remaining goal is to keep converting risks into test evidence. Each failure mode should have a mitigation, a detection method, and a test result proving whether the mitigation is effective.
