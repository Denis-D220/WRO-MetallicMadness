#!/usr/bin/env python3
"""
main_challenge_v7.py  --  v6, plus backing out of a wedge.

THE ONLY DIFFERENCE FROM v6 is the unwedge hook in the main loop: when the car
has stopped moving against something, straighten, reverse an inch, restore the
steering it had, and carry on. Everything else -- perception, the state
machine, the turn profiles, the PID -- is v6 unchanged, so a v7 run stays
comparable with a v6 run.

The detection and the maneuver live in unwedge.py, imported not copied: the
robot cannot OBSERVE hitting a wall (no bumper, no IMU, no encoder feedback,
no current sensing), so the trigger is an inference, and the inference is the
part worth getting right once. See behavior_manager.REVERSE_* and STUCK_*.

NOTE ON KEEPING TWO MAINS. This file is a copy of v6's loop, so a fix made in
one will NOT reach the other. Deliberate for now -- v6 stays as the known-good
program while v7 is on trial -- but it is a debt, not a design: collapse them
behind B.REVERSE_UNWEDGE once v7 has proved out on the mat.

v6's own header follows.

three inputs, one controller.

    CORNER CLASSIFIER   names the turn and its profile
    WALL CLASSIFIER     names the kind of steering correction
    ToF SENSORS + PID   decide how much, and when the car has arrived

WHAT CHANGED FROM v5, AND WHY

v5 found corners with the LINE DETECTOR: it looked for the blue/orange corner
lines, derived a direction from the nearest line's colour, estimated "closeness"
from where that line sat in the image, and then drove every corner with the same
blind arc. Four things had to go right, and on the track they routinely did not:

  * the direction flipped left/right within a single approach (left,left,left,
    right,right -- and it committed on the last frame, into a wall);
  * `both=0` on almost every approach, so the direction rested on the weakest
    evidence available -- a lone line whose colour is unreliable;
  * closeness is a position IN THE IMAGE, not a distance, so it said "a corner is
    ahead" and never "you are there now" -- it once fired with the front ToF
    still reading 1082 mm;
  * one arc for every corner produced exits anywhere from R=33 to R=505.

The corner CLASSIFIER answers all of that in one output. It names the turn
(left/right) AND its severity (close/normal/open), so the direction needs no
vote, the severity picks the steering profile, and the ToF says when to go. Same
approach as the wall classifier, which is the one perception component that has
been consistently right on this robot.

WHAT IS UNCHANGED, DELIBERATELY
    the state machine        behavior_manager.py  (v5's, with the classifier
                             route added -- the priority order, the ToF vetoes,
                             the re-arm guard and the emergency escalation all
                             still apply)
    race bookkeeping         race_manager.py
    the ToF reader and       imported from main_challenge_01_v4 / v5, never
    wall follower            copied -- two copies always drift

STATUS: the logic in behavior_manager.py and race_manager.py is unit-tested off
robot. EVERYTHING IN THIS FILE touches hardware and has NOT been run. Bench it
with DRY_RUN = True (no motor) before letting it drive.
"""

import time

import cv2
import Jetson.GPIO as GPIO

import behavior_manager as B
from behavior_manager import BehaviorManager, Percept
from race_manager import RaceManager
from color_tuning_Dualcam import (CAM_WIDTH, apply_color_tuning,
                                  camera_pipeline, load_params)
from motor_driver import MotorSerial
from pid_line_follower import LineFollowerPID
from servo_controller import ServoController
from robot_io import (CornerTriggers, HeadingWallFollower, SensorAdapter,
                      button_pressed, set_pin, sleep_with_abort, valid_tof,
                      wait_for_button_press)
# The dual-camera pillar PID, IMPORTED not copied -- pillar_pid_dualcam.py is
# the one place the strip geometry and the steering sign live, and it carries
# the --selftest that proves them.
from unwedge import StuckDetector, reverse_nudge
from pillar_pid_dualcam import (PillarPID, Strip, choose as choose_pillar,
                                detect as detect_pillars, placed as pillar_placed)


# =========================================================
# HARDWARE / PORTS
# =========================================================

MOTOR_PORT = "/dev/ttyUSB0"
SENSOR_PORT = "/dev/ttyUSB0"     # same STM32 -> shared serial, sensor owns reads
SERVO_PORT = "/dev/ttyACM0"
SERVO_BAUD = 9600
# The button pin lives with set_pin()/button_pressed() in robot_io, which is
# the only place that reads it. Nothing here to edit.

DRY_RUN = False

# Shutdown only. The motor runs in forward_continuous, so a lost STOP frame
# means the car keeps going after the program exits -- see the finally block.
MOTOR_STOP_TRIES = 5
MOTOR_STOP_TIMEOUT_S = 0.4                  # True = never command the motor (bench testing)

# =========================================================
# MODELS -- only two now
# =========================================================

MODELS_DIR = "/home/daniel/WRO2026-MetallicMadness/Models"
CORNER_MODEL = f"{MODELS_DIR}/Best_Model_Corner_v3.pt"   # corner-TYPE classifier
WALL_MODEL = f"{MODELS_DIR}/Walls_model_v3.pt"
# Loaded ONLY when behavior_manager.PILLAR_PID_ENABLED. A detector on two full
# 1280x720 frames is far heavier than the two 224px classifiers, so challenge 1
# -- which has no pillars -- should not pay for it.
PILLAR_MODEL = f"{MODELS_DIR}/Pillar_model_v1.pt"

PANE = B.PANE
CAM_IDS = B.CAM_IDS
CLASS_BACKS_TRIGGER_FRAMES = B.CLASS_BACKS_TRIGGER_FRAMES
CORNER_EVERY = B.CORNER_EVERY
WALL_EVERY = B.WALL_EVERY
TURN_WALL_HZ = B.TURN_WALL_HZ

CRUISE_SPEED_PCT = B.CRUISE_SPEED_PCT
FINISH_DRIVE_S = B.FINISH_DRIVE_S

WALL_FOLLOW_MODE = B.WALL_FOLLOW_MODE
WALL_TARGET_MM = B.WALL_TARGET_MM
WF_KH, WF_KD = B.WF_KH, B.WF_KD
WF_RR_OFFSET_MM = B.WF_RR_OFFSET_MM
WF_OUT_LIMIT = B.WF_OUT_LIMIT
WF_DEADBAND = B.WF_DEADBAND
WF_HEADING_MAX_MM = B.WF_HEADING_MAX_MM

PID_KP, PID_KI, PID_KD = B.PID_KP, B.PID_KI, B.PID_KD
PID_OUT_LIMIT = B.PID_OUT_LIMIT
PID_DEADBAND = B.PID_DEADBAND
PID_MEDIAN_WINDOW = B.PID_MEDIAN_WINDOW
PID_SLEW_LIMIT = B.PID_SLEW_LIMIT
PID_KH = B.PID_KH

SIDE_MAX_AGE_S = B.SIDE_MAX_AGE_S
LR_HZ = B.LR_HZ
FRONT_HZ = B.FRONT_HZ
DEBUG_HZ = B.DEBUG_HZ


# =========================================================
# PERCEPTION -- two classifiers, one composite
# =========================================================

class Perception:
    """Runs the corner-type and wall classifiers and returns one Percept."""

    def __init__(self):
        from ultralytics import YOLO
        print(f"Loading corner classifier {CORNER_MODEL} ...")
        self.corner = YOLO(CORNER_MODEL)
        print(f"Loading wall classifier {WALL_MODEL} ...")
        self.wall = YOLO(WALL_MODEL)
        self.corner.to("cuda")
        self.wall.to("cuda")
        self.pillar = None
        self.pillar_pid = None
        self.strip = None
        if B.PILLAR_PID_ENABLED:
            print(f"Loading pillar detector {PILLAR_MODEL} ...")
            self.pillar = YOLO(PILLAR_MODEL)
            self.pillar.to("cuda")
            self.pillar_pid = PillarPID()
            self.strip = Strip(frame_w=CAM_WIDTH,
                               per_cam=B.PILLAR_PID_SECTIONS)
        self._last_pillar = None
        self.cam_params = {sid: load_params(sensor_id=sid) for sid in CAM_IDS}
        self._frame = 0
        self._last = {"corner": (None, 0.0), "wall": (None, 0.0)}
        self.frames_since_turn_class = 10**6   # see CLASS_BACKS_TRIGGER
        self._warned = False

    def announce_pillars(self):
        """Say whether the pillar half of the controller is live."""
        if not B.PILLAR_PID_ENABLED:
            print("[v7] pillar PID OFF -- detector not loaded, no cost. "
                  "behavior_manager.PILLAR_PID_ENABLED turns it on.")
            return
        tgt = B.PILLAR_PID_TARGET_SECTION
        print(f"[v7] pillar PID ON  -- {PILLAR_MODEL}")
        print(f"       strip {self.strip.n_sections} sections across "
              f"{self.strip.width}px, every {B.PILLAR_EVERY} frame(s)")
        print(f"       targets: green -> section {tgt['green']}, "
              f"red -> section {tgt['red']}")

    def announce_tuning(self):
        """State what the models are actually being fed. Once, at startup."""
        if not B.CAMERA_TUNING:
            print("[v7] camera colour tuning OFF -- models see raw frames, "
                  "which is what they were trained on")
            return
        print(f"[v7] camera colour tuning ON, stage={B.CAMERA_TUNING_STAGE!r}")
        for sid in CAM_IDS:
            print(f"       cam{sid}: {self.cam_params[sid]}")
        print("[v7] WARNING: collect_corner_classes.py and "
              "collect_steering_classes.py save UNTUNED frames,")
        print("     so these weights never saw a colour-corrected image. "
              "Expect the classifiers to get")
        print("     WORSE, not better, until the collectors tune too and "
              "the models are retrained.")

    def check_classes(self):
        """Warn once if a model's classes are not the ones we expect."""
        want_corner = set(B.CORNER_TURN_PROFILE) | {"straight"}
        got = set(getattr(self.corner, "names", {}).values())
        if got and not got <= want_corner:
            print(f"\nWARNING: corner model classes {sorted(got)}\n"
                  f"         are not all in {sorted(want_corner)} -- an unknown "
                  "class can never pick a turn profile.\n")
        got_w = set(getattr(self.wall, "names", {}).values())
        if got_w and got_w != set(B.WALL_STEER_US):
            print(f"\nWARNING: wall model classes {sorted(got_w)}\n"
                  f"         differ from {sorted(B.WALL_STEER_US)}.\n")

    def composite(self, frames):
        """The layout BOTH classifiers were trained on: cam0 | cam1, 960x270.

        Colour tuning is applied here, or not at all -- see
        behavior_manager.CAMERA_TUNING. Each camera gets its OWN parameters even
        in "resized" mode, because the composite spans both and one camera's
        gains are wrong for the other's half.
        """
        tune = B.CAMERA_TUNING
        full = tune and B.CAMERA_TUNING_STAGE == "full"

        panes = []
        for sid in CAM_IDS:
            frame = frames.get(sid)
            if frame is None:
                continue
            if full:
                frame = apply_color_tuning(frame, self.cam_params[sid])
            pane = cv2.resize(frame, PANE)
            if tune and not full:
                pane = apply_color_tuning(pane, self.cam_params[sid])
            panes.append(pane)
        if not panes:
            return None
        return panes[0] if len(panes) == 1 else cv2.hconcat(panes)

    @staticmethod
    def _top1(model, image):
        r = model(image, verbose=False)[0]
        if r.probs is None:
            return None, 0.0
        i = int(r.probs.top1)
        return r.names[i], float(r.probs.top1conf)

    def pillar_update(self, frames, percept, force=False):
        """Run the pillar detector and let the dual-cam PID decide the steering.

        Fills the Percept fields the state machine already gates on -- colour,
        area, centre -- plus what the PID adds: the steering it wants and
        whether the pillar has been placed. When disabled, or on a frame the
        cadence skips, the LAST result is reused rather than cleared: dropping
        to "no pillar" every other frame would break the consecutive-frame
        agreement PILLAR_ACTIONABLE_FRAMES needs.
        """
        if self.pillar is None:
            return None
        # force=True is hold_turn: self._frame only advances in update(), which
        # is not running during a turn, so the cadence check would freeze on
        # whatever it last evaluated to and hand back a stale pillar forever.
        if force or self._frame % B.PILLAR_EVERY == 0 or self._last_pillar is None:
            found = detect_pillars(self.pillar, frames, self.strip,
                                   B.PILLAR_PID_MIN_CONF)
            chosen = choose_pillar(found)
            steer, _info = self.pillar_pid.update(self.strip, chosen)
            self._last_pillar = (chosen, steer,
                                 bool(chosen and pillar_placed(self.strip,
                                                               chosen, found)))
        chosen, steer, done = self._last_pillar
        if chosen is None:
            percept.pillar_color = None
            percept.pillar_area = 0.0
            percept.pillar_cx = None
            percept.pillar_steer_us = 0.0
            percept.pillar_placed = False
            return None
        percept.pillar_color = chosen.colour
        percept.pillar_area = chosen.area
        percept.pillar_cx = 0.5 * (chosen.box[0] + chosen.box[2])
        percept.frame_w = CAM_WIDTH
        percept.pillar_steer_us = steer
        percept.pillar_placed = done
        return chosen

    def wall_only(self, frames):
        """(class, conf) from the WALL model alone -- for use during a turn.

        The corner classifier is skipped: the turn type is already committed and
        re-running it mid-turn would only slow the loop that is watching for the
        car to come parallel.
        """
        sample = self.composite(frames)
        if sample is None:
            return None, 0.0
        return self._top1(self.wall, sample)

    def update(self, frames, percept):
        """Fill in the model fields. Returns the composite for the debug view."""
        self._frame += 1
        sample = self.composite(frames)
        if sample is None:
            return None

        if self._frame % CORNER_EVERY == 0:
            self._last["corner"] = self._top1(self.corner, sample)
        if self._frame % WALL_EVERY == 0:
            self._last["wall"] = self._top1(self.wall, sample)

        name, conf = self._last["corner"]
        # 'straight' is a real answer meaning "no corner here" -- pass it through
        # as no class rather than as a turn the profile table cannot serve.
        percept.corner_class = name if name in B.CORNER_TURN_PROFILE else None
        percept.corner_class_conf = conf if percept.corner_class else 0.0
        if percept.corner_class:
            self.frames_since_turn_class = 0
        else:
            self.frames_since_turn_class += 1

        percept.wall_class, percept.wall_conf = self._last["wall"]
        return sample


# =========================================================
# MAIN
# =========================================================

def main():
    set_pin()
    print("Button pin ready.")

    servo = ServoController(port=SERVO_PORT, baud_rate=SERVO_BAUD)
    servo.connect()
    servo.center_steering()

    # Sweep the wheels once so you can see from the mat that the program is
    # alive, without watching a console. Ends centred -- a car left on lock
    # would drive its first metre sideways.
    for u in (-200.0, 200.0, -200.0, 200.0):
        servo.steer_delta(u, channel=0)
        time.sleep(0.25)
    servo.center_steering()

    # The sensor object OWNS /dev/ttyUSB0 and does the reading; the motor reuses
    # that same serial and only WRITES, guarded by the sensor's lock. The two
    # devices are one STM32 on one port -- exactly v4/v5's arrangement.
    sensors = SensorAdapter(port=SENSOR_PORT, lr_hz=LR_HZ, front_hz=FRONT_HZ)
    sensors.open()
    sensors.set_front_enabled(False)      # heavy matrix poll off through start

    motor = MotorSerial(MOTOR_PORT, debug=False)
    shared = SENSOR_PORT == MOTOR_PORT
    if shared:
        motor.ser = sensors.ser
        motor_lock = sensors.write_lock
        print("[v7] motor sharing the sensor serial link")
    else:
        motor.open()
        motor_lock = None

    perception = Perception()
    perception.check_classes()
    perception.announce_tuning()
    perception.announce_pillars()

    caps = {}
    for sid in CAM_IDS:
        cap = cv2.VideoCapture(camera_pipeline(sid), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            for c in caps.values():
                c.release()
            servo.close(); sensors.close(); GPIO.cleanup()
            raise SystemExit(f"Could not open camera sensor-id={sid}. "
                             "Try: sudo systemctl restart nvargus-daemon")
        caps[sid] = cap

    race = RaceManager()
    behavior = BehaviorManager(cruise_speed=CRUISE_SPEED_PCT,
                               turn_steer_us=B.TURN_STEER_US)
    triggers = CornerTriggers()
    pid = LineFollowerPID(kp=PID_KP, ki=PID_KI, kd=PID_KD,
                          out_limit=PID_OUT_LIMIT, deadband=PID_DEADBAND,
                          median_window=PID_MEDIAN_WINDOW,
                          slew_limit=PID_SLEW_LIMIT,
                          kh=PID_KH, rr_offset=WF_RR_OFFSET_MM,
                          offset_ref_mm=B.WF_RR_OFFSET_REF_MM,
                          heading_max=WF_HEADING_MAX_MM)
    hwf = HeadingWallFollower(target=WALL_TARGET_MM, rr_offset=WF_RR_OFFSET_MM,
                              kh=WF_KH, kd=WF_KD, out_limit=WF_OUT_LIMIT,
                              deadband=WF_DEADBAND,
                              heading_max=WF_HEADING_MAX_MM,
                              median_window=PID_MEDIAN_WINDOW)

    speed_now = 0

    def _motor(fn, *a):
        """All motor writes are fire-and-forget under the shared-bus lock."""
        if DRY_RUN:
            return
        if motor_lock:
            with motor_lock:
                fn(*a, wait_response=False)
        else:
            fn(*a, wait_response=False)

    def motor_go(pct):
        _motor(motor.set_speed, pct)
        _motor(motor.forward_continuous)

    def motor_stop():
        _motor(motor.stop)

    def motor_back():
        """Backwards at the emergency crawl -- THIS MOVE IS BLIND.

        There is no rear sensor on this robot, so nothing can see what is
        behind the car. Slow and brief is the entire safety argument; the
        worst case is meant to be touching whatever it just came off. See
        REVERSE_NUDGE_S.
        """
        _motor(motor.set_speed, B.REVERSE_SPEED_PCT)
        _motor(motor.reverse_continuous)

    def motor_pulse(fn, times=4, gap=0.05):
        """Repeat a start/stop command so one dropped frame cannot stall it."""
        for _ in range(times):
            fn()
            time.sleep(gap)

    def read_frames():
        out = {}
        for sid in CAM_IDS:
            ok, f = caps[sid].read()
            if ok and f is not None:
                out[sid] = f
        return out

    def turn_complete(snap) -> bool:
        """Is the car PARALLEL to the wall yet? (heading, not position)

        The earlier version asked whether both side distances were inside a lane
        band, which is a POSITION test -- and a car can finish a turn perfectly
        parallel while still close to a wall. On the track that rejected every
        real completion (exits at R=58/43/36) so every turn ran to its time
        limit. RF and RR match only when the car is parallel, so their difference
        is the heading signal. WF_RR_OFFSET_MM is what makes RF == RR + offset
        mean parallel for this mounting; while it is 0.0 (uncalibrated)
        TURN_CLOSED_LOOP stays False and this is never consulted.
        """
        rf = valid_tof(snap.get("right_front_mm"))
        rr = valid_tof(snap.get("right_rear_mm"))
        if rf is None or rr is None:
            return None         # THREE-VALUED: True parallel / False measurably
        off = WF_RR_OFFSET_MM or pid.rr_offset_estimate()
        if not off:
            return None         # no opinion yet -- the timer owns the turn
        rng = max(1.0, 0.5 * (rf + rr))
        if rng < B.TURN_DONE_MIN_RANGE_MM:
            return None         # too close to measure heading -- see
        if B.WF_RR_OFFSET_REF_MM > 0:
            off = off * rng / B.WF_RR_OFFSET_REF_MM
        limit = B.TURN_DONE_HEADING_MM * rng / B.TURN_DONE_HEADING_REF_MM
        return abs(rf - (rr + off)) <= limit

    def hold_turn(max_s, base_steer=None, turn_dir=None):
        """Hold the steering lock for the turn. Returns (elapsed, reason, mean).

        (None, None, 1.0) if the button aborted. With TURN_CLOSED_LOOP off and
        TURN_WALL_ASSIST off this is a plain timed turn -- the behaviour every
        previous run was tuned against.

        `mean` is the average of the wall-assist multiplier over the turn: 1.00
        means the full profile lock was held throughout. The caller scales the
        drawback by it, because a turn that was eased accumulated less rotation
        and so has less to cancel.
        """
        start = time.monotonic()
        aligned = 0
        scale, scale_sum, scale_n = 1.0, 0.0, 0
        next_wall = 0.0
        why_wall = ""
        assist = (B.TURN_WALL_ASSIST and base_steer is not None
                  and turn_dir in ("left", "right") and TURN_WALL_HZ > 0)
        # The pillar rides along on the same camera read (see
        # TURN_PILLAR_ASSIST). bias is held between its slower ticks.
        pillar_on = (B.TURN_PILLAR_ASSIST and B.PILLAR_PID_ENABLED
                     and perception.pillar is not None and base_steer is not None)
        bias, tick, pillar_note = 0.0, 0, ""
        last_cmd = base_steer if base_steer is not None else 0.0

        def finish(elapsed, reason):
            mean = scale_sum / scale_n if scale_n else 1.0
            if assist and mean < 0.995:
                reason = f"{reason}; wall eased x{mean:.2f}"
                if why_wall:
                    reason = f"{reason} [{why_wall}]"
            if pillar_note:
                reason = f"{reason}; {pillar_note}"
            return elapsed, reason, mean, bias

        # Scaled to THIS profile: an absolute floor longer than the whole hold
        # deletes the check. See TURN_MIN_TIME_FRAC.
        min_t = min(B.TURN_MIN_TIME_S, B.TURN_MIN_TIME_FRAC * max_s)

        while True:
            if button_pressed():
                return None, None, 1.0, 0.0
            elapsed = time.monotonic() - start
            over = elapsed - max_s
            if over >= 0.0:
                snap_x = sensors.snapshot()
                near = [d for d in (valid_tof(snap_x.get("left_mm")),
                                    valid_tof(snap_x.get("right_front_mm")))
                        if d is not None and d <= B.TURN_EXTEND_MIN_SIDE_MM]
                may_extend = (B.TURN_CLOSED_LOOP and B.TURN_EXTEND_MAX_S > 0
                              and over < B.TURN_EXTEND_MAX_S
                              and not near
                              and turn_complete(snap_x) is False)
                if not may_extend:
                    return finish(elapsed, f"timed {max_s:.1f}s"
                                  + (f" +{over:.1f}s not round" if over >= 0.05
                                     else ""))

            # The parallel test runs FIRST every iteration, so the camera work
            # below can only delay it by one loop period -- never skip it.
            if B.TURN_CLOSED_LOOP and elapsed >= min_t:
                aligned = aligned + 1 if turn_complete(sensors.snapshot()) is True else 0
                if aligned >= B.TURN_DONE_HOLD_FRAMES:
                    return finish(elapsed, "sensors: parallel")

            now = time.monotonic()
            if (assist or pillar_on) and now >= next_wall:
                next_wall = now + 1.0 / TURN_WALL_HZ
                tick += 1
                frames = read_frames()          # ONE read, shared by both

                if pillar_on and tick % B.TURN_PILLAR_EVERY == 0:
                    scratch = Percept()
                    perception.pillar_update(frames, scratch, force=True)
                    # A turn is a committed arc; the pillar gets a smaller
                    # share of it than it does on a straight.
                    new_bias = max(-B.TURN_PILLAR_BIAS_LIMIT_US,
                                   min(B.TURN_PILLAR_BIAS_LIMIT_US,
                                       B.pillar_bias(scratch)))
                    if new_bias != bias:
                        bias = new_bias
                        pillar_note = (f"pillar {scratch.pillar_color} "
                                       f"{bias:+.0f}us" if bias else "")

                wname, wconf = perception.wall_only(frames) if assist else (None, 0.0)
                snap = sensors.snapshot()
                new_scale, why = B.turn_wall_scale(
                    turn_dir, wname, wconf,
                    valid_tof(snap.get("left_mm")),
                    valid_tof(snap.get("right_front_mm")),
                    prev=scale)
                scale_sum += new_scale
                scale_n += 1
                if why:
                    why_wall = why
                scale = new_scale
                cap = max(B.TURN_STEER_US, abs(base_steer))
                want = max(-cap, min(cap, base_steer * scale + bias))
                if abs(want - last_cmd) >= 1.0:
                    last_cmd = want
                    servo.steer_delta(want, channel=0)
            time.sleep(0.005 if (assist or pillar_on) else 0.01)

    def sides_fresh(snap) -> bool:
        """Has the side pair actually been updated recently? (see SIDE_MAX_AGE_S)

        A frozen reading is indistinguishable from a real one by value alone --
        L=480 R=486 held for six frames looked like a perfectly centred car.
        """
        age = snap.get("lr_age_s")
        return age is None or age <= SIDE_MAX_AGE_S

    def wall_follow_steer(snap):
        """The ToF answer to 'how much'. Centre mode needs no direction."""
        fresh = sides_fresh(snap)
        left = valid_tof(snap.get("left_mm"))
        right = valid_tof(snap.get("right_front_mm"))
        rear = valid_tof(snap.get("right_rear_mm"))
        outer = race.outer_wall
        outer_dist = None
        if outer is not None:
            od = right if outer == "right" else left
            if od:
                outer_dist = od
        if not fresh:
            return pid.last_command(), outer_dist
        if WALL_FOLLOW_MODE == "center" or outer is None:
            if left is None or right is None:
                return None, outer_dist
            # rf = right_FRONT (already `right`), rr = right_rear.
            u, info = pid.update(left, right, right, rear)
            return (u if info.get("valid") else None), outer_dist
        u, info = hwf.update(right, rear)
        if not info.get("valid"):
            return None, outer_dist
        return u, (info["rf"] if outer == "right" else outer_dist)

    # Diagnostics for the two silent failure modes. Both were invisible in the
    # logs: a frozen side pair looked like a centred car, and an uncalibrated
    # heading term looks exactly like no heading term at all.
    diag = {"stale_run": 0, "stale_total": 0, "stale_worst": 0,
            "offset_announced": False,
            "front_run": 0, "front_total": 0, "front_worst": 0,
            "front_announced": False,
            "front_close_run": 0}

    def note_front(p):
        """How long has the front matrix been saying nothing? (see diag)"""
        if p.front_mm is not None:
            if diag["front_run"] >= B.CORNER_FRONT_DEAD_FRAMES:
                print(f"    [front] back after {diag['front_run']} dead frames")
            diag["front_run"] = 0
            return
        diag["front_run"] += 1
        diag["front_total"] += 1
        diag["front_worst"] = max(diag["front_worst"], diag["front_run"])
        # Announce the crossing ONCE per outage, at the point where the corner
        # logic stops waiting for the front and lets the classifier commit
        # unaided -- that is the moment the run changes character.
        if diag["front_run"] == B.CORNER_FRONT_DEAD_FRAMES:
            print(f"    [front] MATRIX DEAD for "
                  f"{B.CORNER_FRONT_DEAD_FRAMES} frames -- corners now commit "
                  f"on the classifier alone")

    def note_sides(snap):
        if sides_fresh(snap):
            if diag["stale_run"] >= 3:
                print(f"    [sides] {diag['stale_run']} frames of STALE L/R "
                      f"(held steering through it)")
            diag["stale_run"] = 0
        else:
            diag["stale_run"] += 1
            diag["stale_total"] += 1
            diag["stale_worst"] = max(diag["stale_worst"], diag["stale_run"])
        if not diag["offset_announced"] and (off := pid.rr_offset_estimate()):
            diag["offset_announced"] = True
            print(f"*** heading term LIVE: learned RF-RR offset = {off:+.0f}mm "
                  f"(kh={PID_KH}) ***")

    stuck = StuckDetector()
    aborted = False
    try:
        print("Warming up cameras + models ...")
        t0 = time.monotonic()
        for _ in range(15):
            f = read_frames()
            if f:
                perception.update(f, Percept())
        print(f"warm-up done in {time.monotonic() - t0:.1f}s")

        print(f"v7 ready. DRY_RUN={DRY_RUN}. Press the button to START...")
        wait_for_button_press()
        print("START")

        servo.center_steering()
        pid.reset()
        hwf.reset()
        _motor(motor.forward_continuous)
        for _ in range(4):
            motor_go(CRUISE_SPEED_PCT)
            time.sleep(0.04)
        speed_now = CRUISE_SPEED_PCT
        # Motor going -> safe to turn the front matrix on. It stays off through
        # the start pulses so its scan cannot eat a start frame on the shared bus.
        sensors.set_front_enabled(True)

        last_dbg = 0.0
        dbg_interval = 1.0 / DEBUG_HZ if DEBUG_HZ > 0 else 0.0
        loop_hz, last_t = 0.0, 0.0

        while not race.finished:
            frames = read_frames()
            if not frames:
                continue
            if button_pressed():
                aborted = True
                break

            now = time.monotonic()
            if last_t:
                dt = now - last_t
                if dt > 0:
                    loop_hz = 0.9 * loop_hz + 0.1 / dt if loop_hz else 1.0 / dt
            last_t = now

            percept = Percept()
            sample = perception.update(frames, percept)
            if sample is None:
                continue
            # Pillars run on the FULL frames, not the classifier composite --
            # it is a detector, and 480x270 panes throw away the resolution the
            # boxes need. No-op when PILLAR_PID_ENABLED is off.
            seen_pillar = perception.pillar_update(frames, percept)

            snap = sensors.snapshot()
            percept.left_mm = valid_tof(snap.get("left_mm"))
            percept.right_mm = valid_tof(snap.get("right_front_mm"))
            percept.front_mm = (valid_tof(snap.get("front_mm"))
                                if snap.get("front_valid") else None)
            percept.finishing = race.finishing
            percept.direction = race.direction

            note_sides(snap)
            u_pid, outer_dist = wall_follow_steer(snap)
            fired = triggers.update(outer_dist, snap, race.direction is not None)
            # See CLASS_BACKS_TRIGGER_FRAMES: a ToF trigger only counts when
            # the classifier has recently agreed there is a corner here.
            class_backs = (perception.frames_since_turn_class
                           <= CLASS_BACKS_TRIGGER_FRAMES)
            front_agrees = (percept.front_mm is None
                            or 0 < percept.front_mm <= B.CORNER_FRONT_MM)
            if (percept.front_mm is not None
                    and 0 < percept.front_mm <= B.CORNER_FRONT_MM):
                diag["front_close_run"] += 1
            else:
                diag["front_close_run"] = 0
            front_backs = (diag["front_close_run"]
                           >= B.FRONT_BACKS_TRIGGER_FRAMES)
            percept.corner_trigger = (fired is not None
                                      and race.can_count_corner()
                                      and (class_backs or front_backs)
                                      and front_agrees)

            note_front(percept)
            decision = behavior.update(percept)

            if decision.note_pillar:
                race.note_pillar(decision.note_pillar)

            # --- steering ---
            steer = decision.steer_us
            if decision.owner in ("pid", "wall") and u_pid is not None:
                if decision.owner == "pid":
                    steer = u_pid
                elif (u_pid > 0) == (steer > 0):
                    steer = u_pid if abs(u_pid) > abs(steer) else steer

            if decision.nudge_us:
                lean = steer + decision.nudge_us
                lean = max(-PID_OUT_LIMIT, min(PID_OUT_LIMIT, lean))
                if steer > 0:
                    lean = max(0.0, lean)
                elif steer < 0:
                    lean = min(0.0, lean)
                steer = lean

            bias = (0.0 if decision.state in (B.CORNER_TURN, B.EMERGENCY)
                    else B.pillar_bias(percept))
            bias_effect = 0.0
            if bias:
                before = steer
                steer = max(-B.PILLAR_BIAS_CEILING_US,
                            min(B.PILLAR_BIAS_CEILING_US, steer + bias))
                bias_effect = steer - before

            if decision.state == B.CORNER_TURN:
                turn_dir = decision.turn or race.direction
                # The classifier IS the direction observation -- a WRO loop turns
                # the same way at every corner, so lock from the first one.
                if race.direction is None and turn_dir in ("left", "right"):
                    race.force_direction(turn_dir, "corner-1 classifier")
                    print(f"*** direction LOCKED {turn_dir} "
                          f"(outer wall = {race.outer_wall}) ***")
                if decision.count_corner:
                    if not race.count_corner(fired or "classifier"):
                        print("    (corner debounced -- not turning again)")
                        # cancelled, NOT finished: no turn was driven, so the
                        # arm this commit spent has to go back. See
                        # BehaviorManager.corner_turn_cancelled.
                        behavior.corner_turn_cancelled()
                        continue
                    print(f"*** corner {race.corners_done}/{race.total_corners} "
                          f"(lap {race.lap}) {turn_dir} "
                          f"[{decision.reason}] ***")

                if decision.speed_pct != speed_now:
                    motor_go(decision.speed_pct)
                    speed_now = decision.speed_pct
                servo.steer_delta(steer, channel=0)
                turned, why_end, wall_mean, end_bias = hold_turn(
                    decision.turn_time_s or B.TURN_TIME_S, steer, turn_dir)
                if turned is None:
                    aborted = True
                    break
                print(f"    turn ended after {turned:.2f}s ({why_end})")

                if decision.drawback_us and decision.drawback_time_s:
                    back = decision.drawback_us * wall_mean
                    if B.DRAWBACK_SCALES_WITH_TIME:
                        profile_s = decision.turn_time_s or B.TURN_TIME_S
                        frac = turned / profile_s if profile_s > 0 else 1.0
                        frac = max(B.DRAWBACK_TIME_FRAC_MIN,
                                   min(B.DRAWBACK_TIME_FRAC_MAX, frac))
                        back *= frac
                    back = -back if steer > 0 else back
                    if end_bias:
                        merged = back + end_bias
                        back = (max(0.0, min(back, merged)) if back > 0
                                else min(0.0, max(back, merged)))
                    servo.steer_delta(back, channel=0)
                    if sleep_with_abort(decision.drawback_time_s):
                        aborted = True
                        break
                servo.center_steering()
                motor_go(CRUISE_SPEED_PCT)
                speed_now = CRUISE_SPEED_PCT
                pid.reset()
                hwf.reset()
                triggers.reset_after_corner()
                behavior.corner_turn_finished()
                continue

            wall_clamp = 0.0
            if B.WALL_CLAMP_EVERY_PATH:
                before_clamp = steer
                steer, clamp_notes = B.constrain_toward_walls(
                    steer, percept.left_mm, percept.right_mm)
                wall_clamp = steer - before_clamp

            if B.REVERSE_UNWEDGE and stuck.update(
                    percept.left_mm, percept.right_mm,
                    sides_fresh(snap), speed_now > 0):
                print(f"    *** WEDGED on the {stuck.side} "
                      f"(L={percept.left_mm} R={percept.right_mm}) -- "
                      f"straighten, back until clear "
                      f"(<={B.REVERSE_NUDGE_S:.2f}s), "
                      f"restore steer={steer:+.0f} "
                      f"[attempt {stuck.attempts}/{B.REVERSE_MAX_ATTEMPTS}] ***")
                key = ("left_mm" if stuck.side == "left"
                       else "right_front_mm")

                def _clear():
                    d = valid_tof(sensors.snapshot().get(key))
                    return d is not None and d >= B.REVERSE_CLEAR_MM

                _st, backed = reverse_nudge(
                    steer_now=steer,
                    set_steer=lambda u: servo.steer_delta(u, channel=0),
                    motor_stop=motor_stop,
                    motor_reverse=motor_back,
                    motor_forward=lambda: motor_go(decision.speed_pct),
                    clear=_clear)
                print(f"    ...backed {backed:.2f}s of "
                      f"{B.REVERSE_NUDGE_S:.2f}s allowed")
                speed_now = decision.speed_pct
                stuck.clear()
                # The car has MOVED since these last integrated. Carrying a
                # derivative across a reversal is how a controller reacts to a
                # jump it made itself.
                pid.reset()
                hwf.reset()
                continue

            servo.steer_delta(steer, channel=0)

            want_speed = decision.speed_pct
            if diag["stale_run"] >= B.BLIND_SLOW_FRAMES:
                want_speed = min(want_speed, B.BLIND_SPEED_PCT)
            if want_speed != speed_now:
                motor_go(want_speed)
                speed_now = want_speed

            if decision.state == B.FINISHING and race.finish_elapsed() >= FINISH_DRIVE_S:
                race.mark_finished()

            if dbg_interval and (now - last_dbg) >= dbg_interval:
                last_dbg = now
                print(f"[v7] {loop_hz:5.1f}hz {decision.state:<16} "
                      f"own={decision.owner:<9} steer={steer:+7.1f} "
                      f"spd={decision.speed_pct} | "
                      + f"corner={percept.corner_class}"
                      + f"({percept.corner_class_conf:.2f}"
                      + ("" if percept.corner_class is None
                         else ("" if B.BehaviorManager.locked_class(
                                        percept.corner_class,
                                        percept.direction) == percept.corner_class
                               else ">>%s" % race.direction))
                      + ") "
                      f"wall={percept.wall_class}({percept.wall_conf:.2f}) "
                      + (f"pillar={percept.pillar_color}"
                         f"({bias_effect:+.0f}us"
                         f"{'<-%+.0f' % bias if abs(bias_effect - bias) >= 1 else ''}"
                         f"{'/placed' if percept.pillar_placed else ''}"
                         + (f" s{perception.strip.sections(seen_pillar.strip_x):.1f}"
                            f"->{B.PILLAR_PID_TARGET_SECTION[percept.pillar_color] + 0.5:.1f}"
                            f" a={seen_pillar.area:.0f}"
                            if seen_pillar is not None else "")
                         + ") "
                         if percept.pillar_color else "")
                      + (f"clamp={wall_clamp:+.0f}us " if abs(wall_clamp) >= 1
                         else "")
                      + f"trig={fired}"
                      + ("" if fired is None else
                         ("" if percept.corner_trigger and class_backs
                          else "(front)" if percept.corner_trigger
                          else "(MUTED)"))
                      + f" | L={percept.left_mm} R={percept.right_mm} "
                      f"F={percept.front_mm} | {race.status()} "
                      f"[{decision.reason}]")

        if diag["front_total"]:
            print("")
            print(f"[front] {diag['front_total']} frames with NO front reading, "
                  f"longest run {diag['front_worst']}. The front is the only "
                  f"'you have arrived' signal a corner has; past "
                  f"{B.CORNER_FRONT_DEAD_FRAMES} the classifier commits alone.")
        if diag["stale_total"]:
            print("")
            print(f"[sides] {diag['stale_total']} stale L/R frames total, "
                  f"longest run {diag['stale_worst']}. A long run means the "
                  f"STM32 stopped answering, not that the car sat still.")
        off = pid.rr_offset_estimate()
        print(f"[heading] learned RF-RR offset: "
              f"{'never calibrated' if off is None else f'{off:+.0f}mm'}")

    except KeyboardInterrupt:
        aborted = True
    finally:
        try:
            sensors.stop_reader()          # port stays open, thread stops
        except Exception:
            pass
        stopped = DRY_RUN
        for attempt in range(MOTOR_STOP_TRIES):
            if stopped:
                break
            try:
                if motor_lock:
                    with motor_lock:
                        ack = motor.stop(timeout_s=MOTOR_STOP_TIMEOUT_S,
                                         wait_response=True)
                else:
                    ack = motor.stop(timeout_s=MOTOR_STOP_TIMEOUT_S,
                                     wait_response=True)
                stopped = ack is not None   # None = no answer, NOT success
            except Exception as exc:
                print(f"    motor stop attempt {attempt + 1} raised: {exc}")
            if not stopped:
                time.sleep(0.05)
        if not stopped:
            # Nothing answered. Shout, and keep firing blind on the way out --
            # an unacknowledged frame may still have landed.
            print("!! MOTOR DID NOT ACKNOWLEDGE STOP AFTER "
                  f"{MOTOR_STOP_TRIES} TRIES -- CUT POWER IF IT IS STILL "
                  "MOVING")
            for _ in range(MOTOR_STOP_TRIES):
                try:
                    _motor(motor.stop)
                except Exception:
                    pass
                time.sleep(0.04)
        servo.center_steering()
        servo.close()
        sensors.close()
        if not shared:
            motor.close()
        for cap in caps.values():
            cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("ABORTED." if aborted else f"DONE. {race.status()}")


if __name__ == "__main__":
    main()
