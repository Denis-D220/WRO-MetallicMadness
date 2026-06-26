"""
main_challenge_01_v4.py  --  v2's proven camera-corners + PID wall-following,
ported to the NEW VL53L8CX front + 3-side firmware. Tuning sandbox.

WHY v4 exists: v2 is the working base (camera reliably detects corners), but its
old sensor parser cannot read the new firmware, so on the new robot EVERY side
read came back -1 and the PID never actually wall-followed (it ran straight the
whole time). v4 keeps v2's logic and swaps in the new-firmware reader so the
sensors are live again.

Changes vs v2 (everything else is identical):
  * Sensors: SideSensorsV3 via a thin adapter (SensorAdapter) that re-exposes
    v2's snapshot keys. New firmware: 0x0105 -> RIGHT_FRONT|RIGHT_REAR|LEFT (all
    three sides in one frame); 0x0003 -> VL53L8CX 4x4 matrix. The PID follows the
    RIGHT_FRONT wall (mapped to right_mm); right_rear is available but unused for
    now (kept simple, like v2).
  * Front-barrier trigger: now driven by the 4x4 matrix (row 1 -> front_mm). The
    barrier only enters row 1 at ~95 cm, so "clear road" == front INVALID (not a
    big number); the arm/fire logic is matrix-aware (see the FRONT config below).

    STRAIGHT : motor forward; steering = PID(left, right_front); camera + sensor
               + front-matrix watch for a corner.
    CORNER   : detection OFF; timed turn; cooldown straight; PID reset.

Shared serial link: the motor and the sensors are the SAME STM32 on one port, so
the sensor object OWNS /dev/ttyUSB0 and reads; the motor reuses that serial and
only WRITES (wait_response=False), guarded by a lock. Same as v2.

main_challenge_01_v2.py is left UNTOUCHED as the working backup.
"""

import logging
import os
import statistics
import sys
import time
from collections import deque
from logging.handlers import RotatingFileHandler

import cv2
import Jetson.GPIO as GPIO

from servo_controller import ServoController
from motor_driver import MotorSerial
from turn_models import CornerDetector, draw_result
from color_tuning import apply_color_tuning
from sensor_distance_v3 import SideSensorsV3, FRONT_BARRIER_ROW
from pid_line_follower import LineFollowerPID


# =========================================================
# LOGGING
# =========================================================
# main.py runs unattended as a systemd service at competition startup (see
# service/wro-main.service), so every lifecycle event must land in a file we can
# read after the run -- the judges' table has no console. setup_logging() wires a
# rotating file (logs/main.log, 5 MB x 5) AND stdout, so the same messages show
# live when run by hand and persist on disk under systemd. The high-frequency
# per-frame telemetry ([WATCH] / [FRONT 4x4]) is emitted at DEBUG so it does not
# flood INFO logs; set WRO_LOG_LEVEL=DEBUG (or LOG_LEVEL constant) to see it.

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
LOG_FILE = os.path.join(LOG_DIR, "main.log")
LOG_LEVEL = os.environ.get("WRO_LOG_LEVEL", "INFO").upper()
LOG_MAX_BYTES = 5 * 1024 * 1024     # 5 MB per file before rotating
LOG_BACKUPS = 5                     # keep 5 rotated backups (~25 MB total)

logger = logging.getLogger("wro.main")


def setup_logging(level: str = LOG_LEVEL) -> logging.Logger:
    """Configure logging to BOTH a rotating file (logs/main.log) and stdout.

    Idempotent (safe to call once at startup): clears any existing handlers so a
    re-run under systemd Restart= does not duplicate lines. stdout is line-/
    stream-flushed and PYTHONUNBUFFERED=1 is set in the unit file, so log lines
    appear promptly in `journalctl` instead of being held in a pipe buffer.
    """
    os.makedirs(LOG_DIR, exist_ok=True)

    numeric_level = getattr(logging, level, logging.INFO)
    root = logging.getLogger()
    root.setLevel(numeric_level)
    for handler in list(root.handlers):   # avoid duplicate handlers on re-init
        root.removeHandler(handler)

    fmt = logging.Formatter(
        "%(asctime)s %(levelname)-7s [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    file_handler = RotatingFileHandler(
        LOG_FILE, maxBytes=LOG_MAX_BYTES, backupCount=LOG_BACKUPS)
    file_handler.setFormatter(fmt)
    file_handler.setLevel(numeric_level)
    root.addHandler(file_handler)

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(fmt)
    console_handler.setLevel(numeric_level)
    root.addHandler(console_handler)

    return logger


class SensorAdapter(SideSensorsV3):
    """New-firmware reader (SideSensorsV3) with v2's snapshot keys layered on top.

    v2's code expects right_mm / left_mm / valid and the front_* keys. The new
    reader exposes right_front_mm / right_rear_mm / left_mm and the VL53L8CX
    matrix front_*. We follow the RIGHT_FRONT wall, so right_mm == right_front.
    """

    def snapshot(self) -> dict:
        """Return the new-firmware sensor snapshot with v2's legacy keys layered on.

        Adds right_mm (== RIGHT_FRONT, the wall we follow), a combined valid flag,
        and age/timestamp aliases so the v2 main loop keeps working unchanged.
        """
        s = super().snapshot()
        s["right_mm"] = s["right_front_mm"]
        s["valid"] = s["left_valid"] and s["right_front_valid"]
        s["age_s"] = s["lr_age_s"]
        s["last_lr_time"] = s["lr_last_time"]
        return s


class HeadingWallFollower:
    """Two-right-sensor wall follower: hold a gap to the right wall AND keep the
    car PARALLEL. Distance-only steering overshoots wall-to-wall because it is
    blind to the car's angle; adding the RF-RR heading term damps that.

    Steering convention matches LineFollowerPID(side_sign=+1): positive u steers
    TOWARD the right wall. Too far (gap>target) -> +u; nose pointed AWAY from the
    wall (RF>RR) -> +u. Both push the car back to parallel at the target gap.

    RF/RR are median-filtered (RR spikes low now and then), and an out-of-band
    RF-RR is treated as a spike -> heading drops to 0 for that frame.
    """

    def __init__(self, target, rr_offset, kh, kd, out_limit, deadband,
                 heading_max, max_valid_mm=2000.0, median_window=3):
        """Configure the follower.

        target       desired gap to the right wall (mm).
        rr_offset    added to RR so RF == RR+offset means PARALLEL (calibration).
        kh, kd       heading and gap gains (us per mm).
        out_limit    max |u| output (us).
        deadband     gap error below this is treated as zero (anti-jitter, mm).
        heading_max  |RF-RR| above this = sensor spike -> ignore the angle (mm).
        max_valid_mm readings outside (0, this] are rejected.
        median_window per-side median filter length (spike rejection).
        """
        self.target = target
        self.rr_offset = rr_offset
        self.kh = kh
        self.kd = kd
        self.out_limit = out_limit
        self.deadband = deadband
        self.heading_max = heading_max
        self.max_valid_mm = max_valid_mm
        self.window = max(1, int(median_window))
        self.reset()

    def reset(self):
        """Clear the median histories and last output. Call at start and after
        each turn so the follower begins fresh on the new wall."""
        self._rf_hist = deque(maxlen=self.window)
        self._rr_hist = deque(maxlen=self.window)
        self.last_u = 0.0

    def _valid(self, v):
        """True if v is a usable distance reading (not None, in (0, max_valid_mm])."""
        return v is not None and 0 < v <= self.max_valid_mm

    @staticmethod
    def _clamp(x, lo, hi):
        """Clamp x to the inclusive range [lo, hi]."""
        return max(lo, min(hi, x))

    def update(self, rf, rr):
        """Returns (u, info). info['valid'] False -> RF unusable, hold center."""
        if not self._valid(rf):
            return self.last_u, {"valid": False}

        self._rf_hist.append(float(rf))
        rf_f = statistics.median(self._rf_hist)

        if self._valid(rr):
            self._rr_hist.append(float(rr))
        rr_f = statistics.median(self._rr_hist) if self._rr_hist else None

        if rr_f is not None:
            rr_aligned = rr_f + self.rr_offset
            heading = rf_f - rr_aligned          # >0 nose AWAY from the wall
            if abs(heading) > self.heading_max:  # RR spike -> ignore the angle
                heading = 0.0
                dist = rf_f
            else:
                dist = 0.5 * (rf_f + rr_aligned)
        else:
            heading = 0.0
            dist = rf_f

        gap_err = dist - self.target
        if abs(gap_err) < self.deadband:
            gap_err = 0.0

        u = self.kh * heading + self.kd * gap_err
        u = self._clamp(u, -self.out_limit, self.out_limit)
        self.last_u = u
        return u, {"valid": True, "heading": heading, "dist": dist,
                   "rf": rf_f, "u": u}


# =========================================================
# HARDWARE / PORT CONFIGURATION
# =========================================================

MOTOR_PORT = "/dev/ttyUSB0"
SENSOR_PORT = "/dev/ttyUSB0"     # same STM32 as the motor -> shared serial
SERVO_PORT = "/dev/ttyACM0"
SERVO_BAUD = 9600
BUTTON_PIN = 7


# =========================================================
# CHALLENGE / TRACK CONFIGURATION
# =========================================================

NUM_LAPS = 3
CORNERS_PER_LAP = 4
TOTAL_CORNERS = NUM_LAPS * CORNERS_PER_LAP

DRIVE_DIRECTION = "CW"           # DEPRECATED for direction: WRO forbids a preset --
                                 # the car must auto-detect. Only used by
                                 # default_direction() when AUTO_LOCK_DIRECTION is
                                 # False (a non-competition debug mode). Keep
                                 # AUTO_LOCK_DIRECTION = True for real runs.
# Lock the track turn direction from the FIRST corner's CAMERA detection, then
# reuse it for ALL corners and to choose the outer wall. WRO requires auto-detect,
# so corner-1 direction comes ONLY from the camera vote (camera_dir_vote); the car
# WAITS for that vote rather than guessing. Keep this True for competition.
AUTO_LOCK_DIRECTION = True

MOTOR_SPEED_PCT = 70             # STM32 global_motor_speed (0-100), set on init

TURN_STEER_US = 250
STEER_LEFT_US = -TURN_STEER_US
STEER_RIGHT_US = +TURN_STEER_US
TURN_TIME_S = 3.5

REQUIRE_BOTH_LINES = True
TRIGGER_CLOSENESS = 0.60
CONFIRM_FRAMES = 2
CORNER_COOLDOWN_S = 0.4          # blind straight after a turn. Scale DOWN with
                                 # speed: at 90% a 0.8s blind window ate the far
                                 # part of the straight, so the front barrier was
                                 # only ~820mm away when detection resumed -> the
                                 # turn fired too close and ran wide. 0.4s lets the
                                 # front be seen farther out, so it turns earlier.
FINISH_STRAIGHT_S = 1.0

# Post-turn LEAD steer. After each turn, instead of coasting straight through the
# cooldown, hold a fixed steering "lean" so the car deliberately approaches one
# wall. Leaning toward the OUTER wall makes its distance collapse cleanly at the
# next corner, so the sensor trigger fires reliably (this is what rescues corner-2
# detection at high speed). LEAD_STEER_US is the lean magnitude (us); 0 = off
# (old straight behavior). LEAD_TOWARD = "outer" (toward followed wall) or "inner".
LEAD_AFTER_CORNER = True
LEAD_STEER_US = 80
LEAD_TOWARD = "outer"

# After the track direction is locked (first corner), let the camera confirm a
# corner from a SINGLE close line instead of requiring both colors.
# DISABLED: it fired a fake corner -- right after a turn, the just-passed corner's
# single line (orange, close=1.00) re-triggered an extra corner. The sensor
# fallback already catches camera-missed corners, so we keep the camera strict.
RELAX_BOTH_LINES_AFTER_LOCK = False

# Sensor-based corner fallback. When the FOLLOWED outer wall distance collapses,
# the car has physically reached the corner (the outer wall wraps in front of
# the side sensor). If the camera missed the corner, this turns anyway so the
# car never drives into the wall. Active only after the direction is locked and
# only in "outer" wall-follow mode.
#
# Fire EARLY (high threshold) so there is room to turn before reaching the wall.
# To stop the post-turn recovery dip (~237-260 mm observed) from false-firing the
# next corner, the trigger RE-ARMS only after the outer wall has receded past
# SENSOR_REARM_MM since the last corner. So the sequence each straight is:
# corner -> recovery (disarmed) -> wall recedes past re-arm (armed) -> next
# corner's collapse fires.
ENABLE_SENSOR_CORNER = True
SENSOR_CORNER_MM = 250.0         # followed-wall distance below this = at corner
SENSOR_CORNER_FRAMES = 3         # consecutive samples below threshold to fire
SENSOR_REARM_MM = 330.0          # wall must recede past this to re-arm the trigger

# Front-barrier corner trigger -- now the VL53L8CX 4x4 matrix (command 0x0003).
# sensor_distance_v3.front_scalars() reduces row 1 of the matrix to front_mm.
# IMPORTANT difference from the old single-zone sensor: the barrier only enters
# row 1 at ~95 cm; farther than that the whole row reads >2500 -> front INVALID.
# So "clear road ahead" == front INVALID, NOT a large number. The arm/fire logic
# is therefore matrix-aware:
#   ARM  while the barrier is far/absent (front INVALID, or valid >= FRONT_ARM_MM)
#   FIRE when it closes into [FRONT_MIN_TRUST_MM, FRONT_CORNER_MM]
# Below ~50 cm row 1 collapses to ~285 mm (flaky), so MIN_TRUST rejects it and we
# fire BEFORE then. Measured sweep fired cleanly at the first solid detection
# (~954 mm). Only active after the direction is locked (corner 1 stays camera).
ENABLE_FRONT_CORNER = True
FRONT_ARM_MM = 1000.0            # valid >= this (or barrier ABSENT) -> armed
FRONT_CORNER_MM = 900.0          # fire when the wall closes to <= this. Set with the
                                 # TILTED front sensor: the tilt changes where the
                                 # wall lands in row 1, so the fire distance is
                                 # re-tuned to 900 (ARM 1000 stays just above the
                                 # fire window). Re-check the front= value at each
                                 # corner after the tilt and nudge these if needed.

FRONT_MIN_TRUST_MM = 330.0       # ignore row-1 below this (floor / sub-50cm flake).
                                 # LOWERED 450->330: the corner OUTER WALL (vs the
                                 # tuned barrier obstacle) only enters row 1 at
                                 # ~360-430 mm -- it jumps 2300 -> ~380 with nothing
                                 # in the old [450,1000] window, so the front trigger
                                 # NEVER fired on corners. 330 catches the ~360+ wall
                                 # while still rejecting the ~285 mm floor flake.
FRONT_CORNER_FRAMES = 2          # fresh front frames in-window to fire. RAISED
                                 # 1->2: at 1, a single transient frame right after a
                                 # turn fired a FALSE corner (zone-2 corner 3 -> inner
                                 # wall). 2 needs two in-window frames, rejecting the
                                 # one-frame mis-aimed-exit catch.
FRONT_MIN_CELLS = 2              # row-1 cells in the barrier band required to FIRE.
                                 # A lone in-band cell among OPEN cells is an edge/
                                 # wall-corner catch, not the barrier face -> firing
                                 # on it turns too soon (seen at the lap-2 corner).
# Crash failsafe: if the barrier gets this close and we somehow have NOT turned
# (window skipped by noise, or never armed), turn NOW regardless -- better an
# early/extra turn than driving into the wall (seen: front ran 1085->...->3 mm).
FRONT_EMERGENCY_MM = 380.0
FRONT_EMERGENCY_FRAMES = 2

# Poll the front 4x4 matrix BEFORE the direction locks too. Originally the matrix
# was off until corner 1 locked (to protect the motor start frame on the shared
# bus); but that left corner 1 with the camera as its ONLY trigger, so a bad
# start zone drove into the wall. We still keep it off through the motor-start
# pulses, then enable it just before the main loop so corner 1 gets the early
# front trigger (the camera fires late -> the zone-2 wall hit).
FRONT_ENABLE_FROM_START = True

# Pre-lock corner detection (corner 1, before the direction is locked). The
# original code had NO camera-independent fallback here, so a bad start zone
# (camera misses one/both colored lines) drove straight into the corner wall
# (seen in zones 1 and 3). We add a geometry trigger that ALSO infers the turn
# direction, since the camera normally supplies it:
#   * front wall/barrier ahead (same window as the post-lock front trigger), OR
#   * RIGHT nose-in: RF collapses while RR stays back (the outer wall has wrapped
#     in front of the right side). RF ~= RR means merely STARTING near the right
#     wall (zone 3), NOT a corner -- the RR-RF gap is what tells them apart.
# Direction = turn toward the MORE-OPEN side: the side reading farther (or the
# side that reads INVALID = out of range = the open corner gap) is the inside.
ENABLE_PRELOCK_CORNER = True
PRELOCK_NOSEIN_MM = 250.0        # RF below this (with the gap) = right wall ahead
PRELOCK_NOSEIN_GAP_MM = 150.0    # RR-RF gap required: corner, not parallel-near-wall
PRELOCK_RF_HARD_MM = 130.0       # RF below this fires regardless of the RR-RF gap.
                                 # When the car reaches the corner PARALLEL (hugging
                                 # the outer wall, zone 3), RF and RR collapse
                                 # TOGETHER (gap ~30-60 mm), so the nose-in gap never
                                 # triggers -- but RF still drops to ~20-60 mm. A
                                 # plain low-RF floor catches that. Start zones read
                                 # RF >= ~150, so 130 won't false-fire at the start.
PRELOCK_CORNER_FRAMES = 2        # consecutive frames to fire (rejects 1-frame noise)

# First-corner DIRECTION resolution. WRO requires the car to AUTO-DETECT the track
# direction at runtime -- NO preset/operator input. turn_models derives the turn
# from the NEAREST line's color (blue->left, orange->right). A lone FAR line is
# unreliable (it reads as "nearest" and inverts), so a vote is only counted from a
# BOTH-lines frame OR a CLOSE lone line (the near line, reliable -- CAM_DIR_LONE_CLOSE).
# Corner-1 direction = the confident camera vote (camera_dir_vote). If a non-camera
# trigger fires before the vote lands, the car WAITS (keeps approaching) rather than
# guessing; only an emergency (wall about to be hit) forces a sensor-only last
# resort (emergency_dir). Subsequent corners reuse the locked direction.
CAM_DIR_MIN_VOTES = 1            # both-lines frames required before the vote counts.
                                 # LOWERED 3->1: corner 1 only yields ~1 both-frame
                                 # before the front trigger fires, so 3 was never
                                 # reached and the preset always decided (CW turned
                                 # LEFT despite the camera seeing the right corner).
                                 # both-frames are the reliable kind, so 1 is enough
                                 # to let the camera auto-pick direction; raise back
                                 # toward 2-3 if a wrong auto-lock is ever seen.
CAM_DIR_CONFIDENCE = 0.6         # winning side must be >= this fraction of votes
CAM_DIR_LONE_CLOSE = 0.62        # a SINGLE line this close (or closer) counts as a
                                 # vote: high closeness = it's the NEAR line, so its
                                 # color is reliable. Boosts the vote rate (esp. for
                                 # RIGHT corners where both-line frames are rare) so
                                 # the camera can auto-decide direction in time.

SHOW_VIEW = False
DEBUG_PRINT_HZ = 5.0
DEBUG_FRONT_MATRIX = True         # dump the raw 4x4 matrix in [WATCH] for tuning


# =========================================================
# PID WALL-FOLLOWING CONFIGURATION
# =========================================================

ENABLE_PID = True                # False -> behaves like v1 (center steering)
# On the FIRST sector (before corner 1) the car is hand-placed straight and
# aligned, so there is nothing to correct -- and the sensors may still be settling
# right after start. Hold dead-center on that first straight and let the PID take
# over only AFTER corner 1, where the exit angle actually needs cleaning up.
PID_SKIP_FIRST_SECTOR = True     # hold dead-center on the first straight (PID off).
                                 # NOTE: this does NOT fix off-center start zones --
                                 # the real corner-1 problem is detection, not
                                 # steering (see pre-lock fallback work).
LR_HZ = 20.0                     # side-sensor poll rate (0x0105, all 3 sides)
FRONT_HZ = 10.0                  # VL53L8CX 4x4 matrix poll rate (0x0003)

# Wall-following mode:
#   "outer"  -> follow ONLY the outer wall at WALL_TARGET_MM (robust: the inner
#               wall is discontinuous at corners and corrupts difference mode).
#   "center" -> classic difference (R-L); keeps the car in the MIDDLE using both
#               walls. The corner fallback still watches the outer wall, so a
#               missing inner wall at a corner is handled by the sensor trigger.
#               If the car wanders where the inner wall has mid-straight gaps,
#               switch back to "outer".
WALL_FOLLOW_MODE = "outer"
# Distance to hold from the outer wall. Calibrate: center the car in a straight,
# read sensor_distance.py, use the OUTER-side value (your static test ~470).
WALL_TARGET_MM = 470.0

# PID gains (steering microseconds per mm of L/R difference).
# KP raised for real centering authority; KD damps the steering->position
# double-integrator (safe now that the median filter cleans the input).
PID_KP = 0.70
PID_KI = 0.00                    # add ~0.10 only if a steady lean remains
PID_KD = 0.15
PID_OUT_LIMIT = 150.0            # max centering steer (keep < TURN_STEER_US)
PID_CENTER_OFFSET = 12.0         # center mode: (R-L) at true center. Static test
                                 # read LEFT 456 / RIGHT 468 -> offset = 468-456.
PID_STEER_SIGN = 1               # flip to -1 if corrections go the wrong way
PID_DEADBAND = 8.0               # mm; ignore tiny errors to avoid jitter
PID_MEDIAN_WINDOW = 3            # median filter length per side (rejects spikes)
PID_SLEW_LIMIT = 100.0           # max steering change per update (us); 0 = off

# Two-right-sensor HEADING wall-follower (the lane-keeping fix). The distance-only
# PID is blind to the car's ANGLE, so it exits turns angled, over-corrects, and
# overshoots WALL-TO-WALL (the lap-3 wedge: RF=89 one corner, L=17 the next). The
# RF-RR heading term holds the car PARALLEL, killing the overshoot. Only works
# when the OUTER wall is on the RIGHT (CCW / left turns -- which this track is);
# for the other direction it falls back to the distance-only PID automatically.
# Set ENABLE_HEADING = False to revert exactly to the previous behavior.
ENABLE_HEADING = True
WF_KH = 1.2                      # heading gain (us per mm of RF-RR). Raise for a
                                 # stiffer parallel hold; lower if it oscillates.
WF_KD = 0.6                      # gap gain (us per mm of distance error)
WF_RR_OFFSET_MM = 0.0            # add to RR so RF==RR+offset means PARALLEL.
                                 # Calibrate: park the car parallel to the wall,
                                 # read RF-RR, set this = that value. If the car
                                 # holds a steady angle, tune this first.
WF_OUT_LIMIT = 150.0             # shared steering limit (us; keep < TURN_STEER_US)
WF_DEADBAND = 8.0                # mm; gap deadband
WF_HEADING_MAX_MM = 200.0        # |RF-RR| above this = RR spike -> ignore heading

CAMERA_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1, format=NV12, colorimetry=bt601 ! "
    "nvvidconv ! "
    "video/x-raw, width=1280, height=720, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)


# =========================================================
# BUTTON HELPERS
# =========================================================

def set_pin():
    """Initialize the start/abort push button on BUTTON_PIN as a GPIO input."""
    GPIO.setwarnings(False)
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(BUTTON_PIN, GPIO.IN)
    logger.info("Button input pin: %s", GPIO.input(BUTTON_PIN))
    time.sleep(1)


def button_pressed() -> bool:
    """True while the button is held down (pin reads HIGH)."""
    return GPIO.input(BUTTON_PIN) == GPIO.HIGH


def wait_for_release():
    """Block until the button is released, then debounce briefly."""
    while button_pressed():
        time.sleep(0.02)
    time.sleep(0.05)


def wait_for_button_press():
    """Block until a clean press: wait for release, then a press, then release.
    Used to gate the start so a held button can't immediately retrigger."""
    while button_pressed():
        time.sleep(0.02)
    while not button_pressed():
        time.sleep(0.02)
    time.sleep(0.05)
    wait_for_release()


def sleep_with_abort(duration_s: float) -> bool:
    """Sleep up to duration_s, but return True early if the button is pressed
    (an abort request). Returns False if the full duration elapsed. Used for the
    blind turn and finish straight so the button can always halt the car."""
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        if button_pressed():
            time.sleep(0.05)
            return True
        time.sleep(0.01)
    return False


# =========================================================
# STEERING / DETECTION HELPERS
# =========================================================

def default_direction() -> str:
    """Turn side implied by the DRIVE_DIRECTION constant (CCW->left, else right).
    Debug/fallback only -- in competition direction is auto-detected, not preset."""
    return "left" if DRIVE_DIRECTION.upper() == "CCW" else "right"


def outer_for(track_dir):
    """Outer-wall sensor side for a given track turn direction (None if unknown)."""
    if track_dir == "left":
        return "right"
    if track_dir == "right":
        return "left"
    return None


def steer_us_for(direction: str) -> int:
    """Servo steering offset (us) for a turn in the given direction."""
    return STEER_LEFT_US if direction == "left" else STEER_RIGHT_US


def both_lines_present(result) -> bool:
    """True if the detector saw BOTH a blue and an orange corner line this frame
    (the high-confidence case for picking turn direction)."""
    colors = {line["color"] for line in result["lines"]}
    return "blue" in colors and "orange" in colors


def corner_confirmed(result) -> bool:
    """Strict camera corner test: a detected, close-enough corner with a turn side
    and (if required) both lines. Helper for camera-based confirmation."""
    if not result["detected"] or result["turn"] is None:
        return False
    if result["distance_norm"] is None or result["distance_norm"] < TRIGGER_CLOSENESS:
        return False
    if REQUIRE_BOTH_LINES and not both_lines_present(result):
        return False
    return True


# =========================================================
# MAIN
# =========================================================

def main():
    """Run one full Open Challenge attempt: set up hardware, wait for the button,
    then loop STRAIGHT (wall-follow + corner watch) and CORNER (timed turn +
    follower recovery) until 12 corners (3 laps) are done, auto-detecting the
    track direction at corner 1. Always stops the motor and cleans up on exit."""
    logger.info("=== WRO 2026 Metallic Madness main.py starting ===")
    logger.info("Competition waiting mode: initializing hardware, motors will "
                "stay STOPPED until the Start button is pressed.")
    set_pin()
    logger.info("Button pin ready (sensor init).")

    can_show = SHOW_VIEW and bool(os.environ.get("DISPLAY"))
    if SHOW_VIEW and not can_show:
        logger.warning("SHOW_VIEW requested but no $DISPLAY; running headless.")

    logger.info("Initializing servo/steering controller on %s", SERVO_PORT)
    servo = ServoController(port=SERVO_PORT, baud_rate=SERVO_BAUD)
    servo.connect()
    servo.center_steering()

    # ---- Side sensors own the serial; motor shares it if same port ----
    sensors = None
    if ENABLE_PID:
        logger.info("Initializing side/front sensors on %s", SENSOR_PORT)
        sensors = SensorAdapter(port=SENSOR_PORT, lr_hz=LR_HZ, front_hz=FRONT_HZ)
        sensors.open()
        # Keep the heavy VL53L8CX matrix poll OFF until corner 1 locks the
        # direction (it is unused before then, and its scan can drop the motor
        # start frame on the shared bus). Enabled right after the lock below.
        sensors.set_front_enabled(False)

    logger.info("Initializing motor/serial controller on %s", MOTOR_PORT)
    motor = MotorSerial(MOTOR_PORT, debug=False)
    shared = ENABLE_PID and (SENSOR_PORT == MOTOR_PORT)
    if shared:
        motor.ser = sensors.ser          # reuse the sensor's serial object
        motor_lock = sensors.write_lock
        logger.info("Motor sharing the sensor serial link (%s)", MOTOR_PORT)
    else:
        motor.open()
        motor_lock = None

    def motor_forward():
        """Command continuous forward (write-only), holding the shared-bus lock if
        the motor shares the sensor's serial port."""
        if motor_lock:
            with motor_lock:
                motor.forward_continuous(wait_response=False)
        else:
            motor.forward_continuous(wait_response=False)

    def motor_stop():
        """Command motor stop (write-only), holding the shared-bus lock if shared."""
        if motor_lock:
            with motor_lock:
                motor.stop(wait_response=False)
        else:
            motor.stop(wait_response=False)

    def motor_set_speed(percent):
        """Set the STM32 global motor speed 0-100 (write-only), lock if shared."""
        if motor_lock:
            with motor_lock:
                motor.set_speed(percent, wait_response=False)
        else:
            motor.set_speed(percent, wait_response=False)

    def motor_pulse(fn, times=4, gap=0.05):
        """Send a fire-and-forget motor command a few times so it survives any
        single dropped frame on the shared STM32 link. Used only at start/stop."""
        for _ in range(times):
            fn()
            time.sleep(gap)

    pid = LineFollowerPID(
        kp=PID_KP, ki=PID_KI, kd=PID_KD,
        out_limit=PID_OUT_LIMIT, center_offset=PID_CENTER_OFFSET,
        steer_sign=PID_STEER_SIGN, deadband=PID_DEADBAND,
        median_window=PID_MEDIAN_WINDOW, slew_limit=PID_SLEW_LIMIT,
    )

    hwf = HeadingWallFollower(
        target=WALL_TARGET_MM, rr_offset=WF_RR_OFFSET_MM, kh=WF_KH, kd=WF_KD,
        out_limit=WF_OUT_LIMIT, deadband=WF_DEADBAND,
        heading_max=WF_HEADING_MAX_MM, median_window=PID_MEDIAN_WINDOW,
    )

    logger.info("Loading corner detector (camera/model init)...")
    detector = CornerDetector()

    logger.info("Opening camera pipeline (GStreamer/nvargus)...")
    cap = cv2.VideoCapture(CAMERA_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        logger.error("Could not open camera. Try: "
                     "sudo systemctl restart nvargus-daemon")
        servo.close()
        if sensors:
            sensors.close()
        if not shared:
            motor.close()
        GPIO.cleanup()
        raise SystemExit(1)
    logger.info("Camera opened.")

    def steer_straight(outer_side):
        """Steer on the straight.

        outer_side ("right"/"left") -> follow that single outer wall.
        outer_side None (direction not locked yet) -> center on both walls.
        Returns (debug_string, outer_dist) where outer_dist is the FILTERED
        followed-wall distance in mm (or None if not in single-wall mode / the
        reading was invalid). outer_dist feeds the sensor corner fallback.
        """
        if not ENABLE_PID or sensors is None:
            servo.center_steering()
            return "center", None

        # First sector: start position is known-straight -> hold center, no PID.
        # (corners_done is read from the enclosing scope; it is 0 until corner 1.)
        if PID_SKIP_FIRST_SECTOR and corners_done == 0:
            servo.center_steering()
            return "sector1: center (PID off)", None

        snap = sensors.snapshot()

        # Two-right-sensor heading follower -- ALWAYS follow the RIGHT wall once the
        # direction is locked (parallel-hold only works where we have two sensors).
        #   CCW: right wall = OUTER wall (target 470).
        #   CW : right wall = INNER wall. The lane is ~symmetric (centered reads
        #        L ~= RF ~= 480), so the SAME 470 target centers the car AND gives
        #        the parallel-hold that distance-only PID on the left wall lacked
        #        (CW wandered into the inner wall after ~10 corners without it).
        if (ENABLE_HEADING and WALL_FOLLOW_MODE == "outer"
                and outer_side is not None):
            rf = snap["right_front_mm"]
            rr = snap["right_rear_mm"]
            u, info = hwf.update(rf, rr)
            if info["valid"]:
                servo.steer_delta(u, channel=0)
                # outer_dist feeds the sensor corner fallback, which only makes
                # sense when RIGHT is the OUTER wall (CCW). In CW the right wall is
                # the INNER wall -- it dips below the sensor threshold mid-straight
                # and would false-fire -- so hand back None and rely on the front.
                od = info["rf"] if outer_side == "right" else None
                return (f"RF={rf} RR={rr} hd={info['heading']:.0f} "
                        f"dist={info['dist']:.0f} tgt={WALL_TARGET_MM:.0f} "
                        f"u={u:.0f}", od)
            servo.center_steering()
            return f"RF={rf} INVALID->center", None

        if WALL_FOLLOW_MODE == "outer" and outer_side is not None:
            dist = snap["right_mm"] if outer_side == "right" else snap["left_mm"]
            sign = +1 if outer_side == "right" else -1
            u, info = pid.update_single(dist, WALL_TARGET_MM, sign)
            if info["valid"]:
                servo.steer_delta(u, channel=0)
                return (f"{outer_side}={dist}->{info['wall_f']:.0f} "
                        f"tgt={WALL_TARGET_MM:.0f} e={info['error']:.0f} u={u:.0f}",
                        info["wall_f"])
            servo.center_steering()
            return f"{outer_side}={dist} INVALID->center", None

        # center (difference) mode: before direction is locked, or by config.
        # The PID steers on the L/R difference (stay in the middle), but the
        # corner fallback still needs the OUTER wall distance -- hand it back here
        # so corner detection is independent of the steering mode. Before the
        # lock, outer_side is None and no distance is forwarded (no fallback yet).
        outer_dist = None
        if outer_side is not None:
            od = snap["right_mm"] if outer_side == "right" else snap["left_mm"]
            if 0 < od <= 2000:
                outer_dist = float(od)

        u, info = pid.update(snap["left_mm"], snap["right_mm"])
        if info["valid"]:
            servo.steer_delta(u, channel=0)
            return (f"L={snap['left_mm']}->{info['left_f']:.0f} "
                    f"R={snap['right_mm']}->{info['right_f']:.0f} "
                    f"e={info['error']:.0f} u={u:.0f} [center]", outer_dist)
        servo.center_steering()
        return f"L={snap['left_mm']} R={snap['right_mm']} INVALID->center", outer_dist

    aborted = False
    corners_done = 0
    confirm = 0
    both_seen = False        # latched: both lines seen during this approach
    latched_turn = None      # turn side captured when both lines were visible
    outer_low_frames = 0     # consecutive frames the followed wall was collapsed
    sensor_armed = False     # re-armed once the outer wall recedes past re-arm mm
    front_win_frames = 0     # consecutive FRESH front frames inside the fire window
    front_near_frames = 0    # consecutive FRESH front frames below the crash floor
    front_armed = False      # armed once the barrier was far/absent
    prev_front_time = 0.0    # last front reading timestamp processed (debounce)
    prelock_frames = 0       # consecutive pre-lock geometry-corner frames (corner 1)
    cam_left_votes = 0       # both-lines frames whose nearest line -> LEFT (corner 1)
    cam_right_votes = 0      # both-lines frames whose nearest line -> RIGHT (corner 1)
    # track turn direction: locked at the first corner (auto) or preset (config).
    track_dir = None if AUTO_LOCK_DIRECTION else default_direction()
    last_dbg = 0.0
    dbg_interval = 1.0 / DEBUG_PRINT_HZ if DEBUG_PRINT_HZ > 0 else 0.0

    def camera_dir_vote(left_votes, right_votes):
        """Corner-1 turn direction from the CAMERA vote ONLY (WRO requires the car
        to auto-detect direction -- no preset/operator input allowed). Returns
        "left"/"right" once a confident vote exists, else None (= undecided, wait)."""
        total = left_votes + right_votes
        if total < CAM_DIR_MIN_VOTES:
            return None
        if left_votes >= right_votes and left_votes >= CAM_DIR_CONFIDENCE * total:
            return "left"
        if right_votes > left_votes and right_votes >= CAM_DIR_CONFIDENCE * total:
            return "right"
        return None

    def emergency_dir(snap, right_in_front):
        """Last-resort direction if the wall is an emergency distance away and the
        camera STILL has not decided (rare). Sensor-only (no preset), so still
        'auto'. RF collapsed -> right wall ahead -> turn left; L collapsed -> turn
        right; else turn toward the more-open side."""
        rf = snap.get("right_front_mm", -1)
        lf = snap.get("left_mm", -1)
        if right_in_front or (0 < rf < 130):
            return "left"
        if 0 < lf < 130:
            return "right"
        # turn toward whichever side reads farther (the open / inner side)
        return "left" if (lf > rf) else "right"

    try:
        # SAFETY: force the motor into a stopped/safe state BEFORE waiting for the
        # Start button. WRO rules forbid any motion before the judge says "Go" and
        # the single Start button is pressed; the car may have powered on with the
        # STM32 in an unknown state, so we pulse STOP (survives a dropped frame on
        # the shared bus) and center the steering first. Nothing below moves the
        # car until wait_for_button_press() returns.
        motor_pulse(motor_stop)
        servo.center_steering()
        logger.info("Initial safe state: motor STOPPED, steering centered.")

        logger.info("Ready. %d lap(s) = %d corners. PID=%s.",
                    NUM_LAPS, TOTAL_CORNERS, "ON" if ENABLE_PID else "OFF")
        logger.info("WAITING for Start button (press to START)...")
        wait_for_button_press()
        logger.info("Start button PRESSED -> START. Round/challenge started.")

        # Warm up the camera + detector BEFORE moving: pull frames so the auto-
        # exposure settles and YOLO's slow first inference is done. Otherwise
        # corner 1 (which can be <1 m away at a close start zone) is read from cold,
        # dark frames, the camera misses the corner lines, and there is no direction
        # vote in time -> wrong/last-resort turn. This is what made CW corner 1 fail.
        for _ in range(15):
            ret, wf = cap.read()
            if ret:
                detector.detect(wf)
            time.sleep(0.02)

        servo.center_steering()
        pid.reset()
        hwf.reset()
        # Start FORWARD with no reverse twitch. On this firmware a set-speed frame
        # spins the motor in its idle direction (REVERSE) the instant it lands, so
        # setting speed first makes the car lurch backwards before forward arrives.
        # Fix: send FORWARD first to fix the direction, then interleave set-speed
        # with forward so a forward frame immediately follows every speed frame --
        # the motor never sits in reverse. Repeats also cover any dropped frame.
        motor_forward()
        for _ in range(4):
            motor_set_speed(MOTOR_SPEED_PCT)
            motor_forward()
            time.sleep(0.04)
        logger.info("Motor speed set to %d%% -- vehicle moving.", MOTOR_SPEED_PCT)
        motor_forward()

        # Motor is going -> safe to turn the front matrix on now (kept off through
        # the start pulses so its scan can't eat the start frame on the shared
        # bus). This gives corner 1 the early front trigger / pre-lock fallback.
        if FRONT_ENABLE_FROM_START and sensors is not None:
            sensors.set_front_enabled(True)

        # ---- STRAIGHT state ----
        while corners_done < TOTAL_CORNERS:
            ret, frame = cap.read()
            if not ret:
                continue

            if button_pressed():
                aborted = True
                break

            result = detector.detect(frame)
            steer_info, outer_dist = steer_straight(outer_for(track_dir))

            # --- Robust corner trigger (tolerant of flicker) ---
            # Latch "lines seen" + the turn side while they are visible. Before
            # the direction is locked we require BOTH colored lines (to pick the
            # side safely); after the lock a single close-enough line is enough.
            detected = result["detected"]
            close = result["distance_norm"]
            close_ok = detected and close is not None and close >= TRIGGER_CLOSENESS

            require_both = REQUIRE_BOTH_LINES and not (
                RELAX_BOTH_LINES_AFTER_LOCK and track_dir is not None)

            if detected and result["turn"] is not None:
                if both_lines_present(result) or not require_both:
                    both_seen = True
                    latched_turn = result["turn"]
                # Camera direction VOTE (corner 1 only -- WRO requires auto-detect).
                # A vote is trustworthy when the line we're reading is genuinely the
                # NEAR one: either BOTH lines are present (the lower one wins), OR a
                # single line is CLOSE (high closeness = it's the near line, so its
                # color is reliable). A lone FAR line (low closeness) is the
                # ambiguous case that inverted before -- those we still ignore.
                vote_ok = both_lines_present(result) or (
                    close is not None and close >= CAM_DIR_LONE_CLOSE)
                if track_dir is None and vote_ok:
                    if result["turn"] == "left":
                        cam_left_votes += 1
                    elif result["turn"] == "right":
                        cam_right_votes += 1

            # Accumulate confirm while close + lines already seen; only reset when
            # the corner leaves view entirely (so a one-frame flicker is OK).
            if both_seen and close_ok:
                confirm += 1
            elif not detected:
                confirm = 0

            # --- Sensor corner fallback (plan A) ---
            # After the direction is locked, the followed outer wall collapsing
            # below SENSOR_CORNER_MM means we are at the corner; turn even if the
            # camera missed it, so the car never drives into the outer wall. The
            # trigger only fires once re-armed (wall receded past SENSOR_REARM_MM
            # since the last corner) so the post-turn recovery dip can't fire it.
            sensor_corner = False
            if (ENABLE_SENSOR_CORNER and track_dir is not None
                    and outer_dist is not None):
                if outer_dist >= SENSOR_REARM_MM:
                    sensor_armed = True
                if outer_dist < SENSOR_CORNER_MM:
                    outer_low_frames += 1
                else:
                    outer_low_frames = 0
                if sensor_armed and outer_low_frames >= SENSOR_CORNER_FRAMES:
                    sensor_corner = True
            else:
                outer_low_frames = 0

            # --- Front-barrier corner trigger (VL53L8CX 4x4 matrix) ---
            # Matrix-aware: ARM while the barrier is far/absent (front INVALID, or
            # valid >= FRONT_ARM_MM), then FIRE when row-1 closes into the window
            # [FRONT_MIN_TRUST_MM, FRONT_CORNER_MM] (~95cm down to the ~50cm flake
            # floor). The barrier jumps straight from invalid to ~954 mm, so
            # arming on "far/absent" (not on a big number) is what lets the very
            # first frames count. Stepped only on a FRESH reading. Only after lock.
            front_corner = False
            front_dist = None
            front_cells = 0
            if (ENABLE_FRONT_CORNER and track_dir is not None
                    and sensors is not None):
                fsnap = sensors.snapshot()
                front_valid = fsnap["front_valid"]
                front_cells = fsnap.get("front_cells", 0)
                front_dist = fsnap["front_mm"] if front_valid else None
                if fsnap["front_last_time"] != prev_front_time:
                    prev_front_time = fsnap["front_last_time"]
                    fd = fsnap["front_mm"]
                    if (not front_valid) or fd >= FRONT_ARM_MM:
                        front_armed = True
                    # FIRE only on a barrier seen by >= FRONT_MIN_CELLS row-1
                    # cells: a lone in-band cell among open cells is an edge/wall
                    # corner, not the barrier face (turns too soon).
                    in_window = (front_valid
                                 and FRONT_MIN_TRUST_MM <= fd <= FRONT_CORNER_MM
                                 and front_cells >= FRONT_MIN_CELLS)
                    if front_armed and in_window:
                        front_win_frames += 1
                    elif not in_window:
                        front_win_frames = 0
                    # Crash failsafe count: barrier dangerously close (below the
                    # window) and still no turn -> count toward an emergency turn.
                    if front_valid and 0 < fd <= FRONT_EMERGENCY_MM:
                        front_near_frames += 1
                    else:
                        front_near_frames = 0
                if front_armed and front_win_frames >= FRONT_CORNER_FRAMES:
                    front_corner = True
                if front_near_frames >= FRONT_EMERGENCY_FRAMES:
                    front_corner = True          # failsafe: turn or crash
            else:
                front_win_frames = 0
                front_near_frames = 0

            # --- Pre-lock corner detection (corner 1 only) ---
            # Before the direction locks, the camera is the ONLY trigger and it
            # needs both colored lines; a bad start zone misses them and drives
            # into the wall. Here we detect the corner from geometry AND infer the
            # turn direction (the camera normally supplies it). Active only while
            # track_dir is None, so it never interferes with corners 2..12.
            prelock_corner = False
            right_in_front = False   # right (outer) wall wrapped ahead -> implies LEFT
            if (ENABLE_PRELOCK_CORNER and track_dir is None
                    and sensors is not None):
                psnap = sensors.snapshot()
                p_rf = (psnap["right_front_mm"]
                        if psnap["right_front_valid"] else None)
                p_rr = (psnap["right_rear_mm"]
                        if psnap["right_rear_valid"] else None)
                p_front_valid = psnap["front_valid"]
                p_front = psnap["front_mm"] if p_front_valid else None
                p_cells = psnap.get("front_cells", 0)

                # (a) a wall/barrier is ahead -> a corner is here.
                front_ahead = (p_front_valid
                               and FRONT_MIN_TRUST_MM <= p_front <= FRONT_CORNER_MM
                               and p_cells >= FRONT_MIN_CELLS)
                # (b) right nose-in: RF collapsed AND well below RR -> the outer
                #     wall wrapped in front of the right side at an ANGLE.
                right_nosein = (p_rf is not None and p_rr is not None
                                and p_rf < PRELOCK_NOSEIN_MM
                                and (p_rr - p_rf) >= PRELOCK_NOSEIN_GAP_MM)
                # (c) right hard-collapse: RF very low regardless of the gap -> the
                #     car reached the corner PARALLEL to the outer wall (zone 3),
                #     so RF and RR drop together and (b) never fires.
                right_hard = p_rf is not None and p_rf < PRELOCK_RF_HARD_MM
                right_corner = right_nosein or right_hard

                if front_ahead or right_corner:
                    prelock_frames += 1
                else:
                    prelock_frames = 0

                if prelock_frames >= PRELOCK_CORNER_FRAMES:
                    prelock_corner = True
                    # The DIRECTION is decided by resolve_first_corner_dir() at the
                    # lock below. We only forward the geometry fact it needs: a right
                    # nose-in / hard-collapse means the RIGHT (outer) wall wrapped in
                    # front -> a LEFT turn, REGARDLESS of lateral position (it's a
                    # front-vs-rear comparison on the outer wall). Raw side distance
                    # is NOT used to pick the side -- hugging the inner wall makes the
                    # inner side read small, which would invert the guess.
                    right_in_front = right_corner
            else:
                prelock_frames = 0

            now = time.monotonic()
            if dbg_interval and now - last_dbg >= dbg_interval:
                last_dbg = now
                close_s = f"{close:.2f}" if close is not None else "-"
                ssnap = sensors.snapshot() if sensors is not None else {}
                # Before the lock the front block above is skipped, so fall back to
                # the live snapshot here -> front is visible during pre-lock too.
                if front_dist is not None:
                    front_s = f"{front_dist}"
                elif ssnap and ssnap.get("front_valid"):
                    front_s = f"{ssnap['front_mm']}"
                    front_cells = ssnap.get("front_cells", 0)
                else:
                    front_s = "-"
                sides_s = (f"L={ssnap.get('left_mm')} "
                           f"RF={ssnap.get('right_front_mm')} "
                           f"RR={ssnap.get('right_rear_mm')}") if ssnap else "-"
                logger.debug(
                      "[WATCH] both=%s seen=%s near=%s close=%s latch=%s dir=%s "
                      "confirm=%d/%d low=%d/%d arm=%d front=%s(%dc) "
                      "fwin=%d/%d fnear=%d/%d farm=%d pre=%d/%d "
                      "votes(L=%d/R=%d) [%s] corners=%d/%d | %s",
                      both_lines_present(result), both_seen,
                      result['nearest_color'], close_s, latched_turn, track_dir,
                      confirm, CONFIRM_FRAMES,
                      outer_low_frames, SENSOR_CORNER_FRAMES, int(sensor_armed),
                      front_s, front_cells,
                      front_win_frames, FRONT_CORNER_FRAMES,
                      front_near_frames, FRONT_EMERGENCY_FRAMES, int(front_armed),
                      prelock_frames, PRELOCK_CORNER_FRAMES,
                      cam_left_votes, cam_right_votes, sides_s,
                      corners_done, TOTAL_CORNERS, steer_info)
                if DEBUG_FRONT_MATRIX and sensors is not None:
                    rows = sensors.snapshot()["front_matrix"]
                    if rows:
                        cells = " / ".join(
                            " ".join(f"{v:4d}" for v in r) for r in rows)
                        logger.debug("        [FRONT 4x4] %s  (row%s->front_mm=%s)",
                                     cells, FRONT_BARRIER_ROW, front_s)

            if can_show:
                view = apply_color_tuning(frame, detector.color_params)
                draw_result(view, result)
                cv2.imshow("Challenge 01 v4 - corner + PID + VL53L8 front", view)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    aborted = True
                    break

            camera_corner = confirm >= CONFIRM_FRAMES and latched_turn is not None
            if (not camera_corner and not sensor_corner and not front_corner
                    and not prelock_corner):
                continue

            # ---- CORNER state (no detection, no PID) ----
            # Lock the track direction on the FIRST corner, then reuse it for all.
            # WRO: the car must AUTO-DETECT direction -> corner-1 direction comes
            # ONLY from the camera vote. If a non-camera trigger (front/sensor/
            # prelock) wants to fire but the camera has not decided yet, we do NOT
            # guess -- we keep approaching so the camera can read the corner. Only an
            # emergency (wall about to be hit) forces a sensor-only last-resort turn.
            if track_dir is None:
                cam_dir = camera_dir_vote(cam_left_votes, cam_right_votes)
                if cam_dir is not None:
                    track_dir, dir_src = cam_dir, (
                        f"camera({cam_left_votes}L/{cam_right_votes}R)")
                else:
                    snap_now = sensors.snapshot() if sensors is not None else {}
                    fd_now = (snap_now.get("front_mm", -1)
                              if snap_now.get("front_valid") else -1)
                    # Emergency = wall about to be hit ahead, OR the right wall has
                    # already collapsed in front (so "keep approaching" would just
                    # scrape it -- don't loop into the wall waiting for a vote).
                    emergency = (0 < fd_now <= FRONT_EMERGENCY_MM) or right_in_front
                    if not emergency:
                        # camera undecided + not an emergency -> keep approaching so
                        # the camera can vote; re-check next frame (do NOT turn).
                        continue
                    track_dir = emergency_dir(snap_now, right_in_front)
                    dir_src = "EMERGENCY-sensor(no camera vote)"
                logger.info("*** Track direction LOCKED: %s via %s "
                            "(outer wall = %s sensor; votes L=%d/R=%d) ***",
                            track_dir, dir_src, outer_for(track_dir),
                            cam_left_votes, cam_right_votes)
                # Direction known -> make sure the front matrix poll is on for the
                # remaining corners (already on if FRONT_ENABLE_FROM_START).
                if sensors is not None:
                    sensors.set_front_enabled(True)
            direction = track_dir
            if camera_corner:
                trigger = "camera"
            elif prelock_corner:
                trigger = "prelock(geom)"
            elif front_corner:
                trigger = f"front(<{FRONT_CORNER_MM:.0f})"
            else:
                trigger = f"sensor(outer<{SENSOR_CORNER_MM:.0f})"
            lap = corners_done // CORNERS_PER_LAP + 1
            logger.info("Corner %d/%d (lap %d)  turn -> %s  (via %s, latch=%s)",
                        corners_done + 1, TOTAL_CORNERS, lap, direction,
                        trigger, latched_turn)

            servo.steer_delta(steer_us_for(direction), channel=0)
            logger.info("  TURN %s (%.2fs)", direction.upper(), TURN_TIME_S)
            if sleep_with_abort(TURN_TIME_S):
                aborted = True
                break
            servo.center_steering()

            corners_done += 1
            confirm = 0
            both_seen = False
            latched_turn = None
            outer_low_frames = 0
            sensor_armed = False
            front_win_frames = 0
            front_near_frames = 0
            front_armed = False
            prelock_frames = 0

            # Post-turn recovery: run the wall-follower (detection OFF) through the
            # cooldown instead of coasting blind-straight. The now-calibrated
            # follower straightens the car and pulls it back to the target lane
            # BEFORE corner detection resumes, so a mis-aimed turn exit no longer
            # fires a false corner on the new straight (zone-2 corner 3 -> inner
            # wall). Reset filters first so the follower starts fresh on the new
            # wall; do NOT reset after -- keep the lane state it just established.
            pid.reset()
            hwf.reset()
            logger.info("  cooldown follow %.2fs (no detection)", CORNER_COOLDOWN_S)
            cd_deadline = time.monotonic() + CORNER_COOLDOWN_S
            while time.monotonic() < cd_deadline:
                if button_pressed():
                    aborted = True
                    break
                steer_straight(outer_for(track_dir))   # follower steers; no detect
                time.sleep(0.01)
            if aborted:
                break

            # Do NOT force-arm the front here. Let it re-arm NATURALLY (in the front
            # block) only once the road ahead is seen CLEAR -- front INVALID or
            # >= FRONT_ARM_MM. A real next corner exits onto a clear straight (front
            # starts >1400 -> arms -> fires on the approach ramp). A BAD/over-rotated
            # exit leaves a wall ~480 mm dead ahead with no ramp; force-arming used
            # to fire a false corner on it and turn the car into the INNER wall
            # (lap-completing corner, both directions). Natural re-arm suppresses
            # that false fire; the emergency failsafe still catches a genuine wall.
            front_armed = False

        # ---- Finish ----
        if not aborted:
            logger.info("All %d corners done. Finish straight %.2fs",
                        TOTAL_CORNERS, FINISH_STRAIGHT_S)
            servo.center_steering()
            aborted = sleep_with_abort(FINISH_STRAIGHT_S)

        motor_pulse(motor_stop)              # pulse so the button always halts it
        servo.center_steering()
        if aborted:
            logger.info("ABORTED. Motor stopped.")
        else:
            logger.info("DONE: laps completed. Motor stopped.")

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt: stopping.")
        motor_pulse(motor_stop)

    except Exception:
        # Fatal: log the full traceback so an unattended systemd run leaves a
        # diagnosable record, then re-raise so the unit exits non-zero (Restart=
        # on-failure can act). The finally block still stops the motor first.
        logger.exception("FATAL exception in main loop -- stopping motor.")
        try:
            motor_pulse(motor_stop)
        except Exception:
            logger.exception("Failed to stop motor during fatal handler.")
        raise

    finally:
        logger.info("Clean shutdown: stopping motor and releasing hardware.")
        try:
            motor_pulse(motor_stop)
        except Exception:
            logger.exception("Failed to stop motor during cleanup.")
        servo.center_steering()
        servo.close()
        if sensors:
            sensors.close()
        if not shared:
            motor.close()
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        logger.info("Cleaned up. Exit.")


if __name__ == "__main__":
    # No direction argument: WRO requires the car to AUTO-DETECT the track
    # direction at runtime from the camera (see camera_dir_vote / corner-1 lock).
    # setup_logging() FIRST so even an init-time crash is captured with a
    # traceback in logs/main.log (and stdout -> journalctl under systemd).
    setup_logging()
    try:
        main()
    except SystemExit:
        raise
    except Exception:
        logger.exception("Unhandled exception at top level -- exiting.")
        raise
