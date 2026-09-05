"""
robot_io.py  --  the layer between the hardware and the policy.

EXTRACTED so main_challenge_v6 stands on its own. v6 used to reach into
main_challenge_01_v4 for SensorAdapter/HeadingWallFollower and into
main_challenge_v5 for the GPIO helpers, valid_tof and CornerTriggers. That
dragged two whole superseded programs -- and through v4, turn_models.py and
color_tuning.py, neither of which v6 uses at all -- into the running system.

Nothing is COPIED here. v4 and v5 now import these names back from this file,
so there is still exactly one definition of each; only the direction of the
dependency changed. Two copies always drift, and a sign convention that drifts
is how a car ends up turning the wrong way.

Tuning lives in behavior_manager.py, as everywhere else. The only value defined
here is BUTTON_PIN, which identifies this machine rather than how it drives.
"""

import time
from collections import deque
import statistics

import Jetson.GPIO as GPIO

import behavior_manager as B
from behavior_manager import (FRONT_ARM_MM, FRONT_CORNER_FRAMES,
                              FRONT_CORNER_MM, FRONT_MIN_CELLS,
                              FRONT_MIN_TRUST_MM, SENSOR_CORNER_FRAMES,
                              SENSOR_CORNER_MM, SENSOR_REARM_MM,
                              TOF_MAX_MM, TOF_MIN_MM)
from sensor_distance_v3 import SideSensorsV3

BUTTON_PIN = 7                   # BOARD numbering; identifies this machine


def valid_tof(v):
    """A ToF reading, or None when it cannot be believed."""
    if v is None:
        return None
    return float(v) if TOF_MIN_MM <= float(v) <= TOF_MAX_MM else None
def set_pin():
    GPIO.setwarnings(False)
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(BUTTON_PIN, GPIO.IN)
    time.sleep(1)


def button_pressed() -> bool:
    return GPIO.input(BUTTON_PIN) == GPIO.HIGH


def wait_for_button_press():
    while button_pressed():
        time.sleep(0.02)
    while not button_pressed():
        time.sleep(0.02)
    time.sleep(0.05)
    while button_pressed():
        time.sleep(0.02)


def sleep_with_abort(duration_s: float) -> bool:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        if button_pressed():
            return True
        time.sleep(0.01)
    return False
class CornerTriggers:
    """The non-camera corner detectors, collapsed to a single boolean.

    v4 kept four independent triggers with their own counters. The state machine
    only needs to know "a corner is here", so the counting lives here and the
    Behavior Manager sees Percept.corner_trigger.
    """

    def __init__(self):
        self.outer_low = 0
        self.sensor_armed = False
        self.front_win = 0
        self.front_armed = True

    def reset_after_corner(self):
        self.outer_low = 0
        self.sensor_armed = False
        self.front_win = 0
        self.front_armed = False

    def update(self, outer_dist, snap, direction_locked: bool) -> str | None:
        """Returns the trigger name that fired, or None."""
        if not direction_locked:
            return None

        # Outer wall collapsing = the wall has wrapped in front of the side
        # sensor. Re-arms only after the wall recedes, so the post-turn recovery
        # dip cannot fire the next corner.
        if outer_dist is not None:
            if outer_dist >= SENSOR_REARM_MM:
                self.sensor_armed = True
            self.outer_low = self.outer_low + 1 if outer_dist < SENSOR_CORNER_MM else 0
            if self.sensor_armed and self.outer_low >= SENSOR_CORNER_FRAMES:
                return "outer-wall"

        if snap and snap.get("front_valid"):
            fd = snap.get("front_mm")
            cells = snap.get("front_cells", 0)
            if fd is not None and fd >= FRONT_ARM_MM:
                self.front_armed = True
            in_window = (fd is not None
                         and FRONT_MIN_TRUST_MM <= fd <= FRONT_CORNER_MM
                         and cells >= FRONT_MIN_CELLS)
            self.front_win = self.front_win + 1 if in_window else 0
            if self.front_armed and self.front_win >= FRONT_CORNER_FRAMES:
                return "front-matrix"
        else:
            self.front_armed = True          # barrier absent == far == armed
        return None


class SensorAdapter(SideSensorsV3):
    """New-firmware reader (SideSensorsV3) with v2's snapshot keys layered on top.

    v2's code expects right_mm / left_mm / valid and the front_* keys. The new
    reader exposes right_front_mm / right_rear_mm / left_mm and the VL53L8CX
    matrix front_*. We follow the RIGHT_FRONT wall, so right_mm == right_front.
    """

    def snapshot(self) -> dict:
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
        self._rf_hist = deque(maxlen=self.window)
        self._rr_hist = deque(maxlen=self.window)
        self.last_u = 0.0

    def _valid(self, v):
        return v is not None and 0 < v <= self.max_valid_mm

    @staticmethod
    def _clamp(x, lo, hi):
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
