"""
pid_line_follower.py  --  keep the car centered on the track using L/R sensors.

Reads the left and right side distances, computes how far the car is from the
track centerline, and runs a PID controller to produce a steering correction
(in servo microseconds) that re-centers the car.

Error definition (width-independent):
    error = (right_mm - left_mm) - center_offset

    - Centered  -> left == right -> error ~ 0
    - Car drifts toward the LEFT wall  -> left shrinks, right grows -> error > 0
    - Car drifts toward the RIGHT wall -> error < 0

    Using the DIFFERENCE (not the sum) means it works for both the 1000 mm and
    600 mm corridors without changes.

Output:
    steering_us = steer_sign * PID(error), clamped to +/- out_limit.
    steer_sign flips the direction if your servo's sign is opposite (test once:
    push the car toward the left wall; the correction should steer it right).

Invalid readings (sensor not ready, 0, or > max_valid_mm, e.g. a wall gap at a
corner) freeze the integrator and return the last command so the car does not
jerk on a bad sample.

HEADING (optional, kh > 0)
    The error above is a POSITION. A car can sit exactly on the centreline while
    pointed 30 degrees across it, and this controller reads that as zero error --
    which is why it recovers from a corner exit so slowly: it has to wait for the
    angle to TURN INTO a position error before it reacts at all, and by then it
    is already at the wall.

    The two right-side sensors measure the angle directly. Parallel to the wall,
    RF and RR read the same (plus a fixed mounting offset); nose swung away from
    the wall, RF > RR. Feeding that in as a second term makes this a controller
    on where the car IS *and* where it is POINTED:

        u = steer_sign * (kp*err + ki*int + kd*d/dt + kh*heading)

    Same sign convention throughout: positive u steers TOWARD the right wall,
    and heading > 0 (nose away from the right wall) asks for exactly that.

    THE OFFSET IS LEARNED, NOT MEASURED. RF - RR is only zero when the two
    sensors are perfectly aligned, and bench readings of a "parallel" car came
    out +152, +174 and -46 depending on the pose -- the measurement was of the
    pose, not the mounting. So the offset is instead learned while driving: on a
    straight, with the car centred and the steering quiet, RF - RR IS the
    mounting offset by definition. Until enough of those samples exist the
    heading term stays exactly 0, so an uncalibrated car behaves as it did
    before.
"""

import statistics
import time
from collections import deque


class LineFollowerPID:
    def __init__(
        self,
        kp: float = 0.30,
        ki: float = 0.00,
        kd: float = 0.10,
        out_limit: float = 150.0,
        center_offset: float = 0.0,
        steer_sign: int = 1,
        max_error: float = 400.0,
        deadband: float = 8.0,
        max_valid_mm: float = 2000.0,
        integral_limit: float = 300.0,
        median_window: int = 3,
        slew_limit: float = 0.0,
        kh: float = 0.0,
        rr_offset: float = 0.0,
        offset_ref_mm: float = 0.0,   # 0 = treat rr_offset as a fixed gap
        heading_max: float = 200.0,
        learn_offset=None,       # None = learn only if no offset was given
        learn_error_mm: float = 80.0,
        learn_steer_us: float = 55.0,
        learn_quiet_frames: int = 3,
        learn_samples: int = 12,
        learn_window: int = 41,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = out_limit
        self.center_offset = center_offset
        self.steer_sign = steer_sign
        self.max_error = max_error
        self.deadband = deadband
        self.max_valid_mm = max_valid_mm
        self.integral_limit = integral_limit
        # Input noise rejection: median over the last N valid samples per side.
        self.median_window = max(1, int(median_window))
        # Output slew-rate limit: max change in steering_us per update (0 = off).
        self.slew_limit = slew_limit

        # Heading term (see the module docstring). kh = 0 disables it entirely.
        self.kh = kh
        self.rr_offset = rr_offset
        self.offset_ref_mm = offset_ref_mm
        self.heading_max = heading_max
        self.learn_offset = (rr_offset == 0.0) if learn_offset is None             else learn_offset
        self.learn_error_mm = learn_error_mm
        self.learn_steer_us = learn_steer_us
        self.learn_quiet_frames = max(1, int(learn_quiet_frames))
        self.learn_samples = max(1, int(learn_samples))
        self.learn_window = max(1, int(learn_window))
        # Survives reset(): the mounting does not change between corners, and
        # relearning from scratch after every turn would mean the heading term
        # is dead exactly when it is needed most.
        self._offset_hist = deque(maxlen=self.learn_window)

        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._last_u = 0.0
        self._left_hist = deque(maxlen=self.median_window)
        self._right_hist = deque(maxlen=self.median_window)
        self._single_hist = deque(maxlen=self.median_window)
        self._rf_hist = deque(maxlen=self.median_window)
        self._rr_hist = deque(maxlen=self.median_window)
        self._heading = 0.0
        self._quiet_run = 0

    # -----------------------------------------------------

    def _valid(self, value) -> bool:
        return value is not None and 0 < value <= self.max_valid_mm

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def _pid_step(self, error, now, heading=0.0):
        """Core PID on an already-formed error. Updates state, returns (u, terms).

        `heading` is an offset-corrected RF-RR in mm, already validated by
        _heading_term(); 0.0 means "no angle information", not "parallel".
        """
        if self._prev_time is None:
            dt = 0.0
        else:
            dt = now - self._prev_time
        self._prev_time = now

        if dt > 0:
            self._integral += error * dt
            self._integral = self._clamp(self._integral,
                                         -self.integral_limit, self.integral_limit)
            derivative = (error - self._prev_error) / dt
        else:
            derivative = 0.0

        p = self.kp * error
        i = self.ki * self._integral
        d = self.kd * derivative
        h = self.kh * heading

        u = self.steer_sign * (p + i + d + h)
        u = self._clamp(u, -self.out_limit, self.out_limit)

        # Slew-rate limit: a single noisy frame cannot slam the steering.
        if self.slew_limit > 0:
            u = self._clamp(u, self._last_u - self.slew_limit,
                            self._last_u + self.slew_limit)

        self._prev_error = error
        self._last_u = u
        return u, {"p": p, "i": i, "d": d, "h": h}

    def last_command(self):
        """The steering last commanded. Used to HOLD through a sensor dropout."""
        return self._last_u

    def rr_offset_estimate(self):
        """The RF-RR mounting offset in use, or None if not known yet.

        A configured offset counts: callers use this to decide whether the
        heading signal can be trusted at all (main_challenge's turn_complete
        does), and "configured" is at least as trustworthy as "learned".
        """
        if not self.learn_offset:
            return self.rr_offset or None
        if len(self._offset_hist) < self.learn_samples:
            return None
        return statistics.median(self._offset_hist)

    def _heading_term(self, rf_mm, rr_mm, raw_error):
        """Offset-corrected RF - RR in mm. 0.0 when it cannot be trusted.

        Returning 0.0 for "unknown" is deliberate: an unknown angle must not
        move the servo, and a car with no rear sensor keeps the exact behaviour
        it had before this term existed.
        """
        if self.kh == 0.0 or not (self._valid(rf_mm) and self._valid(rr_mm)):
            return 0.0

        self._rf_hist.append(float(rf_mm))
        self._rr_hist.append(float(rr_mm))
        raw = statistics.median(self._rf_hist) - statistics.median(self._rr_hist)

        if self.learn_offset:
            self._quiet_run = (self._quiet_run + 1
                               if abs(self._last_u) <= self.learn_steer_us
                               else 0)
            if (self._quiet_run >= self.learn_quiet_frames
                    and abs(raw_error) <= self.learn_error_mm):
                self._offset_hist.append(raw)
            offset = self.rr_offset_estimate()
            if offset is None:
                return 0.0                # not calibrated yet -> no angle term
        else:
            offset = self.rr_offset

        if self.offset_ref_mm > 0:
            rng = max(1.0, 0.5 * (float(rf_mm) + float(rr_mm)))
            offset = offset * rng / self.offset_ref_mm
        heading = raw - offset
        # An out-of-band difference is an RR dropout, not a 90-degree car.
        if abs(heading) > self.heading_max:
            return 0.0
        return heading

    def update(self, left_mm, right_mm, rf_mm=None, rr_mm=None):
        """
        Center the car using BOTH side sensors (difference error), and -- when
        kh > 0 and the two right sensors are available -- damp with the car's
        ANGLE to the right wall as well. See the module docstring.

        Width-independent, but needs both walls reliable.
        Returns (steering_us, info).
        """
        now = time.monotonic()

        if not (self._valid(left_mm) and self._valid(right_mm)):
            self._prev_time = now
            return self._last_u, {"valid": False, "error": None,
                                  "p": 0.0, "i": 0.0, "d": 0.0, "h": 0.0,
                                  "heading": None,
                                  "left_f": None, "right_f": None}

        self._left_hist.append(float(left_mm))
        self._right_hist.append(float(right_mm))
        left_f = statistics.median(self._left_hist)
        right_f = statistics.median(self._right_hist)

        raw_error = (right_f - left_f) - self.center_offset
        # Learn the offset from the RAW error: the deadband would otherwise
        # report 0 for a car that is merely close to centred.
        heading = self._heading_term(rf_mm, rr_mm, raw_error)
        self._heading = heading

        error = self._clamp(raw_error, -self.max_error, self.max_error)
        if abs(error) < self.deadband:
            error = 0.0

        u, terms = self._pid_step(error, now, heading)
        return u, {"valid": True, "error": error, **terms,
                   "heading": heading, "rr_offset": self.rr_offset_estimate(),
                   "left_f": left_f, "right_f": right_f}

    def update_single(self, distance_mm, target_mm, side_sign=1):
        """
        Follow ONE wall at a fixed distance (robust when the other wall is
        unreliable, e.g. the discontinuous inner wall).

        side_sign = +1 to follow the RIGHT wall, -1 to follow the LEFT wall, so
        the steering sign comes out consistent with update().
        """
        now = time.monotonic()

        if not self._valid(distance_mm):
            self._prev_time = now
            return self._last_u, {"valid": False, "error": None,
                                  "p": 0.0, "i": 0.0, "d": 0.0, "wall_f": None}

        self._single_hist.append(float(distance_mm))
        wall_f = statistics.median(self._single_hist)

        error = side_sign * (wall_f - target_mm)
        error = self._clamp(error, -self.max_error, self.max_error)
        if abs(error) < self.deadband:
            error = 0.0

        u, terms = self._pid_step(error, now)
        return u, {"valid": True, "error": error, **terms, "wall_f": wall_f}
