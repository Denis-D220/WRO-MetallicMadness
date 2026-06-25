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

        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._last_u = 0.0
        self._left_hist = deque(maxlen=self.median_window)
        self._right_hist = deque(maxlen=self.median_window)
        self._single_hist = deque(maxlen=self.median_window)

    # -----------------------------------------------------

    def _valid(self, value) -> bool:
        return value is not None and 0 < value <= self.max_valid_mm

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def _pid_step(self, error, now):
        """Core PID on an already-formed error. Updates state, returns (u, terms)."""
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

        u = self.steer_sign * (p + i + d)
        u = self._clamp(u, -self.out_limit, self.out_limit)

        # Slew-rate limit: a single noisy frame cannot slam the steering.
        if self.slew_limit > 0:
            u = self._clamp(u, self._last_u - self.slew_limit,
                            self._last_u + self.slew_limit)

        self._prev_error = error
        self._last_u = u
        return u, {"p": p, "i": i, "d": d}

    def update(self, left_mm, right_mm):
        """
        Center the car using BOTH side sensors (difference error).
        Width-independent, but needs both walls reliable.
        Returns (steering_us, info).
        """
        now = time.monotonic()

        if not (self._valid(left_mm) and self._valid(right_mm)):
            self._prev_time = now
            return self._last_u, {"valid": False, "error": None,
                                  "p": 0.0, "i": 0.0, "d": 0.0,
                                  "left_f": None, "right_f": None}

        self._left_hist.append(float(left_mm))
        self._right_hist.append(float(right_mm))
        left_f = statistics.median(self._left_hist)
        right_f = statistics.median(self._right_hist)

        error = (right_f - left_f) - self.center_offset
        error = self._clamp(error, -self.max_error, self.max_error)
        if abs(error) < self.deadband:
            error = 0.0

        u, terms = self._pid_step(error, now)
        return u, {"valid": True, "error": error, **terms,
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
