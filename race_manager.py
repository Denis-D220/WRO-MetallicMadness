"""
race_manager.py  --  remembers WHERE the robot is in the three-lap race.

This is the top of the v5 hierarchy: it owns direction, corner count, lap count,
finishing, and which pillars have already been dealt with. It holds NO steering
logic and touches NO hardware, so it can be unit-tested off the robot -- which
matters, because a lap-counting bug only shows up 90 seconds into a run.

    RACE MANAGER          <- this file: direction / lap / corner / finishing
        v
    BEHAVIOR MANAGER      <- behavior_manager.py: which maneuver owns the car
        v
    PID / ToF             <- main_challenge_v5.py: how much to steer

WHY CORNERS ARE DEBOUNCED: v4 learned this the hard way. A corner fires from any
of four triggers (camera, side-wall collapse, front matrix, pre-lock geometry),
and the same physical corner can fire more than one of them within a few frames.
Counting each as a separate corner would finish the race a lap early -- so a
corner is only counted once per CORNER_LOCKOUT_S.

WRO DIRECTION RULE: the car must AUTO-DETECT the track direction at runtime; a
preset is not allowed. Direction is therefore locked from camera votes at the
first corner and reused for the rest of the race (see v4's camera_dir_vote).
"""

import time

import behavior_manager as B


# Tuning lives in behavior_manager.py -- edit it there, not here.
CORNERS_PER_LAP = B.CORNERS_PER_LAP
LAPS_TO_FINISH = B.LAPS_TO_FINISH
TOTAL_CORNERS = CORNERS_PER_LAP * LAPS_TO_FINISH      # 12

CORNER_LOCKOUT_S = B.CORNER_LOCKOUT_S

DIR_MIN_VOTES = B.DIR_MIN_VOTES
DIR_CONFIDENCE = B.DIR_CONFIDENCE


class RaceManager:
    """Race bookkeeping: direction, corners, laps, finishing, pillar memory."""

    def __init__(self, total_corners: int = TOTAL_CORNERS,
                 corners_per_lap: int = CORNERS_PER_LAP,
                 lockout_s: float = CORNER_LOCKOUT_S,
                 clock=time.monotonic):
        self.total_corners = total_corners
        self.corners_per_lap = corners_per_lap
        self.lockout_s = lockout_s
        self._clock = clock

        self.direction = None          # "left" | "right" (turn direction)
        self.direction_source = None
        self.corners_done = 0
        self.last_corner_t = None
        self.finishing = False
        self.finished = False
        self._finish_started = None

        self._left_votes = 0
        self._right_votes = 0
        self._pillars_handled = []     # (color, corner_index) already passed

    # ---------------- direction ----------------

    def vote_direction(self, turn: str, trustworthy: bool) -> None:
        """Count a camera vote for the first corner's turn direction.

        `trustworthy` is the caller's judgement that this frame is the reliable
        kind -- both colored lines visible, or a single CLOSE line. A lone FAR
        line reads as "nearest" and inverts, which is what made v4 lock the wrong
        way before CAM_DIR_LONE_CLOSE was added.
        """
        if self.direction is not None or not trustworthy:
            return
        if turn == "left":
            self._left_votes += 1
        elif turn == "right":
            self._right_votes += 1

    def try_lock_direction(self) -> str | None:
        """Lock the direction if the votes are confident. None = keep waiting."""
        if self.direction is not None:
            return self.direction
        total = self._left_votes + self._right_votes
        if total < DIR_MIN_VOTES:
            return None
        if self._left_votes >= self._right_votes and \
                self._left_votes >= DIR_CONFIDENCE * total:
            return self._lock("left", f"camera({self._left_votes}L/{self._right_votes}R)")
        if self._right_votes > self._left_votes and \
                self._right_votes >= DIR_CONFIDENCE * total:
            return self._lock("right", f"camera({self._left_votes}L/{self._right_votes}R)")
        return None

    def force_direction(self, turn: str, source: str) -> str:
        """Last-resort lock (emergency geometry) when the camera never decided."""
        return self._lock(turn, source)

    def _lock(self, turn: str, source: str) -> str:
        self.direction = turn
        self.direction_source = source
        return turn

    @property
    def outer_wall(self) -> str | None:
        """Which side the OUTER wall is on, given the turn direction.

        Left turns (CCW) put the outer wall on the RIGHT, and vice versa. The
        wall follower and the sensor corner fallback both key off this.
        """
        if self.direction is None:
            return None
        return "right" if self.direction == "left" else "left"

    # ---------------- corners and laps ----------------

    def can_count_corner(self) -> bool:
        """False while inside the lockout -- a re-trigger of the same corner."""
        if self.last_corner_t is None:
            return True
        return (self._clock() - self.last_corner_t) >= self.lockout_s

    def count_corner(self, trigger: str = "") -> bool:
        """Record ONE completed corner. Returns False if debounced away."""
        if self.finished or not self.can_count_corner():
            return False
        self.corners_done += 1
        self.last_corner_t = self._clock()
        if self.corners_done >= self.total_corners and not self.finishing:
            self.finishing = True
            self._finish_started = self.last_corner_t
        return True

    @property
    def lap(self) -> int:
        """1-based lap currently being driven (capped at LAPS_TO_FINISH)."""
        return min(self.corners_done // self.corners_per_lap + 1, LAPS_TO_FINISH)

    @property
    def corner_in_lap(self) -> int:
        """1-based index of the NEXT corner within the current lap."""
        return self.corners_done % self.corners_per_lap + 1

    @property
    def corners_remaining(self) -> int:
        return max(0, self.total_corners - self.corners_done)

    def finish_elapsed(self) -> float:
        """Seconds since finishing mode began (0 if not finishing)."""
        if self._finish_started is None:
            return 0.0
        return self._clock() - self._finish_started

    def mark_finished(self) -> None:
        """Called once the car has crept back into the start zone and stopped."""
        self.finished = True

    # ---------------- pillar memory ----------------

    def note_pillar(self, color: str) -> None:
        """Remember a pillar passed, tagged with the corner it happened at.

        Stops a pillar that is still in view AFTER being passed from starting a
        second avoidance maneuver -- the recovery steer swings it back into
        frame, which without memory reads as a brand new pillar.
        """
        self._pillars_handled.append((color, self.corners_done))

    def pillars_this_sector(self) -> int:
        return sum(1 for _c, i in self._pillars_handled if i == self.corners_done)

    @property
    def pillars_handled(self) -> int:
        return len(self._pillars_handled)

    # ---------------- reporting ----------------

    def status(self) -> str:
        d = self.direction or "?"
        state = "FINISHED" if self.finished else ("FINISHING" if self.finishing
                                                  else "racing")
        return (f"dir={d} lap={self.lap}/{LAPS_TO_FINISH} "
                f"corner={self.corner_in_lap}/{self.corners_per_lap} "
                f"({self.corners_done}/{self.total_corners}) "
                f"pillars={self.pillars_handled} {state}")
