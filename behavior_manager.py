"""
behavior_manager.py  --  decides WHICH maneuver owns the car, and blends the
other models in as CONSTRAINTS rather than as competing steering commands.

THE CENTRAL IDEA (and the reason this is a state machine, not an if/elif chain):

    One behavior OWNS the maneuver. The others constrain it.

Evaluating `if pillar: elif wall: elif corner:` every frame lets the owner change
between consecutive frames -- pillar, wall, pillar, corner, pillar -- and the
steering jitters instead of completing anything. Here a behavior is ENTERED and
keeps control until its own exit condition fires. Only an emergency can preempt.

    EMERGENCY  ->  PILLAR  ->  WALL  ->  CORNER  ->  CRUISE
    (priority order for ENTERING a state, not a per-frame re-vote)

"ACTIONABLE" is what stops a pillar 2 m away from hijacking the car: the pillar
model detects it, but PILLAR_APPROACH only starts once its bbox area says it is
close enough to matter.

CONSTRAINTS are how a pillar pass avoids ending in a wall. The owning behavior
proposes a steering value; the wall class and the ToF distances then clamp it:

    Wall AI says WHAT KIND of correction is needed.
    ToF/PID says HOW MUCH.

No hardware, no cv2, no model loading here -- everything is plain data in and a
Decision out, so the whole state machine is unit-testable off the robot.
"""

from dataclasses import dataclass, field


# ---------------- states ----------------

CRUISE = "CRUISE"
PILLAR_APPROACH = "PILLAR_APPROACH"
PILLAR_PASS = "PILLAR_PASS"
PILLAR_RECOVER = "PILLAR_RECOVER"
CORNER_APPROACH = "CORNER_APPROACH"
CORNER_TURN = "CORNER_TURN"
CORNER_EXIT = "CORNER_EXIT"
EMERGENCY = "EMERGENCY"
FINISHING = "FINISHING"
DONE = "DONE"



GREEN_PASSES_LEFT = True

# ---- CAMERA COLOUR TUNING --------------------------------------------------

CAMERA_TUNING = False

CAMERA_TUNING_STAGE = "resized"


PILLAR_MAX_AREA = 150000.0
PILLAR_ACTIONABLE_FRAMES = 2      # consecutive frames before a pillar owns the
                                  # car; one bad frame must not start a maneuver

PILLAR_ACTIONABLE_AREA = 30000.0  # bbox px^2 before a pillar influences the car
                                  # PILLAR_ACT_AREA). Smaller = react earlier.
PILLAR_PASS_AREA = 6000.0         # bbox this large = alongside it; start passing
PILLAR_LOST_FRAMES = 6            # frames without the pillar before RECOVER
PILLAR_COLOR_SWITCH_FRAMES = 3    # frames of a DIFFERENT colour before the pass
PILLAR_RECOVER_FRAMES = 8        # frames of straightening before CRUISE
PILLAR_STEER_US = 120.0           # base avoidance steer
PILLAR_SPEED_PCT = 50             # slow down while passing

# THE TURN ITSELF.
TURN_STEER_US = 250.0             # full lock during the blind turn
TURN_SPEED_PCT = 75               # speed DURING the turn -- stated explicitly.
TURN_TIME_S = 3.4              

CORNER_TURN_PROFILE = {
    "close_turn_left":  (-270.0, 3.9),
    "close_turn_right": (+270.0, 3.9),
    "turn_left":        (-260.0, 3.5),
    "turn_right":       (+260.0, 3.4),
    "open_turn_left":   (-200.0, 2.8),
    "open_turn_right":  (+200.0, 2.8),
}
CORNER_CLASS_MIN_CONF = 0.75      # below this the classifier does not decide
CORNER_CLASS_FRAMES = 3           # consecutive agreeing frames before committing


CORNER_LOCK_MIRRORS_CLASS = True

# Scaled with CRUISE_SPEED_PCT so the approach stays a real slowdown: against
# 58 a value of 55 would be a 3-point drop, which is not one.
CORNER_APPROACH_SPEED_PCT = 52    # slow into the corner: more frames to decide,
                                  # and a tighter line through it

TURN_CLOSED_LOOP = True
TURN_MIN_TIME_S = 3.2            # closed-loop only: never end before this

TURN_MIN_TIME_FRAC = 0.75        # effective floor = min(TURN_MIN_TIME_S, frac x hold)

TURN_EXTEND_MAX_S = 0.25

TURN_DONE_MIN_RANGE_MM = 250.0

TURN_EXTEND_MIN_SIDE_MM = 150.0   # either side inside this: end on time
TURN_DONE_HEADING_MM = 60.0       # |RF - RR - offset| below this = parallel
TURN_DONE_HEADING_REF_MM = 450.0  # the range TURN_DONE_HEADING_MM was tuned at
TURN_DONE_HOLD_FRAMES = 3         # must hold, to reject a transient crossing

CORNER_DRAWBACK_PROFILE = {
    "close_turn_left":  (100.0, 0.20),
    "close_turn_right": (115.0, 0.20),
    "turn_left":        (90.0, 0.25),
    "turn_right":       (95.0, 0.25),
    "open_turn_left":   (80.0, 0.25),
    "open_turn_right":  (80.0, 0.25),
}

# The fallback, for turns the classifier did NOT name: the legacy
# direction-vote path and the forced-out-of-EMERGENCY escape, which both run the
# generic TURN_STEER_US arc and so have no class to look up.
DRAWBACK_STEER_US = 130.0         # counter-steer magnitude (opposite the turn)
DRAWBACK_TIME_S = 0.35            # how long to hold it; 0 disables entirely

DRAWBACK_SCALES_WITH_TIME = True  # False restores the flat, profile-sized value
DRAWBACK_TIME_FRAC_MIN = 0.50     # never below half
DRAWBACK_TIME_FRAC_MAX = 1.25     # never more than a quarter over


def drawback_for(name):
    """(microseconds, seconds) of counter-steer to apply after this turn type."""
    return CORNER_DRAWBACK_PROFILE.get(name,
                                       (DRAWBACK_STEER_US, DRAWBACK_TIME_S))


TURN_WALL_ASSIST = True
TURN_WALL_MIN_CONF = 0.80         # stricter than WALL_MIN_CONF: mid-turn the
                                  # camera sees poses the model never trained on
TURN_WALL_INNER_NEAR = 0.55       # near_* on the inside -> ease the lock to this
TURN_WALL_INNER_CLOSE = 0.80      # close_* is the milder class -> ease less
TURN_WALL_OUTER = 1.00            # wall on the OUTSIDE -> the turn is running
                                  # wide; full lock is exactly what it needs
TURN_WALL_MIN_SCALE = 0.45        # never ease below this: the car must still
                                  # come round, and a stalled turn is a DNF
TURN_WALL_SLEW = 0.15             # max change per update, so one frame cannot
                                  # collapse the lock mid-corner
TURN_WALL_TOF_MM = 300.0          # same veto as WALL_AI_MAX_MM: if the ToF for
                                  # that side says the wall is NOT near, the
                                  # classifier does not get to ease the turn


def _slew_scale(prev: float, target: float) -> float:
    step = max(-TURN_WALL_SLEW, min(TURN_WALL_SLEW, target - prev))
    return max(TURN_WALL_MIN_SCALE, min(1.0, prev + step))


def turn_wall_scale(turn, wall_class, wall_conf, left_mm, right_mm,
                    prev: float = 1.0):
    """(scale, why) -- multiply the turn lock by this. 1.0 = the blind turn.

    Pure data in, number out: no hardware, no models, so the whole assist is
    unit-testable off the robot like the rest of this file.
    """
    if not TURN_WALL_ASSIST or turn not in ("left", "right"):
        return 1.0, ""
    if wall_class not in WALL_ACT_CLASSES or wall_conf < TURN_WALL_MIN_CONF:
        return _slew_scale(prev, 1.0), ""

    side = "left" if wall_class.endswith("_left") else "right"
    dist = left_mm if side == "left" else right_mm
    if dist is not None and dist > TURN_WALL_TOF_MM:
        # The measurement contradicts the classification -- the same rule the
        # straights already use. near_right at 1.00 confidence has been logged
        # with the ToF reading 365mm on BOTH sides.
        return _slew_scale(prev, 1.0), f"vetoed {side}={dist:.0f}mm"

    if side != turn:                          # wall on the OUTSIDE of the turn
        return _slew_scale(prev, TURN_WALL_OUTER), f"{wall_class} outer"

    target = (TURN_WALL_INNER_NEAR if wall_class.startswith("near_")
              else TURN_WALL_INNER_CLOSE)
    return _slew_scale(prev, target), f"{wall_class} inner"

CORNER_APPROACH_CLOSENESS = 0.45  # corner-model closeness that starts APPROACH
CORNER_FIRE_CLOSENESS = 0.60      # v4's TRIGGER_CLOSENESS -- camera says "corner"
CORNER_EXIT_FRAMES = 8            # blind straightening after the timed turn

CORNER_FRONT_MM = 1100.0           # front ToF must be within this to commit
                                  # (v4's FRONT_CORNER_MM)
CORNER_CONFIRM_FRAMES = 2         # v4's CONFIRM_FRAMES: consecutive camera frames

CORNER_REARM_MM = 1200.0
CORNER_REARM_TIMEOUT_FRAMES = 25  # ~2s at 13Hz still unarmed -> arm anyway

CORNER_FRONT_DEAD_FRAMES = 20     # ~1.5s at 13Hz with no usable front reading

CORNER_REARM_FRAMES = 3           # ...for this many consecutive frames. ONE far
CORNER_CAMERA_ONLY_CLOSE = 0.88   # last resort, and ONLY when the front ToF has

CORNER_LONE_CLOSE = 0.65          # v4's CAM_DIR_LONE_CLOSE: a lone line
                                  # this close counts as a vote

WALL_STEER_US = {                 # what each wall class asks for, in microseconds
    "near_left": +95.0,          # wall close on the LEFT -> steer RIGHT
    "near_right": -95.0,
    "close_left": +85.0,
    "close_right": -85.0,
    "straight": 0.0,
    "barrier_front": 0.0,         # handled as an event, not a steer
}
WALL_ACT_CLASSES = {"near_left", "near_right", "close_left", "close_right"}

BARRIER_FRONT_DECIDES = False

BARRIER_NUDGE_ENABLED = False     # OFF with BARRIER_FRONT_DECIDES: this
                                  # steers on the same unreliable class
BARRIER_NUDGE_US = 90.0           # magnitude, signed by the locked direction
BARRIER_NUDGE_FRAMES = 4          # "a brief moment" -- ~0.2s at 19Hz
BARRIER_NUDGE_COOLDOWN = 12       # frames before it may fire again (~0.6s), so
                                  # a run of barrier_front frames produces one
                                  # nudge rather than a continuous lean
BARRIER_NUDGE_MIN_CONF = 0.75     # stricter than WALL_MIN_CONF: this steers
                                  # without any other evidence agreeing
BARRIER_NUDGE_MAX_FRONT_MM = 1500.0   # ...but None is allowed, because the
BARRIER_NUDGE_MIN_SIDE_MM = 400.0     # never lean toward the INSIDE of the
WALL_MIN_CONF = 0.65              # below this the wall model does not own the car
WALL_HOLD_FRAMES = 4              # frames a wall correction keeps control

EMERGENCY_FRONT_MM = 200.0        # ToF front closer than this = stop/avoid now
EMERGENCY_SIDE_MM = 90.0          # either side this close = scraping

EMERGENCY_STEER_FRAC = 1.0        # x turn_steer_us. 0.6 could not leave a wall.

EMERGENCY_SPEED_PCT = 45          # while steering away from a close wall
REVERSE_SPEED_PCT = 50            # while backing out of a wedge (see unwedge)

STUCK_SIDE_MM = 85.0              # closer than this counts toward a wedge
STUCK_SPREAD_MM = 10.0            # ...and moving less than this across the window
STUCK_FRAMES = 6                  # ~0.46s at 13Hz. Fewer risks firing on the
                                  # 98,98,105,105 case, which recovered alone.



REVERSE_CLEAR_MM = 80.0           # stop reversing once the wedged side reads
                                  # this. Above STUCK_SIDE_MM (55) with margin,
                                  # below EMERGENCY_SIDE_MM (90) so the escape
                                  # steer is still in charge afterwards.
REVERSE_POLL_S = 0.02             # how often to look while reversing
REVERSE_NUDGE_S = 0.50            # CEILING on the reverse. The car only
                                  # moves for the tail of this; the head is
                                  # spent spinning the drive up.
REVERSE_SETTLE_S = 0.25           # BOTH settles: the car must stop and the
REVERSE_COOLDOWN_S = 1.5          # a jammed car must not reverse every frame
REVERSE_MAX_ATTEMPTS = 6          # 0 = unlimited. A cap so a broken run ends
                                  # as a broken run rather than as a car
                                  # shuffling back and forth until time expires.
REVERSE_UNWEDGE = True            # the whole feature, off in one line

BARRIER_FRONT_RUN_FRAMES = 28     # ~3s at 13Hz. Between the 18 seen on a real
                                  # approach and the 115 driving into a wall.
BARRIER_FRONT_RUN_CONF = 0.90     # only confident frames extend the run
BARRIER_FRONT_RUN_DECIDES = True  # the whole rule, off in one line

EMERGENCY_ESCAPE_CLOSE_TURN = True
EMERGENCY_MAX_FRAMES = 12         # stuck this long -> force the corner instead.
                                  # Seen on the track: the car oscillated +-150
                                  # in EMERGENCY for 13 frames at a corner and
                                  # never turned. An extra turn beats a stall.

WALL_AI_MAX_MM = 300.0            # side reading above this vetoes the classifier

CONSTRAINT_SIDE_MM = 200.0        # inside this, cap steering TOWARD that wall
CONSTRAINT_HARD_MM = 130.0        # inside this, forbid steering toward it at all

WALL_CLAMP_EVERY_PATH = True      # False restores the old pillar-only clamp



PANE = (480, 270)
CAM_IDS = (0, 1)

CORNER_EVERY = 1
WALL_EVERY = 1

CLASS_BACKS_TRIGGER_FRAMES = 12          # ~0.6s at 19Hz

FRONT_BACKS_TRIGGER_FRAMES = 6           # ~0.45s at 13Hz. Above the 1 frame
                                         # that caused the false turn, below
                                         # the 8+ a real corner produced.

# How often the wall classifier runs DURING a turn (see TURN_WALL_ASSIST). Keep
# well below the main loop rate: the same loop polls the ToF for the parallel
# test, and camera work is what slows that poll down.
TURN_WALL_HZ = 10.0

CRUISE_SPEED_PCT = 58
FINISH_DRIVE_S = 1.2

WALL_FOLLOW_MODE = "center"              # hold the middle off BOTH walls
WALL_TARGET_MM = 470.0
WF_KH, WF_KD = 0.8, 0.6
WF_RR_OFFSET_MM = 214.0                  # RF - RR when parallel, AT the range below
WF_RR_OFFSET_REF_MM = 430.0              # ...the range that value was measured at
WF_OUT_LIMIT = 110.0
WF_DEADBAND = 8.0
WF_HEADING_MAX_MM = 200.0

PID_KP, PID_KI, PID_KD = 0.63, 0.00, 0.25

STEER_MAX_US = 340.0              # 89% of the left travel, 40us of margin

PID_OUT_LIMIT = 110.0
PID_DEADBAND = 8.0
PID_MEDIAN_WINDOW = 3
PID_SLEW_LIMIT = 100.0

PID_KH = 0.8

SIDE_MAX_AGE_S = 0.25                    # sides refresh at LR_HZ

BLIND_SLOW_FRAMES = 5             # stale side frames before backing off
BLIND_SPEED_PCT = 30              # ...and what to crawl at until they return

SIDE_STALE_DECAY_S = 0.5                 # NOT WIRED UP -- see above

LR_HZ = 20.0                             # side sensor poll rate
FRONT_HZ = 10.0                          # front matrix poll rate (heavier)
DEBUG_HZ = 5.0                           # debug line rate

# ---- race bookkeeping (race_manager.py) ------------------------------------
CORNERS_PER_LAP = 4
LAPS_TO_FINISH = 3
CORNER_LOCKOUT_S = 5.0                   # must exceed TURN_TIME_S + drawback
DIR_MIN_VOTES = 2
DIR_CONFIDENCE = 0.6

SENSOR_CORNER_MM = 250.0
SENSOR_CORNER_FRAMES = 3
SENSOR_REARM_MM = 330.0

FRONT_ARM_MM = 1000.0
FRONT_CORNER_MM = 900.0
FRONT_MIN_TRUST_MM = 330.0
FRONT_CORNER_FRAMES = 2
FRONT_MIN_CELLS = 2

PILLAR_PID_ONE_SIDED = True

PILLAR_PID_SECTIONS = 3            # sections PER CAMERA (6 across the strip)
PILLAR_PID_TARGET_SECTION = {      # index into the 6-section strip
    "green": 4,                    # INNER section of the RIGHT camera
    "red": 0,                      # outermost section of the LEFT camera
}

PILLAR_PID_KI = 0.0
PILLAR_PID_KD = 6.0
PILLAR_PID_DEADBAND = 0.20         # sections; inside this the pillar is placed
PILLAR_PID_SLEW = 150.0            # us per frame; scaled with the range
PILLAR_PID_MEDIAN = 3              # frames of median on the pillar position

PILLAR_PID_MIN_CONF = 0.35         # below this a detection does not steer

# Area bounds are the ones the state machine already trusts:
# PILLAR_ACTIONABLE_AREA (too far to matter) and PILLAR_MAX_AREA (a phantom).

PILLAR_PID_ENABLED = False

PILLAR_PID_BIAS_ONLY = True

PILLAR_BIAS_LIMIT_US = 200.0

PILLAR_FULL_SCALE_SECTIONS = 4.0
PILLAR_PID_KP = PILLAR_BIAS_LIMIT_US / PILLAR_FULL_SCALE_SECTIONS
PILLAR_PID_OUT_LIMIT = PILLAR_BIAS_LIMIT_US   # clipping below it would hide the
                                              # cap and make KP a lie

TURN_PILLAR_BIAS_LIMIT_US = 80.0  # 200 lock - 80 = 120 left, above the 112 floor

PILLAR_BIAS_CEILING_US = min(PID_OUT_LIMIT + PILLAR_BIAS_LIMIT_US,
                             STEER_MAX_US)   # never past the servo stop

TURN_PILLAR_ASSIST = True

TURN_PILLAR_EVERY = 2

PILLAR_EVERY = 2

# ---- what counts as a believable ToF reading (valid_tof) -------------------
# Read on EVERY frame of a v6 run: 0 and -1 mean "no measurement" on this
# firmware, and writing guards as `0 < d <= limit` silently disabled them.
TOF_MIN_MM = 1.0
TOF_MAX_MM = 2000.0


@dataclass
class Percept:
    """Everything the three models plus the ToF ring saw this frame."""
    # pillar model
    pillar_color: str | None = None      # "red" | "green"
    pillar_area: float = 0.0             # bbox px^2, the distance proxy
    pillar_cx: float | None = None       # bbox centre x, pixels
    frame_w: int = 1280
    pillar_steer_us: float = 0.0         # what the dual-cam pillar PID asks
    pillar_placed: bool = False          # the PID says this pillar has reached
    wall_class: str | None = None
    wall_conf: float = 0.0
    # corner model
    corner_class: str | None = None      # v6: the corner-type
                                         # classifier's answer, e.g.
                                         # 'close_turn_left'
    corner_class_conf: float = 0.0
    corner_detected: bool = False
    corner_turn: str | None = None       # "left" | "right"
    corner_closeness: float | None = None
    corner_both: bool = False            # BOTH line colours in one camera's frame
    corner_fresh: bool = False           # the model actually RAN this frame.
    corner_trigger: bool = False         # a non-camera trigger fired (v4 logic)
    # ToF
    left_mm: float | None = None
    right_mm: float | None = None
    front_mm: float | None = None
    # race context
    direction: str | None = None
    finishing: bool = False


@dataclass
class Decision:
    state: str
    steer_us: float                      # desired steering delta
    speed_pct: int
    owner: str                           # which model owns this maneuver
    reason: str = ""
    constraints: list = field(default_factory=list)
    nudge_us: float = 0.0                # barrier nudge, ADDED to the PID by
                                         # main rather than replacing it (see
                                         # BARRIER_NUDGE_*). 0.0 = not nudging.
    turn_time_s: float | None = None     # how long to hold the blind turn
    drawback_us: float = 0.0             # counter-steer applied after the turn
    drawback_time_s: float = 0.0         # ...held for this long (0 = skip)
    turn: str | None = None              # the turn this decision COMMANDS.
                                         # main logs THIS, never a raw percept
                                         # field -- they came from different
                                         # frames and disagreed on the track.
    count_corner: bool = False           # tells RaceManager a corner completed
    note_pillar: str | None = None       # tells RaceManager a pillar was passed


def pillar_steer_sign(color: str) -> float:
    """+1 steers RIGHT, -1 steers LEFT, for this pillar colour."""
    green_sign = -1.0 if GREEN_PASSES_LEFT else +1.0
    if color == "green":
        return green_sign
    if color == "red":
        return -green_sign
    return 0.0


def constrain_toward_walls(steer: float, left_mm, right_mm):
    """(steer, notes) -- never steer further toward a wall already close.

    Module level because the pillar BIAS is added in the main loop, after the
    owner's steering has been chosen, and it has to face the same rule. A bias
    free to push the car into a wall the ToF is already shouting about would be
    the phantom-pillar failure all over again.
    """
    notes = []
    # steer > 0 is RIGHT, so the right wall constrains positive steering.
    for side, dist, sign in (("right", right_mm, +1.0),
                             ("left", left_mm, -1.0)):
        if dist is None or dist <= 0:
            continue
        if steer * sign <= 0:                      # not steering toward it
            continue
        if dist < CONSTRAINT_HARD_MM:
            steer = 0.0
            notes.append(f"{side}wall {dist:.0f}mm: blocked")
        elif dist < CONSTRAINT_SIDE_MM:
            scale = ((dist - CONSTRAINT_HARD_MM)
                     / (CONSTRAINT_SIDE_MM - CONSTRAINT_HARD_MM))
            steer *= scale
            notes.append(f"{side}wall {dist:.0f}mm: x{scale:.2f}")
    return steer, notes


def pillar_bias(p: Percept) -> float:
    """How much the pillar ADDS to whatever is already being commanded.

    0.0 whenever the pillar is not steering: disabled, not in bias mode, no
    pillar, or one outside the area bounds the state machine trusts.
    """
    if not (PILLAR_PID_ENABLED and PILLAR_PID_BIAS_ONLY):
        return 0.0
    if p.pillar_color not in ("red", "green"):
        return 0.0
    if not (PILLAR_ACTIONABLE_AREA <= p.pillar_area <= PILLAR_MAX_AREA):
        return 0.0
    return max(-PILLAR_BIAS_LIMIT_US,
               min(PILLAR_BIAS_LIMIT_US, p.pillar_steer_us))


class BehaviorManager:
    """The state machine. Feed it a Percept, get a Decision."""

    def __init__(self, cruise_speed: int = 70, turn_steer_us: float = 250.0):
        self.state = CRUISE
        self.cruise_speed = cruise_speed
        self.turn_steer_us = turn_steer_us
        self.frames_in_state = 0
        self._pillar_color = None
        self._color_swap = 0
        self._pillar_missing = 0
        self._pillar_seen = 0
        self._wall_hold = 0
        self._corner_confirm = 0
        self._corner_armed = True
        self._rearm_frames = 0
        self._unarmed_frames = 0
        self._front_dead_frames = 0
        self._barrier_run = 0
        self._corner_class_last = None
        self._corner_class_run = 0
        self._last_wall_class = None
        self._nudge_left = 0
        self._nudge_cool = 0
        self._nudge_now = (0.0, "")

    # ---------------- state plumbing ----------------

    def _enter(self, state: str) -> None:
        if state != self.state:
            self.state = state
            self.frames_in_state = 0

    # ---------------- constraints ----------------

    def _apply_constraints(self, steer: float, p: Percept):
        """Clamp the owner's steering so it cannot drive into something."""
        steer, notes = constrain_toward_walls(steer, p.left_mm, p.right_mm)

        # The wall classifier is an independent opinion; if it says a wall is
        # NEAR on the side we are steering toward, damp further.
        if p.wall_class in WALL_ACT_CLASSES and p.wall_conf >= WALL_MIN_CONF:
            wall_side = "left" if p.wall_class.endswith("_left") else "right"
            toward = (steer > 0 and wall_side == "right") or \
                     (steer < 0 and wall_side == "left")
            if toward and p.wall_class.startswith("near_"):
                steer *= 0.4
                notes.append(f"wallAI {p.wall_class}: x0.40")

        steer = max(-self.turn_steer_us, min(self.turn_steer_us, steer))
        return steer, notes

    # ---------------- emergencies ----------------

    @staticmethod
    def _emergency(p: Percept) -> str | None:
        """Only PHYSICAL measurements raise an emergency.

        barrier_front used to be here, and that was the bug: a corner IS a wall
        dead ahead, so the wall classifier correctly shouting barrier_front sent
        the car to EMERGENCY -- which outranks CORNER -- and it oscillated
        instead of turning. barrier_front is now a CORNER TRIGGER (see
        _corner_actionable). The ToF front distance still raises a real
        emergency, because that is a measurement rather than a classification.
        """
        if p.front_mm is not None and 0 < p.front_mm <= EMERGENCY_FRONT_MM:
            return f"front {p.front_mm:.0f}mm"
        for side, dist in (("left", p.left_mm), ("right", p.right_mm)):
            if dist is not None and 0 < dist <= EMERGENCY_SIDE_MM:
                return f"{side} {dist:.0f}mm"
        return None

    # ---------------- the tick ----------------

    def update(self, p: Percept) -> Decision:
        self.frames_in_state += 1

        # Open road ahead re-arms the corner trigger, but only when it
        # HOLDS -- see CORNER_REARM_FRAMES.
        if p.front_mm is not None and p.front_mm >= CORNER_REARM_MM:
            self._rearm_frames += 1
            self._unarmed_frames = 0
            if self._rearm_frames >= CORNER_REARM_FRAMES:
                self._corner_armed = True
        else:
            self._rearm_frames = 0

        if p.front_mm is not None and 0 < p.front_mm <= TOF_MAX_MM:
            self._front_dead_frames = 0
        else:
            self._front_dead_frames += 1

        # How long the wall model has been saying "wall dead ahead" with
        # nothing acting on it. One frame proves nothing; an unbroken run
        # proves the corner never fired. See BARRIER_FRONT_RUN_FRAMES.
        if (p.wall_class == "barrier_front"
                and p.wall_conf >= BARRIER_FRONT_RUN_CONF):
            self._barrier_run += 1
        else:
            self._barrier_run = 0

        if self._corner_armed:
            self._unarmed_frames = 0
        else:
            self._unarmed_frames += 1
            if self._unarmed_frames >= CORNER_REARM_TIMEOUT_FRAMES:
                self._corner_armed = True
                self._unarmed_frames = 0

        if self.state == DONE:
            return Decision(DONE, 0.0, 0, "race", "finished")

        # EMERGENCY preempts every other owner -- but it must not become a trap.
        emg = self._emergency(p)
        if emg is not None:
            was = self.state
            self._enter(EMERGENCY)
            stuck = was == EMERGENCY and self.frames_in_state >= EMERGENCY_MAX_FRAMES
            turn, _why = self.corner_direction(p)
            front_blocked = ((p.front_mm is not None
                              and 0 < p.front_mm <= EMERGENCY_FRONT_MM)
                             or self.barrier_ahead(p))
            scraped = None
            for side, dist in (("right", p.right_mm), ("left", p.left_mm)):
                if dist is not None and 0 < dist <= EMERGENCY_SIDE_MM:
                    scraped = side
                    break
            turns_away = scraped is not None and turn is not None and scraped != turn
            if stuck and (front_blocked or turns_away) and turn in ("left", "right"):
                # Wedged at a corner with a known turn available: take it rather
                # than keep oscillating in place.
                self._enter(CORNER_TURN)
                self._corner_armed = False
                self._rearm_frames = 0
                # The TIGHTEST arc available, not the generic one -- a car
                # against the outer wall has less room than any ordinary
                # corner. See EMERGENCY_ESCAPE_CLOSE_TURN.
                name = f"close_turn_{turn}"
                if EMERGENCY_ESCAPE_CLOSE_TURN and name in CORNER_TURN_PROFILE:
                    esc_us, esc_s = CORNER_TURN_PROFILE[name]
                    esc_us = abs(esc_us) * (1 if turn == "right" else -1)
                    db_us, db_s = drawback_for(name)
                    how = f"; {name}"
                else:
                    esc_us = self.turn_steer_us * (1 if turn == "right" else -1)
                    esc_s = TURN_TIME_S
                    db_us, db_s = DRAWBACK_STEER_US, DRAWBACK_TIME_S
                    how = ""
                return Decision(CORNER_TURN, esc_us,
                                TURN_SPEED_PCT, "corner",
                                f"forced out of EMERGENCY ({emg}"
                                + (f"; pinned {scraped}, turning away"
                                   if turns_away and not front_blocked else "")
                                + how + ")",
                                turn=turn, turn_time_s=esc_s,
                                drawback_us=db_us, drawback_time_s=db_s,
                                count_corner=True)
            return self._emergency_decision(p, emg)
        if self.state == EMERGENCY:
            self._enter(CRUISE)          # cleared; fall through and re-decide

        # One tick per frame, before any behaviour can return. Which decisions
        # actually CARRY it is decided below -- a corner turn, a pillar pass and
        # the exit settle all ignore it.
        self._nudge_now = self._barrier_nudge(p)

        if p.finishing:
            self._enter(FINISHING)
            steer, notes = self._apply_constraints(0.0, p)
            return Decision(FINISHING, steer, self.cruise_speed, "race",
                            "creeping to the start zone", notes)

        if self.state in (PILLAR_APPROACH, PILLAR_PASS, PILLAR_RECOVER):
            return self._pillar_tick(p)
        if self.state in (CORNER_APPROACH, CORNER_TURN, CORNER_EXIT):
            return self._corner_tick(p)

        # --- nothing owns the car: priority order decides who takes it ---
        if not PILLAR_PID_BIAS_ONLY and self._pillar_actionable(p):
            self._pillar_color = p.pillar_color
            self._color_swap = 0
            self._pillar_missing = 0
            self._enter(PILLAR_APPROACH)
            return self._pillar_tick(p)

        if self._wall_actionable(p):
            return self._wall_tick(p)

        if self._corner_actionable(p):
            self._corner_confirm = 0
            self._enter(CORNER_APPROACH)
            return self._corner_tick(p)

        self._enter(CRUISE)
        steer, notes = self._apply_constraints(0.0, p)
        nudge, why = self._nudge_now
        return Decision(CRUISE, steer, self.cruise_speed, "pid",
                        "clear track" + (f"; {why}" if why else ""),
                        notes, nudge_us=nudge)

    # ---------------- behaviours ----------------

    def _emergency_decision(self, p: Percept, why: str) -> Decision:
        """Steer away from whatever is too close; crawl rather than stop dead.

        Stopping loses the run; easing away usually saves it. A barrier straight
        ahead is the exception the corner logic should already have caught, so
        here we turn toward the open side.
        """
        steer = 0.0
        if p.left_mm is not None and p.right_mm is not None:
            f = EMERGENCY_STEER_FRAC
            steer = self.turn_steer_us * (f if p.left_mm < p.right_mm else -f)
        elif p.direction is not None:
            f = EMERGENCY_STEER_FRAC
            steer = self.turn_steer_us * (-f if p.direction == "left" else f)
        return Decision(EMERGENCY, steer, EMERGENCY_SPEED_PCT,
                        "emergency", why)

    def _pillar_actionable(self, p: Percept) -> bool:
        """A pillar close enough to matter, seen for long enough to believe."""
        plausible = (p.pillar_color in ("red", "green")
                     and PILLAR_ACTIONABLE_AREA <= p.pillar_area <= PILLAR_MAX_AREA)
        self._pillar_seen = self._pillar_seen + 1 if plausible else 0
        return self._pillar_seen >= PILLAR_ACTIONABLE_FRAMES

    def _pillar_tick(self, p: Percept) -> Decision:
        visible = p.pillar_color is not None and p.pillar_area > 0
        self._pillar_missing = 0 if visible else self._pillar_missing + 1

        if visible and p.pillar_color != self._pillar_color:
            self._color_swap += 1
            if self._color_swap >= PILLAR_COLOR_SWITCH_FRAMES:
                self._pillar_color = p.pillar_color
                self._color_swap = 0
        elif visible:
            self._color_swap = 0
        color = self._pillar_color or p.pillar_color

        if self.state == PILLAR_RECOVER:
            if self.frames_in_state >= PILLAR_RECOVER_FRAMES:
                self._enter(CRUISE)
                steer, notes = self._apply_constraints(0.0, p)
                return Decision(CRUISE, steer, self.cruise_speed, "pid",
                                "pillar cleared", notes)
            steer, notes = self._apply_constraints(0.0, p)
            return Decision(PILLAR_RECOVER, steer, PILLAR_SPEED_PCT, "pillar",
                            "straightening after the pass", notes)

        # Lost sight of it for long enough -> it is behind us.
        if self._pillar_missing >= PILLAR_LOST_FRAMES:
            self._enter(PILLAR_RECOVER)
            steer, notes = self._apply_constraints(0.0, p)
            return Decision(PILLAR_RECOVER, steer, PILLAR_SPEED_PCT, "pillar",
                            f"{color} pillar passed", notes,
                            note_pillar=color)

        if p.pillar_area >= PILLAR_PASS_AREA:
            self._enter(PILLAR_PASS)

        if PILLAR_PID_ENABLED:
            steer = p.pillar_steer_us
            why = (f"{color} PID {steer:+.0f}us area={p.pillar_area:.0f}"
                   + ("  PLACED" if p.pillar_placed else ""))
        else:
            # Legacy: steer to put the pillar on the correct side, scaling the
            # base steer by how far off-centre it already is. A pillar already
            # on the side we want needs less; on the wrong side, more.
            sign = pillar_steer_sign(color)
            steer = sign * PILLAR_STEER_US
            if p.pillar_cx is not None and p.frame_w:
                offset = (p.pillar_cx - p.frame_w / 2.0) / (p.frame_w / 2.0)
                steer *= max(0.3, min(1.6, 1.0 - offset * sign))
            why = f"{color} area={p.pillar_area:.0f}"

        steer, notes = self._apply_constraints(steer, p)
        return Decision(self.state, steer, PILLAR_SPEED_PCT, "pillar",
                        why, notes)

    @staticmethod
    def wall_ai_vetoed(p: Percept) -> str | None:
        """Does the ToF contradict the wall classifier? Returns why, or None.

        The classifier is a single confident guess from one image; the ToF is a
        direct measurement. When they disagree about whether a wall is near, the
        measurement wins.
        """
        if p.wall_class not in WALL_ACT_CLASSES:
            return None
        side = "left" if p.wall_class.endswith("_left") else "right"
        dist = p.left_mm if side == "left" else p.right_mm
        if dist is not None and dist > WALL_AI_MAX_MM:
            return f"ToF {side}={dist:.0f}mm > {WALL_AI_MAX_MM:.0f}"
        return None

    def _barrier_nudge(self, p: Percept):
        """(steer_us, why) -- a brief lean toward the locked turn direction.

        CALL EXACTLY ONCE PER update(): it owns the fire/hold/cooldown counters.
        Returns (0.0, "") whenever it is not firing.
        """
        if not BARRIER_NUDGE_ENABLED or p.direction not in ("left", "right"):
            self._nudge_left = 0
            return 0.0, ""

        sign = -1.0 if p.direction == "left" else 1.0

        # Already firing: run it out. The model does not get to extend it --
        # "brief" has to mean brief even while barrier_front keeps reporting.
        if self._nudge_left > 0:
            self._nudge_left -= 1
            if self._nudge_left == 0:
                self._nudge_cool = BARRIER_NUDGE_COOLDOWN
            return sign * BARRIER_NUDGE_US, f"barrier nudge {p.direction}"

        if self._nudge_cool > 0:
            self._nudge_cool -= 1
            return 0.0, ""

        if (p.wall_class != "barrier_front"
                or p.wall_conf < BARRIER_NUDGE_MIN_CONF):
            return 0.0, ""
        # A wall the front says is 1.8m away is not a barrier.
        if (p.front_mm is not None
                and p.front_mm > BARRIER_NUDGE_MAX_FRONT_MM):
            return 0.0, ""
        # Never lean into a wall that is already close on that side.
        inner = p.left_mm if p.direction == "left" else p.right_mm
        if inner is not None and 0 < inner < BARRIER_NUDGE_MIN_SIDE_MM:
            return 0.0, ""

        self._nudge_left = BARRIER_NUDGE_FRAMES - 1
        return sign * BARRIER_NUDGE_US, f"barrier nudge {p.direction}"

    def _wall_actionable(self, p: Percept) -> bool:
        if p.wall_class not in WALL_ACT_CLASSES or p.wall_conf < WALL_MIN_CONF:
            return False
        return self.wall_ai_vetoed(p) is None

    def _wall_tick(self, p: Percept) -> Decision:
        """Wall AI chooses the KIND of correction; constraints scale it."""
        self._enter(CRUISE)              # wall correction rides on top of cruise
        self._wall_hold = WALL_HOLD_FRAMES
        self._last_wall_class = p.wall_class
        steer = WALL_STEER_US.get(p.wall_class, 0.0)
        steer, notes = self._apply_constraints(steer, p)
        nudge, why = self._nudge_now
        return Decision(CRUISE, steer, self.cruise_speed, "wall",
                        f"{p.wall_class} {p.wall_conf:.2f}"
                        + (f"; {why}" if why else ""),
                        notes, nudge_us=nudge)

    @staticmethod
    def barrier_ahead(p: Percept) -> bool:
        """The wall classifier seeing a wall dead ahead == arriving at a corner.

        This used to route to EMERGENCY, which outranks CORNER, so the car
        oscillated at the corner instead of turning it. The ToF front matrix
        frequently reads INVALID here (F=None in the logs) because the wall is
        outside its row-1 window, so this classification is often the ONLY
        warning that the corner has arrived.
        """
        if not BARRIER_FRONT_DECIDES:
            return False
        if p.wall_class != "barrier_front" or p.wall_conf < WALL_MIN_CONF:
            return False
        # ...unless the ToF disagrees. It called barrier_front at front=1014 mm
        # on the track and fired a corner a metre early. Same rule as the side
        # walls: a measurement beats a classification.
        if p.front_mm is not None and p.front_mm > CORNER_FRONT_MM:
            return False
        return True

    def corner_class_turn(self, p: Percept):
        """(class_name, steer_us, hold_s) from the corner-type classifier, or None.

        v6's route into a corner. The classifier names the turn outright, so
        there is no direction vote, no line-colour heuristic and no shared arc --
        each type carries its own steering and duration from CORNER_TURN_PROFILE.

        It still has to say the SAME thing CORNER_CLASS_FRAMES times running: one
        confident frame is how the old path locked the wrong direction, and this
        model will have the same failure mode on a borderline corner.
        """
        name = self.locked_class(p.corner_class, p.direction)
        if name not in CORNER_TURN_PROFILE or p.corner_class_conf < CORNER_CLASS_MIN_CONF:
            self._corner_class_run = 0
            self._corner_class_last = None
            return None
        if name == self._corner_class_last:
            self._corner_class_run += 1
        else:
            self._corner_class_last = name
            self._corner_class_run = 1
        if self._corner_class_run < CORNER_CLASS_FRAMES:
            return None
        steer, hold = CORNER_TURN_PROFILE[name]
        return name, steer, hold

    @staticmethod
    def locked_class(name, direction):
        """Force a corner class onto the locked side. None = unusable.

        Before the lock (direction is None) the classifier is the ONLY source of
        the turn side, so it passes through untouched -- that first corner is
        what establishes the lock in the first place.
        """
        if name is None or direction not in ("left", "right"):
            return name
        if name.endswith("_" + direction):
            return name
        if not CORNER_LOCK_MIRRORS_CLASS:
            return None
        wrong = "left" if direction == "right" else "right"
        return name[:-len(wrong)] + direction

    def _corner_actionable(self, p: Percept) -> bool:
        if p.corner_trigger or self.barrier_ahead(p) or self.barrier_sustained():
            return True
        if self.corner_class_turn(p) is not None:
            return True
        return (p.corner_detected and p.corner_closeness is not None
                and p.corner_closeness >= CORNER_APPROACH_CLOSENESS)

    def barrier_sustained(self) -> bool:
        """The wall model has said 'wall ahead' long enough to be believed.

        ONLY while the front matrix is dead. With a live front the ordinary
        arrival test already works, and running this alongside it would
        reintroduce the exact false commit BARRIER_FRONT_DECIDES exists to
        prevent.
        """
        return (BARRIER_FRONT_RUN_DECIDES
                and self._front_dead_frames >= CORNER_FRONT_DEAD_FRAMES
                and self._barrier_run >= BARRIER_FRONT_RUN_FRAMES)

    def corner_direction(self, p: Percept):
        """(turn, why). None means NOT DECIDED -- keep approaching, do not guess.

        ONE AUTHORITY: the RaceManager owns the track direction, using v4's
        proven camera-vote rule (both-lines frames, or a lone CLOSE line, count;
        a lone FAR line does not). This class used to run a SECOND, stricter vote
        of its own -- and on the track the two disagreed: the log printed
        "waiting for direction: only 1L/0R (need 6)" on the very same frame that
        already read dir=left, because the RaceManager had locked after one
        trustworthy frame. Two systems with different thresholds means the loose
        one silently wins, so there is now only one.

        Once locked it never changes: a WRO loop is a square, so every corner
        turns the same way, and a confirmed corner beats any single camera frame.
        """
        if p.direction in ("left", "right"):
            return p.direction, "locked"
        return None, "direction not locked yet"

    def _corner_tick(self, p: Percept) -> Decision:
        # v6 route: the corner classifier names the turn AND its profile. It
        # still needs the front ToF to say the car has arrived (see below) and
        # the re-arm to say this is not the corner just left.
        classed = self.corner_class_turn(p)
        if classed is not None and self._corner_armed and self.state != CORNER_EXIT:
            name, steer_us, hold_s = classed
            front_ok = (p.front_mm is not None
                        and 0 < p.front_mm <= CORNER_FRONT_MM)
            # See CORNER_FRONT_DEAD_FRAMES: with no front there is no "when",
            # and the classifier is the only thing left that knows a corner is
            # here at all.
            front_dead = self._front_dead_frames >= CORNER_FRONT_DEAD_FRAMES
            if front_ok or p.corner_trigger or front_dead:
                turn = "right" if steer_us > 0 else "left"
                # Per-type counter-steer: how much yaw is left to cancel
                # depends on the arc that was just driven.
                db_us, db_s = drawback_for(name)
                self._enter(CORNER_TURN)
                self._corner_armed = False
                self._rearm_frames = 0
                return Decision(CORNER_TURN, steer_us, TURN_SPEED_PCT, "corner",
                                f"{name} (conf={p.corner_class_conf:.2f}; "
                                f"front={p.front_mm}"
                                + ("; FRONT DEAD" if front_dead and not front_ok
                                   else "") + ")",
                                turn=turn, turn_time_s=hold_s,
                                drawback_us=db_us, drawback_time_s=db_s,
                                count_corner=True)

        turn, why = self.corner_direction(p)

        if self.state == CORNER_EXIT:
            if self.frames_in_state >= CORNER_EXIT_FRAMES:
                self._enter(CRUISE)
                steer, notes = self._apply_constraints(0.0, p)
                return Decision(CRUISE, steer, self.cruise_speed, "pid",
                                "corner done", notes)
            # Hand the steering to the wall follower rather than coasting
            # blind: the timed turn is deliberately short now, and the PID is
            # what actually settles the car onto the new straight.
            steer, notes = self._apply_constraints(0.0, p)
            return Decision(CORNER_EXIT, steer, self.cruise_speed, "pid",
                            "exiting (PID settling)", notes)

        if self.state == CORNER_TURN:
            # The timed turn itself is executed by the main loop (it owns the
            # clock and the servo); this just reports who is in charge.
            return Decision(CORNER_TURN,
                            self.turn_steer_us * (1 if turn == "right" else -1),
                            TURN_SPEED_PCT, "corner", f"turning {turn}",
                            turn=turn)

        # CORNER_APPROACH: commit once close enough, or when a sensor trigger
        # fires (v4's front matrix / wall collapse), but never without knowing
        # which way to turn -- WRO requires the direction to be auto-detected.
        close = p.corner_closeness or 0.0

        # v4-style camera confirmation: consecutive frames, not one lucky one.
        if p.corner_fresh and p.corner_detected and close >= CORNER_FIRE_CLOSENESS:
            self._corner_confirm += 1
        elif p.corner_fresh:
            self._corner_confirm = 0
        camera_ready = self._corner_confirm >= CORNER_CONFIRM_FRAMES

        # The front ToF is what says "you have ARRIVED". Closeness only says a
        # corner is ahead somewhere.
        front_close = (p.front_mm is not None and 0 < p.front_mm <= CORNER_FRONT_MM)

        committed = self._corner_armed and (
                     p.corner_trigger                       # ToF triggers (v4)
                     or self.barrier_ahead(p)               # wall AI: wall ahead
                     or self.barrier_sustained()            # ...for long enough
                                                            # with a dead front:
                                                            # the last thing left
                     or (camera_ready and front_close)      # camera AND front
                     or (p.front_mm is None                 # front blind,
                         and close >= CORNER_CAMERA_ONLY_CLOSE))  # trust cam
        if committed and turn in ("left", "right"):
            self._enter(CORNER_TURN)
            self._corner_armed = False
            return Decision(CORNER_TURN,
                            self.turn_steer_us * (1 if turn == "right" else -1),
                            TURN_SPEED_PCT, "corner",
                            f"turn {turn} ({why}; front={p.front_mm} close={close:.2f})",
                            turn=turn, turn_time_s=TURN_TIME_S,
                            drawback_us=DRAWBACK_STEER_US,
                            drawback_time_s=DRAWBACK_TIME_S,
                            count_corner=True)
        if committed:
            steer, notes = self._apply_constraints(0.0, p)
            return Decision(CORNER_APPROACH, steer, CORNER_APPROACH_SPEED_PCT,
                            "pid", f"waiting for direction: {why}", notes)

        if not committed and not p.corner_detected and self.frames_in_state > 30:
            self._enter(CRUISE)          # corner drifted out of view; give up
            steer, notes = self._apply_constraints(0.0, p)
            return Decision(CRUISE, steer, self.cruise_speed, "pid",
                            "corner lost", notes)


        steer, notes = self._apply_constraints(0.0, p)
        nudge, why = self._nudge_now
        return Decision(CORNER_APPROACH, steer, CORNER_APPROACH_SPEED_PCT, "pid",
                        f"approach close={close:.2f} turn={turn}"
                        + (f"; {why}" if why else ""),
                        notes, nudge_us=nudge)

    def corner_turn_finished(self) -> None:
        """Main loop calls this when the timed turn has completed."""
        self._enter(CORNER_EXIT)

    def corner_turn_cancelled(self) -> None:
        """The main loop REFUSED a commit this machine had already made.

        _corner_tick spends the arm the instant it returns CORNER_TURN, but the
        RaceManager only gets a say afterwards, in the main loop -- so a commit
        debounced by CORNER_LOCKOUT_S left the machine disarmed for a maneuver
        that was never driven.

        That is how the clockwise run lost a corner. Corner 4 committed, turned
        for 2.8s, and the next corner arrived while the 5s lockout -- which is
        measured from the START of a turn -- still had ~0.5s left. The commit
        was debounced, the arm was already gone, and neither route back could
        fire: re-arming by distance needs the front ABOVE CORNER_REARM_MM and
        the car was driving INTO the corner (934 -> 882 -> 714 -> 490 -> dead),
        while CORNER_REARM_TIMEOUT_FRAMES arrived only after the front had gone
        blind. The classifier held turn_right at 1.00 the whole way and nothing
        happened. The counter stuck at 4/12.

        A maneuver that never ran must not consume the arm. Restoring it lets
        the next tick try again -- and be debounced again, harmlessly, once per
        CORNER_EXIT_FRAMES -- until the lockout expires and the turn actually
        happens. The front-distance re-arm is not bypassed, because arming was
        never lost in the first place.
        """
        self._corner_armed = True
        self._rearm_frames = 0
        self._unarmed_frames = 0
        self._enter(CORNER_EXIT)

    def set_done(self) -> None:
        self._enter(DONE)
