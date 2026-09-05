#!/usr/bin/env python3
"""
pillar_pid_dualcam.py  --  test the dual-camera pillar PID from the schematics.

    python3 pillar_pid_dualcam.py --selftest      # geometry + signs, no model
    python3 pillar_pid_dualcam.py --pairs images  # offline, saved camera pairs
    python3 pillar_pid_dualcam.py                 # live on the robot
    python3 pillar_pid_dualcam.py --headless      # over SSH

THE IDEA (Green_pid_dualcam_squematic.png / Red_pid_dualcam_squematic.png)

    Lay both cameras side by side as ONE panoramic strip and cut it into
    sections. A pillar has a section where it BELONGS once the car has passed it
    the way the rules require. The PID drives it there.

        strip = [ LEFT camera | RIGHT camera ]   ==   [ cam1 | cam0 ]
                |  0  |  1  |  2  |  3  |  4  |  5  |

    WRO: the car passes RED on its right and GREEN on its left. So after a
    correct pass the pillar has ended up on the opposite side of the car:

        GREEN -> car's RIGHT -> target section 5   (rightmost of the RIGHT cam)
        RED   -> car's LEFT  -> target section 0   (leftmost  of the LEFT cam)

    which is exactly what the schematics draw: the Target arrow points right for
    green, left for red, and the Error Value is the gap the PID closes.

    "...and not visible in the other camera" is the END STATE, not a separate
    rule -- a pillar sitting in section 5 is by construction out of the left
    camera. It is reported as `placed` so you can see it happen.

ONE FORMULA, BOTH COLOURS

        error   = pillar_x - target_x          (in section units)
        steer   = PID(error)                   (negative = LEFT, as everywhere)

    The colour never enters the maths; it only picks the target. That falls out
    correctly on its own:

        green: pillar is LEFT of its target  -> error < 0 -> steer LEFT
        red:   pillar is RIGHT of its target -> error > 0 -> steer RIGHT

    which is "pass green on the left, red on the right". A separate sign per
    colour would be a second place to get the rule backwards.

PRIORITY
    When several pillars are visible the LARGEST BOX wins. Area is the distance
    proxy this robot already uses, so the largest is the nearest, and the
    nearest is the one about to be hit. Ties and phantoms are filtered by
    PILLAR_ACTIONABLE_AREA / PILLAR_MAX_AREA, the bounds the state machine
    already trusts.

USED BY main_challenge_v6
    v6 imports Strip, Pillar, PillarPID, detect, choose and placed from HERE --
    it does not carry a copy. Two copies of a sign convention is how one of them
    ends up backwards. Run --selftest after touching anything in this file.

    Whether v6 uses it at all is behavior_manager.PILLAR_PID_ENABLED; with that
    off, v6 never even loads the detector.

RUN STANDALONE TO JUDGE IT
    --selftest proves the geometry and every steering direction with no model
    and no cameras. --pairs replays saved camera pairs and renders the strip,
    the sections, the chosen pillar, the target and the steering it would
    command. Neither touches the motor.
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))       # the project root

import behavior_manager as B                                    # noqa: E402
from pid_line_follower import LineFollowerPID                   # noqa: E402
from color_tuning_Dualcam import CAM_WIDTH, camera_pipeline     # noqa: E402

DEFAULT_MODEL = os.path.join(HERE, "Models", "Pillar_model_v1.pt")

# Physical mounting, as everywhere else in this project.
CAM_RIGHT, CAM_LEFT = 0, 1
# The strip reads left-to-right like the world: LEFT camera first.
STRIP_ORDER = (CAM_LEFT, CAM_RIGHT)

TEST_MAX_STEER_US = 340.0        # 89% of the 380us the servo has to the left

TEST_FULL_SCALE_SECTIONS = 4.0   # error that commands full steering
TEST_SLEW_US = 250.0             # us per frame; a bad box still cannot slam it

PREVIEW_FILE = os.path.join(HERE, "pillar_pid_preview.jpg")
WIN = "pillar PID (dual cam)"

COLOURS = {"green": (0, 220, 0), "red": (0, 0, 255)}


def colour_of(name: str) -> str:
    return "green" if "green" in name.lower() else "red"


# =========================================================
# Strip geometry
# =========================================================

@dataclass
class Strip:
    """Where each camera and each section sits along the panoramic strip."""
    frame_w: int
    per_cam: int                      # sections per camera

    @property
    def width(self):
        return self.frame_w * len(STRIP_ORDER)

    @property
    def n_sections(self):
        return self.per_cam * len(STRIP_ORDER)

    @property
    def section_w(self):
        return self.width / self.n_sections

    def x_of(self, sensor_id, cx):
        """A camera's own pixel x -> x along the strip."""
        return STRIP_ORDER.index(sensor_id) * self.frame_w + cx

    def sections(self, strip_x):
        """Strip x in SECTION units: 0.0 at the far left, n_sections at the right."""
        return strip_x / self.section_w

    def section_index(self, strip_x):
        return min(self.n_sections - 1, int(strip_x // self.section_w))

    def target_sections(self, colour):
        """Centre of this colour's target section, in section units."""
        return B.PILLAR_PID_TARGET_SECTION[colour] + 0.5

    def camera_of(self, strip_x):
        return STRIP_ORDER[min(len(STRIP_ORDER) - 1,
                               int(strip_x // self.frame_w))]


@dataclass
class Pillar:
    colour: str
    conf: float
    area: float
    sensor_id: int
    box: tuple                        # x1, y1, x2, y2 in that camera's frame
    strip_x: float                    # centre along the strip


def usable(p: Pillar) -> bool:
    """The same gates the state machine applies before a pillar owns the car."""
    return (p.conf >= B.PILLAR_PID_MIN_CONF
            and B.PILLAR_ACTIONABLE_AREA <= p.area <= B.PILLAR_MAX_AREA)


def choose(pillars):
    """Largest box wins: area is the distance proxy, so largest == nearest."""
    ok = [p for p in pillars if usable(p)]
    return max(ok, key=lambda p: p.area) if ok else None


def placed(strip: Strip, p: Pillar, others) -> bool:
    """Is this pillar in its target section AND out of the other camera?

    The second half is the schematics' "not visible in the left camera". It is
    a consequence of reaching the target, not an extra condition, but reporting
    it separately shows whether the geometry really works out.
    """
    in_target = strip.section_index(p.strip_x) == \
        B.PILLAR_PID_TARGET_SECTION[p.colour]
    other_cam = CAM_LEFT if p.colour == "green" else CAM_RIGHT
    clear = not any(o.colour == p.colour and o.sensor_id == other_cam
                    and usable(o) for o in others)
    return in_target and clear


# =========================================================
# The controller
# =========================================================

class PillarPID:
    """PID on 'how many sections is the pillar from where it should be'.

    Wraps LineFollowerPID rather than growing a second PID implementation, so
    the median filter, deadband, integral clamp and slew limit are the ones
    already proven on the wall follower.
    """

    def __init__(self, kp=None, out_limit=None, slew=None):
        """Gains default to behavior_manager's; the rig overrides them.

        Overridable so pillar_pid_dualcam can try a different steering law
        WITHOUT moving the numbers main_challenge_v6 runs on.
        """
        self.pid = LineFollowerPID(
            kp=B.PILLAR_PID_KP if kp is None else kp,
            ki=B.PILLAR_PID_KI, kd=B.PILLAR_PID_KD,
            out_limit=(B.PILLAR_PID_OUT_LIMIT if out_limit is None
                       else out_limit),
            deadband=B.PILLAR_PID_DEADBAND,
            median_window=B.PILLAR_PID_MEDIAN,
            slew_limit=B.PILLAR_PID_SLEW if slew is None else slew,
            max_error=float(B.PILLAR_PID_SECTIONS * len(STRIP_ORDER)))
        self._last_colour = None

    def reset(self):
        self.pid.reset()

    def update(self, strip: Strip, p: Pillar):
        """(steer_us, info). Positive steers RIGHT, negative LEFT."""
        if p is None:
            self.pid.reset()
            self._last_colour = None
            return 0.0, {"valid": False, "reason": "no pillar"}

        # A different colour is a different setpoint; carrying the integrator
        # and the median across that would blend two unrelated targets.
        if p.colour != self._last_colour:
            self.pid.reset()
            self._last_colour = p.colour

        pos = strip.sections(p.strip_x)
        tgt = strip.target_sections(p.colour)

        if B.PILLAR_PID_ONE_SIDED:
            past = (pos > tgt) if p.colour == "green" else (pos < tgt)
            if past:
                self.pid.reset()
                return 0.0, {"valid": True, "pos": pos, "target": tgt,
                             "error": pos - tgt, "colour": p.colour,
                             "past": True}
        # +1.0 on BOTH sides: LineFollowerPID._valid rejects 0, and a pillar at
        # the very left edge is legitimately at section 0.0. The error is a
        # difference, so the shift cancels exactly.
        u, info = self.pid.update_single(pos + 1.0, tgt + 1.0, side_sign=+1)
        info.update({"pos": pos, "target": tgt, "error": pos - tgt,
                     "colour": p.colour})
        return u, info


# =========================================================
# Detection
# =========================================================

def detect(model, frames, strip: Strip, conf):
    """Every pillar in either camera, placed on the strip."""
    out = []
    for sid, frame in frames.items():
        if frame is None:
            continue
        r = model(frame, imgsz=640, verbose=False, conf=conf)[0]
        for box in (r.boxes if r.boxes is not None else []):
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = 0.5 * (x1 + x2)
            out.append(Pillar(
                colour=colour_of(model.names[int(box.cls[0].item())]),
                conf=float(box.conf[0].item()),
                area=(x2 - x1) * (y2 - y1),
                sensor_id=sid, box=(x1, y1, x2, y2),
                strip_x=strip.x_of(sid, cx)))
    return out


# =========================================================
# Rendering
# =========================================================

def render(frames, strip: Strip, pillars, chosen, steer, info, view_h=300):
    """The strip, its sections, every pillar, the target and the command."""
    panes = []
    for sid in STRIP_ORDER:
        f = frames.get(sid)
        if f is None:
            f = np.zeros((strip.frame_w * 9 // 16, strip.frame_w, 3), np.uint8)
            cv2.putText(f, f"cam{sid} no frame", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        panes.append(f)
    sheet = cv2.hconcat([cv2.resize(p, (strip.frame_w, panes[0].shape[0]))
                         for p in panes])
    scale = view_h / sheet.shape[0]
    sheet = cv2.resize(sheet, (int(sheet.shape[1] * scale), view_h))
    sx = sheet.shape[1] / strip.width          # strip px -> view px

    h = sheet.shape[0]
    # section dividers; the camera seam is drawn heavier
    for i in range(1, strip.n_sections):
        x = int(i * strip.section_w * sx)
        heavy = (i % strip.per_cam == 0)
        cv2.line(sheet, (x, 0), (x, h), (255, 255, 255), 3 if heavy else 1)
    # Tint each colour's target section, so where the PID is driving the
    # pillar is visible rather than inferred from the arrow alone.
    for colour, sec in B.PILLAR_PID_TARGET_SECTION.items():
        x0 = int(sec * strip.section_w * sx)
        x1 = int((sec + 1) * strip.section_w * sx)
        band = sheet[:, max(0, x0):min(sheet.shape[1], x1)]
        if band.size:
            tint = np.full_like(band, COLOURS[colour])
            cv2.addWeighted(band, 0.85, tint, 0.15, 0, dst=band)

    # Section numbers: strip index, plus which of the camera's three it is.
    for i in range(strip.n_sections):
        cx = int((i + 0.5) * strip.section_w * sx)
        cv2.putText(sheet, f"{i}", (cx - 6, h - 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 2)
        cv2.putText(sheet, f"({i % strip.per_cam + 1}/{strip.per_cam})",
                    (cx - 22, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (150, 150, 150), 1)

    # Which camera each half is, over the half it belongs to.
    for slot, sid in enumerate(STRIP_ORDER):
        name = "LEFT" if sid == CAM_LEFT else "RIGHT"
        cv2.putText(sheet, f"cam{sid} ({name}) -- {strip.per_cam} sections",
                    (int(slot * strip.frame_w * sx) + 12, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    for p in pillars:
        x1, y1, x2, y2 = p.box
        off = STRIP_ORDER.index(p.sensor_id) * strip.frame_w
        col = COLOURS[p.colour]
        dim = tuple(int(c * 0.45) for c in col)
        is_chosen = chosen is not None and p is chosen
        cv2.rectangle(sheet,
                      (int((off + x1) * sx), int(y1 * scale)),
                      (int((off + x2) * sx), int(y2 * scale)),
                      col if is_chosen else dim, 3 if is_chosen else 1)
        cv2.putText(sheet, f"{p.colour} {p.conf:.2f} a={p.area:.0f}",
                    (int((off + x1) * sx), max(12, int(y1 * scale) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    col if is_chosen else dim, 1 if not is_chosen else 2)

    if chosen is not None:
        col = COLOURS[chosen.colour]
        px = int(chosen.strip_x * sx)
        tx = int(strip.target_sections(chosen.colour) * strip.section_w * sx)
        cv2.arrowedLine(sheet, (px, h // 2), (tx, h // 2), (0, 165, 255), 2,
                        tipLength=0.03)
        cv2.line(sheet, (tx, 0), (tx, h), col, 2)
        cv2.putText(sheet, "TARGET", (max(2, tx - 40), 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

    bar = np.zeros((88, sheet.shape[1], 3), np.uint8)
    if chosen is None:
        lines = ["no actionable pillar", "steer = 0.0 (PID reset)"]
    else:
        lines = [f"{chosen.colour.upper()}  area={chosen.area:.0f}  "
                 f"conf={chosen.conf:.2f}  cam{chosen.sensor_id}  "
                 f"section {strip.section_index(chosen.strip_x)} "
                 f"-> {B.PILLAR_PID_TARGET_SECTION[chosen.colour]}",
                 f"error={info.get('error', 0):+.2f} sections   "
                 f"steer={steer:+7.1f}us  "
                 f"({'LEFT' if steer < 0 else 'RIGHT' if steer > 0 else 'centre'})"
                 + ("   PLACED" if info.get("placed") else "")]
    for i, t in enumerate(lines):
        cv2.putText(bar, t, (10, 32 + 34 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 2)
    return cv2.vconcat([sheet, bar])


# =========================================================
# Self test -- geometry and signs, no model, no cameras
# =========================================================

def _colour_switch_resets(strip) -> bool:
    """A new colour is a new setpoint; the integrator must not carry over."""
    ctl = PillarPID()
    g = Pillar("green", 0.9, 5000.0, CAM_LEFT, (0, 0, 1, 1),
               strip.x_of(CAM_LEFT, 0.02 * strip.frame_w))
    for _ in range(5):
        ctl.update(strip, g)
    r = Pillar("red", 0.9, 5000.0, CAM_LEFT, (0, 0, 1, 1),
               strip.x_of(CAM_LEFT, 0.02 * strip.frame_w))
    first, _ = ctl.update(strip, r)
    return abs(first) <= B.PILLAR_PID_SLEW + 1e-6


def selftest() -> int:
    W = 1280
    strip = Strip(frame_w=W, per_cam=B.PILLAR_PID_SECTIONS)
    print(f"strip: {len(STRIP_ORDER)} cameras x {W}px = {strip.width}px, "
          f"{strip.n_sections} sections of {strip.section_w:.0f}px")
    print(f"order: {[f'cam{c}' for c in STRIP_ORDER]}  "
          f"(cam{CAM_LEFT}=LEFT first -- the strip reads like the world)\n")

    def at(sid, frac):
        return strip.x_of(sid, frac * W)

    checks = []
    # section indices
    checks.append(("cam1 (LEFT) far left  -> section 0",
                   strip.section_index(at(CAM_LEFT, 0.05)) == 0))
    checks.append(("cam1 (LEFT) far right -> section 2",
                   strip.section_index(at(CAM_LEFT, 0.95)) == 2))
    checks.append(("cam0 (RIGHT) far left -> section 3",
                   strip.section_index(at(CAM_RIGHT, 0.05)) == 3))
    checks.append(("cam0 (RIGHT) far right-> section 5",
                   strip.section_index(at(CAM_RIGHT, 0.95)) == 5))
    checks.append(("red target is in the LEFT camera",
                   B.PILLAR_PID_TARGET_SECTION["red"] < B.PILLAR_PID_SECTIONS))
    checks.append(("green target is in the RIGHT camera",
                   B.PILLAR_PID_TARGET_SECTION["green"] >= B.PILLAR_PID_SECTIONS))
    checks.append(("both targets are inside the camera the pillar ends up in",
                   strip.camera_of(strip.target_sections("green")
                                   * strip.section_w) == CAM_RIGHT
                   and strip.camera_of(strip.target_sections("red")
                                       * strip.section_w) == CAM_LEFT))

    # The fraction of a frame that lands exactly on a target section's CENTRE.
    def centre_frac(colour):
        tgt_x = strip.target_sections(colour) * strip.section_w
        cam = CAM_RIGHT if colour == "green" else CAM_LEFT
        return (tgt_x - STRIP_ORDER.index(cam) * W) / W

    print(f"{'case':<58}{'error':>9}{'steer':>9}   {'dir':<7} expected")
    print("-" * 94)
    sign_ok = True
    CASES = (
        # the schematics themselves: pillar straddling the camera seam
        ("green", CAM_RIGHT, 0.02, "LEFT",   "schematic: drive it right"),
        ("red",   CAM_LEFT,  0.98, "RIGHT",  "schematic: drive it left"),
        # sitting exactly on target
        ("green", CAM_RIGHT, centre_frac("green"), "centre", "on target"),
        ("red",   CAM_LEFT,  centre_frac("red"),   "centre", "on target"),
        # PAST the target in the exit direction -> silence, not a pull-back.
        # See behavior_manager.PILLAR_PID_ONE_SIDED.
        ("green", CAM_RIGHT, 0.98, "centre", "past target, lets it go"),
        ("red",   CAM_LEFT,  0.02, "centre", "past target, lets it go"),
        # worst case: at the wrong end of the strip entirely
        ("green", CAM_LEFT,  0.02, "LEFT",   "far side"),
        ("red",   CAM_RIGHT, 0.98, "RIGHT",  "far side"),
    )
    for colour, sid, frac, want, why in CASES:
        ctl = PillarPID()
        p = Pillar(colour, 0.9, 5000.0, sid, (0, 0, 10, 10), at(sid, frac))
        steer, info = ctl.update(strip, p)
        got = "LEFT" if steer < -1 else "RIGHT" if steer > 1 else "centre"
        sign_ok &= got == want
        print(f"  {colour:<5} cam{sid} at {frac:>5.1%}  {why:<28}"
              f"{info['error']:>+9.2f}{steer:>9.1f}   {got:<7}"
              f"{'ok' if got == want else 'EXPECTED ' + want}")

    checks.append(("every steering direction matches the WRO rule", sign_ok))

    # The slew limit means one frame cannot slam the servo; it ramps.
    ctl = PillarPID()
    far = Pillar("green", 0.9, 5000.0, CAM_LEFT, (0, 0, 1, 1), at(CAM_LEFT, 0.02))
    ramp = [ctl.update(strip, far)[0] for _ in range(6)]
    print("")
    print(f"  worst-case ramp over 6 frames "
          f"(slew {B.PILLAR_PID_SLEW:.0f}us/frame): "
          + " ".join(f"{v:+.0f}" for v in ramp))
    checks.append(("ramps to the output limit, never jumps there",
                   abs(ramp[0]) <= B.PILLAR_PID_SLEW + 1e-6
                   and abs(ramp[-1]) == B.PILLAR_PID_OUT_LIMIT))
    checks.append(("switching colour resets the controller",
                   _colour_switch_resets(strip)))

    ok_area = (B.PILLAR_ACTIONABLE_AREA + B.PILLAR_MAX_AREA) / 2.0
    big = Pillar("red", 0.9, ok_area * 1.5, CAM_LEFT, (0, 0, 1, 1), 100.0)
    small = Pillar("green", 0.99, ok_area, CAM_RIGHT, (0, 0, 1, 1), 2400.0)
    phantom = Pillar("green", 0.99, B.PILLAR_MAX_AREA * 2, CAM_RIGHT,
                     (0, 0, 1, 1), 2400.0)
    tiny = Pillar("red", 0.99, B.PILLAR_ACTIONABLE_AREA * 0.2, CAM_LEFT,
                  (0, 0, 1, 1), 100.0)
    checks.append(("largest area wins", choose([small, big]) is big))
    checks.append(("phantom rejected by PILLAR_MAX_AREA",
                   choose([small, phantom]) is small))
    checks.append(("too-far pillar ignored", choose([tiny]) is None))
    checks.append(("low confidence ignored",
                   choose([Pillar("red", 0.10, ok_area, CAM_LEFT,
                                  (0, 0, 1, 1), 100.0)]) is None))
    # placed
    # Put it in whatever section green is currently aimed at, measured from
    # the RIGHT camera's own left edge.
    tgt_g = B.PILLAR_PID_TARGET_SECTION["green"]
    g_frac = ((tgt_g + 0.5) * strip.section_w - W) / W
    g_ok = Pillar("green", 0.9, ok_area, CAM_RIGHT, (0, 0, 1, 1),
                  at(CAM_RIGHT, g_frac))
    g_ghost = Pillar("green", 0.9, ok_area, CAM_LEFT, (0, 0, 1, 1),
                     at(CAM_LEFT, 0.5))
    checks.append((f"green in section {tgt_g}, absent left  -> placed",
                   placed(strip, g_ok, [g_ok])))
    checks.append((f"green in section {tgt_g} but ALSO left -> not placed",
                   not placed(strip, g_ok, [g_ok, g_ghost])))
    # Put it in whatever section red is currently aimed at.
    r_frac = ((B.PILLAR_PID_TARGET_SECTION["red"] + 0.5)
              * strip.section_w) / W
    r_ok = Pillar("red", 0.9, ok_area, CAM_LEFT, (0, 0, 1, 1), at(CAM_LEFT, r_frac))
    r_ghost = Pillar("red", 0.9, ok_area, CAM_RIGHT, (0, 0, 1, 1),
                     at(CAM_RIGHT, 0.5))
    tgt_r = B.PILLAR_PID_TARGET_SECTION["red"]
    checks.append((f"red in section {tgt_r}, absent right   -> placed",
                   placed(strip, r_ok, [r_ok])))
    checks.append((f"red in section {tgt_r} but ALSO right  -> not placed",
                   not placed(strip, r_ok, [r_ok, r_ghost])))

    # the one-sided target: silent once past, still active on the approach
    def bias_at(colour, sid, frac):
        ctl = PillarPID()
        return ctl.update(strip, Pillar(colour, 0.9, ok_area, sid,
                                        (0, 0, 1, 1), at(sid, frac)))[0]
    checks.append(("red past its target is silent",
                   bias_at("red", CAM_LEFT, 0.05) == 0.0))
    checks.append(("red approaching its target still steers",
                   bias_at("red", CAM_LEFT, 0.95) > 1.0))
    checks.append(("green past its target is silent",
                   bias_at("green", CAM_RIGHT, 0.99) == 0.0))
    checks.append(("green approaching its target still steers",
                   bias_at("green", CAM_RIGHT, 0.05) < -1.0))

    print()
    for name, good in checks:
        print(f"  {'PASS' if good else 'FAIL'}  {name}")
    n = sum(g for _n, g in checks)
    print(f"\n  {n}/{len(checks)}")
    return 0 if n == len(checks) else 1


# =========================================================
# Runners
# =========================================================

def run_pairs(model, args) -> int:
    """Offline, over matched cam0/cam1 pairs from collect_pillar_images.py."""
    # The collector writes to Pillar_model_training/images, so a bare name is
    # tried there as well as at the root -- this file moved, its data did not.
    cands = ([args.pairs] if os.path.isabs(args.pairs) else
             [os.path.join(HERE, args.pairs),
              os.path.join(HERE, "Pillar_model_training", args.pairs)])
    root = next((c for c in cands
                 if os.path.isdir(os.path.join(c, "cam0"))), cands[0])
    d0, d1 = os.path.join(root, "cam0"), os.path.join(root, "cam1")
    if not (os.path.isdir(d0) and os.path.isdir(d1)):
        print("Expected cam0/ and cam1/ (collect_pillar_images.py layout) in:")
        for c in cands:
            print(f"  {c}")
        return 1
    names = sorted(set(os.listdir(d0)) & set(os.listdir(d1)))
    if args.limit:
        names = names[:args.limit]
    print(f"{len(names)} matched pairs from {root}")

    # Beside the images, not inside them -- the trainer scans the image folders.
    out_dir = os.path.normpath(os.path.join(root, os.pardir, "pid_preview"))
    if args.save:
        os.makedirs(out_dir, exist_ok=True)

    ctl = PillarPID()
    strip = None
    seen = {"green": 0, "red": 0}
    acted = {"green": 0, "red": 0}
    n_placed = n_none = 0
    steers = []

    for fn in names:
        frames = {CAM_RIGHT: cv2.imread(os.path.join(d0, fn)),
                  CAM_LEFT: cv2.imread(os.path.join(d1, fn))}
        if frames[CAM_RIGHT] is None or frames[CAM_LEFT] is None:
            continue
        if strip is None:
            strip = Strip(frame_w=frames[CAM_RIGHT].shape[1],
                          per_cam=B.PILLAR_PID_SECTIONS)
        pillars = detect(model, frames, strip, args.conf)
        for p in pillars:
            seen[p.colour] += 1
        chosen = choose(pillars)
        steer, info = ctl.update(strip, chosen)
        if chosen is None:
            n_none += 1
        else:
            acted[chosen.colour] += 1
            info["placed"] = placed(strip, chosen, pillars)
            n_placed += bool(info["placed"])
            steers.append(steer)
        if args.save:
            cv2.imwrite(os.path.join(out_dir, fn),
                        render(frames, strip, pillars, chosen, steer, info))

    print(f"\n  detections        green={seen['green']}  red={seen['red']}")
    print(f"  frames steered    green={acted['green']}  red={acted['red']}")
    print(f"  frames with none  {n_none} of {len(names)}")
    if steers:
        a = np.array(steers)
        print(f"  steering us       min={a.min():+.0f}  median={np.median(a):+.0f}"
              f"  max={a.max():+.0f}   left={int((a<0).sum())} right={int((a>0).sum())}")
        print(f"  already placed    {n_placed}")
    if seen["red"] == 0 or seen["green"] == 0:
        missing = "RED" if seen["red"] == 0 else "GREEN"
        print("")
        print(f"  NOTE: zero {missing} detections. Check the detector with "
              "eval_pillar_model.py before")
        print("  blaming the PID -- half of this idea cannot be exercised by a model that")
        print("  cannot see that colour.")
    if args.save:
        print(f"\n  annotated frames -> {out_dir}")
    return 0


SENSOR_PORT = "/dev/ttyUSB0"
SERVO_PORT = "/dev/ttyACM0"
SERVO_BAUD = 9600


def run_live(model, args) -> int:
    """v6's perception and steering, with the motor left out entirely.

    Same camera pipeline, same button start, same loop shape and debug cadence
    as main_challenge_v6, and the same PillarPID the robot uses -- so the number
    on screen is the number v6 would be adding to its own steering. What is
    missing is everything that is not the pillar: no corner classifier, no wall
    classifier, no wall-following PID, and NO MOTOR.

    WITH NO PILLAR IN VIEW THE WHEEL CENTRES. That is the point of the rig.

    The ToF is still read, but only to display and to apply
    constrain_toward_walls, which can REDUCE a bias pointing at a close wall and
    never introduces steering of its own. --no-wall-guard removes even that.
    """
    # Imported HERE, not at module scope: --selftest and --pairs have to keep
    # working off the robot, and test_area_pillar.py imports from this file.
    from servo_controller import ServoController
    from robot_io import (SensorAdapter, button_pressed, set_pin, valid_tof,
                          wait_for_button_press)

    set_pin()
    print("Button pin ready.")
    servo = ServoController(port=SERVO_PORT, baud_rate=SERVO_BAUD)
    servo.connect()
    servo.center_steering()

    # No MotorSerial at all, so nothing shares this port and nothing can command
    # the wheels. The sensor object owns it outright.
    sensors = SensorAdapter(port=SENSOR_PORT, lr_hz=B.LR_HZ, front_hz=B.FRONT_HZ)
    sensors.open()

    caps = {}
    for sid in STRIP_ORDER:
        cap = cv2.VideoCapture(camera_pipeline(sid), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            for c in caps.values():
                c.release()
            servo.close()
            sensors.close()
            print(f"Could not open camera sensor-id={sid}. "
                  "Try: sudo systemctl restart nvargus-daemon")
            return 1
        caps[sid] = cap
    for _ in range(15):
        for c in caps.values():
            c.grab()

    strip = Strip(frame_w=CAM_WIDTH, per_cam=B.PILLAR_PID_SECTIONS)
    # Proportional across the WHOLE strip: the largest error the geometry can
    # produce maps to the largest steering the rig is allowed.
    full_scale = min(TEST_FULL_SCALE_SECTIONS, strip.n_sections - 0.5)
    rig_kp = TEST_MAX_STEER_US / full_scale
    ctl = PillarPID(kp=rig_kp, out_limit=TEST_MAX_STEER_US, slew=TEST_SLEW_US)
    dbg = 1.0 / B.DEBUG_HZ if B.DEBUG_HZ > 0 else 0.0

    print("")
    print("PILLAR STEERING ONLY -- THE MOTOR IS NEVER COMMANDED")
    print(f"  steering   = pillar PID alone, PROPORTIONAL over the whole strip")
    print(f"               {rig_kp:.0f}us per section, "
          f"{TEST_MAX_STEER_US:.0f}us at the full {full_scale:.1f}-section error")
    print(f"               (v6 still runs KP={B.PILLAR_PID_KP:.0f} / cap "
          f"{B.PILLAR_BIAS_LIMIT_US:.0f}us -- untouched)")
    print(f"               area >= {B.PILLAR_ACTIONABLE_AREA:.0f}")
    print(f"  targets    = green -> section "
          f"{B.PILLAR_PID_TARGET_SECTION['green']}, red -> section "
          f"{B.PILLAR_PID_TARGET_SECTION['red']}")
    print(f"  wall guard = {'OFF' if args.no_wall_guard else 'on (trims only)'}")
    print(f"  window     = {'OFF (jpg to ' + PREVIEW_FILE + ')' if args.headless else WIN}")
    print("Press the button to START (and again to STOP), 'q' in the window,"
          " or ctrl-c.")
    wait_for_button_press()

    last_dbg = 0.0
    loop_hz = 0.0
    last_t = None
    seen = {"green": 0, "red": 0}
    steered = 0
    frames_n = 0
    stopped_by = None
    last_preview = 0.0
    if not args.headless:
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, min(1400, strip.width // 2), args.view_h + 90)

    try:
        while True:
            if button_pressed():
                stopped_by = "button"
                break

            now = time.monotonic()
            if last_t:
                dt = now - last_t
                if dt > 0:
                    loop_hz = 0.9 * loop_hz + 0.1 / dt if loop_hz else 1.0 / dt
            last_t = now

            frames = {}
            for sid in STRIP_ORDER:
                ok, f = caps[sid].read()
                frames[sid] = f if ok else None
            if all(f is None for f in frames.values()):
                continue
            frames_n += 1

            snap = sensors.snapshot()
            left = valid_tof(snap.get("left_mm"))
            right = valid_tof(snap.get("right_front_mm"))
            front = valid_tof(snap.get("front_mm"))

            pillars = detect(model, frames, strip, args.conf)
            for p in pillars:
                seen[p.colour] += 1
            chosen = choose(pillars)
            raw, _info = ctl.update(strip, chosen)

            # NO v6 bias cap here: the rig is testing the steering law on its
            # own, so the PID's own out_limit (TEST_MAX_STEER_US) is the bound.
            steer = 0.0
            if chosen is not None:
                steer = raw
                if not args.no_wall_guard:
                    steer, _notes = B.constrain_toward_walls(steer, left, right)
            if abs(steer) > 1e-6:
                steered += 1

            servo.steer_delta(steer, channel=0)

            # The window: both cameras side by side with their three sections
            # each, the chosen pillar, the target band and the command.
            if args.headless:
                if (now - last_preview) >= args.preview_every:
                    last_preview = now
                    cv2.imwrite(PREVIEW_FILE,
                                render(frames, strip, pillars, chosen, steer,
                                       dict(_info, placed=bool(
                                           chosen and placed(strip, chosen,
                                                             pillars)))))
            else:
                cv2.imshow(WIN, render(frames, strip, pillars, chosen, steer,
                                       dict(_info, placed=bool(
                                           chosen and placed(strip, chosen,
                                                             pillars))),
                                       view_h=args.view_h))
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    stopped_by = "q"
                    break

            if dbg and (now - last_dbg) >= dbg:
                last_dbg = now
                if chosen is None:
                    what = "pillar=none (centred)"
                else:
                    what = (f"pillar={chosen.colour}({raw:+.0f}us"
                            f"{'/placed' if placed(strip, chosen, pillars) else ''})"
                            f" sec {strip.section_index(chosen.strip_x)}"
                            f"->{B.PILLAR_PID_TARGET_SECTION[chosen.colour]}"
                            f" a={chosen.area:.0f}")
                print(f"[pillar] {loop_hz:5.1f}hz steer={steer:+7.1f} | {what}"
                      f" | L={left} R={right} F={front}")
    except KeyboardInterrupt:
        stopped_by = "ctrl-c"
    finally:
        servo.center_steering()
        servo.close()
        sensors.close()
        for c in caps.values():
            c.release()
        if not args.headless:
            cv2.destroyAllWindows()
        try:
            import Jetson.GPIO as GPIO
            GPIO.cleanup()
        except Exception:
            pass

    print("")
    print(f"STOPPED: {stopped_by}" if stopped_by else "DONE")
    print(f"  frames {frames_n}   the wheel moved off centre on "
          f"{steered} ({100.0 * steered / max(frames_n, 1):.0f}%)")
    print(f"  detections: green={seen['green']}  red={seen['red']}")
    if not any(seen.values()):
        print("  NOTHING was ever detected, so the wheel stayed centred the")
        print("  whole time. Check the detector with test/test_area_pillar.py")
        print("  before reading anything into this.")
    return 0


def parse_args():
    ap = argparse.ArgumentParser(
        description="Dual-camera pillar PID -- steering only, no motor.")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--conf", type=float, default=B.PILLAR_PID_MIN_CONF)
    ap.add_argument("--selftest", action="store_true",
                    help="geometry and steering signs only; no model, no car")
    ap.add_argument("--pairs", default=None,
                    help="offline: folder holding cam0/ and cam1/ (e.g. images)")
    ap.add_argument("--limit", type=int, default=0, help="offline: first N pairs")
    ap.add_argument("--save", action="store_true",
                    help="offline: write annotated frames to pid_preview/")
    ap.add_argument("--no-wall-guard", action="store_true",
                    help="do not let a close wall trim the bias")
    ap.add_argument("--headless", action="store_true",
                    help="no window; write the same view to a jpg for SFTP")
    ap.add_argument("--preview-every", type=float, default=0.5,
                    help="headless: seconds between jpg writes")
    ap.add_argument("--view-h", type=int, default=340,
                    help="height of each camera pane in the window")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    if args.selftest:
        return selftest()
    if not os.path.exists(args.model):
        print(f"Model not found: {args.model}")
        return 1
    from ultralytics import YOLO
    print(f"Loading {args.model} ...")
    model = YOLO(args.model)
    if args.pairs:
        return run_pairs(model, args)
    try:
        model.to("cuda")
    except Exception as exc:
        print(f"  staying on CPU ({exc})")
    return run_live(model, args)


if __name__ == "__main__":
    raise SystemExit(main())
