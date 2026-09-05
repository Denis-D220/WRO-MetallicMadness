"""
unwedge.py -- notice the car has stopped moving, back off an inch, carry on.

WHY THIS IS NOT A ONE-LINER. The car has no bumper switch, no IMU, no motor
current sensing and reads no encoder count back: "the car hits a wall" is not
an event anything on this robot can observe. It has to be INFERRED from the
only stream available, the ToF sides -- and the inference has one trap that has
already cost this project two runs:

    A FROZEN SENSOR LOOKS EXACTLY LIKE A STUCK CAR.

L=492 R=472 repeated for thirteen frames is either a car pinned against
something or a serial link that stopped delivering, and the VALUES cannot tell
you which. So freshness is not a nicety here, it is the whole difference
between backing out of a wedge and throwing the car into reverse on a straight.
Every gate below exists for that reason.

The signature of a real wedge, from the logs (printed lines, ~2.5 frames each):

    R = 24, 24, 19, 20, 18, 18, 18, 18, 18      <- ended a run
    L = 18, 18, 19, 18, 16, 15                  <- ended a run
    R = 38, 38, 27, 26                          <- ended a run

against a car merely driving close to a wall and recovering on its own:

    L = 98, 98, 105, 105, 133, 147, 181         <- must NOT fire
    R = 494, 448, 211, 38                       <- must NOT fire

Close AND unchanging AND fresh AND under power. Any one of those alone is a
false positive waiting to happen.

A HEAD-ON HIT IS NOT DETECTABLE HERE, and this file deliberately does not try.
When the car drives nose-first into a wall the side sensors are still looking
at the SIDE walls -- 438 and 480mm in the run that ended that way, and they
would stay there forever. The only trace is both readings going still, and
that was MEASURED against real cruise from the same run:

    tightest both-sides spread over a 10-frame window
        real cruise (385-406mm of honest noise) : 5mm
        nose against a wall                     : 3mm

A 2mm margin, against sensor noise of the same size. A false positive means
throwing the car into reverse at speed on a straight, so that rule was written,
measured, and removed. The camera is what sees forward: a sustained
barrier_front run commits a corner instead -- see
behavior_manager.BARRIER_FRONT_RUN_FRAMES, where the same separation is 18
frames against 115.

Tuning lives in behavior_manager.py, as everywhere else.
"""

import time

import behavior_manager as B


class StuckDetector:
    """Is a side distance close, unchanging, fresh, and the car under power?

    Feed it every frame. It answers True at most once per wedge -- firing is
    latching, so the caller does the maneuver once and then calls clear().
    """

    def __init__(self):
        self.hist = {"left": [], "right": []}
        # -inf, NOT 0.0: with a zero here the cooldown compares against "the
        # nudge at time zero" and swallows the FIRST one, which is the only
        # one that matters. Every real wedge trace failed on exactly this.
        self.last_fire = float("-inf")
        self.attempts = 0
        self.side = None                 # which side was against something

    def clear(self):
        """After a nudge: forget the history so the next verdict is fresh."""
        self.hist = {"left": [], "right": []}
        self.side = None

    def update(self, left_mm, right_mm, sides_fresh, moving, now=None) -> bool:
        """True exactly when a wedge has just been confirmed."""
        now = time.monotonic() if now is None else now

        # A STALE READING PROVES NOTHING. Dropping the history rather than
        # holding it means a freeze can never accumulate toward a verdict --
        # the frames either arrive fresh or they do not count at all.
        if not sides_fresh or not moving:
            self.hist = {"left": [], "right": []}
            return False

        fired = False
        for name, d in (("left", left_mm), ("right", right_mm)):
            h = self.hist[name]
            if d is None or d <= 0 or d > B.STUCK_SIDE_MM:
                h.clear()
                continue
            h.append(float(d))
            if len(h) > B.STUCK_FRAMES:
                h.pop(0)
            if len(h) < B.STUCK_FRAMES:
                continue
            if max(h) - min(h) > B.STUCK_SPREAD_MM:
                continue
            # Confirmed. The cooldown stops a car that is genuinely jammed from
            # reversing on every frame for the rest of the run.
            if self._allowed(now):
                self._arm(now, name)
                fired = True

        return fired

    def _allowed(self, now) -> bool:
        """Cooldown and attempt cap -- shared by both rules."""
        if now - self.last_fire < B.REVERSE_COOLDOWN_S:
            return False
        return not (B.REVERSE_MAX_ATTEMPTS
                    and self.attempts >= B.REVERSE_MAX_ATTEMPTS)

    def _arm(self, now, side):
        self.last_fire = now
        self.attempts += 1
        self.side = side


def reverse_nudge(steer_now, set_steer, motor_stop, motor_reverse,
                  motor_forward, sleep=time.sleep, clear=None):
    """Straighten, back off an inch, restore the steering, drive on.

    Exactly the sequence asked for, and the order matters at both ends:

      * STRAIGHT BEFORE REVERSING. Reversing on lock swings the nose across
        the track, and the car is against a wall precisely because there is no
        room for that. Straight backs it out along the line it came in on.
      * RESTORE BEFORE DRIVING. The steering the car had was the escape the
        follower or the emergency was already asking for -- it was right, it
        just had no room to work. Give it back the instant there is room, and
        do it BEFORE the wheels turn, so the first centimetre forward is aimed.

    THE REVERSE IS BLIND. There is no rear sensor on this robot, so nothing
    here can see what is behind the car. That is the whole reason it is a nudge
    and not a maneuver: keep REVERSE_NUDGE_S short enough that the worst case
    is touching whatever the car just came off.

    Returns the steering that was restored, for the log.
    """
    set_steer(0.0)                       # straight
    motor_stop()
    sleep(B.REVERSE_SETTLE_S)            # the car must actually STOP, and the
                                         # servo must actually REACH centre,
                                         # before anything drives backwards
    set_steer(0.0)                       # re-assert: this is the command that
                                         # has to be in force while the wheels
                                         # are turning, not merely one that was
                                         # sent earlier
    motor_reverse()
    backed = 0.0
    if clear is None:
        sleep(B.REVERSE_NUDGE_S)
        backed = B.REVERSE_NUDGE_S
    else:
        while backed < B.REVERSE_NUDGE_S:
            sleep(B.REVERSE_POLL_S)
            backed += B.REVERSE_POLL_S
            if clear():
                break
    motor_stop()
    sleep(B.REVERSE_SETTLE_S)            # let the reverse END before the wheel
                                         # turns -- restoring lock on a car
                                         # still rolling backwards steers it
                                         # the wrong way
    set_steer(steer_now)                 # back to what it was
    motor_forward()
    return steer_now, backed


# =========================================================
# Self-test -- real traces from the track logs, no robot needed
# =========================================================

def selftest() -> int:
    checks = []

    def run(side, seq, fresh=True, moving=True, per_line=3):
        """Feed a printed-log trace. Each printed line is ~2.5-3 real frames,
        so repeat each value -- feeding one sample per line understates how
        much evidence the detector actually gets."""
        d = StuckDetector()
        t, hit = 0.0, False
        for v in seq:
            for _ in range(per_line):
                t += 1.0 / 13.0
                a = v if side == "left" else None
                b = v if side == "right" else None
                if d.update(a, b, fresh, moving, now=t):
                    hit = True
        return hit

    # --- wedges that ended real runs: MUST fire ---
    checks.append(("wedge R=24,24,19,20,18,18,18,18,18",
                   run("right", [24, 24, 19, 20, 18, 18, 18, 18, 18])))
    checks.append(("wedge L=18,18,19,18,16,15",
                   run("left", [18, 18, 19, 18, 16, 15])))
    checks.append(("wedge R=38,38,27,26",
                   run("right", [38, 38, 27, 26])))
    checks.append(("wedge L=15,16,18,18,17,17",
                   run("left", [15, 16, 18, 18, 17, 17])))
    # The one the 55mm threshold missed entirely: pinned at 59-74mm, frozen
    # within 3mm, for eighty frames, while the emergency steered +250 away.
    checks.append(("wedge L=60,61,60,62,60,59,60,61 (the 55-90mm band)",
                   run("left", [60, 61, 60, 62, 60, 59, 60, 61])))

    # --- driving, not stuck: MUST NOT fire ---
    checks.append(("close but recovering L=98,105,133,147,181",
                   not run("left", [98, 98, 105, 105, 133, 147, 181])))
    checks.append(("driving past R=494,448,211,38",
                   not run("right", [494, 448, 211, 38])))
    checks.append(("normal cruise L=435,423,395,385,422",
                   not run("left", [435, 423, 395, 385, 422])))
    checks.append(("wall follower working R=211,245,274,320",
                   not run("right", [211, 245, 274, 320])))

    # --- the trap: a FROZEN sensor is not a stuck car ---
    checks.append(("STALE frozen L=18 x12 -> silent",
                   not run("left", [18] * 12, fresh=False)))
    checks.append(("STALE frozen R=472 x12 -> silent",
                   not run("right", [472] * 12, fresh=False)))
    checks.append(("stopped car at 18mm -> silent (not under power)",
                   not run("left", [18] * 12, moving=False)))

    # --- gating ---
    d = StuckDetector()
    t = 0.0
    fires = 0
    for _ in range(400):
        t += 1.0 / 13.0
        if d.update(None, 18.0, True, True, now=t):
            fires += 1
    span = 400 / 13.0
    checks.append((f"cooldown+cap: {span:.0f}s pinned -> {fires} nudges, not 400",
                   fires <= B.REVERSE_MAX_ATTEMPTS))
    checks.append(("attempt cap respected",
                   d.attempts <= B.REVERSE_MAX_ATTEMPTS))
    checks.append(("clear() resets the history",
                   (d.clear() or True) and d.hist == {"left": [], "right": []}))

    # --- the maneuver: order is the whole point ---
    log = []
    got, _secs = reverse_nudge(
        steer_now=-150.0,
        set_steer=lambda u: log.append(f"steer={u:+.0f}"),
        motor_stop=lambda: log.append("stop"),
        motor_reverse=lambda: log.append("reverse"),
        motor_forward=lambda: log.append("forward"),
        sleep=lambda s: None)
    checks.append(("straight BEFORE reversing",
                   log.index("steer=+0") < log.index("reverse")))
    # The re-assert: a straight command must be the LAST steering command
    # before the wheels turn, not merely an earlier one.
    checks.append(("straight is the LAST steer command before reversing",
                   log[log.index("reverse") - 1] == "steer=+0"))
    checks.append(("stopped BEFORE reversing",
                   log.index("stop") < log.index("reverse")))
    checks.append(("stopped again BEFORE restoring steer",
                   log.index("reverse") < log.index("steer=-150")))
    checks.append(("steering restored BEFORE driving on",
                   log.index("steer=-150") < log.index("forward")))
    checks.append(("returns the steering it restored", got == -150.0))

    # --- backs up only as far as it has to ---
    def timed(clear_after):
        """How long it reverses when the side clears after N polls."""
        n = {"i": 0}
        def clear():
            n["i"] += 1
            return n["i"] >= clear_after
        _s, secs = reverse_nudge(
            steer_now=-150.0, set_steer=lambda u: None,
            motor_stop=lambda: None, motor_reverse=lambda: None,
            motor_forward=lambda: None, sleep=lambda s: None, clear=clear)
        return secs
    early = timed(3)
    checks.append((f"clears early -> stops at {early:.2f}s not "
                   f"{B.REVERSE_NUDGE_S:.2f}s", early < B.REVERSE_NUDGE_S))
    never = timed(10**9)
    checks.append((f"never clears -> capped at {B.REVERSE_NUDGE_S:.2f}s",
                   never <= B.REVERSE_NUDGE_S + 1e-9))
    checks.append(("no clear-check -> full ceiling (old behaviour)",
                   abs(reverse_nudge(
                       steer_now=0.0, set_steer=lambda u: None,
                       motor_stop=lambda: None, motor_reverse=lambda: None,
                       motor_forward=lambda: None, sleep=lambda s: None)[1]
                       - B.REVERSE_NUDGE_S) < 1e-9))

    ok = sum(1 for _, c in checks if c)
    for name, c in checks:
        print(f"  {'PASS' if c else 'FAIL'}  {name}")
    print(f"\n  {ok}/{len(checks)}")
    return 0 if ok == len(checks) else 1


if __name__ == "__main__":
    raise SystemExit(selftest())
