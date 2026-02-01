from typing import Dict, List
from Pillar_recognition import Detection


class PillarCounter:
    """
    Enter/Exit counter per pillar color (GREEN/RED) with debounce.

    "In recognition range" is defined as:
      - detection zone is in recognition_zones
      - detection area >= min_area (optional)
    """

    def __init__(
        self,
        recognition_zones={"CENTER"},
        min_area: float = 0,
        enter_frames: int = 3,
        exit_frames: int = 6
    ):
        self.recognition_zones = set(recognition_zones)
        self.min_area = min_area
        self.enter_frames = enter_frames
        self.exit_frames = exit_frames

        self.state: Dict[str, Dict] = {
            "GREEN": {"in": False, "in_streak": 0, "out_streak": 0, "entered": 0, "exited": 0},
            "RED":   {"in": False, "in_streak": 0, "out_streak": 0, "entered": 0, "exited": 0},
        }

    def _seen_in_range_this_frame(self, dets: List[Detection]) -> Dict[str, bool]:
        seen = {"GREEN": False, "RED": False}
        for d in dets:
            if d.area < self.min_area:
                continue
            if d.zone in self.recognition_zones:
                seen[d.color] = True
        return seen

    def update(self, dets: List[Detection]) -> None:
        """
        Update enter/exit counts based on OUT->IN and IN->OUT transitions.
        """
        seen = self._seen_in_range_this_frame(dets)

        for color in ("GREEN", "RED"):
            s = self.state[color]

            if seen[color]:
                s["in_streak"] += 1
                s["out_streak"] = 0

                # OUT -> IN (debounced)
                if (not s["in"]) and (s["in_streak"] >= self.enter_frames):
                    s["in"] = True
                    s["entered"] += 1

            else:
                s["out_streak"] += 1
                s["in_streak"] = 0

                # IN -> OUT (debounced)
                if s["in"] and (s["out_streak"] >= self.exit_frames):
                    s["in"] = False
                    s["exited"] += 1

    def get_counts(self) -> Dict[str, Dict]:
        """Return the full internal state for GREEN and RED."""
        return self.state

    def summary_lines(self):
        g = self.state["GREEN"]
        r = self.state["RED"]
        return (
            f"GREEN entered={g['entered']} exited={g['exited']} in={g['in']}",
            f"RED   entered={r['entered']} exited={r['exited']} in={r['in']}",
        )