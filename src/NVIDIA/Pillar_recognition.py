from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class Detection:
    color: str        # 'GREEN' or 'RED'
    conf: float
    x1: float
    y1: float
    x2: float
    y2: float
    cx: float
    cy: float
    area: float
    zone: str         # 'LEFT' | 'CENTER' | 'RIGHT'


def zone_from_x(x: float, w: int) -> str:
    """Split screen into 3 vertical zones based on x coordinate."""
    left_border = w / 3.0
    right_border = 2.0 * w / 3.0
    if x < left_border:
        return "LEFT"
    elif x > right_border:
        return "RIGHT"
    return "CENTER"


def normalize_pillar_color(class_name: str) -> Optional[str]:
    """Map model class name to 'GREEN' or 'RED'."""
    lname = class_name.lower()
    if lname.startswith("green"):
        return "GREEN"
    if lname.startswith("red"):
        return "RED"
    return None


class PillarRecognition:
    """
    Pillar recognition + steering hint logic.
    Rules (your current spec):
      - Green is correct ONLY in RIGHT zone.
      - Red is correct ONLY in LEFT zone.
      - CENTER counts as wrong (needs correction).
    """

    def __init__(self, conf_th: float = 0.35):
        self.conf_th = conf_th

    def parse_detections(self, yolo_result, frame_w: int) -> List[Detection]:
        """
        Convert Ultralytics result[0] into a list of Detection objects.
        yolo_result is results[0] from Ultralytics.
        """
        dets: List[Detection] = []

        if yolo_result.boxes is None or len(yolo_result.boxes) == 0:
            return dets

        names: Dict[int, str] = yolo_result.names

        for b in yolo_result.boxes:
            conf = float(b.conf[0])
            if conf < self.conf_th:
                continue

            cls_id = int(b.cls[0])
            class_name = names.get(cls_id, str(cls_id))

            color = normalize_pillar_color(class_name)
            if color is None:
                continue

            x1, y1, x2, y2 = map(float, b.xyxy[0])
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            area = (x2 - x1) * (y2 - y1)
            zone = zone_from_x(cx, frame_w)

            dets.append(Detection(
                color=color, conf=conf,
                x1=x1, y1=y1, x2=x2, y2=y2,
                cx=cx, cy=cy, area=area, zone=zone
            ))

        return dets

    def choose_closest(self, dets: List[Detection]) -> Optional[Detection]:
        """Pick the closest pillar using largest bbox area."""
        if not dets:
            return None
        return max(dets, key=lambda d: d.area)

    def guidance(self, chosen: Optional[Detection]) -> str:
        """
        Return steering hint text, or '' when pillar is already correct.
        """
        if chosen is None:
            return ""

        if chosen.color == "GREEN":
            # Correct only on RIGHT; LEFT or CENTER => GO RIGHT
            return "" if chosen.zone == "RIGHT" else f"GREEN in {chosen.zone} -> GO RIGHT"

        if chosen.color == "RED":
            # Correct only on LEFT; RIGHT or CENTER => GO LEFT
            return "" if chosen.zone == "LEFT" else f"RED in {chosen.zone} -> GO LEFT"

        return ""