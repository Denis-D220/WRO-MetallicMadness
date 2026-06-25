"""
turn_models.py  --  corner-line detection for the Open Challenge.

Detects the blue and orange corner lines with the YOLO corner model and reports
which way to turn and how close the corner is.

Heuristic (with improvements over the first idea):
    - Detect every blue and orange line box.
    - The NEAREST line is the one whose bounding box sits lowest in the frame
      (largest bottom-y). Perspective makes the closer line appear lower; this is
      more reliable than box width because both lines are parallel diagonals and
      can have similar widths. Width is kept as a tie-breaker / confidence.
      (Set NEAREST_METRIC = "width" to use the original width-only rule.)
    - Nearest line BLUE   -> turn LEFT
      Nearest line ORANGE -> turn RIGHT
      (flip with BLUE_TURN / ORANGE_TURN if your round convention differs.)
    - Distance is estimated from the nearest box's vertical position:
      closeness rises as the line moves toward the bottom of the frame.
      Calibrate to millimetres with DISTANCE_CALIB once you have measurements.

detect_corner(model, frame, ...) returns a dict:
    {
        "detected":       bool,                 # any line found?
        "turn":           "left"|"right"|None,  # suggested turn
        "nearest_color":  "blue"|"orange"|None,
        "distance":       float|None,           # mm if calibrated, else == distance_norm
        "distance_norm":  float|None,           # 0.0 far (top)  ..  1.0 close (bottom)
        "nearest":        line dict | None,     # the chosen nearest line
        "lines":          [line dict, ...],     # all detected lines
    }
"""

import cv2
import numpy as np
from ultralytics import YOLO

from color_tuning import load_params, apply_color_tuning


# =========================================================
# CONFIGURATION
# =========================================================

MODEL_PATH = "/home/daniel/WRO2026-MetallicMadness/Models/Best_Model_Corner.pt"

# Turn convention.
BLUE_TURN = "left"
ORANGE_TURN = "right"

# How to pick the nearest line: "bottom_y" (recommended) or "width".
NEAREST_METRIC = "bottom_y"

# Optional distance calibration: list of (bottom_y_pixels, real_mm) points,
# sorted by bottom_y. Leave as None to return the normalized 0..1 closeness.
# Example after measuring:
#   DISTANCE_CALIB = [(300, 1000), (520, 400), (660, 150)]
DISTANCE_CALIB = None


# =========================================================
# Helpers
# =========================================================

def _classify_color(name: str) -> str | None:
    name = name.lower()
    if "blue" in name:
        return "blue"
    if "orange" in name:
        return "orange"
    return None


def _turn_for_color(color: str) -> str | None:
    if color == "blue":
        return BLUE_TURN
    if color == "orange":
        return ORANGE_TURN
    return None


def _interp_distance(bottom_y: float) -> float | None:
    """Map bottom_y (pixels) to mm via the calibration table, if provided."""
    if not DISTANCE_CALIB:
        return None
    table = DISTANCE_CALIB
    if bottom_y <= table[0][0]:
        return float(table[0][1])
    if bottom_y >= table[-1][0]:
        return float(table[-1][1])
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x0 <= bottom_y <= x1:
            return float(y0 + (bottom_y - x0) * (y1 - y0) / (x1 - x0))
    return None


def _nearness_key(line: dict):
    """Sort key: bigger = nearer. Primary metric + width tie-breaker."""
    if NEAREST_METRIC == "width":
        return (line["width"], line["bottom_y"])
    return (line["bottom_y"], line["width"])


# =========================================================
# Main detection function
# =========================================================

def detect_corner(model, frame, color_params: dict | None = None,
                  conf: float | None = None) -> dict:
    """
    Run the corner model on one frame and decide turn + distance.

    Args:
        model:        a loaded ultralytics YOLO model.
        frame:        BGR image.
        color_params: dict from color_tuning.load_params(); if given, the same
                      color correction used during tuning is applied here.
        conf:         confidence threshold; defaults to color_params["conf"] or 0.5.
    """
    if color_params is not None:
        frame = apply_color_tuning(frame, color_params)
        if conf is None:
            conf = color_params.get("conf", 0.5)
    if conf is None:
        conf = 0.5

    h = frame.shape[0]

    results = model(frame, imgsz=640, verbose=False, conf=conf)
    r0 = results[0]

    lines = []
    for box in r0.boxes:
        color = _classify_color(model.names[int(box.cls[0].item())])
        if color is None:
            continue
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        lines.append({
            "color": color,
            "conf": float(box.conf[0].item()),
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "cx": (x1 + x2) / 2.0,
            "cy": (y1 + y2) / 2.0,
            "width": x2 - x1,
            "height": y2 - y1,
            "bottom_y": y2,
        })

    result = {
        "detected": False,
        "turn": None,
        "nearest_color": None,
        "distance": None,
        "distance_norm": None,
        "nearest": None,
        "lines": lines,
    }

    if not lines:
        return result

    nearest = max(lines, key=_nearness_key)

    # Normalized closeness: 0.0 at top of frame (far), 1.0 at bottom (close).
    distance_norm = float(np.clip(nearest["bottom_y"] / float(h), 0.0, 1.0))
    distance_mm = _interp_distance(nearest["bottom_y"])

    result.update({
        "detected": True,
        "turn": _turn_for_color(nearest["color"]),
        "nearest_color": nearest["color"],
        "distance": distance_mm if distance_mm is not None else distance_norm,
        "distance_norm": distance_norm,
        "nearest": nearest,
    })
    return result


def draw_result(frame, result) -> None:
    """Overlay detected lines and the decision (for testing/visualization)."""
    for line in result["lines"]:
        is_nearest = result["nearest"] is line
        color_bgr = (0, 165, 255) if line["color"] == "orange" else (255, 100, 0)
        thickness = 4 if is_nearest else 2
        cv2.rectangle(frame,
                      (int(line["x1"]), int(line["y1"])),
                      (int(line["x2"]), int(line["y2"])),
                      color_bgr, thickness)
        cv2.putText(frame, f"{line['color']} {line['conf']:.2f}",
                    (int(line["x1"]), max(20, int(line["y1"]) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

    if result["detected"]:
        dist = result["distance"]
        dist_txt = f"{dist:.0f}mm" if DISTANCE_CALIB else f"{dist:.2f}"
        hud = (f"TURN {result['turn'].upper()}  "
               f"near={result['nearest_color']}  dist={dist_txt}")
    else:
        hud = "no corner line"
    cv2.putText(frame, hud, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)


# =========================================================
# Convenience wrapper (loads model + tuning once)
# =========================================================

class CornerDetector:
    def __init__(self, model_path: str = MODEL_PATH, device: str = "cuda"):
        self.model = YOLO(model_path)
        self.model.to(device)
        self.color_params = load_params()

    def detect(self, frame) -> dict:
        return detect_corner(self.model, frame, color_params=self.color_params)


# =========================================================
# Standalone camera demo
# =========================================================

def _demo():
    detector = CornerDetector()

    pipeline = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1, format=NV12, colorimetry=bt601 ! "
        "nvvidconv ! "
        "video/x-raw, width=1280, height=720, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Error: Could not open camera. Try: sudo systemctl restart nvargus-daemon")
        raise SystemExit(1)

    print("detect_corner demo. 'q' to quit.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            result = detector.detect(frame)

            if result["detected"]:
                print(f"turn={result['turn']:>5}  near={result['nearest_color']:>6}  "
                      f"dist={result['distance']:.2f}  lines={len(result['lines'])}")

            view = apply_color_tuning(frame, detector.color_params)
            draw_result(view, result)
            cv2.imshow("turn_models - detect_corner", view)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Camera released.")


if __name__ == "__main__":
    _demo()
