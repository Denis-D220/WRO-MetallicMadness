"""
color_tuning.py  --  shared frame color correction for line/pillar detection.

Single source of truth for the color tuning found with Model_Corner_Test_tune.py.
Import this in the corner test and the main pipeline so they never drift.

Usage:
    from color_tuning import load_params, apply_color_tuning

    params = load_params()                 # reads color_tune.txt (or defaults)
    tuned = apply_color_tuning(frame, params)
    results = model(tuned, imgsz=640, conf=params["conf"])
"""

import os

import cv2
import numpy as np


PARAM_FILE = "color_tune.txt"

# Fallback values if color_tune.txt is missing (no change to the frame).
DEFAULT_PARAMS = {
    "R_gain": 1.00,
    "G_gain": 1.00,
    "B_gain": 1.00,
    "bright": 0.0,
    "sat": 1.00,
    "conf": 0.50,
}


def load_params(path: str = PARAM_FILE) -> dict:
    """Load tuning parameters saved by the tuner. Falls back to defaults."""
    params = dict(DEFAULT_PARAMS)

    if not os.path.exists(path):
        print(f"[color_tuning] {path} not found; using defaults (no tuning).")
        return params

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            if key in params:
                try:
                    params[key] = float(value.strip())
                except ValueError:
                    pass

    print(f"[color_tuning] loaded {path}: {params}")
    return params


def apply_color_tuning(frame, params: dict):
    """Apply per-channel gain + brightness offset + saturation scaling."""
    r_gain = params["R_gain"]
    g_gain = params["G_gain"]
    b_gain = params["B_gain"]
    bright = params["bright"]
    sat = params["sat"]

    b, g, r = cv2.split(frame.astype(np.float32))

    b = b * b_gain + bright
    g = g * g_gain + bright
    r = r * r_gain + bright

    out = cv2.merge([b, g, r])
    out = np.clip(out, 0, 255).astype(np.uint8)

    if abs(sat - 1.0) > 1e-3:
        hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * sat, 0, 255)
        out = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    return out
