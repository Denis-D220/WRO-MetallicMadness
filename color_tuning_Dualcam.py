"""
color_tuning_Dualcam.py  --  shared frame color correction, PER CAMERA.

Single source of truth for the color tuning found with Model_Corner_Test_tune.py.
This module holds the IMPLEMENTATION; color_tuning.py is a thin shim that
re-exports it, so the older programs keep working off this one file and no second
copy can drift away from it.

PER-CAMERA tuning: the robot has TWO CSI cameras (sensor-id=0 = physical RIGHT,
sensor-id=1 = physical LEFT). They are different physical modules, so they do NOT
share a color cast -- one set of gains cannot white-balance both. color_tune.txt
therefore holds one section per camera:

    [cam0]
    R_gain=1.13
    ...
    [cam1]
    R_gain=1.05
    ...

A LEGACY flat file (no [camN] headers -- what the old tuner wrote) is still read
and applies to every camera, so nothing breaks before the cameras are re-tuned.

Usage:
    from color_tuning import load_params, apply_color_tuning

    params = load_params(sensor_id=0)      # this camera's section
    tuned = apply_color_tuning(frame, params)
    results = model(tuned, imgsz=640, conf=params["conf"])
"""

import os
import re

import cv2
import numpy as np


PARAM_FILE = "color_tune.txt"

# Physical mounting, mirrored from main_challenge_02_v2.py.
CAM_RIGHT = 0
CAM_LEFT = 1
DEFAULT_SENSOR_ID = CAM_RIGHT      # what load_params() picks when not told


CAM_SENSOR_MODE = 2                # 1920x1080; mode 1 if 30 fps is still ignored
CAM_CAPTURE_WIDTH = 1920           # must equal the sensor mode's width
CAM_CAPTURE_HEIGHT = 1080          # must equal the sensor mode's height
CAM_WIDTH = 1280                   # delivered to OpenCV after nvvidconv scaling
CAM_HEIGHT = 720
CAM_FPS = 30
CAM_GAIN_RANGE = "1 2"             # analog gain cap (sensor allows 22.25x!)
CAM_DIGITAL_GAIN_RANGE = "1 1"     # ISP digital gain; noise multiplier, keep at 1


def camera_pipeline(sensor_id: int, width: int = CAM_WIDTH,
                    height: int = CAM_HEIGHT, fps: int = CAM_FPS,
                    gain_range: str = CAM_GAIN_RANGE,
                    digital_gain_range: str = CAM_DIGITAL_GAIN_RANGE,
                    sensor_mode: int | None = CAM_SENSOR_MODE,
                    capture_width: int = CAM_CAPTURE_WIDTH,
                    capture_height: int = CAM_CAPTURE_HEIGHT) -> str:
    """GStreamer pipeline for one CSI camera with the tuned ISP settings.

    VERIFY IT TOOK: the Argus banner on startup must say the sensor mode you
    asked for and "Frame Rate = 30.000000". If it still says 59.999999, this mode
    refuses the reduced rate -- switch CAM_SENSOR_MODE to 1 (3840x2160 native
    30 fps) and set the capture size to match.

    Pass overrides to deviate for one program (e.g. fps=60 if loop rate turns out
    to matter more than noise), but the defaults are what the robot runs.
    """
    mode = f"sensor-mode={sensor_mode} " if sensor_mode is not None else ""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} {mode}"
        f'gainrange="{gain_range}" '
        f'ispdigitalgainrange="{digital_gain_range}" ! '
        f"video/x-raw(memory:NVMM), width={capture_width}, "
        f"height={capture_height}, framerate={fps}/1, format=NV12, "
        "colorimetry=bt601 ! "
        "nvvidconv ! "
        f"video/x-raw, width={width}, height={height}, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )

# Fallback values if color_tune.txt is missing (no change to the frame).
DEFAULT_PARAMS = {
    "R_gain": 1.00,
    "G_gain": 1.00,
    "B_gain": 1.00,
    "bright": 0.0,
    "sat": 1.00,
    "conf": 0.50,
}

_SECTION_RE = re.compile(r"^\[\s*cam(\d+)\s*\]$", re.IGNORECASE)


def _parse(path: str):
    """Read the tune file. Returns ({sensor_id: params}, legacy_params_or_None).

    legacy_params is set only for an old-style flat file (no [camN] headers); it
    is the value every camera falls back to.
    """
    sections: dict[int, dict] = {}
    flat = dict(DEFAULT_PARAMS)
    saw_flat_key = False
    current = None

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            header = _SECTION_RE.match(line)
            if header:
                current = int(header.group(1))
                sections.setdefault(current, dict(DEFAULT_PARAMS))
                continue

            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            if key not in DEFAULT_PARAMS:
                continue
            try:
                parsed = float(value.strip())
            except ValueError:
                continue

            if current is None:
                flat[key] = parsed
                saw_flat_key = True
            else:
                sections[current][key] = parsed

    return sections, (flat if saw_flat_key else None)


def load_all(path: str = PARAM_FILE) -> dict:
    """Load every camera section as {sensor_id: params}.

    A legacy flat file yields {DEFAULT_SENSOR_ID: params}; a missing file yields
    {} (callers should fall back to DEFAULT_PARAMS).
    """
    if not os.path.exists(path):
        print(f"[color_tuning] {path} not found; using defaults (no tuning).")
        return {}

    sections, legacy = _parse(path)
    if sections:
        return sections
    if legacy is not None:
        return {DEFAULT_SENSOR_ID: legacy}
    return {}


def load_params(path: str = PARAM_FILE, sensor_id: int | None = None) -> dict:
    """Load one camera's tuning parameters. Falls back to defaults.

    sensor_id=None means DEFAULT_SENSOR_ID, which keeps the old single-camera
    behavior for callers that never passed a camera.
    """
    if sensor_id is None:
        sensor_id = DEFAULT_SENSOR_ID

    if not os.path.exists(path):
        print(f"[color_tuning] {path} not found; cam{sensor_id} using defaults.")
        return dict(DEFAULT_PARAMS)

    sections, legacy = _parse(path)

    if sensor_id in sections:
        params = sections[sensor_id]
        source = f"[cam{sensor_id}]"
    elif legacy is not None:
        params = legacy
        source = "legacy flat file (untuned per-camera)"
    elif sections:
        # File is sectioned but this camera is missing -- borrow the lowest id
        # rather than silently reverting to no correction at all.
        donor = min(sections)
        params = sections[donor]
        source = f"[cam{donor}] (no [cam{sensor_id}] section)"
    else:
        params = dict(DEFAULT_PARAMS)
        source = "defaults (file has no usable values)"

    print(f"[color_tuning] loaded {path} cam{sensor_id} from {source}: {params}")
    return dict(params)


def save_params(params_by_cam: dict, path: str = PARAM_FILE) -> None:
    """Write every camera's parameters as a sectioned file."""
    lines = ["# color_tuning.py -- one section per CSI camera.",
             f"# cam{CAM_RIGHT} = physical RIGHT, cam{CAM_LEFT} = physical LEFT.",
             ""]
    for sensor_id in sorted(params_by_cam):
        p = params_by_cam[sensor_id]
        lines.append(f"[cam{sensor_id}]")
        lines.append(f"R_gain={p['R_gain']:.2f}")
        lines.append(f"G_gain={p['G_gain']:.2f}")
        lines.append(f"B_gain={p['B_gain']:.2f}")
        lines.append(f"bright={int(round(p['bright']))}")
        lines.append(f"sat={p['sat']:.2f}")
        lines.append(f"conf={p['conf']:.2f}")
        lines.append("")

    with open(path, "w") as f:
        f.write("\n".join(lines))


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
