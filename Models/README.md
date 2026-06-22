# Models

This folder contains trained machine-learning model files used by **MetallicMadness / ARBIBOT** for WRO Future Engineers autonomous driving.

The models are used by the NVIDIA Jetson Orin Nano to perform visual recognition tasks such as red/green pillar detection and line/corner detection. These files are part of the robot’s perception system and are required for running the autonomous challenge scripts.

## Folder contents

| File / folder | Description |
|---|---|
| `best_model_1.pt` | YOLO model checkpoint used for visual detection. Current expected use: pillar/object detection. Verify final class list and script reference. |
| `best_model_v2.pt` | Updated YOLO model checkpoint used for visual detection. Current expected use: improved pillar/object detection. Verify which challenge script uses this version. |
| `Best_Model_Corner.pt` | YOLO model checkpoint used for line/corner detection in the Open Challenge. |
| `cad/` | Folder reserved for CAD/model-related files or exported mechanical model references if needed. |

## Model purpose

ARBIBOT uses vision models to extract information that distance sensors cannot provide.

The models help the robot:

- detect red and green traffic pillars,
- identify visual track/corner features,
- estimate object position in the camera frame,
- choose the correct obstacle passing side,
- support Open Challenge corner behavior,
- and provide visual input for high-level navigation.

## Current model roles

| Model | Expected role | Used by |
|---|---|---|
| `best_model_1.pt` | Pillar/object detection model | `main_challenge_02.py`, `Pillar_recognition.py` |
| `best_model_v2.pt` | Updated pillar/object detection model | `main_challenge_02.py`, `Pillar_recognition.py` |
| `Best_Model_Corner.pt` | Corner/line detection model | `main_challenge_01_v4.py`, `turn_models.py` |

> Important: Before final submission, verify the exact model filename loaded by each Python script and update this README if the active model changes.

## Expected classes

The obstacle model is expected to detect:

```text
Green_Pillar
Red_Pillar
```

The corner/line model class names should be verified from the final training run and documented here:

```text
[TODO] Add final class names for Best_Model_Corner.pt
```

## How models are used

The Jetson camera captures frames using OpenCV/GStreamer. The Python challenge script sends those frames to a YOLO model using Ultralytics.

General flow:

```text
Camera frame
    -> YOLO model
    -> detection boxes/classes/confidence
    -> navigation logic
    -> steering and motor commands
```

For the Obstacle Challenge:

```text
Camera frame
    -> pillar YOLO model
    -> Red_Pillar / Green_Pillar
    -> screen zone and bounding-box area
    -> pass-side steering logic
```

For the Open Challenge:

```text
Camera frame
    -> corner/line YOLO model
    -> corner or line feature
    -> corner direction / turn support
```

## Recommended model metadata to add

For final documentation, add the following metadata for each model:

| Field | `best_model_1.pt` | `best_model_v2.pt` | `Best_Model_Corner.pt` |
|---|---|---|---|
| Model type | YOLO / [TODO version] | YOLO / [TODO version] | YOLO / [TODO version] |
| Training date | [TODO] | [TODO] | [TODO] |
| Dataset size | [TODO] | [TODO] | [TODO] |
| Classes | [TODO] | [TODO] | [TODO] |
| Image size | [TODO] | [TODO] | [TODO] |
| mAP / validation score | [TODO] | [TODO] | [TODO] |
| Inference FPS on Jetson | [TODO] | [TODO] | [TODO] |
| Active challenge script | [TODO] | [TODO] | [TODO] |
| Status | Candidate / active / archived | Candidate / active / archived | Active / candidate |

## Calibration values

The current development confidence thresholds are documented in `docs/07-calibration-procedures.md`.

Current working values:

```text
Pillar confidence threshold: 0.30
Corner line confidence threshold: 0.48
```

These values should be re-tested after:

- camera angle changes,
- lighting changes,
- model replacement,
- dataset retraining,
- camera color tuning,
- or final field setup.

## Naming convention

Recommended naming convention for future model files:

```text
pillar_yolo_v1.pt
pillar_yolo_v2.pt
corner_yolo_v1.pt
corner_yolo_v2.pt
```

If the current filenames remain unchanged for compatibility with code, do not rename them until the Python scripts are updated.

## Version control note

`.pt` model files can be large. If GitHub rejects a model file because of file size limits, use Git LFS.

Recommended Git LFS setup:

```bash
git lfs install
git lfs track "*.pt"
git add .gitattributes
```

Then add the models normally:

```bash
git add Models/*.pt
```

## Related files

The models are related to these project files:

```text
src/main_challenge_01_v4.py
src/main_challenge_02.py
src/Pillar_recognition.py
src/turn_models.py
src/color_tuning.py
docs/03-software-architecture.md
docs/04-obstacle-strategy.md
docs/07-calibration-procedures.md
docs/10-risk-register.md
```

## Update policy

When a model changes:

1. Add the new `.pt` file to this folder.
2. Update the Python script that loads the model.
3. Update this README with the new filename and role.
4. Record confidence threshold changes in `docs/07-calibration-procedures.md`.
5. Record test results in `docs/06-testing-and-tuning.md`.
6. Keep old models only if they are useful for comparison.

## Current status

The folder currently includes three trained model checkpoints:

```text
best_model_1.pt
best_model_v2.pt
Best_Model_Corner.pt
```

The final active model for each challenge should be confirmed after testing and documented in the metadata table above.
