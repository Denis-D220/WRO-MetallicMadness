# Models

This folder contains the trained machine-learning model files and CAD model references used by **MetallicMadness / ARBIBOT** for the WRO 2026 Future Engineers autonomous driving challenges.

The `.pt` files are used by the NVIDIA Jetson Orin Nano for visual perception. They detect traffic pillars, colored track lines, and corner/line features that support the robot's navigation logic.

The `cad/` folder contains the current 3D chassis design exported as STL files. These CAD files support mechanical reproducibility of the robot.

---

## Folder Contents

| File / folder | Description | Current status |
|---|---|---|
| `best_model_1.pt` | Earlier YOLO11n pillar/object detection checkpoint. Kept for comparison and fallback testing. | Archived / backup |
| `best_model_v2.pt` | Active YOLO11n pillar detection model used for the Obstacle Challenge. | Active |
| `Best_Model_Corner.pt` | Active YOLO11n corner/line detection model used for the Open Challenge. | Active |
| `cad/` | CAD folder containing the three-layer chassis STL design. | Active mechanical reference |

---

## CAD Chassis Design

The `cad/` folder contains the current 3D-printable chassis design for ARBIBOT. The chassis is organized as a three-layer structure so the robot can separate the bottom mechanical base, middle support layer, and top/electronics mounting layer.

| CAD file | Purpose |
|---|---|
| `cad/Chasis_Bottom.stl` | Bottom chassis layer. Supports the lower mechanical structure, wheel/motor layout, and base mounting geometry. |
| `cad/Chasis_Middle.stl` | Middle chassis layer. Provides intermediate support and spacing between the lower mechanical system and upper electronics layout. |
| `cad/Chasis_Top.stl` | Top chassis layer. Supports upper electronics, mounting points, and overall structure closure. |

The filename uses `Chasis` to match the current repository files. If the files are renamed later to `Chassis_*`, this README and any links in the Engineering Journal should be updated in the same commit.

### CAD Design Notes

The three-layer chassis helps the robot remain reproducible because each major structural level is stored as an STL file. The design supports:

- rear-wheel drive motor installation,
- steering servo and linkage placement,
- wheel and axle clearance,
- sensor and bumper mounting,
- electronics placement,
- battery and wiring organization,
- and future mechanical iteration.

Related documentation:

```text
docs/01-mechanical-design.md
engineering-journal/WRO_Engineering_Journal_MetallicMadness.md
v-photos/motor-stearing.png
v-photos/car-bottom.png
```

---

## Active Model Roles

| Model | Role | Classes | Used by |
|---|---|---|---|
| `best_model_v2.pt` | Pillar detection for the Obstacle Challenge | `Green_Pillar`, `Red_Pillar` | `main_challenge_02.py`, `Pillar_recognition.py` |
| `Best_Model_Corner.pt` | Track line / corner support for the Open Challenge | `Blue_line`, `Orange_line` | `main_challenge_01_v4.py`, `turn_models.py` |
| `best_model_1.pt` | Earlier pillar/object detection checkpoint kept for comparison | `Green_Pillar`, `Red_Pillar` | Legacy / fallback reference |

---

## Model Type

All current trained detection models are based on:

```text
Ultralytics YOLO11 nano
YOLO architecture: YOLO11n
Base model: yolo11n.pt
```

YOLO11n was selected because it gives a good balance between inference speed and detection quality on the Jetson Orin Nano. Larger models could improve accuracy, but they would also increase latency and could reduce the responsiveness of the camera, navigation, and serial-sensor loops.

---

## Dataset Summary

| Dataset | Related model file | Classes | Split | Image count |
|---|---|---|---|---:|
| Pillar dataset | `best_model_v2.pt` / `best_model_1.pt` | `Green_Pillar`, `Red_Pillar` | 70% train / 20% validation / 10% test | 100 images |
| Corner / line dataset | `Best_Model_Corner.pt` | `Blue_line`, `Orange_line` | 70% train / 20% validation / 10% test | 180 images |

---

## Training Procedure

Both YOLO models were trained with the same general pipeline. The Roboflow project and class labels differ between the pillar model and the corner/line model.

1. **Data collection**  
   Images were captured from the competition track using the robot's own camera. This keeps the training data close to the real deployment camera angle, lens behavior, and lighting conditions.

2. **Annotation**  
   Images were uploaded to Roboflow and manually labeled image by image using bounding boxes.

3. **Dataset split**  
   The dataset was split as:

   ```text
   70% training
   20% validation
   10% test
   ```

4. **Export format**  
   Datasets were exported from Roboflow in YOLOv11 format, including images, YOLO `.txt` label files, and `data.yaml`.

5. **Training**  
   Training was performed with Ultralytics YOLO using `yolo11n.pt` as the pretrained base model.

   ```python
   from ultralytics import YOLO

   model = YOLO("yolo11n.pt")
   model.train(data=".../data.yaml", epochs=15, imgsz=640)
   ```

6. **Validation and testing**  
   The resulting `best.pt` model was tested on held-out images. After visual verification, the model was copied into the `Models/` folder and used by the Jetson challenge scripts.

---

## Training Configuration

| Field | Value |
|---|---|
| Model family | Ultralytics YOLO |
| Architecture | YOLO11n |
| Base weights | `yolo11n.pt` |
| Annotation tool | Roboflow |
| Labeling method | Manual bounding-box labeling by the team |
| Export format | YOLOv11 |
| Dataset split | 70% train / 20% validation / 10% test |
| Training environment | PC/GPU environment |
| Jetson role | Inference only |
| Epochs | 15 |
| Image size | 640 x 640 |
| Numeric mAP / validation score | Not recorded in the current repository notes |
| Runtime speed on Jetson | Approximately 60 FPS during testing |

---

## How the Models Are Used

The Jetson camera captures frames using OpenCV/GStreamer. The challenge script sends those frames to an Ultralytics YOLO model. The detections are converted into navigation decisions.

General flow:

```text
Camera frame
    -> YOLO11n model
    -> detection boxes / classes / confidence
    -> navigation logic
    -> steering and motor commands
```

Obstacle Challenge flow:

```text
Camera frame
    -> pillar YOLO model
    -> Green_Pillar / Red_Pillar
    -> screen zone + bounding-box area
    -> pass-side steering logic
```

Open Challenge flow:

```text
Camera frame
    -> corner/line YOLO model
    -> Blue_line / Orange_line
    -> corner or line feature
    -> corner direction / turn support
```

---

## Detection Classes

### Pillar model

```text
Green_Pillar
Red_Pillar
```

### Corner / line model

```text
Blue_line
Orange_line
```

---

## Current Confidence Thresholds

The current working confidence thresholds are:

```text
Pillar confidence threshold: 0.30
Corner line confidence threshold: 0.48
```

These values are also documented in:

```text
docs/07-calibration-procedures.md
```

They should be re-tested after major changes to:

- camera angle,
- camera lens or focus,
- lighting conditions,
- model weights,
- dataset augmentation,
- camera color tuning,
- or final field setup.

---

## Model Metadata Table

| Field | `best_model_1.pt` | `best_model_v2.pt` | `Best_Model_Corner.pt` |
|---|---|---|---|
| Model type | YOLO11n | YOLO11n | YOLO11n |
| Role | Earlier pillar/object checkpoint | Active pillar model | Active corner/line model |
| Dataset | Pillar dataset | Pillar dataset | Corner / line dataset |
| Dataset size | 100 images | 100 images | 180 images |
| Classes | `Green_Pillar`, `Red_Pillar` | `Green_Pillar`, `Red_Pillar` | `Blue_line`, `Orange_line` |
| Image size | 640 x 640 | 640 x 640 | 640 x 640 |
| Epochs | 15 | 15 | 15 |
| Inference FPS on Jetson | Approximately 60 FPS pipeline speed | Approximately 60 FPS pipeline speed | Approximately 60 FPS pipeline speed |
| Active challenge script | Legacy / fallback reference | `main_challenge_02.py` | `main_challenge_01_v4.py` |
| Supporting module | `Pillar_recognition.py` | `Pillar_recognition.py` | `turn_models.py` |
| Status | Archived / backup | Active | Active |

---

## Related Source Files

The models are used by these project files:

```text
src/main_challenge_01_v4.py
src/main_challenge_02.py
src/Pillar_recognition.py
src/turn_models.py
src/color_tuning.py
```

Related documentation:

```text
docs/01-mechanical-design.md
docs/03-software-architecture.md
docs/04-obstacle-strategy.md
docs/06-testing-and-tuning.md
docs/07-calibration-procedures.md
docs/10-risk-register.md
engineering-journal/WRO_Engineering_Journal_MetallicMadness.md
```

Related media and CAD evidence:

```text
Models/cad/Chasis_Bottom.stl
Models/cad/Chasis_Middle.stl
Models/cad/Chasis_Top.stl
v-photos/car-bottom.png
v-photos/motor-stearing.png
```

---

## Naming Convention

Recommended naming convention for future model files:

```text
pillar_yolo_v1.pt
pillar_yolo_v2.pt
corner_yolo_v1.pt
corner_yolo_v2.pt
```

Recommended naming convention for future CAD files:

```text
chassis_bottom_v1.stl
chassis_middle_v1.stl
chassis_top_v1.stl
```

The current filenames are kept because they match the existing project history and may be referenced by the repository. If a model or CAD file is renamed, the code and documentation must be updated in the same commit.

---

## Version Control Note

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

STL files can also become large. If GitHub rejects the CAD files, track them with Git LFS too:

```bash
git lfs track "*.stl"
git add .gitattributes
```

---

## Security Note

Do not commit Roboflow API keys or other private tokens to the repository. Use an environment variable instead.

Recommended pattern:

```python
import os
from roboflow import Roboflow

rf = Roboflow(api_key=os.environ["ROBOFLOW_API_KEY"])
```

Recommended `.gitignore` entry:

```text
.env
```

---

## Update Policy

When a model changes:

1. Add the new `.pt` file to this folder.
2. Update the Python script that loads the model.
3. Update this README with the filename, classes, role, and status.
4. Record confidence threshold changes in `docs/07-calibration-procedures.md`.
5. Record test results in `docs/06-testing-and-tuning.md`.
6. Keep old models only if they are useful for comparison or rollback.

When the CAD chassis changes:

1. Add the new `.stl` files to `Models/cad/`.
2. Update this README with the new filenames and purpose.
3. Update `docs/01-mechanical-design.md` if the chassis geometry or layer structure changes.
4. Update the Engineering Journal if the CAD design is part of the final reproducibility package.
5. Keep older CAD files only if they document an important design iteration.

---

## Current Status

The folder includes three trained YOLO11n checkpoints:

```text
best_model_1.pt
best_model_v2.pt
Best_Model_Corner.pt
```

The active challenge models are:

```text
Obstacle Challenge: best_model_v2.pt
Open Challenge: Best_Model_Corner.pt
```

The CAD folder includes the current three-layer chassis STL design:

```text
cad/Chasis_Bottom.stl
cad/Chasis_Middle.stl
cad/Chasis_Top.stl
```

The remaining improvement for this folder is optional: record future numeric validation metrics, such as mAP, precision, and recall, if those values are exported from Ultralytics or Roboflow.
