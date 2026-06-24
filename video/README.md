# Video

This folder contains video evidence for the **MetallicMadness / ARBIBOT** WRO Future Engineers self-driving car project.

The videos in this folder document real robot behavior during testing and challenge runs. They support the engineering documentation by showing that the robot can move autonomously, follow the track, steer, detect the environment, respond to obstacles, and execute challenge-specific behavior.

---

## Folder Contents

| File | Challenge / Test | Description | Current status |
|---|---|---|---|
| `challenge01.mp4` | Open Challenge / Challenge 01 | Video of ARBIBOT running the first WRO Future Engineers challenge. It shows autonomous driving behavior, steering response, lane behavior, corner handling, and system integration during testing. | Available |
| `challenge02.mp4` | Obstacle Challenge / Challenge 02 | Video of ARBIBOT running the Obstacle Challenge with red and green traffic pillars. It documents pillar detection, pass-side strategy, steering response, and obstacle-handling behavior. | Available |

---

## Video Purpose

The purpose of this folder is to provide visual evidence for:

- autonomous robot movement,
- steering control,
- lane-following behavior,
- corner handling,
- motor control,
- sensor and software integration,
- Open Challenge testing progress,
- Obstacle Challenge red/green pillar behavior,
- and WRO documentation requirements.

Videos are especially important because they show the complete robot operating as a system, not only individual components working separately.

---

## Challenge Videos

## Open Challenge / Challenge 01

File:

```text
challenge01.mp4
```

Description:

```text
ARBIBOT running the first WRO Future Engineers challenge, also known as the Open Challenge.
```

This video should be used as evidence in:

```text
README.md
engineering-journal/WRO_Engineering_Journal_MetallicMadness.md
docs/06-testing-and-tuning.md
```

Recommended Markdown reference from the root `README.md`:

```markdown
[Open Challenge test video](video/challenge01.mp4)
```

Recommended Markdown reference from `docs/06-testing-and-tuning.md`:

```markdown
[Open Challenge test video](../video/challenge01.mp4)
```

Recommended Markdown reference from `engineering-journal/WRO_Engineering_Journal_MetallicMadness.md`:

```markdown
[Open Challenge test video](../video/challenge01.mp4)
```

---

## Obstacle Challenge / Challenge 02

File:

```text
challenge02.mp4
```

Description:

```text
ARBIBOT running the second WRO Future Engineers challenge, also known as the Obstacle Challenge.
```

This video documents obstacle-handling behavior with red and green traffic pillars. It supports the Obstacle Challenge documentation by showing the robot using vision, steering control, and pass-side logic during a real run.

This video should be used as evidence in:

```text
README.md
engineering-journal/WRO_Engineering_Journal_MetallicMadness.md
docs/04-obstacle-strategy.md
docs/06-testing-and-tuning.md
```

Recommended Markdown reference from the root `README.md`:

```markdown
[Obstacle Challenge test video](video/challenge02.mp4)
```

Recommended Markdown reference from `docs/04-obstacle-strategy.md`:

```markdown
[Obstacle Challenge test video](../video/challenge02.mp4)
```

Recommended Markdown reference from `docs/06-testing-and-tuning.md`:

```markdown
[Obstacle Challenge test video](../video/challenge02.mp4)
```

Recommended Markdown reference from `engineering-journal/WRO_Engineering_Journal_MetallicMadness.md`:

```markdown
[Obstacle Challenge test video](../video/challenge02.mp4)
```

---

## Video Documentation Table

| Video | Challenge / Test | Result / Evidence | Notes |
|---|---|---|---|
| `challenge01.mp4` | Open Challenge | Open Challenge behavior evidence | Shows autonomous driving, lane behavior, corner handling, and system integration. |
| `challenge02.mp4` | Obstacle Challenge | Obstacle Challenge behavior evidence | Shows red/green pillar behavior, obstacle strategy, steering response, and vision-based handling. |

---

## Optional Future Videos

The main challenge videos are now present. Optional future videos can be added if the team wants more evidence for debugging, calibration, or reproducibility.

| File | Description |
|---|---|
| `parking-test.mp4` | Optional video showing the robot entering or completing the parking phase. |
| `sensor-test.mp4` | Optional video showing distance sensor behavior during calibration. |
| `motor-test.mp4` | Optional video showing motor, steering, and encoder validation. |
| `video.md` | Optional summary file with links, descriptions, dates, and notes for each video. |

---

## Video Naming Convention

Use lowercase names with hyphens or simple challenge numbers.

Recommended names:

```text
challenge01.mp4
challenge02.mp4
parking-test.mp4
sensor-calibration-test.mp4
motor-command-test.mp4
```

Avoid spaces in filenames because spaces can make links harder to manage in Markdown and GitHub.

---

## Notes for GitHub

Video files can become large. If GitHub rejects a video upload because of file size limits, use one of these options:

1. Compress the video.
2. Use Git LFS.
3. Upload the video to YouTube or another public video platform.
4. Add the public video link in this README and in `video.md`.

If an external video link is used, include:

- video title,
- challenge name,
- date,
- short description,
- and whether the run was a full success, partial success, or test run.

Recommended Git LFS setup for video files:

```bash
git lfs install
git lfs track "*.mp4"
git add .gitattributes
```

---

## Update Policy

When a new video is added:

1. Place the video file in this folder.
2. Add the video name and description to this README.
3. Reference the video from the root `README.md`.
4. Reference the video from the relevant documentation file in `docs/`.
5. If the video is used as official evidence, also reference it from the Engineering Journal.

---

## Current Status

The folder currently includes video evidence for both main WRO Future Engineers challenges:

```text
challenge01.mp4
challenge02.mp4
```

`challenge01.mp4` documents the Open Challenge.  
`challenge02.mp4` documents the Obstacle Challenge.

Additional parking, sensor, or motor validation videos are optional future evidence files.
