# Video

This folder contains video evidence for the **MetallicMadness / ARBIBOT** WRO Future Engineers self-driving car project.

The videos in this folder are used to document real robot behavior during testing and challenge runs. They support the engineering documentation by showing that the robot can move autonomously, follow the track, steer, detect the environment, and execute challenge-specific behavior.

## Folder contents

| File | Description |
|---|---|
| `challenge01.mp4` | Video of ARBIBOT running the first WRO Future Engineers challenge, also known as the Open Challenge. This video shows autonomous driving behavior, steering response, lane behavior, and system integration during testing. |

## Video purpose

The purpose of this folder is to provide visual evidence for:

- autonomous robot movement,
- steering control,
- lane-following behavior,
- corner handling,
- motor control,
- sensor and software integration,
- Open Challenge testing progress,
- and WRO documentation requirements.

Videos are especially important because they show the complete robot operating as a system, not only individual components working separately.

## Challenge videos

### Open Challenge / Challenge 01

File:

```text
challenge01.mp4
```

Description:

```text
ARBIBOT running the first WRO Future Engineers challenge.
```

This video should be used as evidence in:

```text
README.md
engineering-journal/engineering-journal.md
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

Recommended Markdown reference from `engineering-journal/engineering-journal.md`:

```markdown
[Open Challenge test video](../video/challenge01.mp4)
```

## Recommended future files

The following files should be added as the project progresses:

| File | Description |
|---|---|
| `challenge02.mp4` | Video of the Obstacle Challenge with red and green pillars. |
| `parking-test.mp4` | Video showing the robot entering or completing the parking phase. |
| `sensor-test.mp4` | Optional video showing distance sensor behavior during calibration. |
| `motor-test.mp4` | Optional video showing motor, steering, and encoder validation. |
| `video.md` | Optional summary file with links, descriptions, dates, and notes for each video. |

## Suggested video documentation table

When new videos are added, update this table:

| Video | Challenge / Test | Date | Result | Notes |
|---|---|---|---|---|
| `challenge01.mp4` | Open Challenge | [TODO: date] | [TODO: success / partial / test run] | Robot running first challenge |
| `challenge02.mp4` | Obstacle Challenge | [TODO] | [TODO] | Red/green pillar behavior |
| `parking-test.mp4` | Parking | [TODO] | [TODO] | Parking strategy evidence |

## Video naming convention

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

## Update policy

When a new video is added:

1. Place the video file in this folder.
2. Add the video name and description to this README.
3. Reference the video from the root `README.md`.
4. Reference the video from `docs/06-testing-and-tuning.md`.
5. If the video is used as official evidence, also reference it from the Engineering Journal.

## Current status

The folder currently includes the Open Challenge test video:

```text
challenge01.mp4
```

Additional videos should be added later for the Obstacle Challenge and parking behavior.
