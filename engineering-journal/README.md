# Engineering Journal

This folder contains the Engineering Journal for **MetallicMadness / ARBIBOT**, the WRO Future Engineers self-driving car project.

The Engineering Journal documents the complete engineering process behind the robot, including mechanical design, power architecture, sensor placement, software architecture, obstacle strategy, testing, failures, improvements, and reproducibility information.

## Files

| File / Folder | Description |
|---|---|
| `engineering-journal.md` | Editable Markdown version of the Engineering Journal. This is the source document used for updates. |
| `engineering-journal.pdf` | PDF version prepared for judges, review, printing, and submission. |
| `images/` | Supporting images used in the journal, including robot photos, wiring diagrams, subsystem images, and architecture references. |

## Purpose

The goal of this documentation is to show not only the final robot, but also the engineering reasoning used to design and improve it. The journal explains why components were selected, how subsystems interact, what problems were found during testing, and how the design evolved through iteration.

## Main Topics Covered

- Team and project overview
- WRO design constraints
- Mechanical mobility and steering design
- Power system and sensor architecture
- Jetson Orin Nano and STM32F411 system roles
- Camera, YOLO model, and obstacle detection strategy
- UART communication between Jetson and STM32
- Testing results, failures, and improvements
- Build, setup, and reproducibility notes

## Update Policy

The Markdown file should be updated first. After major changes, the PDF version should be regenerated so both files remain synchronized.

## Recommended Git Workflow

```bash
git add engineering-journal/
git commit -m "docs: add WRO engineering journal"
git push
```
