# WRO 2026 Metallic Madness — `systemd` service

This folder contains the `systemd` unit that auto-starts `main.py` at boot on the
Jetson so the robot is ready for a WRO Open Challenge run without a console.

The unit is **safe by design**: at startup `main.py` initializes the hardware,
forces the motor into a **STOPPED / safe state**, centers the steering, and then
**waits for the physical Start button**. The vehicle does **not** move until the
button is pressed (after the judge says "Go").

- Unit file: [`wro-main.service`](wro-main.service)
- Service runs as user **`daniel`**
- Repo root (assumed): `/home/daniel/WRO2026-MetallicMadness`
- Python: `/usr/bin/python3`

> If your repository path or Python interpreter differ, edit `WorkingDirectory`,
> `ExecStart`, and the `StandardOutput`/`StandardError` paths in
> `wro-main.service` before installing.

---

## Competition startup behavior

```
Power switch ON
      │
      ▼
Jetson boots ──► systemd starts wro-main.service ──► runs main.py
      │
      ▼
Robot initializes (camera, sensors, serial/motor, servo)
      │
      ▼
Motors are explicitly STOPPED + steering centered  (safe state)
      │
      ▼
Robot WAITS for the Start button   ◄── no movement here
      │
   judge says "Go"  ──►  Start button PRESSED
      │
      ▼
Round starts ──► robot begins moving and runs the laps
```

`main.py` logs each of these lifecycle steps to both the console (→ `journalctl`)
and to a rotating file at `logs/main.log`.

---

## Install

```bash
# 0. From the repo root, make sure the logs folder exists (main.py also creates it)
cd /home/daniel/WRO2026-MetallicMadness
mkdir -p logs

# 1. Copy the unit file into systemd's directory
sudo cp service/wro-main.service /etc/systemd/system/wro-main.service

# 2. Reload systemd so it sees the new unit
sudo systemctl daemon-reload

# 3. Enable it to start automatically at every boot
sudo systemctl enable wro-main.service

# 4. Start it now (without rebooting)
sudo systemctl start wro-main.service
```

---

## Everyday commands

```bash
# Status (running? last exit? recent log lines)
sudo systemctl status wro-main.service

# Start / stop / restart
sudo systemctl start   wro-main.service
sudo systemctl stop    wro-main.service
sudo systemctl restart wro-main.service

# Enable (auto-start at boot) / disable (no auto-start)
sudo systemctl enable  wro-main.service
sudo systemctl disable wro-main.service
```

---

## Watching logs

```bash
# Live systemd journal for this service
journalctl -u wro-main.service -f

# This boot only, newest last
journalctl -u wro-main.service -b

# The application's own rotating log (lifecycle events, corner triggers, errors)
tail -f logs/main.log

# Raw stdout / stderr captured by the unit
tail -f logs/service.log
tail -f logs/service-error.log
```

> To see the high-frequency per-frame telemetry (`[WATCH]` / `[FRONT 4x4]`), run
> with `WRO_LOG_LEVEL=DEBUG`. For a manual run:
> `WRO_LOG_LEVEL=DEBUG python3 main.py`. For the service, add
> `Environment=WRO_LOG_LEVEL=DEBUG` to the unit and `daemon-reload` + `restart`.

---

## Test it manually (recommended before relying on the service)

```bash
cd /home/daniel/WRO2026-MetallicMadness

# Syntax check
python3 -m py_compile main.py

# Run by hand (Ctrl+C to stop). Motors stay stopped until the Start button.
python3 main.py
```

---

## Stop / restart during development

While testing, you usually want to stop the auto-started service (so it does not
fight your manual `python3 main.py` over the serial port), then re-enable it
before the competition.

```bash
# Stop the running service during testing
sudo systemctl stop wro-main.service

# Restart it (e.g. after editing code or pulling new files)
sudo systemctl restart wro-main.service

# Disable auto-start at boot while developing
sudo systemctl disable wro-main.service

# Re-enable auto-start (remember to do this before the competition)
sudo systemctl enable wro-main.service
```

---

## Disable auto-start / remove the service

```bash
# Stop it and prevent auto-start at boot
sudo systemctl stop wro-main.service
sudo systemctl disable wro-main.service

# Fully remove the unit
sudo rm /etc/systemd/system/wro-main.service
sudo systemctl daemon-reload
sudo systemctl reset-failed wro-main.service
```

---

## Serial / device permissions

The robot talks to the STM32 over a serial port (e.g. `/dev/ttyUSB0`) and the
servo over `/dev/ttyACM0`, uses the CSI camera, and reads a GPIO button. The
service runs as `daniel`, so that user needs access to those device groups.

```bash
# Check which serial devices exist and their group ownership
ls -l /dev/ttyUSB* /dev/ttyACM*

# Confirm daniel's current groups
groups daniel

# Add daniel to the useful hardware groups
sudo usermod -aG dialout daniel   # serial ports (/dev/ttyUSB*, /dev/ttyACM*)
sudo usermod -aG video   daniel   # camera
sudo usermod -aG gpio    daniel   # GPIO (start button)

# Group membership only takes effect after a fresh login session — reboot to be safe
sudo reboot
```

> If `main.py` fails to open the serial port with a *Permission denied* error,
> it is almost always missing `dialout` membership — re-check `groups daniel`
> after rebooting.

---

## Notes

- `Restart=on-failure` only relaunches on a **crash** (non-zero exit). A clean
  finish (laps completed) exits 0 and is **not** restarted — the car will not
  re-run itself on the track.
- `main.py` writes its own rotating log (`logs/main.log`, 5 MB × 5 backups) in
  addition to the raw `StandardOutput`/`StandardError` files configured here.
