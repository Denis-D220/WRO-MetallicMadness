"""
sensor_distance_v3.py  --  side reader (LEFT, RIGHT_FRONT, RIGHT_REAR) PLUS the
new VL53L8CX 4x4 FRONT matrix.

FIRMWARE NOTE -- the command mapping changed when the front sensor became a
VL53L8CX (an 8x8/4x4 matrix part instead of the old single-zone VL53L4CD):

    0x0105 "RIGHT_FRONT: a mm | RIGHT_REAR: b mm | LEFT: c mm"
            -> all THREE side distances in one frame (right_rear moved here)
    0x0003  VL53L8CX 4x4 matrix (multi-line "Distances:" / "Statuses:" text)
            -> front-barrier distance + bumper skew

(The previous firmware put right_rear on 0x0003 as "FRONT: z mm"; that is gone.)

FRONT MATRIX -> SCALARS (empirical, from test/Serial_CMD_Dist_4x4_3VL53L4CD_final):
    * Row 0 (top) reads >2500 mm -- it shoots over the barrier; DISCARDED.
    * Row 1 (2nd row) tracks the front-barrier distance and is usable across the
      ~60-100 cm turn-decision window, so we derive front_mm from row 1.
    * Below ~50 cm row 1 gets flaky, so callers should TRIGGER the turn at the
      ~65 cm edge (FRONT_CORNER_MM in main) and not rely on sub-50 cm values.
    * The 4 columns of row 1 span the barrier left->right, so the spread between
      the left and the right columns says whether the bumper is parallel to the
      barrier (front_skew_mm; 0 == square-on). Sign needs field calibration.

Shared serial: the motor and the sensors are the same STM32 on one port. This
object OWNS the port and reads; the motor reuses the serial object and only
WRITES (wait_response=False), guarded by write_lock.
"""

import re
import statistics
import threading
import time

import serial


START_BYTE = b"$"
END_BYTE = b"\n"

CMD_READ_SIDES = 0x0105        # "RIGHT_FRONT | RIGHT_REAR | LEFT"
CMD_READ_FRONT = 0x0003        # VL53L8CX 4x4 matrix
MAX_FRAME_LEN = 4096           # matrix frame is multi-line -> larger than sides

# Front-matrix tuning (sensor-level; the corner THRESHOLDS live in main).
FRONT_MAX_MM = 2500            # cells above this are "no barrier" / over-the-top
FRONT_FLOOR_MM = 420          # cells BELOW this in row 1 are the floor, not the
                              # barrier (rows 2-3 read ~130-260 mm; when the
                              # barrier only partly fills row 1 the floor cells
                              # bleed in and drag the median down -> a barrier at
                              # ~70 cm misreads as ~220 mm and the turn fires far
                              # too late). Excluding them makes front_mm track the
                              # BARRIER even from a single in-row cell.
                              # RAISED 350->420 for the TILTED sensor: the tilt
                              # brings the floor into row 1 at ~266-314 mm, only
                              # ~36 mm below the old 350 cutoff -> floor drift could
                              # cross it and false-fire. 420 doubles the margin; the
                              # wall still fires at FRONT_CORNER_MM(900) long before
                              # 420, and sub-420 is covered by the RF<250 + emergency.
FRONT_BARRIER_ROW = 1          # row 0 overshoots (>2500); row 1 == barrier
FRONT_GOOD_STATUS = (5, 9)     # VL53L8CX: 5 valid, 9 ~valid (range OK)

_SIDES_RE = re.compile(
    r"RIGHT_FRONT:\s*(-?\d+)\s*mm"
    r"\s*\|\s*RIGHT_REAR:\s*(-?\d+)\s*mm"
    r"\s*\|\s*LEFT:\s*(-?\d+)\s*mm",
    re.IGNORECASE,
)


def parse_front_matrix(text: str):
    """Parse the multi-line VL53L8CX text into (distances, statuses) 4x4 lists.

    Returns (None, None) on an error/short frame. Each is a list of rows; rows
    may be ragged if the firmware truncated, callers must tolerate that.
    """
    lower = text.lower()
    if any(m in lower for m in ("not available", "not ready", "error", "too fast")):
        return None, None

    distances, statuses = [], []
    section = None
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        low = line.lower()
        if low.startswith("stream"):
            continue
        if low.startswith("distances"):
            section = "d"
            continue
        if low.startswith("statuses"):
            section = "s"
            continue
        values = [int(v) for v in re.findall(r"-?\d+", line)]
        if not values:
            continue
        if section == "d":
            distances.append(values)
        elif section == "s":
            statuses.append(values)

    return distances, statuses


def front_scalars(distances, statuses):
    """Reduce the 4x4 matrix to (front_mm, front_skew_mm, row, n_cells) from row 1.

    front_mm     == median of the BARRIER cells in row 1 (-1 if none). Floor
                    cells (< FRONT_FLOOR_MM) are excluded so a barrier that only
                    partly fills row 1 is not masked by the floor.
    front_skew_mm== mean(right cols) - mean(left cols) over the barrier cells
                    (None if a side is missing). 0 == bumper square to barrier.
    n_cells      == how many row-1 cells are in the barrier band. A lone cell
                    among OPEN (>FRONT_MAX) cells is usually an edge/wall corner
                    catching the sensor, not the barrier face -- the caller can
                    require >= 2 to avoid turning too soon.
    """
    if not distances or len(distances) <= FRONT_BARRIER_ROW:
        return -1, None, [], 0

    row = distances[FRONT_BARRIER_ROW]
    strow = (statuses[FRONT_BARRIER_ROW]
             if statuses and len(statuses) > FRONT_BARRIER_ROW else [])

    def in_barrier_band(d):
        # A barrier cell sits ABOVE the floor and below the overshoot ceiling.
        return FRONT_FLOOR_MM <= d <= FRONT_MAX_MM

    def cell_ok(idx):
        if not in_barrier_band(row[idx]):
            return False
        if idx < len(strow):
            return strow[idx] in FRONT_GOOD_STATUS
        return True

    good = [i for i in range(len(row)) if cell_ok(i)]
    if not good:
        # status may be missing/odd -> fall back to plain band gating
        good = [i for i in range(len(row)) if in_barrier_band(row[i])]
    if not good:
        # no barrier cell in row 1 -> barrier not in view yet (far/absent)
        return -1, None, row, 0

    front_mm = int(round(statistics.median(row[i] for i in good)))

    half = len(row) // 2
    left = [row[i] for i in good if i < half]
    right = [row[i] for i in good if i >= half]
    skew = None
    if left and right:
        skew = (sum(right) / len(right)) - (sum(left) / len(left))

    return front_mm, skew, row, len(good)


class SideSensorsV3:
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        lr_hz: float = 20.0,
        front_hz: float = 10.0,
        rr_hz: float = None,          # DEPRECATED: right_rear is now on 0x0105
        max_valid_mm: int = 2000,
        debug: bool = False,
        ser: "serial.Serial | None" = None,
        write_lock: "threading.Lock | None" = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.sides_interval = 1.0 / lr_hz if lr_hz > 0 else 0.05
        self.front_interval = 1.0 / front_hz if front_hz > 0 else None
        self.max_valid_mm = max_valid_mm
        self.debug = debug

        # Serial: provided (shared with motor) or owned by this object.
        self.ser = ser
        self._own_serial = ser is None
        self.write_lock = write_lock or threading.Lock()

        self._data_lock = threading.Lock()
        self._running = False
        self._paused = False
        self._thread = None
        self.rx_buffer = bytearray()

        # Sides (all three from one 0x0105 frame).
        self.left_mm = -1
        self.right_front_mm = -1
        self.right_rear_mm = -1
        self.last_sides_time = 0.0
        self._last_sides_request = 0.0

        # Front matrix (0x0003).
        self.front_mm = -1
        self.front_skew_mm = None
        self.front_row = []
        self.front_cells = 0          # # of row-1 cells in the barrier band
        self.front_matrix = []        # full 4x4 distances, for on-robot debug
        self.last_front_time = 0.0
        self._last_front_request = 0.0
        self._front_enabled = True    # gate the heavy matrix poll at runtime

    # -----------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------

    def open(self):
        if self.ser is None:
            self.ser = serial.Serial(
                self.port, self.baudrate, timeout=0, write_timeout=0.05
            )
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"[SideSensorsV3] opened {self.port} @ {self.baudrate}")
        else:
            print("[SideSensorsV3] using shared serial object")

        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def set_front_enabled(self, enabled: bool):
        """Enable/disable the VL53L8CX 4x4 matrix poll (0x0003) at runtime.

        The matrix scan makes the STM32 briefly unresponsive on the shared UART,
        so keeping it OFF during motor start + the first sector keeps the bus
        light and stops the forward/stop frame being dropped mid-scan. The sides
        (0x0105) keep polling regardless."""
        self._front_enabled = enabled

    def pause(self):
        """Stop polling so the SHARED serial goes quiet. Use this around a
        fire-and-forget motor command (forward/stop): with no sensor request
        traffic the STM32's RX is idle, so the motor frame lands reliably
        instead of being dropped mid matrix-scan."""
        self._paused = True

    def resume(self):
        """Resume polling and drop any backlog so we parse fresh frames."""
        with self._data_lock:
            self.rx_buffer.clear()
        self._paused = False

    def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self._own_serial and self.ser and self.ser.is_open:
            self.ser.close()
            print("[SideSensorsV3] closed serial")

    # -----------------------------------------------------
    # Public read API
    # -----------------------------------------------------

    def snapshot(self) -> dict:
        with self._data_lock:
            left = self.left_mm
            rf = self.right_front_mm
            rr = self.right_rear_mm
            sides_last = self.last_sides_time
            front = self.front_mm
            skew = self.front_skew_mm
            row = list(self.front_row)
            cells = self.front_cells
            matrix = [list(r) for r in self.front_matrix]
            front_last = self.last_front_time

        now = time.monotonic()
        return {
            "left_mm": left,
            "right_front_mm": rf,
            "right_rear_mm": rr,
            "left_valid": self._valid(left),
            "right_front_valid": self._valid(rf),
            "right_rear_valid": self._valid(rr),
            # right_rear now arrives with right_front in the SAME 0x0105 frame,
            # so the sides timestamp is the freshness for all three. The keys
            # below keep the v3 names the main loop already uses.
            "lr_age_s": (now - sides_last) if sides_last > 0 else None,
            "rr_age_s": (now - sides_last) if sides_last > 0 else None,
            "lr_last_time": sides_last,
            "rr_last_time": sides_last,
            # Front VL53L8CX matrix.
            "front_mm": front,
            "front_valid": self._front_valid(front),
            "front_skew_mm": skew,
            "front_row": row,
            "front_cells": cells,
            "front_matrix": matrix,
            "front_age_s": (now - front_last) if front_last > 0 else None,
            "front_last_time": front_last,
        }

    def _valid(self, value: int) -> bool:
        return 0 < value <= self.max_valid_mm

    def _front_valid(self, value: int) -> bool:
        return 0 < value <= FRONT_MAX_MM

    # -----------------------------------------------------
    # Worker thread
    # -----------------------------------------------------

    def _worker_loop(self):
        while self._running:
            if self._paused:
                time.sleep(0.005)        # bus handed to the motor; send nothing
                continue
            now = time.monotonic()
            if now - self._last_sides_request >= self.sides_interval:
                self._send_command(CMD_READ_SIDES)
                self._last_sides_request = now

            if (self._front_enabled and self.front_interval is not None
                    and now - self._last_front_request >= self.front_interval):
                self._send_command(CMD_READ_FRONT)
                self._last_front_request = now

            self._read_available()
            self._process_buffer()
            time.sleep(0.001)

    def _checksum(self, message: bytes) -> int:
        c = 0
        for b in message:
            c ^= b
        return c

    def _send_command(self, command: int):
        if not self.ser or not self.ser.is_open:
            return
        cmd = command.to_bytes(2, "big")
        length = 7
        body = START_BYTE + length.to_bytes(2, "big") + cmd
        frame = body + bytes([self._checksum(body[1:])]) + END_BYTE
        try:
            with self.write_lock:
                self.ser.write(frame)
        except Exception as e:
            if self.debug:
                print(f"[SideSensorsV3] write error: {e}")

    def _read_available(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.rx_buffer.extend(self.ser.read(n))
        except Exception as e:
            if self.debug:
                print(f"[SideSensorsV3] read error: {e}")

    def _process_buffer(self):
        while True:
            if len(self.rx_buffer) < 7:
                return

            start = self.rx_buffer.find(START_BYTE)
            if start < 0:
                self.rx_buffer.clear()
                return
            if start > 0:
                del self.rx_buffer[:start]
            if len(self.rx_buffer) < 7:
                return

            length = int.from_bytes(self.rx_buffer[1:3], "big")
            if length < 7 or length > MAX_FRAME_LEN:
                del self.rx_buffer[0]
                continue
            if len(self.rx_buffer) < length:
                return

            frame = bytes(self.rx_buffer[:length])
            del self.rx_buffer[:length]

            if frame[-1:] != END_BYTE:
                continue
            if self._checksum(frame[1:-2]) != frame[-2]:
                if self.debug:
                    print("[SideSensorsV3] checksum failed")
                continue

            cmd = int.from_bytes(frame[3:5], "big")
            text = frame[5:-2].decode("utf-8", errors="ignore")
            if cmd == CMD_READ_SIDES:
                self._parse_sides(text)
            elif cmd == CMD_READ_FRONT:
                self._parse_front(text)
            # any other command (e.g. motor ack) is ignored

    def _parse_sides(self, text: str):
        m = _SIDES_RE.search(text)
        if not m:
            return
        right_front = int(m.group(1))
        right_rear = int(m.group(2))
        left = int(m.group(3))
        with self._data_lock:
            self.right_front_mm = right_front
            self.right_rear_mm = right_rear
            self.left_mm = left
            self.last_sides_time = time.monotonic()

    def _parse_front(self, text: str):
        distances, statuses = parse_front_matrix(text)
        if distances is None:
            return
        front_mm, skew, row, n_cells = front_scalars(distances, statuses)
        with self._data_lock:
            self.front_mm = front_mm
            self.front_skew_mm = skew
            self.front_row = row
            self.front_cells = n_cells
            self.front_matrix = distances
            self.last_front_time = time.monotonic()


# -----------------------------------------------------
# Standalone demo
# -----------------------------------------------------

def _demo():
    s = SideSensorsV3(port="/dev/ttyUSB0", debug=True)
    s.open()
    try:
        while True:
            snap = s.snapshot()
            skew = snap["front_skew_mm"]
            skew_s = f"{skew:+5.0f}" if skew is not None else "   -"
            print(f"LEFT={snap['left_mm']:>5}  RF={snap['right_front_mm']:>5}  "
                  f"RR={snap['right_rear_mm']:>5}  "
                  f"FRONT={snap['front_mm']:>5}({int(snap['front_valid'])}) "
                  f"skew={skew_s}  row={snap['front_row']}",
                  end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstop")
    finally:
        s.close()


if __name__ == "__main__":
    _demo()
