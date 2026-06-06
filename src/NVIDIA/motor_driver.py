import serial
import time
from typing import Optional, Dict, Any


START_BYTE = b"$"
END_BYTE = b"\n"

MAX_FRAME_LEN = 2048


# ============================================================
# DD-UART Motor Commands
# ============================================================

CMD_FORWARD_CONTINUOUS = 0x0004
CMD_REVERSE_CONTINUOUS = 0x0005

CMD_MOVE_FORWARD_DEGREE = 0x0101
CMD_MOVE_REVERSE_DEGREE = 0x0102
CMD_STOP = 0x0103


# ============================================================
# Low-level DD-UART protocol helpers
# ============================================================

def calculate_checksum(message: bytes) -> int:
    """
    Calculate XOR checksum.
    """
    checksum = 0
    for byte in message:
        checksum ^= byte
    return checksum


def create_message(command: int, data: bytes = b"") -> bytes:
    """
    Create DD-UART frame:

    [START '$'][LENGTH 2B][COMMAND 2B][DATA][CHECKSUM][END '\n']

    LENGTH includes all bytes in the complete frame.
    """
    if data is None:
        data = b""

    command_bytes = command.to_bytes(2, "big")
    length = 7 + len(data)
    length_bytes = length.to_bytes(2, "big")

    message_without_checksum = START_BYTE + length_bytes + command_bytes + data

    # Checksum excludes START byte and excludes checksum/end.
    checksum = calculate_checksum(message_without_checksum[1:])

    return message_without_checksum + bytes([checksum]) + END_BYTE


def validate_message(frame: bytes) -> bool:
    """
    Validate complete DD-UART frame.
    """
    if not frame:
        return False

    if len(frame) < 7:
        return False

    if frame[0:1] != START_BYTE:
        return False

    if frame[-1:] != END_BYTE:
        return False

    length = int.from_bytes(frame[1:3], "big")
    if len(frame) != length:
        return False

    checksum = calculate_checksum(frame[1:-2])
    if checksum != frame[-2]:
        return False

    return True


def read_exact_until_deadline(
    ser: serial.Serial,
    size: int,
    deadline: float,
) -> Optional[bytes]:
    """
    Read exactly size bytes before deadline.
    """
    data = bytearray()

    while len(data) < size and time.monotonic() < deadline:
        chunk = ser.read(size - len(data))

        if chunk:
            data.extend(chunk)
        else:
            time.sleep(0.002)

    if len(data) != size:
        return None

    return bytes(data)


def read_frame(
    ser: serial.Serial,
    timeout_s: float = 2.0,
    debug_noise: bool = False,
) -> Optional[Dict[str, Any]]:
    """
    Robust DD-UART frame reader.

    It does not use read_until('\\n') for the full frame because payloads
    could theoretically contain newline bytes.

    Steps:
      1. Find START byte '$'
      2. Read 2-byte LENGTH
      3. Read exact remaining frame bytes
      4. Validate checksum and END byte
    """
    deadline = time.monotonic() + timeout_s
    skipped = bytearray()

    while time.monotonic() < deadline:
        byte = ser.read(1)

        if not byte:
            time.sleep(0.002)
            continue

        if byte == START_BYTE:
            break

        skipped.extend(byte)
    else:
        if debug_noise and skipped:
            print(f"Skipped non-frame bytes: {bytes(skipped)!r}")
        return None

    if debug_noise and skipped:
        print(f"Skipped non-frame bytes: {bytes(skipped)!r}")

    length_bytes = read_exact_until_deadline(ser, 2, deadline)
    if length_bytes is None:
        return None

    length = int.from_bytes(length_bytes, "big")

    if length < 7 or length > MAX_FRAME_LEN:
        if debug_noise:
            print(f"Invalid frame length: {length}")
        return None

    remaining_len = length - 3
    remaining = read_exact_until_deadline(ser, remaining_len, deadline)

    if remaining is None:
        return None

    frame = START_BYTE + length_bytes + remaining

    if not validate_message(frame):
        if debug_noise:
            print(f"Invalid frame received: {frame!r}")
        return None

    return {
        "length": length,
        "command": int.from_bytes(frame[3:5], "big"),
        "data": frame[5:-2],
        "raw": frame,
    }


def decode_response_data(data: bytes) -> str:
    """
    Decode response payload safely.
    """
    if not data:
        return "<empty>"

    return data.decode("utf-8", errors="replace")


def degrees_to_payload(degrees: int) -> bytes:
    """
    STM32 expects degrees as uint16 little-endian.
    """
    if not 0 <= degrees <= 65535:
        raise ValueError("degrees must be between 0 and 65535")

    return degrees.to_bytes(2, "little")

# ============================================================
# High-level importable motor controller
# ============================================================

class MotorSerial:
    """
    Importable motor controller for STM32 DD-UART motor commands.

    Example:
        from motor_serial import MotorSerial

        motor = MotorSerial("/dev/ttyUSB0")
        motor.open()
        motor.forward_degrees(180)
        motor.stop()
        motor.close()
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        serial_timeout: float = 0.05,
        write_timeout: float = 1.0,
        startup_delay_s: float = 0.3,
        debug: bool = False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.serial_timeout = serial_timeout
        self.write_timeout = write_timeout
        self.startup_delay_s = startup_delay_s
        self.debug = debug

        self.ser: Optional[serial.Serial] = None

    # --------------------------------------------------------
    # Connection management
    # --------------------------------------------------------

    def open(self) -> None:
        """
        Open serial port.
        """
        if self.ser is not None and self.ser.is_open:
            return

        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.serial_timeout,
            write_timeout=self.write_timeout,
        )

        time.sleep(self.startup_delay_s)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        if self.debug:
            print(f"Motor serial opened: {self.port} @ {self.baudrate}")

    def close(self) -> None:
        """
        Close serial port.
        """
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

            if self.debug:
                print("Motor serial closed.")

    def is_open(self) -> bool:
        """
        Return True if serial port is open.
        """
        return self.ser is not None and self.ser.is_open

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    # --------------------------------------------------------
    # Internal command handling
    # --------------------------------------------------------

    def _require_open(self) -> serial.Serial:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial port is not open. Call motor.open() first.")

        return self.ser

    def send_raw_command(
        self,
        command: int,
        data: bytes = b"",
        show_raw: bool = False,
    ) -> bytes:
        """
        Send raw DD-UART command frame.
        """
        ser = self._require_open()

        frame = create_message(command, data)
        ser.write(frame)
        ser.flush()

        if show_raw or self.debug:
            print(f"Sent: {frame!r}")

        return frame

    def read_matching_response(
        self,
        expected_command: int,
        timeout_s: float = 2.0,
    ) -> Optional[Dict[str, Any]]:
        """
        Read frames until expected command response arrives or timeout expires.
        """
        ser = self._require_open()

        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()

            response = read_frame(
                ser,
                timeout_s=max(0.05, remaining),
                debug_noise=self.debug,
            )

            if response is None:
                continue

            if response["command"] == expected_command:
                return response

            if self.debug:
                data_text = decode_response_data(response["data"])
                print(
                    f"Ignored frame for command 0x{response['command']:04X}: "
                    f"{data_text}"
                )

        return None

    def command(
        self,
        command: int,
        data: bytes = b"",
        timeout_s: float = 2.0,
        wait_response: bool = True,
        name: str = "",
    ) -> Optional[str]:
        """
        Generic command function.

        Returns:
            Response text if valid response is received.
            None if timeout or if wait_response=False.
        """
        label = name or f"0x{command:04X}"

        self.send_raw_command(command, data)

        if not wait_response:
            if self.debug:
                print(f"{label}: sent without waiting response")
            return None

        response = self.read_matching_response(
            expected_command=command,
            timeout_s=timeout_s,
        )

        if response is None:
            if self.debug:
                print(f"{label}: timeout/no valid response")
            return None

        text = decode_response_data(response["data"])

        if self.debug:
            print(f"{label}: {text}")

        return text

    # --------------------------------------------------------
    # Public motor commands
    # --------------------------------------------------------

    def forward_continuous(self, timeout_s: float = 2.0, wait_response: bool = True,) -> Optional[str]:
        """
        Command:
            0x0004 - FORWARD CONTINUOUS

        No degree payload.
        Motor should keep moving forward until STOP is sent.
        """
        return self.command(
            CMD_FORWARD_CONTINUOUS,
            data=b"",
            timeout_s=timeout_s,
            wait_response=wait_response,
            name="FORWARD CONTINUOUS",
        )

    def reverse_continuous(
        self,
        timeout_s: float = 2.0,
        wait_response: bool = True,
    ) -> Optional[str]:
        """
        Command:
            0x0005 - REVERSE CONTINUOUS

        No degree payload.
        Motor should keep moving reverse until STOP is sent.
        """
        return self.command(
            CMD_REVERSE_CONTINUOUS,
            data=b"",
            timeout_s=timeout_s,
            wait_response=wait_response,
            name="REVERSE CONTINUOUS",
        )

    def forward_degrees(
        self,
        degrees: int,
        timeout_s: float = 3.0,
        wait_response: bool = True,
    ) -> Optional[str]:
        """
        Command:
            0x0101 - MOVE FORWARD BY DEGREES
        """
        return self.command(
            CMD_MOVE_FORWARD_DEGREE,
            data=degrees_to_payload(degrees),
            timeout_s=timeout_s,
            wait_response=wait_response,
            name=f"FORWARD {degrees}°",
        )

    def reverse_degrees(
        self,
        degrees: int,
        timeout_s: float = 3.0,
        wait_response: bool = True,
    ) -> Optional[str]:
        """
        Command:
            0x0102 - MOVE REVERSE BY DEGREES
        """
        return self.command(
            CMD_MOVE_REVERSE_DEGREE,
            data=degrees_to_payload(degrees),
            timeout_s=timeout_s,
            wait_response=wait_response,
            name=f"REVERSE {degrees}°",
        )

    def stop(
        self,
        timeout_s: float = 2.0,
        wait_response: bool = True,
    ) -> Optional[str]:
        """
        Command:
            0x0103 - STOP
        """
        return self.command(
            CMD_STOP,
            data=b"",
            timeout_s=timeout_s,
            wait_response=wait_response,
            name="STOP",
        )