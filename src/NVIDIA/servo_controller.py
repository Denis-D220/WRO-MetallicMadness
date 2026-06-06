import serial
import time

# =========================================================
# SERVO / MAESTRO CONFIGURATION
# =========================================================
BAUD_RATE = 9600
PORT = "/dev/ttyACM0"

SERVO_CHANNELS = {
    0: {
        "name": "steering",
        "min": 1280,
        "max": 2900,
        "middle": 1590
    }
}


class ServoController:
    def __init__(self, port=PORT, baud_rate=BAUD_RATE):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
        time.sleep(0.3)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def set_servo_position(self, channel, pulse_width_us):
        """
        Set absolute servo position in microseconds.
        """
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial port is not open.")

        if channel not in SERVO_CHANNELS:
            raise ValueError(f"Channel {channel} not defined.")

        min_pw = SERVO_CHANNELS[channel]["min"]
        max_pw = SERVO_CHANNELS[channel]["max"]

        pulse_width_us = max(min_pw, min(pulse_width_us, max_pw))
        target = pulse_width_us * 4  # Maestro uses quarter-microseconds

        command = bytearray([
            0x84,
            channel,
            target & 0x7F,
            (target >> 7) & 0x7F
        ])
        self.ser.write(command)

    def steer_delta(self, control_signal_us, channel=0):
        """
        Move steering around the middle using the controller output u.
        u is added to the middle pulse.
        """
        middle = SERVO_CHANNELS[channel]["middle"]
        target_pw = int(middle + control_signal_us)
        self.set_servo_position(channel, target_pw)

    def center_steering(self, channel=0):
        middle = SERVO_CHANNELS[channel]["middle"]
        self.set_servo_position(channel, middle)
