from ultralytics import YOLO
import cv2
import time
import Jetson.GPIO as GPIO


from Pillar_recognition import PillarRecognition
from pillar_counter import PillarCounter
from servo_controller import ServoController
from motor_driver import MotorSerial



# =========================================================
# CONTROL / GUIDANCE CONFIGURATION
# =========================================================

# Which pillar color means the robot should pass on which side
TURN_LEFT_COLORS = {"green"}
TURN_RIGHT_COLORS = {"red"}

# Sign gain requested for testing
# Left  -> -1
# Right ->  1
K_LEFT = -1
K_RIGHT = 1

# Textbook PDI parameters:
# u(t) = K * ( e(t) + (1/Ti) * integral(e) + Td * de/dt )
PDI_K = 0.50
PDI_TI = 2.00
PDI_TD = 0.05

# Safety clamp for steering command, around servo middle
MAX_CONTROL_US = 250

# The screen is divided into 3 vertical sections
TARGET_SECTION_DIVISOR = 3.0

# Servo serial configuration
SERVO_PORT = "/dev/ttyACM0"
SERVO_BAUD = 9600


# =========================================================
# HELPER FUNCTIONS
# =========================================================

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def get_turn_direction_from_color(color: str):
    """
    Returns:
        "left", "right", or None
    """
    color = color.lower()
    if color in TURN_LEFT_COLORS:
        return "left"
    if color in TURN_RIGHT_COLORS:
        return "right"
    return None


def compute_error_from_pillar(chosen, frame_width):
    """
    Error definition:

    e = n - target

    where:
      n = length from the pillar to the screen edge,
          depending on the direction the robot should go
      target = width of the target screen section

    Returns:
      error, direction, k_gain, n_length, target_length
    """
    direction = get_turn_direction_from_color(chosen.color)
    if direction is None:
        return None, None, None, None, None

    target_length = frame_width / TARGET_SECTION_DIVISOR

    if direction == "left":
        # Distance from LEFT edge of the screen to LEFT side of the pillar
        n_length = float(chosen.x1)
        k_gain = K_LEFT
    else:
        # Distance from RIGHT side of pillar to RIGHT edge of the screen
        n_length = float(frame_width - chosen.x2)
        k_gain = K_RIGHT

    error = n_length - target_length
    return error, direction, k_gain, n_length, target_length


def draw_error_visual(frame, chosen, direction, n_length, target_length, error):
    """
    Draw:
      - target line
      - n line
      - e line
    """
    h, w = frame.shape[:2]
    y = int(chosen.cy)

    # BGR colors
    color_target = (0, 165, 255)   # orange
    color_n = (0, 0, 255)          # red
    color_e = (255, 0, 0)          # blue

    if direction == "left":
        # target line: from left edge to target section width
        x_target_end = int(target_length)
        cv2.line(frame, (0, y + 18), (x_target_end, y + 18), color_target, 3)

        # n line: from left edge to pillar left edge
        x_n_end = int(chosen.x1)
        cv2.line(frame, (0, y), (x_n_end, y), color_n, 3)

        # e line: between target and n
        cv2.line(frame, (x_target_end, y + 36), (x_n_end, y + 36), color_e, 3)

        cv2.putText(frame, "target", (10, y + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_target, 2)
        cv2.putText(frame, "n", (max(10, x_n_end // 2), y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_n, 2)

    else:
        # target line: from right target boundary to right edge
        x_target_start = int(w - target_length)
        cv2.line(frame, (x_target_start, y + 18), (w - 1, y + 18), color_target, 3)

        # n line: from pillar right edge to right edge
        x_n_start = int(chosen.x2)
        cv2.line(frame, (x_n_start, y), (w - 1, y), color_n, 3)

        # e line: between n and target
        cv2.line(frame, (x_n_start, y + 36), (x_target_start, y + 36), color_e, 3)

        cv2.putText(frame, "target", (x_target_start + 10, y + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_target, 2)
        cv2.putText(frame, "n", (min(w - 80, x_n_start + 10), y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_n, 2)

    cv2.putText(
        frame,
        f"e = {error:.1f}",
        (20, h - 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color_e,
        2
    )


def compute_pdi_control(error, prev_error, integral_error, prev_time):
    """
    Textbook PDI formula:

    u(t) = K * ( e(t) + (1/Ti)*integral(e) + Td*de/dt )

    Returns:
      raw_u, i_term, d_term, new_integral_error, new_prev_time
    """
    now = time.time()
    dt = now - prev_time if prev_time is not None else 0.0

    if dt > 0:
        integral_error += error * dt
        derivative_error = (error - prev_error) / dt
    else:
        derivative_error = 0.0

    if PDI_TI > 0:
        i_term = (1.0 / PDI_TI) * integral_error
    else:
        i_term = 0.0

    d_term = PDI_TD * derivative_error

    raw_u = PDI_K * (error + i_term + d_term)

    return raw_u, i_term, d_term, integral_error, now

# =========================================================
# BUTTON / RUN STATE CONFIGURATION
# =========================================================

button_pin = 7 # Physical pin 7, input

def set_pin():
    GPIO.setwarnings(False)
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)


    # Configure button input
    GPIO.setup(button_pin, GPIO.IN)
    print("InputPin: ",GPIO.input(button_pin))
    time.sleep(1)

def Check_button():
    if GPIO.input(button_pin) == GPIO.HIGH:
        print("Button Pressed!")
        time.sleep(0.05)  # debounce
        print("Cleaning Up...")
        #GPIO.cleanup()
        return True
    else:
        return False


BUTTON_DEBOUNCE_S = 0.30


def check_button_rising_edge(last_button_state, last_button_time):
    """
    Detects only the transition:
        not pressed -> pressed

    This prevents one long press from being counted many times.
    """
    now = time.time()
    current_button_state = bool(Check_button())

    button_event = False

    if current_button_state and not last_button_state:
        if now - last_button_time >= BUTTON_DEBOUNCE_S:
            button_event = True
            last_button_time = now

    return button_event, current_button_state, last_button_time

# =========================================================
# MAIN
# =========================================================

def main():

    set_pin()
    print("Button pin set")

    servo_enabled = False
    last_button_state = False
    last_button_time = 0.0

    # ---- Load model ----
    model = YOLO("/home/daniel/WRO2026-MetallicMadness/Models/best_model_1.pt")
    print("Model classes:", model.names)

    # ---- Jetson camera pipeline ----
    pipeline = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1, format=NV12, colorimetry=bt601 ! "
        "nvvidconv ! "
        "video/x-raw, width=1280, height=720, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Error: Could not open camera. Try: sudo systemctl restart nvargus-daemon")
        raise SystemExit(1)

    # ---- Initialize recognition modules immediately ----
    recog = PillarRecognition(conf_th=0.3)
    counter = PillarCounter(
        recognition_zones={"CENTER"},
        min_area=0,
        enter_frames=3,
        exit_frames=6
    )

    # ---- Servo setup ----
    servo = ServoController(port=SERVO_PORT, baud_rate=SERVO_BAUD)
    servo.connect()

    # IMPORTANT:
    # Do NOT center the servo here if you want zero movement before first button press.
    # servo.center_steering()

    # ----- Motor Setup -----
    motor = MotorSerial("/dev/ttyUSB0", debug=False)
    motor.open()

    # ---- PDI state variables ----
    prev_error = 0.0
    integral_error = 0.0
    prev_time = time.time()

    try:
        while cap.isOpened():

            # =====================================================
            # BUTTON STATE MACHINE
            # =====================================================
            button_event, last_button_state, last_button_time = check_button_rising_edge(
                last_button_state,
                last_button_time
            )

            if button_event:
                if not servo_enabled:
                    servo_enabled = True
                    print("First button press: servo control ENABLED")

                    # Optional: center only when system becomes active
                    servo.center_steering()

                    # Reset PDI when starting active control
                    prev_error = 0.0
                    integral_error = 0.0
                    prev_time = time.time()

                    motor.forward_continuous(wait_response=False)


                else:
                    print("Second button press: stopping program")
                    motor.stop(wait_response=False)
                    break

            ret, frame = cap.read()
            if not ret:
                break

            # ---- Optional color tuning ----
            b, g, r = cv2.split(frame)
            b = cv2.subtract(b, 18)
            r = cv2.add(r, 10)
            r = cv2.add(r, 10)
            g = cv2.add(g, 10)
            b = cv2.add(b, 10)
            frame = cv2.merge([b, g, r])

            h, w = frame.shape[:2]

            # ---- Draw 3-zone boundaries ----
            x1 = int(w / 3)
            x2 = int(2 * w / 3)
            cv2.line(frame, (x1, 0), (x1, h), (255, 255, 255), 1)
            cv2.line(frame, (x2, 0), (x2, h), (255, 255, 255), 1)

            # ---- YOLO inference runs immediately, even before first button press ----
            results = model(frame, imgsz=640, verbose=False)
            r0 = results[0]

            # ---- Parse and choose pillar ----
            dets = recog.parse_detections(r0, frame_w=w)
            chosen = recog.choose_closest(dets)
            msg = recog.guidance(chosen)

            # ---- Update counter ----
            counter.update(dets)

            # ---- Draw detections ----
            for d in dets:
                cv2.circle(frame, (int(d.cx), int(d.cy)), 5, (0, 255, 255), -1)
                cv2.putText(
                    frame,
                    f"{d.color} {d.zone} {d.conf:.2f}",
                    (int(d.x1), max(20, int(d.y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

            # =====================================================
            # GUIDANCE + PDI + SERVO STEERING
            # Recognition always runs.
            # Servo only moves after first button press.
            # =====================================================
            if chosen is not None:
                error, direction, k_gain, n_length, target_length = compute_error_from_pillar(chosen, w)

                if error is not None:
                    raw_u, i_term, d_term, integral_error, prev_time = compute_pdi_control(
                        error=error,
                        prev_error=prev_error,
                        integral_error=integral_error,
                        prev_time=prev_time
                    )

                    u = k_gain * raw_u
                    u = clamp(u, -MAX_CONTROL_US, MAX_CONTROL_US)

                    # Critical gate:
                    # Servo command is sent ONLY if the first button press happened.
                    if servo_enabled:
                        servo.steer_delta(u, channel=0)

                    prev_error = error

                    draw_error_visual(frame, chosen, direction, n_length, target_length, error)

                    cv2.putText(
                        frame,
                        f"DIR={direction}  Ksign={k_gain:+d}  n={n_length:.1f}  target={target_length:.1f}",
                        (20, h - 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2
                    )

                    cv2.putText(
                        frame,
                        f"u={u:.1f}  raw={raw_u:.1f}  e={error:.1f}  I={i_term:.1f}  D={d_term:.1f}",
                        (20, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2
                    )

                else:
                    if servo_enabled:
                        servo.center_steering()

                    prev_error = 0.0
                    integral_error = 0.0
                    prev_time = time.time()

            else:
                if servo_enabled:
                    servo.center_steering()

                prev_error = 0.0
                integral_error = 0.0
                prev_time = time.time()

            # ---- System status text ----
            if servo_enabled:
                status_text = "SERVO: ENABLED - press button again to stop"
            else:
                status_text = "SERVO: WAITING - press button to start"

            cv2.putText(
                frame,
                status_text,
                (20, h - 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )

            # ---- Guidance text ----
            if msg:
                cv2.putText(
                    frame,
                    msg,
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (255, 255, 255),
                    2
                )

            # ---- Counter info ----
            line1, line2 = counter.summary_lines()
            cv2.putText(frame, line1, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, line2, (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("Pillars: Recognition + Counting + PDI Steering", frame)

            # Keyboard quit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Keyboard q pressed: stopping program")
                motor.close()
                servo.close()
                break

    finally:
        # Same cleanup behavior as q
        if servo_enabled:
            servo.center_steering()

        servo.close()
        cap.release()
        cv2.destroyAllWindows()
        motor.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()