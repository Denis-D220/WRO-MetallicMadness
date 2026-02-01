from ultralytics import YOLO
import cv2
import time

model = YOLO("/home/daniel/WRO2026-MetallicMadness/Models/best_model_1.pt")
print("Model classes:", model.names)

pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=True"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Error: Could not open camera. Try: sudo systemctl restart nvargus-daemon")
    raise SystemExit(1)

# ---------------- CONFIG ----------------
CONF_TH = 0.35

# Define which zones count as "recognition range"
# Example: count when pillar is in CENTER (common if you want "it entered my lane")
RECOGNITION_ZONES = {"CENTER"}        # or {"LEFT","CENTER","RIGHT"} to count any detection
# Optional: also require a minimum bbox area (proxy for "close enough")
MIN_AREA = 0                          # set >0 to require closeness, e.g. 2500

# Debounce settings (reduce flicker double-counting)
ENTER_FRAMES = 3                      # need 3 consecutive frames IN before counting entered
EXIT_FRAMES = 6                       # need 6 consecutive frames OUT before counting exited

# ---------------- HELPERS ----------------
def zone_from_x(x, w):
    left_border = w / 3.0
    right_border = 2.0 * w / 3.0
    if x < left_border:
        return "LEFT"
    elif x > right_border:
        return "RIGHT"
    return "CENTER"

def is_pillar_name(name: str):
    lname = name.lower()
    if lname.startswith("green"):
        return "GREEN"
    if lname.startswith("red"):
        return "RED"
    return None

# ---------------- STATE (per color) ----------------
state = {
    "GREEN": {"in": False, "in_streak": 0, "out_streak": 0, "entered": 0, "exited": 0},
    "RED":   {"in": False, "in_streak": 0, "out_streak": 0, "entered": 0, "exited": 0},
}

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    # Draw zone separators for debugging
    x1 = int(w / 3)
    x2 = int(2 * w / 3)
    cv2.line(frame, (x1, 0), (x1, h), (255, 255, 255), 1)
    cv2.line(frame, (x2, 0), (x2, h), (255, 255, 255), 1)

    # Run YOLO
    results = model(frame, imgsz=640, verbose=False)
    r0 = results[0]

    # Track "seen in recognition zone this frame" for each color
    seen_in_zone = {"GREEN": False, "RED": False}

    if r0.boxes is not None and len(r0.boxes) > 0:
        for b in r0.boxes:
            conf = float(b.conf[0])
            if conf < CONF_TH:
                continue

            cls_id = int(b.cls[0])
            name = r0.names.get(cls_id, str(cls_id))
            color = is_pillar_name(name)
            if color is None:
                continue

            x1b, y1b, x2b, y2b = map(float, b.xyxy[0])
            area = (x2b - x1b) * (y2b - y1b)
            if area < MIN_AREA:
                continue

            bx = (x1b + x2b) / 2.0
            by = (y1b + y2b) / 2.0
            zone = zone_from_x(bx, w)

            # Mark detection center for debugging
            cv2.circle(frame, (int(bx), int(by)), 5, (0, 255, 255), -1)
            cv2.putText(frame, f"{color} {zone}", (int(x1b), int(y1b) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Recognition rule: only count this frame if detection is inside allowed zones
            if zone in RECOGNITION_ZONES:
                seen_in_zone[color] = True

    # Update state machines (enter/exit counting) for GREEN and RED
    for color in ("GREEN", "RED"):
        s = state[color]
        if seen_in_zone[color]:
            s["in_streak"] += 1
            s["out_streak"] = 0
            # OUT -> IN transition (debounced)
            if (not s["in"]) and (s["in_streak"] >= ENTER_FRAMES):
                s["in"] = True
                s["entered"] += 1
        else:
            s["out_streak"] += 1
            s["in_streak"] = 0
            # IN -> OUT transition (debounced)
            if s["in"] and (s["out_streak"] >= EXIT_FRAMES):
                s["in"] = False
                s["exited"] += 1

    # On-screen counters
    cv2.putText(frame,
                f"GREEN entered={state['GREEN']['entered']} exited={state['GREEN']['exited']} in={state['GREEN']['in']}",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame,
                f"RED   entered={state['RED']['entered']} exited={state['RED']['exited']} in={state['RED']['in']}",
                (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("YOLO - Enter/Exit Counter", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()