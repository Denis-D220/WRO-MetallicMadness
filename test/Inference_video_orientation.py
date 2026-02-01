from ultralytics import YOLO
import cv2

model = YOLO("/home/daniel/WRO2026-MetallicMadness/Models/best_model_1.pt")
print("Model classes:", model.names)

pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=21/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=True sync=false"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Error: Could not open camera. Try: sudo systemctl restart nvargus-daemon")
    raise SystemExit(1)

CONF_TH = 0.35
CENTER_OFFSET_PX = 0

def zone_from_x(x, w):
    """Return 'LEFT', 'CENTER', or 'RIGHT' based on x coordinate."""
    left_border = w / 3.0
    right_border = 2.0 * w / 3.0
    if x < left_border:
        return "LEFT"
    elif x > right_border:
        return "RIGHT"
    return "CENTER"

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    cx_screen = (w // 2) + CENTER_OFFSET_PX
    cy_screen = h // 2

    # Draw 3-zone separators
    x1 = int(w / 3)
    x2 = int(2 * w / 3)
    cv2.line(frame, (x1, 0), (x1, h), (255, 255, 255), 1)
    cv2.line(frame, (x2, 0), (x2, h), (255, 255, 255), 1)

    # Optional: center mark
    cv2.circle(frame, (cx_screen, cy_screen), 5, (255, 255, 255), 2)

    results = model(frame, imgsz=640, conf=0.28, verbose=False)#model(frame, imgsz=640, conf=0.19, verbose=True)
    r0 = results[0]

    # Choose closest pillar (largest bbox area)
    chosen = None  # (name, conf, x1,y1,x2,y2, area, bx, by)
    if r0.boxes is not None and len(r0.boxes) > 0:
        for b in r0.boxes:
            conf = float(b.conf[0])
            if conf < CONF_TH:
                continue

            cls_id = int(b.cls[0])
            name = r0.names.get(cls_id, str(cls_id))

            x1b, y1b, x2b, y2b = map(float, b.xyxy[0])
            area = (x2b - x1b) * (y2b - y1b)
            bx = (x1b + x2b) / 2.0
            by = (y1b + y2b) / 2.0

            if (chosen is None) or (area > chosen[6]):
                chosen = (name, conf, x1b, y1b, x2b, y2b, area, bx, by)

    direction_text = ""  # show nothing when correct
    if chosen is not None:
        name, conf, x1b, y1b, x2b, y2b, area, bx, by = chosen

        # Determine 3-zone position
        zone = zone_from_x(bx, w)

        # Mark pillar center
        cv2.circle(frame, (int(bx), int(by)), 6, (0, 255, 255), -1)

        # --- YOUR 3-ZONE RULES ---
        # Green is correct ONLY in RIGHT zone.
        # Red is correct ONLY in LEFT zone.
        # CENTER is treated as wrong for both.
        lname = name.lower()

        if lname.startswith("green"):
            if zone != "RIGHT":
                direction_text = f"GREEN in {zone} -> GO RIGHT"
        elif lname.startswith("red"):
            if zone != "LEFT":
                direction_text = f"RED in {zone} -> GO LEFT"
        else:
            # Fallback if naming differs
            direction_text = f"{name} in {zone}"

    if direction_text:
        cv2.putText(
            frame,
            direction_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    cv2.imshow("YOLO - 3 Zone Logic", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
