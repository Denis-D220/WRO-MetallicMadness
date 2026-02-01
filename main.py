from ultralytics import YOLO
import cv2

from Pillar_recognition import PillarRecognition
from pillar_counter import PillarCounter


def main():
    # ---- Load model (your uploaded one) ----
    model = YOLO("/home/daniel/WRO2026-MetallicMadness/Models/best_model_1.pt")
    print("Model classes:", model.names)

    # ---- Jetson camera pipeline ----
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

    # ---- Initialize modules ----
    recog = PillarRecognition(conf_th=0.35)

    # Count enter/exit when pillar enters the CENTER zone
    counter = PillarCounter(recognition_zones={"CENTER"}, min_area=0, enter_frames=3, exit_frames=6)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]

        # Draw 3-zone boundaries
        x1 = int(w / 3)
        x2 = int(2 * w / 3)
        cv2.line(frame, (x1, 0), (x1, h), (255, 255, 255), 1)
        cv2.line(frame, (x2, 0), (x2, h), (255, 255, 255), 1)

        # ---- YOLO inference ONCE ----
        results = model(frame, imgsz=640, verbose=False)
        r0 = results[0]

        # ---- Use pillar recognition module ----
        dets = recog.parse_detections(r0, frame_w=w)
        chosen = recog.choose_closest(dets)
        msg = recog.guidance(chosen)

        # ---- Use pillar counter module ----
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

        # ---- Draw guidance only when needed ----
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

        # ---- Draw counter info ----
        line1, line2 = counter.summary_lines()
        cv2.putText(frame, line1, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, line2, (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Pillars: Recognition + Counting", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()