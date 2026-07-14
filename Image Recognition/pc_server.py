from flask import Flask, request, Response
from ultralytics import YOLO
import cv2
import numpy as np

app = Flask(__name__)
model = YOLO("best.pt")

latest_frame = None

@app.route("/detect", methods=["POST"])
def detect():
    img_bytes = request.data
    np_arr = np.frombuffer(img_bytes, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    results = model(frame, imgsz=320, conf=0.40, verbose=False)[0]
    annotated = results.plot()

    labels = []   # must be before the for loop

    for box in results.boxes:
        cls_id = int(box.cls[0])
        label = model.names[cls_id]
        conf = float(box.conf[0])
        labels.append(f"{label}:{conf:.2f}")

    detected_text = ",".join(labels) if labels else "None"
    print("Detected:", detected_text)

    cv2.imshow("Laptop - YOLO Result", annotated)
    cv2.waitKey(1)

    ok, jpg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])

    response = Response(jpg.tobytes(), mimetype="image/jpeg")
    response.headers["X-Detections"] = detected_text
    return response

if __name__ == "__main__":
    print("PC YOLO server running...")
    app.run(host="0.0.0.0", port=5000)