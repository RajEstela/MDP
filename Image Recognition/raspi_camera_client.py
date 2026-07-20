import cv2
import requests
import numpy as np
import time

PC_IP = "192.168.4.2"   # change this
SERVER_URL = f"http://{PC_IP}:5000/detect"

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to capture frame")
        break

    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])

    if not ok:
        continue

    try:
        response = requests.post(
            SERVER_URL,
            data=jpg.tobytes(),
            headers={"Content-Type": "image/jpeg"},
            timeout=3
        )

        img_array = np.frombuffer(response.content, np.uint8)
        result_frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        cv2.imshow("Raspberry Pi - YOLO Result From PC", result_frame)

    except Exception as e:
        print("Connection error:", e)
        time.sleep(1)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()