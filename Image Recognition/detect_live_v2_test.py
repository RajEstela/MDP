import cv2
from ultralytics import YOLO

# 1. Load your custom trained AI brain
model = YOLO("best.pt")

# 2. Initialize the camera feed via OpenCV
cap = cv2.VideoCapture(0)

# Check if the camera opened correctly
if not cap.isOpened():
    print("Error: Could not open video feed.")
    exit()

print("AI Recognition active! Press 'q' to quit.")

while True:
    # Capture frame-by-frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # 3. Run the YOLO model on the current live frame
    results = model(frame, conf=0.12, iou=0.4, verbose=False)

    # --- A.5 FEEDBACK LOGIC ---
    for r in results:
        boxes = r.boxes
        for box in boxes:
            class_id = int(box.cls[0])
            
            if class_id == 100:
                print("Bullseye!")
                # LATER: Send signal to Algorithm/STM here
            else:
                print(f"valid image, Image_ID {class_id}")
                # LATER: Send signal to Android Tablet here
    # --------------------------

    # 4. Use OpenCV to draw the bounding boxes and text
    annotated_frame = results[0].plot()

    # 5. Display the live video window
    cv2.imshow("MDP Robot Car - Live Image Recognition", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()