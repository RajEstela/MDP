import cv2
from ultralytics import YOLO

# 1. Load your custom trained AI brain
# Make sure 'best.pt' is in the exact same folder as this script
model = YOLO("best.pt")

# 2. Initialize the camera feed via OpenCV
# '0' is usually the built-in webcam. 
# Change to '1' or '2' if you have an external USB PiCamera plugged in.
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
    # conf=0.5 means only show boxes if the AI is at least 50% confident
    # results = model(frame, conf=0.5)
    # Added verbose=False to stop the terminal spam!
    # added iou to increase sensitivity
    results = model(frame, conf=0.12, iou=0.4, verbose=False)

    # 4. Use OpenCV to draw the bounding boxes and text onto our live frame
    # 'plot()' automatically draws the rectangles and ID labels for us
    annotated_frame = results[0].plot()

    # 5. Display the live video window on your PC monitor
    cv2.imshow("MDP Robot Car - Live Image Recognition", annotated_frame)

    # Stop the program smoothly if the user hits the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up and close all windows when finished
cap.release()
cv2.destroyAllWindows()