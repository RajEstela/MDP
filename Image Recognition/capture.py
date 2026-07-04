import cv2

# '0' is usually the default ID for your built-in MacBook webcam
cap = cv2.VideoCapture(0)

# Check if the webcam opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

img_counter = 0

print("Press 's' to save an image. Press 'q' to quit.")

while True:
    # Capture the video frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break

    # Display the resulting frame in a window
    cv2.imshow('MacBook Webcam', frame)

    # Wait for a key press (1 millisecond delay)
    key = cv2.waitKey(1) & 0xFF

    # If 's' is pressed, save the image
    if key == ord('s'):
        img_name = f"dataset_image_{img_counter}.png"
        cv2.imwrite(img_name, frame)
        print(f"{img_name} saved!")
        img_counter += 1
        
    # If 'q' is pressed, exit the loop
    elif key == ord('q'):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()