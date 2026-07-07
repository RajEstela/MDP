import socket
import cv2
import struct

# !!! IMPORTANT: CHANGE THIS TO YOUR MACBOOK'S WI-FI IP ADDRESS !!!
# MACBOOK_IP = '192.168.1.100' 
MACBOOK_IP = '10.91.218.239' 
PORT = 5005

# 1. Connect to the MacBook
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"Connecting to MacBook Brain at {MACBOOK_IP}...")
client_socket.connect((MACBOOK_IP, PORT))
print("Connected successfully!")

# 2. Initialize the PiCamera
cap = cv2.VideoCapture(0)
# We lower the resolution slightly to ensure the Wi-Fi transfer is lightning fast
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while True:
    ret, frame = cap.read()
    if not ret: 
        print("Failed to grab frame")
        break

    # 3. Compress the frame to JPEG so it flies over the Wi-Fi
    result, encoded_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    data = encoded_frame.tobytes()

    # 4. Send the size of the frame, followed by the frame itself
    client_socket.sendall(struct.pack(">L", len(data)) + data)

    # 5. Wait for the MacBook to tell us what it saw!
    response = client_socket.recv(1024).decode('utf-8')
    
    # 6. Act on the feedback (Your teammate links their code here)
    if response == "BULLSEYE":
        print("ACTION: BULLSEYE DETECTED - Tell STM to dodge!")
    elif response.startswith("TARGET"):
        print(f"ACTION: {response} FOUND - Update Android Tablet!")
    else:
        # "NONE" received, do nothing, just keep driving
        pass
        
cap.release()
client_socket.close()