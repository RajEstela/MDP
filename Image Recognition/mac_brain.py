import socket
import cv2
import numpy as np
import struct
from ultralytics import YOLO

# 1. Load the AI Brain
model = YOLO("best.pt")

# 2. Setup the Wi-Fi Server (Listens on Port 5000)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# '0.0.0.0' means it listens to any device connected to the Mac's network
server_socket.bind(('0.0.0.0', 5000))
server_socket.listen(1)

print("MacBook Brain is active! Waiting for Raspberry Pi to connect...")
conn, addr = server_socket.accept()
print(f"Connection established with Raspberry Pi at {addr}")

# Variables to handle the incoming data stream
data = b""
payload_size = struct.calcsize(">L")

while True:
    # 3. Receive the frame size from the Pi
    while len(data) < payload_size:
        packet = conn.recv(4096)
        if not packet: break
        data += packet
    if not data: break

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]

    # 4. Receive the actual video frame data
    while len(data) < msg_size:
        data += conn.recv(4096)
    frame_data = data[:msg_size]
    data = data[msg_size:]

    # 5. Decode the image and run YOLO
    np_data = np.frombuffer(frame_data, dtype=np.uint8)
    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
    
    results = model(frame, conf=0.35, iou=0.4, verbose=False)
    
    # 6. Decide what command to send back
    command = "NONE"
    for r in results:
        for box in r.boxes:
            class_id = int(box.cls[0])
            if class_id == 100:
                command = "BULLSEYE"
            else:
                command = f"TARGET,{class_id}"
            break # We just grab the first valid detection

    # 7. Send the text command back down the Wi-Fi pipe to the Pi
    conn.sendall(command.encode('utf-8'))

    # 8. Show the annotated video on your MacBook screen
    annotated_frame = results[0].plot()
    cv2.imshow("MacBook Remote Brain - Live Feed", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

conn.close()
cv2.destroyAllWindows()