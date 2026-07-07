import socket
import struct
import io
import time
from picamera import PiCamera # Using the Pi's built-in camera library!

# !!! IMPORTANT: CHANGE THIS TO YOUR MACBOOK'S WI-FI IP ADDRESS !!!
MACBOOK_IP = '192.168.3.13' 
PORT = 5005

# 1. Connect to the MacBook
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"Connecting to MacBook Brain at {MACBOOK_IP}:{PORT}...")
client_socket.connect((MACBOOK_IP, PORT))
print("Connected successfully!")

# 2. Initialize the built-in PiCamera
camera = PiCamera()
camera.resolution = (320, 240)
print("Warming up camera...")
time.sleep(2) # The physical camera sensor needs 2 seconds to adjust to light

# Create a temporary memory stream to hold our image data
stream = io.BytesIO()

print("Streaming video to MacBook...")

# 3. Capture continuous video frames directly to JPEG format
for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
    # Get the raw JPEG byte data
    data = stream.getvalue()
    
    # 4. Send the size of the frame, followed by the frame itself
    client_socket.sendall(struct.pack(">L", len(data)) + data)
    
    # 5. Wait for the MacBook to tell us what it saw!
    response = client_socket.recv(1024).decode('utf-8')
    
    # 6. Act on the feedback
    if response == "BULLSEYE":
        print("ACTION: BULLSEYE DETECTED - Tell STM to dodge!")
    elif response.startswith("TARGET"):
        print(f"ACTION: {response} FOUND - Update Android Tablet!")
        
    # Clear the memory stream ready for the next video frame
    stream.seek(0)
    stream.truncate()