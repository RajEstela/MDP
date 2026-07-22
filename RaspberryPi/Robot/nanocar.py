import serial
import struct
import time
import threading

SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

HEADER = 0x5A
PACKET_LEN = 0x0C
ID = 0x01
FUNCTION = 0x01
RESERVE = 0x00
CRC_IGNORE = 0xFF

SEND_INTERVAL = 0.01        # How often to send packets to the car in seconds (heartbeat interval)
MANUAL_DRIVE_TIMEOUT = 0.5  # How long to hold a manual drive command before stopping (in seconds)

DRIVE_SPEED = 1000          # SPEED value for forward/backward movement (0-1000)
ROTATION_SPEED = 100        # SPEED value for rotation (0-1000)
FULL_LOCK = 1000            # Rotation angle value for full left/right lock (0-1000)

DEGREES_PER_STEP = 15.0     # How many degrees the car rotates in one step (tuned experimentally)
LEFT_STEP_DURATION = 0.4575 # How long it takes to rotate 15 degrees left at ROTATION_SPEED (tuned experimentally)
RIGHT_STEP_DURATION = 0.473 # How long it takes to rotate 15 degrees right at ROTATION_SPEED (tuned experimentally)

SEGMENT_DURATION = 0.37     # How long it takes to drive 30cm at DRIVE_SPEED (tuned experimentally)
FORWARD_OFFSET_ANGLE = 260  # Offset angle for driving straight forward 30cm (tuned experimentally)
FORWARD_OFFSET_ANGLE_10CM = 320 # Offset angle for driving straight forward 10cm (tuned experimentally)
FORWARD_OFFSET_ANGLE_20CM = 330 # Offset angle for driving straight forward 20cm (tuned experimentally)
FORWARD_SEGMENT_DURATION_10CM = 0.210 # Duration for driving straight forward 10cm (tuned experimentally)
FORWARD_SEGMENT_DURATION_20CM = 0.305 # Duration for driving straight forward 20cm (tuned experimentally)

BACKWARD_OFFSET_ANGLE = -365 # Offset angle for driving straight backward (tuned experimentally)
BACKWARD_OFFSET_ANGLE_10CM = -600 # Offset angle for driving straight backward 10cm (tuned experimentally)
BACKWARD_OFFSET_ANGLE_20CM = -450 # Offset angle for driving straight backward 20cm (tuned experimentally)
BACKWARD_SEGMENT_DURATION_10CM = 0.208 # Duration for driving straight backward 10cm (tuned experimentally)
BACKWARD_SEGMENT_DURATION_20CM = 0.303 # Duration for driving straight backward 20cm (tuned experimentally)


def _packet_value(value):
    return int(round(max(-1000, min(1000, value))))


def build_packet(x_speed, z_angle):
    x_speed = _packet_value(x_speed)
    z_angle = _packet_value(z_angle)
    x_b = struct.pack(">h", x_speed)
    y_b = struct.pack(">h", 0)
    z_b = struct.pack(">h", z_angle)
    return bytes([HEADER, PACKET_LEN, ID, FUNCTION]) + x_b + y_b + z_b + bytes([RESERVE, CRC_IGNORE])


class NanoCarLink:
    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self.port = port
        self.baud = baud
        self.ser = None
        self.current_x = 0
        self.current_z = 0
        self.running = False
        self._thread = None
        self._lock = threading.Lock()
        self._manual_drive_deadline = None

    def connect(self):
        print("[CONNECT] Opening serial port " + self.port + " at " + str(self.baud) + " baud...")
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(0.5)
        print("[CONNECT] Serial port opened successfully")

    def disconnect(self):
        if self.ser:
            print("[DISCONNECT] Stopping heartbeat and sending stop...")
            self.stop_heartbeat()
            self._send_now(0, 0)
            time.sleep(0.1)
            self.ser.close()
            print("[DISCONNECT] Serial port closed")

    def _send_now(self, x_speed, z_angle):
        packet = build_packet(x_speed, z_angle)
        self.ser.write(packet)

    def start_heartbeat(self):
        self.running = True
        def heartbeat():
            while self.running:
                timed_out = False
                with self._lock:
                    if (
                        self._manual_drive_deadline is not None
                        and time.monotonic() >= self._manual_drive_deadline
                    ):
                        self.current_x = 0
                        self.current_z = 0
                        self._manual_drive_deadline = None
                        timed_out = True
                    x = self.current_x
                    z = self.current_z
                if timed_out:
                    print("[MANUAL] Command timeout - stopped")
                self._send_now(x, z)
                time.sleep(SEND_INTERVAL)
        self._thread = threading.Thread(target=heartbeat, daemon=True)
        self._thread.start()
        print("[HEARTBEAT] Started - sending packets every " + str(SEND_INTERVAL) + "s")

    def stop_heartbeat(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=1)
        print("[HEARTBEAT] Stopped")

    def _set_command(self, x_speed, z_angle):
        with self._lock:
            self.current_x = x_speed
            self.current_z = z_angle
            self._manual_drive_deadline = None

    def set_manual_drive(self, x_speed, z_angle):
        x_speed = max(-1000, min(1000, int(x_speed)))
        z_angle = max(-1000, min(1000, int(z_angle)))
        print("[MANUAL] x=" + str(x_speed) + " z=" + str(z_angle))
        with self._lock:
            self.current_x = x_speed
            self.current_z = z_angle
            self._manual_drive_deadline = (
                time.monotonic() + MANUAL_DRIVE_TIMEOUT
                if x_speed != 0 or z_angle != 0
                else None
            )

    def drive(self, x_speed, z_angle, duration):
        print("[DRIVE] x=" + str(x_speed) + " z=" + str(z_angle) + " for " + str(round(duration, 3)) + "s")
        self._set_command(x_speed, z_angle)
        time.sleep(duration)
        print("[DRIVE] Done")

    def stop(self, hold=0.3):
        print("[STOP] Holding stop for " + str(hold) + "s")
        self._set_command(0, 0)
        time.sleep(hold)
        print("[STOP] Done")

    def establish_connection(self, attempts=10, interval=0.3):
        print("[CONN] Sending " + str(attempts) + " zero packets to establish connection...")
        for i in range(attempts):
            self._send_now(0, 0)
            print("[CONN] Packet " + str(i+1) + "/" + str(attempts) + " sent")
            time.sleep(interval)
        print("[CONN] Done. Watch for buzzer beep + fast green LED on robot.")
        print("[CONN] Starting heartbeat to keep connection alive...")
        self.start_heartbeat()
        print("[CONN] Ready.")

        
    def move_forward(self, cm, offset_angle=FORWARD_OFFSET_ANGLE, segment_duration=SEGMENT_DURATION):
        segment = 30.0  # drive in 30cm chunks
        full = int(cm / segment)
        remainder = cm % segment
        for i in range(full):
            self.drive(DRIVE_SPEED, offset_angle, segment_duration)
            self.stop(0.5)
        if remainder > 0:
            _segment_duration = segment_duration
            _offset_angle = offset_angle
            if offset_angle == FORWARD_OFFSET_ANGLE or segment_duration == SEGMENT_DURATION: 
                # if not overridden from tuning, use special values for 10cm and 20cm
                if remainder == 20:
                    _segment_duration = FORWARD_SEGMENT_DURATION_20CM
                    _offset_angle = FORWARD_OFFSET_ANGLE_20CM
                elif remainder == 10:
                    _segment_duration = FORWARD_SEGMENT_DURATION_10CM
                    _offset_angle = FORWARD_OFFSET_ANGLE_10CM

            self.drive(DRIVE_SPEED, _offset_angle, _segment_duration)
        self.stop()

    def move_backward(self, cm, offset_angle=BACKWARD_OFFSET_ANGLE, segment_duration=SEGMENT_DURATION):
        segment = 30.0  # drive in 30cm chunks
        full = int(cm / segment)
        remainder = cm % segment
        for i in range(full):
            self.drive(-DRIVE_SPEED, offset_angle, segment_duration)
            self.stop(0.5)
        if remainder > 0:                 
            _segment_duration = segment_duration
            _offset_angle = offset_angle
            if offset_angle == BACKWARD_OFFSET_ANGLE or segment_duration == SEGMENT_DURATION:
                # if not overridden from tuning, use special values for 10cm and 20cm
                if remainder == 20:
                    _segment_duration = BACKWARD_SEGMENT_DURATION_20CM
                    _offset_angle = BACKWARD_OFFSET_ANGLE_20CM
                elif remainder == 10:
                    _segment_duration = BACKWARD_SEGMENT_DURATION_10CM
                    _offset_angle = BACKWARD_OFFSET_ANGLE_10CM

            self.drive(-DRIVE_SPEED, _offset_angle, _segment_duration)
        self.stop()
            
    def rotate_left(self, degrees, rotation_speed=ROTATION_SPEED, step_duration=LEFT_STEP_DURATION):
        steps_needed = degrees / DEGREES_PER_STEP
        full_steps = int(steps_needed)
        remainder = steps_needed - full_steps
        print("[ROT_L] " + str(degrees) + " deg | steps=" + str(full_steps) + " remainder=" + str(round(remainder,3)))
        for i in range(full_steps):
            print("[ROT_L] Step " + str(i+1) + "/" + str(full_steps))
            self.drive(rotation_speed, FULL_LOCK, step_duration)
            self.drive(-rotation_speed, FULL_LOCK, step_duration)
            self.stop(0.5)
            
        if remainder > 0:
            partial = step_duration * remainder
            print("[ROT_L] Partial step: " + str(round(partial,3)) + "s")
            self.drive(rotation_speed, FULL_LOCK, partial)
            self.drive(-rotation_speed, FULL_LOCK, partial)
        self.stop()
        print("[ROT_L] Complete")
        
    def rotate_right(self, degrees, rotation_speed=ROTATION_SPEED, step_duration=RIGHT_STEP_DURATION):
        steps_needed = degrees / DEGREES_PER_STEP
        full_steps = int(steps_needed)
        remainder = steps_needed - full_steps
        print("[ROT_R] " + str(degrees) + " deg | steps=" + str(full_steps) + " remainder=" + str(round(remainder,3)))
        for i in range(full_steps):
            print("[ROT_R] Step " + str(i+1) + "/" + str(full_steps))
            self.drive(rotation_speed, -FULL_LOCK, step_duration)
            self.drive(-rotation_speed, -FULL_LOCK, step_duration)
            self.stop(0.5)

        if remainder > 0:
            partial = step_duration * remainder
            print("[ROT_R] Partial step: " + str(round(partial,3)) + "s")
            self.drive(rotation_speed, -FULL_LOCK, partial)
            self.drive(-rotation_speed, -FULL_LOCK, partial)
        self.stop()
        print("[ROT_R] Complete")
        

    def rotate_left_90(self):
        print("[ROT_L90] Calling rotate_left(90)")
        self.rotate_left(90)

    def rotate_right_90(self):
        print("[ROT_R90] Calling rotate_right(90)")
        self.rotate_right(90)


def main():
    print("[MAIN] Starting NanoCar link")
    car = NanoCarLink()
    car.connect()

    try:
        car.establish_connection()

        print("\nChoose test:")
        print("  1 = move_forward(10cm)")
        print("  2 = move_backward(10cm)")
        print("  3 = rotate_left_90")
        print("  4 = rotate_right_90")
        print("  5 = rotate_left(360 degrees)")
        print("  6 = rotate_right(360 degrees)")
        print("  q = quit")
    
        while True:
            choice = input("> ").strip()
            if choice == "1":
                car.move_forward(10)
            elif choice == "2":
                car.move_backward(10)
            elif choice == "3":
                car.rotate_left_90()
            elif choice == "4":
                car.rotate_right_90()
            elif choice == "5":
                car.rotate_left(360)
            elif choice == "6":
                car.rotate_right(360)
            elif choice.lower() == "q":
                break
            else:
                print("Unknown option")
 
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user")
    finally:
        car.disconnect()
        print("[MAIN] Exiting")
 
 
if __name__ == "__main__":
    main()
