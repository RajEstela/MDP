import argparse
import base64
import json
import select
import time
import socket
import sys

import cv2
import numpy as np
from ultralytics import YOLO

MODEL_PATH = "best.pt"

# Confirmed lab image-ID table: 1-9, 10='zero', 11='V'...15='Z', bullseye=100.
# The model's training classes are already named with these exact IDs as
# strings (see model.names), but YOLO orders class *indices* alphabetically,
# so index != ID. Always resolve through model.names, never compare raw
# class_id to an ID directly.
BULLSEYE_ID = 100
CAR_PORT = 5000
DEFAULT_SCAN_DEGREES = 30
SCAN_DIRECTIONS = ("RR", "RL", "RL")

ID_DESCRIPTIONS = {
    1: "Up arrow", 2: "down arrow", 3: "right arrow", 4: "left arrow",
    5: "Go", 6: "six", 7: "seven", 8: "eight", 9: "nine", 10: "zero",
    11: "Alphabet V", 12: "Alphabet W", 13: "Alphabet X", 14: "Alphabet Y",
    15: "Alphabet Z", BULLSEYE_ID: "Bullseye",
}


def resolve_target_id(model, class_id: int) -> int:
    name = model.names.get(class_id)
    try:
        return int(name)
    except (TypeError, ValueError):
        return -1


def send_car_command(host: str, port: int, command: str) -> None:
    """Send one movement command to the Raspberry Pi and verify its reply."""
    payload = json.dumps({"id": "bullseye-scan", "cmd": command}) + "\n"
    with socket.create_connection((host, port), timeout=10.0) as car:
        car.sendall(payload.encode("utf-8"))
        raw = car.recv(4096).decode("utf-8").strip()

    response = json.loads(raw)
    if response.get("status") != 200:
        raise RuntimeError(response.get("msg", "car rejected the command"))


def run_standalone(conf: float):
    model = YOLO(MODEL_PATH)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: could not open webcam.")
        sys.exit(1)

    print("Standalone mode (no network, local webcam). Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to grab frame.")
            break

        results = model(frame, conf=conf, iou=0.4, verbose=False)
        annotated = results[0].plot()
        cv2.imshow("PC Infer - standalone", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def run_serve(port: int, conf: float, car_port: int, scan_degrees: int):
    model = YOLO(MODEL_PATH)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    print(f"Waiting for the RPi to connect on port {port}...")

    conn, addr = server.accept()
    print(f"RPi connected from {addr}")
    reader = conn.makefile("rb")
    scan_step = 0

    print("Streaming live. Press 'q' in the video window to quit.")
    try:
        while True:
            line = reader.readline()
            if not line:
                print("RPi disconnected.")
                break

            # The RPi streams continuously and never waits for our reply, so
            # frames pile up in the socket buffer while we're busy (e.g.
            # blocked in send_car_command). Drain to the newest one so we
            # don't spend the next several iterations processing a stale
            # backlog instead of the live view.
            while select.select([conn], [], [], 0)[0]:
                newer = reader.readline()
                if not newer:
                    break
                line = newer

            line = line.strip()
            if not line:
                continue

            try:
                parts = line.split(b",", 2)
                if len(parts) != 3 or parts[0] != b"IMG":
                    print(f"Malformed line, ignoring: {line[:60]!r}")
                    continue
                obstacle_id = parts[1].decode("ascii")
                jpeg_bytes = base64.b64decode(parts[2])
            except Exception as exc:
                print(f"Malformed line, ignoring ({exc})")
                continue

            frame = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                print("Could not decode frame, ignoring.")
                continue

            results = model(frame, conf=conf, iou=0.4, verbose=False)
            annotated = results[0].plot()
            cv2.imshow("PC Infer - live from RPi", annotated)

            # Only report an ID when something was actually detected this
            # frame - the annotated window already only draws a box/label
            # when there's a real detection, so this keeps the wire message
            # and the console in sync with what's visibly on screen.
            best_conf = 0.0
            target_id = None
            for box in results[0].boxes:
                box_conf = float(box.conf[0])
                if box_conf > best_conf:
                    resolved = resolve_target_id(model, int(box.cls[0]))
                    if resolved != -1:
                        best_conf = box_conf
                        target_id = resolved

            if target_id is not None:
                description = ID_DESCRIPTIONS.get(target_id, "unknown")
                print(f"Detected: ID {target_id} ({description})  [{best_conf * 100:.0f}% confidence]")
                reply = f"IMGRES,{obstacle_id},{target_id}\n".encode("ascii")
                
                conn.sendall(reply)

                if target_id == BULLSEYE_ID:
                    direction = SCAN_DIRECTIONS[scan_step]
                    command = f"RR085"
                    print(f"Bullseye scan: sending {command} to the car...")
                    try:
                        send_car_command(addr[0], car_port, command)
                        send_car_command(addr[0], car_port, "FW050")
                        send_car_command(addr[0], car_port, "RL090")
                        send_car_command(addr[0], car_port, "FW070")
                        send_car_command(addr[0], car_port, "RL090")
                        time.sleep(2)
                    except (ConnectionError, OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
                        print(f"Bullseye scan movement failed: {exc}")
                    else:
                        scan_step = (scan_step + 1) % len(SCAN_DIRECTIONS)
                elif scan_step:
                    print("Non-bullseye object found; bullseye scan complete.")
                    scan_step = 0

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        conn.close()
        server.close()


def main():
    parser = argparse.ArgumentParser(
        description="PC-side YOLO inference: standalone webcam test, or a live server for the RPi's camera stream."
    )
    parser.add_argument(
        "--serve", action="store_true",
        help="Listen for the RPi's live frame stream instead of using the local webcam."
    )
    parser.add_argument("--port", type=int, default=5005, help="TCP port to listen on when --serve is used.")
    parser.add_argument(
        "--car-port", type=int, default=CAR_PORT,
        help="Raspberry Pi movement-server port used for bullseye scanning."
    )
    parser.add_argument(
        "--scan-degrees", type=int, default=DEFAULT_SCAN_DEGREES,
        help="Degrees to rotate for each right/left bullseye scan step."
    )
    parser.add_argument(
        "--conf", type=float, default=0.2,
        help="Confidence threshold for detections (0-1). Raise this if random objects (e.g. legs, chairs) "
             "get flagged as false positives; lower it if real targets aren't being picked up."
    )
    args = parser.parse_args()

    if not 1 <= args.scan_degrees <= 359:
        parser.error("--scan-degrees must be between 1 and 359")

    if args.serve:
        run_serve(args.port, args.conf, args.car_port, args.scan_degrees)
    else:
        run_standalone(args.conf)


if __name__ == "__main__":
    main()
