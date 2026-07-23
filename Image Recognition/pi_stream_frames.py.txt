import argparse
import base64
import socket
import sys
import threading

import cv2

DEFAULT_PORT = 5005
RESULT_RELAY_HOST = "127.0.0.1"
RESULT_RELAY_PORT = 5002

# Confirmed lab image-ID table (see pc_infer_server.py for the source of
# truth / class_id resolution - this copy is just for friendly console
# output on the RPi side).
ID_DESCRIPTIONS = {
    1: "Up arrow", 2: "down arrow", 3: "right arrow", 4: "left arrow",
    5: "Go", 6: "six", 7: "seven", 8: "eight", 9: "nine", 10: "zero",
    11: "Alphabet V", 12: "Alphabet W", 13: "Alphabet X", 14: "Alphabet Y",
    15: "Alphabet Z", 100: "Bullseye",
}


def _open_camera(brightness: float = None, exposure: float = None) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: could not open camera.")
        sys.exit(1)

    # Best-effort - the RPi Camera Module's V4L2 driver (bcm2835-v4l2) has
    # inconsistent support for these properties depending on OS/driver
    # version. If a value doesn't seem to take effect, use v4l2-ctl
    # directly instead (see note_v2.txt), it's the more reliable lever.
    if exposure is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = manual mode (V4L2 convention via OpenCV)
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    if brightness is not None:
        cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)

    if brightness is not None or exposure is not None:
        print(f"Camera properties requested: brightness={brightness}, exposure={exposure}")
        print(f"Camera properties actually read back: brightness={cap.get(cv2.CAP_PROP_BRIGHTNESS)}, "
              f"exposure={cap.get(cv2.CAP_PROP_EXPOSURE)}")

    return cap


def run_local_preview(brightness: float, exposure: float):
    cap = _open_camera(brightness, exposure)

    print("Local preview mode (no network). Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to grab frame.")
            break

        cv2.imshow("RPi camera - local preview", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def run_snapshot(path: str, brightness: float, exposure: float):
    # No display needed - useful over a plain SSH session with no monitor
    # and no X11 forwarding set up.
    cap = _open_camera(brightness, exposure)

    # Grab and discard a few frames first - some USB/CSI cameras return a
    # black/stale frame on the very first read while auto-exposure settles.
    for _ in range(5):
        cap.read()

    ok, frame = cap.read()
    cap.release()

    if not ok:
        print("Failed to grab frame.")
        sys.exit(1)

    cv2.imwrite(path, frame)
    print(f"Saved snapshot to {path} ({frame.shape[1]}x{frame.shape[0]}). "
          f"scp it back to your Mac to view it.")


def _listen_for_results(sock):
    reader = sock.makefile("rb")
    relay = None

    def connect_relay():
        try:
            connected = socket.create_connection((RESULT_RELAY_HOST, RESULT_RELAY_PORT), timeout=2.0)
            print(f"Forwarding detected IDs to {RESULT_RELAY_HOST}:{RESULT_RELAY_PORT}")
            return connected
        except OSError as exc:
            print(f"Could not connect to result relay at {RESULT_RELAY_HOST}:{RESULT_RELAY_PORT}: {exc}")
            return None

    relay = connect_relay()

    while True:
        line = reader.readline()
        if not line:
            print("Connection to PC closed.")
            break

        line = line.strip()
        if not line:
            continue

        parts = line.split(b",")
        target_id = None
        if len(parts) == 3 and parts[0] == b"IMGRES":
            try:
                target_id = int(parts[2])
            except ValueError:
                target_id = None

        if target_id is not None:
            description = ID_DESCRIPTIONS.get(target_id, "unknown")
            print(f"[PC] Detected: ID {target_id} ({description})")
            if relay is None:
                relay = connect_relay()
            if relay is not None:
                try:
                    relay.sendall((line.decode("utf-8", errors="replace") + "\n").encode("utf-8"))
                except OSError:
                    try:
                        relay.close()
                    except OSError:
                        pass
                    relay = None
        else:
            print(f"[PC] {line.decode('ascii', errors='replace')}")

    if relay is not None:
        try:
            relay.close()
        except OSError:
            pass


def run_stream(host: str, port: int, obstacle_id: str, jpeg_quality: int, resize_width: int,
               brightness: float, exposure: float):
    cap = _open_camera(brightness, exposure)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to PC at {host}:{port}...")
    sock.connect((host, port))
    print("Connected. Streaming live frames - press Ctrl+C to stop.")

    listener = threading.Thread(target=_listen_for_results, args=(sock,), daemon=True)
    listener.start()

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to grab frame.")
                break

            if resize_width > 0:
                h, w = frame.shape[:2]
                if w > resize_width:
                    scale = resize_width / w
                    frame = cv2.resize(frame, (resize_width, int(h * scale)))

            ok, buf = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                continue

            b64 = base64.b64encode(buf).decode("ascii")
            message = f"IMG,{obstacle_id},{b64}\n".encode("ascii")
            sock.sendall(message)
    except KeyboardInterrupt:
        print("Stopping stream.")
    finally:
        cap.release()
        sock.close()


def main():
    parser = argparse.ArgumentParser(
        description="RPi-side camera capture: local preview, or live stream to the PC for YOLO inference."
    )
    parser.add_argument("--host", help="PC IP address to stream frames to. Omit (or use --local-preview) for camera-only test.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--obstacle-id", default="0",
        help="Obstacle ID tag sent with each frame (placeholder until wired to the real algorithm/hub)."
    )
    parser.add_argument("--jpeg-quality", type=int, default=70)
    parser.add_argument(
        "--resize-width", type=int, default=416,
        help="Downscale frames to this width before sending, keeps the stream fast (0 to disable)."
    )
    parser.add_argument("--local-preview", action="store_true", help="Just show the camera locally, no network.")
    parser.add_argument(
        "--snapshot", metavar="PATH",
        help="No-display camera test: capture one frame and save it to PATH (e.g. snapshot.jpg), "
             "then scp it back to view. Use this over plain SSH with no monitor/X11 set up."
    )
    parser.add_argument(
        "--brightness", type=float, default=None,
        help="Best-effort camera brightness override (driver-dependent range, try values and see what sticks)."
    )
    parser.add_argument(
        "--exposure", type=float, default=None,
        help="Best-effort manual exposure override (driver-dependent range). Omit to leave auto-exposure on."
    )
    args = parser.parse_args()

    if args.snapshot:
        run_snapshot(args.snapshot, args.brightness, args.exposure)
    elif args.local_preview or not args.host:
        run_local_preview(args.brightness, args.exposure)
    else:
        run_stream(args.host, args.port, args.obstacle_id, args.jpeg_quality, args.resize_width,
                   args.brightness, args.exposure)


if __name__ == "__main__":
    main()