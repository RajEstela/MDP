import socket
import threading
import bluetooth
import time
import json
import copy
import re
from nanocar import NanoCarLink

# WiFi config
WIFI_HOST = "0.0.0.0"
WIFI_PORT = 5000
ARENA_PORT = 5001

# Bluetooth config
BT_PORT = 1
BT_UUID = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
RFCOMM_DEVICE = "/dev/rfcomm0"

# Shared car instance
car = NanoCarLink()

# Shared arena state and connected relay clients
arena_lock = threading.Lock()
arena_clients_lock = threading.Lock()
bluetooth_clients_lock = threading.Lock()
latest_arena = None
arena_clients = {}
bluetooth_clients = {}

DIRECTIONS = {"N", "E", "S", "W"}
ADD_RE = re.compile(r"^ADD,([^,]+),\((\d+),(\d+)\)$", re.IGNORECASE)


def _json_line(payload):
    return json.dumps(payload, separators=(",", ":")) + "\n"


def _default_arena():
    return {
        "version": 1,
        "type": "arena",
        "revision": 0,
        "grid": {
            "columns": 20,
            "rows": 20,
            "cellCm": 10,
            "origin": "bottom-left",
        },
        "robot": {"x": 1, "y": 1, "direction": "N"},
        "obstacles": [],
    }


def _required_int(value, field):
    if isinstance(value, bool):
        raise ValueError(field + " must be an integer")
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ValueError(field + " must be an integer")


def normalize_arena(payload):
    if payload.get("type") != "arena":
        raise ValueError("type must be arena")

    grid = payload.get("grid") or {}
    columns = _required_int(grid.get("columns"), "grid.columns")
    rows = _required_int(grid.get("rows"), "grid.rows")
    cell_cm = _required_int(grid.get("cellCm"), "grid.cellCm")
    origin = str(grid.get("origin", ""))
    if columns <= 0 or rows <= 0 or cell_cm <= 0:
        raise ValueError("grid dimensions and cellCm must be positive")
    if origin != "bottom-left":
        raise ValueError("grid.origin must be bottom-left")

    robot = payload.get("robot") or {}
    robot_x = _required_int(robot.get("x"), "robot.x")
    robot_y = _required_int(robot.get("y"), "robot.y")
    robot_direction = str(robot.get("direction", "")).upper()
    if robot_x not in range(columns) or robot_y not in range(rows):
        raise ValueError("robot position is outside the arena")
    if robot_direction not in DIRECTIONS:
        raise ValueError("robot.direction must be N, E, S, or W")

    normalized_obstacles = []
    seen_ids = set()
    for index, obstacle in enumerate(payload.get("obstacles") or []):
        obstacle_id = str(obstacle.get("id", "")).strip().upper()
        x = _required_int(obstacle.get("x"), "obstacles[" + str(index) + "].x")
        y = _required_int(obstacle.get("y"), "obstacles[" + str(index) + "].y")
        direction = str(obstacle.get("direction", "")).upper()
        target_id = obstacle.get("targetId")
        if not obstacle_id or obstacle_id in seen_ids:
            raise ValueError("obstacle IDs must be non-empty and unique")
        if x not in range(columns) or y not in range(rows):
            raise ValueError("obstacle " + obstacle_id + " is outside the arena")
        if direction not in DIRECTIONS:
            raise ValueError("obstacle " + obstacle_id + " has an invalid direction")
        if target_id is not None:
            target_id = _required_int(target_id, "obstacle " + obstacle_id + " targetId")
        seen_ids.add(obstacle_id)
        normalized_obstacles.append({
            "id": obstacle_id,
            "x": x,
            "y": y,
            "direction": direction,
            "targetId": target_id,
        })

    return {
        "version": _required_int(payload.get("version", 1), "version"),
        "type": "arena",
        "revision": _required_int(payload.get("revision", 0), "revision"),
        "grid": {
            "columns": columns,
            "rows": rows,
            "cellCm": cell_cm,
            "origin": origin,
        },
        "robot": {
            "x": robot_x,
            "y": robot_y,
            "direction": robot_direction,
        },
        "obstacles": normalized_obstacles,
    }


def _send_socket_line(conn, lock, payload):
    data = payload if isinstance(payload, str) else json.dumps(payload, separators=(",", ":"))
    with lock:
        conn.sendall((data.rstrip("\r\n") + "\n").encode("utf-8"))


def broadcast_arena(arena):
    with arena_clients_lock:
        clients = list(arena_clients.items())
    for conn, write_lock in clients:
        try:
            _send_socket_line(conn, write_lock, arena)
        except Exception as e:
            print("[ARENA] Send failed: " + str(e))


def broadcast_to_bluetooth(payload):
    data = payload if isinstance(payload, str) else json.dumps(payload, separators=(",", ":"))
    with bluetooth_clients_lock:
        clients = list(bluetooth_clients.items())
    for conn, write_lock in clients:
        try:
            with write_lock:
                conn.send((data.rstrip("\r\n") + "\n").encode("utf-8"))
        except Exception as e:
            print("[BT] Status relay failed: " + str(e))


def store_and_broadcast_arena(payload, source):
    global latest_arena
    arena = normalize_arena(payload)
    with arena_lock:
        latest_arena = copy.deepcopy(arena)
    print(
        "[ARENA] Revision=" + str(arena["revision"])
        + " obstacles=" + str(len(arena["obstacles"]))
        + " source=" + source
    )
    broadcast_arena(arena)
    return {
        "type": "arena_ack",
        "revision": arena["revision"],
        "status": 200,
        "msg": "Arena forwarded to algorithm",
    }


def apply_arena_delta(raw):
    global latest_arena
    text = raw.strip()
    upper = text.upper()
    with arena_lock:
        arena = copy.deepcopy(latest_arena or _default_arena())
        obstacles = arena["obstacles"]

        match = ADD_RE.match(text)
        if match:
            obstacle_id, x, y = match.groups()
            obstacle_id = obstacle_id.upper()
            existing = next((item for item in obstacles if item["id"] == obstacle_id), None)
            if existing is None:
                existing = {
                    "id": obstacle_id,
                    "direction": "N",
                    "targetId": None,
                }
                obstacles.append(existing)
            existing["x"] = int(x)
            existing["y"] = int(y)
        elif upper.startswith("SUB,"):
            obstacle_id = upper.split(",", 1)[1]
            arena["obstacles"] = [item for item in obstacles if item["id"] != obstacle_id]
        elif upper.startswith("FACE,"):
            parts = upper.split(",")
            if len(parts) != 3 or parts[2] not in DIRECTIONS:
                raise ValueError("FACE requires obstacle ID and direction")
            existing = next((item for item in obstacles if item["id"] == parts[1]), None)
            if existing is None:
                raise ValueError("Unknown obstacle " + parts[1])
            existing["direction"] = parts[2]
        elif upper.startswith("TARGET,"):
            parts = upper.split(",")
            if len(parts) not in (3, 4):
                raise ValueError("TARGET requires obstacle ID, target ID, and optional direction")
            existing = next((item for item in obstacles if item["id"] == parts[1]), None)
            if existing is None:
                raise ValueError("Unknown obstacle " + parts[1])
            existing["targetId"] = int(parts[2])
            if len(parts) == 4:
                if parts[3] not in DIRECTIONS:
                    raise ValueError("Invalid target direction")
                existing["direction"] = parts[3]
        else:
            return None

        arena["revision"] = int(arena.get("revision", 0)) + 1
        arena = normalize_arena(arena)
        latest_arena = arena

    return {
        "type": "arena_edit_ack",
        "revision": arena["revision"],
        "status": 200,
        "msg": "Arena edit cached",
    }


def handle_message(raw, source="unknown"):
    text = raw.strip()
    try:
        if text.startswith("{"):
            payload = json.loads(text)
            if payload.get("type") == "arena":
                return json.dumps(store_and_broadcast_arena(payload, source), separators=(",", ":"))

        arena_result = apply_arena_delta(text)
        if arena_result is not None:
            return json.dumps(arena_result, separators=(",", ":"))
    except (ValueError, TypeError, json.JSONDecodeError) as e:
        return json.dumps({"status": 400, "msg": "Invalid arena data: " + str(e)})

    return handle_command(text, source=source)


def _parse_tune_values(command, expected_count):
    parts = command.split("/")
    if len(parts) != expected_count:
        raise ValueError("Tune command requires " + str(expected_count) + " values")
    try:
        return [float(part) for part in parts]
    except ValueError:
        raise ValueError("Tune command values must be numeric")


def handle_command(raw, source="unknown"):
    """
    Parse and execute either a raw command like FW010 or a JSON command of
    the form {"id": ..., "cmd": "FW010"}.

    Commands:
        FWxxx  - move forward xxx cm        e.g. FW020 = forward 20cm
        BWxxx  - move backward xxx cm       e.g. BW010 = backward 10cm
        RLxxx  - rotate left xxx degrees    e.g. RL090 = rotate left 90 deg
        RRxxx  - rotate right xxx degrees   e.g. RR045 = rotate right 45 deg
        TFW... - tune move forward          e.g. TFW010/190/98 (cm, offset angle, cm/sec)
        TBW... - tune move backward         e.g. TBW010/-220/98 (cm, offset angle, cm/sec)
        TRL... - tune rotate left           e.g. TRL090/100/0.455 (degrees, rotation speed, step duration)
        TRR... - tune rotate right          e.g. TRR090/100/0.4751 (degrees, rotation speed, step duration)
        DRIVE,x,z - continuous manual drive; x and z are -1000 to 1000
        STOP   - stop immediately

    Returns a JSON string {"id": ..., "status": <int>, "msg": "..."}.
    Status 200 = success, 400 = bad/unknown command, 500 = execution error.
    """
    req_id = None
    try:
        raw = raw.strip()
        if raw.startswith("{"):
            req = json.loads(raw)
            req_id = req.get("id")
            msg = str(req.get("cmd", "")).strip().upper()
        else:
            msg = raw.upper()
    except (json.JSONDecodeError, AttributeError) as e:
        print("[CMD] Source=" + source + " invalid JSON: " + raw)
        return json.dumps({"id": req_id, "status": 400, "msg": "Invalid JSON: " + str(e)})

    print("[CMD] Source=" + source + " id=" + str(req_id) + " msg=" + msg)

    try:
        if msg == "STOP":
            car.stop()
            status, resp_msg = 200, "STOP OK"

        elif msg.startswith("DRIVE,"):
            parts = msg.split(",")
            if len(parts) != 3:
                raise ValueError("DRIVE requires speed and steering")
            speed = max(-1000, min(1000, int(parts[1])))
            steering = max(-1000, min(1000, int(parts[2])))
            car.set_manual_drive(speed, steering)
            status, resp_msg = 200, "DRIVE," + str(speed) + "," + str(steering) + " OK"

        elif msg.startswith("FW") and len(msg) >= 4:
            cm = float(msg[2:])
            car.move_forward(cm)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("BW") and len(msg) >= 4:
            cm = float(msg[2:])
            car.move_backward(cm)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("TFW") and len(msg) >= 4:
            cm, offset_angle, cm_per_second = _parse_tune_values(msg[3:], 3)
            car.move_forward(cm, offset_angle=offset_angle, cm_per_second=cm_per_second)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("TBW") and len(msg) >= 4:
            cm, offset_angle, cm_per_second = _parse_tune_values(msg[3:], 3)
            car.move_backward(cm, offset_angle=offset_angle, cm_per_second=cm_per_second)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("RL") and len(msg) >= 4:
            deg = float(msg[2:])
            car.rotate_left(deg)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("RR") and len(msg) >= 4:
            deg = float(msg[2:])
            car.rotate_right(deg)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("TRL") and len(msg) >= 4:
            degrees, rotation_speed, step_duration = _parse_tune_values(msg[3:], 3)
            car.rotate_left(degrees, rotation_speed=rotation_speed, step_duration=step_duration)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("TRR") and len(msg) >= 4:
            degrees, rotation_speed, step_duration = _parse_tune_values(msg[3:], 3)
            car.rotate_right(degrees, rotation_speed=rotation_speed, step_duration=step_duration)
            status, resp_msg = 200, msg + " OK"

        else:
            print("[CMD] Unknown command: " + msg)
            status, resp_msg = 400, "Unknown command: " + msg

    except ValueError as e:
        print("[CMD] Parse error for '" + msg + "': " + str(e))
        status, resp_msg = 400, "Parse error: " + str(e)

    except Exception as e:
        print("[CMD] Execution error for '" + msg + "': " + str(e))
        status, resp_msg = 500, "Execution error: " + str(e)

    return json.dumps({"id": req_id, "status": status, "msg": resp_msg})


# WiFi client handler
def handle_wifi_client(conn, addr):
    print("[WIFI] Connected from " + str(addr))
    try:
        while True:
            data = conn.recv(1024).decode("utf-8").strip()
            if not data:
                break
            result = handle_command(data, source="wifi:" + str(addr))
            conn.sendall(result.encode("utf-8"))
    except Exception as e:
        print("[WIFI] Client error: " + str(e))
    finally:
        conn.close()
        print("[WIFI] Disconnected: " + str(addr))


def start_wifi_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((WIFI_HOST, WIFI_PORT))
    server.listen(5)
    print("[WIFI] Listening on port " + str(WIFI_PORT))
    while True:
        try:
            conn, addr = server.accept()
            t = threading.Thread(target=handle_wifi_client, args=(conn, addr), daemon=True)
            t.start()
        except Exception as e:
            print("[WIFI] Server error: " + str(e))


# Bluetooth client handler
def handle_bt_client(conn, addr):
    print("[BT] Connected from " + str(addr))
    pending = b""
    write_lock = threading.Lock()
    with bluetooth_clients_lock:
        bluetooth_clients[conn] = write_lock
    try:
        while True:
            chunk = conn.recv(1024)
            if not chunk:
                break
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                command = line.decode("utf-8", errors="replace").strip()
                if not command:
                    continue
                result = handle_message(command, source="bt:" + str(addr))
                with write_lock:
                    conn.send((result + "\n").encode("utf-8"))
    except Exception as e:
        print("[BT] Client error: " + str(e))
    finally:
        with bluetooth_clients_lock:
            bluetooth_clients.pop(conn, None)
        conn.close()
        print("[BT] Disconnected: " + str(addr))


def start_bt_server():
    server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server.bind(("", BT_PORT))
    server.listen(1)
    bluetooth.advertise_service(
        server,
        "NanoCar",
        service_id=BT_UUID,
        service_classes=[BT_UUID, bluetooth.SERIAL_PORT_CLASS],
        profiles=[bluetooth.SERIAL_PORT_PROFILE]
    )
    print("[BT] Listening on RFCOMM channel " + str(BT_PORT))
    while True:
        try:
            conn, addr = server.accept()
            t = threading.Thread(target=handle_bt_client, args=(conn, addr), daemon=True)
            t.start()
        except Exception as e:
            print("[BT] Server error: " + str(e))


def handle_rfcomm_device():
    """
    Read commands from an already-bound Linux RFCOMM device.

    Use this path when commands are visible with:
        cat /dev/rfcomm0

    In that setup Android is writing to the rfcomm device file, not necessarily
    to the PyBluez server socket above.
    """
    print("[RFCOMM] Watching " + RFCOMM_DEVICE)
    while True:
        try:
            with open(RFCOMM_DEVICE, "r+b", buffering=0) as dev:
                print("[RFCOMM] Opened " + RFCOMM_DEVICE)
                while True:
                    chunk = dev.read(1024)
                    if not chunk:
                        print("[RFCOMM] Device closed")
                        break

                    text = chunk.decode("utf-8", errors="replace")
                    commands = [part.strip() for part in text.replace("\r", "\n").split("\n") if part.strip()]
                    if not commands and text.strip():
                        commands = [text.strip()]

                    for command in commands:
                        result = handle_message(command, source="rfcomm:" + RFCOMM_DEVICE)
                        dev.write((result + "\n").encode("utf-8"))

        except FileNotFoundError:
            print("[RFCOMM] " + RFCOMM_DEVICE + " not found. Bind or connect Bluetooth first.")
            time.sleep(2)
        except PermissionError as e:
            print("[RFCOMM] Permission error opening " + RFCOMM_DEVICE + ": " + str(e))
            time.sleep(2)
        except Exception as e:
            print("[RFCOMM] Reader error: " + str(e))
            time.sleep(2)


def handle_arena_client(conn, addr):
    print("[ARENA] Algorithm connected from " + str(addr))
    pending = b""
    write_lock = threading.Lock()
    with arena_clients_lock:
        arena_clients[conn] = write_lock
    try:
        with arena_lock:
            arena = copy.deepcopy(latest_arena)
        if arena is not None:
            _send_socket_line(conn, write_lock, arena)

        broadcast_to_bluetooth({
            "type": "algorithm_status",
            "state": "connected",
            "message": "Laptop algorithm connected",
        })

        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                try:
                    payload = json.loads(text)
                    if payload.get("type") == "algorithm_status":
                        broadcast_to_bluetooth(payload)
                    elif payload.get("type") == "request_arena":
                        with arena_lock:
                            arena = copy.deepcopy(latest_arena)
                        if arena is not None:
                            _send_socket_line(conn, write_lock, arena)
                except (json.JSONDecodeError, AttributeError) as e:
                    print("[ARENA] Invalid laptop message: " + str(e))
    except Exception as e:
        print("[ARENA] Algorithm client error: " + str(e))
    finally:
        with arena_clients_lock:
            arena_clients.pop(conn, None)
        conn.close()
        broadcast_to_bluetooth({
            "type": "algorithm_status",
            "state": "disconnected",
            "message": "Laptop algorithm disconnected",
        })
        print("[ARENA] Algorithm disconnected: " + str(addr))


def start_arena_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((WIFI_HOST, ARENA_PORT))
    server.listen(5)
    print("[ARENA] Listening on port " + str(ARENA_PORT))
    while True:
        try:
            conn, addr = server.accept()
            thread = threading.Thread(target=handle_arena_client, args=(conn, addr), daemon=True)
            thread.start()
        except Exception as e:
            print("[ARENA] Server error: " + str(e))


def main():
    print("[MAIN] Starting NanoCar server")
    print("[MAIN] Commands: FWxxx BWxxx RLxxx RRxxx DRIVE,x,z STOP")

    car.connect()
    car.establish_connection()

    wifi_thread = threading.Thread(target=start_wifi_server, daemon=True)
    wifi_thread.start()

    arena_thread = threading.Thread(target=start_arena_server, daemon=True)
    arena_thread.start()

    bt_thread = threading.Thread(target=start_bt_server, daemon=True)
    bt_thread.start()

    rfcomm_thread = threading.Thread(target=handle_rfcomm_device, daemon=True)
    rfcomm_thread.start()

    print("[MAIN] Bluetooth, movement, and arena servers running")
    print("[MAIN] Movement WiFi port: " + str(WIFI_PORT))
    print("[MAIN] Arena relay port: " + str(ARENA_PORT))
    print("[MAIN] BT channel: " + str(BT_PORT))
    print("[MAIN] RFCOMM device: " + RFCOMM_DEVICE)
    print("[MAIN] Press Ctrl+C to stop")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down")
        car.disconnect()


if __name__ == "__main__":
    main()
