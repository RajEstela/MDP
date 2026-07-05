import socket
import threading
import bluetooth
import time
import json
from nanocar import NanoCarLink

# WiFi config
WIFI_HOST = "0.0.0.0"
WIFI_PORT = 5000

# Bluetooth config
BT_PORT = 1
BT_UUID = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

# Shared car instance
car = NanoCarLink()


def handle_command(raw, source="unknown"):
    """
    Parse and execute a JSON command of the form {"id": ..., "cmd": "..."}.

    Commands:
        FWxxx  - move forward xxx cm        e.g. FW020 = forward 20cm
        BWxxx  - move backward xxx cm       e.g. BW010 = backward 10cm
        RLxxx  - rotate left xxx degrees    e.g. RL090 = rotate left 90 deg
        RRxxx  - rotate right xxx degrees   e.g. RR045 = rotate right 45 deg
        STOP   - stop immediately

    Returns a JSON string {"id": ..., "status": <int>, "msg": "..."}.
    Status 200 = success, 400 = bad/unknown command, 500 = execution error.
    """
    req_id = None
    try:
        req = json.loads(raw)
        req_id = req.get("id")
        msg = str(req.get("cmd", "")).strip().upper()
    except (json.JSONDecodeError, AttributeError) as e:
        print("[CMD] Source=" + source + " invalid JSON: " + raw)
        return json.dumps({"id": req_id, "status": 400, "msg": "Invalid JSON: " + str(e)})

    print("[CMD] Source=" + source + " id=" + str(req_id) + " msg=" + msg)

    try:
        if msg == "STOP":
            car.stop()
            status, resp_msg = 200, "STOP OK"

        elif msg.startswith("FW") and len(msg) >= 4:
            cm = float(msg[2:])
            car.move_forward(cm)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("BW") and len(msg) >= 4:
            cm = float(msg[2:])
            car.move_backward(cm)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("RL") and len(msg) >= 4:
            deg = float(msg[2:])
            car.rotate_left(deg)
            status, resp_msg = 200, msg + " OK"

        elif msg.startswith("RR") and len(msg) >= 4:
            deg = float(msg[2:])
            car.rotate_right(deg)
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
    try:
        while True:
            data = conn.recv(1024).decode("utf-8").strip()
            if not data:
                break
            result = handle_command(data, source="bt:" + str(addr))
            conn.send(result.encode("utf-8"))
    except Exception as e:
        print("[BT] Client error: " + str(e))
    finally:
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


def main():
    print("[MAIN] Starting NanoCar server")
    print("[MAIN] Commands: FWxxx BWxxx RLxxx RRxxx STOP")

    car.connect()
    car.establish_connection()

    wifi_thread = threading.Thread(target=start_wifi_server, daemon=True)
    wifi_thread.start()

    bt_thread = threading.Thread(target=start_bt_server, daemon=True)
    bt_thread.start()

    print("[MAIN] Both servers running")
    print("[MAIN] WiFi port: " + str(WIFI_PORT))
    print("[MAIN] BT channel: " + str(BT_PORT))
    print("[MAIN] Press Ctrl+C to stop")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down")
        car.disconnect()


if __name__ == "__main__":
    main()
