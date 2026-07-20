"""Verification run for simulator/main.py --live mode: spins up throwaway
local mock arena (:5001) and car (:5000) TCP servers on loopback, then runs
the live pygame loop end-to-end (including --execute) for a bounded number
of frames, checking that the algorithm_status sequence reaches "completed".

Run from Algorithm/: python verify_live_sim.py
"""
import json
import os
import socket
import threading
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

from simulator.main import _run_live

ARENA_SNAPSHOT = {
    "version": 1, "type": "arena", "revision": 1,
    "grid": {"columns": 20, "rows": 20, "cellCm": 10, "origin": "bottom-left"},
    "robot": {"x": 1, "y": 1, "direction": "N"},
    "obstacles": [
        {"id": "B1", "x": 5, "y": 16, "direction": "S", "targetId": None},
        {"id": "B2", "x": 11, "y": 14, "direction": "W", "targetId": None},
    ],
}

received_statuses: list[dict] = []
sent_commands: list[str] = []


def _run_mock_arena_server() -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 5001))
    server.listen(1)
    conn, _ = server.accept()
    conn.sendall((json.dumps(ARENA_SNAPSHOT) + "\n").encode())
    while True:
        try:
            data = conn.recv(4096)
        except OSError:
            break
        if not data:
            break
        for line in data.decode().splitlines():
            if line.strip():
                status = json.loads(line)
                received_statuses.append(status)
                print("[mock-arena] status:", status)


def _run_mock_car_server() -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 5000))
    server.listen(1)
    conn, _ = server.accept()
    stream = conn.makefile("r", encoding="utf-8", newline="\n")
    for line in stream:
        if not line.strip():
            continue
        req = json.loads(line)
        sent_commands.append(req["cmd"])
        resp = json.dumps({"id": req["id"], "status": 200, "msg": req["cmd"] + " OK"})
        conn.sendall((resp + "\n").encode())


def main() -> None:
    threading.Thread(target=_run_mock_arena_server, daemon=True).start()
    threading.Thread(target=_run_mock_car_server, daemon=True).start()
    time.sleep(0.3)

    _run_live(host="127.0.0.1", execute=True, max_frames=1800)

    states = [s.get("state") for s in received_statuses]
    print(f"\nStatus sequence: {states}")
    print(f"Commands sent to mock car: {len(sent_commands)}")
    ok = states[:3] == ["planning", "route_ready", "running"] and states[-1] == "completed"
    ok = ok and len(sent_commands) > 0
    print("PASS" if ok else "FAIL")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
