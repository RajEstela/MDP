"""Verification run for simulator/main.py --live mode: spins up throwaway
local mock arena (:5001) and car (:5000) TCP servers on loopback, then runs
the live pygame loop end-to-end (including --execute) for a bounded number
of frames, checking that the algorithm_status sequence reaches "completed"
TWICE in a row on the same TCP connection — i.e. that a second arena
snapshot arriving after the first run finishes triggers a full second
planning -> route_ready -> running -> completed cycle, with commands
actually sent to the mock car for both runs. A single-run check would not
exercise the state _start_new_run() resets between runs (see the
final-review fix for the stale-exec_progress bug this caught).

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
run1_command_count: int | None = None


def _run_mock_arena_server() -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 5001))
    server.listen(1)
    conn, _ = server.accept()
    conn.sendall((json.dumps(ARENA_SNAPSHOT) + "\n").encode())
    global run1_command_count
    completed_seen = 0
    while True:
        try:
            data = conn.recv(4096)
        except OSError:
            break
        if not data:
            break
        for line in data.decode().splitlines():
            if not line.strip():
                continue
            status = json.loads(line)
            received_statuses.append(status)
            print("[mock-arena] status:", status)
            if status.get("state") == "completed":
                completed_seen += 1
                if completed_seen == 1:
                    # Run 1 finished. All of its commands have already
                    # reached the mock car by now (the executor thread
                    # sends "completed" only after CarConnection is done).
                    # Snapshot the command count, then push a second arena
                    # snapshot (new revision) on the SAME connection to
                    # trigger a full second run — this is what would have
                    # caught the stale-exec_progress bug.
                    run1_command_count = len(sent_commands)
                    snapshot2 = dict(ARENA_SNAPSHOT, revision=2)
                    conn.sendall((json.dumps(snapshot2) + "\n").encode())


def _run_mock_car_server() -> None:
    # _run_car_executor opens a brand-new CarConnection (fresh TCP
    # connection) per run and closes it when that run's commands are done,
    # so this server must accept a new connection per run rather than
    # accept() once — otherwise run 2's commands have nowhere to land.
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', 5000))
    server.listen(1)
    while True:
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

    # Two full planning->route_ready->running->completed cycles need more
    # headroom than one; the original single-run budget was ~1800 frames.
    _run_live(host="127.0.0.1", execute=True, max_frames=3200)

    states = [s.get("state") for s in received_statuses]
    print(f"\nStatus sequence: {states}")
    print(f"Commands sent to mock car: {len(sent_commands)} (run 1: {run1_command_count})")

    completed_indices = [i for i, s in enumerate(states) if s == "completed"]
    ok = len(completed_indices) >= 2
    if ok:
        cycle1 = states[:completed_indices[0] + 1]
        cycle2 = states[completed_indices[0] + 1:completed_indices[1] + 1]
        ok = ok and cycle1[:3] == ["planning", "route_ready", "running"] and cycle1[-1] == "completed"
        ok = ok and cycle2[:3] == ["planning", "route_ready", "running"] and cycle2[-1] == "completed"

    # Run 2 must have actually driven the car: since both runs re-plan the
    # identical arena/route, run 2 should send the same number of commands
    # run 1 did, roughly doubling the total. If Finding 1's stale
    # exec_progress bug were still present, the main thread would jump
    # straight from "executing" to "done" for run 2 without waiting for its
    # executor thread, so run 2's commands would still be in flight (or a
    # third run could spawn a second concurrent executor) — either way this
    # total would NOT cleanly double.
    ok = ok and run1_command_count is not None and run1_command_count > 0
    ok = ok and len(sent_commands) == 2 * run1_command_count

    print("PASS" if ok else "FAIL")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
