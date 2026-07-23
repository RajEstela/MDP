import json

import pytest

from arena_feed import arena_to_obstacles, arena_to_robot_start, send_status
from simulator.types import Obstacle, RobotState


def _snapshot(**overrides) -> dict:
    base = {
        "version": 1,
        "type": "arena",
        "revision": 1,
        "grid": {"columns": 20, "rows": 20, "cellCm": 10, "origin": "bottom-left"},
        "robot": {"x": 1, "y": 1, "direction": "N"},
        "obstacles": [
            {"id": "B1", "x": 5, "y": 16, "direction": "S", "targetId": None},
        ],
    }
    base.update(overrides)
    return base


# ── arena_to_obstacles ───────────────────────────────────────────────────

def test_arena_to_obstacles_converts_grid_cells_to_cm():
    obstacles = arena_to_obstacles(_snapshot())
    assert obstacles == [Obstacle(x=50, y=160, face='S', id='B1')]


def test_arena_to_obstacles_rejects_wrong_type():
    with pytest.raises(ValueError):
        arena_to_obstacles(_snapshot(type="not-arena"))


def test_arena_to_obstacles_rejects_wrong_grid_size():
    snap = _snapshot()
    snap["grid"]["columns"] = 10
    with pytest.raises(ValueError):
        arena_to_obstacles(snap)


def test_arena_to_obstacles_rejects_wrong_cell_cm():
    snap = _snapshot()
    snap["grid"]["cellCm"] = 5
    with pytest.raises(ValueError):
        arena_to_obstacles(snap)


def test_arena_to_obstacles_rejects_non_bottom_left_origin():
    snap = _snapshot()
    snap["grid"]["origin"] = "top-left"
    with pytest.raises(ValueError):
        arena_to_obstacles(snap)


def test_arena_to_obstacles_rejects_invalid_direction():
    snap = _snapshot(obstacles=[{"id": "B1", "x": 5, "y": 16, "direction": "X"}])
    with pytest.raises(ValueError):
        arena_to_obstacles(snap)


def test_arena_to_obstacles_missing_id_yields_none():
    snap = _snapshot(obstacles=[{"x": 5, "y": 16, "direction": "S"}])
    obstacles = arena_to_obstacles(snap)
    assert obstacles[0].id is None


# ── arena_to_robot_start ─────────────────────────────────────────────────

def test_arena_to_robot_start_north():
    # Body CENTER is the same point regardless of facing direction (unlike
    # the front tip, which used to shift with heading) — only theta differs.
    start = arena_to_robot_start(_snapshot(robot={"x": 1, "y": 1, "direction": "N"}))
    assert start == RobotState(x=25.0, y=25.0, theta=90.0)


def test_arena_to_robot_start_east():
    start = arena_to_robot_start(_snapshot(robot={"x": 1, "y": 1, "direction": "E"}))
    assert start == RobotState(x=25.0, y=25.0, theta=0.0)


def test_arena_to_robot_start_south():
    start = arena_to_robot_start(_snapshot(robot={"x": 1, "y": 1, "direction": "S"}))
    assert start == RobotState(x=25.0, y=25.0, theta=270.0)


def test_arena_to_robot_start_west():
    start = arena_to_robot_start(_snapshot(robot={"x": 1, "y": 1, "direction": "W"}))
    assert start == RobotState(x=25.0, y=25.0, theta=180.0)


def test_arena_to_robot_start_rejects_invalid_direction():
    with pytest.raises(ValueError):
        arena_to_robot_start(_snapshot(robot={"x": 1, "y": 1, "direction": "NE"}))


def test_arena_to_robot_start_rejects_out_of_range_position():
    with pytest.raises(ValueError):
        arena_to_robot_start(_snapshot(robot={"x": 20, "y": 1, "direction": "N"}))


# ── send_status ───────────────────────────────────────────────────────────

class _FakeSocket:
    def __init__(self):
        self.sent = b""

    def sendall(self, data: bytes) -> None:
        self.sent += data


def test_send_status_writes_newline_terminated_json():
    sock = _FakeSocket()
    send_status(sock, 3, "route_ready", "Route calculated", commandCount=7)
    assert sock.sent.endswith(b"\n")
    payload = json.loads(sock.sent.decode())
    assert payload == {
        "type": "algorithm_status",
        "revision": 3,
        "state": "route_ready",
        "message": "Route calculated",
        "commandCount": 7,
    }


# ── listen ────────────────────────────────────────────────────────────────

import socket as socket_module
import threading
import time

from arena_feed import listen


def _free_port() -> int:
    s = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_STREAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


# Sockets deliberately kept open (never closed) so that a leaked background
# `listen()` thread parks in a blocking read forever instead of hitting EOF
# and spinning on the reconnect loop. Holding a reference here prevents
# garbage collection from closing them out from under the parked thread.
# See test_listen_dedupes_identical_consecutive_snapshots.
_KEEPALIVE_SOCKETS: list = []


def test_listen_invokes_callback_for_one_snapshot(monkeypatch):
    port = _free_port()
    monkeypatch.setattr('arena_feed.ARENA_PORT', port)

    server = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_STREAM)
    server.setsockopt(socket_module.SOL_SOCKET, socket_module.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', port))
    server.listen(1)

    line = json.dumps(_snapshot()) + "\n"

    def run_server():
        conn, _ = server.accept()
        conn.sendall(line.encode())
        time.sleep(0.2)
        conn.close()
        server.close()

    threading.Thread(target=run_server, daemon=True).start()

    received = []
    listen('127.0.0.1', lambda snap, sock: received.append(snap), once=True)

    assert len(received) == 1
    assert received[0]["revision"] == 1


def test_listen_dedupes_identical_consecutive_snapshots(monkeypatch):
    port = _free_port()
    monkeypatch.setattr('arena_feed.ARENA_PORT', port)

    server = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_STREAM)
    server.setsockopt(socket_module.SOL_SOCKET, socket_module.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', port))
    server.listen(1)

    snap1 = json.dumps(_snapshot()) + "\n"
    snap2 = json.dumps(_snapshot(revision=2)) + "\n"

    def run_server():
        conn, _ = server.accept()
        conn.sendall(snap1.encode())
        conn.sendall(snap1.encode())  # duplicate, should be skipped
        conn.sendall(snap2.encode())
        # Deliberately do NOT close the connection or the listening socket.
        # `listen()` runs with once=False in a daemon thread below; if the
        # server closed its end here, the client would hit EOF, fall into
        # the reconnect branch, sleep RECONNECT_DELAY_S, and then spin
        # `socket.create_connection` against this port forever for the rest
        # of the pytest session (see review finding: leaked never-
        # terminating background thread). By keeping the connection open,
        # the client's `for line in stream` loop instead blocks forever on
        # a plain `recv()` after processing the last snapshot: no more
        # connect attempts, no more prints, no interference with a later
        # test's ephemeral port. Stash references so GC doesn't close them
        # out from under the parked thread.
        _KEEPALIVE_SOCKETS.append(conn)
        _KEEPALIVE_SOCKETS.append(server)

    threading.Thread(target=run_server, daemon=True).start()

    received = []
    t = threading.Thread(
        target=listen,
        args=('127.0.0.1', lambda snap, sock: received.append(snap)),
        kwargs={"once": False},
        daemon=True,
    )
    t.start()
    time.sleep(0.6)

    assert [s["revision"] for s in received] == [1, 2]


def test_listen_reports_status_transitions(monkeypatch):
    port = _free_port()
    monkeypatch.setattr('arena_feed.ARENA_PORT', port)

    server = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_STREAM)
    server.setsockopt(socket_module.SOL_SOCKET, socket_module.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', port))
    server.listen(1)

    line = json.dumps(_snapshot()) + "\n"

    def run_server():
        conn, _ = server.accept()
        conn.sendall(line.encode())
        time.sleep(0.2)
        conn.close()
        server.close()

    threading.Thread(target=run_server, daemon=True).start()

    statuses = []
    listen('127.0.0.1', lambda snap, sock: None, once=True, on_status=statuses.append)

    assert statuses == ["connecting", "connected"]


def test_listen_reports_error_status_when_callback_raises(monkeypatch):
    port = _free_port()
    monkeypatch.setattr('arena_feed.ARENA_PORT', port)

    server = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_STREAM)
    server.setsockopt(socket_module.SOL_SOCKET, socket_module.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', port))
    server.listen(1)

    line = json.dumps(_snapshot()) + "\n"
    received = {}

    def run_server():
        conn, _ = server.accept()
        conn.sendall(line.encode())
        with conn.makefile("r", encoding="utf-8", newline="\n") as stream:
            reply = stream.readline()
        received["payload"] = json.loads(reply)
        conn.close()
        server.close()

    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    def raising_callback(snap, sock):
        raise ValueError("boom")

    # once=True: listen() returns after handling the single snapshot
    # (including the caught callback exception), so this runs on the main
    # test thread with no background thread of its own to leak.
    listen('127.0.0.1', raising_callback, once=True)

    server_thread.join(timeout=2.0)
    assert not server_thread.is_alive()

    payload = received["payload"]
    assert payload["type"] == "algorithm_status"
    assert payload["state"] == "error"
    assert "boom" in payload["message"]
