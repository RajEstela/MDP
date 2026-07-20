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
    assert obstacles == [Obstacle(x=50, y=160, face='S')]


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


# ── arena_to_robot_start ─────────────────────────────────────────────────

def test_arena_to_robot_start_north():
    start = arena_to_robot_start(_snapshot(robot={"x": 1, "y": 1, "direction": "N"}))
    assert start == RobotState(x=15.0, y=30.0, theta=90.0)


def test_arena_to_robot_start_east():
    start = arena_to_robot_start(_snapshot(robot={"x": 1, "y": 1, "direction": "E"}))
    assert start == RobotState(x=30.0, y=15.0, theta=0.0)


def test_arena_to_robot_start_south():
    start = arena_to_robot_start(_snapshot(robot={"x": 1, "y": 1, "direction": "S"}))
    assert start == RobotState(x=15.0, y=0.0, theta=270.0)


def test_arena_to_robot_start_west():
    start = arena_to_robot_start(_snapshot(robot={"x": 1, "y": 1, "direction": "W"}))
    assert start == RobotState(x=0.0, y=15.0, theta=180.0)


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
