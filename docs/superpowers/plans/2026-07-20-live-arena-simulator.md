# Live Arena Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let `simulator/main.py` receive the live arena configuration (obstacles + robot start pose) from the Raspberry Pi over TCP, run the visual pygame simulation against it, then drive the physical car through the same route while showing live progress.

**Architecture:** A new shared module `arena_feed.py` owns arena-JSON parsing/validation and the RPi TCP listen loop (used by both the existing headless `live_arena.py` and the new `simulator/main.py --live` mode). `simulator/main.py` gains a `--live` mode that runs an arena-listener thread and, after the route animates, a car-executor thread, coordinating through a small thread-safe state object and a queue — keeping the pygame frame loop itself single-threaded and responsive.

**Tech Stack:** Python 3.14, pygame-ce, pytest, stdlib `socket`/`threading`/`queue`.

## Global Constraints

- Arena grid is fixed at 20×20 cells, 10cm cells (`GRID_SIZE`/`CELL_CM` in `simulator/config.py`) — snapshots that don't match are rejected.
- `grid.origin` must be `"bottom-left"`.
- Arena feed port is `5001` (`arena_feed.ARENA_PORT`); car command port is `5000` (`app_config.RPI_PORT`, via `comms.CarConnection`).
- `robot.x`/`robot.y` in the arena JSON are 0-indexed grid cells identifying the **center cell** of the robot's 3×3-cell (30×30cm) footprint. Direction → heading: N=90°, E=0°, S=270°, W=180° (0°=East, 90°=North, CCW positive — the simulator's existing convention).
- Car execution is opt-in via `--execute`; without it, live mode only simulates.
- All Python commands below run from `Algorithm/` (the existing convention in this repo — see `Claude_Code_Handoff_Algorithm_Module.md`).

**Task order matters here:** each task assumes every earlier-numbered task is already merged, since Task 4 calls a `planner.py` signature that Task 3 introduces, and Task 6 calls interfaces from Tasks 1, 2, 3, and 5.

---

### Task 1: `arena_feed.py` — snapshot parsing (obstacles, robot start, status)

**Files:**
- Create: `Algorithm/arena_feed.py`
- Test: `Algorithm/tests/test_arena_feed.py`

**Interfaces:**
- Consumes: `simulator.config.{CELL_CM, GRID_SIZE}`, `simulator.types.{Obstacle, RobotState}` (all existing).
- Produces: `arena_feed.arena_to_obstacles(snapshot: dict) -> list[Obstacle]`, `arena_feed.arena_to_robot_start(snapshot: dict) -> RobotState`, `arena_feed.send_status(sock, revision: int, state: str, message: str, **details) -> None`, `arena_feed.ARENA_PORT`, `arena_feed.RECONNECT_DELAY_S`. Used by Task 2 (`listen`), Task 4 (`live_arena.py`), and Task 6 (`simulator/main.py`).

- [ ] **Step 1: Write the failing tests**

Create `Algorithm/tests/test_arena_feed.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/test_arena_feed.py -v` (from `Algorithm/`)
Expected: `ModuleNotFoundError: No module named 'arena_feed'`

- [ ] **Step 3: Write `arena_feed.py`**

Create `Algorithm/arena_feed.py`:

```python
"""Shared arena-feed protocol client: receives arena snapshots from the
Raspberry Pi (relayed from the Android app) over TCP and reports algorithm
status back on the same connection.

Used by both live_arena.py (headless) and simulator/main.py --live (visual).
Wire format matches RaspberryPi/Robot/server.py's normalize_arena.
"""
import json
import math
import socket
import time
from typing import Callable

from simulator.config import CELL_CM, GRID_SIZE
from simulator.types import Obstacle, RobotState

ARENA_PORT = 5001
RECONNECT_DELAY_S = 2.0

_DIRECTION_THETA = {'N': 90.0, 'E': 0.0, 'S': 270.0, 'W': 180.0}
_ROBOT_HALF_WIDTH_CM = 15.0


def arena_to_obstacles(snapshot: dict) -> list[Obstacle]:
    """Validate an Android arena snapshot and convert grid cells to cm."""
    if snapshot.get("type") != "arena":
        raise ValueError("message type is not arena")

    grid = snapshot.get("grid") or {}
    columns = int(grid.get("columns", 0))
    rows = int(grid.get("rows", 0))
    cell_cm = int(grid.get("cellCm", 0))
    origin = grid.get("origin")
    if columns != GRID_SIZE or rows != GRID_SIZE:
        raise ValueError(
            f"planner requires {GRID_SIZE}x{GRID_SIZE}, received {columns}x{rows}"
        )
    if cell_cm != CELL_CM:
        raise ValueError(f"planner requires {CELL_CM} cm cells, received {cell_cm}")
    if origin != "bottom-left":
        raise ValueError("planner requires a bottom-left coordinate origin")

    obstacles = []
    for item in snapshot.get("obstacles") or []:
        face = str(item.get("direction", "")).upper()
        if face not in ("N", "E", "S", "W"):
            raise ValueError(f"invalid direction for obstacle {item.get('id')}")
        obstacles.append(
            Obstacle(
                x=int(item["x"]) * cell_cm,
                y=int(item["y"]) * cell_cm,
                face=face,
            )
        )
    return obstacles


def arena_to_robot_start(snapshot: dict) -> RobotState:
    """Validate an Android arena snapshot's robot field and convert it to
    the apex pose the simulator tracks.

    robot.x/y are 0-indexed grid cells identifying the center cell of the
    robot's 3x3-cell (30x30cm) footprint. The apex (front-center point) is
    the footprint center offset by half the robot width along the facing
    direction.
    """
    grid = snapshot.get("grid") or {}
    columns = int(grid.get("columns", 0))
    rows = int(grid.get("rows", 0))
    cell_cm = int(grid.get("cellCm", 0))

    robot = snapshot.get("robot") or {}
    direction = str(robot.get("direction", "")).upper()
    if direction not in _DIRECTION_THETA:
        raise ValueError(f"invalid robot direction: {robot.get('direction')!r}")

    try:
        x = int(robot["x"])
        y = int(robot["y"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("robot.x and robot.y must be integers")
    if not (0 <= x < columns and 0 <= y < rows):
        raise ValueError(f"robot position ({x},{y}) is outside the arena")

    center_x = x * cell_cm + cell_cm / 2
    center_y = y * cell_cm + cell_cm / 2
    theta = _DIRECTION_THETA[direction]
    rad = math.radians(theta)
    apex_x = center_x + _ROBOT_HALF_WIDTH_CM * math.cos(rad)
    apex_y = center_y + _ROBOT_HALF_WIDTH_CM * math.sin(rad)
    return RobotState(x=apex_x, y=apex_y, theta=theta)


def send_status(sock: socket.socket, revision: int, state: str, message: str, **details) -> None:
    payload = {
        "type": "algorithm_status",
        "revision": revision,
        "state": state,
        "message": message,
        **details,
    }
    sock.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))
```

Note: `listen()` is added in Task 2 — this step only covers the pure-function pieces the tests above exercise.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_arena_feed.py -v`
Expected: all tests PASS

- [ ] **Step 5: Commit**

```bash
git add Algorithm/arena_feed.py Algorithm/tests/test_arena_feed.py
git commit -m "feat: add arena_feed module for snapshot parsing and status replies"
```

---

### Task 2: `arena_feed.listen()` — connect/reconnect loop

**Files:**
- Modify: `Algorithm/arena_feed.py`
- Test: `Algorithm/tests/test_arena_feed.py`

**Interfaces:**
- Consumes: `arena_feed.send_status` (Task 1).
- Produces: `arena_feed.listen(host: str, on_snapshot: Callable[[dict, socket.socket], None], once: bool = False, on_status: Callable[[str], None] | None = None) -> None`. Used by Task 4 (`live_arena.py`) and Task 6 (`simulator/main.py`).

- [ ] **Step 1: Write the failing tests**

Append to `Algorithm/tests/test_arena_feed.py`:

```python
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
        time.sleep(0.3)
        conn.close()
        server.close()

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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/test_arena_feed.py -v -k listen`
Expected: FAIL with `AttributeError`/`ImportError` — `listen` not defined in `arena_feed`.

- [ ] **Step 3: Add `listen()` to `arena_feed.py`**

Append to `Algorithm/arena_feed.py`:

```python
def listen(
    host: str,
    on_snapshot: Callable[[dict, socket.socket], None],
    once: bool = False,
    on_status: Callable[[str], None] | None = None,
) -> None:
    """Connect to the RPi's arena feed and invoke on_snapshot(snapshot, sock)
    for each new, distinct arena snapshot received. Reconnects on drop.

    on_status, if given, is called with 'connecting' / 'connected' /
    'reconnecting' as the connection state changes.
    """
    def _status(s: str) -> None:
        if on_status:
            on_status(s)

    last_snapshot_signature = None
    while True:
        _status("connecting")
        try:
            print(f"[arena] connecting to {host}:{ARENA_PORT}...")
            with socket.create_connection((host, ARENA_PORT), timeout=10.0) as sock:
                sock.settimeout(None)
                _status("connected")
                print("[arena] connected; waiting for tablet arena snapshot")
                with sock.makefile("r", encoding="utf-8", newline="\n") as stream:
                    for line in stream:
                        if not line.strip():
                            continue
                        snapshot = json.loads(line)
                        if snapshot.get("type") != "arena":
                            continue
                        revision = int(snapshot.get("revision", 0))
                        signature = json.dumps(snapshot, sort_keys=True, separators=(",", ":"))
                        if signature == last_snapshot_signature:
                            continue
                        last_snapshot_signature = signature
                        try:
                            on_snapshot(snapshot, sock)
                        except Exception as exc:
                            print("[arena] processing error: " + str(exc))
                            send_status(sock, revision, "error", str(exc))
                        if once:
                            return
        except (ConnectionError, OSError, json.JSONDecodeError) as exc:
            print("[arena] connection lost: " + str(exc))
            _status("reconnecting")

        if once:
            return
        time.sleep(RECONNECT_DELAY_S)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_arena_feed.py -v`
Expected: all tests PASS (12 tests from Task 1 + 3 from this task)

- [ ] **Step 5: Commit**

```bash
git add Algorithm/arena_feed.py Algorithm/tests/test_arena_feed.py
git commit -m "feat: add arena_feed.listen connect/reconnect loop"
```

---

### Task 3: `planner.py` — accept an optional start pose

**Files:**
- Modify: `Algorithm/simulator/planner.py:259-302` (`get_top_n_routes`, `get_commands`)
- Test: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Consumes: nothing new.
- Produces: `get_top_n_routes(obstacles, n=5, start: RobotState | None = None)`, `get_commands(obstacles, start: RobotState | None = None)` — both default to today's config-derived start when `start` is omitted, so all existing callers are unaffected. Used by Task 4 (`live_arena.py`) and Task 6 (`simulator/main.py`).

- [ ] **Step 1: Write the failing tests**

Add to `Algorithm/simulator/tests/test_logic.py` (add `get_top_n_routes` to the existing `from simulator.planner import ...` line at the top of the file):

```python
def test_get_commands_uses_provided_start():
    custom_start = RobotState(x=100.0, y=100.0, theta=0.0)
    obs = [Obstacle(x=150, y=100, face='W')]
    cmds = get_commands(obs, start=custom_start)
    state = custom_start
    for cmd in cmds:
        remaining = cmd.value
        while remaining > 0.001:
            state, remaining = step_command(state, cmd, remaining)
    pose = obstacle_approach_pose(obs[0])
    assert math.hypot(state.x - pose.x, state.y - pose.y) < 2.0


def test_get_commands_without_start_keeps_default_behavior():
    cmds_default = get_commands(OBSTACLES)
    cmds_explicit_default = get_commands(OBSTACLES, start=None)
    assert cmds_default == cmds_explicit_default


def test_get_top_n_routes_uses_provided_start():
    custom_start = RobotState(x=100.0, y=100.0, theta=0.0)
    obs = [Obstacle(x=150, y=100, face='W')]
    routes = get_top_n_routes(obs, n=1, start=custom_start)
    _, length = routes[0]
    assert abs(length - 30.0) < 0.01
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest simulator/tests/test_logic.py -v -k "provided_start or without_start"`
Expected: FAIL with `TypeError: get_commands() got an unexpected keyword argument 'start'`

- [ ] **Step 3: Update `planner.py`**

In `Algorithm/simulator/planner.py`, replace:

```python
def get_top_n_routes(
    obstacles: list[Obstacle],
    n: int = 5,
) -> list[tuple[list[Command], float]]:
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
```

with:

```python
def get_top_n_routes(
    obstacles: list[Obstacle],
    n: int = 5,
    start: RobotState | None = None,
) -> list[tuple[list[Command], float]]:
    if start is None:
        start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
```

and replace:

```python
def get_commands(obstacles: list[Obstacle]) -> list[Command]:
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
```

with:

```python
def get_commands(obstacles: list[Obstacle], start: RobotState | None = None) -> list[Command]:
    if start is None:
        start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest simulator/tests/test_logic.py -v`
Expected: all tests PASS (65 existing + 3 new)

- [ ] **Step 5: Commit**

```bash
git add Algorithm/simulator/planner.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat: let get_commands/get_top_n_routes accept an explicit start pose"
```

---

### Task 4: Refactor `live_arena.py` onto `arena_feed`, fix robot-start bug

**Files:**
- Modify: `Algorithm/live_arena.py` (full rewrite of the file — it shrinks significantly)
- Test: `Algorithm/tests/test_live_arena.py`

**Interfaces:**
- Consumes: `arena_feed.{arena_to_obstacles, arena_to_robot_start, listen, send_status}` (Tasks 1–2), `simulator.planner.get_commands(obstacles, start=None)` (Task 3).
- Produces: `live_arena.process_snapshot(sock, snapshot, host, execute) -> None` (unchanged name/behavior, now start-pose-correct), `live_arena.parse_args()`.

- [ ] **Step 1: Write the failing test**

Create `Algorithm/tests/test_live_arena.py`:

```python
from unittest.mock import Mock

import live_arena
from simulator.types import Command, RobotState


def test_process_snapshot_passes_robot_start_to_get_commands(monkeypatch):
    captured = {}

    def fake_get_commands(obstacles, start=None):
        captured["obstacles"] = obstacles
        captured["start"] = start
        return [Command("FW", 10.0)]

    monkeypatch.setattr(live_arena, "get_commands", fake_get_commands)

    snapshot = {
        "version": 1, "type": "arena", "revision": 3,
        "grid": {"columns": 20, "rows": 20, "cellCm": 10, "origin": "bottom-left"},
        "robot": {"x": 1, "y": 1, "direction": "N"},
        "obstacles": [],
    }
    sock = Mock()
    live_arena.process_snapshot(sock, snapshot, host="1.2.3.4", execute=False)

    assert captured["start"] == RobotState(x=15.0, y=30.0, theta=90.0)


def test_process_snapshot_sends_route_ready_status_without_execute(monkeypatch):
    monkeypatch.setattr(live_arena, "get_commands", lambda obstacles, start=None: [Command("FW", 10.0)])
    snapshot = {
        "version": 1, "type": "arena", "revision": 3,
        "grid": {"columns": 20, "rows": 20, "cellCm": 10, "origin": "bottom-left"},
        "robot": {"x": 1, "y": 1, "direction": "N"},
        "obstacles": [],
    }
    sock = Mock()
    live_arena.process_snapshot(sock, snapshot, host="1.2.3.4", execute=False)

    sent_payloads = [call.args[0] for call in sock.sendall.call_args_list]
    states = [c.decode() for c in sent_payloads]
    assert any('"state":"route_ready"' in s for s in states)
    assert not any('"state":"running"' in s for s in states)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/test_live_arena.py -v`
Expected: FAIL with `AssertionError` on `captured["start"]` — the current `live_arena.py` ignores the `robot` field and always uses the fixed config start pose.

- [ ] **Step 3: Rewrite `live_arena.py`**

Replace the full contents of `Algorithm/live_arena.py` with:

```python
"""Receive arena snapshots from the Raspberry Pi and calculate a route.

Connect the laptop to the nanocar Wi-Fi network, then run from Algorithm/:

    python live_arena.py

Route calculation is the default. Add --execute only when the physical car is
ready for the calculated movement commands to be sent through TCP port 5000.
"""

import argparse

from arena_feed import arena_to_obstacles, arena_to_robot_start, listen, send_status
from comms import CarConnection, RPI_HOST, serialize
from simulator.planner import get_commands


def process_snapshot(sock, snapshot: dict, host: str, execute: bool) -> None:
    revision = int(snapshot.get("revision", 0))
    send_status(sock, revision, "planning", "Calculating route")
    obstacles = arena_to_obstacles(snapshot)
    start = arena_to_robot_start(snapshot)
    commands = get_commands(obstacles, start=start)
    movement_commands = [command for command in commands if serialize(command) is not None]

    print(f"[arena] revision {revision}: {len(obstacles)} obstacles")
    for item, obstacle in zip(snapshot.get("obstacles") or [], obstacles):
        print(
            f"  {item.get('id')}: grid=({item.get('x')},{item.get('y')}) "
            f"cm=({obstacle.x},{obstacle.y}) face={obstacle.face}"
        )
    print(f"[route] {len(movement_commands)} movement commands")
    print("  " + " ".join(serialize(command) for command in movement_commands))

    send_status(
        sock,
        revision,
        "route_ready",
        "Route calculated",
        commandCount=len(movement_commands),
    )
    if not execute:
        print("[route] calculation complete; --execute was not supplied")
        return

    send_status(sock, revision, "running", "Sending route to nanocar")
    with CarConnection(host=host) as car:
        car.send_commands(commands)
    send_status(sock, revision, "completed", "Route completed")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive tablet arena data through the nanocar Pi")
    parser.add_argument("--host", default=RPI_HOST, help="Raspberry Pi hotspot IP address")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="send the calculated route to the physical car on port 5000",
    )
    parser.add_argument("--once", action="store_true", help="handle one arena snapshot and exit")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    listen(
        args.host,
        lambda snapshot, sock: process_snapshot(sock, snapshot, args.host, args.execute),
        once=args.once,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_live_arena.py -v`
Expected: both tests PASS

- [ ] **Step 5: Run the full test suite to confirm no regressions**

Run: `python -m pytest -v`
Expected: all tests PASS (existing `simulator/tests/test_logic.py` and `tests/test_app_config.py` unaffected)

- [ ] **Step 6: Commit**

```bash
git add Algorithm/live_arena.py Algorithm/tests/test_live_arena.py
git commit -m "refactor: rebuild live_arena.py on arena_feed, fix robot start-pose bug"
```

---

### Task 5: `comms.py` — progress callback on `send_commands`

**Files:**
- Modify: `Algorithm/comms.py`
- Test: `Algorithm/tests/test_comms.py`

**Interfaces:**
- Consumes: nothing new.
- Produces: `CarConnection.send_commands(cmds, on_progress: Callable[[int, int, str], None] | None = None) -> None` — `on_progress(sent, total, wire)` is called after each acknowledged movement command. Used by Task 6's car-executor thread.

- [ ] **Step 1: Write the failing tests**

Create `Algorithm/tests/test_comms.py`:

```python
import json

from comms import CarConnection, serialize
from simulator.types import Command


class _FakeSocket:
    def __init__(self, responses: list[bytes]):
        self._responses = list(responses)
        self.sent: list[bytes] = []

    def sendall(self, data: bytes) -> None:
        self.sent.append(data)

    def recv(self, bufsize: int) -> bytes:
        return self._responses.pop(0)


def _ok_response() -> bytes:
    return json.dumps({"id": 1, "status": 200, "msg": "OK"}).encode()


def _make_connection(n_responses: int) -> CarConnection:
    conn = CarConnection.__new__(CarConnection)
    conn._host = "test-host"
    conn._port = 0
    conn._seq = 0
    conn._sock = _FakeSocket([_ok_response() for _ in range(n_responses)])
    return conn


def test_serialize_formats_command():
    assert serialize(Command("FW", 50.0)) == "FW050"


def test_serialize_wait_returns_none():
    assert serialize(Command("WAIT", 300.0)) is None


def test_send_commands_calls_on_progress_for_movement_commands_only():
    conn = _make_connection(n_responses=2)
    calls = []
    conn.send_commands(
        [Command("FW", 50.0), Command("WAIT", 300.0), Command("RL", 90.0)],
        on_progress=lambda sent, total, wire: calls.append((sent, total, wire)),
    )
    assert calls == [(1, 2, "FW050"), (2, 2, "RL090")]


def test_send_commands_without_on_progress_still_sends_all():
    conn = _make_connection(n_responses=1)
    conn.send_commands([Command("BW", 20.0)])
    assert len(conn._sock.sent) == 1
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/test_comms.py -v`
Expected: `test_send_commands_calls_on_progress_for_movement_commands_only` FAILs with `TypeError: send_commands() got an unexpected keyword argument 'on_progress'`; the other 3 tests PASS already.

- [ ] **Step 3: Update `comms.py`**

Add `from typing import Callable` to the imports at the top of `Algorithm/comms.py`:

```python
import json
import socket
from typing import Callable
from simulator.types import Command

from app_config import RPI_HOST, RPI_PORT, RPI_TIMEOUT_S as _TIMEOUT_S
```

Replace the `send_commands` method:

```python
    def send_commands(self, cmds: list[Command]) -> None:
        """Send a sequence of commands, waiting for status 200 after each one."""
        total = sum(1 for c in cmds if c.kind != 'WAIT')
        sent = 0
        for cmd in cmds:
            if cmd.kind == 'WAIT':
                continue
            sent += 1
            print(f"[comms] ({sent}/{total})", end=' ')
            self.send_command(cmd)
```

with:

```python
    def send_commands(
        self,
        cmds: list[Command],
        on_progress: "Callable[[int, int, str], None] | None" = None,
    ) -> None:
        """Send a sequence of commands, waiting for status 200 after each one.

        If on_progress is given, it's called as on_progress(sent, total, wire)
        after each successful command acknowledgment.
        """
        total = sum(1 for c in cmds if c.kind != 'WAIT')
        sent = 0
        for cmd in cmds:
            if cmd.kind == 'WAIT':
                continue
            sent += 1
            wire = serialize(cmd)
            print(f"[comms] ({sent}/{total})", end=' ')
            self.send_command(cmd)
            if on_progress:
                on_progress(sent, total, wire)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_comms.py -v`
Expected: all 4 tests PASS

- [ ] **Step 5: Commit**

```bash
git add Algorithm/comms.py Algorithm/tests/test_comms.py
git commit -m "feat: add on_progress callback to CarConnection.send_commands"
```

---

### Task 6: `simulator/main.py` — `--live` mode

**Files:**
- Modify: `Algorithm/simulator/main.py`

**Interfaces:**
- Consumes: `arena_feed.{ARENA_PORT, listen, arena_to_obstacles, arena_to_robot_start, send_status}` (Tasks 1–2), `simulator.planner.get_top_n_routes(obstacles, n, start=...)` (Task 3), `comms.CarConnection` with `send_commands(cmds, on_progress=...)` (Task 5), `app_config.RPI_HOST`.
- Produces: CLI `python -m simulator.main --live [--host IP] [--execute]`; internal `_run_live(host, execute, max_frames=None)` (the `max_frames` escape hatch exists solely so Task 7's verification script can run it bounded instead of forever).

This task has no red/green unit-test cycle — it's a pygame event loop, which this codebase already tests via a manual capture script (`capture_sim.py`) rather than pytest. Its correctness is verified by Task 7, which actually runs it end-to-end.

- [ ] **Step 1: Update imports**

In `Algorithm/simulator/main.py`, replace:

```python
import math
import sys

import pygame

from simulator.arena import draw_arena, draw_obstacles, draw_path, trace_path_points
from simulator.config import ARENA_PX, FPS, START_THETA, START_X_CM, START_Y_CM
from simulator.planner import generate_random_obstacles, get_top_n_routes
from simulator.robot import draw_robot, step_command
from simulator.types import Obstacle, RobotState
```

with:

```python
import math
import queue
import sys
import threading
import time

import pygame

import arena_feed
from app_config import RPI_HOST
from comms import CarConnection
from simulator.arena import draw_arena, draw_obstacles, draw_path, trace_path_points
from simulator.config import ARENA_PX, FPS, START_THETA, START_X_CM, START_Y_CM
from simulator.planner import generate_random_obstacles, get_top_n_routes
from simulator.robot import draw_robot, step_command
from simulator.types import Obstacle, RobotState
```

- [ ] **Step 2: Make `_compute` accept an explicit start pose**

Replace (around `simulator/main.py:127-146`):

```python
def _compute(screen, font_b, obstacles):
    """Show loading screen, compute top-N routes; return (routes, traced, opt_cmds, opt_len)."""
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    n_obs = len(obstacles)
    n_routes = min(5, math.factorial(n_obs))
    screen.fill((30, 30, 30))
    draw_arena(screen)
    draw_obstacles(screen, obstacles)
    msg = font_b.render(
        f"Computing optimal path across {math.factorial(n_obs)} permutation{'s' if n_obs > 1 else ''}…",
        True, (255, 215, 0),
    )
    screen.blit(msg, (ARENA_PX // 2 - msg.get_width() // 2, ARENA_PX // 2 - 12))
    pygame.display.flip()
    pygame.event.pump()

    routes = get_top_n_routes(obstacles, n=n_routes)
    opt_cmds, opt_len = routes[0]
    traced = [trace_path_points(start, cmds) for cmds, _ in routes]
    return routes, traced, opt_cmds, opt_len
```

with:

```python
def _compute(screen, font_b, obstacles, start: RobotState | None = None):
    """Show loading screen, compute top-N routes; return (routes, traced, opt_cmds, opt_len, start)."""
    if start is None:
        start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    n_obs = len(obstacles)
    n_routes = min(5, math.factorial(n_obs))
    screen.fill((30, 30, 30))
    draw_arena(screen)
    draw_obstacles(screen, obstacles)
    msg = font_b.render(
        f"Computing optimal path across {math.factorial(n_obs)} permutation{'s' if n_obs > 1 else ''}…",
        True, (255, 215, 0),
    )
    screen.blit(msg, (ARENA_PX // 2 - msg.get_width() // 2, ARENA_PX // 2 - 12))
    pygame.display.flip()
    pygame.event.pump()

    routes = get_top_n_routes(obstacles, n=n_routes, start=start)
    opt_cmds, opt_len = routes[0]
    traced = [trace_path_points(start, cmds) for cmds, _ in routes]
    return routes, traced, opt_cmds, opt_len, start
```

Update the one call site (around `simulator/main.py:165-172`) — replace:

```python
    def new_run():
        obs = fixed_obstacles if fixed else generate_random_obstacles(n_random)
        rts, trc, opt_c, opt_l = _compute(screen, font_b, obs)
        robot = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
        return obs, rts, trc, opt_c, opt_l, robot, list(opt_c), None, 0.0, 0, 0
```

with:

```python
    def new_run():
        obs = fixed_obstacles if fixed else generate_random_obstacles(n_random)
        rts, trc, opt_c, opt_l, start = _compute(screen, font_b, obs)
        robot = RobotState(x=start.x, y=start.y, theta=start.theta)
        return obs, rts, trc, opt_c, opt_l, robot, list(opt_c), None, 0.0, 0, 0
```

- [ ] **Step 3: Run the existing test suite to confirm the `_compute` change didn't break demo mode**

Run: `python -m pytest -v` (from `Algorithm/`)
Expected: all tests PASS (this change doesn't touch anything under test, but confirms nothing else imports `_compute` with the old 4-tuple return)

- [ ] **Step 4: Add live-mode state and networking helpers**

Insert after `_draw_hud` and before `_compute` in `Algorithm/simulator/main.py`:

```python
class _LiveState:
    """Thread-safe state shared between the arena listener thread, the car
    executor thread, and the pygame main thread."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.connection_status = "connecting"
        self.last_error: str | None = None
        self.last_error_time: float = 0.0
        self.exec_progress: dict | None = None

    def set_status(self, status: str) -> None:
        with self.lock:
            self.connection_status = status

    def set_error(self, message: str) -> None:
        with self.lock:
            self.last_error = message
            self.last_error_time = time.monotonic()


def _on_snapshot(snapshot: dict, sock, out_queue: "queue.Queue", live_state: _LiveState) -> None:
    try:
        obstacles = arena_feed.arena_to_obstacles(snapshot)
        start = arena_feed.arena_to_robot_start(snapshot)
    except ValueError as exc:
        live_state.set_error(str(exc))
        raise
    revision = int(snapshot.get("revision", 0))
    arena_feed.send_status(sock, revision, "planning", "Calculating route")
    out_queue.put((obstacles, start, revision, sock))


def _run_car_executor(commands: list, host: str, sock, revision: int, live_state: _LiveState) -> None:
    total = sum(1 for c in commands if c.kind != 'WAIT')
    with live_state.lock:
        live_state.exec_progress = {"index": 0, "total": total, "last_wire": "", "done": False, "error": None}

    def on_progress(sent: int, sent_total: int, wire: str) -> None:
        with live_state.lock:
            live_state.exec_progress["index"] = sent
            live_state.exec_progress["last_wire"] = wire

    arena_feed.send_status(sock, revision, "running", "Sending route to nanocar")
    try:
        with CarConnection(host=host) as car:
            car.send_commands(commands, on_progress=on_progress)
    except Exception as exc:
        with live_state.lock:
            live_state.exec_progress["error"] = str(exc)
            live_state.exec_progress["done"] = True
        arena_feed.send_status(sock, revision, "error", str(exc))
        return

    with live_state.lock:
        live_state.exec_progress["done"] = True
    arena_feed.send_status(sock, revision, "completed", "Route completed")


def _draw_connection_banner(surface, font, live_state: _LiveState) -> None:
    with live_state.lock:
        status = live_state.connection_status
        last_error = live_state.last_error
        last_error_time = live_state.last_error_time

    now = time.monotonic()
    if last_error and now - last_error_time < 5.0:
        msg = font.render(f"Arena error: {last_error}", True, (255, 80, 80))
        surface.blit(msg, (8, ARENA_PX - 26))
        return

    colors = {"connecting": (200, 200, 0), "connected": (80, 220, 80), "reconnecting": (255, 140, 0)}
    col = colors.get(status, (200, 200, 200))
    msg = font.render(f"[{status}]", True, col)
    surface.blit(msg, (ARENA_PX - msg.get_width() - 8, 8))


def _draw_exec_overlay(surface, font_b, progress: dict | None) -> None:
    bar = pygame.Surface((ARENA_PX, 30), pygame.SRCALPHA)
    bar.fill((0, 0, 0, 190))
    surface.blit(bar, (0, ARENA_PX - 66))
    if progress is None:
        text, col = "Preparing to send commands to car...", (200, 200, 0)
    elif progress.get("error"):
        text, col = f"Car error: {progress['error']}", (255, 80, 80)
    elif progress.get("done"):
        text, col = "All commands sent — car finished", (80, 255, 120)
    else:
        text = f"Sending to car: {progress['last_wire']} ({progress['index']}/{progress['total']})..."
        col = (255, 215, 0)
    msg = font_b.render(text, True, col)
    surface.blit(msg, (ARENA_PX // 2 - msg.get_width() // 2, ARENA_PX - 60))
```

- [ ] **Step 5: Add `_parse_live_args` and `_run_live`**

Insert immediately before `def main() -> None:` in `Algorithm/simulator/main.py`:

```python
def _parse_live_args(args: list[str]) -> tuple[str, bool]:
    """Parse the remaining CLI args (after --live is stripped) into (host, execute)."""
    host = RPI_HOST
    execute = False
    i = 0
    while i < len(args):
        if args[i] == '--host':
            if i + 1 >= len(args):
                print("Usage: python -m simulator.main --live [--host <ip>] [--execute]")
                sys.exit(1)
            host = args[i + 1]
            i += 2
        elif args[i] == '--execute':
            execute = True
            i += 1
        else:
            print(f"Unknown live-mode argument: {args[i]!r}")
            sys.exit(1)
    return host, execute


def _run_live(host: str, execute: bool, max_frames: int | None = None) -> None:
    pygame.init()
    screen = pygame.display.set_mode((ARENA_PX, ARENA_PX))
    pygame.display.set_caption("MDP Simulator — live")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont('Arial', 16)
    font_b = pygame.font.SysFont('Arial', 18, bold=True)

    live_state = _LiveState()
    out_queue: "queue.Queue" = queue.Queue()
    listener = threading.Thread(
        target=arena_feed.listen,
        args=(host, lambda snap, sock: _on_snapshot(snap, sock, out_queue, live_state)),
        kwargs={"on_status": live_state.set_status},
        daemon=True,
    )
    listener.start()

    phase = 'waiting_for_arena'
    obstacles: list[Obstacle] = []
    routes: list = []
    traced: list = []
    opt_cmds: list = []
    opt_len = 0.0
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    cmd_queue: list = []
    active = None
    remaining = 0.0
    anim_frames = 0
    obstacles_visited = 0
    current_sock = None
    current_revision = 0
    executor_started = False

    def _start_new_run(obs, start, revision, sock):
        nonlocal obstacles, routes, traced, opt_cmds, opt_len, state, cmd_queue
        nonlocal active, remaining, anim_frames, obstacles_visited
        nonlocal current_sock, current_revision, executor_started, phase
        obstacles = obs
        routes, traced, opt_cmds, opt_len, start = _compute(screen, font_b, obstacles, start)
        state = RobotState(x=start.x, y=start.y, theta=start.theta)
        cmd_queue = list(opt_cmds)
        active = None
        remaining = 0.0
        anim_frames = 0
        obstacles_visited = 0
        current_sock = sock
        current_revision = revision
        executor_started = False
        phase = 'animate'

    frame = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_q, pygame.K_ESCAPE):
                running = False

        if phase in ('waiting_for_arena', 'done'):
            latest = None
            while True:
                try:
                    latest = out_queue.get_nowait()
                except queue.Empty:
                    break
            if latest is not None:
                _start_new_run(*latest)

        screen.fill((30, 30, 30))
        draw_arena(screen)

        if phase == 'waiting_for_arena':
            msg = font_b.render(
                f"Waiting for arena data from {host}:{arena_feed.ARENA_PORT}...",
                True, (210, 210, 210),
            )
            screen.blit(msg, (ARENA_PX // 2 - msg.get_width() // 2, ARENA_PX // 2 - 12))

        elif phase in ('animate', 'executing', 'done'):
            draw_path(screen, traced[0], _ROUTE_COLORS[0], width=2)
            draw_obstacles(screen, obstacles)
            _draw_legend(screen, font, font_b, routes, phase, len(obstacles))

            if phase == 'animate':
                if active is None and cmd_queue:
                    active = cmd_queue.pop(0)
                    remaining = active.value
                    if active.kind == 'WAIT':
                        obstacles_visited += 1
                if active is not None:
                    state, remaining = step_command(state, active, remaining)
                    if remaining <= 0:
                        active = None
                anim_frames += 1
                if not cmd_queue and active is None:
                    arena_feed.send_status(
                        current_sock, current_revision, "route_ready", "Route calculated",
                        commandCount=sum(1 for c in opt_cmds if c.kind != 'WAIT'),
                    )
                    phase = 'executing' if execute else 'done'

            draw_robot(screen, state)
            _draw_hud(
                screen, font, font_b, anim_frames / FPS, opt_len,
                obstacles_visited, len(obstacles), False, phase != 'animate',
            )

            if phase == 'executing':
                if not executor_started:
                    executor_started = True
                    threading.Thread(
                        target=_run_car_executor,
                        args=(list(opt_cmds), host, current_sock, current_revision, live_state),
                        daemon=True,
                    ).start()
                with live_state.lock:
                    progress = dict(live_state.exec_progress) if live_state.exec_progress else None
                _draw_exec_overlay(screen, font_b, progress)
                if progress and progress.get("done"):
                    phase = 'done'

            if phase == 'done':
                done_msg = font_b.render(
                    "Run complete — waiting for next arena snapshot", True, (0, 255, 120),
                )
                screen.blit(done_msg, (ARENA_PX // 2 - done_msg.get_width() // 2, 40))

        _draw_connection_banner(screen, font, live_state)
        pygame.display.flip()
        clock.tick(FPS)

        frame += 1
        if max_frames is not None and frame >= max_frames:
            running = False

    pygame.quit()
```

- [ ] **Step 6: Wire `--live` into `main()`**

Replace the start of `def main() -> None:`:

```python
def main() -> None:
    fixed_obstacles, n_random = _parse_args()
```

with:

```python
def main() -> None:
    argv = sys.argv[1:]
    if '--live' in argv:
        host, execute = _parse_live_args([a for a in argv if a != '--live'])
        _run_live(host, execute)
        return

    fixed_obstacles, n_random = _parse_args()
```

- [ ] **Step 7: Run the full test suite**

Run: `python -m pytest -v` (from `Algorithm/`)
Expected: all tests PASS — this task adds no new pytest coverage (see task intro), so this step is a regression check on everything from Tasks 1–5.

- [ ] **Step 8: Commit**

```bash
git add Algorithm/simulator/main.py
git commit -m "feat: add --live mode to simulator/main.py (network arena feed + car execution)"
```

---

### Task 7: End-to-end verification

**Files:**
- Create: `Algorithm/verify_live_sim.py`

**Interfaces:**
- Consumes: `simulator.main._run_live(host, execute, max_frames)` (Task 6).
- Produces: nothing consumed by later tasks — this is the final functional check for the whole plan.

- [ ] **Step 1: Write the verification script**

Create `Algorithm/verify_live_sim.py`:

```python
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
```

- [ ] **Step 2: Run it and confirm PASS**

Run: `python verify_live_sim.py` (from `Algorithm/`)
Expected: console shows the mock arena receiving `planning` → `route_ready` → `running` → `completed` status messages, the mock car server logs each `FW`/`BW`/`RL`/`RR` command it received, and the script prints `PASS` and exits 0. If it prints `FAIL` or raises, fix the underlying issue in Task 6's code before proceeding — do not consider this plan complete on a `FAIL`.

- [ ] **Step 3: Commit**

```bash
git add Algorithm/verify_live_sim.py
git commit -m "test: add end-to-end verification script for --live mode"
```

---

## Manual hardware check (not automatable — perform when the car is available)

Once Task 7 passes, do one real run against the actual RPi before relying on this for the competition:

1. Connect the laptop to the nanocar's Wi-Fi.
2. Run `python -m simulator.main --live` (no `--execute` yet) and trigger an arena send from the Android app. Confirm the simulator shows "Waiting for arena data...", then animates the received obstacles/robot pose correctly, then shows "Run complete."
3. Re-run with `python -m simulator.main --live --execute` and confirm the physical car receives and follows the same commands while the overlay tracks progress.
