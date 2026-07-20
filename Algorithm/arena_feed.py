"""Shared arena-feed protocol client: receives arena snapshots from the
Raspberry Pi (relayed from the Android app) over TCP and reports algorithm
status back on the same connection.

Used by both live_arena.py (headless) and simulator/main.py --live (visual).
Wire format matches RaspberryPi/Robot/server.py's normalize_arena.
"""
import json
import socket
import time
from typing import Callable

from simulator.config import CELL_CM, GRID_SIZE
from simulator.types import Obstacle, RobotState

ARENA_PORT = 5001
RECONNECT_DELAY_S = 2.0

_DIRECTION_THETA = {'N': 90.0, 'E': 0.0, 'S': 270.0, 'W': 180.0}
_ROBOT_HALF_WIDTH_CM = 15.0
# Precomputed offset vectors for each direction to avoid floating point errors
_DIRECTION_OFFSET = {
    'N': (0.0, _ROBOT_HALF_WIDTH_CM),
    'E': (_ROBOT_HALF_WIDTH_CM, 0.0),
    'S': (0.0, -_ROBOT_HALF_WIDTH_CM),
    'W': (-_ROBOT_HALF_WIDTH_CM, 0.0),
}


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
    offset_x, offset_y = _DIRECTION_OFFSET[direction]
    apex_x = center_x + offset_x
    apex_y = center_y + offset_y
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
