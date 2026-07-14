"""Receive arena snapshots from the Raspberry Pi and calculate a route.

Connect the laptop to the nanocar Wi-Fi network, then run from Algorithm/:

    python live_arena.py

Route calculation is the default. Add --execute only when the physical car is
ready for the calculated movement commands to be sent through TCP port 5000.
"""

import argparse
import json
import socket
import time

from comms import CarConnection, RPI_HOST, serialize
from simulator.config import CELL_CM, GRID_SIZE
from simulator.planner import get_commands
from simulator.types import Obstacle


ARENA_PORT = 5001
RECONNECT_DELAY_S = 2.0


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


def send_status(sock: socket.socket, revision: int, state: str, message: str, **details) -> None:
    payload = {
        "type": "algorithm_status",
        "revision": revision,
        "state": state,
        "message": message,
        **details,
    }
    sock.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8"))


def process_snapshot(sock: socket.socket, snapshot: dict, host: str, execute: bool) -> None:
    revision = int(snapshot.get("revision", 0))
    send_status(sock, revision, "planning", "Calculating route")
    obstacles = arena_to_obstacles(snapshot)
    commands = get_commands(obstacles)
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


def listen(host: str, execute: bool, once: bool) -> None:
    last_snapshot_signature = None
    while True:
        try:
            print(f"[arena] connecting to {host}:{ARENA_PORT}...")
            with socket.create_connection((host, ARENA_PORT), timeout=10.0) as sock:
                sock.settimeout(None)
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
                            process_snapshot(sock, snapshot, host, execute)
                        except Exception as exc:
                            print("[arena] processing error: " + str(exc))
                            send_status(sock, revision, "error", str(exc))
                        if once:
                            return
        except (ConnectionError, OSError, json.JSONDecodeError) as exc:
            print("[arena] connection lost: " + str(exc))

        if once:
            return
        time.sleep(RECONNECT_DELAY_S)


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
    listen(host=args.host, execute=args.execute, once=args.once)
