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

    def on_obstacle_reached(obstacle_id: str) -> None:
        send_status(sock, revision, "obstacle_reached", f"Reached obstacle {obstacle_id}", obstacleId=obstacle_id)

    send_status(sock, revision, "running", "Sending route to nanocar")
    with CarConnection(host=host) as car:
        car.send_commands(commands, on_obstacle_reached=on_obstacle_reached)
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
