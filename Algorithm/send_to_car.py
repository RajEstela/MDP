"""
Compute the optimal path and send all commands to the car via WiFi.

Run from the Algorithm/ directory:

  Use hardcoded OBSTACLES in simulator/planner.py:
    python send_to_car.py

  Pass one or more obstacles on the command line:
    python send_to_car.py 100,80,N
    python send_to_car.py 100,80,N 150,100,E 50,120,S

  Add --dry-run to print commands without connecting:
    python send_to_car.py 100,80,N --dry-run

  Format: x,y,Face  where x/y are cm from bottom-left and Face is N/S/E/W.
"""
import sys

from comms import CarConnection
from simulator.planner import OBSTACLES, get_commands
from simulator.types import Obstacle


def _parse_obstacle(spec: str) -> Obstacle:
    spec = spec.strip()
    if spec.lower().startswith('obstacle(') and spec.endswith(')'):
        spec = spec[9:-1]
    parts = [p.strip() for p in spec.split(',')]
    if len(parts) != 3:
        raise ValueError(f"Expected x,y,Face — got {spec!r}")
    x, y, face = int(parts[0]), int(parts[1]), parts[2].upper()
    if face not in ('N', 'S', 'E', 'W'):
        raise ValueError(f"Face must be N/S/E/W — got {face!r}")
    return Obstacle(x=x, y=y, face=face)


def _parse_args() -> tuple[list[Obstacle], bool]:
    """Returns (obstacles, dry_run)."""
    raw = sys.argv[1:]
    dry_run = '--dry-run' in raw
    args = [a for a in raw if a != '--dry-run']

    if not args:
        return OBSTACLES, dry_run

    # Backwards compat: allow leading 'A5_Test' / 'A5' token
    if args[0].upper().replace('-', '_') in ('A5', 'A5_TEST'):
        args = args[1:]
        if not args:
            print("Usage: python send_to_car.py A5_Test x,y,Face ...")
            print("  e.g. python send_to_car.py A5_Test 100,80,N")
            sys.exit(1)

    obstacles: list[Obstacle] = []
    for spec in args:
        try:
            obstacles.append(_parse_obstacle(spec))
        except ValueError as exc:
            print(f"Bad obstacle spec {spec!r}: {exc}")
            sys.exit(1)
    return obstacles, dry_run


def main() -> None:
    obstacles, dry_run = _parse_args()
    n = len(obstacles)
    label = f"{n} obstacle{'s' if n != 1 else ''}"
    if dry_run:
        label += " [DRY RUN]"
    print(f"[{label}] Computing optimal path...")
    for i, obs in enumerate(obstacles):
        print(f"  [{i+1}] x={obs.x} y={obs.y} face={obs.face}")

    cmds = get_commands(obstacles)
    movement_cmds = [c for c in cmds if c.kind != 'WAIT']
    print(f"  {len(movement_cmds)} movement commands "
          f"({len(cmds) - len(movement_cmds)} WAIT steps skipped)")

    if dry_run:
        print("Commands that would be sent:")
        for c in movement_cmds:
            print(f"  {c.kind}{round(c.value):03d}")
        print("Dry run complete — no connection made.")
        return

    with CarConnection() as car:
        car.send_commands(cmds)

    print("Done — all commands sent.")


if __name__ == '__main__':
    main()
