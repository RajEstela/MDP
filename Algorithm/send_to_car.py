"""
Compute the optimal path and send all commands to the car via WiFi.

Run from the Algorithm/ directory:

  Full run (uses OBSTACLES list in simulator/planner.py):
    python send_to_car.py

  A5 single-obstacle test:
    python send_to_car.py A5_Test 100,80,N
    python send_to_car.py A5_Test "Obstacle(100,80,N)"

  x, y are cm from the bottom-left corner of the arena.
  Face is N / S / E / W.
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
    args = [a for a in sys.argv[1:] if a != '--dry-run']
    dry_run = '--dry-run' in sys.argv[1:]

    if not args:
        return OBSTACLES, dry_run
    mode = args[0].upper().replace('-', '_')
    if mode in ('A5', 'A5_TEST'):
        if len(args) < 2:
            print("Usage: python send_to_car.py A5_Test x,y,Face [--dry-run]")
            print("  e.g. python send_to_car.py A5_Test 100,80,N")
            sys.exit(1)
        try:
            return [_parse_obstacle(args[1])], dry_run
        except ValueError as exc:
            print(f"Bad obstacle spec: {exc}")
            sys.exit(1)
    print(f"Unknown mode {args[0]!r}. Run without arguments for full 5-obstacle run.")
    sys.exit(1)


def main() -> None:
    obstacles, dry_run = _parse_args()
    label = "A5 test" if len(obstacles) == 1 else f"{len(obstacles)}-obstacle run"
    if dry_run:
        label += " [DRY RUN]"
    print(f"[{label}] Computing optimal path...")
    if len(obstacles) == 1:
        obs = obstacles[0]
        print(f"  Obstacle: x={obs.x} y={obs.y} face={obs.face}")

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
