"""
Compute the optimal path and send all commands to the car via WiFi.

Run from the Algorithm/ directory:
    python send_to_car.py

The OBSTACLES list below should match the actual obstacle positions for your
run. Edit x/y (cm from bottom-left corner of arena) and face ('N','S','E','W').
"""
from comms import CarConnection
from simulator.planner import OBSTACLES, get_commands


def main() -> None:
    print("Computing optimal path...")
    cmds = get_commands(OBSTACLES)
    movement_cmds = [c for c in cmds if c.kind != 'WAIT']
    print(f"Path: {len(movement_cmds)} movement commands "
          f"({len(cmds) - len(movement_cmds)} WAIT steps skipped)")

    with CarConnection() as car:
        car.send_commands(cmds)

    print("Done — all commands sent.")


if __name__ == '__main__':
    main()
