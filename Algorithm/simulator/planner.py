import itertools

from simulator.config import APPROACH_CM, CELL_CM, TURN_RADIUS_CM
from simulator.dubins import dubins_optimal
from simulator.types import Command, DubinsPath, Obstacle, RobotState

OBSTACLES: list[Obstacle] = [
    Obstacle(x=50, y=50, face='N'),
    Obstacle(x=100, y=30, face='E'),
    Obstacle(x=150, y=80, face='S'),
    Obstacle(x=80, y=130, face='W'),
    Obstacle(x=130, y=160, face='N'),
]

_SEGMENT_KINDS: dict[str, tuple[str, str, str]] = {
    'LSL': ('AL', 'FW', 'AL'),
    'LSR': ('AL', 'FW', 'AR'),
    'RSL': ('AR', 'FW', 'AL'),
    'RSR': ('AR', 'FW', 'AR'),
    'LRL': ('AL', 'AR', 'AL'),
    'RLR': ('AR', 'AL', 'AR'),
}


def obstacle_approach_pose(obs: Obstacle) -> RobotState:
    cx = obs.x + CELL_CM / 2
    cy = obs.y + CELL_CM / 2
    d = CELL_CM / 2 + APPROACH_CM
    if obs.face == 'N':
        return RobotState(x=cx, y=cy + d, theta=270)
    if obs.face == 'S':
        return RobotState(x=cx, y=cy - d, theta=90)
    if obs.face == 'E':
        return RobotState(x=cx + d, y=cy, theta=180)
    # face == 'W'
    return RobotState(x=cx - d, y=cy, theta=0)


def _total_dubins_length(start: RobotState, poses: list[RobotState], r: float) -> float:
    total = 0.0
    current = start
    for pose in poses:
        total += dubins_optimal(current, pose, r).total
        current = pose
    return total


def _hamiltonian_optimal_order(start: RobotState, poses: list[RobotState], r: float) -> list[RobotState]:
    best: list[RobotState] = []
    best_len = float('inf')
    for perm in itertools.permutations(poses):
        length = _total_dubins_length(start, list(perm), r)
        if length < best_len:
            best_len = length
            best = list(perm)
    return best


def dubins_to_commands(path: DubinsPath) -> list[Command]:
    k1, k2, k3 = _SEGMENT_KINDS[path.path_type]
    cmds = []
    for kind, seg in zip((k1, k2, k3), (path.seg1, path.seg2, path.seg3)):
        if seg > 0.01:
            cmds.append(Command(kind, seg))
    return cmds


def get_commands(obstacles: list[Obstacle]) -> list[Command]:
    start = RobotState(x=0, y=0, theta=90)
    poses = [obstacle_approach_pose(obs) for obs in obstacles]
    ordered = _hamiltonian_optimal_order(start, poses, TURN_RADIUS_CM)
    current = start
    cmds: list[Command] = []
    for pose in ordered:
        path = dubins_optimal(current, pose, TURN_RADIUS_CM)
        cmds += dubins_to_commands(path)
        current = pose
    return cmds
