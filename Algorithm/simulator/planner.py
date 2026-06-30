import itertools
import math

from simulator.config import APPROACH_CM, ARENA_CM, CELL_CM, START_THETA, START_X_CM, START_Y_CM, TURN_RADIUS_CM
from simulator.dubins import dubins_lrl, dubins_lsl, dubins_lsr, dubins_optimal, dubins_rlr, dubins_rsl, dubins_rsr
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


def _path_in_bounds(q1: RobotState, cmds: list[Command], r: float) -> bool:
    """Sample the path at 2cm intervals; return True iff every point is inside the arena."""
    x, y, theta = q1.x, q1.y, q1.theta
    step = 2.0
    for cmd in cmds:
        remaining = cmd.value
        while remaining > 0.001:
            advance = min(step, remaining)
            if cmd.kind == 'FW':
                rad = math.radians(theta)
                x += advance * math.cos(rad)
                y += advance * math.sin(rad)
            elif cmd.kind in ('AL', 'AR'):
                sign = 1 if cmd.kind == 'AL' else -1
                rad = math.radians(theta)
                new_rad = rad + sign * advance / r
                x += sign * r * (math.sin(new_rad) - math.sin(rad))
                y -= sign * r * (math.cos(new_rad) - math.cos(rad))
                theta = math.degrees(new_rad) % 360
            remaining -= advance
            if not (0 <= x <= ARENA_CM and 0 <= y <= ARENA_CM):
                return False
    return True


def _dubins_bounded(q1: RobotState, q2: RobotState, r: float) -> DubinsPath:
    """Shortest Dubins path from q1 to q2 that stays within the arena.
    Tries all 6 path types in ascending length order; falls back to shortest if none fit."""
    candidates = sorted(
        (c for c in [
            dubins_lsl(q1, q2, r), dubins_rsr(q1, q2, r),
            dubins_lsr(q1, q2, r), dubins_rsl(q1, q2, r),
            dubins_rlr(q1, q2, r), dubins_lrl(q1, q2, r),
        ] if c is not None),
        key=lambda p: p.total,
    )
    for path in candidates:
        if _path_in_bounds(q1, dubins_to_commands(path), r):
            return path
    return candidates[0]  # all paths exit bounds — return shortest anyway


def dubins_to_commands(path: DubinsPath) -> list[Command]:
    k1, k2, k3 = _SEGMENT_KINDS[path.path_type]
    cmds = []
    for kind, seg in zip((k1, k2, k3), (path.seg1, path.seg2, path.seg3)):
        if seg > 0.01:
            cmds.append(Command(kind, seg))
    return cmds


def get_commands(obstacles: list[Obstacle]) -> list[Command]:
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    poses = [obstacle_approach_pose(obs) for obs in obstacles]
    ordered = _hamiltonian_optimal_order(start, poses, TURN_RADIUS_CM)
    current = start
    cmds: list[Command] = []
    for pose in ordered:
        path = _dubins_bounded(current, pose, TURN_RADIUS_CM)
        cmds += dubins_to_commands(path)
        current = pose
    return cmds
