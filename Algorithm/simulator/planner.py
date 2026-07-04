import itertools
import math
import random as _random

from simulator.config import APPROACH_CM, ARENA_CM, CELL_CM, FPS, GRID_SIZE, START_THETA, START_X_CM, START_Y_CM
from simulator.types import Command, Obstacle, RobotState

OBSTACLES: list[Obstacle] = [
    Obstacle(x=50,  y=50,  face='N'),
    Obstacle(x=100, y=60,  face='E'),
    Obstacle(x=150, y=80,  face='S'),
    Obstacle(x=80,  y=130, face='W'),
    Obstacle(x=130, y=130, face='N'),
]

def _valid_faces(col: int, row: int) -> list[str]:
    """Return face directions whose approach pose fits inside the arena with ≥30 cm boundary margin."""
    cx = col * CELL_CM + CELL_CM / 2
    cy = row * CELL_CM + CELL_CM / 2
    d = CELL_CM / 2 + APPROACH_CM   # 25 cm standoff from face centre
    m = 30.0                          # 30 cm boundary margin
    faces: list[str] = []
    if m <= cx <= ARENA_CM - m and m <= cy + d <= ARENA_CM - m:
        faces.append('N')
    if m <= cx <= ARENA_CM - m and m <= cy - d <= ARENA_CM - m:
        faces.append('S')
    if m <= cx + d <= ARENA_CM - m and m <= cy <= ARENA_CM - m:
        faces.append('E')
    if m <= cx - d <= ARENA_CM - m and m <= cy <= ARENA_CM - m:
        faces.append('W')
    return faces


def generate_random_obstacles(n: int = 5, seed: int | None = None) -> list[Obstacle]:
    """Generate n random obstacles with valid approach poses.

    Guarantees:
    - No obstacle in the 40×40 cm start zone (bottom-left 4×4 cells).
    - Every approach pose fits in the arena with ≥30 cm boundary clearance.
    - Obstacles are at least 2 cells apart (Chebyshev distance ≥ 2) so their
      clearance zones don't fully overlap.
    """
    rng = _random.Random(seed)

    pool: list[tuple[int, int, list[str]]] = []
    for col in range(GRID_SIZE):
        for row in range(GRID_SIZE):
            if col < 4 and row < 4:        # exclude start zone
                continue
            faces = _valid_faces(col, row)
            if faces:
                pool.append((col, row, faces))

    rng.shuffle(pool)
    obstacles: list[Obstacle] = []
    used: set[tuple[int, int]] = set()

    for col, row, faces in pool:
        if len(obstacles) == n:
            break
        if any(max(abs(col - c), abs(row - r)) < 2 for c, r in used):
            continue
        face = rng.choice(faces)
        used.add((col, row))
        obstacles.append(Obstacle(x=col * CELL_CM, y=row * CELL_CM, face=face))

    return obstacles


_ROBOT_CLEARANCE = 20.0          # cm clearance from every obstacle cell edge


def _angle_diff(from_deg: float, to_deg: float) -> float:
    """Signed shortest angular difference. Positive = left (RL), negative = right (RR)."""
    diff = (to_deg - from_deg + 180) % 360 - 180
    return diff if diff != -180 else 180


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


def _point_hits_obstacle(x: float, y: float, obstacles: list[Obstacle]) -> bool:
    for obs in obstacles:
        if (obs.x - _ROBOT_CLEARANCE < x < obs.x + CELL_CM + _ROBOT_CLEARANCE and
                obs.y - _ROBOT_CLEARANCE < y < obs.y + CELL_CM + _ROBOT_CLEARANCE):
            return True
    return False


def _path_in_bounds(
    q1: RobotState,
    cmds: list[Command],
    obstacles: list[Obstacle] | None = None,
) -> bool:
    """Sample FW/BW at 2 cm intervals; RL/RR update heading only. True iff path stays in arena and clear of obstacles."""
    x, y, theta = q1.x, q1.y, q1.theta
    step = 2.0
    obs_list = obstacles or []
    for cmd in cmds:
        if cmd.kind in ('RL', 'RR'):
            sign = 1 if cmd.kind == 'RL' else -1
            theta = (theta + sign * cmd.value) % 360
            continue
        remaining = cmd.value
        while remaining > 0.001:
            advance = min(step, remaining)
            if cmd.kind == 'FW':
                rad = math.radians(theta)
                x += advance * math.cos(rad)
                y += advance * math.sin(rad)
            elif cmd.kind == 'BW':
                rad = math.radians(theta)
                x -= advance * math.cos(rad)
                y -= advance * math.sin(rad)
            remaining -= advance
            if not (0 <= x <= ARENA_CM and 0 <= y <= ARENA_CM):
                return False
            if obs_list and _point_hits_obstacle(x, y, obs_list):
                return False
    return True


def _direct_leg(q1: RobotState, q2: RobotState) -> tuple[list[Command], float]:
    """Build rotate→FW→rotate commands from q1 to q2."""
    dx = q2.x - q1.x
    dy = q2.y - q1.y
    dist = math.hypot(dx, dy)
    cmds: list[Command] = []

    if dist > 0.01:
        travel = math.degrees(math.atan2(dy, dx)) % 360
        r1 = _angle_diff(q1.theta, travel)
        if abs(r1) > 0.01:
            cmds.append(Command('RL' if r1 > 0 else 'RR', abs(r1)))
        cmds.append(Command('FW', dist))
        r2 = _angle_diff(travel, q2.theta)
        if abs(r2) > 0.01:
            cmds.append(Command('RL' if r2 > 0 else 'RR', abs(r2)))
    else:
        rot = _angle_diff(q1.theta, q2.theta)
        if abs(rot) > 0.01:
            cmds.append(Command('RL' if rot > 0 else 'RR', abs(rot)))

    return cmds, dist


def _bypass_waypoints(obs: Obstacle) -> list[tuple[float, float]]:
    """8 candidate bypass points around an obstacle (4 corners + 4 side midpoints)."""
    c = _ROBOT_CLEARANCE
    x, y, half = obs.x, obs.y, CELL_CM / 2
    return [
        (x + CELL_CM + c, y + CELL_CM + c),  # NE
        (x - c,           y + CELL_CM + c),  # NW
        (x + CELL_CM + c, y - c),             # SE
        (x - c,           y - c),             # SW
        (x + half,        y + CELL_CM + c),   # N
        (x + half,        y - c),             # S
        (x + CELL_CM + c, y + half),          # E
        (x - c,           y + half),          # W
    ]



def _plan_leg(
    q1: RobotState,
    q2: RobotState,
    obstacles: list[Obstacle] | None = None,
) -> tuple[list[Command], float]:
    """Plan a collision-free straight-line path from q1 to q2.

    Pass 1: direct route (rotate → FW → rotate).
    Pass 2: if blocked, try 8 bypass waypoints per obstacle.
    Fallback: direct route even if it clips an obstacle.
    """
    obs_list = obstacles or []

    cmds, dist = _direct_leg(q1, q2)
    if not obs_list or _path_in_bounds(q1, cmds, obs_list):
        return cmds, dist

    best_cmds: list[Command] | None = None
    best_dist = float('inf')

    for obs in obs_list:
        for wx, wy in _bypass_waypoints(obs):
            if not (0 <= wx <= ARENA_CM and 0 <= wy <= ARENA_CM):
                continue
            dx1, dy1 = wx - q1.x, wy - q1.y
            dx2, dy2 = q2.x - wx, q2.y - wy
            d1, d2 = math.hypot(dx1, dy1), math.hypot(dx2, dy2)
            if d1 < 0.01 or d2 < 0.01:
                continue

            h1 = math.degrees(math.atan2(dy1, dx1)) % 360
            h2 = math.degrees(math.atan2(dy2, dx2)) % 360

            seg: list[Command] = []
            rot1 = _angle_diff(q1.theta, h1)
            if abs(rot1) > 0.01:
                seg.append(Command('RL' if rot1 > 0 else 'RR', abs(rot1)))
            seg.append(Command('FW', d1))
            rot2 = _angle_diff(h1, h2)
            if abs(rot2) > 0.01:
                seg.append(Command('RL' if rot2 > 0 else 'RR', abs(rot2)))
            seg.append(Command('FW', d2))
            rot3 = _angle_diff(h2, q2.theta)
            if abs(rot3) > 0.01:
                seg.append(Command('RL' if rot3 > 0 else 'RR', abs(rot3)))

            total = d1 + d2
            if total < best_dist and _path_in_bounds(q1, seg, obs_list):
                best_dist = total
                best_cmds = seg

    if best_cmds is not None:
        return best_cmds, best_dist

    return _direct_leg(q1, q2)


def _total_straight_length(start: RobotState, poses: list[RobotState]) -> float:
    total = 0.0
    current = start
    for pose in poses:
        total += math.hypot(pose.x - current.x, pose.y - current.y)
        current = pose
    return total


def _hamiltonian_optimal_order(
    start: RobotState,
    poses: list[RobotState],
) -> list[RobotState]:
    best: list[RobotState] = []
    best_len = float('inf')
    for perm in itertools.permutations(poses):
        length = _total_straight_length(start, list(perm))
        if length < best_len:
            best_len = length
            best = list(perm)
    return best


def get_top_n_routes(
    obstacles: list[Obstacle],
    n: int = 5,
) -> list[tuple[list[Command], float]]:
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    obs_poses = [(obs, obstacle_approach_pose(obs)) for obs in obstacles]
    poses = [p for _, p in obs_poses]

    ranked: list[tuple[float, list[RobotState]]] = []
    for perm in itertools.permutations(poses):
        length = _total_straight_length(start, list(perm))
        ranked.append((length, list(perm)))
    ranked.sort(key=lambda x: x[0])

    routes: list[tuple[list[Command], float]] = []
    for _, ordered_poses in ranked[:n]:
        cmds: list[Command] = []
        total_actual = 0.0
        current = start
        for pose in ordered_poses:
            target_obs = next(obs for obs, p in obs_poses if p.x == pose.x and p.y == pose.y)
            other_obstacles = [o for o in obstacles if o is not target_obs]
            leg_cmds, leg_len = _plan_leg(current, pose, other_obstacles)
            cmds += leg_cmds
            cmds.append(Command('WAIT', 5.0 * FPS))
            total_actual += leg_len
            current = pose
        routes.append((cmds, total_actual))

    routes.sort(key=lambda x: x[1])
    return routes


def get_commands(obstacles: list[Obstacle]) -> list[Command]:
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    obs_poses = [(obs, obstacle_approach_pose(obs)) for obs in obstacles]
    poses = [p for _, p in obs_poses]
    ordered_poses = _hamiltonian_optimal_order(start, poses)
    current = start
    cmds: list[Command] = []
    for pose in ordered_poses:
        target_obs = next(obs for obs, p in obs_poses if p.x == pose.x and p.y == pose.y)
        other_obstacles = [o for o in obstacles if o is not target_obs]
        leg_cmds, _ = _plan_leg(current, pose, other_obstacles)
        cmds += leg_cmds
        cmds.append(Command('WAIT', 5.0 * FPS))
        current = pose
    return cmds
