import itertools
import math
import random as _random

from app_config import DEFAULT_OBSTACLES
from simulator.config import (
    APPROACH_CM,
    ARENA_CM,
    CELL_CM,
    FPS,
    GRID_SIZE,
    MAX_APPROACH_CM,
    MIN_APPROACH_CM,
    START_THETA,
    START_X_CM,
    START_Y_CM,
    WALL_MARGIN_CM,
)
from simulator.types import Command, Obstacle, RobotState

OBSTACLES: list[Obstacle] = DEFAULT_OBSTACLES

def _valid_faces(col: int, row: int) -> list[str]:
    """Return face directions whose grid-aligned approach pose fits inside the arena with ≥30 cm boundary margin."""
    ox = col * CELL_CM
    oy = row * CELL_CM
    m = 30.0
    faces: list[str] = []
    if m <= ox <= ARENA_CM - m and m <= oy + CELL_CM + APPROACH_CM <= ARENA_CM - m:
        faces.append('N')
    if m <= ox <= ARENA_CM - m and m <= oy - APPROACH_CM <= ARENA_CM - m:
        faces.append('S')
    if m <= ox + CELL_CM + APPROACH_CM <= ARENA_CM - m and m <= oy <= ARENA_CM - m:
        faces.append('E')
    if m <= ox - APPROACH_CM <= ARENA_CM - m and m <= oy <= ARENA_CM - m:
        faces.append('W')
    return faces


def generate_random_obstacles(n: int = 5, seed: int | None = None) -> list[Obstacle]:
    """Generate n random obstacles with valid grid-aligned approach poses.

    Guarantees:
    - No obstacle in the 40×40 cm start zone (bottom-left 4×4 cells).
    - Every approach pose fits in the arena with ≥30 cm boundary clearance.
    - Obstacles are at least 6 cells apart (Chebyshev distance ≥ 6): each
      obstacle's 50 cm clearance zone (20 cm each side + 10 cm cell) is then
      separated by ≥10 cm from every neighbour's zone, ensuring the car can
      always navigate between any pair of obstacles.
    """
    rng = _random.Random(seed)

    pool: list[tuple[int, int, list[str]]] = []
    for col in range(GRID_SIZE):
        for row in range(GRID_SIZE):
            if col < 5 and row < 6:        # exclude start zone + one row clearance buffer
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
        if any(max(abs(col - c), abs(row - r)) < 6 for c, r in used):
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


def obstacle_approach_pose(obs: Obstacle, approach_cm: float = APPROACH_CM) -> RobotState:
    """Grid-aligned approach pose: apex on a grid-line intersection in front of the obstacle face, `approach_cm` from it."""
    if obs.face == 'N':
        return RobotState(x=obs.x, y=obs.y + CELL_CM + approach_cm, theta=270)
    if obs.face == 'S':
        return RobotState(x=obs.x, y=obs.y - approach_cm, theta=90)
    if obs.face == 'E':
        return RobotState(x=obs.x + CELL_CM + approach_cm, y=obs.y, theta=180)
    # face == 'W'
    return RobotState(x=obs.x - approach_cm, y=obs.y, theta=0)


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
    """Sample FW/BW at 2 cm intervals; RL/RR update heading only. True iff path
    stays WALL_MARGIN_CM clear of every arena wall (a ground ruler runs along
    the perimeter that a too-close turn could catch) and clear of obstacles."""
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
            if not (WALL_MARGIN_CM <= x <= ARENA_CM - WALL_MARGIN_CM
                    and WALL_MARGIN_CM <= y <= ARENA_CM - WALL_MARGIN_CM):
                return False
            if obs_list and _point_hits_obstacle(x, y, obs_list):
                return False
    return True


def _grid_leg(q1: RobotState, q2: RobotState, horizontal_first: bool = True) -> tuple[list[Command], float]:
    """Grid-aligned L-path: one horizontal segment then one vertical (or reversed).

    Rotates to a cardinal direction (0/90/180/270) before each FW segment.
    Returns (commands, manhattan_distance).
    """
    dx = q2.x - q1.x
    dy = q2.y - q1.y
    cmds: list[Command] = []
    dist = abs(dx) + abs(dy)

    if abs(dx) < 0.01 and abs(dy) < 0.01:
        rot = _angle_diff(q1.theta, q2.theta)
        if abs(rot) > 0.01:
            cmds.append(Command('RL' if rot > 0 else 'RR', abs(rot)))
        return cmds, 0.0

    h_heading = 0.0 if dx > 0 else 180.0   # East / West
    v_heading = 90.0 if dy > 0 else 270.0  # North / South

    segs: list[tuple[float, float]] = []
    if horizontal_first:
        if abs(dx) > 0.01:
            segs.append((h_heading, abs(dx)))
        if abs(dy) > 0.01:
            segs.append((v_heading, abs(dy)))
    else:
        if abs(dy) > 0.01:
            segs.append((v_heading, abs(dy)))
        if abs(dx) > 0.01:
            segs.append((h_heading, abs(dx)))

    current_theta = q1.theta
    for heading, length in segs:
        rot = _angle_diff(current_theta, heading)
        if abs(rot) > 0.01:
            cmds.append(Command('RL' if rot > 0 else 'RR', abs(rot)))
        cmds.append(Command('FW', length))
        current_theta = heading

    rot = _angle_diff(current_theta, q2.theta)
    if abs(rot) > 0.01:
        cmds.append(Command('RL' if rot > 0 else 'RR', abs(rot)))

    return cmds, dist


def _plan_leg(
    q1: RobotState,
    q2: RobotState,
    obstacles: list[Obstacle] | None = None,
) -> tuple[list[Command], float]:
    """Plan a grid-aligned L-path from q1 to q2.

    Tries H-first, then V-first, then a two-segment detour via a grid
    intersection chosen to route around blocking obstacles.
    """
    obs_list = obstacles or []

    cmds_h, dist = _grid_leg(q1, q2, horizontal_first=True)
    if not obs_list or _path_in_bounds(q1, cmds_h, obs_list):
        return cmds_h, dist

    cmds_v, _ = _grid_leg(q1, q2, horizontal_first=False)
    if _path_in_bounds(q1, cmds_v, obs_list):
        return cmds_v, dist

    # Both simple L-paths blocked — search for a two-leg detour via an
    # intermediate grid-line intersection that bypasses blocking obstacles.
    # Candidate x/y values: endpoints plus grid lines just outside each
    # obstacle's clearance zone (the zone ends strictly at obs.x±20/+30).
    candidate_xs: set[float] = {q1.x, q2.x}
    candidate_ys: set[float] = {q1.y, q2.y}
    for obs in obs_list:
        for cx in (obs.x - 2 * CELL_CM, obs.x + 3 * CELL_CM):
            if WALL_MARGIN_CM <= cx <= ARENA_CM - WALL_MARGIN_CM:
                candidate_xs.add(cx)
        for cy in (obs.y - 2 * CELL_CM, obs.y + 3 * CELL_CM):
            if WALL_MARGIN_CM <= cy <= ARENA_CM - WALL_MARGIN_CM:
                candidate_ys.add(cy)

    best_cmds: list[Command] = cmds_h  # last-resort fallback
    best_dist = float('inf')

    for wx in sorted(candidate_xs):
        for wy in sorted(candidate_ys):
            if abs(wx - q1.x) < 0.01 and abs(wy - q1.y) < 0.01:
                continue
            if abs(wx - q2.x) < 0.01 and abs(wy - q2.y) < 0.01:
                continue
            for wtheta in (0.0, 90.0, 180.0, 270.0):
                w = RobotState(x=wx, y=wy, theta=wtheta)
                for h1 in (True, False):
                    cmds1, d1 = _grid_leg(q1, w, horizontal_first=h1)
                    if not _path_in_bounds(q1, cmds1, obs_list):
                        continue
                    for h2 in (True, False):
                        cmds2, d2 = _grid_leg(w, q2, horizontal_first=h2)
                        if not _path_in_bounds(w, cmds2, obs_list):
                            continue
                        total = d1 + d2
                        if total < best_dist:
                            best_dist = total
                            best_cmds = cmds1 + cmds2

    return best_cmds, (best_dist if best_dist < float('inf') else dist)


_APPROACH_STEP_CM = 1.0  # search granularity within [MIN_APPROACH_CM, MAX_APPROACH_CM]


def _best_leg_to_obstacle(
    current: RobotState,
    obs: Obstacle,
    obstacles: list[Obstacle],
) -> tuple[list[Command], float, RobotState]:
    """Plan the leg from `current` to `obs`'s target face, choosing the
    shortest approach distance in [MIN_APPROACH_CM, MAX_APPROACH_CM] that
    produces a collision-free, in-bounds path. `obs` itself is excluded from
    collision checking for this leg — its own clearance zone would otherwise
    block getting closer than _ROBOT_CLEARANCE (20cm) — every other obstacle
    and the wall margin are still enforced. Returns (leg_cmds, leg_len,
    approach_pose actually used). Falls back to the MAX_APPROACH_CM attempt
    if no distance in range validates (better to be far than to report
    nothing)."""
    other_obstacles = [o for o in obstacles if o is not obs]
    steps = max(1, round((MAX_APPROACH_CM - MIN_APPROACH_CM) / _APPROACH_STEP_CM))
    fallback: tuple[list[Command], float, RobotState] | None = None
    for i in range(steps + 1):
        approach_cm = MIN_APPROACH_CM + i * (MAX_APPROACH_CM - MIN_APPROACH_CM) / steps
        pose = obstacle_approach_pose(obs, approach_cm)
        leg_cmds, leg_len = _plan_leg(current, pose, other_obstacles)
        if fallback is None:
            fallback = (leg_cmds, leg_len, pose)
        if _path_in_bounds(current, leg_cmds, other_obstacles):
            return leg_cmds, leg_len, pose
    return fallback


def _total_manhattan_length(start: RobotState, poses: list[RobotState]) -> float:
    total = 0.0
    current = start
    for pose in poses:
        total += abs(pose.x - current.x) + abs(pose.y - current.y)
        current = pose
    return total


def _hamiltonian_optimal_order(
    start: RobotState,
    poses: list[RobotState],
) -> list[RobotState]:
    best: list[RobotState] = []
    best_len = float('inf')
    for perm in itertools.permutations(poses):
        length = _total_manhattan_length(start, list(perm))
        if length < best_len:
            best_len = length
            best = list(perm)
    return best


def get_top_n_routes(
    obstacles: list[Obstacle],
    n: int = 5,
    start: RobotState | None = None,
) -> list[tuple[list[Command], float]]:
    if start is None:
        start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    obs_poses = [(obs, obstacle_approach_pose(obs)) for obs in obstacles]
    poses = [p for _, p in obs_poses]

    ranked: list[tuple[float, list[RobotState]]] = []
    for perm in itertools.permutations(poses):
        length = _total_manhattan_length(start, list(perm))
        ranked.append((length, list(perm)))
    ranked.sort(key=lambda x: x[0])

    obs_by_pose = {id(pose): obs for obs, pose in obs_poses}

    routes: list[tuple[list[Command], float]] = []
    for _, ordered_poses in ranked[:n]:
        cmds: list[Command] = []
        total_actual = 0.0
        current = start
        for pose in ordered_poses:
            obs = obs_by_pose[id(pose)]
            leg_cmds, leg_len, actual_pose = _best_leg_to_obstacle(current, obs, obstacles)
            cmds += leg_cmds
            cmds.append(Command('WAIT', 5.0 * FPS))
            total_actual += leg_len
            current = actual_pose
        routes.append((cmds, total_actual))

    routes.sort(key=lambda x: x[1])
    return routes


def get_commands(obstacles: list[Obstacle], start: RobotState | None = None) -> list[Command]:
    if start is None:
        start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    obs_poses = [(obs, obstacle_approach_pose(obs)) for obs in obstacles]
    poses = [p for _, p in obs_poses]
    obs_by_pose = {id(pose): obs for obs, pose in obs_poses}
    ordered_poses = _hamiltonian_optimal_order(start, poses)
    current = start
    cmds: list[Command] = []
    for pose in ordered_poses:
        obs = obs_by_pose[id(pose)]
        leg_cmds, _, actual_pose = _best_leg_to_obstacle(current, obs, obstacles)
        cmds += leg_cmds
        cmds.append(Command('WAIT', 5.0 * FPS))
        current = actual_pose
    return cmds
