import itertools
import math
import random as _random

from simulator.config import APPROACH_CM, ARENA_CM, CELL_CM, FPS, GRID_SIZE, ROBOT_W_CM, START_THETA, START_X_CM, START_Y_CM, TURN_RADIUS_CM
from simulator.dubins import dubins_lrl, dubins_lsl, dubins_lsr, dubins_rlr, dubins_rsl, dubins_rsr
from simulator.types import Command, DubinsPath, Obstacle, RobotState

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
    m = TURN_RADIUS_CM + 5           # 30 cm boundary margin
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


_SEGMENT_KINDS: dict[str, tuple[str, str, str]] = {
    'LSL': ('AL', 'FW', 'AL'),
    'LSR': ('AL', 'FW', 'AR'),
    'RSL': ('AR', 'FW', 'AL'),
    'RSR': ('AR', 'FW', 'AR'),
    'LRL': ('AL', 'AR', 'AL'),
    'RLR': ('AR', 'AL', 'AR'),
}

_ROBOT_CLEARANCE = 20.0          # cm clearance from every obstacle cell edge
_BACKUP_DISTANCES = (25, 40, 60, 80)  # cm to try reversing before forward Dubins


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
    r: float,
    obstacles: list[Obstacle] | None = None,
) -> bool:
    """Sample at 2 cm intervals; return True iff every point is inside the arena and clear of obstacles.
    Handles FW, BW, AL, AR commands."""
    x, y, theta = q1.x, q1.y, q1.theta
    step = 2.0
    obs_list = obstacles or []
    for cmd in cmds:
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
            if obs_list and _point_hits_obstacle(x, y, obs_list):
                return False
    return True


def dubins_to_commands(path: DubinsPath) -> list[Command]:
    k1, k2, k3 = _SEGMENT_KINDS[path.path_type]
    cmds = []
    for kind, seg in zip((k1, k2, k3), (path.seg1, path.seg2, path.seg3)):
        if seg > 0.01:
            cmds.append(Command(kind, seg))
    return cmds


def _all_dubins(q1: RobotState, q2: RobotState, r: float) -> list[DubinsPath]:
    """Return all valid Dubins path types sorted shortest first."""
    return sorted(
        (c for c in [
            dubins_lsl(q1, q2, r), dubins_rsr(q1, q2, r),
            dubins_lsr(q1, q2, r), dubins_rsl(q1, q2, r),
            dubins_rlr(q1, q2, r), dubins_lrl(q1, q2, r),
        ] if c is not None),
        key=lambda p: p.total,
    )


def _plan_leg(
    q1: RobotState,
    q2: RobotState,
    r: float,
    obstacles: list[Obstacle] | None = None,
) -> tuple[list[Command], float]:
    """Plan the best collision-free path from q1 to q2.

    Strategy:
      1. Try all 6 forward Dubins types (shortest first).
      2. If none clear, try reversing N cm then forward Dubins (for each backup distance).
      3. Fall back to shortest forward Dubins if everything fails.

    Returns (commands, total_distance_cm).
    """
    candidates = _all_dubins(q1, q2, r)

    # ── Pass 1: forward-only Dubins ──────────────────────────────────────────
    for path in candidates:
        cmds = dubins_to_commands(path)
        if _path_in_bounds(q1, cmds, r, obstacles):
            return cmds, path.total

    # ── Pass 2: reverse N cm then forward Dubins ─────────────────────────────
    rad = math.radians(q1.theta)
    for backup in _BACKUP_DISTANCES:
        q_back = RobotState(
            x=q1.x - backup * math.cos(rad),
            y=q1.y - backup * math.sin(rad),
            theta=q1.theta,
        )
        if not (0 <= q_back.x <= ARENA_CM and 0 <= q_back.y <= ARENA_CM):
            continue

        bw_cmd = Command('BW', float(backup))
        # Verify the backup leg itself is clear
        if obstacles and not _path_in_bounds(q1, [bw_cmd], r, obstacles):
            continue

        for path in _all_dubins(q_back, q2, r):
            fwd_cmds = dubins_to_commands(path)
            all_cmds = [bw_cmd] + fwd_cmds
            if _path_in_bounds(q1, all_cmds, r, obstacles):
                return all_cmds, backup + path.total

    # ── Fallback: shortest forward path (may still clip an obstacle) ─────────
    best = candidates[0]
    return dubins_to_commands(best), best.total


def _dubins_bounded(
    q1: RobotState,
    q2: RobotState,
    r: float,
    obstacles: list[Obstacle] | None = None,
) -> DubinsPath:
    """Forward-only bounded Dubins path used for Hamiltonian ordering cost.
    Returns shortest valid path; falls back to shortest if none are clear."""
    candidates = _all_dubins(q1, q2, r)
    for path in candidates:
        if _path_in_bounds(q1, dubins_to_commands(path), r, obstacles):
            return path
    return candidates[0]


def _total_dubins_length(
    start: RobotState,
    poses: list[RobotState],
    r: float,
    obstacles: list[Obstacle] | None = None,
) -> float:
    total = 0.0
    current = start
    for pose in poses:
        total += _dubins_bounded(current, pose, r, obstacles).total
        current = pose
    return total


def _hamiltonian_optimal_order(
    start: RobotState,
    poses: list[RobotState],
    r: float,
    obstacles: list[Obstacle] | None = None,
) -> list[RobotState]:
    best: list[RobotState] = []
    best_len = float('inf')
    for perm in itertools.permutations(poses):
        length = _total_dubins_length(start, list(perm), r, obstacles)
        if length < best_len:
            best_len = length
            best = list(perm)
    return best


def get_top_n_routes(
    obstacles: list[Obstacle],
    n: int = 5,
) -> list[tuple[list[Command], float]]:
    """Return the N shortest routes as (commands, total_length_cm) pairs, best first."""
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    obs_poses = [(obs, obstacle_approach_pose(obs)) for obs in obstacles]
    poses = [p for _, p in obs_poses]

    ranked: list[tuple[float, list[RobotState]]] = []
    for perm in itertools.permutations(poses):
        length = _total_dubins_length(start, list(perm), TURN_RADIUS_CM, obstacles)
        ranked.append((length, list(perm)))
    ranked.sort(key=lambda x: x[0])

    routes: list[tuple[list[Command], float]] = []
    for total_len, ordered_poses in ranked[:n]:
        cmds: list[Command] = []
        total_actual = 0.0
        current = start
        for pose in ordered_poses:
            target_obs = next(obs for obs, p in obs_poses if p.x == pose.x and p.y == pose.y)
            other_obstacles = [o for o in obstacles if o is not target_obs]
            leg_cmds, leg_len = _plan_leg(current, pose, TURN_RADIUS_CM, other_obstacles)
            cmds += leg_cmds
            cmds.append(Command('WAIT', 5.0 * FPS))
            total_actual += leg_len
            current = pose
        routes.append((cmds, total_actual))

    # Re-sort by actual planned length (may differ from heuristic ordering cost)
    routes.sort(key=lambda x: x[1])
    return routes


def get_commands(obstacles: list[Obstacle]) -> list[Command]:
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    obs_poses = [(obs, obstacle_approach_pose(obs)) for obs in obstacles]
    poses = [p for _, p in obs_poses]
    ordered_poses = _hamiltonian_optimal_order(start, poses, TURN_RADIUS_CM, obstacles)
    current = start
    cmds: list[Command] = []
    for pose in ordered_poses:
        target_obs = next(obs for obs, p in obs_poses if p.x == pose.x and p.y == pose.y)
        other_obstacles = [o for o in obstacles if o is not target_obs]
        leg_cmds, _ = _plan_leg(current, pose, TURN_RADIUS_CM, other_obstacles)
        cmds += leg_cmds
        cmds.append(Command('WAIT', 5.0 * FPS))
        current = pose
    return cmds
