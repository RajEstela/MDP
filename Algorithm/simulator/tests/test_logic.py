import math

from simulator.arena import cm_to_px
from simulator.dubins import dubins_lrl, dubins_lsl, dubins_lsr, dubins_optimal, dubins_rlr, dubins_rsl, dubins_rsr
from simulator.config import START_THETA, START_X_CM, START_Y_CM
from simulator.planner import OBSTACLES, get_commands, get_top_n_routes, generate_random_obstacles, obstacle_approach_pose, _hamiltonian_optimal_order, _angle_diff, _plan_leg, _path_in_bounds
from simulator.robot import arc_step, move_forward, rotate, step_command
from simulator.types import Command, DubinsPath, Obstacle, RobotState


def _sim_dubins_path(q1: RobotState, path, r: float = 25) -> RobotState:
    """Simulate a DubinsPath directly using arc_step/move_forward (bypasses step_command)."""
    kind_map = {
        'LSL': ('AL', 'FW', 'AL'), 'LSR': ('AL', 'FW', 'AR'),
        'RSL': ('AR', 'FW', 'AL'), 'RSR': ('AR', 'FW', 'AR'),
        'LRL': ('AL', 'AR', 'AL'), 'RLR': ('AR', 'AL', 'AR'),
    }
    seg_kinds = kind_map[path.path_type]
    state = q1
    for kind, seg_len in zip(seg_kinds, [path.seg1, path.seg2, path.seg3]):
        remaining = seg_len
        while remaining > 0.001:
            adv = min(2.0, remaining)
            if kind == 'FW':
                state = move_forward(state, adv)
            elif kind == 'AL':
                state = arc_step(state, adv, clockwise=False, r=r)
            elif kind == 'AR':
                state = arc_step(state, adv, clockwise=True, r=r)
            remaining -= adv
    return state


def test_robotstate_fields():
    s = RobotState(x=0.0, y=0.0, theta=90.0)
    assert s.x == 0.0
    assert s.y == 0.0
    assert s.theta == 90.0


def test_obstacle_fields():
    o = Obstacle(x=50, y=50, face='N')
    assert o.x == 50
    assert o.face == 'N'


def test_command_fields():
    c = Command(kind='FW', value=40.0)
    assert c.kind == 'FW'
    assert c.value == 40.0


def test_move_forward_north():
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    result = move_forward(state, 10.0)
    assert abs(result.x) < 0.001
    assert abs(result.y - 10.0) < 0.001
    assert result.theta == 90.0


def test_move_forward_east():
    state = RobotState(x=0.0, y=0.0, theta=0.0)
    result = move_forward(state, 10.0)
    assert abs(result.x - 10.0) < 0.001
    assert abs(result.y) < 0.001


def test_move_backward_north():
    state = RobotState(x=0.0, y=50.0, theta=90.0)
    result = move_forward(state, -10.0)
    assert abs(result.y - 40.0) < 0.001


def test_rotate_left_increases_theta():
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    result = rotate(state, 90.0, clockwise=False)
    assert abs(result.theta - 180.0) < 0.001


def test_rotate_right_decreases_theta():
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    result = rotate(state, 90.0, clockwise=True)
    assert abs(result.theta - 0.0) < 0.001


def test_rotate_wraps_below_zero():
    state = RobotState(x=0.0, y=0.0, theta=10.0)
    result = rotate(state, 20.0, clockwise=True)
    assert abs(result.theta - 350.0) < 0.001


def test_step_command_fw_advances_position():
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    cmd = Command(kind='FW', value=10.0)
    new_state, remaining = step_command(state, cmd, 10.0)
    assert remaining < 10.0
    assert new_state.y > 0.0


def test_step_command_bw_retreats_position():
    state = RobotState(x=0.0, y=50.0, theta=90.0)
    cmd = Command(kind='BW', value=10.0)
    new_state, remaining = step_command(state, cmd, 10.0)
    assert new_state.y < 50.0


def test_step_command_rr_reduces_theta():
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    cmd = Command(kind='RR', value=90.0)
    new_state, remaining = step_command(state, cmd, 90.0)
    assert remaining < 90.0
    assert new_state.theta < 90.0


def test_step_command_rl_increases_theta():
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    cmd = Command(kind='RL', value=90.0)
    new_state, remaining = step_command(state, cmd, 90.0)
    assert new_state.theta > 90.0


def test_cm_to_px_origin():
    # Bottom-left of arena (0cm, 0cm) → pygame bottom-left = (0, 800)
    assert cm_to_px(0, 0) == (0, 800)


def test_cm_to_px_top_left():
    # Top-left of arena (0cm, 200cm) → pygame top-left = (0, 0)
    assert cm_to_px(0, 200) == (0, 0)


def test_cm_to_px_bottom_right():
    # Bottom-right of arena (200cm, 0cm) → pygame bottom-right = (800, 800)
    assert cm_to_px(200, 0) == (800, 800)


def test_cm_to_px_center():
    assert cm_to_px(100, 100) == (400, 400)


def test_obstacles_count():
    assert len(OBSTACLES) == 5


def test_obstacles_valid_faces():
    valid = {'N', 'S', 'E', 'W'}
    assert all(o.face in valid for o in OBSTACLES)


def test_get_commands_non_empty():
    cmds = get_commands(OBSTACLES)
    assert len(cmds) > 0


def test_get_commands_all_valid_kinds():
    cmds = get_commands(OBSTACLES)
    valid = {'FW', 'BW', 'RL', 'RR', 'WAIT'}
    assert all(c.kind in valid for c in cmds)


def test_get_commands_positive_values():
    cmds = get_commands(OBSTACLES)
    assert all(c.value > 0 for c in cmds)


# ── Task 1: DubinsPath + arc_step ──────────────────────────────────────────

def test_dubins_path_fields():
    p = DubinsPath(path_type='LSL', seg1=10.0, seg2=20.0, seg3=30.0, total=60.0)
    assert p.path_type == 'LSL'
    assert p.seg1 == 10.0
    assert p.total == 60.0


def test_arc_step_left_quarter_circle():
    state = RobotState(0, 0, 0)
    result = arc_step(state, ds=math.pi / 2 * 25, clockwise=False, r=25)
    assert abs(result.x - 25) < 0.01
    assert abs(result.y - 25) < 0.01
    assert abs(result.theta - 90) < 0.01


def test_arc_step_right_quarter_circle():
    state = RobotState(0, 0, 0)
    result = arc_step(state, ds=math.pi / 2 * 25, clockwise=True, r=25)
    assert abs(result.x - 25) < 0.01
    assert abs(result.y + 25) < 0.01
    assert abs(result.theta - 270) < 0.01


def test_arc_step_full_circle_returns_to_origin():
    state = RobotState(10, 20, 45)
    result = arc_step(state, ds=2 * math.pi * 30, clockwise=False, r=30)
    assert abs(result.x - 10) < 0.1
    assert abs(result.y - 20) < 0.1
    assert abs(result.theta - 45) < 0.1


# ── Task 2: dubins.py ──────────────────────────────────────────────────────

def test_dubins_straight_line():
    # Same heading, target directly ahead — should be pure straight, ~zero arcs
    q1 = RobotState(0, 0, 0)
    q2 = RobotState(100, 0, 0)
    path = dubins_optimal(q1, q2, r=25)
    assert abs(path.total - 100) < 0.1


def test_dubins_optimal_returns_shortest():
    q1 = RobotState(0, 0, 0)
    q2 = RobotState(50, 50, 90)
    path = dubins_optimal(q1, q2, r=25)
    for fn in [dubins_lsl, dubins_rsr, dubins_lsr, dubins_rsl, dubins_rlr, dubins_lrl]:
        candidate = fn(q1, q2, r=25)
        if candidate is not None:
            assert path.total <= candidate.total + 0.001


def test_dubins_lsl_same_start_end():
    # Zero-displacement: all segments should be zero (or path total ~0)
    q = RobotState(0, 0, 0)
    path = dubins_lsl(q, q, r=25)
    assert path is not None
    assert path.total < 0.01


def test_dubins_path_type_is_correct_string():
    q1 = RobotState(0, 0, 0)
    q2 = RobotState(50, 50, 90)
    valid_types = {'LSL', 'LSR', 'RSL', 'RSR', 'RLR', 'LRL'}
    path = dubins_optimal(q1, q2, r=25)
    assert path.path_type in valid_types


def test_dubins_total_equals_sum_of_segs():
    q1 = RobotState(45, 0, 45)
    q2 = RobotState(80, 60, 135)
    path = dubins_optimal(q1, q2, r=25)
    assert abs(path.total - (path.seg1 + path.seg2 + path.seg3)) < 0.001


def test_dubins_segments_non_negative():
    q1 = RobotState(10, 30, 180)
    q2 = RobotState(90, 10, 270)
    path = dubins_optimal(q1, q2, r=25)
    assert path.seg1 >= 0
    assert path.seg2 >= 0
    assert path.seg3 >= 0


def test_get_commands_produces_arc_commands():
    cmds = get_commands(OBSTACLES)
    kinds = {c.kind for c in cmds}
    assert kinds <= {'FW', 'BW', 'RL', 'RR', 'WAIT'}
    assert 'RL' in kinds or 'RR' in kinds


def test_get_commands_all_values_positive():
    cmds = get_commands(OBSTACLES)
    assert all(c.value > 0 for c in cmds)


def test_dubins_path_reaches_target():
    q1 = RobotState(0, 0, 90)
    q2 = RobotState(100, 100, 0)
    path = dubins_optimal(q1, q2, r=25)
    state = _sim_dubins_path(q1, path)
    assert abs(state.x - q2.x) < 0.5
    assert abs(state.y - q2.y) < 0.5
    assert abs((state.theta - q2.theta + 180) % 360 - 180) < 1.0


# ── RLR/LRL endpoint validation ─────────────────────────────────────────────

def test_dubins_rlr_endpoint():
    q1 = RobotState(0, 0, 315)
    q2 = RobotState(30, 0, 135)
    r = 25
    path = dubins_rlr(q1, q2, r)
    assert path is not None, "RLR path should be feasible for these waypoints"
    assert path.path_type == 'RLR'
    state = _sim_dubins_path(q1, path, r)
    assert abs(state.x - q2.x) < 0.5, f"x off by {abs(state.x - q2.x):.3f} cm"
    assert abs(state.y - q2.y) < 0.5, f"y off by {abs(state.y - q2.y):.3f} cm"
    assert abs((state.theta - q2.theta + 180) % 360 - 180) < 1.0


def test_dubins_lrl_endpoint():
    q1 = RobotState(0, 0, 45)
    q2 = RobotState(30, 0, 225)
    r = 25
    path = dubins_lrl(q1, q2, r)
    assert path is not None, "LRL path should be feasible for these waypoints"
    assert path.path_type == 'LRL'
    state = _sim_dubins_path(q1, path, r)
    assert abs(state.x - q2.x) < 0.5, f"x off by {abs(state.x - q2.x):.3f} cm"
    assert abs(state.y - q2.y) < 0.5, f"y off by {abs(state.y - q2.y):.3f} cm"
    assert abs((state.theta - q2.theta + 180) % 360 - 180) < 1.0


# ── Task 3: Stage 3 obstacle approach ──────────────────────────────────────


def test_approach_pose_north():
    obs = Obstacle(x=50, y=50, face='N')
    pose = obstacle_approach_pose(obs)
    assert abs(pose.x - 50) < 0.01   # grid-aligned: obs.x (left edge)
    assert abs(pose.y - 80) < 0.01   # obs.y + CELL_CM + APPROACH_CM = 50+10+20
    assert abs(pose.theta - 270) < 0.01


def test_approach_pose_south():
    obs = Obstacle(x=50, y=50, face='S')
    pose = obstacle_approach_pose(obs)
    assert abs(pose.x - 50) < 0.01   # grid-aligned: obs.x
    assert abs(pose.y - 30) < 0.01   # obs.y - APPROACH_CM = 50-20
    assert abs(pose.theta - 90) < 0.01


def test_approach_pose_east():
    obs = Obstacle(x=50, y=50, face='E')
    pose = obstacle_approach_pose(obs)
    assert abs(pose.x - 80) < 0.01   # obs.x + CELL_CM + APPROACH_CM = 50+10+20
    assert abs(pose.y - 50) < 0.01   # grid-aligned: obs.y (bottom edge)
    assert abs(pose.theta - 180) < 0.01


def test_approach_pose_west():
    obs = Obstacle(x=50, y=50, face='W')
    pose = obstacle_approach_pose(obs)
    assert abs(pose.x - 30) < 0.01   # obs.x - APPROACH_CM = 50-20
    assert abs(pose.y - 50) < 0.01   # grid-aligned: obs.y
    assert abs(pose.theta - 0) < 0.01


def test_approach_pose_robot_faces_obstacle():
    # Robot heading must point FROM robot position TOWARD the obstacle face.
    # For face='E', robot is east of obstacle, theta=180 (facing West toward face). ✓
    # Quick sanity: heading is 180° opposite to face direction.
    face_to_heading = {'N': 270, 'S': 90, 'E': 180, 'W': 0}
    for face, expected_theta in face_to_heading.items():
        obs = Obstacle(x=50, y=50, face=face)
        pose = obstacle_approach_pose(obs)
        assert abs(pose.theta - expected_theta) < 0.01, f"face={face}: got theta={pose.theta}, expected {expected_theta}"


# ── Task 2 (Stage 3): _hamiltonian_optimal_order ────────────────────────────

def test_hamiltonian_visits_all_poses():
    start = RobotState(0, 0, 90)
    poses = [RobotState(50, 0, 0), RobotState(100, 0, 0), RobotState(150, 0, 0)]
    result = _hamiltonian_optimal_order(start, poses)
    assert len(result) == 3
    result_coords = {(p.x, p.y) for p in result}
    expected_coords = {(p.x, p.y) for p in poses}
    assert result_coords == expected_coords

def test_hamiltonian_single_pose():
    start = RobotState(0, 0, 90)
    poses = [RobotState(50, 50, 0)]
    result = _hamiltonian_optimal_order(start, poses)
    assert len(result) == 1
    assert result[0].x == 50 and result[0].y == 50

def test_hamiltonian_selects_shorter_order():
    # Start at (0,0,0). Pose B=(10,0,0) is close; Pose A=(100,0,0) is far.
    # Visiting B first (0→10→100) is shorter total than A first (0→100→10→back).
    # Both end at 0°, so the far→near order is always longer.
    start = RobotState(0, 0, 0)
    a = RobotState(100, 0, 0)
    b = RobotState(10, 0, 0)
    result = _hamiltonian_optimal_order(start, [a, b])
    assert result[0].x == b.x

def test_hamiltonian_five_poses_returns_five():
    start = RobotState(0, 0, 90)
    poses = [obstacle_approach_pose(obs) for obs in OBSTACLES]
    result = _hamiltonian_optimal_order(start, poses)
    assert len(result) == 5


# ── Task 3 (Stage 3): get_commands wired ────────────────────────────────────

def test_get_commands_arc_commands_present():
    cmds = get_commands(OBSTACLES)
    assert any(c.kind in ('RL', 'RR') for c in cmds)

def test_get_commands_no_unknown_kinds():
    cmds = get_commands(OBSTACLES)
    valid = {'FW', 'BW', 'RL', 'RR', 'WAIT'}
    assert all(c.kind in valid for c in cmds)

def test_get_commands_reaches_final_approach_pose():
    """Simulate full command sequence; verify robot ends near a valid approach
    pose for one of the obstacles, at any distance in [MIN_APPROACH_CM, MAX_APPROACH_CM]
    (the planner picks the shortest distance that's actually collision-free, not
    always a fixed distance)."""
    import math
    from simulator.planner import MAX_APPROACH_CM, MIN_APPROACH_CM
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    cmds = get_commands(OBSTACLES)
    state = start
    for cmd in cmds:
        remaining = cmd.value
        while remaining > 0.001:
            state, remaining = step_command(state, cmd, remaining)
    n_steps = round(MAX_APPROACH_CM - MIN_APPROACH_CM)
    candidate_poses = [
        obstacle_approach_pose(obs, MIN_APPROACH_CM + i * (MAX_APPROACH_CM - MIN_APPROACH_CM) / n_steps)
        for obs in OBSTACLES
        for i in range(n_steps + 1)
    ]
    closest = min(candidate_poses, key=lambda p: math.hypot(state.x - p.x, state.y - p.y))
    assert math.hypot(state.x - closest.x, state.y - closest.y) < 1.0


# ── LSR/RSL endpoint validation ─────────────────────────────────────────────

def test_dubins_lsr_endpoint():
    q1 = RobotState(0, 0, 0)
    q2 = RobotState(80, 0, 180)
    r = 25
    path = dubins_lsr(q1, q2, r)
    assert path is not None, "LSR path should be feasible for these waypoints"
    assert path.path_type == 'LSR'
    state = _sim_dubins_path(q1, path, r)
    assert abs(state.x - q2.x) < 0.5, f"x off by {abs(state.x - q2.x):.3f} cm"
    assert abs(state.y - q2.y) < 0.5, f"y off by {abs(state.y - q2.y):.3f} cm"
    assert abs((state.theta - q2.theta + 180) % 360 - 180) < 1.0


def test_dubins_rsl_endpoint():
    q1 = RobotState(0, 0, 180)
    q2 = RobotState(80, 0, 0)
    r = 25
    path = dubins_rsl(q1, q2, r)
    assert path is not None, "RSL path should be feasible for these waypoints"
    assert path.path_type == 'RSL'
    state = _sim_dubins_path(q1, path, r)
    assert abs(state.x - q2.x) < 0.5, f"x off by {abs(state.x - q2.x):.3f} cm"
    assert abs(state.y - q2.y) < 0.5, f"y off by {abs(state.y - q2.y):.3f} cm"
    assert abs((state.theta - q2.theta + 180) % 360 - 180) < 1.0


# ── Stage 4: _angle_diff ────────────────────────────────────────────────────

def test_angle_diff_same_heading():
    assert _angle_diff(90, 90) == 0.0

def test_angle_diff_left_90():
    assert abs(_angle_diff(0, 90) - 90) < 0.01

def test_angle_diff_right_90():
    assert abs(_angle_diff(90, 0) - (-90)) < 0.01

def test_angle_diff_shortest_right():
    # 10° to 350°: going right 20° is shorter than going left 340°
    assert abs(_angle_diff(10, 350) - (-20)) < 0.01

def test_angle_diff_shortest_left():
    # 350° to 10°: going left 20° is shorter than going right 340°
    assert abs(_angle_diff(350, 10) - 20) < 0.01

def test_angle_diff_exactly_180():
    # Exactly 180° away returns +180 (left)
    assert abs(_angle_diff(0, 180) - 180) < 0.01


# ── Stage 4: straight-line pathing ─────────────────────────────────────────

def test_get_commands_uses_rl_rr_not_arcs():
    cmds = get_commands(OBSTACLES)
    assert not any(c.kind in ('AL', 'AR') for c in cmds)
    assert any(c.kind in ('RL', 'RR') for c in cmds)

def test_plan_leg_direct_no_arcs():
    q1 = RobotState(x=20, y=30, theta=90)   # grid-aligned start
    q2 = RobotState(x=50, y=80, theta=270)  # grid-aligned end
    cmds, dist = _plan_leg(q1, q2, obstacles=[])
    assert all(c.kind in ('FW', 'BW', 'RL', 'RR', 'WAIT') for c in cmds)
    assert any(c.kind == 'FW' for c in cmds)
    assert dist > 0


def test_plan_leg_grid_aligned():
    q1 = RobotState(x=20, y=30, theta=90)   # facing North
    q2 = RobotState(x=50, y=80, theta=270)  # facing South
    cmds, dist = _plan_leg(q1, q2, obstacles=[])
    # Manhattan distance = |50-20| + |80-30| = 30 + 50 = 80
    assert abs(dist - 80) < 0.01
    # Two FW segments: 30 cm horizontal + 50 cm vertical
    fw_cmds = [c for c in cmds if c.kind == 'FW']
    assert len(fw_cmds) == 2
    fw_values = sorted(c.value for c in fw_cmds)
    assert abs(fw_values[0] - 30) < 0.01
    assert abs(fw_values[1] - 50) < 0.01

def test_path_in_bounds_handles_rl_rr():
    state = RobotState(x=100, y=100, theta=0)
    cmds = [
        Command(kind='RL', value=90),
        Command(kind='FW', value=20),
        Command(kind='RR', value=45),
    ]
    assert _path_in_bounds(state, cmds) is True


def test_path_in_bounds_rejects_path_inside_wall_margin():
    """A ground ruler runs along the arena perimeter — paths must stay
    WALL_MARGIN_CM clear of every wall, not just inside the raw 0..200 arena."""
    from simulator.config import WALL_MARGIN_CM
    state = RobotState(x=WALL_MARGIN_CM + 5, y=100, theta=180)  # facing West
    cmds = [Command(kind='FW', value=10)]  # would land 5cm inside the margin
    assert _path_in_bounds(state, cmds) is False


def test_path_in_bounds_accepts_path_at_wall_margin_boundary():
    from simulator.config import WALL_MARGIN_CM
    state = RobotState(x=WALL_MARGIN_CM + 5, y=100, theta=180)  # facing West
    cmds = [Command(kind='FW', value=5)]  # lands exactly on the margin boundary
    assert _path_in_bounds(state, cmds) is True


# ── Variable approach distance (10-40cm depending on the best route) ───────

def test_best_leg_to_obstacle_prefers_shortest_distance_when_clear():
    from simulator.planner import _best_leg_to_obstacle, MIN_APPROACH_CM
    obs = Obstacle(x=100, y=100, face='N')
    current = RobotState(x=100, y=50, theta=90)
    _, _, pose = _best_leg_to_obstacle(current, obs, [obs])
    expected = obstacle_approach_pose(obs, MIN_APPROACH_CM)
    assert abs(pose.x - expected.x) < 0.01
    assert abs(pose.y - expected.y) < 0.01


def test_best_leg_to_obstacle_picks_larger_distance_when_shortest_is_blocked():
    """A neighboring obstacle's clearance zone blocks approach distances
    10-24cm; the function must skip to the first distance that's actually
    clear (25cm here) instead of returning an invalid short-distance pose."""
    from simulator.planner import _best_leg_to_obstacle, MIN_APPROACH_CM
    obs = Obstacle(x=100, y=100, face='N')
    blocker = Obstacle(x=100, y=105, face='S')
    current = RobotState(x=100, y=50, theta=90)
    leg_cmds, _, pose = _best_leg_to_obstacle(current, obs, [obs, blocker])
    assert pose.y > obstacle_approach_pose(obs, MIN_APPROACH_CM).y
    assert abs(pose.y - 135.0) < 0.01
    assert _path_in_bounds(current, leg_cmds, [blocker])


def test_no_collision_random_arenas():
    """50 random arenas: every planned leg must clear ALL obstacles (including target)."""
    from simulator.config import START_X_CM, START_Y_CM, START_THETA
    for seed in range(50):
        obstacles = generate_random_obstacles(5, seed=seed)
        start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
        obs_poses = [(obs, obstacle_approach_pose(obs)) for obs in obstacles]
        poses = [p for _, p in obs_poses]
        ordered = _hamiltonian_optimal_order(start, poses)
        current = start
        for pose in ordered:
            leg_cmds, _ = _plan_leg(current, pose, obstacles)
            assert _path_in_bounds(current, leg_cmds, obstacles), (
                f"Collision detected: seed={seed}, "
                f"from=({current.x},{current.y}) to=({pose.x},{pose.y})"
            )
            current = pose


# ── Task 3: optional start pose parameter ──────────────────────────────────

def test_get_commands_uses_provided_start():
    from simulator.planner import MIN_APPROACH_CM
    custom_start = RobotState(x=100.0, y=100.0, theta=0.0)
    obs = [Obstacle(x=150, y=100, face='W')]
    cmds = get_commands(obs, start=custom_start)
    state = custom_start
    for cmd in cmds:
        remaining = cmd.value
        while remaining > 0.001:
            state, remaining = step_command(state, cmd, remaining)
    # Nothing blocks this obstacle, so the shortest valid approach (MIN_APPROACH_CM) is used.
    pose = obstacle_approach_pose(obs[0], MIN_APPROACH_CM)
    assert math.hypot(state.x - pose.x, state.y - pose.y) < 2.0


def test_get_commands_without_start_keeps_default_behavior():
    cmds_default = get_commands(OBSTACLES)
    cmds_explicit_default = get_commands(OBSTACLES, start=None)
    assert cmds_default == cmds_explicit_default


def test_get_top_n_routes_uses_provided_start():
    from simulator.planner import MIN_APPROACH_CM
    custom_start = RobotState(x=100.0, y=100.0, theta=0.0)
    obs = [Obstacle(x=150, y=100, face='W')]
    routes = get_top_n_routes(obs, n=1, start=custom_start)
    _, length = routes[0]
    # Nothing blocks this obstacle, so the shortest valid approach (MIN_APPROACH_CM=10)
    # is used: pose.x = 150 - 10 = 140, Manhattan distance from x=100 is 40.
    expected = abs((150 - MIN_APPROACH_CM) - custom_start.x)
    assert abs(length - expected) < 0.01
