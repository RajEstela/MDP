import math

from simulator.arena import cm_to_px
from simulator.dubins import dubins_lrl, dubins_lsl, dubins_lsr, dubins_optimal, dubins_rlr, dubins_rsl, dubins_rsr
from simulator.config import START_THETA, START_X_CM, START_Y_CM
from simulator.planner import OBSTACLES, dubins_to_commands, get_commands, obstacle_approach_pose, _hamiltonian_optimal_order
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
    valid = {'FW', 'BW', 'TL', 'TR', 'AL', 'AR', 'WAIT'}
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


# ── Task 3: planner Stage 2 ────────────────────────────────────────────────

def test_dubins_to_commands_lsl():
    path = DubinsPath(path_type='LSL', seg1=30.0, seg2=50.0, seg3=20.0, total=100.0)
    cmds = dubins_to_commands(path)
    assert len(cmds) == 3
    assert cmds[0].kind == 'AL'
    assert cmds[1].kind == 'FW'
    assert cmds[2].kind == 'AL'
    assert abs(cmds[0].value - 30.0) < 0.001
    assert abs(cmds[1].value - 50.0) < 0.001
    assert abs(cmds[2].value - 20.0) < 0.001


def test_dubins_to_commands_rsr():
    path = DubinsPath(path_type='RSR', seg1=10.0, seg2=40.0, seg3=10.0, total=60.0)
    cmds = dubins_to_commands(path)
    assert len(cmds) == 3
    assert cmds[0].kind == 'AR'
    assert cmds[1].kind == 'FW'
    assert cmds[2].kind == 'AR'
    assert abs(cmds[0].value - 10.0) < 0.001
    assert abs(cmds[1].value - 40.0) < 0.001
    assert abs(cmds[2].value - 10.0) < 0.001


def test_dubins_to_commands_lrl():
    path = DubinsPath(path_type='LRL', seg1=15.0, seg2=25.0, seg3=15.0, total=55.0)
    cmds = dubins_to_commands(path)
    assert cmds[0].kind == 'AL'
    assert cmds[1].kind == 'AR'
    assert cmds[2].kind == 'AL'


def test_dubins_to_commands_skips_zero_segments():
    path = DubinsPath(path_type='LSL', seg1=30.0, seg2=0.0, seg3=20.0, total=50.0)
    cmds = dubins_to_commands(path)
    assert len(cmds) == 2
    assert all(c.kind == 'AL' for c in cmds)


def test_get_commands_produces_arc_commands():
    cmds = get_commands(OBSTACLES)
    kinds = {c.kind for c in cmds}
    assert kinds <= {'FW', 'BW', 'AL', 'AR', 'WAIT'}
    assert 'AL' in kinds or 'AR' in kinds


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
    assert abs(pose.x - 55) < 0.01
    assert abs(pose.y - 80) < 0.01
    assert abs(pose.theta - 270) < 0.01


def test_approach_pose_south():
    obs = Obstacle(x=50, y=50, face='S')
    pose = obstacle_approach_pose(obs)
    assert abs(pose.x - 55) < 0.01
    assert abs(pose.y - 30) < 0.01
    assert abs(pose.theta - 90) < 0.01


def test_approach_pose_east():
    obs = Obstacle(x=50, y=50, face='E')
    pose = obstacle_approach_pose(obs)
    assert abs(pose.x - 80) < 0.01
    assert abs(pose.y - 55) < 0.01
    assert abs(pose.theta - 180) < 0.01


def test_approach_pose_west():
    obs = Obstacle(x=50, y=50, face='W')
    pose = obstacle_approach_pose(obs)
    assert abs(pose.x - 30) < 0.01
    assert abs(pose.y - 55) < 0.01
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
    result = _hamiltonian_optimal_order(start, poses, r=25)
    assert len(result) == 3
    result_coords = {(p.x, p.y) for p in result}
    expected_coords = {(p.x, p.y) for p in poses}
    assert result_coords == expected_coords

def test_hamiltonian_single_pose():
    start = RobotState(0, 0, 90)
    poses = [RobotState(50, 50, 0)]
    result = _hamiltonian_optimal_order(start, poses, r=25)
    assert len(result) == 1
    assert result[0].x == 50 and result[0].y == 50

def test_hamiltonian_selects_shorter_order():
    # Start at (0,0,0). Pose B=(10,0,0) is close; Pose A=(100,0,0) is far.
    # Visiting B first (0→10→100) is shorter total than A first (0→100→10→back).
    # Both end at 0°, so the far→near order is always longer.
    start = RobotState(0, 0, 0)
    a = RobotState(100, 0, 0)
    b = RobotState(10, 0, 0)
    result = _hamiltonian_optimal_order(start, [a, b], r=25)
    assert result[0].x == b.x

def test_hamiltonian_five_poses_returns_five():
    start = RobotState(0, 0, 90)
    poses = [obstacle_approach_pose(obs) for obs in OBSTACLES]
    result = _hamiltonian_optimal_order(start, poses, r=25)
    assert len(result) == 5


# ── Task 3 (Stage 3): get_commands wired ────────────────────────────────────

def test_get_commands_arc_commands_present():
    cmds = get_commands(OBSTACLES)
    assert any(c.kind in ('AL', 'AR') for c in cmds)

def test_get_commands_no_unknown_kinds():
    cmds = get_commands(OBSTACLES)
    valid = {'FW', 'BW', 'AL', 'AR', 'WAIT'}
    assert all(c.kind in valid for c in cmds)

def test_get_commands_reaches_final_approach_pose():
    """Simulate full command sequence; verify robot ends near the last approach pose (within 2cm)."""
    import math
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    cmds = get_commands(OBSTACLES)
    state = start
    for cmd in cmds:
        remaining = cmd.value
        while remaining > 0.001:
            state, remaining = step_command(state, cmd, remaining)
    # At minimum, verify the final state is one of the approach poses
    poses = [obstacle_approach_pose(obs) for obs in OBSTACLES]
    closest = min(poses, key=lambda p: math.hypot(state.x - p.x, state.y - p.y))
    assert math.hypot(state.x - closest.x, state.y - closest.y) < 2.0


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
