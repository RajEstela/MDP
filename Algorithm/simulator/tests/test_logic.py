import math

from simulator.arena import cm_to_px
from simulator.planner import OBSTACLES, get_commands
from simulator.robot import arc_step, move_forward, rotate, step_command
from simulator.types import Command, DubinsPath, Obstacle, RobotState


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


def test_step_command_tr_reduces_theta():
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    cmd = Command(kind='TR', value=90.0)
    new_state, remaining = step_command(state, cmd, 90.0)
    assert remaining < 90.0
    assert new_state.theta < 90.0


def test_step_command_tl_increases_theta():
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    cmd = Command(kind='TL', value=90.0)
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
    valid = {'FW', 'BW', 'TL', 'TR'}
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


def test_step_command_al_arcs_left():
    state = RobotState(0, 0, 0)
    cmd = Command(kind='AL', value=math.pi / 2 * 25)
    new_state, remaining = step_command(state, cmd, math.pi / 2 * 25)
    assert remaining < math.pi / 2 * 25
    assert new_state.y > 0


def test_step_command_ar_arcs_right():
    state = RobotState(0, 0, 0)
    cmd = Command(kind='AR', value=math.pi / 2 * 25)
    new_state, remaining = step_command(state, cmd, math.pi / 2 * 25)
    assert remaining < math.pi / 2 * 25
    assert new_state.y < 0
