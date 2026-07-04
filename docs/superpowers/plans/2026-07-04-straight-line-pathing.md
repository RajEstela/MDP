# Stage 4: Straight-Line Pathing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Dubins arc-based path planning with straight-line movement plus in-place rotation (RL/RR) to match the physical car's command set.

**Architecture:** Every leg q1→q2 becomes: rotate in-place (RL/RR) → drive straight (FW) → rotate in-place (RL/RR). `_plan_leg` is rewritten using a new `_direct_leg` helper and an 8-point bypass-waypoint fallback for obstacle avoidance. Hamiltonian ordering switches from Dubins path length to Euclidean straight-line distance.

**Tech Stack:** Python 3.12, pytest, `Algorithm/simulator/` package

## Global Constraints

- Valid command kinds after this change: `FW`, `BW`, `RL`, `RR`, `WAIT` — no `AL`, `AR`, `TL`, `TR`
- `_angle_diff` returns positive = left (RL), negative = right (RR), range (-180, 180]
- Bypass waypoints offset by `_ROBOT_CLEARANCE = 20.0` cm from obstacle cell edges
- `dubins.py` is NOT modified — kept for reference, just not imported by planner
- Run tests with: `cd Algorithm && python -m pytest simulator/tests/ -v`

---

### Task 1: Update `robot.py` and fix affected tests

**Files:**
- Modify: `Algorithm/simulator/robot.py`
- Modify: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Produces: `step_command` handles `RL` and `RR`; raises `ValueError` for `AL`, `AR`, `TL`, `TR`
- Produces: `arc_step` stays in `robot.py` (needed for Dubins reference tests below)

- [ ] **Step 1: Update `step_command` and import in `robot.py`**

Replace the full `step_command` function body:

```python
def step_command(
    state: RobotState, cmd: Command, remaining: float
) -> tuple[RobotState, float]:
    if cmd.kind == 'FW':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return move_forward(state, advance), remaining - advance
    if cmd.kind == 'BW':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return move_forward(state, -advance), remaining - advance
    if cmd.kind == 'RL':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=False), remaining - advance
    if cmd.kind == 'RR':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=True), remaining - advance
    if cmd.kind == 'WAIT':
        return state, remaining - 1.0
    raise ValueError(f"Unknown command kind: {cmd.kind!r}")
```

Update the import on line 5 — remove `ROBOT_H_CM` and `TURN_RADIUS_CM` (both unused after this change):

```python
from simulator.config import CELL_CM, CELL_PX, DEG_PER_FRAME, ROBOT_W_CM, STEP_CM_PER_FRAME
```

- [ ] **Step 2: Update step_command tests in `test_logic.py`**

Replace the `TL`/`TR` tests (lines 84–96) with `RL`/`RR`:

```python
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
```

Delete the `AL`/`AR` step_command tests entirely (lines 176–189):
```python
# DELETE both of these:
# def test_step_command_al_arcs_left
# def test_step_command_ar_arcs_right
```

- [ ] **Step 3: Add `_sim_dubins_path` helper and update Dubins endpoint tests**

The five Dubins endpoint tests currently use `step_command` to drive AL/AR commands — that will now raise `ValueError`. Add this helper near the top of the test file (after imports) and update all five tests to use it:

```python
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
```

Update `test_dubins_path_reaches_target` (line 297) to use the helper:

```python
def test_dubins_path_reaches_target():
    q1 = RobotState(0, 0, 90)
    q2 = RobotState(100, 100, 0)
    path = dubins_optimal(q1, q2, r=25)
    state = _sim_dubins_path(q1, path)
    assert abs(state.x - q2.x) < 0.5
    assert abs(state.y - q2.y) < 0.5
    assert abs((state.theta - q2.theta + 180) % 360 - 180) < 1.0
```

Update `test_dubins_rlr_endpoint` (line 314):

```python
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
```

Update `test_dubins_lrl_endpoint` (line 332):

```python
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
```

Update `test_dubins_lsr_endpoint` (line 460):

```python
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
```

Update `test_dubins_rsl_endpoint` (line 478):

```python
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
```

- [ ] **Step 4: Run tests — skip integration tests that still use old AL/AR planner output**

```
cd Algorithm && python -m pytest simulator/tests/ -v -k "not get_commands"
```

Expected: all non-get_commands tests pass. (Integration tests are temporarily excluded — the planner still emits AL/AR which step_command no longer accepts; those are fixed in Task 3.)

- [ ] **Step 5: Commit**

```bash
git add Algorithm/simulator/robot.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat: rename TL/TR → RL/RR, remove AL/AR from step_command; update tests"
```

---

### Task 2: Add `_angle_diff` helper to `planner.py`

**Files:**
- Modify: `Algorithm/simulator/planner.py`
- Modify: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Produces: `_angle_diff(from_deg: float, to_deg: float) -> float`
  - Returns signed shortest angular difference
  - Positive → rotate left (RL), negative → rotate right (RR)
  - Result is in the range (-180, 180]

- [ ] **Step 1: Write failing tests**

Add to `Algorithm/simulator/tests/test_logic.py` (after existing imports):

```python
from simulator.planner import _angle_diff
```

Add test functions:

```python
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
```

- [ ] **Step 2: Run tests to confirm they fail**

```
cd Algorithm && python -m pytest simulator/tests/test_logic.py -k "angle_diff" -v
```

Expected: `ImportError` — `_angle_diff` not yet defined.

- [ ] **Step 3: Add `_angle_diff` to `planner.py`**

Add after the module-level constants (after `_BACKUP_DISTANCES`), before `obstacle_approach_pose`:

```python
def _angle_diff(from_deg: float, to_deg: float) -> float:
    """Signed shortest angular difference. Positive = left (RL), negative = right (RR)."""
    return (to_deg - from_deg + 180) % 360 - 180
```

- [ ] **Step 4: Run tests to confirm they pass**

```
cd Algorithm && python -m pytest simulator/tests/test_logic.py -k "angle_diff" -v
```

Expected: 6 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add Algorithm/simulator/planner.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat: add _angle_diff helper to planner"
```

---

### Task 3: Rewrite `planner.py` — straight-line pathing + update all remaining tests

**Files:**
- Modify: `Algorithm/simulator/planner.py`
- Modify: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Consumes: `_angle_diff(from_deg, to_deg) -> float` (Task 2)
- Produces:
  - `_direct_leg(q1: RobotState, q2: RobotState) -> tuple[list[Command], float]`
  - `_bypass_waypoints(obs: Obstacle) -> list[tuple[float, float]]`
  - `_plan_leg(q1: RobotState, q2: RobotState, obstacles: list[Obstacle] | None = None) -> tuple[list[Command], float]`
  - `_hamiltonian_optimal_order(start: RobotState, poses: list[RobotState]) -> list[RobotState]`
  - `get_commands(obstacles: list[Obstacle]) -> list[Command]`

- [ ] **Step 1: Write failing integration tests**

Add to `Algorithm/simulator/tests/test_logic.py`:

```python
from simulator.planner import _plan_leg, _path_in_bounds

# ── Stage 4: straight-line pathing ─────────────────────────────────────────

def test_get_commands_uses_rl_rr_not_arcs():
    cmds = get_commands(OBSTACLES)
    assert not any(c.kind in ('AL', 'AR') for c in cmds)
    assert any(c.kind in ('RL', 'RR') for c in cmds)

def test_plan_leg_direct_no_arcs():
    q1 = RobotState(x=20, y=35, theta=90)
    q2 = RobotState(x=55, y=80, theta=270)
    cmds, dist = _plan_leg(q1, q2, obstacles=[])
    assert all(c.kind in ('FW', 'BW', 'RL', 'RR', 'WAIT') for c in cmds)
    assert any(c.kind == 'FW' for c in cmds)
    assert dist > 0

def test_path_in_bounds_handles_rl_rr():
    state = RobotState(x=100, y=100, theta=0)
    cmds = [
        Command(kind='RL', value=90),
        Command(kind='FW', value=20),
        Command(kind='RR', value=45),
    ]
    assert _path_in_bounds(state, cmds) is True
```

- [ ] **Step 2: Run tests to confirm they fail**

```
cd Algorithm && python -m pytest simulator/tests/test_logic.py -k "rl_rr or no_arcs or handles_rl" -v
```

Expected: FAIL — functions not yet updated.

- [ ] **Step 3: Update `_path_in_bounds` in `planner.py`**

Replace the entire `_path_in_bounds` function. The new version removes `r` from the signature, removes the `AL`/`AR` branch, and handles `RL`/`RR` atomically (no position change):

```python
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
```

- [ ] **Step 4: Add `_direct_leg` and `_bypass_waypoints` to `planner.py`**

Add both functions after `_angle_diff`, before the existing `_plan_leg`:

```python
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
```

- [ ] **Step 5: Replace `_plan_leg` in `planner.py`**

Replace the entire existing `_plan_leg` function:

```python
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
```

- [ ] **Step 6: Replace Hamiltonian helpers in `planner.py`**

Delete `_total_dubins_length`, `_dubins_bounded`, `_all_dubins`, `dubins_to_commands`, and `_SEGMENT_KINDS`.

Replace `_hamiltonian_optimal_order` with this new version (removes `r` and `obstacles` params):

```python
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
```

Also delete `_BACKUP_DISTANCES` module-level constant.

- [ ] **Step 7: Update `get_commands` and `get_top_n_routes` in `planner.py`**

Replace `get_commands`:

```python
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
```

Replace `get_top_n_routes`:

```python
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
```

- [ ] **Step 8: Remove dead imports from `planner.py`**

Delete the entire Dubins import line:
```python
# DELETE this line:
from simulator.dubins import dubins_lrl, dubins_lsl, dubins_lsr, dubins_rlr, dubins_rsl, dubins_rsr
```

Update the config import — remove `TURN_RADIUS_CM` and `ROBOT_W_CM`:
```python
from simulator.config import APPROACH_CM, ARENA_CM, CELL_CM, FPS, GRID_SIZE, START_THETA, START_X_CM, START_Y_CM
```

- [ ] **Step 9: Update `test_logic.py` — imports and removed tests**

By this point there are three separate planner import lines (original line 6, plus the two additions from Task 2 and Task 3 Step 1). Consolidate them all into one line — remove `dubins_to_commands`, keep everything else:

```python
from simulator.planner import OBSTACLES, get_commands, obstacle_approach_pose, _hamiltonian_optimal_order, _angle_diff, _plan_leg, _path_in_bounds
```

Delete the four `dubins_to_commands` tests (they test a function that no longer exists in planner):
```python
# DELETE all four of these:
# def test_dubins_to_commands_lsl
# def test_dubins_to_commands_rsr
# def test_dubins_to_commands_lrl
# def test_dubins_to_commands_skips_zero_segments
```

- [ ] **Step 10: Update `test_logic.py` — fix valid-kinds and arc-presence tests**

Update `test_get_commands_all_valid_kinds` (line 132):
```python
def test_get_commands_all_valid_kinds():
    cmds = get_commands(OBSTACLES)
    valid = {'FW', 'BW', 'RL', 'RR', 'WAIT'}
    assert all(c.kind in valid for c in cmds)
```

Update `test_get_commands_produces_arc_commands` (line 285):
```python
def test_get_commands_produces_arc_commands():
    cmds = get_commands(OBSTACLES)
    kinds = {c.kind for c in cmds}
    assert kinds <= {'FW', 'BW', 'RL', 'RR', 'WAIT'}
    assert 'RL' in kinds or 'RR' in kinds
```

Update `test_get_commands_arc_commands_present` (line 433):
```python
def test_get_commands_arc_commands_present():
    cmds = get_commands(OBSTACLES)
    assert any(c.kind in ('RL', 'RR') for c in cmds)
```

Update `test_get_commands_no_unknown_kinds` (line 437):
```python
def test_get_commands_no_unknown_kinds():
    cmds = get_commands(OBSTACLES)
    valid = {'FW', 'BW', 'RL', 'RR', 'WAIT'}
    assert all(c.kind in valid for c in cmds)
```

- [ ] **Step 11: Update `test_logic.py` — fix Hamiltonian test signatures**

Remove the `r=25` argument from all four Hamiltonian test calls:

```python
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
```

- [ ] **Step 12: Update `test_get_commands_reaches_final_approach_pose` to use step_command**

The planner now emits RL/RR which step_command handles. Update the simulation back to step_command:

```python
def test_get_commands_reaches_final_approach_pose():
    """Simulate full command sequence; verify robot ends near the last approach pose (within 2cm)."""
    start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    cmds = get_commands(OBSTACLES)
    state = start
    for cmd in cmds:
        remaining = cmd.value
        while remaining > 0.001:
            state, remaining = step_command(state, cmd, remaining)
    poses = [obstacle_approach_pose(obs) for obs in OBSTACLES]
    closest = min(poses, key=lambda p: math.hypot(state.x - p.x, state.y - p.y))
    assert math.hypot(state.x - closest.x, state.y - closest.y) < 2.0
```

- [ ] **Step 13: Run the full test suite**

```
cd Algorithm && python -m pytest simulator/tests/ -v
```

Expected: all tests pass. Key assertions:
- `test_get_commands_uses_rl_rr_not_arcs` → PASS
- `test_plan_leg_direct_no_arcs` → PASS
- `test_path_in_bounds_handles_rl_rr` → PASS
- `test_get_commands_all_valid_kinds` → PASS
- All angle_diff tests → PASS
- All approach pose and Hamiltonian tests → PASS
- All Dubins math tests (dubins.py unchanged) → PASS

- [ ] **Step 14: Commit**

```bash
git add Algorithm/simulator/planner.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat(stage4): straight-line pathing — RL/RR replaces Dubins arcs in planner"
```
