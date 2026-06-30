# MDP Simulator Stage 3 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire real obstacle approach poses and brute-force Hamiltonian path ordering into `get_commands()` so the simulator visits every obstacle face-to-face in the shortest total Dubins path.

**Architecture:** `planner.py` gains `obstacle_approach_pose` (face direction → RobotState 20cm from face) and `_hamiltonian_optimal_order` (itertools.permutations, pick min-total-length). `get_commands` calls both and chains Dubins paths between each ordered pose. `config.py` gains `APPROACH_CM = 20`.

**Tech Stack:** Python 3.12+, pytest, stdlib only (math, itertools)

## Global Constraints

- `APPROACH_CM = 20` — robot standoff from obstacle face surface, in cm. Must be in `config.py`, never hardcoded.
- `CELL_CM = 10` — already in config; use it in approach pose math, never hardcode 10.
- `TURN_RADIUS_CM = 25.0` — already in config; use for all Dubins calls.
- All new functions go in `Algorithm/simulator/planner.py` or `Algorithm/simulator/config.py`. No new files.
- All tests go in `Algorithm/simulator/tests/test_logic.py`. No new test files.
- No pygame imports anywhere in planner.py or config.py.
- Run tests with: `cd Algorithm && python -m pytest simulator/tests/test_logic.py -v`
- All 43 existing tests must continue to pass after every task.
- Start position: `RobotState(x=0, y=0, theta=90)` — robot starts bottom-left of arena, heading North.

---

### Task 1: `APPROACH_CM` + `obstacle_approach_pose`

**Files:**
- Modify: `Algorithm/simulator/config.py` (add constant)
- Modify: `Algorithm/simulator/planner.py` (add function + import)
- Modify: `Algorithm/simulator/tests/test_logic.py` (add tests + import)

**Interfaces:**
- Consumes: `Obstacle` from `simulator.types`, `CELL_CM` and `APPROACH_CM` from `simulator.config`
- Produces: `obstacle_approach_pose(obs: Obstacle) -> RobotState` — used in Task 3's `get_commands`

- [ ] **Step 1: Write failing tests**

Add to the bottom of `test_logic.py`:

```python
from simulator.planner import obstacle_approach_pose

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
```

- [ ] **Step 2: Run to verify they fail**

```
cd Algorithm && python -m pytest simulator/tests/test_logic.py -k "approach_pose" -v
```

Expected: `ImportError` or `AttributeError` — `obstacle_approach_pose` not yet defined.

- [ ] **Step 3: Add `APPROACH_CM` to config**

In `Algorithm/simulator/config.py`, append:

```python
APPROACH_CM = 20   # robot standoff from obstacle face surface, in cm
```

Full `config.py` after change:

```python
CELL_CM = 10
GRID_SIZE = 20
CELL_PX = 40
ARENA_PX = 800
FPS = 60
TURN_RADIUS_CM = 25.0
ROBOT_W_CM = 20
ROBOT_H_CM = 21
STEP_CM_PER_FRAME = 2.0
DEG_PER_FRAME = 3.0
APPROACH_CM = 20   # robot standoff from obstacle face surface, in cm
```

- [ ] **Step 4: Add `obstacle_approach_pose` to planner.py**

At top of `planner.py`, update the config import to include `APPROACH_CM` and `CELL_CM`:

```python
from simulator.config import APPROACH_CM, CELL_CM, TURN_RADIUS_CM
```

Add this function below the `_SEGMENT_KINDS` dict and before `dubins_to_commands`:

```python
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
```

Note: `cx + d` where `d = CELL_CM/2 + APPROACH_CM = 5 + 20 = 25`. So for `face='E'`, `x = obs.x + 5 + 25 = obs.x + 30` ✓ (= obs.x + CELL_CM + APPROACH_CM).

- [ ] **Step 5: Update the import in `test_logic.py`**

Replace the existing planner import line:

```python
from simulator.planner import OBSTACLES, dubins_to_commands, get_commands
```

with:

```python
from simulator.planner import OBSTACLES, dubins_to_commands, get_commands, obstacle_approach_pose
```

(Remove the separate `from simulator.planner import obstacle_approach_pose` you added in Step 1, and fold it into the consolidated import.)

- [ ] **Step 6: Run all tests**

```
cd Algorithm && python -m pytest simulator/tests/test_logic.py -v
```

Expected: all 43 original tests pass + 5 new approach pose tests pass = **48 tests**.

- [ ] **Step 7: Commit**

```bash
git add Algorithm/simulator/config.py Algorithm/simulator/planner.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat(stage3): add APPROACH_CM config and obstacle_approach_pose"
```

---

### Task 2: `_hamiltonian_optimal_order`

**Files:**
- Modify: `Algorithm/simulator/planner.py` (add function + itertools import)
- Modify: `Algorithm/simulator/tests/test_logic.py` (add tests + import)

**Interfaces:**
- Consumes: `dubins_optimal` from `simulator.dubins`, `TURN_RADIUS_CM` from config, `RobotState` from types
- Produces: `_hamiltonian_optimal_order(start: RobotState, poses: list[RobotState], r: float) -> list[RobotState]` — used in Task 3's `get_commands`

Note: `_hamiltonian_optimal_order` has a leading underscore — it is module-private. Tests import it directly for unit testing. This is fine.

- [ ] **Step 1: Write failing tests**

Add to the import at the top of `test_logic.py`:

```python
from simulator.planner import OBSTACLES, dubins_to_commands, get_commands, obstacle_approach_pose, _hamiltonian_optimal_order
```

Add these tests at the bottom of `test_logic.py`:

```python
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
```

- [ ] **Step 2: Run to verify they fail**

```
cd Algorithm && python -m pytest simulator/tests/test_logic.py -k "hamiltonian" -v
```

Expected: `ImportError` or `AttributeError`.

- [ ] **Step 3: Add `itertools` import and `_hamiltonian_optimal_order` to planner.py**

At top of `planner.py`, add `import itertools` before the `from simulator...` imports:

```python
import itertools

from simulator.config import APPROACH_CM, CELL_CM, TURN_RADIUS_CM
from simulator.dubins import dubins_optimal
from simulator.types import Command, DubinsPath, Obstacle, RobotState
```

Add these two functions below `obstacle_approach_pose` and before `dubins_to_commands`:

```python
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
```

- [ ] **Step 4: Run all tests**

```
cd Algorithm && python -m pytest simulator/tests/test_logic.py -v
```

Expected: all 48 previous tests pass + 4 new Hamiltonian tests = **52 tests**.

- [ ] **Step 5: Commit**

```bash
git add Algorithm/simulator/planner.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat(stage3): add _hamiltonian_optimal_order with brute-force 5! search"
```

---

### Task 3: Wire `get_commands` + cleanup

**Files:**
- Modify: `Algorithm/simulator/planner.py` (rewrite `get_commands`, cleanup dead guards, raise on unknown command)
- Modify: `Algorithm/simulator/robot.py` (raise on unknown `cmd.kind`)
- Modify: `Algorithm/simulator/dubins.py` (remove dead `if p_sq < 0` guards in `dubins_lsl` and `dubins_rsr`)
- Modify: `Algorithm/simulator/tests/test_logic.py` (integration tests, update import)

**Interfaces:**
- Consumes: `obstacle_approach_pose`, `_hamiltonian_optimal_order`, `dubins_to_commands`, `dubins_optimal`, `TURN_RADIUS_CM`
- Produces: `get_commands(obstacles: list[Obstacle]) -> list[Command]` — final wired version

- [ ] **Step 1: Write failing integration tests**

Add to `test_logic.py` at the bottom:

```python
# ── Task 3 (Stage 3): get_commands wired ────────────────────────────────────

def test_get_commands_arc_commands_present():
    cmds = get_commands(OBSTACLES)
    assert any(c.kind in ('AL', 'AR') for c in cmds)

def test_get_commands_no_unknown_kinds():
    cmds = get_commands(OBSTACLES)
    valid = {'FW', 'BW', 'AL', 'AR'}
    assert all(c.kind in valid for c in cmds)

def test_get_commands_reaches_all_approach_poses():
    """Simulate full command sequence; verify robot visits each approach pose (within 2cm)."""
    import math
    start = RobotState(x=0, y=0, theta=90)
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
```

- [ ] **Step 2: Run to verify tests currently fail or pass incorrectly**

```
cd Algorithm && python -m pytest simulator/tests/test_logic.py -k "get_commands" -v
```

The existing `test_get_commands_*` tests may still pass (they tested the demo waypoints). The new ones will fail because `get_commands` still uses hardcoded waypoints.

- [ ] **Step 3: Rewrite `get_commands` in planner.py**

Replace the current `get_commands` function body:

```python
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
```

- [ ] **Step 4: Remove dead `p_sq < 0` guards in dubins.py**

In `Algorithm/simulator/dubins.py`, find `dubins_lsl` and `dubins_rsr`. Both have:

```python
if p_sq < 0:
    return None
```

`p_sq = tmp0**2 + tmp1**2` is a sum of squares — always ≥ 0. Remove these guards from both functions. (For `dubins_lsr` and `dubins_rsl`, `p_sq = ... - 4` CAN be negative, so leave those guards in place.)

After removing, `dubins_lsl` and `dubins_rsr` should call `sqrt(p_sq)` directly without the guard.

- [ ] **Step 5: Add `raise` for unknown command kinds in robot.py**

In `Algorithm/simulator/robot.py`, replace the final line of `step_command`:

```python
    return state, 0.0
```

with:

```python
    raise ValueError(f"Unknown command kind: {cmd.kind!r}")
```

- [ ] **Step 6: Run all tests**

```
cd Algorithm && python -m pytest simulator/tests/test_logic.py -v
```

Expected: all 52 previous tests pass + 3 new integration tests = **55 tests** (minimum).

If `test_get_commands_reaches_all_approach_poses` is slow (it simulates many steps), it will still pass — just takes a few seconds.

- [ ] **Step 7: Verify simulator still runs visually**

```
cd Algorithm && python main.py
```

Watch the robot trace Dubins paths through all 5 obstacle approach poses. The robot should stop face-to-face in front of each obstacle's marked face (orange tick). If the simulator crashes, check the traceback — most likely `get_commands` returning empty list or approach pose out of bounds.

- [ ] **Step 8: Commit**

```bash
git add Algorithm/simulator/planner.py Algorithm/simulator/robot.py Algorithm/simulator/dubins.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat(stage3): wire get_commands with approach poses and Hamiltonian ordering; cleanup"
```
