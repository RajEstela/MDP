# MDP Simulator — Stage 3 Design Spec
**Date:** 2026-06-30
**Scope:** Obstacle approach pose generation + Hamiltonian path ordering, wired into `get_commands()`
**Prerequisite:** Stage 2 complete (commit 4393b0b, 43 tests passing)

---

## 1. Goal

Replace the hardcoded demo waypoints in `get_commands()` with real obstacle approach poses and a brute-force Hamiltonian path ordering so the robot visits every obstacle face-to-face in the shortest total path.

---

## 2. Files Changed

| File | Change |
|---|---|
| `Algorithm/simulator/config.py` | Add `APPROACH_CM` constant |
| `Algorithm/simulator/planner.py` | Add `obstacle_approach_pose`, `_hamiltonian_optimal_order`; rewrite `get_commands`; remove `_total_dubins_length` helper |
| `Algorithm/simulator/tests/test_logic.py` | Approach pose tests, Hamiltonian tests, updated integration tests |

`dubins.py`, `robot.py`, `arena.py`, `main.py`, `types.py` — **no changes.**

---

## 3. Coordinate System

`Obstacle.x, Obstacle.y` = **bottom-left corner** of the 10×10cm cell (confirmed by `arena.py` line 36).

- Cell center: `(obs.x + CELL_CM/2, obs.y + CELL_CM/2)`
- North face surface: at `y = obs.y + CELL_CM`
- South face surface: at `y = obs.y`
- East face surface: at `x = obs.x + CELL_CM`
- West face surface: at `x = obs.x`

---

## 4. Approach Poses

### 4.1 `APPROACH_CM` constant

Add to `config.py`:
```python
APPROACH_CM = 20   # robot standoff from obstacle face surface, in cm
```

### 4.2 `obstacle_approach_pose`

Places the robot reference point `APPROACH_CM` in front of the obstacle face, heading directly toward it:

| face | Robot position | Robot theta |
|---|---|---|
| `N` | `(obs.x + CELL_CM/2, obs.y + CELL_CM + APPROACH_CM)` | 270° (facing South) |
| `S` | `(obs.x + CELL_CM/2, obs.y - APPROACH_CM)` | 90° (facing North) |
| `E` | `(obs.x + CELL_CM + APPROACH_CM, obs.y + CELL_CM/2)` | 180° (facing West) |
| `W` | `(obs.x - APPROACH_CM, obs.y + CELL_CM/2)` | 0° (facing East) |

With `CELL_CM=10`, `APPROACH_CM=20`:
- `Obstacle(50, 50, 'N')` → `RobotState(55, 80, 270)`
- `Obstacle(50, 50, 'S')` → `RobotState(55, 30, 90)`
- `Obstacle(50, 50, 'E')` → `RobotState(80, 55, 180)`
- `Obstacle(50, 50, 'W')` → `RobotState(30, 55, 0)`

The robot heading is opposite to the face direction (robot faces INTO the face):
- N face → theta=270° (South) ✓
- S face → theta=90° (North) ✓
- E face → theta=180° (West) ✓
- W face → theta=0° (East) ✓

**Signature:**
```python
def obstacle_approach_pose(obs: Obstacle) -> RobotState
```

---

## 5. Hamiltonian Ordering

Brute-force all `n!` orderings of the approach poses and pick the one with minimum total Dubins path length starting from the robot's initial position.

### 5.1 Helper

```python
def _total_dubins_length(start: RobotState, poses: list[RobotState], r: float) -> float
```

Computes the sum of `dubins_optimal(q_i, q_{i+1}, r).total` for the chain `start → poses[0] → poses[1] → … → poses[-1]`.

### 5.2 Main function

```python
def _hamiltonian_optimal_order(start: RobotState, poses: list[RobotState], r: float) -> list[RobotState]
```

- Uses `itertools.permutations(poses)`
- 5 obstacles → 5! = 120 iterations; 10 obstacles → 10! = 3.6M (fine for 5, not for 10 — but the competition has exactly 5)
- Returns the permutation list with minimum total length

### 5.3 Tie-breaking

Ties broken by whichever permutation `itertools.permutations` yields first — deterministic, no special handling needed.

---

## 6. Updated `get_commands`

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

---

## 7. Cleanup Items (from Stage 2 final review minors)

Include in Stage 3 Task 3:
1. Remove dead `if p_sq < 0: return None` guards in `dubins_lsl` and `dubins_rsr` — `p_sq` is a sum of squares, always ≥ 0
2. Add `raise ValueError(f"Unknown command kind: {cmd.kind!r}")` at end of `step_command` instead of silently returning `(state, 0.0)`

---

## 8. Tests

### 8.1 Approach pose (4 directional + 1 heading invariant)

```python
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
```

### 8.2 Hamiltonian ordering

```python
def test_hamiltonian_visits_all_poses():
    start = RobotState(0, 0, 90)
    poses = [RobotState(50, 0, 0), RobotState(100, 0, 0), RobotState(150, 0, 0)]
    result = _hamiltonian_optimal_order(start, poses, r=25)
    assert len(result) == 3
    assert set((p.x, p.y) for p in result) == set((p.x, p.y) for p in poses)

def test_hamiltonian_single_pose():
    start = RobotState(0, 0, 90)
    poses = [RobotState(50, 50, 0)]
    result = _hamiltonian_optimal_order(start, poses, r=25)
    assert len(result) == 1
    assert result[0].x == 50

def test_hamiltonian_selects_shorter_order():
    # Pose A at (100, 0), Pose B at (10, 0). Start at (0, 0, 0).
    # A→B travels 100 then back 90; B→A travels 10 then 90 more. B→A should be shorter.
    start = RobotState(0, 0, 0)
    a = RobotState(100, 0, 0)
    b = RobotState(10, 0, 0)
    result = _hamiltonian_optimal_order(start, [a, b], r=25)
    # First visit should be the nearer one
    assert result[0].x == b.x
```

### 8.3 Integration

```python
def test_get_commands_uses_all_obstacles():
    # get_commands should produce a non-empty arc-containing sequence
    cmds = get_commands(OBSTACLES)
    assert len(cmds) > 0
    assert any(c.kind in ('AL', 'AR') for c in cmds)

def test_get_commands_reaches_all_approach_poses():
    # Simulate full command sequence; verify robot visits each approach pose within 5cm
    start = RobotState(x=0, y=0, theta=90)
    poses = {obstacle_approach_pose(obs) for obs in OBSTACLES}  # use set for O(1) lookup
    # ... simulate and check
```

> Note: the endpoint simulation for `get_commands` is expensive (5 paths × many steps). Include it but use a generous tolerance (2cm).

---

## 9. Stage 3 Integration Points

After Stage 3, `get_commands` is fully wired. The only remaining work before socket integration:
- Socket module (Week 5 — out of scope for Stage 3)
