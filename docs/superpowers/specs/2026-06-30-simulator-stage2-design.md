# MDP Simulator — Stage 2 Design Spec
**Date:** 2026-06-30
**Scope:** Dubins path geometry — hand-rolled 6-path-type implementation, arc animation, demo wiring into planner
**Out of scope:** Obstacle approach waypoint generation (Stage 3), Hamiltonian ordering (Stage 3), socket integration (Week 5)

---

## 1. Files Changed

| File | Change |
|---|---|
| `Algorithm/simulator/dubins.py` | NEW — pure Dubins geometry, no pygame/config dependency |
| `Algorithm/simulator/types.py` | Add `DubinsPath` dataclass |
| `Algorithm/simulator/robot.py` | Add `arc_step()`, extend `step_command` with 'AL'/'AR' |
| `Algorithm/simulator/planner.py` | Add `dubins_to_commands()`, replace hardcoded body of `get_commands()` |
| `Algorithm/simulator/tests/test_logic.py` | Add arc kinematics tests, Dubins unit tests, briefing worked example test |

`main.py`, `arena.py`, `config.py` — no changes.

---

## 2. New Data Type (`types.py`)

```python
@dataclass
class DubinsPath:
    path_type: str   # 'LSL' | 'LSR' | 'RSL' | 'RSR' | 'RLR' | 'LRL'
    seg1: float      # first segment length in cm
    seg2: float      # second segment length in cm
    seg3: float      # third segment length in cm
    total: float     # seg1 + seg2 + seg3
```

Segment semantics by path type:

| Path type | seg1 | seg2 | seg3 |
|---|---|---|---|
| LSL | Left arc (cm) | Straight (cm) | Left arc (cm) |
| LSR | Left arc (cm) | Straight (cm) | Right arc (cm) |
| RSL | Right arc (cm) | Straight (cm) | Left arc (cm) |
| RSR | Right arc (cm) | Straight (cm) | Right arc (cm) |
| LRL | Left arc (cm) | Right arc (cm) | Left arc (cm) |
| RLR | Right arc (cm) | Left arc (cm) | Right arc (cm) |

---

## 3. `dubins.py` — Pure Geometry Module

No imports from `config`, `arena`, `robot`, or `pygame`. Takes `r` as an explicit parameter so it is fully testable in isolation.

### Public API

```python
def dubins_lsl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None
def dubins_rsr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None
def dubins_lsr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None
def dubins_rsl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None
def dubins_rlr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None
def dubins_lrl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None
def dubins_optimal(q1: RobotState, q2: RobotState, r: float) -> DubinsPath
```

### Algorithm structure (all path functions follow this pattern)

1. **Normalise:** translate/rotate so q1 is at origin facing East; scale positions by `1/r`. Work in the normalised frame throughout.
2. **Compute `(t, p, q)` segment lengths** using the closed-form formula for that path type (see §3.1).
3. **Feasibility check:** return `None` if any intermediate discriminant is negative (path geometrically impossible at this radius).
4. **Denormalise:** multiply `t, p, q` by `r` to recover lengths in cm.
5. **Return** `DubinsPath(path_type, t*r, p*r, q*r, (t+p+q)*r)`.

`dubins_optimal` calls all 6, filters `None`, returns the one with minimum `total`.

### 3.1 Normalised-frame formulas

All angles in radians. `mod2pi(x) = x % (2*pi)`.

Let:
```
dx = (q2.x - q1.x) / r
dy = (q2.y - q1.y) / r
alpha = radians(q1.theta)
beta  = radians(q2.theta)
```

**LSL:**
```
tmp0  = dx + sin(alpha) - sin(beta)
tmp1  = dy - cos(alpha) + cos(beta)
p_sq  = tmp0**2 + tmp1**2
if p_sq < 0: return None
p     = sqrt(p_sq)
theta = atan2(tmp1, tmp0)
t     = mod2pi(theta - alpha)
q     = mod2pi(beta - theta)
```

**RSR:**
```
tmp0  = dx - sin(alpha) + sin(beta)
tmp1  = dy + cos(alpha) - cos(beta)
p_sq  = tmp0**2 + tmp1**2
if p_sq < 0: return None
p     = sqrt(p_sq)
theta = atan2(tmp1, tmp0)
t     = mod2pi(alpha - theta)
q     = mod2pi(theta - beta)
```

**LSR:**
```
tmp0  = dx + sin(alpha) + sin(beta)
tmp1  = dy - cos(alpha) - cos(beta)
p_sq  = tmp0**2 + tmp1**2 - 4
if p_sq < 0: return None
p     = sqrt(p_sq)
theta = atan2(-cos(alpha) - cos(beta), tmp0) - atan2(-2, p)
t     = mod2pi(theta - alpha)
q     = mod2pi(theta - beta)
```

**RSL:**
```
tmp0  = dx - sin(alpha) - sin(beta)
tmp1  = dy + cos(alpha) + cos(beta)
p_sq  = tmp0**2 + tmp1**2 - 4
if p_sq < 0: return None
p     = sqrt(p_sq)
theta = atan2(cos(alpha) + cos(beta), tmp0) - atan2(2, p)
t     = mod2pi(alpha - theta)
q     = mod2pi(beta - theta)
```

**RLR:**
```
tmp0  = (dx - sin(alpha) + sin(beta)) / 6 + cos(alpha) / 3 - cos(beta) / 3
if abs(tmp0) > 1: return None
p     = mod2pi(2*pi - acos(tmp0))
t     = mod2pi(alpha - atan2(cos(alpha) - cos(beta), dx - sin(alpha) + sin(beta)) + p/2)
q     = mod2pi(alpha - beta - t + p)
```

**LRL:**
```
tmp0  = (dx + sin(alpha) - sin(beta)) / 6 - cos(alpha) / 3 + cos(beta) / 3
if abs(tmp0) > 1: return None
p     = mod2pi(2*pi - acos(tmp0))
t     = mod2pi(-alpha + atan2(-cos(alpha) + cos(beta), dx + sin(alpha) - sin(beta)) + p/2)
q     = mod2pi(beta - alpha - t + p)
```

> **Implementer note:** verify these formulas against the worked numeric example in `algorithms_briefing_24SS.pdf` before finalising. Extract the exact (x1, y1, θ1, x2, y2, θ2) and expected path type + segment lengths from the PDF — that is the ground truth for unit tests.

---

## 4. Arc Kinematics (`robot.py`)

### New pure function: `arc_step`

```python
def arc_step(state: RobotState, ds: float, clockwise: bool, r: float) -> RobotState:
    sign = -1 if clockwise else 1
    theta_rad = math.radians(state.theta)
    new_theta_rad = theta_rad + sign * ds / r
    new_x = state.x + sign * r * (math.sin(new_theta_rad) - math.sin(theta_rad))
    new_y = state.y - sign * r * (math.cos(new_theta_rad) - math.cos(theta_rad))
    return RobotState(x=new_x, y=new_y, theta=math.degrees(new_theta_rad) % 360)
```

**Derivation:** the center of curvature is perpendicular to heading at distance `r` (left for CCW, right for CW). After advancing `ds` along the arc, the robot rotates `ds/r` radians around that center. Applying the rotation to the robot-relative vector yields the formula above.

**Verified spot-checks:**
- theta=0°, r=25, ds=π/2·25, left arc → (+25, +25, 90°) ✓
- theta=0°, r=25, ds=π/2·25, right arc → (+25, −25, 270°) ✓

### Extension to `step_command`

Append two branches (after existing FW/BW/TL/TR):

```python
if cmd.kind == 'AL':
    advance = min(STEP_CM_PER_FRAME, remaining)
    return arc_step(state, advance, clockwise=False, r=TURN_RADIUS_CM), remaining - advance
if cmd.kind == 'AR':
    advance = min(STEP_CM_PER_FRAME, remaining)
    return arc_step(state, advance, clockwise=True, r=TURN_RADIUS_CM), remaining - advance
```

`TURN_RADIUS_CM` is already in `config.py`. `main.py` requires no changes.

---

## 5. Planner Stage 2 (`planner.py`)

### New: `dubins_to_commands`

```python
def dubins_to_commands(path: DubinsPath) -> list[Command]:
    kinds = {
        'LSL': ('AL', 'FW', 'AL'), 'LSR': ('AL', 'FW', 'AR'),
        'RSL': ('AR', 'FW', 'AL'), 'RSR': ('AR', 'FW', 'AR'),
        'LRL': ('AL', 'AR', 'AL'), 'RLR': ('AR', 'AL', 'AR'),
    }
    k1, k2, k3 = kinds[path.path_type]
    cmds = []
    for kind, seg in zip((k1, k2, k3), (path.seg1, path.seg2, path.seg3)):
        if seg > 0.01:   # skip effectively-zero segments
            cmds.append(Command(kind, seg))
    return cmds
```

### Updated `get_commands`

```python
def get_commands(obstacles: list[Obstacle]) -> list[Command]:
    # Stage 2: demo Dubins paths through 3 hardcoded waypoints.
    # Stage 3 replaces this with approach-waypoint generation + Hamiltonian ordering.
    waypoints = [
        RobotState(x=100, y=100, theta=0),
        RobotState(x=150, y=50,  theta=180),
        RobotState(x=60,  y=160, theta=90),
    ]
    current = RobotState(x=0, y=0, theta=90)
    cmds: list[Command] = []
    for wp in waypoints:
        path = dubins_optimal(current, wp, TURN_RADIUS_CM)
        cmds += dubins_to_commands(path)
        current = wp
    return cmds
```

---

## 6. Testing

All tests in `Algorithm/simulator/tests/test_logic.py` — pure functions only, no pygame.

### 6.1 Arc kinematics

```python
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
```

### 6.2 Briefing worked example

```python
def test_dubins_briefing_example():
    # Implementer: open algorithms_briefing_24SS.pdf and extract the exact values.
    # Handoff notes: positions (30,10) → (90,70), r=20, total ≈ 84.85 cm.
    # Fill in theta1, theta2, expected path_type, and expected seg1/seg2/seg3 from the PDF.
    path = dubins_optimal(RobotState(30, 10, THETA1), RobotState(90, 70, THETA2), r=20)
    assert path.path_type == EXPECTED_TYPE
    assert abs(path.total - 84.85) < 0.01
```

### 6.3 Endpoint verification

```python
def test_dubins_path_reaches_target():
    q1 = RobotState(0, 0, 90)
    q2 = RobotState(100, 100, 0)
    path = dubins_optimal(q1, q2, r=25)
    # Simulate following the path in STEP_CM_PER_FRAME increments
    state = q1
    for cmd in dubins_to_commands(path):
        remaining = cmd.value
        while remaining > 0.001:
            state, remaining = step_command(state, cmd, remaining)
    assert abs(state.x - q2.x) < 0.5
    assert abs(state.y - q2.y) < 0.5
    assert abs((state.theta - q2.theta + 180) % 360 - 180) < 1.0

def test_dubins_to_commands_lsl():
    path = DubinsPath('LSL', seg1=30, seg2=50, seg3=20, total=100)
    cmds = dubins_to_commands(path)
    assert cmds[0].kind == 'AL'
    assert cmds[1].kind == 'FW'
    assert cmds[2].kind == 'AL'
    assert abs(cmds[0].value - 30) < 0.001

def test_dubins_to_commands_skips_zero_segments():
    path = DubinsPath('LSL', seg1=30, seg2=0, seg3=20, total=50)
    cmds = dubins_to_commands(path)
    assert len(cmds) == 2
    assert all(c.kind == 'AL' for c in cmds)
```

### 6.4 Dubins invariants

```python
def test_dubins_optimal_returns_shortest():
    q1 = RobotState(0, 0, 0)
    q2 = RobotState(50, 50, 90)
    path = dubins_optimal(q1, q2, r=25)
    for fn in [dubins_lsl, dubins_rsr, dubins_lsr, dubins_rsl, dubins_rlr, dubins_lrl]:
        candidate = fn(q1, q2, r=25)
        if candidate is not None:
            assert path.total <= candidate.total + 0.001

def test_dubins_straight_line():
    # Same heading, q2 directly ahead → path should be pure straight (RSR or LSL with zero arcs)
    q1 = RobotState(0, 0, 0)
    q2 = RobotState(100, 0, 0)
    path = dubins_optimal(q1, q2, r=25)
    assert abs(path.total - 100) < 0.1
```

---

## 7. Stage 3 Integration Points

These are the only changes Stage 3 needs to make:

1. **`planner.py` `get_commands()`** — replace the 3 hardcoded waypoints with:
   - `obstacle_approach_pose(obs)` → RobotState (derived from face direction)
   - Brute-force 5! orderings, pick minimum total Dubins length
2. **`dubins.py`** — no changes; `dubins_optimal` is already the correct interface
3. **`robot.py`** — no changes; arc animation already works

---

## 8. Open Items

- Exact angles (θ1, θ2) and expected output for the briefing worked example must be extracted from `algorithms_briefing_24SS.pdf` before writing the test. The handoff confirms positions and total length (84.85 cm) but not headings or path type.
- `TURN_RADIUS_CM = 25.0` in config is still an estimate; does not affect Dubins math correctness (it's a parameter), only the visual demo.
