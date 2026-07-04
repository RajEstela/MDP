# MDP Simulator — Stage 4 Design Spec: Straight-Line Pathing
**Date:** 2026-07-04
**Scope:** Replace Dubins arc-based path planning with straight-line + in-place rotation to match the physical car's actual command set.
**Prerequisite:** Stage 3 complete (commit fe90527, all tests passing)

---

## 1. Background & Motivation

The physical car accepts these commands:
- `FWxxx` — move forward xxx cm
- `BWxxx` — move backward xxx cm
- `RLxxx` — rotate left xxx degrees (firmware handles K-turn internally)
- `RRxxx` — rotate right xxx degrees (firmware handles K-turn internally)
- `STOP`  — stop immediately

The current simulator and planner use **Dubins paths** — smooth arcs (`AL`/`AR`) + straight lines — which exist to solve the minimum-turn-radius constraint of a car that cannot rotate in place. The physical car's `RL`/`RR` commands provide an in-place heading change (implemented internally as a K-turn: forward arc → backward arc → repeat). This eliminates the minimum-turn-radius constraint from the planner's perspective.

**Result**: Dubins paths are no longer the right model. The optimal path between any two oriented waypoints is now: rotate in-place → drive straight → rotate in-place.

---

## 2. Motion Model

For each leg `q1 → q2`:

1. **RL/RR** — rotate in-place to face `q2` (shortest angular path)
2. **FW** — drive straight to `q2` (Euclidean distance)
3. **RL/RR** — rotate in-place from travel heading to `q2.theta`

Steps 1 and 3 are omitted if the angle delta is negligible (< 0.01°).

The signed shortest angular difference helper:
```
_angle_diff(from_deg, to_deg) → value in (-180, 180]
  positive → rotate left (RL)
  negative → rotate right (RR)
```

Example: robot at `(20, 35, 90°)` travelling to `(55, 80, 270°)`:
- Travel heading = atan2(45, 35) ≈ 52°. Diff from 90° = -38° → `RR038`
- Distance ≈ 57 cm → `FW057`
- Diff from 52° to 270° = +142° → `RL142`

---

## 3. Command Set Changes

| Old kind | New kind | Notes |
|---|---|---|
| `AL` (arc left, cm) | — | Removed — arcs no longer generated |
| `AR` (arc right, cm) | — | Removed — arcs no longer generated |
| `TL` (turn left, deg) | `RL` (rotate left, deg) | Renamed to match physical protocol |
| `TR` (turn right, deg) | `RR` (rotate right, deg) | Renamed to match physical protocol |
| `FW` | `FW` | Unchanged |
| `BW` | `BW` | Unchanged |
| `WAIT` | `WAIT` | Unchanged (simulator only) |

`Command` dataclass is unchanged — `kind` is still a string, `value` is still a float.

`dubins.py` is kept for reference but is no longer imported by the planner.

---

## 4. Obstacle Avoidance

### Pass 1 — Direct path
Build the `RL/RR → FW → RL/RR` command sequence and run through `_path_in_bounds`. If clear, use it.

### Pass 2 — Bypass waypoints
If the direct `FW` segment hits an obstacle, generate **8 bypass candidate points** per blocking obstacle:
- 4 corners: NE, NW, SE, SW — each offset by `_ROBOT_CLEARANCE` (20 cm) from the obstacle cell edge
- 4 side midpoints: N, S, E, W — each offset by `_ROBOT_CLEARANCE` from the obstacle cell edge

Exact coordinates for obstacle at `(obs.x, obs.y)` with `c = _ROBOT_CLEARANCE`:
```
NE: (obs.x + CELL_CM + c,  obs.y + CELL_CM + c)
NW: (obs.x - c,             obs.y + CELL_CM + c)
SE: (obs.x + CELL_CM + c,  obs.y - c)
SW: (obs.x - c,             obs.y - c)
N:  (obs.x + CELL_CM/2,    obs.y + CELL_CM + c)
S:  (obs.x + CELL_CM/2,    obs.y - c)
E:  (obs.x + CELL_CM + c,  obs.y + CELL_CM/2)
W:  (obs.x - c,             obs.y + CELL_CM/2)
```

For each candidate waypoint `w`:
- Check `q1 → w` direct path is clear
- Check `w → q2` direct path is clear
- If both clear, record the two-segment total distance

Pick the shortest valid two-segment route.

### Fallback
If no bypass waypoint produces a fully clear path, return the direct path (may clip an obstacle — same behaviour as the current Dubins fallback).

### `_path_in_bounds` update
Add `RL`/`RR` handling: these commands don't move the robot position, so update the tracked heading and skip bounds/obstacle checks for those steps.

```
elif cmd.kind in ('RL', 'RR'):
    sign = 1 if cmd.kind == 'RL' else -1
    theta = (theta + sign * cmd.value) % 360
    remaining = 0   # consume entirely; no position check needed
```

Remove the `AL`/`AR` branch.

---

## 5. Hamiltonian Ordering

The Hamiltonian ordering cost is updated from Dubins total path length to **Euclidean straight-line distance**:

```
cost = sum of math.dist((qi.x, qi.y), (qi+1.x, qi+1.y)) for each consecutive pair
```

The brute-force 5! = 120 permutation search is unchanged.

Helpers removed: `_total_dubins_length`, `_dubins_bounded`, `_all_dubins`, `dubins_to_commands`, `_SEGMENT_KINDS`.

---

## 6. Files Changed

| File | Change |
|---|---|
| `Algorithm/simulator/planner.py` | Rewrite `_plan_leg`; add `_angle_diff`; add bypass waypoint logic; update `_path_in_bounds`; update Hamiltonian cost; remove all Dubins helpers and imports |
| `Algorithm/simulator/robot.py` | Rename `TL/TR` → `RL/RR`; remove `AL/AR` arc branches |
| `Algorithm/simulator/config.py` | No change (`TURN_RADIUS_CM` stays as documented constant) |
| `Algorithm/simulator/dubins.py` | No change (kept for reference, not imported by planner) |
| `Algorithm/simulator/types.py` | No change |
| `Algorithm/simulator/tests/test_logic.py` | Update command kind assertions (`AL/AR` removed; `TL/TR` → `RL/RR`) |

---

## 7. Tests

### 7.1 `_angle_diff`
```python
def test_angle_diff_no_rotation():      assert _angle_diff(90, 90) == 0
def test_angle_diff_left():             assert abs(_angle_diff(0, 90) - 90) < 0.01
def test_angle_diff_right():            assert abs(_angle_diff(90, 0) - (-90)) < 0.01
def test_angle_diff_shortest_path():    assert abs(_angle_diff(10, 350) - (-20)) < 0.01  # right is shorter
```

### 7.2 `_plan_leg_direct` (clear path)
```python
def test_plan_leg_direct_produces_rl_fw():
    # q1 facing East, q2 directly North — expect RR/RL + FW + RL/RR, no AL/AR
    cmds, dist = _plan_leg(q1, q2, obstacles=[])
    assert all(c.kind in ('FW', 'BW', 'RL', 'RR', 'WAIT') for c in cmds)
    assert any(c.kind == 'FW' for c in cmds)
```

### 7.3 Integration
```python
def test_get_commands_uses_rl_rr_not_arcs():
    cmds = get_commands(OBSTACLES)
    assert not any(c.kind in ('AL', 'AR') for c in cmds)
    assert any(c.kind in ('RL', 'RR') for c in cmds)
```

### 7.4 Existing tests
All existing approach pose and Hamiltonian ordering tests remain valid — their logic is unchanged.
