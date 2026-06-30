# MDP Simulator — Stage 1 Design Spec
**Date:** 2026-06-30
**Scope:** B.1 checklist item — pygame simulator shell with arena rendering and animated robot movement
**Out of scope:** Dubins path geometry (Stage 2), Hamiltonian ordering (Stage 3), socket integration (Week 5)

---

## 1. File Structure

```
Algorithm/
  simulator/
    config.py     ← all constants (scale, arena size, animation speed, robot dims)
    arena.py      ← draws grid, start zone, obstacles; owns cm→px conversion
    robot.py      ← robot state dataclass + drawing logic
    planner.py    ← returns List[Command]; hardcoded for Stage 1, replaced in Stage 2
    main.py       ← pygame game loop, wires all modules together
```

---

## 2. Constants (`config.py`)

| Constant | Value | Notes |
|---|---|---|
| `CELL_CM` | 10 | cm per grid cell |
| `GRID_SIZE` | 20 | 20×20 cells |
| `CELL_PX` | 40 | pixels per cell → 800×800 window |
| `ARENA_PX` | 800 | `GRID_SIZE × CELL_PX` |
| `FPS` | 60 | frame rate |
| `TURN_RADIUS_CM` | 25.0 | configurable; used in Stage 2, declared here |
| `ROBOT_W_CM` | 20 | robot width (cm) |
| `ROBOT_H_CM` | 21 | robot height (cm) |
| `STEP_CM_PER_FRAME` | 2 | forward/backward animation speed (cm/frame) |
| `DEG_PER_FRAME` | 3 | rotation animation speed (degrees/frame) |

All constants are single-line changes. No magic numbers elsewhere in the codebase.

---

## 3. Data Model

### Robot state (`robot.py`)
```python
@dataclass
class RobotState:
    x: float      # cm, bottom-left of robot footprint, bottom-left-origin frame
    y: float      # cm
    theta: float  # degrees; 0=East, 90=North (standard math convention)
```
Initial position: `x=20, y=20, theta=90` (centre of 40×40cm start zone, facing North).

### Obstacle (`arena.py` or shared dataclass)
```python
@dataclass
class Obstacle:
    x: int    # cm, bottom-left of 10×10cm footprint
    y: int    # cm
    face: str # 'N' | 'S' | 'E' | 'W' — target image face
```
5 obstacles hardcoded in `main.py` for Stage 1. No obstacle-placement UI (that's Android's job, C.5–C.7).

### Command primitive (`planner.py`)
```python
@dataclass
class Command:
    kind: str      # 'FW' | 'BW' | 'TL' | 'TR'
    value: float   # cm for FW/BW; degrees for TL/TR
```

---

## 4. Coordinate System

- **Logical frame:** bottom-left origin, Y-up, units in cm. Matches the algorithm briefing's `(h, k, F)` notation and `PROTOCOL.md`.
- **Screen frame:** pygame uses top-left origin, Y-down.
- **Single conversion point** in `arena.py`:
```python
def cm_to_px(x_cm: float, y_cm: float) -> tuple[int, int]:
    px = int(x_cm * CELL_PX / CELL_CM)
    py = int(ARENA_PX - y_cm * CELL_PX / CELL_CM)
    return px, py
```
No other module performs coordinate conversion — they call this function.

---

## 5. Rendering

Draw order each frame: **arena → obstacles → robot** (robot always on top).

### `arena.py`
- Dark background fill
- Light grey grid lines at every cell boundary
- Start zone: semi-transparent green filled rectangle (4×4 cells, bottom-left)
- Each obstacle: grey filled 1×1 cell square + orange tick mark on the target face edge

### `robot.py`
- Filled rectangle (20×21cm footprint) rotated to current `theta`
- Filled triangle on front-centre as facing indicator
- Drawn at robot's current `(x, y, theta)` — no trail

---

## 6. Animation Loop (`main.py`)

State machine with two variables: `active_command: Command | None` and `remaining: float` (cm or degrees left).

```
each frame:
  1. Handle pygame events (quit / keyboard)
  2. If active_command is None and queue non-empty → pop next, set remaining
  3. If active_command:
       FW/BW → advance robot position by STEP_PX_PER_FRAME along theta; decrement remaining
       TL/TR → rotate robot by DEG_PER_FRAME; decrement remaining
       if remaining ≤ 0 → clear active_command (command complete)
  4. Draw: arena.draw() → arena.draw_obstacles() → robot.draw()
  5. pygame.display.flip(); clock.tick(FPS)
```

### Keyboard controls
| Key | Action |
|---|---|
| `SPACE` | Pause / resume animation |
| `R` | Reset robot to start pose, reload command queue |
| `Q` / `ESC` | Quit |

---

## 7. Hardcoded Test Scenario (`planner.py` Stage 1)

5 obstacles at fixed positions to exercise the full arena:

| # | x (cm) | y (cm) | face |
|---|---|---|---|
| 1 | 50 | 50 | N |
| 2 | 100 | 30 | E |
| 3 | 150 | 80 | S |
| 4 | 80 | 130 | W |
| 5 | 130 | 160 | N |

Hardcoded command list visits approximate waypoints near each obstacle. Purpose: prove the animation loop and rendering work end-to-end before real path planning is wired in.

---

## 8. Stage 2 Integration Points

These are the only changes needed when Stage 2 (Dubins) is ready:

1. **`planner.py`** — replace hardcoded `get_commands()` body with real Dubins + Hamiltonian logic. Signature unchanged.
2. **`config.py`** — `TURN_RADIUS_CM` already declared, will be consumed by Stage 2 planner.
3. **`robot.py` turns** — `TL`/`TR` rotate-in-place can be swapped for arc-path animation. Main loop state machine does not need to change.

---

## 9. Open Items (do not assume)

- `PROTOCOL.md` §3 coordinate frame not yet ratified by team — verify before wiring in socket commands.
- `TURN_RADIUS_CM = 25.0` is an estimate; measure empirically on the real car before Stage 2.
- Grid resolution (10cm/20×20) is the agreed default; confirm against `MDP_Functional_Specification.md` if it appears in the repo.
