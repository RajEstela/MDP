# MDP Simulator Stage 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a pygame simulator shell that renders the 200×200cm arena with obstacles and animates a hardcoded FW/BW/TL/TR command sequence on the robot, satisfying checklist item B.1.

**Architecture:** Six focused modules — `config.py` (constants), `types.py` (dataclasses), `arena.py` (coordinate conversion + rendering), `robot.py` (pure movement logic + rendering), `planner.py` (hardcoded command list), `main.py` (pygame game loop). Pure logic functions are separated from pygame calls so they can be unit tested without a display.

**Tech Stack:** Python 3.11+, pygame 2.5+, pytest 8.0+, stdlib (math, dataclasses)

## Global Constraints

- All distances in code: cm. All angles: degrees. All screen coordinates: px.
- Coordinate frame: bottom-left origin, Y-up in cm-space. `cm_to_px()` in `arena.py` is the single point that flips to pygame's Y-down screen space. No other module does coordinate math.
- Robot initial pose: `x=0.0, y=0.0, theta=90.0` (bottom-left of start zone, facing North).
- Grid: 20×20 cells at 10 cm/cell and 40 px/cell → 800×800 px window.
- No magic numbers outside `config.py`.
- All tests run from the `Algorithm/` directory: `python -m pytest simulator/tests/ -v`
- Run simulator from `Algorithm/` directory: `python -m simulator.main`

---

### Task 1: Project scaffold + config + data types

**Files:**
- Create: `Algorithm/requirements.txt`
- Create: `Algorithm/simulator/__init__.py`
- Create: `Algorithm/simulator/config.py`
- Create: `Algorithm/simulator/types.py`
- Create: `Algorithm/simulator/tests/__init__.py`
- Create: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Produces from `config`: `CELL_CM=10`, `GRID_SIZE=20`, `CELL_PX=40`, `ARENA_PX=800`, `FPS=60`, `TURN_RADIUS_CM=25.0`, `ROBOT_W_CM=20`, `ROBOT_H_CM=21`, `STEP_CM_PER_FRAME=2.0`, `DEG_PER_FRAME=3.0`
- Produces from `types`: `RobotState(x: float, y: float, theta: float)`, `Obstacle(x: int, y: int, face: str)`, `Command(kind: str, value: float)`

- [ ] **Step 1: Write the failing tests**

Create `Algorithm/simulator/tests/test_logic.py`:
```python
from simulator.types import RobotState, Obstacle, Command


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
```

- [ ] **Step 2: Run tests to confirm they fail**

```
cd Algorithm
python -m pytest simulator/tests/test_logic.py -v
```
Expected: `ERROR` — `ModuleNotFoundError: No module named 'simulator.types'`

- [ ] **Step 3: Create scaffold and implement config + types**

`Algorithm/requirements.txt`:
```
pygame>=2.5.0
pytest>=8.0.0
```

`Algorithm/simulator/__init__.py` — empty file.

`Algorithm/simulator/tests/__init__.py` — empty file.

`Algorithm/simulator/config.py`:
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
```

`Algorithm/simulator/types.py`:
```python
from dataclasses import dataclass


@dataclass
class RobotState:
    x: float
    y: float
    theta: float  # degrees; 0=East, 90=North (standard math convention, CCW positive)


@dataclass
class Obstacle:
    x: int
    y: int
    face: str  # 'N' | 'S' | 'E' | 'W' — which face carries the target image


@dataclass
class Command:
    kind: str    # 'FW' | 'BW' | 'TL' | 'TR'
    value: float # cm for FW/BW; degrees for TL/TR
```

- [ ] **Step 4: Install requirements**

```
cd Algorithm
pip install -r requirements.txt
```
Expected: `Successfully installed pygame-... pytest-...` (or "already satisfied")

- [ ] **Step 5: Run tests to confirm they pass**

```
python -m pytest simulator/tests/test_logic.py -v
```
Expected: `3 passed`

- [ ] **Step 6: Commit**

```bash
git add Algorithm/
git commit -m "feat: project scaffold, config constants, and data types"
```

---

### Task 2: Pure movement logic (robot.py)

**Files:**
- Create: `Algorithm/simulator/robot.py`
- Modify: `Algorithm/simulator/tests/test_logic.py` (add movement tests)

**Interfaces:**
- Consumes: `RobotState`, `Command` from `simulator.types`; `STEP_CM_PER_FRAME`, `DEG_PER_FRAME` from `simulator.config`
- Produces: `move_forward(state: RobotState, cm: float) -> RobotState`, `rotate(state: RobotState, deg: float, clockwise: bool) -> RobotState`, `step_command(state: RobotState, cmd: Command, remaining: float) -> tuple[RobotState, float]`

- [ ] **Step 1: Add failing movement tests**

Append to `Algorithm/simulator/tests/test_logic.py`:
```python
from simulator.robot import move_forward, rotate, step_command


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
```

- [ ] **Step 2: Run to confirm failures**

```
python -m pytest simulator/tests/test_logic.py -v
```
Expected: 3 pass (from Task 1), 10 fail with `ImportError: cannot import name 'move_forward'`

- [ ] **Step 3: Implement robot.py**

Create `Algorithm/simulator/robot.py`:
```python
import math

from simulator.config import DEG_PER_FRAME, STEP_CM_PER_FRAME
from simulator.types import Command, RobotState


def move_forward(state: RobotState, cm: float) -> RobotState:
    rad = math.radians(state.theta)
    return RobotState(
        x=state.x + cm * math.cos(rad),
        y=state.y + cm * math.sin(rad),
        theta=state.theta,
    )


def rotate(state: RobotState, deg: float, clockwise: bool) -> RobotState:
    delta = -deg if clockwise else deg
    return RobotState(x=state.x, y=state.y, theta=(state.theta + delta) % 360)


def step_command(
    state: RobotState, cmd: Command, remaining: float
) -> tuple[RobotState, float]:
    if cmd.kind == 'FW':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return move_forward(state, advance), remaining - advance
    if cmd.kind == 'BW':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return move_forward(state, -advance), remaining - advance
    if cmd.kind == 'TL':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=False), remaining - advance
    if cmd.kind == 'TR':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=True), remaining - advance
    return state, 0.0
```

- [ ] **Step 4: Run all tests to confirm they pass**

```
python -m pytest simulator/tests/test_logic.py -v
```
Expected: `13 passed`

- [ ] **Step 5: Commit**

```bash
git add Algorithm/simulator/robot.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat: pure movement logic with unit tests"
```

---

### Task 3: Coordinate conversion (arena.py)

**Files:**
- Create: `Algorithm/simulator/arena.py` (cm_to_px only — draw functions added in Task 5)
- Modify: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Consumes: `CELL_PX`, `CELL_CM`, `ARENA_PX` from `simulator.config`
- Produces: `cm_to_px(x_cm: float, y_cm: float) -> tuple[int, int]`

- [ ] **Step 1: Add failing coordinate conversion tests**

Append to `Algorithm/simulator/tests/test_logic.py`:
```python
from simulator.arena import cm_to_px


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
```

- [ ] **Step 2: Run to confirm failures**

```
python -m pytest simulator/tests/test_logic.py -v
```
Expected: 13 pass, 4 fail with `ImportError: cannot import name 'cm_to_px'`

- [ ] **Step 3: Create arena.py with cm_to_px**

Create `Algorithm/simulator/arena.py`:
```python
from simulator.config import ARENA_PX, CELL_CM, CELL_PX


def cm_to_px(x_cm: float, y_cm: float) -> tuple[int, int]:
    px = int(x_cm * CELL_PX / CELL_CM)
    py = int(ARENA_PX - y_cm * CELL_PX / CELL_CM)
    return px, py
```

- [ ] **Step 4: Run all tests to confirm they pass**

```
python -m pytest simulator/tests/test_logic.py -v
```
Expected: `17 passed`

- [ ] **Step 5: Commit**

```bash
git add Algorithm/simulator/arena.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat: coordinate conversion cm_to_px with unit tests"
```

---

### Task 4: Hardcoded planner (planner.py)

**Files:**
- Create: `Algorithm/simulator/planner.py`
- Modify: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Consumes: `Obstacle`, `Command` from `simulator.types`
- Produces: `OBSTACLES: list[Obstacle]` (5 items), `get_commands(obstacles: list[Obstacle]) -> list[Command]`

- [ ] **Step 1: Add failing planner tests**

Append to `Algorithm/simulator/tests/test_logic.py`:
```python
from simulator.planner import OBSTACLES, get_commands


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
```

- [ ] **Step 2: Run to confirm failures**

```
python -m pytest simulator/tests/test_logic.py -v
```
Expected: 17 pass, 5 fail with `ImportError: cannot import name 'OBSTACLES'`

- [ ] **Step 3: Implement planner.py**

Create `Algorithm/simulator/planner.py`:
```python
from simulator.types import Command, Obstacle

OBSTACLES: list[Obstacle] = [
    Obstacle(x=50, y=50, face='N'),
    Obstacle(x=100, y=30, face='E'),
    Obstacle(x=150, y=80, face='S'),
    Obstacle(x=80, y=130, face='W'),
    Obstacle(x=130, y=160, face='N'),
]


def get_commands(obstacles: list[Obstacle]) -> list[Command]:
    # Stage 1: hardcoded sequence to exercise the animation loop.
    # Stage 2 replaces this body with Dubins path + Hamiltonian ordering.
    return [
        Command('FW', 50),
        Command('TR', 90),
        Command('FW', 60),
        Command('TL', 45),
        Command('FW', 40),
        Command('TR', 90),
        Command('BW', 20),
        Command('TL', 90),
        Command('FW', 80),
    ]
```

- [ ] **Step 4: Run all tests to confirm they pass**

```
python -m pytest simulator/tests/test_logic.py -v
```
Expected: `22 passed`

- [ ] **Step 5: Commit**

```bash
git add Algorithm/simulator/planner.py Algorithm/simulator/tests/test_logic.py
git commit -m "feat: hardcoded planner stub with unit tests"
```

---

### Task 5: Rendering (arena.py draw functions + robot.py draw function)

**Files:**
- Modify: `Algorithm/simulator/arena.py` (add `draw_arena`, `draw_obstacles`)
- Modify: `Algorithm/simulator/robot.py` (add `draw_robot`)

No unit tests for draw functions (they require a pygame display). Verified by the visual smoke test in Step 3.

**Interfaces:**
- Consumes: `cm_to_px` from `simulator.arena`; `RobotState`, `Obstacle` from `simulator.types`; all constants from `simulator.config`
- Produces: `draw_arena(surface: pygame.Surface) -> None`, `draw_obstacles(surface: pygame.Surface, obstacles: list[Obstacle]) -> None`, `draw_robot(surface: pygame.Surface, state: RobotState) -> None`

- [ ] **Step 1: Add draw_arena and draw_obstacles to arena.py**

Replace the contents of `Algorithm/simulator/arena.py` with:
```python
import pygame

from simulator.config import ARENA_PX, CELL_CM, CELL_PX, GRID_SIZE
from simulator.types import Obstacle


def cm_to_px(x_cm: float, y_cm: float) -> tuple[int, int]:
    px = int(x_cm * CELL_PX / CELL_CM)
    py = int(ARENA_PX - y_cm * CELL_PX / CELL_CM)
    return px, py


def draw_arena(surface: pygame.Surface) -> None:
    surface.fill((30, 30, 30))
    for i in range(GRID_SIZE + 1):
        pos = i * CELL_PX
        pygame.draw.line(surface, (60, 60, 60), (pos, 0), (pos, ARENA_PX))
        pygame.draw.line(surface, (60, 60, 60), (0, pos), (ARENA_PX, pos))
    # Start zone: 40×40cm = 4×4 cells, bottom-left of arena
    _, top_py = cm_to_px(0, 40)
    start_size = 4 * CELL_PX
    start_surf = pygame.Surface((start_size, start_size), pygame.SRCALPHA)
    start_surf.fill((0, 180, 0, 80))
    surface.blit(start_surf, (0, top_py))


def draw_obstacles(surface: pygame.Surface, obstacles: list[Obstacle]) -> None:
    for obs in obstacles:
        left_px, bottom_py = cm_to_px(obs.x, obs.y)
        _, top_py = cm_to_px(obs.x, obs.y + CELL_CM)
        size_px = CELL_PX
        right_px = left_px + size_px
        rect = pygame.Rect(left_px, top_py, size_px, size_px)
        pygame.draw.rect(surface, (150, 150, 150), rect)
        pygame.draw.rect(surface, (80, 80, 80), rect, 1)
        tick_color = (255, 140, 0)
        tick_w = 4
        if obs.face == 'N':
            pygame.draw.line(surface, tick_color, (left_px, top_py), (right_px, top_py), tick_w)
        elif obs.face == 'S':
            pygame.draw.line(surface, tick_color, (left_px, bottom_py), (right_px, bottom_py), tick_w)
        elif obs.face == 'E':
            pygame.draw.line(surface, tick_color, (right_px, top_py), (right_px, bottom_py), tick_w)
        elif obs.face == 'W':
            pygame.draw.line(surface, tick_color, (left_px, top_py), (left_px, bottom_py), tick_w)
```

- [ ] **Step 2: Add draw_robot to robot.py**

Replace the full contents of `Algorithm/simulator/robot.py` (keep existing functions, add imports and draw_robot):
```python
import math

import pygame

from simulator.arena import cm_to_px
from simulator.config import CELL_CM, CELL_PX, DEG_PER_FRAME, ROBOT_H_CM, ROBOT_W_CM, STEP_CM_PER_FRAME
from simulator.types import Command, RobotState


def move_forward(state: RobotState, cm: float) -> RobotState:
    rad = math.radians(state.theta)
    return RobotState(
        x=state.x + cm * math.cos(rad),
        y=state.y + cm * math.sin(rad),
        theta=state.theta,
    )


def rotate(state: RobotState, deg: float, clockwise: bool) -> RobotState:
    delta = -deg if clockwise else deg
    return RobotState(x=state.x, y=state.y, theta=(state.theta + delta) % 360)


def step_command(
    state: RobotState, cmd: Command, remaining: float
) -> tuple[RobotState, float]:
    if cmd.kind == 'FW':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return move_forward(state, advance), remaining - advance
    if cmd.kind == 'BW':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return move_forward(state, -advance), remaining - advance
    if cmd.kind == 'TL':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=False), remaining - advance
    if cmd.kind == 'TR':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=True), remaining - advance
    return state, 0.0


def draw_robot(surface: pygame.Surface, state: RobotState) -> None:
    w_px = int(ROBOT_W_CM * CELL_PX / CELL_CM)  # 80px
    h_px = int(ROBOT_H_CM * CELL_PX / CELL_CM)  # 84px

    robot_surf = pygame.Surface((w_px, h_px), pygame.SRCALPHA)
    pygame.draw.rect(robot_surf, (30, 100, 200), (0, 0, w_px, h_px))

    # Facing arrow: triangle pointing right (East = theta=0, the unrotated default)
    arrow = [
        (w_px, h_px // 2),
        (w_px - 15, h_px // 2 - 10),
        (w_px - 15, h_px // 2 + 10),
    ]
    pygame.draw.polygon(robot_surf, (255, 220, 0), arrow)

    # pygame.transform.rotate is CCW in screen space; our theta is CCW from East;
    # pygame's Y-flip means screen-CCW visually matches math-CCW convention.
    rotated = pygame.transform.rotate(robot_surf, state.theta)

    cx_cm = state.x + ROBOT_W_CM / 2
    cy_cm = state.y + ROBOT_H_CM / 2
    cx_px, cy_px = cm_to_px(cx_cm, cy_cm)
    rect = rotated.get_rect(center=(cx_px, cy_px))
    surface.blit(rotated, rect)
```

- [ ] **Step 3: Confirm unit tests still pass**

```
python -m pytest simulator/tests/test_logic.py -v
```
Expected: `22 passed` (draw functions are untested; just ensure no import errors broke existing tests)

- [ ] **Step 4: Commit**

```bash
git add Algorithm/simulator/arena.py Algorithm/simulator/robot.py
git commit -m "feat: arena and robot rendering functions"
```

---

### Task 6: Main game loop (main.py)

**Files:**
- Create: `Algorithm/simulator/main.py`

No automated tests — verified with a visual walkthrough.

**Interfaces:**
- Consumes: `draw_arena`, `draw_obstacles`, `cm_to_px` from `simulator.arena`; `draw_robot`, `step_command` from `simulator.robot`; `OBSTACLES`, `get_commands` from `simulator.planner`; `RobotState` from `simulator.types`; `ARENA_PX`, `FPS` from `simulator.config`

- [ ] **Step 1: Implement main.py**

Create `Algorithm/simulator/main.py`:
```python
import pygame

from simulator.arena import draw_arena, draw_obstacles
from simulator.config import ARENA_PX, FPS
from simulator.planner import OBSTACLES, get_commands
from simulator.robot import draw_robot, step_command
from simulator.types import RobotState


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((ARENA_PX, ARENA_PX))
    pygame.display.set_caption("MDP Simulator — Stage 1")
    clock = pygame.time.Clock()

    commands = get_commands(OBSTACLES)
    start_pose = RobotState(x=0.0, y=0.0, theta=90.0)

    state = RobotState(x=start_pose.x, y=start_pose.y, theta=start_pose.theta)
    queue: list = list(commands)
    active = None
    remaining: float = 0.0
    paused = False

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    state = RobotState(x=start_pose.x, y=start_pose.y, theta=start_pose.theta)
                    queue = list(commands)
                    active = None
                    remaining = 0.0

        if not paused:
            if active is None and queue:
                active = queue.pop(0)
                remaining = active.value
            if active is not None:
                state, remaining = step_command(state, active, remaining)
                if remaining <= 0:
                    active = None

        screen.fill((30, 30, 30))
        draw_arena(screen)
        draw_obstacles(screen, OBSTACLES)
        draw_robot(screen, state)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Run all unit tests to confirm nothing is broken**

```
python -m pytest simulator/tests/test_logic.py -v
```
Expected: `22 passed`

- [ ] **Step 3: Visual integration test — run the simulator**

```
python -m simulator.main
```

Verify each of the following before proceeding:

| Check | Expected |
|---|---|
| Window opens | 800×800 dark grey window titled "MDP Simulator — Stage 1" |
| Grid | Light grey lines every 40px (10cm) |
| Start zone | Semi-transparent green 160×160px block at bottom-left |
| Obstacles | 5 grey squares at correct positions, each with an orange tick on the named face (N=top, S=bottom, E=right, W=left) |
| Robot at start | Blue rectangle with yellow arrow, positioned at bottom-left of start zone, arrow pointing UP (North) |
| Animation | Robot moves through FW/TR/FW/TL/BW sequence smoothly |
| SPACE | Pauses and resumes animation |
| R | Robot snaps back to start, animation replays from beginning |
| Q / ESC | Window closes cleanly |

- [ ] **Step 4: Commit**

```bash
git add Algorithm/simulator/main.py
git commit -m "feat: pygame game loop — Stage 1 simulator complete (B.1)"
```
