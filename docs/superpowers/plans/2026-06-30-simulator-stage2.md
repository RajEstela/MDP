# MDP Simulator Stage 2 — Dubins Paths + Arc Animation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the Stage 1 pygame simulator with hand-rolled Dubins path geometry and real arc animation, replacing the hardcoded FW/TL/TR demo with smooth curved motion through three waypoints.

**Architecture:** A pure-Python `dubins.py` module computes the 6 Dubins path types analytically and picks the shortest; `robot.py` gains an `arc_step` function that advances the robot along a circular arc per frame; `planner.py` chains `dubins_optimal → dubins_to_commands` to convert waypoints into `Command` objects that `main.py` already consumes unchanged.

**Tech Stack:** Python 3.14, pytest 8+, pygame-ce 2.5+ (no changes to rendering in this stage)

## Global Constraints

- Python 3.14; no `import pygame` at module level — pygame imports only inside `draw_*` functions
- Test runner: `python -m pytest simulator/tests/test_logic.py -v` from `Algorithm/` directory
- All tests are pure-Python (no pygame, no display required)
- `TURN_RADIUS_CM = 25.0` from `simulator.config` — do not hardcode 25 anywhere
- `STEP_CM_PER_FRAME = 2.0` from `simulator.config` — do not hardcode 2 anywhere
- Coordinate convention: x right, y up, theta in degrees CCW from East (0=East, 90=North)
- `DubinsPath.total` must equal `seg1 + seg2 + seg3` exactly as passed to the constructor
- Zero-segment threshold: segments with `abs(seg) < 0.01` are dropped by `dubins_to_commands`
- `dubins.py` must not import from `config`, `arena`, `robot`, or `pygame` — pure geometry only
- No changes to `main.py`, `arena.py`, or `config.py`
- Commit after each task; use `git add <specific files>` not `git add .`

---

### Task 1: `DubinsPath` dataclass + `arc_step` kinematics + `AL`/`AR` in `step_command`

**Files:**
- Modify: `Algorithm/simulator/types.py`
- Modify: `Algorithm/simulator/robot.py`
- Modify: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Consumes: `RobotState`, `Command` already in `simulator.types`; `STEP_CM_PER_FRAME`, `TURN_RADIUS_CM` from `simulator.config`
- Produces:
  - `DubinsPath` dataclass in `simulator.types` with fields `path_type: str`, `seg1: float`, `seg2: float`, `seg3: float`, `total: float`
  - `arc_step(state: RobotState, ds: float, clockwise: bool, r: float) -> RobotState` in `simulator.robot`
  - `step_command` extended to handle `'AL'` (arc-left / CCW) and `'AR'` (arc-right / CW) command kinds

---

- [ ] **Step 1: Add `import math` and new test imports to `test_logic.py`, then write all failing tests for this task**

  Open `Algorithm/simulator/tests/test_logic.py`. Change the top of the file so it reads:

  ```python
  import math

  from simulator.arena import cm_to_px
  from simulator.planner import OBSTACLES, get_commands
  from simulator.robot import arc_step, move_forward, rotate, step_command
  from simulator.types import Command, DubinsPath, Obstacle, RobotState
  ```

  Then append these 6 tests at the bottom of the file:

  ```python
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
  ```

- [ ] **Step 2: Run the new tests to verify they FAIL**

  ```
  cd Algorithm
  python -m pytest simulator/tests/test_logic.py -v -k "dubins_path or arc_step or al_arcs or ar_arcs"
  ```

  Expected: 6 FAILED with `ImportError: cannot import name 'DubinsPath'` and `ImportError: cannot import name 'arc_step'`

- [ ] **Step 3: Add `DubinsPath` to `types.py`**

  Open `Algorithm/simulator/types.py`. Replace the entire file with:

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
      kind: str    # 'FW' | 'BW' | 'TL' | 'TR' | 'AL' | 'AR'
      value: float # cm for FW/BW/AL/AR; degrees for TL/TR


  @dataclass
  class DubinsPath:
      path_type: str  # 'LSL' | 'LSR' | 'RSL' | 'RSR' | 'RLR' | 'LRL'
      seg1: float     # first segment length in cm
      seg2: float     # second segment length in cm
      seg3: float     # third segment length in cm
      total: float    # seg1 + seg2 + seg3
  ```

- [ ] **Step 4: Run `test_dubins_path_fields` alone to verify it PASSES**

  ```
  python -m pytest simulator/tests/test_logic.py::test_dubins_path_fields -v
  ```

  Expected: 1 PASSED

- [ ] **Step 5: Add `arc_step` to `robot.py` and extend `step_command` with `'AL'`/`'AR'`**

  Open `Algorithm/simulator/robot.py`. Replace the entire file with:

  ```python
  import math
  from typing import TYPE_CHECKING

  from simulator.arena import cm_to_px
  from simulator.config import CELL_CM, CELL_PX, DEG_PER_FRAME, ROBOT_H_CM, ROBOT_W_CM, STEP_CM_PER_FRAME, TURN_RADIUS_CM
  from simulator.types import Command, RobotState

  if TYPE_CHECKING:
      import pygame

  _ARROW_INSET_PX = 15  # arrow triangle depth in px
  _ARROW_HALF_PX = 10   # arrow triangle half-height in px


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


  def arc_step(state: RobotState, ds: float, clockwise: bool, r: float) -> RobotState:
      sign = -1 if clockwise else 1
      theta_rad = math.radians(state.theta)
      new_theta_rad = theta_rad + sign * ds / r
      new_x = state.x + sign * r * (math.sin(new_theta_rad) - math.sin(theta_rad))
      new_y = state.y - sign * r * (math.cos(new_theta_rad) - math.cos(theta_rad))
      return RobotState(x=new_x, y=new_y, theta=math.degrees(new_theta_rad) % 360)


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
      if cmd.kind == 'AL':
          advance = min(STEP_CM_PER_FRAME, remaining)
          return arc_step(state, advance, clockwise=False, r=TURN_RADIUS_CM), remaining - advance
      if cmd.kind == 'AR':
          advance = min(STEP_CM_PER_FRAME, remaining)
          return arc_step(state, advance, clockwise=True, r=TURN_RADIUS_CM), remaining - advance
      return state, 0.0


  def draw_robot(surface: "pygame.Surface", state: RobotState) -> None:
      import pygame

      w_px = int(ROBOT_W_CM * CELL_PX / CELL_CM)  # 80px
      h_px = int(ROBOT_H_CM * CELL_PX / CELL_CM)  # 84px

      robot_surf = pygame.Surface((w_px, h_px), pygame.SRCALPHA)
      pygame.draw.rect(robot_surf, (30, 100, 200), (0, 0, w_px, h_px))

      # Facing arrow: triangle pointing right (East = theta=0, the unrotated default)
      arrow = [
          (w_px, h_px // 2),
          (w_px - _ARROW_INSET_PX, h_px // 2 - _ARROW_HALF_PX),
          (w_px - _ARROW_INSET_PX, h_px // 2 + _ARROW_HALF_PX),
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

- [ ] **Step 6: Run all tests to verify all 28 pass**

  ```
  python -m pytest simulator/tests/test_logic.py -v
  ```

  Expected: 28 passed (22 existing + 6 new). If any existing tests fail, you have a regression — do not proceed until all pass.

- [ ] **Step 7: Commit**

  ```bash
  git add Algorithm/simulator/types.py Algorithm/simulator/robot.py Algorithm/simulator/tests/test_logic.py
  git commit -m "feat: add DubinsPath type, arc_step kinematics, AL/AR commands"
  ```

---

### Task 2: `dubins.py` — 6 Dubins path functions + `dubins_optimal`

**Files:**
- Create: `Algorithm/simulator/dubins.py`
- Modify: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Consumes: `DubinsPath`, `RobotState` from `simulator.types` (added in Task 1)
- Produces:
  - `dubins_lsl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None`
  - `dubins_rsr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None`
  - `dubins_lsr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None`
  - `dubins_rsl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None`
  - `dubins_rlr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None`
  - `dubins_lrl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None`
  - `dubins_optimal(q1: RobotState, q2: RobotState, r: float) -> DubinsPath`

  All functions accept positions in cm, heading in degrees; return segment lengths in cm.

---

- [ ] **Step 1: Append Dubins tests to `test_logic.py`**

  Add this import line at the top of `Algorithm/simulator/tests/test_logic.py`, after the existing imports:

  ```python
  from simulator.dubins import dubins_lrl, dubins_lsl, dubins_lsr, dubins_optimal, dubins_rlr, dubins_rsl, dubins_rsr
  ```

  Then append these tests at the bottom of the file:

  ```python
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
      q1 = RobotState(0, 0, 45)
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
  ```

- [ ] **Step 2: Run these new tests to verify they FAIL**

  ```
  python -m pytest simulator/tests/test_logic.py -v -k "dubins_straight or dubins_optimal or dubins_lsl or dubins_path_type or dubins_total or dubins_segments"
  ```

  Expected: 6 FAILED with `ModuleNotFoundError: No module named 'simulator.dubins'`

- [ ] **Step 3: Create `Algorithm/simulator/dubins.py`**

  Create the file with this content:

  ```python
  import math

  from simulator.types import DubinsPath, RobotState


  def _mod2pi(x: float) -> float:
      return x % (2 * math.pi)


  def dubins_lsl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
      dx = (q2.x - q1.x) / r
      dy = (q2.y - q1.y) / r
      alpha = math.radians(q1.theta)
      beta = math.radians(q2.theta)
      tmp0 = dx + math.sin(alpha) - math.sin(beta)
      tmp1 = dy - math.cos(alpha) + math.cos(beta)
      p_sq = tmp0 ** 2 + tmp1 ** 2
      if p_sq < 0:
          return None
      p = math.sqrt(p_sq)
      theta = math.atan2(tmp1, tmp0)
      t = _mod2pi(theta - alpha)
      q = _mod2pi(beta - theta)
      return DubinsPath('LSL', t * r, p * r, q * r, (t + p + q) * r)


  def dubins_rsr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
      dx = (q2.x - q1.x) / r
      dy = (q2.y - q1.y) / r
      alpha = math.radians(q1.theta)
      beta = math.radians(q2.theta)
      tmp0 = dx - math.sin(alpha) + math.sin(beta)
      tmp1 = dy + math.cos(alpha) - math.cos(beta)
      p_sq = tmp0 ** 2 + tmp1 ** 2
      if p_sq < 0:
          return None
      p = math.sqrt(p_sq)
      theta = math.atan2(tmp1, tmp0)
      t = _mod2pi(alpha - theta)
      q = _mod2pi(theta - beta)
      return DubinsPath('RSR', t * r, p * r, q * r, (t + p + q) * r)


  def dubins_lsr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
      dx = (q2.x - q1.x) / r
      dy = (q2.y - q1.y) / r
      alpha = math.radians(q1.theta)
      beta = math.radians(q2.theta)
      tmp0 = dx + math.sin(alpha) + math.sin(beta)
      tmp1 = dy - math.cos(alpha) - math.cos(beta)
      p_sq = tmp0 ** 2 + tmp1 ** 2 - 4
      if p_sq < 0:
          return None
      p = math.sqrt(p_sq)
      theta = math.atan2(-math.cos(alpha) - math.cos(beta), tmp0) - math.atan2(-2, p)
      t = _mod2pi(theta - alpha)
      q = _mod2pi(theta - beta)
      return DubinsPath('LSR', t * r, p * r, q * r, (t + p + q) * r)


  def dubins_rsl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
      dx = (q2.x - q1.x) / r
      dy = (q2.y - q1.y) / r
      alpha = math.radians(q1.theta)
      beta = math.radians(q2.theta)
      tmp0 = dx - math.sin(alpha) - math.sin(beta)
      tmp1 = dy + math.cos(alpha) + math.cos(beta)
      p_sq = tmp0 ** 2 + tmp1 ** 2 - 4
      if p_sq < 0:
          return None
      p = math.sqrt(p_sq)
      theta = math.atan2(math.cos(alpha) + math.cos(beta), tmp0) - math.atan2(2, p)
      t = _mod2pi(alpha - theta)
      q = _mod2pi(beta - theta)
      return DubinsPath('RSL', t * r, p * r, q * r, (t + p + q) * r)


  def dubins_rlr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
      dx = (q2.x - q1.x) / r
      dy = (q2.y - q1.y) / r
      alpha = math.radians(q1.theta)
      beta = math.radians(q2.theta)
      tmp0 = (
          (dx - math.sin(alpha) + math.sin(beta)) / 6
          + math.cos(alpha) / 3
          - math.cos(beta) / 3
      )
      if abs(tmp0) > 1:
          return None
      p = _mod2pi(2 * math.pi - math.acos(tmp0))
      t = _mod2pi(
          alpha
          - math.atan2(
              math.cos(alpha) - math.cos(beta),
              dx - math.sin(alpha) + math.sin(beta),
          )
          + p / 2
      )
      q = _mod2pi(alpha - beta - t + p)
      return DubinsPath('RLR', t * r, p * r, q * r, (t + p + q) * r)


  def dubins_lrl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
      dx = (q2.x - q1.x) / r
      dy = (q2.y - q1.y) / r
      alpha = math.radians(q1.theta)
      beta = math.radians(q2.theta)
      tmp0 = (
          (dx + math.sin(alpha) - math.sin(beta)) / 6
          - math.cos(alpha) / 3
          + math.cos(beta) / 3
      )
      if abs(tmp0) > 1:
          return None
      p = _mod2pi(2 * math.pi - math.acos(tmp0))
      t = _mod2pi(
          -alpha
          + math.atan2(
              -math.cos(alpha) + math.cos(beta),
              dx + math.sin(alpha) - math.sin(beta),
          )
          + p / 2
      )
      q = _mod2pi(beta - alpha - t + p)
      return DubinsPath('LRL', t * r, p * r, q * r, (t + p + q) * r)


  def dubins_optimal(q1: RobotState, q2: RobotState, r: float) -> DubinsPath:
      candidates = [
          dubins_lsl(q1, q2, r),
          dubins_rsr(q1, q2, r),
          dubins_lsr(q1, q2, r),
          dubins_rsl(q1, q2, r),
          dubins_rlr(q1, q2, r),
          dubins_lrl(q1, q2, r),
      ]
      return min((c for c in candidates if c is not None), key=lambda p: p.total)
  ```

- [ ] **Step 4: Run all 34 tests to verify they all PASS**

  ```
  python -m pytest simulator/tests/test_logic.py -v
  ```

  Expected: 34 passed (28 from Task 1 + 6 new). If any fail, check the formula sign conventions in the spec: §3.1 of `docs/superpowers/specs/2026-06-30-simulator-stage2-design.md`.

- [ ] **Step 5: Commit**

  ```bash
  git add Algorithm/simulator/dubins.py Algorithm/simulator/tests/test_logic.py
  git commit -m "feat: add dubins.py with 6 path types and optimal selection"
  ```

---

### Task 3: `planner.py` Stage 2 — `dubins_to_commands` + demo waypoints

**Files:**
- Modify: `Algorithm/simulator/planner.py`
- Modify: `Algorithm/simulator/tests/test_logic.py`

**Interfaces:**
- Consumes:
  - `DubinsPath` from `simulator.types` (Task 1)
  - `dubins_optimal(q1, q2, r)` from `simulator.dubins` (Task 2)
  - `TURN_RADIUS_CM` from `simulator.config`
  - `arc_step` via `step_command` with `'AL'`/`'AR'` (Task 1)
- Produces:
  - `dubins_to_commands(path: DubinsPath) -> list[Command]` in `simulator.planner`
  - `get_commands(obstacles: list[Obstacle]) -> list[Command]` — now returns AL/AR/FW commands

---

- [ ] **Step 1: Append planner tests to `test_logic.py`**

  Add this import to the top of `Algorithm/simulator/tests/test_logic.py`:

  ```python
  from simulator.planner import OBSTACLES, dubins_to_commands, get_commands
  ```

  (Replace the existing `from simulator.planner import OBSTACLES, get_commands` line.)

  Then append these tests at the bottom of the file:

  ```python
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
      assert cmds[0].kind == 'AR'
      assert cmds[1].kind == 'FW'
      assert cmds[2].kind == 'AR'


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
      assert kinds <= {'FW', 'BW', 'AL', 'AR'}
      assert 'AL' in kinds or 'AR' in kinds


  def test_get_commands_all_values_positive():
      cmds = get_commands(OBSTACLES)
      assert all(c.value > 0 for c in cmds)


  def test_dubins_path_reaches_target():
      q1 = RobotState(0, 0, 90)
      q2 = RobotState(100, 100, 0)
      path = dubins_optimal(q1, q2, r=25)
      cmds = dubins_to_commands(path)
      state = q1
      for cmd in cmds:
          remaining = cmd.value
          while remaining > 0.001:
              state, remaining = step_command(state, cmd, remaining)
      assert abs(state.x - q2.x) < 0.5
      assert abs(state.y - q2.y) < 0.5
      assert abs((state.theta - q2.theta + 180) % 360 - 180) < 1.0
  ```

  Also update `test_get_commands_all_valid_kinds` (currently at line 130) — change `valid` to include arc commands:

  ```python
  def test_get_commands_all_valid_kinds():
      cmds = get_commands(OBSTACLES)
      valid = {'FW', 'BW', 'TL', 'TR', 'AL', 'AR'}
      assert all(c.kind in valid for c in cmds)
  ```

- [ ] **Step 2: Run the new tests to verify they FAIL**

  ```
  python -m pytest simulator/tests/test_logic.py -v -k "dubins_to_commands or arc_commands or all_values_positive or reaches_target"
  ```

  Expected: 7 FAILED with `ImportError: cannot import name 'dubins_to_commands' from 'simulator.planner'`

- [ ] **Step 3: Rewrite `planner.py`**

  Replace the entire content of `Algorithm/simulator/planner.py` with:

  ```python
  from simulator.config import TURN_RADIUS_CM
  from simulator.dubins import dubins_optimal
  from simulator.types import Command, DubinsPath, Obstacle, RobotState

  OBSTACLES: list[Obstacle] = [
      Obstacle(x=50, y=50, face='N'),
      Obstacle(x=100, y=30, face='E'),
      Obstacle(x=150, y=80, face='S'),
      Obstacle(x=80, y=130, face='W'),
      Obstacle(x=130, y=160, face='N'),
  ]

  _SEGMENT_KINDS: dict[str, tuple[str, str, str]] = {
      'LSL': ('AL', 'FW', 'AL'),
      'LSR': ('AL', 'FW', 'AR'),
      'RSL': ('AR', 'FW', 'AL'),
      'RSR': ('AR', 'FW', 'AR'),
      'LRL': ('AL', 'AR', 'AL'),
      'RLR': ('AR', 'AL', 'AR'),
  }


  def dubins_to_commands(path: DubinsPath) -> list[Command]:
      k1, k2, k3 = _SEGMENT_KINDS[path.path_type]
      cmds = []
      for kind, seg in zip((k1, k2, k3), (path.seg1, path.seg2, path.seg3)):
          if seg > 0.01:
              cmds.append(Command(kind, seg))
      return cmds


  def get_commands(obstacles: list[Obstacle]) -> list[Command]:
      # Stage 2: Dubins paths through 3 hardcoded demo waypoints.
      # Stage 3 replaces this with obstacle approach poses + Hamiltonian ordering.
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

- [ ] **Step 4: Run the full test suite to verify all 41 tests pass**

  ```
  python -m pytest simulator/tests/test_logic.py -v
  ```

  Expected: 41 passed (34 from Tasks 1–2 + 7 new). If `test_get_commands_all_valid_kinds` or `test_get_commands_positive_values` fail, check the planner output — the old tests for those are now updated in the file.

  If you see failures on the old `test_get_commands_*` tests, verify you updated `test_get_commands_all_valid_kinds` in Step 1 to use `{'FW', 'BW', 'TL', 'TR', 'AL', 'AR'}`.

- [ ] **Step 5: Run the simulator visually and confirm arc motion**

  ```
  python -m simulator.main
  ```

  Press SPACE to start. You should see the robot follow smooth curved arcs (not point-turns). Press R to reset, Q to quit. If the robot moves only in straight lines or only rotates in place, `step_command` is not dispatching to `arc_step` — check the `'AL'`/`'AR'` branches added in Task 1.

- [ ] **Step 6: Commit**

  ```bash
  git add Algorithm/simulator/planner.py Algorithm/simulator/tests/test_logic.py
  git commit -m "feat: Stage 2 planner — dubins_to_commands and arc-animated waypoints"
  ```
