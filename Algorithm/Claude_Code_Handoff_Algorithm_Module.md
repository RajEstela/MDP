# MDP Algorithm Module — Handoff to Claude Code

**Course:** SC2079/CE3004/CZ3004 Multidisciplinary Design Project, NTU Special Term 2025/26
**My role:** P1 — Algorithm / Path Planning / Simulator
**Project window:** ~23 Jun – 23 Jul 2026. Week-3 checklist deadline is the immediate pressure point.

---

## 1. What this project actually is

Four-person team builds an autonomous robot car that navigates a physical 200cm×200cm arena, recognises target images mounted on obstacles, and reports results to an Android tablet. Grading is **checklist-based, progressive, per-item** (20% of grade, Week 3 deadline) plus a live competition, plus an individual quiz (per-member accountability — know your own module cold).

**Team split:**

- **P1 (me):** Algorithm / Path Planning / Simulator — pure software, runs on PC.
- **P2:** Android (Samsung Tab 7 Lite, Android Studio) — Bluetooth control, arena visualization GUI.
- **P3:** Image Recognition (PC/RPi-side) — detect/classify the 15-image pool.
- **P4:** Hardware (RPi 4 + STM32F103RCT6 firmware) — the critical path / single point of failure.

---

## 2. Checklist items I own (B.1–B.3)

| Item | Requirement | Status |
| --- | --- | --- |
| **B.1** | Simulator displays arena, start zone, obstacles, robot moving forward/back/turning. | ✅ Done |
| **B.2** | Hamiltonian-path algorithm: robot visits each image once from start zone. | ✅ Done |
| **B.3** | Shortest-time Hamiltonian path specifically (not just any valid path). | ✅ Done |

---

## 3. Arena ground truth

- **Movement area:** 200cm × 200cm (virtual boundaries). `ARENA_CM = 200`.
- **Start zone:** 40×40cm, bottom-left corner. Robot starts with apex at `(20, 35, 90°)` — body spans y=5..35, entirely inside the zone.
- **Grid:** 20×20 cells, each 10×10cm (`CELL_CM = 10`, `GRID_SIZE = 20`).
- **Obstacles:** 5 per run. 10×10cm footprint, axis-aligned. One face carries the target image.
- **Robot footprint:** 30cm × 30cm square, 3×3 grid cells (`ROBOT_W_CM = 30`, `ROBOT_H_CM = 30`).
- **Camera:** fixed, front-centre. Best recognition distance ≈ **20cm** → `APPROACH_CM = 20`.
- **Turning radius:** ~25cm → `TURN_RADIUS_CM = 25.0` (configurable, not yet empirically measured).
- **Coordinate frame:** bottom-left origin. θ=0° East, θ=90° North (increasing y), θ=180° West, θ=270° South.
- **Pixel scale:** 1 cm = 4 px (`CELL_PX = 40`), window is 800×800px (`ARENA_PX = 800`).

---

## 4. File structure

```text
Algorithm/
├── simulator/
│   ├── config.py       — all tuneable constants
│   ├── types.py        — RobotState, Command, DubinsPath, Obstacle dataclasses
│   ├── arena.py        — pygame grid/arena drawing, cm_to_px()
│   ├── robot.py        — kinematics (move_forward, arc_step, step_command), draw_robot
│   ├── dubins.py       — 6 Dubins path types + dubins_optimal()
│   ├── planner.py      — obstacle_approach_pose, Hamiltonian ordering, get_commands
│   ├── main.py         — pygame event loop, animation
│   └── tests/
│       └── test_logic.py   — 57 unit tests (all passing)
├── requirements.txt
└── Claude_Code_Handoff_Algorithm_Module.md  ← this file
```

Run simulator: `python -m simulator.main` from `Algorithm/`

Run tests: `python -m pytest simulator/tests/test_logic.py` from `Algorithm/`

---

## 5. What has been built — stage by stage

### Stage 1: Pygame skeleton

- 800×800px window, 20×20 grid, start zone, obstacle rendering.
- Robot drawn as a blue rectangle with yellow facing arrow.
- Manual/space-bar pause, `R` to reset, Q/Escape to quit.
- FPS = 60, `STEP_CM_PER_FRAME = 2.0`, `DEG_PER_FRAME = 3.0`.

### Stage 2: Dubins path geometry

All six path types implemented in `dubins.py`: LSL, LSR, RSL, RSR, RLR, LRL.

- `dubins_lsl`, `dubins_rsr` — always return a path (p_sq ≥ 0 guaranteed, guard removed).
- `dubins_lsr`, `dubins_rsl` — return `None` if p_sq < 4 (no valid straight segment exists).
- `dubins_rlr`, `dubins_lrl` — return `None` if `|tmp| > 1` (arcs don't reach).
- `dubins_optimal(q1, q2, r)` — tries all 6, returns shortest non-None.
- **Formula fix (LSR):** correct theta = `atan2(tmp1, tmp0) + atan2(2, p)` where `tmp1 = dy − cos α − cos β`. Original formula was missing `dy`.
- **Formula fix (RSL):** correct theta = `atan2(tmp1, tmp0) − atan2(2, p)` where `tmp1 = dy + cos α + cos β`.
- **Formula fix (RLR/LRL):** correct `tmp = 1.0 − d_sq/8.0`. Original had `d_sq/8.0 − 1.0`.
- Command kinds from Dubins segments: AL (arc left), AR (arc right), FW (forward straight).

### Stage 3: Obstacle approach poses + Hamiltonian ordering

`planner.py`:

- `obstacle_approach_pose(obs)` — computes the RobotState the robot must reach to face the obstacle's target image. Robot stops `APPROACH_CM = 20cm` from the obstacle face, facing directly toward it.
  - 'N' face → robot at `(cx, cy+25, 270°)` (north of obstacle, facing south)
  - 'S' face → `(cx, cy−25, 90°)`
  - 'E' face → `(cx+25, cy, 180°)`
  - 'W' face → `(cx−25, cy, 0°)`
- `_hamiltonian_optimal_order(start, poses, r)` — brute-forces all 5! = 120 permutations, picks lowest total bounded Dubins cost.
- `_dubins_bounded(q1, q2, r, obstacles)` — tries all 6 Dubins types sorted by length; picks first path that stays inside arena AND clears all obstacles (except the target). Falls back to shortest if none qualify.
- `_path_in_bounds(q1, cmds, r, obstacles)` — samples path at 2cm intervals; checks arena bounds AND 20cm clearance from every obstacle cell edge.
- `get_commands(obstacles)` — full pipeline: approach poses → Hamiltonian order → bounded Dubins path per leg → command list with `WAIT` after each obstacle.

### Simulator animation behaviour

- `WAIT` command: robot holds position for 5 seconds (300 frames at 60 FPS) after reaching each approach pose.
- `step_command` in `robot.py` handles: FW, BW, TL, TR, AL, AR, WAIT.

---

## 6. Key bugs fixed and why

| Bug | Root cause | Fix |
| --- | --- | --- |
| Robot left arena (early) | Arc kinematics sign error; start at (0,0) instead of (20,20) | Corrected `arc_step` sign convention; moved start to centre of start zone |
| Robot still left arena on 2nd obstacle | `Obstacle(x=100,y=30)` → approach y=35, only 35cm from bottom with r=25 | Moved demo obstacles so all approach poses have ≥40cm margin from boundaries |
| Hamiltonian used unconstrained cost | `_total_dubins_length` called `dubins_optimal` (ignores bounds/obstacles) | Changed to call `_dubins_bounded` so ordering prefers feasible paths |
| Robot collided with obstacles | `_path_in_bounds` only checked arena boundary, not obstacle cells | Added `_point_hits_obstacle` check with 20cm clearance around each obstacle cell |
| Robot couldn't approach target obstacle | Target obstacle's own 20cm clearance zone blocked the final approach | Exclude the target obstacle from collision check when computing that leg's path |

---

## 7. Current hardcoded demo obstacles

```python
OBSTACLES: list[Obstacle] = [
    Obstacle(x=50,  y=50,  face='N'),   # approach → (55,  80,  270°)
    Obstacle(x=100, y=60,  face='E'),   # approach → (130, 65,  180°)
    Obstacle(x=150, y=80,  face='S'),   # approach → (155, 60,   90°)
    Obstacle(x=80,  y=130, face='W'),   # approach → (60,  135,   0°)
    Obstacle(x=130, y=130, face='N'),   # approach → (135, 160, 270°)
]
```

All approach poses have ≥40cm clearance from every arena boundary.

---

## 8. Stage 4 — NOT YET STARTED

**Goal:** replace hardcoded demo obstacles with live data from the Android app via TCP socket.

**Architecture decided:**

- RPi is the TCP **server** (port TBD, ~5000).
- PC (running this simulator) is the TCP **client**.
- Android app sends obstacle data to RPi; RPi forwards to PC OR Android sends directly to PC — to be confirmed with P2/P4.
- Message format: `OBSTACLE,col,row,face\n` (newline-terminated).
  - `col`, `row` are 1-indexed grid cell coordinates (1–20).
  - `face` ∈ `{N, S, E, W}`.
  - 5 messages total, one per obstacle.

**Coordinate conversion (grid cell → cm):**

```python
obs_x_cm = (col - 1) * CELL_CM   # e.g. col=5 → x=40
obs_y_cm = (row - 1) * CELL_CM   # e.g. row=3 → y=20
```

**Planned implementation:**

- CLI flag `--connect <host>` to enable live mode.
- Without flag: uses hardcoded `OBSTACLES` (demo mode, current behaviour).
- With flag: pygame shows "Waiting for arena data..." screen while a background thread listens for 5 `OBSTACLE` messages, then starts the animation.
- Live-mode socket receives messages, parses into `list[Obstacle]`, calls `get_commands()`, begins animation — same pipeline as demo mode.

---

## 9. Known open questions

1. **TCP port** — not yet agreed with team. Placeholder ~5000.
2. **Who sends obstacle data to whom** — Android→RPi→PC or Android→PC directly? Confirm with P2/P4.
3. **Actual measured turning radius** — `TURN_RADIUS_CM = 25.0` is an estimate. Get real measurement from P4 and update `config.py`.
4. **PROTOCOL.md coordinate frame** — confirm the team's `PROTOCOL.md` uses the same bottom-left origin convention. If not, a coordinate transform is needed at the socket boundary.
5. **Fallback when all 6 Dubins types exit bounds** — `_dubins_bounded` returns the shortest path anyway. For competition arena layouts, some obstacle placements could still produce out-of-bounds paths. May need a re-planning strategy (e.g., intermediate waypoints) if this occurs in testing.

---

## 10. Robot reference-point convention

`state.x, state.y` is the **apex of the direction triangle** — i.e., the front-centre of the robot body. The robot body (30×30 cm) is drawn 15 cm behind the apex along `−θ`.

Path lines and Dubins kinematics both operate on this apex point. Approach poses sit the apex `APPROACH_CM = 20 cm` from the obstacle face (camera-to-face distance).

## 11. Integration contract (for later — not blocking B.1–B.3)

PC ↔ Wi-Fi TCP ↔ RPi ↔ USB-UART ↔ STM32. Motion commands are lock-step: send one primitive, wait for DONE confirmation, send next. The simulator's `get_commands()` already outputs a flat command list compatible with this model.
