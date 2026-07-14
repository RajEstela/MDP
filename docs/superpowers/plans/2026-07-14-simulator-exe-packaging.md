# Simulator .exe Packaging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Package `Algorithm/simulator/main.py` and `Algorithm/send_to_car.py` into standalone Windows executables (`simulator.exe`, `send_to_car.exe`) committed to git, driven by a human-editable `config.yaml` that teammates can change without Python or a rebuild.

**Architecture:** A new `Algorithm/app_config.py` module loads `config.yaml` (YAML with fallback to built-in defaults for any missing file/key) and exposes the values as module-level constants. Existing modules (`simulator/config.py`, `comms.py`, `simulator/planner.py`) are re-pointed to read from `app_config` instead of hardcoded literals, with CLI args continuing to override per-run as they do today. PyInstaller then bundles both entry points as `--onefile` executables into `Algorithm/dist/`, alongside a copy of `config.yaml` that the exes resolve at runtime relative to their own location.

**Tech Stack:** Python 3.x, pygame-ce, pyyaml (new), PyInstaller (new, build-time only), pytest.

## Global Constraints

- PyInstaller `--onefile` mode, two executables: `simulator.exe` (from `simulator/main.py`) and `send_to_car.exe` (from `send_to_car.py`).
- Both executables keep the console window visible — do not pass `--windowed`/`--noconsole`.
- `config.yaml` lives at `Algorithm/config.yaml` (source of truth, committed) and a copy is placed at `Algorithm/dist/config.yaml` at build time; each exe resolves `config.yaml` from its own directory at runtime.
- Precedence: `config.yaml` supplies defaults; existing CLI args (`--random N`, explicit `x,y,Face` obstacle lists, `--dry-run`) override for that run — CLI parsing logic in `main.py`/`send_to_car.py` does not change.
- The loader must never crash on a missing file or missing keys — always fall back to built-in defaults that exactly match the current hardcoded values, with a console warning.
- `pyyaml` goes in `Algorithm/requirements.txt` (runtime dependency, gets bundled). `pyinstaller` goes in a new `Algorithm/requirements-build.txt` (build-time only, not needed by teammates running the committed exe).
- `Algorithm/.gitignore` ignores `build/` and `*.spec` (regenerable PyInstaller scratch output); `Algorithm/dist/` is NOT ignored — its contents are committed.
- All commands in this plan run from the `Algorithm/` directory unless stated otherwise.

---

### Task 1: Add dependencies and create config.yaml

**Files:**
- Modify: `Algorithm/requirements.txt`
- Create: `Algorithm/requirements-build.txt`
- Create: `Algorithm/config.yaml`

**Interfaces:**
- Produces: `Algorithm/config.yaml` on disk with keys `rpi.{host,port,timeout_s}`, `simulator.{cell_cm,grid_size,cell_px,arena_px,fps,turn_radius_cm,robot_w_cm,robot_h_cm,step_cm_per_frame,deg_per_frame,approach_cm,start_x_cm,start_y_cm,start_theta}`, and `default_obstacles` (list of `{x, y, face}`). Task 2 reads this schema.

- [ ] **Step 1: Add pyyaml to requirements.txt**

Edit `Algorithm/requirements.txt` to:

```
pygame-ce>=2.5.0
pytest>=8.0.0
pyyaml>=6.0
```

- [ ] **Step 2: Create requirements-build.txt**

Create `Algorithm/requirements-build.txt`:

```
pyinstaller>=6.0
```

- [ ] **Step 3: Create config.yaml**

Create `Algorithm/config.yaml`:

```yaml
# MDP simulator & car-link configuration.
# Edit values below and re-run the .exe — no rebuild needed.

rpi:
  host: 192.168.3.3   # RPi IP on the car's WiFi AP — changes per demo network
  port: 5000
  timeout_s: 60.0      # large rotations (e.g. 360 deg) can take ~24s

simulator:
  cell_cm: 10
  grid_size: 20
  cell_px: 40
  arena_px: 800
  fps: 60
  turn_radius_cm: 25.0
  robot_w_cm: 30
  robot_h_cm: 30
  step_cm_per_frame: 2.0
  deg_per_frame: 3.0
  approach_cm: 20
  start_x_cm: 20.0
  start_y_cm: 30.0
  start_theta: 90.0

default_obstacles:
  - {x: 50,  y: 100, face: N}
  - {x: 110, y: 100, face: E}
  - {x: 50,  y: 160, face: S}
  - {x: 110, y: 160, face: W}
  - {x: 170, y: 60,  face: N}
```

- [ ] **Step 4: Install dependencies**

Run: `pip install -r requirements.txt`
Expected: pyyaml installs successfully (pygame-ce, pytest already present).

- [ ] **Step 5: Commit**

```bash
git add requirements.txt requirements-build.txt config.yaml
git commit -m "chore: add pyyaml dependency and config.yaml schema"
```

---

### Task 2: Implement app_config.py loader (TDD)

**Files:**
- Create: `Algorithm/tests/__init__.py`
- Create: `Algorithm/tests/test_app_config.py`
- Create: `Algorithm/app_config.py`

**Interfaces:**
- Consumes: `Algorithm/config.yaml` (schema from Task 1).
- Produces: `app_config.DEFAULTS` (dict), `app_config.load_config(path: Path) -> dict`, `app_config.config_dir() -> Path`, and module-level constants `RPI_HOST`, `RPI_PORT`, `RPI_TIMEOUT_S`, `CELL_CM`, `GRID_SIZE`, `CELL_PX`, `ARENA_PX`, `FPS`, `TURN_RADIUS_CM`, `ROBOT_W_CM`, `ROBOT_H_CM`, `STEP_CM_PER_FRAME`, `DEG_PER_FRAME`, `APPROACH_CM`, `START_X_CM`, `START_Y_CM`, `START_THETA`, `DEFAULT_OBSTACLES` (`list[Obstacle]`). Tasks 3–5 import these.

- [ ] **Step 1: Write the failing tests**

Create `Algorithm/tests/__init__.py` (empty file).

Create `Algorithm/tests/test_app_config.py`:

```python
from app_config import DEFAULTS, _deep_merge, load_config


def test_deep_merge_overlays_nested_dict():
    defaults = {'rpi': {'host': 'a', 'port': 1}}
    overrides = {'rpi': {'host': 'b'}}
    merged = _deep_merge(defaults, overrides)
    assert merged == {'rpi': {'host': 'b', 'port': 1}}


def test_deep_merge_replaces_list_wholesale():
    defaults = {'default_obstacles': [{'x': 1}]}
    overrides = {'default_obstacles': [{'x': 2}, {'x': 3}]}
    merged = _deep_merge(defaults, overrides)
    assert merged == {'default_obstacles': [{'x': 2}, {'x': 3}]}


def test_load_config_missing_file_returns_defaults(tmp_path):
    result = load_config(tmp_path / 'does_not_exist.yaml')
    assert result == DEFAULTS


def test_load_config_partial_file_falls_back_for_missing_keys(tmp_path):
    config_file = tmp_path / 'config.yaml'
    config_file.write_text('rpi:\n  host: 10.0.0.5\n')
    result = load_config(config_file)
    assert result['rpi']['host'] == '10.0.0.5'
    assert result['rpi']['port'] == DEFAULTS['rpi']['port']
    assert result['simulator'] == DEFAULTS['simulator']


def test_load_config_invalid_yaml_falls_back_to_defaults(tmp_path):
    config_file = tmp_path / 'config.yaml'
    config_file.write_text('rpi: [unclosed\n')
    result = load_config(config_file)
    assert result == DEFAULTS


def test_load_config_overrides_default_obstacles(tmp_path):
    config_file = tmp_path / 'config.yaml'
    config_file.write_text('default_obstacles:\n  - {x: 1, y: 2, face: N}\n')
    result = load_config(config_file)
    assert result['default_obstacles'] == [{'x': 1, 'y': 2, 'face': 'N'}]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest tests/test_app_config.py -v`
Expected: FAIL/ERROR — `ModuleNotFoundError: No module named 'app_config'`

- [ ] **Step 3: Implement app_config.py**

Create `Algorithm/app_config.py`:

```python
"""Loads Algorithm/config.yaml at runtime, falling back to built-in defaults
for any missing file or key so the app never crashes on a stale/partial config.

Resolves config.yaml relative to the running executable when frozen
(PyInstaller --onefile), or relative to this file when run from source.
"""
import sys
from pathlib import Path

import yaml

from simulator.types import Obstacle

DEFAULTS: dict = {
    'rpi': {
        'host': '192.168.3.3',
        'port': 5000,
        'timeout_s': 60.0,
    },
    'simulator': {
        'cell_cm': 10,
        'grid_size': 20,
        'cell_px': 40,
        'arena_px': 800,
        'fps': 60,
        'turn_radius_cm': 25.0,
        'robot_w_cm': 30,
        'robot_h_cm': 30,
        'step_cm_per_frame': 2.0,
        'deg_per_frame': 3.0,
        'approach_cm': 20,
        'start_x_cm': 20.0,
        'start_y_cm': 30.0,
        'start_theta': 90.0,
    },
    'default_obstacles': [
        {'x': 50,  'y': 100, 'face': 'N'},
        {'x': 110, 'y': 100, 'face': 'E'},
        {'x': 50,  'y': 160, 'face': 'S'},
        {'x': 110, 'y': 160, 'face': 'W'},
        {'x': 170, 'y': 60,  'face': 'N'},
    ],
}


def config_dir() -> Path:
    """Directory to look for config.yaml in: the exe's own folder when frozen, else this file's folder."""
    if getattr(sys, 'frozen', False):
        return Path(sys.executable).parent
    return Path(__file__).parent


def _deep_merge(defaults: dict, overrides: dict) -> dict:
    """Merge `overrides` onto `defaults`, recursing into nested dicts.
    Non-dict values (including lists) are replaced wholesale when present in overrides."""
    merged = {}
    for key, default_val in defaults.items():
        override_val = overrides.get(key)
        if isinstance(default_val, dict) and isinstance(override_val, dict):
            merged[key] = _deep_merge(default_val, override_val)
        elif key in overrides:
            merged[key] = override_val
        else:
            merged[key] = default_val
    return merged


def load_config(path: Path) -> dict:
    """Load and merge config.yaml at `path` onto DEFAULTS.

    Missing file, unreadable file, or malformed YAML all fall back to
    DEFAULTS (with a console warning) rather than raising.
    """
    if not path.exists():
        print(f"[app_config] {path} not found — using built-in defaults")
        return DEFAULTS
    try:
        with open(path, 'r') as f:
            raw = yaml.safe_load(f) or {}
    except yaml.YAMLError as exc:
        print(f"[app_config] failed to parse {path}: {exc} — using built-in defaults")
        return DEFAULTS
    return _deep_merge(DEFAULTS, raw)


_cfg = load_config(config_dir() / 'config.yaml')

RPI_HOST = _cfg['rpi']['host']
RPI_PORT = _cfg['rpi']['port']
RPI_TIMEOUT_S = _cfg['rpi']['timeout_s']

CELL_CM = _cfg['simulator']['cell_cm']
GRID_SIZE = _cfg['simulator']['grid_size']
CELL_PX = _cfg['simulator']['cell_px']
ARENA_PX = _cfg['simulator']['arena_px']
FPS = _cfg['simulator']['fps']
TURN_RADIUS_CM = _cfg['simulator']['turn_radius_cm']
ROBOT_W_CM = _cfg['simulator']['robot_w_cm']
ROBOT_H_CM = _cfg['simulator']['robot_h_cm']
STEP_CM_PER_FRAME = _cfg['simulator']['step_cm_per_frame']
DEG_PER_FRAME = _cfg['simulator']['deg_per_frame']
APPROACH_CM = _cfg['simulator']['approach_cm']
START_X_CM = _cfg['simulator']['start_x_cm']
START_Y_CM = _cfg['simulator']['start_y_cm']
START_THETA = _cfg['simulator']['start_theta']

DEFAULT_OBSTACLES = [Obstacle(x=o['x'], y=o['y'], face=o['face']) for o in _cfg['default_obstacles']]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/test_app_config.py -v`
Expected: PASS — 6 passed

- [ ] **Step 5: Sanity check against the real config.yaml**

Run: `python -c "import app_config; print(app_config.RPI_HOST, app_config.CELL_CM, len(app_config.DEFAULT_OBSTACLES))"`
Expected: `192.168.3.3 10 5`

- [ ] **Step 6: Commit**

```bash
git add tests/__init__.py tests/test_app_config.py app_config.py
git commit -m "feat: add app_config.py YAML config loader with fallback defaults"
```

---

### Task 3: Wire simulator/config.py to app_config

**Files:**
- Modify: `Algorithm/simulator/config.py` (full file, currently 16 lines)

**Interfaces:**
- Consumes: `app_config.{CELL_CM, GRID_SIZE, CELL_PX, ARENA_PX, FPS, TURN_RADIUS_CM, ROBOT_W_CM, ROBOT_H_CM, STEP_CM_PER_FRAME, DEG_PER_FRAME, APPROACH_CM, START_X_CM, START_Y_CM, START_THETA}` (Task 2).
- Produces: Same constant names as before (`CELL_CM`, `GRID_SIZE`, ..., `ARENA_CM`, `START_THETA`) — unchanged for all existing importers (`simulator/main.py`, `simulator/planner.py`, `simulator/arena.py`, `simulator/robot.py`, `simulator/tests/test_logic.py`).

- [ ] **Step 1: Rewrite simulator/config.py**

Replace the full contents of `Algorithm/simulator/config.py`:

```python
from app_config import (
    APPROACH_CM,
    ARENA_PX,
    CELL_CM,
    CELL_PX,
    DEG_PER_FRAME,
    FPS,
    GRID_SIZE,
    ROBOT_H_CM,
    ROBOT_W_CM,
    START_THETA,
    START_X_CM,
    START_Y_CM,
    STEP_CM_PER_FRAME,
    TURN_RADIUS_CM,
)

ARENA_CM = GRID_SIZE * CELL_CM  # 200 — full arena side length in cm
```

- [ ] **Step 2: Run the full test suite**

Run: `python -m pytest simulator/tests/test_logic.py -v`
Expected: PASS — all tests pass (no change in count/behavior; `simulator/config.py`'s public names are unchanged).

- [ ] **Step 3: Commit**

```bash
git add simulator/config.py
git commit -m "refactor: source simulator/config.py constants from app_config"
```

---

### Task 4: Wire comms.py to app_config

**Files:**
- Modify: `Algorithm/comms.py:22-24`

**Interfaces:**
- Consumes: `app_config.{RPI_HOST, RPI_PORT, RPI_TIMEOUT_S}` (Task 2).
- Produces: Same names `RPI_HOST`, `RPI_PORT` as module-level constants in `comms.py` (unchanged for `CarConnection.__init__` default args and any other importers).

- [ ] **Step 1: Replace hardcoded constants**

In `Algorithm/comms.py`, replace lines 22-24:

```python
RPI_HOST = '192.168.3.3'
RPI_PORT = 5000
_TIMEOUT_S = 60.0  # large rotations (e.g. 360°) can take ~24 s
```

with:

```python
from app_config import RPI_HOST, RPI_PORT, RPI_TIMEOUT_S as _TIMEOUT_S
```

Move this import line up next to the existing imports at the top of the file (after `from simulator.types import Command`), and delete the old `RPI_HOST`/`RPI_PORT`/`_TIMEOUT_S` literal assignments.

- [ ] **Step 2: Sanity check**

Run: `python -c "import comms; print(comms.RPI_HOST, comms.RPI_PORT)"`
Expected: `192.168.3.3 5000`

- [ ] **Step 3: Commit**

```bash
git add comms.py
git commit -m "refactor: source comms.py RPi connection settings from app_config"
```

---

### Task 5: Wire planner.py default obstacles to app_config, verify end-to-end

**Files:**
- Modify: `Algorithm/simulator/planner.py:1-14`

**Interfaces:**
- Consumes: `app_config.DEFAULT_OBSTACLES` (Task 2).
- Produces: Same `OBSTACLES: list[Obstacle]` module-level name in `simulator/planner.py` (unchanged for `simulator/main.py`, `send_to_car.py`, `simulator/tests/test_logic.py`).

- [ ] **Step 1: Replace the hardcoded OBSTACLES list**

In `Algorithm/simulator/planner.py`, replace lines 1-14:

```python
import itertools
import math
import random as _random

from simulator.config import APPROACH_CM, ARENA_CM, CELL_CM, FPS, GRID_SIZE, START_THETA, START_X_CM, START_Y_CM
from simulator.types import Command, Obstacle, RobotState

OBSTACLES: list[Obstacle] = [
    Obstacle(x=50,  y=100, face='N'),
    Obstacle(x=110, y=100, face='E'),
    Obstacle(x=50,  y=160, face='S'),
    Obstacle(x=110, y=160, face='W'),
    Obstacle(x=170, y=60,  face='N'),
]
```

with:

```python
import itertools
import math
import random as _random

from app_config import DEFAULT_OBSTACLES
from simulator.config import APPROACH_CM, ARENA_CM, CELL_CM, FPS, GRID_SIZE, START_THETA, START_X_CM, START_Y_CM
from simulator.types import Command, Obstacle, RobotState

OBSTACLES: list[Obstacle] = DEFAULT_OBSTACLES
```

- [ ] **Step 2: Run the full test suite**

Run: `python -m pytest simulator/tests/test_logic.py tests/test_app_config.py -v`
Expected: PASS — all tests pass, including `test_get_top_n_routes_returns_5_obstacles` (or equivalent asserting `len(OBSTACLES) == 5`).

- [ ] **Step 3: Verify send_to_car.py picks up config.yaml obstacles**

Run: `python send_to_car.py --dry-run`
Expected: stdout lists 5 obstacles matching `config.yaml`'s `default_obstacles` (x=50/y=100/face=N, x=110/y=100/face=E, x=50/y=160/face=S, x=110/y=160/face=W, x=170/y=60/face=N), ends with `Dry run complete — no connection made.`

- [ ] **Step 4: Verify fallback when config.yaml is absent**

Run (PowerShell):
```powershell
Rename-Item config.yaml config.yaml.bak
python send_to_car.py --dry-run
Rename-Item config.yaml.bak config.yaml
```
Expected: identical obstacle list/output to Step 3 (fallback defaults match), plus a `[app_config] ... not found — using built-in defaults` warning line; `config.yaml` is restored to its original name afterward.

- [ ] **Step 5: Commit**

```bash
git add simulator/planner.py
git commit -m "refactor: source planner.py default obstacles from app_config"
```

---

### Task 6: Add build script and update .gitignore

**Files:**
- Modify: `Algorithm/.gitignore`
- Create: `Algorithm/build_exe.ps1`

**Interfaces:**
- Produces: `Algorithm/build_exe.ps1`, a PowerShell script that Task 7 runs to produce `Algorithm/dist/simulator.exe`, `Algorithm/dist/send_to_car.exe`, `Algorithm/dist/config.yaml`.

- [ ] **Step 1: Update .gitignore**

Edit `Algorithm/.gitignore` to add two lines (keep existing entries):

```
**/__pycache__/
**/*.pyc
**/*.pyo
.pytest_cache/
build/
*.spec
```

- [ ] **Step 2: Create build_exe.ps1**

Create `Algorithm/build_exe.ps1`:

```powershell
<#
Builds simulator.exe and send_to_car.exe with PyInstaller and copies
config.yaml alongside them in dist/. Run from the Algorithm/ directory:

    .\build_exe.ps1

Requires: pip install -r requirements-build.txt (and requirements.txt)

If PyInstaller reports it doesn't support the active Python version,
rebuild inside a Python 3.11/3.12 venv instead — this only affects the
machine doing the build, not teammates running the committed .exe.

--paths . puts Algorithm/ on PyInstaller's import search path, which
simulator/main.py needs since it does `from simulator.arena import ...`
(an absolute import) from *inside* the simulator package.
#>

$ErrorActionPreference = "Stop"

Set-Location $PSScriptRoot

pyinstaller --onefile --distpath dist --workpath build --specpath build --paths . --name simulator simulator/main.py

pyinstaller --onefile --distpath dist --workpath build --specpath build --paths . --name send_to_car send_to_car.py

Copy-Item -Path "config.yaml" -Destination "dist\config.yaml" -Force

Write-Host "Build complete: dist\simulator.exe, dist\send_to_car.exe, dist\config.yaml"
```

- [ ] **Step 3: Commit**

```bash
git add .gitignore build_exe.ps1
git commit -m "feat: add PyInstaller build script for simulator/send_to_car exes"
```

---

### Task 7: Build the executables and commit dist/

**Files:**
- Create (generated, then committed): `Algorithm/dist/simulator.exe`, `Algorithm/dist/send_to_car.exe`, `Algorithm/dist/config.yaml`

**Interfaces:**
- Consumes: `Algorithm/build_exe.ps1` (Task 6), `Algorithm/config.yaml` (Task 1), all wired modules from Tasks 2-5.
- Produces: the final deliverable — a self-contained `Algorithm/dist/` folder teammates run after `git pull`.

- [ ] **Step 1: Install build-time dependency**

Run: `pip install -r requirements-build.txt`
Expected: pyinstaller installs successfully. If it errors that the active Python version is unsupported, create/activate a Python 3.12 venv and retry (`py -3.12 -m venv .venv312; .venv312\Scripts\pip install -r requirements.txt -r requirements-build.txt`) before continuing.

- [ ] **Step 2: Run the build script**

Run: `.\build_exe.ps1`
Expected: no errors; final line `Build complete: dist\simulator.exe, dist\send_to_car.exe, dist\config.yaml`; `Algorithm\dist\` now contains those three files.

- [ ] **Step 3: Verify send_to_car.exe**

Run: `.\dist\send_to_car.exe --dry-run`
Expected: same output as Task 5 Step 3 (5 obstacles from `config.yaml`, ending in `Dry run complete — no connection made.`) — proves the frozen exe resolves `config.yaml` from its own directory correctly.

- [ ] **Step 4: Verify simulator.exe**

Run: `.\dist\simulator.exe`
Expected: a pygame window opens titled "MDP Simulator — 5 random obstacles", console shows no import errors. Press `Q` or `Esc` to close it.

- [ ] **Step 5: Verify a config.yaml edit takes effect without rebuilding**

Edit `Algorithm\dist\config.yaml`'s `default_obstacles` list down to a single entry, e.g.:

```yaml
default_obstacles:
  - {x: 60, y: 60, face: N}
```

Run: `.\dist\send_to_car.exe --dry-run`
Expected: output now shows exactly 1 obstacle at x=60/y=60/face=N.

Then restore `Algorithm\dist\config.yaml` to match `Algorithm\config.yaml` (re-copy or `git checkout -- dist/config.yaml` once Step 2's build output is staged, or simply re-run Step 2 to regenerate it) before committing.

- [ ] **Step 6: Commit**

```bash
git add dist/simulator.exe dist/send_to_car.exe dist/config.yaml
git commit -m "build: add packaged simulator.exe and send_to_car.exe"
```

- [ ] **Step 7: Confirm no stray build artifacts**

Run: `git status`
Expected: clean working tree — no untracked `build/`, `*.spec`, or other PyInstaller scratch files (they're excluded by the `.gitignore` update in Task 6).
