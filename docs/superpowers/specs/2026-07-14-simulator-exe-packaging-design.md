# MDP Simulator — Packaging Design Spec: Standalone .exe + config.yaml
**Date:** 2026-07-14
**Scope:** Package `simulator/main.py` and `send_to_car.py` as standalone Windows executables that teammates can run without installing Python, with runtime-tunable settings moved into a committed `config.yaml`.

---

## 1. Background & Motivation

Currently, running the simulator or sending commands to the car requires each teammate to have Python installed, install `requirements.txt`, and run `python -m simulator.main` / `python send_to_car.py` from the `Algorithm/` directory. Settings that change per-session or per-network (RPi IP address, simulator constants) are hardcoded in source (`comms.py`, `simulator/config.py`, `simulator/planner.py`), so changing them means editing and recommitting Python files (e.g. commit `3e10b29` existed solely to update `RPI_HOST`).

Goal: teammates run `git pull` and get working `.exe`s directly, and can tweak settings by editing a plain YAML file — no Python install, no code edits.

---

## 2. config.yaml

**Location:** `Algorithm/config.yaml` — single source of truth, committed to git, human-edited, with inline comments.

**Schema:**
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

`arena_cm` (currently `GRID_SIZE * CELL_CM`) stays a derived value computed in code, not duplicated in YAML.

**Loader — `Algorithm/app_config.py`:**
- Resolves `config.yaml`'s path relative to the running executable: `Path(sys.executable).parent` when frozen (`getattr(sys, 'frozen', False)`), else `Path(__file__).parent` when run from source.
- Parses YAML with `pyyaml`. If the file is missing or a key is absent, falls back to the current hardcoded value for that key (logs a warning to console) — so a stale or partial `config.yaml` never crashes the app.
- Exposes the values as module-level constants (`RPI_HOST`, `RPI_PORT`, `CELL_CM`, etc.) so existing call sites change their import source, not their usage.

**Files updated to source values from `app_config`:**
| File | Change |
|---|---|
| `Algorithm/simulator/config.py` | Constants become re-exports of `app_config` values instead of literals |
| `Algorithm/comms.py` | `RPI_HOST`, `RPI_PORT`, `_TIMEOUT_S` sourced from `app_config` |
| `Algorithm/simulator/planner.py` | `OBSTACLES` default list sourced from `app_config.DEFAULT_OBSTACLES` |

**Precedence:** `config.yaml` supplies defaults; CLI args (`--random N`, explicit `x,y,Face` lists, `--dry-run`) override for that run, exactly as today. No changes to `_parse_args` logic in `main.py` / `send_to_car.py` beyond sourcing their fallback obstacle list from config instead of the `planner.OBSTACLES` literal.

---

## 3. Packaging

**Tool:** PyInstaller, `--onefile` mode, two separate executables built from `Algorithm/`:
- `simulator.exe` ← `simulator/main.py`
- `send_to_car.exe` ← `send_to_car.py`

**Console:** both keep the console window (no `--windowed`) — this is an actively-developed project and console tracebacks/print output are more useful right now than a clean window. `simulator.exe` still opens its pygame window as normal; the console stays open behind/alongside it.

**Output layout — `Algorithm/dist/`** (committed to git):
```
Algorithm/dist/
  simulator.exe
  send_to_car.exe
  config.yaml       # copy of Algorithm/config.yaml, placed here at build time
```
Both exes resolve `config.yaml` from their own directory at runtime (see §2 loader), so this directory is self-contained — a teammate can copy just `Algorithm/dist/` anywhere and it still works.

**Build script — `Algorithm/build_exe.ps1`:**
- Runs `pyinstaller --onefile --distpath dist --workpath build --specpath build simulator/main.py --name simulator`, likewise for `send_to_car.py`.
- Copies `config.yaml` into `dist/` after both builds succeed.
- Whoever changes simulator/config code re-runs this script and commits the refreshed `dist/` folder as part of their change.

**Dependencies:**
- `Algorithm/requirements.txt` gains `pyyaml` (runtime dependency, bundled into the exe).
- New `Algorithm/requirements-build.txt` containing `pyinstaller` — only needed by whoever builds the `.exe`s, not by teammates who just run them or by contributors who only run from source.

**`.gitignore` changes (`Algorithm/.gitignore`):**
- Add `build/` (PyInstaller's intermediate scratch directory — regenerable, not `dist/`) and `*.spec`.
- `dist/` is intentionally **not** ignored — its contents are committed.

**Build environment note:** local dev Python is 3.14.4, which may be ahead of PyInstaller's current supported-version matrix. Implementation should verify `pyinstaller --version` runs cleanly and produces a working exe under the available Python; if 3.14 isn't supported, `build_exe.ps1` should note (in a comment) that it needs to be run under 3.11/3.12 via a separate venv. This only affects the machine doing the build/commit — teammates running the committed `.exe` are unaffected either way.

---

## 4. Workflow After This Change

**Teammates (consumers):**
1. `git pull`
2. Double-click `Algorithm/dist/simulator.exe` (or run `send_to_car.exe` from a terminal for its CLI args/output)
3. To change the RPi IP or simulator constants: edit `Algorithm/dist/config.yaml`, save, re-run the exe — no rebuild.

**Whoever changes Algorithm source code:**
1. Edit code / `config.yaml` as normal.
2. Run `Algorithm/build_exe.ps1` to refresh `Algorithm/dist/`.
3. Commit source changes + refreshed `dist/` together.

---

## 5. Files Changed

| File | Change |
|---|---|
| `Algorithm/config.yaml` | New — committed config source of truth |
| `Algorithm/app_config.py` | New — YAML loader with fallback-to-hardcoded-defaults |
| `Algorithm/simulator/config.py` | Constants re-exported from `app_config` |
| `Algorithm/comms.py` | `RPI_HOST`/`RPI_PORT`/`_TIMEOUT_S` sourced from `app_config` |
| `Algorithm/simulator/planner.py` | Default `OBSTACLES` sourced from `app_config.DEFAULT_OBSTACLES` |
| `Algorithm/build_exe.ps1` | New — PyInstaller build script for both exes |
| `Algorithm/requirements.txt` | Add `pyyaml` |
| `Algorithm/requirements-build.txt` | New — `pyinstaller` |
| `Algorithm/.gitignore` | Add `build/`, `*.spec`; do not ignore `dist/` |
| `Algorithm/dist/simulator.exe`, `send_to_car.exe`, `config.yaml` | New — committed build output |

---

## 6. Testing

- `pytest` suite (`simulator/tests/test_logic.py`) must still pass unchanged — `app_config` fallback defaults match current hardcoded values exactly, so logic/tests are unaffected.
- Manual verification (run from source): `python -m simulator.main` and `python send_to_car.py --dry-run` still work with `config.yaml` present, and still work (using fallback defaults) if `config.yaml` is temporarily renamed away — confirms the fallback path doesn't crash.
- Manual verification (packaged): run `Algorithm/dist/simulator.exe` and `Algorithm/dist/send_to_car.exe --dry-run` on the build machine; edit `Algorithm/dist/config.yaml` (e.g. change `default_obstacles`) and confirm a re-run picks up the change without rebuilding.
- Confirm `git status` after building shows only intended `dist/` changes (no `build/` or `*.spec` files leaking into the commit).
