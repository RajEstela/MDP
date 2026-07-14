# Bullseye Face-Change Maneuver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `Image Recognition/pc_infer_server.py`'s in-place-rotation-only "bullseye scan" with a maneuver that physically walks the robot to an adjacent face of the obstacle (drive + rotate, not just rotate), so it can find a real target image after seeing a Bullseye marker.

**Architecture:** A new dependency-free module `face_maneuver.py` holds the pure geometry (the 5-command sequence and its constants), unit-tested without any hardware or ML dependencies. `pc_infer_server.py` gains a `drain_buffered_frames()` I/O helper and has its `run_serve()` state machine rewired to run the full maneuver per Bullseye detection, counting faces visited instead of rotation sub-steps, with a give-up path after 3 attempts.

**Tech Stack:** Python 3.9 (confirmed via teammate's traceback: `Library/.../Python3.framework/Versions/3.9`), stdlib only for the new module; `pc_infer_server.py` itself depends on `opencv-python` (`cv2`) and `ultralytics` (already a dependency, unchanged by this plan).

## Global Constraints

- Obstacle geometry: 10cm cube (`OBSTACLE_HALF_WIDTH_CM = 5`), 20cm standoff (`APPROACH_CM = 20`) — must match `Algorithm/simulator/config.py`'s `CELL_CM`/`APPROACH_CM` (documented via comment, not imported — different Python environments).
- The face-change maneuver is exactly: `RR090`, `FW<step>`, `RL090`, `FW<step>`, `RL090` — always counter-clockwise, always this 5-step shape. `<step>` defaults to `25` (`APPROACH_CM + OBSTACLE_HALF_WIDTH_CM`) and is CLI-tunable via `--face-step-cm`.
- `MAX_FACE_ATTEMPTS = 3` (a square obstacle has 3 other faces besides the current one).
- On a Bullseye detection: run the full 5-command maneuver (abort remaining steps and don't advance the face count if any command fails); on success, increment the face-visited count; at `MAX_FACE_ATTEMPTS`, print `"Checked all 4 faces, no target image found — giving up on this obstacle."` and stop attempting further maneuvers until a non-Bullseye detection resets state.
- After every maneuver attempt (success, failure, or give-up), drain any frames already buffered in the socket before resuming normal per-frame processing.
- `--scan-degrees` / `DEFAULT_SCAN_DEGREES` / `SCAN_DIRECTIONS` are removed entirely (superseded by the fixed maneuver + `--face-step-cm`).
- **This dev environment has neither `cv2` nor `ultralytics` installed** (confirmed: `ModuleNotFoundError` for both) — `pc_infer_server.py` cannot be executed or import-smoke-tested here. Verification for tasks touching it is limited to `python -m py_compile` (syntax only) plus careful manual review; real behavioral verification happens on the team's actual rig (steps included in each task, copied from the spec).
- **File sync:** implement against this repo checkout (`C:\School\MDP\Image Recognition\pc_infer_server.py`). The team has been actively editing a separate copy on a Mac (`/Users/weinam/Documents/SC2079_group/MDP/Image Recognition/pc_infer_server.py`) during live debugging — after this plan is implemented and committed here, the team must reconcile/sync it onto the Mac copy before the next physical test (pull down, or manually re-apply if the Mac has diverged further).

---

### Task 1: `face_maneuver.py` — pure geometry module (TDD)

**Files:**
- Create: `Image Recognition/face_maneuver.py`
- Create: `Image Recognition/test_face_maneuver.py`

**Interfaces:**
- Produces: `face_maneuver.OBSTACLE_HALF_WIDTH_CM` (`int`, `5`), `face_maneuver.APPROACH_CM` (`int`, `20`), `face_maneuver.DEFAULT_FACE_STEP_CM` (`int`, `25`), `face_maneuver.MAX_FACE_ATTEMPTS` (`int`, `3`), `face_maneuver.build_face_change_commands(face_step_cm: int) -> list[str]`. Task 3 imports all five names.

- [ ] **Step 1: Write the failing tests**

Create `Image Recognition/test_face_maneuver.py`:

```python
from face_maneuver import build_face_change_commands


def test_build_face_change_commands_default_step():
    assert build_face_change_commands(25) == ["RR090", "FW025", "RL090", "FW025", "RL090"]


def test_build_face_change_commands_custom_step():
    assert build_face_change_commands(30) == ["RR090", "FW030", "RL090", "FW030", "RL090"]


def test_build_face_change_commands_single_digit_step():
    assert build_face_change_commands(5) == ["RR090", "FW005", "RL090", "FW005", "RL090"]
```

- [ ] **Step 2: Run the tests to verify they fail**

Run (from `Image Recognition/`): `python -m pytest test_face_maneuver.py -v`
Expected: FAIL / ERROR — `ModuleNotFoundError: No module named 'face_maneuver'`

- [ ] **Step 3: Implement face_maneuver.py**

Create `Image Recognition/face_maneuver.py`:

```python
"""Geometry for repositioning the robot to an adjacent obstacle face.

Constants mirror Algorithm/simulator/config.py's CELL_CM / APPROACH_CM so
both parts of the project agree on the real rig's obstacle size and
standoff distance. Not imported directly (separate Python environments -
this runs on the Mac alongside pc_infer_server.py) - keep the two files'
values in sync by hand if the rig's dimensions change.
"""

OBSTACLE_HALF_WIDTH_CM = 5   # half of Algorithm/simulator/config.py's CELL_CM (10cm obstacle)
APPROACH_CM = 20             # matches Algorithm/simulator/config.py's APPROACH_CM
DEFAULT_FACE_STEP_CM = APPROACH_CM + OBSTACLE_HALF_WIDTH_CM  # 25
MAX_FACE_ATTEMPTS = 3        # a square obstacle has 3 other faces besides the current one


def build_face_change_commands(face_step_cm: int) -> list[str]:
    """Return the 5-command sequence that walks the robot counter-clockwise
    around a square obstacle's corner to the next face: turn away from the
    corner, drive past it, turn back toward the obstacle, close the
    remaining distance, then square up to face the new face directly.

    Every step is relative to the robot's current heading, so this same
    sequence works from any starting face on a square obstacle.
    """
    step = f"FW{face_step_cm:03d}"
    return ["RR090", step, "RL090", step, "RL090"]
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python -m pytest test_face_maneuver.py -v`
Expected: PASS — 3 passed

- [ ] **Step 5: Commit**

```bash
git add "Image Recognition/face_maneuver.py" "Image Recognition/test_face_maneuver.py"
git commit -m "feat: add face-change maneuver geometry module"
```

---

### Task 2: `drain_buffered_frames()` helper in pc_infer_server.py

**Files:**
- Modify: `Image Recognition/pc_infer_server.py` (add a new function; no other changes in this task)

**Interfaces:**
- Consumes: nothing from Task 1.
- Produces: `drain_buffered_frames(sock: socket.socket, reader) -> int`. Task 3 calls this after each maneuver attempt.

Note: this function is added but **not yet called** by anything in this task — `run_serve()` still uses the old scan logic until Task 3. This is intentional (Task 3 wires it in); it's not dead code left behind.

- [ ] **Step 1: Add the function**

In `Image Recognition/pc_infer_server.py`, insert this new function immediately after `send_car_command` (currently ends around line 48, right before `def run_standalone`):

```python
def drain_buffered_frames(sock: socket.socket, reader) -> int:
    """Discard any frames already buffered locally (non-blocking), so the
    next frame processed after a maneuver reflects the robot's current
    position rather than a stale frame captured before it moved.

    Returns the number of frames discarded.
    """
    drained = 0
    sock.setblocking(False)
    try:
        while True:
            try:
                line = reader.readline()
            except (BlockingIOError, OSError):
                break
            if not line:
                break
            drained += 1
    finally:
        sock.setblocking(True)
    return drained
```

- [ ] **Step 2: Syntax-check (full runtime test not possible in this environment — see Global Constraints)**

Run (from `Image Recognition/`): `python -m py_compile pc_infer_server.py`
Expected: no output, exit code 0 (silent success)

- [ ] **Step 3: Commit**

```bash
git add "Image Recognition/pc_infer_server.py"
git commit -m "feat: add drain_buffered_frames helper to pc_infer_server.py"
```

---

### Task 3: Rewire run_serve()/main() to use the face-change maneuver

**Files:**
- Modify: `Image Recognition/pc_infer_server.py` (imports, constants block, `run_serve()`, `main()`)

**Interfaces:**
- Consumes: `face_maneuver.{OBSTACLE_HALF_WIDTH_CM, APPROACH_CM, DEFAULT_FACE_STEP_CM, MAX_FACE_ATTEMPTS, build_face_change_commands}` (Task 1), `drain_buffered_frames(sock, reader) -> int` (Task 2, already in this file).
- Produces: `run_serve(port: int, conf: float, car_port: int, face_step_cm: int)` (signature change: 4th param renamed from `scan_degrees` to `face_step_cm`), CLI flag `--face-step-cm` replacing `--scan-degrees`.

- [ ] **Step 1: Update imports and remove the old scan constants**

Replace lines 1-21 of `Image Recognition/pc_infer_server.py` (the imports block through `SCAN_DIRECTIONS = ("RR", "RL", "RL")`):

```python
import argparse
import base64
import json
import socket
import sys

import cv2
import numpy as np
from ultralytics import YOLO

MODEL_PATH = "best.pt"

# Confirmed lab image-ID table: 1-9, 10='zero', 11='V'...15='Z', bullseye=100.
# The model's training classes are already named with these exact IDs as
# strings (see model.names), but YOLO orders class *indices* alphabetically,
# so index != ID. Always resolve through model.names, never compare raw
# class_id to an ID directly.
BULLSEYE_ID = 100
CAR_PORT = 5000
DEFAULT_SCAN_DEGREES = 30
SCAN_DIRECTIONS = ("RR", "RL", "RL")
```

with:

```python
import argparse
import base64
import json
import socket
import sys

import cv2
import numpy as np
from ultralytics import YOLO

from face_maneuver import (
    APPROACH_CM,
    DEFAULT_FACE_STEP_CM,
    MAX_FACE_ATTEMPTS,
    OBSTACLE_HALF_WIDTH_CM,
    build_face_change_commands,
)

MODEL_PATH = "best.pt"

# Confirmed lab image-ID table: 1-9, 10='zero', 11='V'...15='Z', bullseye=100.
# The model's training classes are already named with these exact IDs as
# strings (see model.names), but YOLO orders class *indices* alphabetically,
# so index != ID. Always resolve through model.names, never compare raw
# class_id to an ID directly.
BULLSEYE_ID = 100
CAR_PORT = 5000
```

- [ ] **Step 2: Rewrite run_serve()'s signature and scan state**

Find this block near the top of `run_serve` (currently around line 76-88):

```python
def run_serve(port: int, conf: float, car_port: int, scan_degrees: int):
    model = YOLO(MODEL_PATH)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    print(f"Waiting for the RPi to connect on port {port}...")

    conn, addr = server.accept()
    print(f"RPi connected from {addr}")
    reader = conn.makefile("rb")
    scan_step = 0
```

Replace with:

```python
def run_serve(port: int, conf: float, car_port: int, face_step_cm: int):
    model = YOLO(MODEL_PATH)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    print(f"Waiting for the RPi to connect on port {port}...")

    conn, addr = server.accept()
    print(f"RPi connected from {addr}")
    reader = conn.makefile("rb")
    faces_checked = 0
    gave_up = False
```

- [ ] **Step 3: Replace the Bullseye-handling branch**

Find this block (currently around lines 142-154, inside the `if target_id is not None:` section, after the `conn.sendall(reply)` line):

```python
                if target_id == BULLSEYE_ID:
                    direction = SCAN_DIRECTIONS[scan_step]
                    command = f"{direction}{scan_degrees:03d}"
                    print(f"Bullseye scan: sending {command} to the car...")
                    try:
                        send_car_command(addr[0], car_port, command)
                    except (ConnectionError, OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
                        print(f"Bullseye scan movement failed: {exc}")
                    else:
                        scan_step = (scan_step + 1) % len(SCAN_DIRECTIONS)
                elif scan_step:
                    print("Non-bullseye object found; bullseye scan complete.")
                    scan_step = 0
```

Replace with:

```python
                if target_id == BULLSEYE_ID:
                    if gave_up:
                        pass  # already exhausted all faces for this obstacle - keep
                              # logging detections, but stop attempting to move
                    else:
                        print(f"Bullseye scan: repositioning to next face "
                              f"(attempt {faces_checked + 1}/{MAX_FACE_ATTEMPTS})...")
                        maneuver_ok = True
                        for command in build_face_change_commands(face_step_cm):
                            print(f"Bullseye scan: sending {command} to the car...")
                            try:
                                send_car_command(addr[0], car_port, command)
                            except (ConnectionError, OSError, RuntimeError, ValueError,
                                    json.JSONDecodeError) as exc:
                                print(f"Bullseye scan movement failed: {exc}")
                                maneuver_ok = False
                                break

                        drained = drain_buffered_frames(conn, reader)
                        if drained:
                            print(f"Bullseye scan: discarded {drained} stale frame(s) "
                                  f"buffered during the maneuver.")

                        if maneuver_ok:
                            faces_checked += 1
                            if faces_checked >= MAX_FACE_ATTEMPTS:
                                gave_up = True
                                print("Checked all 4 faces, no target image found "
                                      "— giving up on this obstacle.")
                elif faces_checked or gave_up:
                    print("Non-bullseye object found; bullseye scan complete.")
                    faces_checked = 0
                    gave_up = False
```

- [ ] **Step 4: Update main()'s CLI arguments**

Find this block in `main()` (currently around lines 177-189):

```python
    parser.add_argument(
        "--scan-degrees", type=int, default=DEFAULT_SCAN_DEGREES,
        help="Degrees to rotate for each right/left bullseye scan step."
    )
    parser.add_argument(
        "--conf", type=float, default=0.2,
        help="Confidence threshold for detections (0-1). Raise this if random objects (e.g. legs, chairs) "
             "get flagged as false positives; lower it if real targets aren't being picked up."
    )
    args = parser.parse_args()

    if not 1 <= args.scan_degrees <= 359:
        parser.error("--scan-degrees must be between 1 and 359")

    if args.serve:
        run_serve(args.port, args.conf, args.car_port, args.scan_degrees)
    else:
        run_standalone(args.conf)
```

Replace with:

```python
    parser.add_argument(
        "--face-step-cm", type=int, default=DEFAULT_FACE_STEP_CM,
        help="Distance in cm to drive during each leg of the face-change maneuver when a Bullseye "
             f"is detected. Defaults to {DEFAULT_FACE_STEP_CM} (matches a "
             f"{OBSTACLE_HALF_WIDTH_CM * 2}cm obstacle with a {APPROACH_CM}cm standoff). Tune this "
             "if your rig's obstacle size or standoff distance differs."
    )
    parser.add_argument(
        "--conf", type=float, default=0.2,
        help="Confidence threshold for detections (0-1). Raise this if random objects (e.g. legs, chairs) "
             "get flagged as false positives; lower it if real targets aren't being picked up."
    )
    args = parser.parse_args()

    if args.face_step_cm <= 0:
        parser.error("--face-step-cm must be positive")

    if args.serve:
        run_serve(args.port, args.conf, args.car_port, args.face_step_cm)
    else:
        run_standalone(args.conf)
```

- [ ] **Step 5: Syntax-check (full runtime test not possible in this environment — see Global Constraints)**

Run (from `Image Recognition/`): `python -m py_compile pc_infer_server.py`
Expected: no output, exit code 0 (silent success)

Also re-run Task 1's tests to confirm they're unaffected:
Run: `python -m pytest test_face_maneuver.py -v`
Expected: PASS — 3 passed

- [ ] **Step 6: Commit**

```bash
git add "Image Recognition/pc_infer_server.py"
git commit -m "feat: replace rotation-only bullseye scan with face-change maneuver"
```

- [ ] **Step 7: Update note_v2.txt with the manual verification steps**

Append a new section to the end of `Image Recognition/note_v2.txt`:

```
============================================================
FACE-CHANGE MANEUVER (bullseye handling) - manual test checklist
============================================================
Before testing: confirm RaspberryPi/Robot/server.py is running on the RPi
IN ADDITION TO pi_stream_frames.py (this was the original "car never
moves" bug - the movement server is a separate process that must also be
started).

1. Point the camera at an obstacle's Bullseye face. Confirm the console
   prints the 5-step maneuver sequence in order:
       RR090, FW025, RL090, FW025, RL090
   (FW025 assumes the default --face-step-cm=25; adjust if you passed a
   different value) and the robot ends up facing a different, adjacent
   face afterward.
2. Confirm a non-Bullseye detection after a successful face-change resets
   state correctly - console should print "Non-bullseye object found;
   bullseye scan complete."
3. If feasible, test the give-up path (an obstacle with Bullseye on all 4
   faces, or manually block the target faces) and confirm the give-up
   message ("Checked all 4 faces, no target image found - giving up on
   this obstacle.") appears exactly once, with no further movement
   attempts until a non-Bullseye detection resets state.
4. Confirm no stale-frame misfire: after a maneuver completes, the next
   "Detected" line should reflect the robot's new, post-maneuver camera
   view, not a repeat of the old face's Bullseye.
5. --face-step-cm is tunable without a code edit if the real obstacle/
   standoff distance differs from the assumed 10cm obstacle / 20cm
   standoff, e.g.:
       python3 pc_infer_server.py --serve --port 5006 --face-step-cm 30
```

- [ ] **Step 8: Commit the note update**

```bash
git add "Image Recognition/note_v2.txt"
git commit -m "docs: add manual test checklist for the face-change maneuver"
```
