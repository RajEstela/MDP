# Bullseye Face-Change Maneuver — Design Spec
**Date:** 2026-07-14
**Scope:** `Image Recognition/pc_infer_server.py` — replace the current in-place rotation "scan" with a maneuver that physically repositions the robot to an adjacent face of the obstacle when the current face shows a Bullseye marker.

---

## 1. Background & Motivation

`pc_infer_server.py --serve` runs YOLO inference on the RPi's live camera stream. When it detects `ID 100 (Bullseye)` — a marker meaning "no target image on this face" — it's supposed to reposition the robot to check a different face of the same obstacle for the real target image.

The current implementation (`SCAN_DIRECTIONS = ("RR", "RL", "RL")`, `DEFAULT_SCAN_DEGREES = 30`) only ever sends small in-place rotation commands (`RRxxx`/`RLxxx`). Debugging with the team confirmed:

1. **Root cause of "car never moves" (resolved):** `send_car_command()` connects to the RPi's separate movement server (`RaspberryPi/Robot/server.py`, port 5000), which was never started alongside `pi_stream_frames.py`. Every attempt failed with `[Errno 61] Connection refused`. Starting that server fixed connectivity — rotation commands now succeed on the real robot.
2. **Remaining gap (this spec):** rotating in place from a fixed standoff distance doesn't bring an *adjacent face* of a square obstacle into view — the robot needs to physically walk around the obstacle's corner. The car has working `FW`/`BW` (drive straight, cm) and `RL`/`RR` (rotate in place, degrees) primitives (`RaspberryPi/Robot/nanocar.py`), but the current scan logic never issues `FW`/`BW`.

This spec replaces the rotate-only "scan" with a proper corner-walking maneuver.

---

## 2. Physical Geometry

Reuses the real-world dimensions already established in `Algorithm/simulator/config.py`, so both parts of the project stay consistent:

- Obstacle: 10cm cube (`CELL_CM = 10` → half-width `h = 5`)
- Standoff distance from the face being inspected: 20cm (`APPROACH_CM = 20`)

**The maneuver** (all steps relative to the robot's current heading — no compass/absolute positioning needed):

```
RR090  →  FW025  →  RL090  →  FW025  →  RL090
```

| Step | Command | Purpose |
|---|---|---|
| 1 | `RR090` | Turn away from the current face, toward the corner |
| 2 | `FW025` | Drive past the corner (`APPROACH_CM + h` = 25cm) |
| 3 | `RL090` | Turn back toward the obstacle |
| 4 | `FW025` | Close in to the new face's standoff line (25cm) |
| 5 | `RL090` | Square up, now facing the new face directly |

Because every step is relative and the obstacle is a perfect square, this exact 5-command sequence walks the robot from **any** starting face to its counter-clockwise-adjacent neighbor, regardless of the obstacle's absolute orientation in the world. Going clockwise would just mirror every `RL`↔`RR`, but this spec only implements counter-clockwise (per team decision).

Leg distance (25cm) is computed from named constants (`OBSTACLE_HALF_WIDTH_CM = 5`, `APPROACH_CM = 20`), not a bare literal, with a comment noting they should match `Algorithm/simulator/config.py`'s `CELL_CM`/`APPROACH_CM`.

**Rejected alternative:** a smooth continuous-arc maneuver via the raw `DRIVE,x,z` command instead of discrete turn/drive/turn steps. Rejected because it's open-loop/timing-based with no odometry feedback and would need extensive blind tuning; the turn/drive/turn approach reuses `FW`/`BW`/`RL`/`RR`, which are already calibrated and confirmed working on the real car.

---

## 3. State Machine Changes

**Current behavior:** `scan_step` (0, 1, 2) indexes into `SCAN_DIRECTIONS`, incrementing after each successful single-rotation command; resets to 0 when a non-Bullseye object is detected.

**New behavior:**

- `scan_step` now counts **faces visited** via a full maneuver (0 to `MAX_FACE_ATTEMPTS = 3`, since a square has 3 other faces besides the one currently showing Bullseye).
- On a Bullseye detection (and not already in a given-up state — see below), run the full 5-command maneuver. Each command is sent via `send_car_command()` and blocks until the car confirms completion before the next command is sent, so the 5 steps execute strictly in order.
- **On success** (all 5 commands succeed): increment `scan_step`. If `scan_step` reaches `MAX_FACE_ATTEMPTS`, enter the give-up state (see below) instead of attempting a 4th maneuver.
- **On failure** (any of the 5 commands errors): abort the remaining steps of *this* maneuver, print the failure, and do **not** advance `scan_step` — the same face-change is retried from scratch on the next Bullseye frame. (A partially-completed maneuver leaves the robot's physical position between the old and new face; retrying from scratch is a known simplification — see Open Question below.)
- **Give-up state:** once `scan_step` reaches `MAX_FACE_ATTEMPTS` with every checked face still showing Bullseye, print `"Checked all 4 faces, no target image found — giving up on this obstacle."` and stop attempting further maneuvers. Further Bullseye detections are logged (the existing "Detected: ..." line) but trigger no movement. The give-up state clears — same as the normal `scan_step` reset — the moment a non-Bullseye object is detected, matching today's `elif scan_step: ... scan_step = 0` reset path.
- **Frame backlog drain:** `pi_stream_frames.py` streams frames continuously with no throttling; a full maneuver takes several real seconds of blocking movement calls, during which the RPi keeps pushing frames into the socket buffer. After each maneuver attempt (success, failure, or entering the give-up state), drain any already-buffered frames from the socket (non-blocking read until nothing is immediately available) before resuming the normal blocking per-frame loop. This prevents the code from reacting to a backlog of stale pre-maneuver frames immediately after finishing a maneuver, which could otherwise trigger a second maneuver before ever evaluating a fresh frame of the new face.

---

## 4. CLI Changes

- Remove `--scan-degrees` / `DEFAULT_SCAN_DEGREES` — the 90° turn is fixed geometry (walking around a square corner), not a tunable sweep angle.
- Add `--face-step-cm` (default `25`, i.e. `APPROACH_CM + OBSTACLE_HALF_WIDTH_CM`) so the team can tune the drive-leg distance on-site without a code edit, if the real rig's obstacle size or standoff differs slightly from the assumed values. Mirrors the existing tunability of `--conf`.
- `--car-port` is unchanged.

---

## 5. Testing

No camera/car hardware is available in this environment, so most of this cannot be verified end-to-end here. Scope of testing:

**Automated (added as part of this change):** `Image Recognition/` currently has no test infrastructure. Add a minimal pytest file (`Image Recognition/test_face_maneuver.py`) with one pure-function unit test asserting the exact 5-command sequence and values a "build face-change commands" helper produces — the only part of this behavior that's testable without hardware.

**Manual (team runs on the real rig):**
1. Confirm `RaspberryPi/Robot/server.py` is running alongside `pi_stream_frames.py` before testing (this was the original root cause — worth a standing reminder).
2. Point the camera at an obstacle's Bullseye face; confirm the console prints the 5-step maneuver sequence (`RR090`, `FW025`, `RL090`, `FW025`, `RL090`) and the robot ends up facing a different, adjacent face.
3. Confirm a non-Bullseye detection after a successful face-change resets state correctly (existing "Non-bullseye object found; bullseye scan complete." message).
4. If feasible, test the give-up path with an obstacle that has Bullseye on all 4 faces (or simulate by manually blocking the target faces) and confirm the give-up message appears once, without further movement attempts.
5. Confirm no stale-frame misfire: after a maneuver completes, the next "Detected" line should reflect the robot's new, post-maneuver camera view, not a repeat of the old face.

---

## 6. File Sync Note

The team is actively iterating on `pc_infer_server.py` directly on a teammate's Mac (path seen in tracebacks: `/Users/weinam/Documents/SC2079_group/MDP/Image Recognition/pc_infer_server.py`), and the file has changed shape mid-session (traceback line numbers shifted between test runs). This spec is written at the logical/architectural level rather than against exact line numbers for that reason. Before implementation, confirm which copy (this repo checkout vs. the Mac's working copy) is authoritative, and sync afterward (commit + push from the Mac, or pull down) so the change isn't lost to drift.

---

## 7. Open Question (not blocking, flagged for awareness)

A maneuver that fails partway through (e.g. the 3rd of 5 commands errors) leaves the robot in an unknown intermediate position — retrying "from scratch" on the next Bullseye frame assumes the robot is still at the original face's standoff position, which may not be true if some physical movement already happened before the failure. This spec accepts that simplification for now (matches the current code's philosophy of not tracking robot pose independently), since a repositioning/recovery strategy would need real odometry or vision-based re-centering, which is out of scope here. Flagging in case the team wants to revisit after seeing how often mid-maneuver failures actually occur in practice.
