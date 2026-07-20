# Live Arena Simulator — Design

**Date:** 2026-07-20
**Status:** Approved for planning

## 1. Problem

For the full competition run, the arena configuration (grid, robot start pose,
obstacles) is created on the Android tablet, sent to the car's Raspberry Pi,
and needs to reach the PC-side simulator, which must:

1. Receive that arena configuration over the network (same JSON format the
   RPi already relays, see §3).
2. Run the visual pygame simulation against the real obstacles/start pose,
   computing and animating the optimal route.
3. Once the simulated route is decided, transmit each movement command to
   the real car (via the RPi) and reflect the car's real progress on screen,
   so the physical car follows the same route that was just simulated.

Today this is split across two disconnected scripts:

- `live_arena.py` — headless. Listens for arena JSON on TCP port 5001,
  computes a route, optionally (`--execute`) sends it to the car on port
  5000. No visuals. Silently ignores the `robot` field in the arena JSON and
  always starts from the fixed config pose.
- `simulator/main.py` — visual pygame animation. Takes obstacles from CLI
  args or generates random ones. No networking, no car execution.

This design merges the two into one live-mode entry point on the visual
simulator, backed by a shared arena-parsing module so both entry points stay
consistent.

## 2. Architecture

```
Android app → RPi (TCP :5001, arena JSON) → PC: arena_feed listener thread
                                                          │ (queue)
                                                pygame main thread
                                            (compute route → animate)
                                                          │ (route animation done)
                                                car_executor thread
                                                          │
                                          RPi (TCP :5000, comms.CarConnection)
                                                          │
                                                STM32 → physical car moves

algorithm_status updates (planning / route_ready / running / completed /
error) flow back over the port-5001 socket throughout, so the Android app
sees progress the same way it does today via live_arena.py.
```

Only two background threads run at any time, both daemon threads (they die
with the process):

- **arena listener** — persistent for the life of the process; owns the
  port-5001 socket, reconnects on drop.
- **car executor** — spawned only after a route's animation finishes with
  `--execute` set; owns a fresh port-5000 `CarConnection` for that run.

Route computation (Hamiltonian ordering + leg planning) stays synchronous on
the pygame main thread exactly as it is today — it's sub-second for 5
obstacles and already shows a "Computing…" loading frame first.

## 3. Arena JSON format (input, unchanged from what the RPi sends)

```json
{
  "version": 1, "type": "arena", "revision": 1,
  "grid": {"columns": 20, "rows": 20, "cellCm": 10, "origin": "bottom-left"},
  "robot": {"x": 1, "y": 1, "direction": "N"},
  "obstacles": [
    {"id": "B1", "x": 5, "y": 16, "direction": "S", "targetId": null}
  ]
}
```

`grid` must match the simulator's compiled-in `GRID_SIZE`/`CELL_CM`
(20×20, 10cm cells) or the snapshot is rejected — same validation
`live_arena.py` already does for obstacles.

`obstacle.id` / `targetId` are logged for visibility (console, same as
today) but are not consumed by path planning — that's the image-recognition
module's concern and out of scope here.

## 4. New shared module: `Algorithm/arena_feed.py`

Extracted from `live_arena.py` so both the headless script and the visual
simulator parse/validate identically instead of drifting apart.

```python
ARENA_PORT = 5001
RECONNECT_DELAY_S = 2.0

def arena_to_obstacles(snapshot: dict) -> list[Obstacle]: ...   # moved, unchanged
def arena_to_robot_start(snapshot: dict) -> RobotState: ...     # new, see §5
def send_status(sock, revision, state, message, **details) -> None: ...  # moved, unchanged
def listen(host, on_snapshot: Callable[[dict, socket.socket], None],
           once: bool = False) -> None: ...  # moved connect/reconnect/dedup loop
```

`listen()` keeps the existing connect → read-lines → JSON-decode → dedup-by-
signature → invoke-callback loop, but the callback is now injectable instead
of hardcoded, so:

- `live_arena.py`'s callback does today's compute + optional `--execute`
  send, headless.
- `simulator/main.py`'s callback (running on the listener thread) just
  validates the snapshot and pushes `(obstacles, robot_start, revision,
  sock)` onto a `queue.Queue` for the pygame main thread to pick up.

Validation errors in either case still get logged, still get an
`algorithm_status: error` reply sent back on the same socket, and don't
crash the listener loop.

## 5. Robot start pose conversion

`robot.x`/`robot.y` are 0-indexed grid cells (matching the RPi's own
`normalize_arena`, which validates them against `range(columns)`/
`range(rows)`). The robot's 30×30cm body is a 3×3 cell footprint **centered**
on that cell:

```python
center_x_cm = x * CELL_CM + CELL_CM / 2   # e.g. x=1 → 15.0
center_y_cm = y * CELL_CM + CELL_CM / 2   # e.g. y=1 → 15.0
```

The simulator tracks the front-center "apex" point, not the body center, so
the apex is the body center offset by half the robot width (15cm) along the
facing direction. Direction → heading (matches the existing θ convention:
0°=East, 90°=North, CCW positive):

| direction | theta | apex offset from center |
|---|---|---|
| N | 90°  | (0, +15) |
| E | 0°   | (+15, 0) |
| S | 270° | (0, -15) |
| W | 180° | (-15, 0) |

E.g. `{"x":1,"y":1,"direction":"N"}` → center (15,15) → apex (15, 30, 90°).

## 6. `planner.py` changes

`get_commands()` and `get_top_n_routes()` gain an optional
`start: RobotState | None = None` parameter, defaulting to the existing
config-derived start pose so all current callers/tests (which don't pass it)
are unaffected. Live mode passes the parsed `arena_to_robot_start()` result.

## 7. `simulator/main.py` — live mode

New flags, additive to the existing CLI: `python -m simulator.main --live
[--host IP] [--execute]`. Existing fixed/random-obstacle CLI usage is
unchanged.

New phases layered onto the existing `show_all → highlight → animate →
done` state machine:

- **`waiting_for_arena`** (live-mode entry phase) — shows "Waiting for arena
  data from `<host>:5001`…" plus a small connection-status line (connected /
  reconnecting) sourced from the listener thread's state. Each frame,
  non-blockingly drains the snapshot queue.
- On a snapshot arriving: parse via `arena_feed`, run `_compute()` (now
  taking the live obstacles + live robot start pose), then go straight to
  `animate` — skipping the `show_all`/`highlight` route-comparison flair,
  same as today's CLI `fixed`-obstacles mode, since this is a real run and
  shouldn't add artificial delay before the car starts moving.
- **`executing`** — entered from `done` only when `--execute` is set. The
  car executor thread opens `CarConnection` (port 5000) and sends the
  optimal command list one at a time, updating a lock-protected progress
  object (`index`, `total`, `last_wire`, `error`) after each acknowledgment.
  The main thread keeps rendering the finished route with the robot at rest,
  plus a status bar: `"Sending to car: FW050 (3/12)…"`, turning green on
  success or red with the error text on failure. `running` / `completed` /
  `error` `algorithm_status` messages are relayed back over the arena socket
  via `arena_feed.send_status`.
- After a run fully completes (animate-only, or animate+execute), the phase
  returns to `waiting_for_arena` so the next arena snapshot is picked up
  without restarting the process. Snapshots arriving while a run is active
  are naturally not consumed until the queue is next polled in
  `waiting_for_arena` — one run completes before the next starts, matching
  `live_arena.py`'s existing sequential behavior.
- Malformed/invalid snapshots produce a transient red banner in the window
  (~5s) in addition to the console log and the `algorithm_status: error`
  reply.

`R` (reset) and the route-comparison keys keep their current meaning in
demo/CLI modes; in `--live` mode `R` is a no-op (obstacles come from the
network, not local regeneration).

## 8. `live_arena.py` changes

Becomes a thin wrapper around `arena_feed.listen`; its `process_snapshot`
now also calls `arena_feed.arena_to_robot_start()` and passes the result
into `get_commands(obstacles, start=...)`, fixing today's latent bug where
the robot always started from the fixed config pose regardless of what the
tablet reported.

## 9. Testing

- New `Algorithm/tests/test_arena_feed.py`: `arena_to_robot_start` cm-math
  for all 4 directions, `arena_to_obstacles` validation (moved from
  existing coverage), `send_status` payload shape.
- `planner.py`: tests confirming `get_commands(obstacles, start=X)` actually
  starts from `X` rather than the config default.
- Manual smoke test (performed during implementation, not just claimed): a
  throwaway local TCP script that sends the exact JSON sample from this spec
  to exercise `--live` end-to-end against the real pygame window, then
  `--live --execute` against `RaspberryPi/Robot/server.py` (or a small mock
  car server) to exercise the execute path.

## 10. Out of scope

- Bluetooth transport (simulator only ever talks TCP, same as today).
- Image-recognition result reporting (`obstacle.id`/`targetId` beyond
  logging).
- Re-planning when an arena snapshot changes mid-run — the in-flight run
  always finishes first.
