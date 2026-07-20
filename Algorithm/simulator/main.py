import math
import queue
import sys
import threading
import time

import pygame

import arena_feed
from app_config import RPI_HOST
from comms import CarConnection
from simulator.arena import draw_arena, draw_obstacles, draw_path, trace_path_points
from simulator.config import ARENA_PX, FPS, START_THETA, START_X_CM, START_Y_CM
from simulator.planner import generate_random_obstacles, get_top_n_routes
from simulator.robot import draw_robot, step_command
from simulator.types import Obstacle, RobotState

# ── phase durations ──────────────────────────────────────────────────────────
_SHOW_ALL_FRAMES  = int(4 * FPS)
_HIGHLIGHT_FRAMES = int(2 * FPS)

# ── route colours: index 0 = optimal (gold), 1–4 = suboptimal ───────────────
_ROUTE_COLORS = [
    (255, 215,   0),
    ( 80, 210, 130),
    ( 80, 160, 230),
    (215, 130,  50),
    (190,  90, 210),
]
_DIM_COLOR = (55, 55, 55)


def _parse_obstacle(spec: str) -> Obstacle:
    """Parse '100,80,N' or 'Obstacle(100,80,N)' into an Obstacle."""
    spec = spec.strip()
    if spec.lower().startswith('obstacle(') and spec.endswith(')'):
        spec = spec[9:-1]
    parts = [p.strip() for p in spec.split(',')]
    if len(parts) != 3:
        raise ValueError(f"Expected x,y,Face — got {spec!r}")
    x, y, face = int(parts[0]), int(parts[1]), parts[2].upper()
    if face not in ('N', 'S', 'E', 'W'):
        raise ValueError(f"Face must be N/S/E/W — got {face!r}")
    return Obstacle(x=x, y=y, face=face)


def _parse_args() -> tuple[list[Obstacle] | None, int]:
    """Return (fixed_obstacles, n_random).

    fixed_obstacles — use these exact obstacles every run (R resets to same set)
    n_random        — generate this many random obstacles each run (fixed=None)

    Usage:
      python -m simulator.main                        # 5 random
      python -m simulator.main --random 3             # 3 random
      python -m simulator.main 100,80,N 150,100,E     # exact obstacles
      python -m simulator.main A5_Test 100,80,N       # single obstacle (A5)
    """
    args = sys.argv[1:]
    if not args:
        return None, 5

    # --random N
    if args[0] in ('--random', '-n'):
        if len(args) < 2 or not args[1].isdigit():
            print("Usage: python -m simulator.main --random <N>")
            sys.exit(1)
        return None, int(args[1])

    # Backwards compat: leading A5_Test token
    if args[0].upper().replace('-', '_') in ('A5', 'A5_TEST'):
        args = args[1:]
        if not args:
            print("Usage: python -m simulator.main A5_Test x,y,Face ...")
            sys.exit(1)

    # One or more obstacle specs
    obstacles: list[Obstacle] = []
    for spec in args:
        try:
            obstacles.append(_parse_obstacle(spec))
        except ValueError as exc:
            print(f"Bad obstacle spec {spec!r}: {exc}")
            sys.exit(1)
    return obstacles, len(obstacles)


def _text(surface, font, msg, color, pos):
    surface.blit(font.render(msg, True, color), pos)


def _draw_legend(surface, font, font_b, routes, phase, n_obs):
    n_perms = math.factorial(n_obs)
    n_routes = len(routes)
    x, y = 8, 8
    surface.blit(
        font_b.render(f"Top {n_routes} of {n_perms} routes:", True, (210, 210, 210)),
        (x, y),
    )
    y += 24
    for i, (_, length) in enumerate(routes):
        is_opt = i == 0
        if phase == 'show_all':
            dot_col, txt_col = _ROUTE_COLORS[i], (255, 255, 255)
        else:
            dot_col = _ROUTE_COLORS[0] if is_opt else _DIM_COLOR
            txt_col = (255, 255, 255) if is_opt else (70, 70, 70)
        tag = "★ OPTIMAL" if is_opt else f"  Route {i + 1}"
        pygame.draw.circle(surface, dot_col, (x + 6, y + 9), 6)
        surface.blit(font.render(f"{tag}: {length:.0f} cm", True, txt_col), (x + 18, y))
        y += 22


def _draw_hud(surface, font, font_b, elapsed_s, opt_len, visited, n_obs, paused, done):
    bar = pygame.Surface((ARENA_PX, 36), pygame.SRCALPHA)
    bar.fill((0, 0, 0, 170))
    surface.blit(bar, (0, ARENA_PX - 36))
    if done:
        status, scol = "Image recognized!" if n_obs == 1 else "All images recognized!", (0, 255, 120)
    elif paused:
        status, scol = "PAUSED  [SPACE] resume", (255, 210, 0)
    else:
        status, scol = "Running...  [SPACE] pause", (200, 200, 200)
    left  = font.render(f"Elapsed: {elapsed_s:.1f} s", True, (200, 200, 200))
    mid   = font_b.render(f"Optimal: {opt_len:.0f} cm  |  Visited: {visited}/{n_obs}", True, (255, 215, 0))
    right = font.render(status, True, scol)
    y = ARENA_PX - 27
    surface.blit(left,  (10, y))
    surface.blit(mid,   (ARENA_PX // 2 - mid.get_width() // 2, y))
    surface.blit(right, (ARENA_PX - right.get_width() - 10, y))


class _LiveState:
    """Thread-safe state shared between the arena listener thread, the car
    executor thread, and the pygame main thread."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.connection_status = "connecting"
        self.last_error: str | None = None
        self.last_error_time: float = 0.0
        self.exec_progress: dict | None = None

    def set_status(self, status: str) -> None:
        with self.lock:
            self.connection_status = status

    def set_error(self, message: str) -> None:
        with self.lock:
            self.last_error = message
            self.last_error_time = time.monotonic()


def _on_snapshot(snapshot: dict, sock, out_queue: "queue.Queue", live_state: _LiveState) -> None:
    try:
        obstacles = arena_feed.arena_to_obstacles(snapshot)
        start = arena_feed.arena_to_robot_start(snapshot)
    except ValueError as exc:
        live_state.set_error(str(exc))
        raise
    revision = int(snapshot.get("revision", 0))
    arena_feed.send_status(sock, revision, "planning", "Calculating route")
    out_queue.put((obstacles, start, revision, sock))


def _run_car_executor(commands: list, host: str, sock, revision: int, live_state: _LiveState) -> None:
    total = sum(1 for c in commands if c.kind != 'WAIT')
    with live_state.lock:
        live_state.exec_progress = {"index": 0, "total": total, "last_wire": "", "done": False, "error": None}

    def on_progress(sent: int, sent_total: int, wire: str) -> None:
        with live_state.lock:
            live_state.exec_progress["index"] = sent
            live_state.exec_progress["last_wire"] = wire

    arena_feed.send_status(sock, revision, "running", "Sending route to nanocar")
    try:
        with CarConnection(host=host) as car:
            car.send_commands(commands, on_progress=on_progress)
    except Exception as exc:
        with live_state.lock:
            live_state.exec_progress["error"] = str(exc)
            live_state.exec_progress["done"] = True
        arena_feed.send_status(sock, revision, "error", str(exc))
        return

    with live_state.lock:
        live_state.exec_progress["done"] = True
    arena_feed.send_status(sock, revision, "completed", "Route completed")


def _draw_connection_banner(surface, font, live_state: _LiveState) -> None:
    with live_state.lock:
        status = live_state.connection_status
        last_error = live_state.last_error
        last_error_time = live_state.last_error_time

    now = time.monotonic()
    if last_error and now - last_error_time < 5.0:
        msg = font.render(f"Arena error: {last_error}", True, (255, 80, 80))
        surface.blit(msg, (8, ARENA_PX - 26))
        return

    colors = {"connecting": (200, 200, 0), "connected": (80, 220, 80), "reconnecting": (255, 140, 0)}
    col = colors.get(status, (200, 200, 200))
    msg = font.render(f"[{status}]", True, col)
    surface.blit(msg, (ARENA_PX - msg.get_width() - 8, 8))


def _draw_exec_overlay(surface, font_b, progress: dict | None) -> None:
    bar = pygame.Surface((ARENA_PX, 30), pygame.SRCALPHA)
    bar.fill((0, 0, 0, 190))
    surface.blit(bar, (0, ARENA_PX - 66))
    if progress is None:
        text, col = "Preparing to send commands to car...", (200, 200, 0)
    elif progress.get("error"):
        text, col = f"Car error: {progress['error']}", (255, 80, 80)
    elif progress.get("done"):
        text, col = "All commands sent — car finished", (80, 255, 120)
    else:
        text = f"Sending to car: {progress['last_wire']} ({progress['index']}/{progress['total']})..."
        col = (255, 215, 0)
    msg = font_b.render(text, True, col)
    surface.blit(msg, (ARENA_PX // 2 - msg.get_width() // 2, ARENA_PX - 60))


def _compute(screen, font_b, obstacles, start: RobotState | None = None):
    """Show loading screen, compute top-N routes; return (routes, traced, opt_cmds, opt_len, start)."""
    if start is None:
        start = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
    n_obs = len(obstacles)
    n_routes = min(5, math.factorial(n_obs))
    screen.fill((30, 30, 30))
    draw_arena(screen)
    draw_obstacles(screen, obstacles)
    msg = font_b.render(
        f"Computing optimal path across {math.factorial(n_obs)} permutation{'s' if n_obs > 1 else ''}…",
        True, (255, 215, 0),
    )
    screen.blit(msg, (ARENA_PX // 2 - msg.get_width() // 2, ARENA_PX // 2 - 12))
    pygame.display.flip()
    pygame.event.pump()

    routes = get_top_n_routes(obstacles, n=n_routes, start=start)
    opt_cmds, opt_len = routes[0]
    traced = [trace_path_points(start, cmds) for cmds, _ in routes]
    return routes, traced, opt_cmds, opt_len, start


def _parse_live_args(args: list[str]) -> tuple[str, bool]:
    """Parse the remaining CLI args (after --live is stripped) into (host, execute)."""
    host = RPI_HOST
    execute = False
    i = 0
    while i < len(args):
        if args[i] == '--host':
            if i + 1 >= len(args):
                print("Usage: python -m simulator.main --live [--host <ip>] [--execute]")
                sys.exit(1)
            host = args[i + 1]
            i += 2
        elif args[i] == '--execute':
            execute = True
            i += 1
        else:
            print(f"Unknown live-mode argument: {args[i]!r}")
            sys.exit(1)
    return host, execute


def _run_live(host: str, execute: bool, max_frames: int | None = None) -> None:
    pygame.init()
    screen = pygame.display.set_mode((ARENA_PX, ARENA_PX))
    pygame.display.set_caption("MDP Simulator — live")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont('Arial', 16)
    font_b = pygame.font.SysFont('Arial', 18, bold=True)

    live_state = _LiveState()
    out_queue: "queue.Queue" = queue.Queue()
    listener = threading.Thread(
        target=arena_feed.listen,
        args=(host, lambda snap, sock: _on_snapshot(snap, sock, out_queue, live_state)),
        kwargs={"on_status": live_state.set_status},
        daemon=True,
    )
    listener.start()

    phase = 'waiting_for_arena'
    obstacles: list[Obstacle] = []
    routes: list = []
    traced: list = []
    opt_cmds: list = []
    opt_len = 0.0
    state = RobotState(x=0.0, y=0.0, theta=90.0)
    cmd_queue: list = []
    active = None
    remaining = 0.0
    anim_frames = 0
    obstacles_visited = 0
    current_sock = None
    current_revision = 0
    executor_started = False

    def _start_new_run(obs, start, revision, sock):
        nonlocal obstacles, routes, traced, opt_cmds, opt_len, state, cmd_queue
        nonlocal active, remaining, anim_frames, obstacles_visited
        nonlocal current_sock, current_revision, executor_started, phase
        obstacles = obs
        routes, traced, opt_cmds, opt_len, start = _compute(screen, font_b, obstacles, start)
        state = RobotState(x=start.x, y=start.y, theta=start.theta)
        cmd_queue = list(opt_cmds)
        active = None
        remaining = 0.0
        anim_frames = 0
        obstacles_visited = 0
        current_sock = sock
        current_revision = revision
        executor_started = False
        phase = 'animate'

    frame = 0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (pygame.K_q, pygame.K_ESCAPE):
                running = False

        if phase in ('waiting_for_arena', 'done'):
            latest = None
            while True:
                try:
                    latest = out_queue.get_nowait()
                except queue.Empty:
                    break
            if latest is not None:
                _start_new_run(*latest)

        screen.fill((30, 30, 30))
        draw_arena(screen)

        if phase == 'waiting_for_arena':
            msg = font_b.render(
                f"Waiting for arena data from {host}:{arena_feed.ARENA_PORT}...",
                True, (210, 210, 210),
            )
            screen.blit(msg, (ARENA_PX // 2 - msg.get_width() // 2, ARENA_PX // 2 - 12))

        elif phase in ('animate', 'executing', 'done'):
            draw_path(screen, traced[0], _ROUTE_COLORS[0], width=2)
            draw_obstacles(screen, obstacles)
            _draw_legend(screen, font, font_b, routes, phase, len(obstacles))

            if phase == 'animate':
                if active is None and cmd_queue:
                    active = cmd_queue.pop(0)
                    remaining = active.value
                    if active.kind == 'WAIT':
                        obstacles_visited += 1
                if active is not None:
                    state, remaining = step_command(state, active, remaining)
                    if remaining <= 0:
                        active = None
                anim_frames += 1
                if not cmd_queue and active is None:
                    arena_feed.send_status(
                        current_sock, current_revision, "route_ready", "Route calculated",
                        commandCount=sum(1 for c in opt_cmds if c.kind != 'WAIT'),
                    )
                    phase = 'executing' if execute else 'done'

            draw_robot(screen, state)
            _draw_hud(
                screen, font, font_b, anim_frames / FPS, opt_len,
                obstacles_visited, len(obstacles), False, phase != 'animate',
            )

            if phase == 'executing':
                if not executor_started:
                    executor_started = True
                    threading.Thread(
                        target=_run_car_executor,
                        args=(list(opt_cmds), host, current_sock, current_revision, live_state),
                        daemon=True,
                    ).start()
                with live_state.lock:
                    progress = dict(live_state.exec_progress) if live_state.exec_progress else None
                _draw_exec_overlay(screen, font_b, progress)
                if progress and progress.get("done"):
                    phase = 'done'

            if phase == 'done':
                done_msg = font_b.render(
                    "Run complete — waiting for next arena snapshot", True, (0, 255, 120),
                )
                screen.blit(done_msg, (ARENA_PX // 2 - done_msg.get_width() // 2, 40))

        _draw_connection_banner(screen, font, live_state)
        pygame.display.flip()
        clock.tick(FPS)

        frame += 1
        if max_frames is not None and frame >= max_frames:
            running = False

    pygame.quit()


def main() -> None:
    argv = sys.argv[1:]
    if '--live' in argv:
        host, execute = _parse_live_args([a for a in argv if a != '--live'])
        _run_live(host, execute)
        return

    fixed_obstacles, n_random = _parse_args()
    fixed = fixed_obstacles is not None

    pygame.init()
    if fixed:
        n = len(fixed_obstacles)
        title = f"MDP Simulator — {n} obstacle{'s' if n != 1 else ''} (fixed)"
    else:
        title = f"MDP Simulator — {n_random} random obstacle{'s' if n_random != 1 else ''}"
    screen = pygame.display.set_mode((ARENA_PX, ARENA_PX))
    pygame.display.set_caption(title)
    clock = pygame.time.Clock()
    font   = pygame.font.SysFont('Arial', 16)
    font_b = pygame.font.SysFont('Arial', 18, bold=True)

    def new_run():
        obs = fixed_obstacles if fixed else generate_random_obstacles(n_random)
        rts, trc, opt_c, opt_l, start = _compute(screen, font_b, obs)
        robot = RobotState(x=start.x, y=start.y, theta=start.theta)
        return obs, rts, trc, opt_c, opt_l, robot, list(opt_c), None, 0.0, 0, 0

    obstacles, routes, traced, opt_cmds, opt_len, \
        state, queue, active, remaining, anim_frames, obstacles_visited = new_run()

    # Fixed obstacles: skip comparison, animate immediately
    phase        = 'animate' if fixed else 'show_all'
    phase_frames = 0
    paused       = False

    running = True
    while running:
        n_obs = len(obstacles)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_r:
                    obstacles, routes, traced, opt_cmds, opt_len, \
                        state, queue, active, remaining, anim_frames, obstacles_visited = new_run()
                    phase = 'animate' if fixed else 'show_all'
                    phase_frames = 0
                    paused = False
                elif event.key == pygame.K_SPACE:
                    if phase in ('show_all', 'highlight'):
                        phase = 'animate'
                        phase_frames = 0
                    else:
                        paused = not paused

        screen.fill((30, 30, 30))
        draw_arena(screen)

        if phase == 'show_all':
            for i in range(len(routes) - 1, -1, -1):
                draw_path(screen, traced[i], _ROUTE_COLORS[i], width=2 if i > 0 else 3)
            draw_obstacles(screen, obstacles)
            _draw_legend(screen, font, font_b, routes, phase, n_obs)
            _text(screen, font_b, "Comparing candidate paths…  [SPACE] skip",
                  (210, 210, 210), (10, ARENA_PX - 30))
            phase_frames += 1
            if phase_frames >= _SHOW_ALL_FRAMES:
                phase = 'highlight'
                phase_frames = 0

        elif phase == 'highlight':
            for i in range(1, len(routes)):
                draw_path(screen, traced[i], _DIM_COLOR, width=1)
            draw_path(screen, traced[0], _ROUTE_COLORS[0], width=3)
            draw_obstacles(screen, obstacles)
            _draw_legend(screen, font, font_b, routes, phase, n_obs)
            _text(screen, font_b, "Optimal path selected — starting run!",
                  (255, 215, 0), (10, ARENA_PX - 30))
            phase_frames += 1
            if phase_frames >= _HIGHLIGHT_FRAMES:
                phase = 'animate'
                phase_frames = 0

        elif phase in ('animate', 'done'):
            draw_path(screen, traced[0], _ROUTE_COLORS[0], width=2)
            draw_obstacles(screen, obstacles)
            _draw_legend(screen, font, font_b, routes, phase, n_obs)

            if phase == 'animate' and not paused:
                if active is None and queue:
                    active = queue.pop(0)
                    remaining = active.value
                    if active.kind == 'WAIT':
                        obstacles_visited += 1
                if active is not None:
                    state, remaining = step_command(state, active, remaining)
                    if remaining <= 0:
                        active = None
                anim_frames += 1
                if not queue and active is None:
                    phase = 'done'

            draw_robot(screen, state)
            _draw_hud(screen, font, font_b, anim_frames / FPS, opt_len,
                      obstacles_visited, n_obs, paused, phase == 'done')

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == '__main__':
    main()
