import math
import sys

import pygame

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


def _parse_args() -> tuple[bool, Obstacle | None]:
    """Return (a5_mode, obstacle_or_None) from sys.argv."""
    args = sys.argv[1:]
    if not args:
        return False, None
    mode = args[0].upper().replace('-', '_')
    if mode in ('A5', 'A5_TEST'):
        if len(args) < 2:
            print("Usage: python -m simulator.main A5_Test x,y,Face")
            print("  e.g. python -m simulator.main A5_Test 100,80,N")
            sys.exit(1)
        try:
            obs = _parse_obstacle(args[1])
        except ValueError as exc:
            print(f"Bad obstacle spec: {exc}")
            sys.exit(1)
        return True, obs
    print(f"Unknown mode {args[0]!r}. Run without arguments for random 5-obstacle mode.")
    sys.exit(1)


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


def _compute(screen, font_b, obstacles):
    """Show loading screen, compute top-N routes; return (routes, traced, opt_cmds, opt_len)."""
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

    routes = get_top_n_routes(obstacles, n=n_routes)
    opt_cmds, opt_len = routes[0]
    traced = [trace_path_points(start, cmds) for cmds, _ in routes]
    return routes, traced, opt_cmds, opt_len


def main() -> None:
    a5_mode, a5_obstacle = _parse_args()

    pygame.init()
    title = "MDP Simulator — A5 Test" if a5_mode else "MDP Simulator — Optimal Hamiltonian Path"
    screen = pygame.display.set_mode((ARENA_PX, ARENA_PX))
    pygame.display.set_caption(title)
    clock = pygame.time.Clock()
    font   = pygame.font.SysFont('Arial', 16)
    font_b = pygame.font.SysFont('Arial', 18, bold=True)

    def new_run():
        obs = [a5_obstacle] if a5_mode else generate_random_obstacles()
        rts, trc, opt_c, opt_l = _compute(screen, font_b, obs)
        robot = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)
        return obs, rts, trc, opt_c, opt_l, robot, list(opt_c), None, 0.0, 0, 0

    obstacles, routes, traced, opt_cmds, opt_len, \
        state, queue, active, remaining, anim_frames, obstacles_visited = new_run()

    # A5 mode: skip comparison phases and go straight to animation
    phase        = 'animate' if a5_mode else 'show_all'
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
                    phase = 'animate' if a5_mode else 'show_all'
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
