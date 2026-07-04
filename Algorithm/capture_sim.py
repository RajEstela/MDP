"""
Verification capture for the MDP simulator.
Patches the game loop to run for a limited number of frames,
capturing screenshots and verifying state at key moments.
"""
import os, sys

os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import pygame
from simulator.arena import draw_arena, draw_obstacles
from simulator.config import ARENA_PX, FPS
from simulator.planner import OBSTACLES, get_commands
from simulator.robot import draw_robot, step_command
from simulator.types import RobotState

SCREENSHOTS_DIR = os.path.join(os.path.dirname(__file__), "verify_screenshots")
os.makedirs(SCREENSHOTS_DIR, exist_ok=True)

def save(surface, name):
    path = os.path.join(SCREENSHOTS_DIR, name)
    pygame.image.save(surface, path)
    print(f"SAVED: {path}")

CHECKS_PASSED = []
CHECKS_FAILED = []

def check(name, condition, detail=""):
    if condition:
        CHECKS_PASSED.append(name)
        print(f"  PASS: {name}" + (f" — {detail}" if detail else ""))
    else:
        CHECKS_FAILED.append(name)
        print(f"  FAIL: {name}" + (f" — {detail}" if detail else ""))

def main():
    pygame.init()
    screen = pygame.display.set_mode((ARENA_PX, ARENA_PX))
    pygame.display.set_caption("MDP Simulator — Stage 1")
    clock = pygame.time.Clock()

    commands = get_commands(OBSTACLES)
    start_pose = RobotState(x=0.0, y=0.0, theta=90.0)
    state = RobotState(x=start_pose.x, y=start_pose.y, theta=start_pose.theta)
    queue = list(commands)
    active = None
    remaining: float = 0.0
    paused = False

    # ── Window properties ────────────────────────────────────────────────────
    caption = pygame.display.get_caption()[0]
    check("Window title", caption == "MDP Simulator — Stage 1", f"got: {caption!r}")
    size = screen.get_size()
    check("Window size 800×800", size == (800, 800), f"got: {size}")

    # ── Initial render (robot at start, no animation yet) ────────────────────
    screen.fill((30, 30, 30))
    draw_arena(screen)
    draw_obstacles(screen, OBSTACLES)
    draw_robot(screen, state)
    pygame.display.flip()
    save(screen, "01_initial.png")

    # Background fill — (30,30,30) before grid/alpha blit; check near edge
    bg = screen.get_at((2, 2))
    check("Background dark grey", bg[:3] == (30, 30, 30), f"got: {bg[:3]}")

    # Grid lines appear at multiples of 40px; pixel at (40,2) should be a grid line (60,60,60)
    grid = screen.get_at((40, 2))
    check("Grid line at x=40px", grid[:3] == (60, 60, 60), f"got: {grid[:3]}")

    # Start zone — bottom-left quadrant. Arena bottom = screen y=800.
    # Start zone covers y_screen 640..800, x_screen 0..160.
    # Robot starts at (0,0) and its centre is near (40,758), so check a spot away from the robot.
    # Use x=140 (near right edge of start zone) y=790 (very bottom), away from robot centre.
    sz = screen.get_at((140, 790))
    check("Start zone green tint exists", sz[1] > sz[0] and sz[1] > sz[2],
          f"RGB {sz[:3]} — green channel should dominate")

    # Obstacle #0: x=50cm,y=50cm → px=200, py=800-200=600. Check centre of obstacle cell.
    obs_cx = int(50 * 40 / 10) + 20   # x_cm * CELL_PX/CELL_CM + half cell = 200+20=220
    obs_cy = 800 - int(50 * 40 / 10) - 20  # 800-200-20 = 580
    obs_px = screen.get_at((obs_cx, obs_cy))
    check("Obstacle #0 grey fill", obs_px[0] > 100 and abs(int(obs_px[0])-int(obs_px[1])) < 20,
          f"RGB {obs_px[:3]}")

    # Obstacle #0 face=N → orange tick on top edge of obstacle rect
    # top_py for obstacle at y=50cm = 800 - (50+10)*4 = 800-240 = 560
    tick_px = screen.get_at((obs_cx, 560))
    check("Obstacle #0 N-face orange tick", tick_px[0] > 200 and tick_px[1] > 100 and tick_px[2] < 50,
          f"RGB {tick_px[:3]}")

    # Robot at start: x=0,y=0,theta=90. Centre of robot in px:
    # cx_cm = 0 + 20/2 = 10 → cx_px = 40
    # cy_cm = 0 + 21/2 = 10.5 → cy_py = 800 - 42 = 758
    robot_px = screen.get_at((40, 758))
    check("Robot blue at start pos", robot_px[2] > 100 and robot_px[2] > robot_px[0],
          f"RGB {robot_px[:3]}")

    print(f"\nRobot initial state: {state}")

    # ── Animate 120 frames ───────────────────────────────────────────────────
    for frame in range(120):
        pygame.event.pump()
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

    save(screen, "02_after_120_frames.png")
    print(f"State after 120 frames: {state}")
    moved = (state.x != 0.0 or state.y != 0.0 or state.theta != 90.0)
    check("Robot moved after animation", moved, f"pos=({state.x:.1f},{state.y:.1f}) theta={state.theta:.1f}")

    # ── Pause: simulate SPACE ────────────────────────────────────────────────
    paused = True
    state_frozen = RobotState(x=state.x, y=state.y, theta=state.theta)
    for _ in range(30):
        pygame.event.pump()
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

    still = (state.x == state_frozen.x and state.y == state_frozen.y and state.theta == state_frozen.theta)
    check("Robot frozen when paused", still,
          f"before=({state_frozen.x:.1f},{state_frozen.y:.1f}), after=({state.x:.1f},{state.y:.1f})")

    # Resume
    paused = False

    # ── Reset: simulate R ────────────────────────────────────────────────────
    state = RobotState(x=start_pose.x, y=start_pose.y, theta=start_pose.theta)
    queue = list(commands)
    active = None
    remaining = 0.0
    screen.fill((30, 30, 30))
    draw_arena(screen)
    draw_obstacles(screen, OBSTACLES)
    draw_robot(screen, state)
    pygame.display.flip()
    save(screen, "03_after_reset.png")
    print(f"State after reset: {state}")
    check("R resets pose to start", state.x == 0.0 and state.y == 0.0 and state.theta == 90.0,
          f"{state}")
    check("R reloads full queue", len(queue) == len(commands),
          f"queue={len(queue)} commands={len(commands)}")

    # ── Quit cleanly ─────────────────────────────────────────────────────────
    pygame.quit()
    check("pygame.quit() called cleanly", True)

    # ── Summary ──────────────────────────────────────────────────────────────
    print(f"\n{'='*50}")
    print(f"PASSED: {len(CHECKS_PASSED)}/{len(CHECKS_PASSED)+len(CHECKS_FAILED)}")
    if CHECKS_FAILED:
        print(f"FAILED: {CHECKS_FAILED}")
        sys.exit(1)
    else:
        print("ALL CHECKS PASSED")

if __name__ == '__main__':
    main()
