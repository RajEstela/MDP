import pygame

from simulator.arena import draw_arena, draw_obstacles
from simulator.config import ARENA_PX, FPS, START_THETA, START_X_CM, START_Y_CM
from simulator.planner import OBSTACLES, get_commands
from simulator.robot import draw_robot, step_command
from simulator.types import RobotState


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((ARENA_PX, ARENA_PX))
    pygame.display.set_caption("MDP Simulator — Stage 1")
    clock = pygame.time.Clock()

    commands = get_commands(OBSTACLES)
    start_pose = RobotState(x=START_X_CM, y=START_Y_CM, theta=START_THETA)

    state = RobotState(x=start_pose.x, y=start_pose.y, theta=start_pose.theta)
    queue: list = list(commands)
    active = None
    remaining: float = 0.0
    paused = False

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    state = RobotState(x=start_pose.x, y=start_pose.y, theta=start_pose.theta)
                    queue = list(commands)
                    active = None
                    remaining = 0.0

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

    pygame.quit()


if __name__ == '__main__':
    main()
