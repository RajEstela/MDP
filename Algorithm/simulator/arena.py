from typing import TYPE_CHECKING

from simulator.config import ARENA_PX, CELL_CM, CELL_PX, GRID_SIZE
from simulator.types import Obstacle

if TYPE_CHECKING:
    import pygame


def cm_to_px(x_cm: float, y_cm: float) -> tuple[int, int]:
    px = int(x_cm * CELL_PX / CELL_CM)
    py = int(ARENA_PX - y_cm * CELL_PX / CELL_CM)
    return px, py


def draw_arena(surface: "pygame.Surface") -> None:
    import pygame

    surface.fill((30, 30, 30))
    for i in range(GRID_SIZE + 1):
        pos = i * CELL_PX
        pygame.draw.line(surface, (60, 60, 60), (pos, 0), (pos, ARENA_PX))
        pygame.draw.line(surface, (60, 60, 60), (0, pos), (ARENA_PX, pos))
    # Start zone: 40×40cm = 4×4 cells, bottom-left of arena
    _, top_py = cm_to_px(0, 4 * CELL_CM)
    start_size = 4 * CELL_PX
    start_surf = pygame.Surface((start_size, start_size), pygame.SRCALPHA)
    start_surf.fill((0, 180, 0, 80))
    surface.blit(start_surf, (0, top_py))


def draw_obstacles(surface: "pygame.Surface", obstacles: list[Obstacle]) -> None:
    import pygame

    for obs in obstacles:
        left_px, bottom_py = cm_to_px(obs.x, obs.y)
        _, top_py = cm_to_px(obs.x, obs.y + CELL_CM)
        size_px = CELL_PX
        right_px = left_px + size_px
        rect = pygame.Rect(left_px, top_py, size_px, size_px)
        pygame.draw.rect(surface, (150, 150, 150), rect)
        pygame.draw.rect(surface, (80, 80, 80), rect, 1)
        tick_color = (255, 140, 0)
        tick_w = 4
        if obs.face == 'N':
            pygame.draw.line(surface, tick_color, (left_px, top_py), (right_px, top_py), tick_w)
        elif obs.face == 'S':
            pygame.draw.line(surface, tick_color, (left_px, bottom_py), (right_px, bottom_py), tick_w)
        elif obs.face == 'E':
            pygame.draw.line(surface, tick_color, (right_px, top_py), (right_px, bottom_py), tick_w)
        elif obs.face == 'W':
            pygame.draw.line(surface, tick_color, (left_px, top_py), (left_px, bottom_py), tick_w)
