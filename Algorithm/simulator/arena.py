import math
from typing import TYPE_CHECKING

from simulator.config import ARENA_PX, CELL_CM, CELL_PX, GRID_SIZE, TURN_RADIUS_CM
from simulator.types import Obstacle

if TYPE_CHECKING:
    import pygame
    from simulator.types import Command, RobotState


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


_IMAGE_FACE_COLOR = (255, 50, 50)   # bright red stripe = image face
_IMAGE_FACE_DEPTH = 7               # px depth of stripe inside obstacle cell


def draw_obstacles(surface: "pygame.Surface", obstacles: list[Obstacle]) -> None:
    import pygame

    for obs in obstacles:
        left_px, bottom_py = cm_to_px(obs.x, obs.y)
        _, top_py = cm_to_px(obs.x, obs.y + CELL_CM)
        size_px = CELL_PX
        right_px = left_px + size_px

        # Obstacle body
        rect = pygame.Rect(left_px, top_py, size_px, size_px)
        pygame.draw.rect(surface, (150, 150, 150), rect)
        pygame.draw.rect(surface, (80, 80, 80), rect, 1)

        # Image-face indicator: bright red stripe inset on the image face
        d = _IMAGE_FACE_DEPTH
        if obs.face == 'N':
            stripe = pygame.Rect(left_px, top_py, size_px, d)
        elif obs.face == 'S':
            stripe = pygame.Rect(left_px, bottom_py - d, size_px, d)
        elif obs.face == 'E':
            stripe = pygame.Rect(right_px - d, top_py, d, size_px)
        else:  # 'W'
            stripe = pygame.Rect(left_px, top_py, d, size_px)
        pygame.draw.rect(surface, _IMAGE_FACE_COLOR, stripe)


def trace_path_points(
    start: "RobotState",
    cmds: list["Command"],
    r: float = TURN_RADIUS_CM,
) -> list[tuple[float, float]]:
    """Simulate a command sequence from start; return (x_cm, y_cm) sample points."""
    points: list[tuple[float, float]] = [(start.x, start.y)]
    x, y, theta = start.x, start.y, start.theta
    step = 2.0
    for cmd in cmds:
        if cmd.kind == 'WAIT':
            continue
        remaining = cmd.value
        while remaining > 0.001:
            advance = min(step, remaining)
            if cmd.kind == 'FW':
                rad = math.radians(theta)
                x += advance * math.cos(rad)
                y += advance * math.sin(rad)
            elif cmd.kind == 'BW':
                rad = math.radians(theta)
                x -= advance * math.cos(rad)
                y -= advance * math.sin(rad)
            elif cmd.kind in ('AL', 'AR'):
                sign = 1 if cmd.kind == 'AL' else -1
                rad = math.radians(theta)
                new_rad = rad + sign * advance / r
                x += sign * r * (math.sin(new_rad) - math.sin(rad))
                y -= sign * r * (math.cos(new_rad) - math.cos(rad))
                theta = math.degrees(new_rad) % 360
            remaining -= advance
            points.append((x, y))
    return points


def draw_path(
    surface: "pygame.Surface",
    points: list[tuple[float, float]],
    color: tuple,
    width: int = 2,
) -> None:
    """Draw a list of (x_cm, y_cm) points as a polyline on the surface."""
    import pygame
    if len(points) < 2:
        return
    pygame.draw.lines(surface, color, False, [cm_to_px(x, y) for x, y in points], width)
