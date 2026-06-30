import math
from typing import TYPE_CHECKING

from simulator.arena import cm_to_px
from simulator.config import CELL_CM, CELL_PX, DEG_PER_FRAME, ROBOT_H_CM, ROBOT_W_CM, STEP_CM_PER_FRAME, TURN_RADIUS_CM
from simulator.types import Command, RobotState

if TYPE_CHECKING:
    import pygame

_ARROW_INSET_PX = 15  # arrow triangle depth in px
_ARROW_HALF_PX = 10   # arrow triangle half-height in px


def move_forward(state: RobotState, cm: float) -> RobotState:
    rad = math.radians(state.theta)
    return RobotState(
        x=state.x + cm * math.cos(rad),
        y=state.y + cm * math.sin(rad),
        theta=state.theta,
    )


def rotate(state: RobotState, deg: float, clockwise: bool) -> RobotState:
    delta = -deg if clockwise else deg
    return RobotState(x=state.x, y=state.y, theta=(state.theta + delta) % 360)


def arc_step(state: RobotState, ds: float, clockwise: bool, r: float) -> RobotState:
    sign = -1 if clockwise else 1
    theta_rad = math.radians(state.theta)
    new_theta_rad = theta_rad + sign * ds / r
    new_x = state.x + sign * r * (math.sin(new_theta_rad) - math.sin(theta_rad))
    new_y = state.y - sign * r * (math.cos(new_theta_rad) - math.cos(theta_rad))
    return RobotState(x=new_x, y=new_y, theta=math.degrees(new_theta_rad) % 360)


def step_command(
    state: RobotState, cmd: Command, remaining: float
) -> tuple[RobotState, float]:
    if cmd.kind == 'FW':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return move_forward(state, advance), remaining - advance
    if cmd.kind == 'BW':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return move_forward(state, -advance), remaining - advance
    if cmd.kind == 'TL':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=False), remaining - advance
    if cmd.kind == 'TR':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=True), remaining - advance
    if cmd.kind == 'AL':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return arc_step(state, advance, clockwise=False, r=TURN_RADIUS_CM), remaining - advance
    if cmd.kind == 'AR':
        advance = min(STEP_CM_PER_FRAME, remaining)
        return arc_step(state, advance, clockwise=True, r=TURN_RADIUS_CM), remaining - advance
    raise ValueError(f"Unknown command kind: {cmd.kind!r}")


def draw_robot(surface: "pygame.Surface", state: RobotState) -> None:
    import pygame

    w_px = int(ROBOT_W_CM * CELL_PX / CELL_CM)  # 80px
    h_px = int(ROBOT_H_CM * CELL_PX / CELL_CM)  # 84px

    robot_surf = pygame.Surface((w_px, h_px), pygame.SRCALPHA)
    pygame.draw.rect(robot_surf, (30, 100, 200), (0, 0, w_px, h_px))

    # Facing arrow: triangle pointing right (East = theta=0, the unrotated default)
    arrow = [
        (w_px, h_px // 2),
        (w_px - _ARROW_INSET_PX, h_px // 2 - _ARROW_HALF_PX),
        (w_px - _ARROW_INSET_PX, h_px // 2 + _ARROW_HALF_PX),
    ]
    pygame.draw.polygon(robot_surf, (255, 220, 0), arrow)

    # pygame.transform.rotate is CCW in screen space; our theta is CCW from East;
    # pygame's Y-flip means screen-CCW visually matches math-CCW convention.
    rotated = pygame.transform.rotate(robot_surf, state.theta)

    cx_cm = state.x + ROBOT_W_CM / 2
    cy_cm = state.y + ROBOT_H_CM / 2
    cx_px, cy_px = cm_to_px(cx_cm, cy_cm)
    rect = rotated.get_rect(center=(cx_px, cy_px))
    surface.blit(rotated, rect)
