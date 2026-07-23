import math
from typing import TYPE_CHECKING

from simulator.arena import cm_to_px
from simulator.config import CELL_CM, CELL_PX, DEG_PER_FRAME, ROBOT_W_CM, STEP_CM_PER_FRAME
from simulator.types import Command, RobotState

if TYPE_CHECKING:
    import pygame

_ARROW_INSET_PX = 22  # arrow triangle depth in px (scaled for 30 cm body)
_ARROW_HALF_PX = 15   # arrow triangle half-height in px


def move_forward(state: RobotState, cm: float) -> RobotState:
    """state.x, state.y is the robot's body center (the real car turns about
    its center, not its front tip) — FW/BW moves it along the current heading."""
    rad = math.radians(state.theta)
    return RobotState(
        x=state.x + cm * math.cos(rad),
        y=state.y + cm * math.sin(rad),
        theta=state.theta,
    )


def rotate(state: RobotState, deg: float, clockwise: bool) -> RobotState:
    """In-place turn about the body center: state.x, state.y is unchanged,
    only heading changes. The front tip sweeps around this fixed center as
    theta changes — see draw_robot for where that tip actually renders."""
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
    if cmd.kind == 'RL':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=False), remaining - advance
    if cmd.kind == 'RR':
        advance = min(DEG_PER_FRAME, remaining)
        return rotate(state, advance, clockwise=True), remaining - advance
    if cmd.kind == 'WAIT':
        return state, remaining - 1.0
    raise ValueError(f"Unknown command kind: {cmd.kind!r}")


def draw_robot(surface: "pygame.Surface", state: RobotState) -> None:
    import pygame

    # Robot is square: 30 cm × 30 cm = 120 px × 120 px
    size_px = int(ROBOT_W_CM * CELL_PX / CELL_CM)

    robot_surf = pygame.Surface((size_px, size_px), pygame.SRCALPHA)
    pygame.draw.rect(robot_surf, (30, 100, 200), (0, 0, size_px, size_px))

    # Facing arrow: tip at the right-centre edge (East when theta=0) — this
    # tip is the robot's front, 15cm ahead of the tracked center point.
    arrow = [
        (size_px, size_px // 2),
        (size_px - _ARROW_INSET_PX, size_px // 2 - _ARROW_HALF_PX),
        (size_px - _ARROW_INSET_PX, size_px // 2 + _ARROW_HALF_PX),
    ]
    pygame.draw.polygon(robot_surf, (255, 220, 0), arrow)

    # pygame.transform.rotate is CCW in screen space; our theta is CCW from East;
    # pygame's Y-flip means screen-CCW visually matches math-CCW convention.
    rotated = pygame.transform.rotate(robot_surf, state.theta)

    # The drawing surface is centered on the robot's body — which state.x,
    # state.y already tracks directly, so no offset is needed here (unlike a
    # front-tip-tracked model, where this point would need to be computed).
    cx_px, cy_px = cm_to_px(state.x, state.y)
    rect = rotated.get_rect(center=(cx_px, cy_px))
    surface.blit(rotated, rect)
