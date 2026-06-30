import math

from simulator.config import DEG_PER_FRAME, STEP_CM_PER_FRAME
from simulator.types import Command, RobotState


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
    return state, 0.0
