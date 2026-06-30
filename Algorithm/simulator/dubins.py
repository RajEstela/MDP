import math

from simulator.types import DubinsPath, RobotState


def _mod2pi(x: float) -> float:
    return x % (2 * math.pi)


def dubins_lsl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
    dx = (q2.x - q1.x) / r
    dy = (q2.y - q1.y) / r
    alpha = math.radians(q1.theta)
    beta = math.radians(q2.theta)
    tmp0 = dx + math.sin(alpha) - math.sin(beta)
    tmp1 = dy - math.cos(alpha) + math.cos(beta)
    p_sq = tmp0 ** 2 + tmp1 ** 2
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    theta = math.atan2(tmp1, tmp0)
    t = _mod2pi(theta - alpha)
    q = _mod2pi(beta - theta)
    return DubinsPath('LSL', t * r, p * r, q * r, (t + p + q) * r)


def dubins_rsr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
    dx = (q2.x - q1.x) / r
    dy = (q2.y - q1.y) / r
    alpha = math.radians(q1.theta)
    beta = math.radians(q2.theta)
    tmp0 = dx - math.sin(alpha) + math.sin(beta)
    tmp1 = dy + math.cos(alpha) - math.cos(beta)
    p_sq = tmp0 ** 2 + tmp1 ** 2
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    theta = math.atan2(tmp1, tmp0)
    t = _mod2pi(alpha - theta)
    q = _mod2pi(theta - beta)
    return DubinsPath('RSR', t * r, p * r, q * r, (t + p + q) * r)


def dubins_lsr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
    dx = (q2.x - q1.x) / r
    dy = (q2.y - q1.y) / r
    alpha = math.radians(q1.theta)
    beta = math.radians(q2.theta)
    tmp0 = dx + math.sin(alpha) + math.sin(beta)
    tmp1 = dy - math.cos(alpha) - math.cos(beta)
    p_sq = tmp0 ** 2 + tmp1 ** 2 - 4
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    theta = math.atan2(-math.cos(alpha) - math.cos(beta), tmp0) - math.atan2(-2, p)
    t = _mod2pi(theta - alpha)
    q = _mod2pi(theta - beta)
    return DubinsPath('LSR', t * r, p * r, q * r, (t + p + q) * r)


def dubins_rsl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
    dx = (q2.x - q1.x) / r
    dy = (q2.y - q1.y) / r
    alpha = math.radians(q1.theta)
    beta = math.radians(q2.theta)
    tmp0 = dx - math.sin(alpha) - math.sin(beta)
    tmp1 = dy + math.cos(alpha) + math.cos(beta)
    p_sq = tmp0 ** 2 + tmp1 ** 2 - 4
    if p_sq < 0:
        return None
    p = math.sqrt(p_sq)
    theta = math.atan2(math.cos(alpha) + math.cos(beta), tmp0) - math.atan2(2, p)
    t = _mod2pi(alpha - theta)
    q = _mod2pi(beta - theta)
    return DubinsPath('RSL', t * r, p * r, q * r, (t + p + q) * r)


def dubins_rlr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
    dx = (q2.x - q1.x) / r
    dy = (q2.y - q1.y) / r
    alpha = math.radians(q1.theta)
    beta = math.radians(q2.theta)
    tmp0 = (
        (dx - math.sin(alpha) + math.sin(beta)) / 6
        + math.cos(alpha) / 3
        - math.cos(beta) / 3
    )
    if abs(tmp0) > 1:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp0))
    t = _mod2pi(
        alpha
        - math.atan2(
            math.cos(alpha) - math.cos(beta),
            dx - math.sin(alpha) + math.sin(beta),
        )
        + p / 2
    )
    q = _mod2pi(alpha - beta - t + p)
    return DubinsPath('RLR', t * r, p * r, q * r, (t + p + q) * r)


def dubins_lrl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
    dx = (q2.x - q1.x) / r
    dy = (q2.y - q1.y) / r
    alpha = math.radians(q1.theta)
    beta = math.radians(q2.theta)
    tmp0 = (
        (dx + math.sin(alpha) - math.sin(beta)) / 6
        - math.cos(alpha) / 3
        + math.cos(beta) / 3
    )
    if abs(tmp0) > 1:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp0))
    t = _mod2pi(
        -alpha
        + math.atan2(
            -math.cos(alpha) + math.cos(beta),
            dx + math.sin(alpha) - math.sin(beta),
        )
        + p / 2
    )
    q = _mod2pi(beta - alpha - t + p)
    return DubinsPath('LRL', t * r, p * r, q * r, (t + p + q) * r)


def dubins_optimal(q1: RobotState, q2: RobotState, r: float) -> DubinsPath:
    candidates = [
        dubins_lsl(q1, q2, r),
        dubins_rsr(q1, q2, r),
        dubins_lsr(q1, q2, r),
        dubins_rsl(q1, q2, r),
        dubins_rlr(q1, q2, r),
        dubins_lrl(q1, q2, r),
    ]
    return min((c for c in candidates if c is not None), key=lambda p: p.total)
