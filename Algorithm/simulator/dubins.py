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
    theta = math.atan2(tmp1, tmp0) + math.atan2(2, p)
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
    theta = math.atan2(tmp1, tmp0) - math.atan2(2, p)
    t = _mod2pi(alpha - theta)
    q = _mod2pi(beta - theta)
    return DubinsPath('RSL', t * r, p * r, q * r, (t + p + q) * r)


def dubins_rlr(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
    dx = (q2.x - q1.x) / r
    dy = (q2.y - q1.y) / r
    alpha = math.radians(q1.theta)
    beta = math.radians(q2.theta)
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta), math.cos(beta)
    # Center-to-center vector between the two R (right/CW) turning circles
    dx_c = dx - sa + sb
    dy_c = dy + ca - cb
    d_sq = dx_c ** 2 + dy_c ** 2
    # Middle L circle exists iff centre-to-centre distance <= 4r (normalised: <= 4)
    if d_sq > 16.0:
        return None
    # cos(p) = 1 - d_sq/8  (from law of cosines on the C1-M-C2 isoceles triangle)
    tmp = 1.0 - d_sq / 8.0
    tmp = max(-1.0, min(1.0, tmp))
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(alpha - math.atan2(dy_c, dx_c) + p / 2)
    q = _mod2pi(alpha - beta - t + p)
    return DubinsPath('RLR', t * r, p * r, q * r, (t + p + q) * r)


def dubins_lrl(q1: RobotState, q2: RobotState, r: float) -> DubinsPath | None:
    dx = (q2.x - q1.x) / r
    dy = (q2.y - q1.y) / r
    alpha = math.radians(q1.theta)
    beta = math.radians(q2.theta)
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta), math.cos(beta)
    # Center-to-center vector between the two L (left/CCW) turning circles
    dx_c = dx + sa - sb
    dy_c = dy - ca + cb
    d_sq = dx_c ** 2 + dy_c ** 2
    # Middle R circle exists iff centre-to-centre distance <= 4r (normalised: <= 4)
    if d_sq > 16.0:
        return None
    # cos(p) = 1 - d_sq/8  (from law of cosines on the C1-M-C2 isoceles triangle)
    tmp = 1.0 - d_sq / 8.0
    tmp = max(-1.0, min(1.0, tmp))
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(-alpha + math.atan2(dy_c, dx_c) + p / 2)
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
