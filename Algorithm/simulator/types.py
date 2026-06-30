from dataclasses import dataclass


@dataclass
class RobotState:
    x: float
    y: float
    theta: float  # degrees; 0=East, 90=North (standard math convention, CCW positive)


@dataclass
class Obstacle:
    x: int
    y: int
    face: str  # 'N' | 'S' | 'E' | 'W' — which face carries the target image


@dataclass
class Command:
    kind: str    # 'FW' | 'BW' | 'TL' | 'TR'
    value: float # cm for FW/BW; degrees for TL/TR
