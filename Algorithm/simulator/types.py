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
    kind: str    # 'FW' | 'BW' | 'TL' | 'TR' | 'AL' | 'AR'
    value: float # cm for FW/BW/AL/AR; degrees for TL/TR


@dataclass
class DubinsPath:
    path_type: str  # 'LSL' | 'LSR' | 'RSL' | 'RSR' | 'RLR' | 'LRL'
    seg1: float     # first segment length in cm
    seg2: float     # second segment length in cm
    seg3: float     # third segment length in cm
    total: float    # seg1 + seg2 + seg3
