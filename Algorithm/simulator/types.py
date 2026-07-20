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
    id: str | None = None  # arena-supplied obstacle ID (e.g. 'B1'); None for local/demo obstacles


@dataclass
class Command:
    kind: str    # 'FW' | 'BW' | 'RL' | 'RR' | 'WAIT'
    value: float # cm for FW/BW; degrees for RL/RR; frames for WAIT
    obstacle_id: str | None = None  # set on the WAIT after a leg: which obstacle was just reached


@dataclass
class DubinsPath:
    path_type: str  # 'LSL' | 'LSR' | 'RSL' | 'RSR' | 'RLR' | 'LRL'
    seg1: float     # first segment length in cm
    seg2: float     # second segment length in cm
    seg3: float     # third segment length in cm
    total: float    # seg1 + seg2 + seg3
