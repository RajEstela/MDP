"""Geometry for repositioning the robot to an adjacent obstacle face.

Constants mirror Algorithm/simulator/config.py's CELL_CM / APPROACH_CM so
both parts of the project agree on the real rig's obstacle size and
standoff distance. Not imported directly (separate Python environments -
this runs on the Mac alongside pc_infer_server.py) - keep the two files'
values in sync by hand if the rig's dimensions change.
"""

OBSTACLE_HALF_WIDTH_CM = 5   # half of Algorithm/simulator/config.py's CELL_CM (10cm obstacle)
APPROACH_CM = 20             # matches Algorithm/simulator/config.py's APPROACH_CM
DEFAULT_FACE_STEP_CM = APPROACH_CM + OBSTACLE_HALF_WIDTH_CM  # 25
MAX_FACE_ATTEMPTS = 3        # a square obstacle has 3 other faces besides the current one


def build_face_change_commands(face_step_cm: int) -> list[str]:
    """Return the 5-command sequence that walks the robot counter-clockwise
    around a square obstacle's corner to the next face: turn away from the
    corner, drive past it, turn back toward the obstacle, close the
    remaining distance, then square up to face the new face directly.

    Every step is relative to the robot's current heading, so this same
    sequence works from any starting face on a square obstacle.
    """
    step = f"FW{face_step_cm:03d}"
    return ["RR090", step, "RL090", step, "RL090"]
