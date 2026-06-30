from simulator.config import ARENA_PX, CELL_CM, CELL_PX


def cm_to_px(x_cm: float, y_cm: float) -> tuple[int, int]:
    px = int(x_cm * CELL_PX / CELL_CM)
    py = int(ARENA_PX - y_cm * CELL_PX / CELL_CM)
    return px, py
