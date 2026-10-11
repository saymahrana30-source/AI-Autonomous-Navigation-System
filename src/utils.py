"""Small helpers shared across modules."""
import math
import cv2
import numpy as np


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def inflate(occ: np.ndarray, cells: int) -> np.ndarray:
    """Grow obstacles by `cells` so the robot's body never touches them."""
    if cells <= 0:
        return occ.copy()
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * cells + 1, 2 * cells + 1))
    return cv2.dilate(occ.astype(np.uint8), k).astype(bool)


def to_cell(x: float, y: float) -> tuple:
    """World (x, y) -> grid (row, col)."""
    return int(math.floor(y)), int(math.floor(x))


def cell_center(cell: tuple) -> tuple:
    """Grid (row, col) -> world (x, y) of the cell centre."""
    return cell[1] + 0.5, cell[0] + 0.5
