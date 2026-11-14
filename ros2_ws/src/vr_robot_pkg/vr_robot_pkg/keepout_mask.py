"""Pure helpers to build the Nav2 keepout filter mask (no ROS imports).

The mask is a (height, width) int8 array; row index = y cell, column = x cell.
Cell value 100 is keepout (lethal), 0 is free.
"""
from dataclasses import dataclass

import numpy as np

FREE = 0
KEEPOUT = 100


@dataclass(frozen=True)
class GridSpec:
    resolution: float = 0.05
    origin_x: float = -2.5
    origin_y: float = -4.25
    width: int = 310
    height: int = 210


def empty_mask(spec):
    """Return an all-free mask of shape (height, width), dtype int8."""
    return np.zeros((spec.height, spec.width), dtype=np.int8)


def world_to_cell(spec, x, y):
    """Return (col, row) containing world point (x, y), or None if outside."""
    col = int(np.floor((x - spec.origin_x) / spec.resolution))
    row = int(np.floor((y - spec.origin_y) / spec.resolution))
    if 0 <= col < spec.width and 0 <= row < spec.height:
        return col, row
    return None


def cell_center(spec, col, row):
    """Return the world coordinates of the center of cell (col, row)."""
    return (spec.origin_x + (col + 0.5) * spec.resolution,
            spec.origin_y + (row + 0.5) * spec.resolution)


def _centers(spec):
    xs = spec.origin_x + (np.arange(spec.width) + 0.5) * spec.resolution
    ys = spec.origin_y + (np.arange(spec.height) + 0.5) * spec.resolution
    return np.meshgrid(xs, ys)  # each (height, width)


def rasterize_polygon(mask, spec, points, value=KEEPOUT):
    """Set cells whose center is inside the polygon (even-odd rule).

    Returns the number of cells set. Fewer than 3 points sets nothing.
    """
    if points is None or len(points) < 3:
        return 0
    gx, gy = _centers(spec)
    inside = np.zeros(gx.shape, dtype=bool)
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        if y1 == y2:
            continue
        crosses = (y1 > gy) != (y2 > gy)
        x_int = x1 + (gy - y1) * (x2 - x1) / (y2 - y1)
        inside ^= crosses & (gx < x_int)
    mask[inside] = value
    return int(inside.sum())


def rasterize_disc(mask, spec, cx, cy, radius, value=KEEPOUT):
    """Set cells whose center is within radius of (cx, cy); returns count."""
    gx, gy = _centers(spec)
    inside = (gx - cx) ** 2 + (gy - cy) ** 2 <= radius ** 2
    mask[inside] = value
    return int(inside.sum())


def to_occupancy_data(mask):
    """Row-major flatten (row 0 = lowest y) as OccupancyGrid.data expects."""
    return [int(v) for v in np.asarray(mask, dtype=np.int8).ravel()]
