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


def _window(lo, hi, origin, resolution, n):
    """Return the (start, stop) index range of cells whose centers may lie in
    [lo, hi] along one axis, clipped to [0, n). None if the window is empty.
    """
    first = max(0.0, np.floor((lo - origin) / resolution - 0.5))
    last = min(n - 1.0, np.ceil((hi - origin) / resolution - 0.5))
    if first > last:
        return None
    return int(first), int(last) + 1


def rasterize_capsule(mask, spec, p0, p1, radius, value=KEEPOUT):
    """Set cells whose center is within radius of the segment p0-p1.

    p0 and p1 are (x, y) in meters and the ends are rounded. When p0 == p1
    this is a disc. Radius <= 0 sets nothing. Returns the number of cells
    inside the capsule. Only the capsule's bounding box is evaluated.
    """
    if radius <= 0:
        return 0
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    cols = _window(min(x0, x1) - radius, max(x0, x1) + radius,
                   spec.origin_x, spec.resolution, spec.width)
    rows = _window(min(y0, y1) - radius, max(y0, y1) + radius,
                   spec.origin_y, spec.resolution, spec.height)
    if cols is None or rows is None:
        return 0
    c0, c1 = cols
    r0, r1 = rows
    xs = spec.origin_x + (np.arange(c0, c1) + 0.5) * spec.resolution
    ys = spec.origin_y + (np.arange(r0, r1) + 0.5) * spec.resolution
    gx, gy = np.meshgrid(xs, ys)
    dx = x1 - x0
    dy = y1 - y0
    length2 = dx * dx + dy * dy
    if length2 > 0:
        t = np.clip(((gx - x0) * dx + (gy - y0) * dy) / length2, 0.0, 1.0)
    else:
        t = 0.0
    px = x0 + t * dx
    py = y0 + t * dy
    inside = (gx - px) ** 2 + (gy - py) ** 2 <= radius ** 2
    mask[r0:r1, c0:c1][inside] = value
    return int(inside.sum())


def rasterize_stroke(mask, spec, points, dilation=0.0, value=KEEPOUT):
    """Paint a brush stroke given as (x, y, r) points, r = brush radius (m).

    The effective radius at each point is max(r, 0) + dilation. A single
    point is a disc; consecutive points are joined by capsules whose radius
    is the larger of their two effective radii. Returns the number of cells
    equal to value after the call that were not equal to it before.
    """
    if len(points) == 0:
        return 0
    radii = [max(float(r), 0.0) + dilation for _, _, r in points]
    before = int(np.count_nonzero(mask == value))
    if len(points) == 1:
        x, y, _ = points[0]
        rasterize_capsule(mask, spec, (x, y), (x, y), radii[0], value)
    for i in range(len(points) - 1):
        xa, ya, _ = points[i]
        xb, yb, _ = points[i + 1]
        rasterize_capsule(mask, spec, (xa, ya), (xb, yb),
                          max(radii[i], radii[i + 1]), value)
    return int(np.count_nonzero(mask == value)) - before


def to_occupancy_data(mask):
    """Row-major flatten (row 0 = lowest y) as OccupancyGrid.data expects."""
    return [int(v) for v in np.asarray(mask, dtype=np.int8).ravel()]
