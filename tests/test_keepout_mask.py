import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2_ws', 'src', 'vr_robot_pkg'))

import numpy as np  # noqa: E402
import pytest  # noqa: E402

from vr_robot_pkg import keepout_mask as km  # noqa: E402

SPEC = km.GridSpec()


def test_empty_mask():
    m = km.empty_mask(SPEC)
    assert m.shape == (210, 310)
    assert m.dtype == np.int8
    assert not m.any()


def test_world_to_cell():
    assert km.world_to_cell(SPEC, -2.5, -4.25) == (0, 0)
    assert km.world_to_cell(SPEC, 12.99, 6.24) == (309, 209)
    assert km.world_to_cell(SPEC, -2.6, 0.0) is None
    assert km.world_to_cell(SPEC, 13.01, 0.0) is None
    assert km.world_to_cell(SPEC, 0.0, 6.3) is None


def test_cell_center_inverse():
    for col, row in [(0, 0), (10, 20), (309, 209)]:
        x, y = km.cell_center(SPEC, col, row)
        assert km.world_to_cell(SPEC, x, y) == (col, row)


def test_polygon_square():
    m = km.empty_mask(SPEC)
    n = km.rasterize_polygon(m, SPEC, [(0, 0), (1, 0), (1, 1), (0, 1)])
    assert n == 400
    assert int((m == 100).sum()) == 400
    c0, r0 = km.world_to_cell(SPEC, 0.01, 0.01)
    block = m[r0:r0 + 20, c0:c0 + 20]
    assert (block == 100).all()
    assert (m == 100).sum() == block.size


def test_polygon_triangle():
    m = km.empty_mask(SPEC)
    n = km.rasterize_polygon(m, SPEC, [(0, 0), (2, 0), (0, 2)])
    assert abs(n - 2.0 / 0.0025) / (2.0 / 0.0025) < 0.05


def test_polygon_too_few_points():
    m = km.empty_mask(SPEC)
    assert km.rasterize_polygon(m, SPEC, [(0, 0), (1, 1)]) == 0
    assert not m.any()


def test_disc():
    m = km.empty_mask(SPEC)
    n = km.rasterize_disc(m, SPEC, 5.0, 0.0, 0.5)
    expected = math.pi * 0.25 / 0.0025
    assert abs(n - expected) / expected < 0.05
    assert int((m == 100).sum()) == n


def test_occupancy_data():
    m = km.empty_mask(SPEC)
    m[0, 3] = 100
    m[1, 0] = 100
    data = km.to_occupancy_data(m)
    assert len(data) == 310 * 210
    assert data[3] == 100
    assert data[310] == 100
    assert sum(data) == 200


def _cell_value(m, x, y):
    col, row = km.world_to_cell(SPEC, x, y)
    return m[row, col]


def _brute_stroke(points, dilation):
    """Reference stroke on the full grid, without bounding-box windows."""
    xs = SPEC.origin_x + (np.arange(SPEC.width) + 0.5) * SPEC.resolution
    ys = SPEC.origin_y + (np.arange(SPEC.height) + 0.5) * SPEC.resolution
    gx, gy = np.meshgrid(xs, ys)
    radii = [max(float(r), 0.0) + dilation for _, _, r in points]
    if len(points) == 1:
        segments = [(points[0], points[0], radii[0])]
    else:
        segments = [(points[i], points[i + 1], max(radii[i], radii[i + 1]))
                    for i in range(len(points) - 1)]
    hit = np.zeros(gx.shape, dtype=bool)
    for (ax, ay, _), (bx, by, _), r in segments:
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        if length2 > 0:
            t = np.clip(((gx - ax) * dx + (gy - ay) * dy) / length2, 0.0, 1.0)
        else:
            t = 0.0
        px = ax + t * dx
        py = ay + t * dy
        hit |= (gx - px) ** 2 + (gy - py) ** 2 <= r ** 2
    out = np.zeros(gx.shape, dtype=np.int8)
    out[hit] = 100
    return out


def test_capsule_horizontal_segment():
    m = km.empty_mask(SPEC)
    n = km.rasterize_capsule(m, SPEC, (0.0, 0.0), (2.0, 0.0), 0.3)
    expected = (2.0 * 0.3 * 2 + math.pi * 0.3 ** 2) / 0.0025
    assert abs(n - expected) / expected < 0.05
    assert int((m == 100).sum()) == n
    # Middle of the segment and points well inside the radius are set.
    assert _cell_value(m, 1.0, 0.0) == 100
    assert _cell_value(m, 1.0, 0.2) == 100
    # Points beyond the radius are not set.
    assert _cell_value(m, 1.0, 0.4) == 0
    assert _cell_value(m, 2.6, 0.0) == 0
    # Just past an endpoint, but within the radius, is set (rounded end).
    assert _cell_value(m, 2.2, 0.0) == 100
    assert _cell_value(m, -0.2, 0.0) == 100
    # A corner-like point at the end is outside a round cap.
    assert _cell_value(m, -0.25, 0.25) == 0


def test_capsule_degenerate_equals_disc():
    m1 = km.empty_mask(SPEC)
    m2 = km.empty_mask(SPEC)
    n1 = km.rasterize_capsule(m1, SPEC, (5.0, 0.0), (5.0, 0.0), 0.5)
    n2 = km.rasterize_disc(m2, SPEC, 5.0, 0.0, 0.5)
    assert n1 == n2 > 0
    assert np.array_equal(m1, m2)


def test_capsule_zero_radius_sets_nothing():
    m = km.empty_mask(SPEC)
    assert km.rasterize_capsule(m, SPEC, (0.0, 0.0), (1.0, 0.0), 0.0) == 0
    assert km.rasterize_capsule(m, SPEC, (0.0, 0.0), (1.0, 0.0), -0.5) == 0
    assert not m.any()


def test_stroke_single_point_equals_disc():
    m1 = km.empty_mask(SPEC)
    m2 = km.empty_mask(SPEC)
    n = km.rasterize_stroke(m1, SPEC, [(5.0, 0.0, 0.3)], dilation=0.1)
    km.rasterize_disc(m2, SPEC, 5.0, 0.0, 0.4)
    assert n == int((m2 == 100).sum()) > 0
    assert np.array_equal(m1, m2)


def test_stroke_dilation_covers_more_cells():
    pts = [(0.0, 0.0, 0.2), (1.0, 0.0, 0.2), (1.0, 1.0, 0.2)]
    m1 = km.empty_mask(SPEC)
    m2 = km.empty_mask(SPEC)
    n1 = km.rasterize_stroke(m1, SPEC, pts)
    n2 = km.rasterize_stroke(m2, SPEC, pts, dilation=0.1)
    assert n2 > n1 > 0
    assert (m1 <= m2).all()
    assert int((m2 == 100).sum()) > int((m1 == 100).sum())


def test_stroke_l_shape_covers_corner_and_legs():
    m = km.empty_mask(SPEC)
    km.rasterize_stroke(m, SPEC, [(0.0, 0.0, 0.1), (2.0, 0.0, 0.1),
                                  (2.0, 2.0, 0.1)])
    assert _cell_value(m, 2.0, 0.0) == 100  # corner
    assert _cell_value(m, 1.0, 0.0) == 100  # horizontal leg
    assert _cell_value(m, 2.0, 1.0) == 100  # vertical leg
    assert _cell_value(m, 1.0, 1.0) == 0    # inside the L, away from strokes


def test_stroke_empty_points():
    m = km.empty_mask(SPEC)
    assert km.rasterize_stroke(m, SPEC, []) == 0
    assert not m.any()


def test_stroke_fully_outside_grid():
    m = km.empty_mask(SPEC)
    assert km.rasterize_stroke(m, SPEC, [(100.0, 100.0, 0.5),
                                         (101.0, 100.0, 0.5)]) == 0
    assert km.rasterize_stroke(m, SPEC, [(-50.0, 0.0, 0.5)],
                               dilation=0.2) == 0
    assert not m.any()


def test_stroke_count_is_newly_set_cells():
    pts = [(-1.0, -1.0, 0.2), (0.0, 0.0, 0.3), (1.5, 0.5, 0.1)]
    m = km.empty_mask(SPEC)
    first = km.rasterize_stroke(m, SPEC, pts)
    assert first > 0
    assert first == int((m == 100).sum())
    assert km.rasterize_stroke(m, SPEC, pts) == 0


def test_stroke_bbox_matches_brute_force():
    rng = np.random.default_rng(7)
    n = 60
    xs = np.linspace(-2.4, 12.9, n) + rng.normal(0.0, 0.15, n)
    ys = np.linspace(-4.2, 6.2, n) + rng.normal(0.0, 0.15, n)
    rs = rng.uniform(0.05, 0.4, n)
    pts = list(zip(xs.tolist(), ys.tolist(), rs.tolist()))
    m = km.empty_mask(SPEC)
    km.rasterize_stroke(m, SPEC, pts, dilation=0.05)
    assert np.array_equal(m, _brute_stroke(pts, 0.05))
