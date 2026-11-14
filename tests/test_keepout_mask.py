import math
import os
import sys
import time

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


# ---------------------------------------------------------------- ROS part
rclpy = pytest.importorskip('rclpy')
try:
    from geometry_msgs.msg import Point32, PolygonStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from vr_robot_pkg.keepout_mask_node import KeepoutMaskNode
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

ros_only = pytest.mark.skipif(not HAVE_ROS, reason='ROS messages unavailable')


def _spin_until(ex, cond, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end and not cond():
        ex.spin_once(timeout_sec=0.05)
    return cond()


@ros_only
def test_node_publishes_and_updates():
    rclpy.init()
    ex = SingleThreadedExecutor()
    node = KeepoutMaskNode()
    helper = rclpy.create_node('keepout_test_helper_%d' % os.getpid())
    try:
        ex.add_node(node)
        ex.add_node(helper)
        # Let the node publish before the subscriber exists (late subscriber).
        for _ in range(5):
            ex.spin_once(timeout_sec=0.05)

        got = []
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        helper.create_subscription(
            OccupancyGrid, '/keepout_filter_mask', got.append, qos)
        pub = helper.create_publisher(
            PolygonStamped, '/keepout_mask_test', 10)

        assert _spin_until(ex, lambda: len(got) >= 1)
        g = got[-1]
        assert g.info.width == 310 and g.info.height == 210
        assert g.info.resolution == pytest.approx(0.05)
        assert g.header.frame_id == 'map'
        assert len(g.data) == 310 * 210
        assert all(v == 0 for v in g.data)

        # Wait for node to see the publisher's subscription.
        assert _spin_until(ex, lambda: pub.get_subscription_count() > 0)

        n_before = len(got)
        sq = PolygonStamped()
        sq.header.frame_id = 'map'
        sq.polygon.points = [Point32(x=float(x), y=float(y), z=0.0)
                             for x, y in [(0, 0), (1, 0), (1, 1), (0, 1)]]
        pub.publish(sq)
        assert _spin_until(ex, lambda: len(got) > n_before)
        assert sum(1 for v in got[-1].data if v == 100) == 400

        n_before = len(got)
        empty = PolygonStamped()
        empty.header.frame_id = 'map'
        pub.publish(empty)
        assert _spin_until(ex, lambda: len(got) > n_before)
        assert all(v == 0 for v in got[-1].data)
    finally:
        ex.shutdown()
        node.destroy_node()
        helper.destroy_node()
        rclpy.shutdown()
