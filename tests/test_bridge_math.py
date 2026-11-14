import math

import cv2
import numpy as np
import pytest

from vr_robot_pkg import bridge_math as bm


def test_ik_forward():
    l, r = bm.diff_drive_ik(0.5, 0.0)
    assert l == pytest.approx(0.5 / bm.WHEEL_RADIUS)
    assert r == pytest.approx(l)


def test_ik_rotation_and_order():
    l, r = bm.diff_drive_ik(0.0, 1.0)
    mag = bm.WHEEL_BASE / 2 / bm.WHEEL_RADIUS
    assert r == pytest.approx(mag)
    assert l == pytest.approx(-mag)  # CCW: right wheel forward


def test_scan_angle_max_and_count():
    assert bm.scan_angle_max() == pytest.approx(math.pi / 2 + math.pi / 450)
    assert len(bm.MockWorld().scan(0, 0, 0)) == bm.SCAN_SAMPLES == 452


def test_convert_ranges():
    out = bm.convert_ranges([11.0, float("nan"), 0.0, 3.2, -1.0, 10.0])
    assert out[:3] == [math.inf] * 3
    assert out[3] == 3.2 and out[4] == math.inf and out[5] == 10.0
    assert all(isinstance(v, float) for v in bm.convert_ranges([1, 2]))


@pytest.mark.parametrize("yaw", [0.0, 0.5, -1.2, 2.5, -3.0, math.pi - 1e-6, -math.pi + 1e-6])
def test_yaw_roundtrip(yaw):
    assert bm.yaw_from_quaternion(*bm.quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_yaw_pi_edge():
    y = bm.yaw_from_quaternion(*bm.quaternion_from_yaw(math.pi))
    assert abs(abs(y) - math.pi) < 1e-9


def test_normalize_quaternion():
    q = bm.normalize_quaternion(bm.LASER_QUATERNION)
    assert math.sqrt(sum(c * c for c in q)) == pytest.approx(1.0)


def test_world_twist_to_body():
    vx, vy, wz = bm.world_twist_to_body(0.0, 1.0, 0.3, math.pi / 2)
    assert (vx, vy, wz) == pytest.approx((1.0, 0.0, 0.3))


def test_integrate_straight():
    assert bm.integrate_unicycle(0, 0, math.pi / 2, 1.0, 0.0, 2.0) == pytest.approx((0, 2.0, math.pi / 2))


def test_integrate_full_circle():
    x, y, yaw = bm.integrate_unicycle(1, 2, 0.3, 0.5, 1.0, 2 * math.pi)
    assert (x, y) == pytest.approx((1, 2), abs=1e-9)
    assert yaw == pytest.approx(0.3)


def test_integrate_quarter_circle():
    # v=1, w=1, t=pi/2 from origin heading +x: radius 1, ends at (1, 1) heading +y
    x, y, yaw = bm.integrate_unicycle(0, 0, 0, 1.0, 1.0, math.pi / 2)
    assert (x, y, yaw) == pytest.approx((1.0, 1.0, math.pi / 2))


def test_wrap_angle():
    assert bm.wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert bm.wrap_angle(-math.pi) == pytest.approx(math.pi)
    assert bm.wrap_angle(0.1) == pytest.approx(0.1)


def test_raycast_walls():
    w = bm.MockWorld()
    assert w.raycast(0, 0, math.pi / 2) == pytest.approx(3.0)
    assert w.raycast(0, 0, -math.pi / 2) == pytest.approx(3.0)
    assert w.raycast(0, 0, math.pi) == pytest.approx(3.0)
    assert w.raycast(0, 0, 0) == pytest.approx(3.0)  # y=0 misses both boxes


def test_raycast_box_face():
    w = bm.MockWorld()
    # box 1 spans x 1.2..1.8, y 0.7..1.3; ray along +x at y=1.0 hits x=1.2
    assert w.raycast(0, 1.0, 0.0) == pytest.approx(1.2)
    assert w.raycast(0, 1.0, 0.0, max_range=1.0) == math.inf


def test_scan_range_limit():
    s = bm.MockWorld().scan(0, 0, 0, range_max=0.5)
    assert len(s) == 452 and all(v == math.inf for v in s)


def test_is_free():
    w = bm.MockWorld()
    assert w.is_free(0, 0)
    assert not w.is_free(2.9, 0)
    assert not w.is_free(1.5, 1.0)
    assert not w.is_free(1.5, 1.45)  # within 0.3 of box face at y=1.3
    assert w.is_free(1.5, 1.7)


def test_flip_rows():
    w, h = 4, 3
    buf = bytearray(w * h * 3)
    buf[0:3] = bytes((10, 20, 30))  # first buffer row, first pixel
    img = bm.flip_rows(buf, w, h)
    assert img.shape == (h, w, 3) and img.dtype == np.uint8
    assert tuple(img[h - 1, 0]) == (10, 20, 30)
    assert tuple(img[0, 0]) == (0, 0, 0)


def test_encode_jpeg():
    img = bm.flip_rows(bm.mock_camera_frame(64, 48), 64, 48)
    data = bm.encode_jpeg(img)
    dec = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    assert dec.shape == img.shape


def test_mock_camera_orientation():
    w, h = 320, 240
    img = bm.flip_rows(bm.mock_camera_frame(w, h, t=1.0), w, h)
    assert tuple(img[0, 0]) == (255, 0, 0)
    assert tuple(img[h - 1, 0]) != (255, 0, 0)
    assert tuple(img[5, w - 1]) == (135, 206, 235)  # sky on top
    assert tuple(img[h - 5, w - 1]) == (90, 90, 90)  # floor on bottom
