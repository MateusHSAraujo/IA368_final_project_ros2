"""Pure-logic helpers for the CoppeliaSim <-> ROS 2 bridge (no ROS imports)."""
from __future__ import annotations

import math

import cv2
import numpy as np

WHEEL_RADIUS = 0.0975
WHEEL_BASE = 0.331
SCAN_SAMPLES = 452
SCAN_ANGLE_MIN = -math.pi / 2
SCAN_ANGLE_INCREMENT = math.pi / 450
SCAN_RANGE_MIN = 0.055
SCAN_RANGE_MAX = 10.0
LASER_OFFSET = (0.0778, 0.0, 0.2498)  # base_link -> laser_link translation
LASER_QUATERNION = (3e-05, 0.00164, 9e-05, 1.0)  # base_link -> laser_link (x, y, z, w)


def wrap_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    r = math.fmod(a + math.pi, 2.0 * math.pi)
    if r <= 0.0:
        r += 2.0 * math.pi
    return r - math.pi


def diff_drive_ik(v: float, w: float, wheel_radius: float = WHEEL_RADIUS,
                  wheel_base: float = WHEEL_BASE) -> tuple[float, float]:
    """Return (left, right) wheel velocities in rad/s for body twist (v, w)."""
    half = wheel_base / 2.0 * w
    return ((v - half) / wheel_radius, (v + half) / wheel_radius)


def scan_angle_max(samples: int = SCAN_SAMPLES, angle_min: float = SCAN_ANGLE_MIN,
                   increment: float = SCAN_ANGLE_INCREMENT) -> float:
    """Angle of the last laser sample."""
    return angle_min + (samples - 1) * increment


def convert_ranges(raw, range_max: float = SCAN_RANGE_MAX) -> list[float]:
    """Convert CoppeliaSim ranges to ROS convention (inf for miss/invalid)."""
    out: list[float] = []
    for r in raw:
        r = float(r)
        if math.isnan(r) or r <= 0.0 or r > range_max:
            out.append(float("inf"))
        else:
            out.append(r)
    return out


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about Z) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) for a pure yaw rotation."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def normalize_quaternion(q) -> tuple[float, float, float, float]:
    """Return the unit quaternion; identity if the norm is zero."""
    n = math.sqrt(sum(float(c) * float(c) for c in q))
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(float(c) / n for c in q)  # type: ignore[return-value]


def world_twist_to_body(vx_world: float, vy_world: float, wz: float,
                        yaw: float) -> tuple[float, float, float]:
    """Rotate a world-frame planar velocity into the robot frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * vx_world + s * vy_world, -s * vx_world + c * vy_world, wz)


def integrate_unicycle(x: float, y: float, yaw: float, v: float, w: float,
                       dt: float) -> tuple[float, float, float]:
    """Exact unicycle integration over dt; yaw wrapped to (-pi, pi]."""
    if abs(w) < 1e-9:
        return (x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), wrap_angle(yaw))
    new_yaw = yaw + w * dt
    x += v / w * (math.sin(new_yaw) - math.sin(yaw))
    y -= v / w * (math.cos(new_yaw) - math.cos(yaw))
    return (x, y, wrap_angle(new_yaw))


class MockWorld:
    """2D world for mock mode: a rectangular room plus axis-aligned boxes."""

    def __init__(self, x_min=-3.0, x_max=3.0, y_min=-3.0, y_max=3.0,
                 boxes=((1.5, 1.0, 0.6, 0.6), (-1.2, -1.5, 0.8, 0.4))):
        self.x_min, self.x_max, self.y_min, self.y_max = x_min, x_max, y_min, y_max
        self.boxes = tuple(tuple(b) for b in boxes)

    @staticmethod
    def _slab(x, y, dx, dy, bx0, bx1, by0, by1):
        """Entry distance of a ray into a box, or None. Ray starts outside."""
        t0, t1 = 0.0, math.inf
        for o, d, lo, hi in ((x, dx, bx0, bx1), (y, dy, by0, by1)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    return None
                continue
            ta, tb = (lo - o) / d, (hi - o) / d
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t0 > t1:
                return None
        return t0

    def raycast(self, x: float, y: float, angle: float,
                max_range: float = SCAN_RANGE_MAX) -> float:
        """Distance to the first wall/box hit, or inf if beyond max_range."""
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        # Walls: exit distance from the room interior.
        for o, d, lo, hi in ((x, dx, self.x_min, self.x_max),
                             (y, dy, self.y_min, self.y_max)):
            if d > 1e-12:
                best = min(best, (hi - o) / d)
            elif d < -1e-12:
                best = min(best, (lo - o) / d)
        for cx, cy, sx, sy in self.boxes:
            hx, hy = sx / 2.0, sy / 2.0
            if cx - hx < x < cx + hx and cy - hy < y < cy + hy:
                continue  # origin inside a box: ignore it
            t = self._slab(x, y, dx, dy, cx - hx, cx + hx, cy - hy, cy + hy)
            if t is not None:
                best = min(best, t)
        best = max(best, 0.0)
        return best if best <= max_range else math.inf

    def scan(self, x: float, y: float, yaw: float,
             laser_offset_x: float = LASER_OFFSET[0], samples: int = SCAN_SAMPLES,
             angle_min: float = SCAN_ANGLE_MIN, increment: float = SCAN_ANGLE_INCREMENT,
             range_max: float = SCAN_RANGE_MAX) -> list[float]:
        """Simulated laser ranges (inf for no hit)."""
        lx = x + laser_offset_x * math.cos(yaw)
        ly = y + laser_offset_x * math.sin(yaw)
        return [self.raycast(lx, ly, yaw + angle_min + i * increment, range_max)
                for i in range(samples)]

    def is_free(self, x: float, y: float, radius: float = 0.3) -> bool:
        """True if a circle of the given radius is inside the room and clear of boxes."""
        if (x - radius < self.x_min or x + radius > self.x_max
                or y - radius < self.y_min or y + radius > self.y_max):
            return False
        for cx, cy, sx, sy in self.boxes:
            nx = min(max(x, cx - sx / 2.0), cx + sx / 2.0)
            ny = min(max(y, cy - sy / 2.0), cy + sy / 2.0)
            if math.hypot(x - nx, y - ny) < radius:
                return False
        return True


def flip_rows(rgb, width: int, height: int) -> np.ndarray:
    """Turn a bottom-up RGB buffer into an upright HxWx3 uint8 image."""
    arr = np.frombuffer(bytes(rgb), dtype=np.uint8)
    if arr.size != width * height * 3:
        raise ValueError(f"buffer has {arr.size} bytes, expected {width * height * 3}")
    return np.ascontiguousarray(arr.reshape(height, width, 3)[::-1])


def encode_jpeg(image_rgb: np.ndarray, quality: int = 70) -> bytes:
    """JPEG-encode an HxWx3 RGB uint8 image."""
    bgr = cv2.cvtColor(np.ascontiguousarray(image_rgb), cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return buf.tobytes()


def mock_camera_frame(width: int = 320, height: int = 240, t: float = 0.0) -> bytes:
    """Synthetic bottom-up RGB buffer (sky/floor, moving bar, red top-left patch)."""
    img = np.empty((height, width, 3), dtype=np.uint8)  # upright first
    horizon = height // 2
    img[:horizon] = (135, 206, 235)  # sky
    img[horizon:] = (90, 90, 90)     # floor
    bar_w = max(width // 16, 2)
    x0 = int(((math.sin(t) + 1.0) / 2.0) * (width - bar_w))
    img[:, x0:x0 + bar_w] = (255, 255, 0)
    ph, pw = max(height // 8, 2), max(width // 8, 2)
    img[:ph, :pw] = (255, 0, 0)  # top-left patch (drawn last)
    return img[::-1].tobytes()  # bottom-up
