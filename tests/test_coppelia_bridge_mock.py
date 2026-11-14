"""Mock-mode integration tests for the CoppeliaSim bridge node (no CoppeliaSim needed)."""
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
np = pytest.importorskip('numpy')
cv2 = pytest.importorskip('cv2')

try:
    from vr_robot_pkg.coppelia_bridge_node import CoppeliaBridgeNode
except Exception as exc:  # pragma: no cover
    pytest.skip(f'bridge node not importable: {exc}', allow_module_level=True)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from vr_robot_pkg import bridge_math as bm


class Harness:
    def __init__(self, params):
        args = ['--ros-args', '-p', 'mock_mode:=true']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args)
        self.bridge = None
        self.probe = None
        try:
            self.bridge = CoppeliaBridgeNode()
            self.probe = rclpy.create_node('probe')
            self.ex = SingleThreadedExecutor()
            self.ex.add_node(self.bridge)
            self.ex.add_node(self.probe)
            self.scans, self.odoms, self.images, self.poses, self.status = [], [], [], [], []
            pr = self.probe
            pr.create_subscription(LaserScan, '/myRobot/scan', self.scans.append, 10)
            pr.create_subscription(Odometry, '/myRobot/odom', self.odoms.append, 10)
            pr.create_subscription(CompressedImage, '/camera/image_raw/compressed',
                                   self.images.append, 10)
            pr.create_subscription(PoseStamped, '/hmi/robot_pose', self.poses.append, 10)
            latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            pr.create_subscription(String, '/myRobot/bridge_status', self.status.append, latched)
            self.cmd_pub = pr.create_publisher(Twist, '/cmd_vel', 10)
            self.cam_pub = pr.create_publisher(Bool, '/camera/enable', 10)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, pr)
        except Exception:
            self.close()
            raise

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.02)

    def spin_until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return predicate()

    def drive(self, v, w, seconds, rate=10.0):
        """Publish a Twist at `rate` Hz for `seconds`; return odom messages seen meanwhile."""
        start = len(self.odoms)
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        end = time.monotonic() + seconds
        next_pub = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_pub:
                self.cmd_pub.publish(msg)
                next_pub = time.monotonic() + 1.0 / rate
            self.ex.spin_once(timeout_sec=0.01)
        return self.odoms[start:]

    def close(self):
        try:
            if self.bridge is not None:
                self.bridge.shutdown_backend()
                self.bridge.destroy_node()
            if self.probe is not None:
                self.probe.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


@pytest.fixture
def h():
    harness = Harness([])
    try:
        yield harness
    finally:
        harness.close()


def yaw_of(odom):
    q = odom.pose.pose.orientation
    return bm.yaw_from_quaternion(q.x, q.y, q.z, q.w)


def test_scan(h):
    assert h.spin_until(lambda: h.scans, 3.0), 'no scan received'
    s = h.scans[-1]
    assert len(s.ranges) == 452
    assert s.header.frame_id == 'laser_link'
    assert s.angle_min == pytest.approx(-math.pi / 2)
    assert s.angle_increment == pytest.approx(math.pi / 450)
    assert s.angle_max == pytest.approx(s.angle_min + 451 * s.angle_increment)
    assert all(r > 0 for r in s.ranges)


def test_odom_and_tf(h):
    assert h.spin_until(lambda: h.odoms, 3.0), 'no odom received'
    o = h.odoms[-1]
    assert o.header.frame_id == 'odom'
    assert o.child_frame_id == 'base_link'

    def have_tf():
        return (h.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time())
                and h.tf_buffer.can_transform('base_link', 'laser_link', rclpy.time.Time())
                and h.tf_buffer.can_transform('base_link', 'camera_optical_frame',
                                              rclpy.time.Time()))
    assert h.spin_until(have_tf, 5.0), 'TF frames not available'
    t = h.tf_buffer.lookup_transform('base_link', 'laser_link', rclpy.time.Time())
    tr = t.transform.translation
    assert (tr.x, tr.y, tr.z) == pytest.approx((0.0778, 0.0, 0.2498), abs=1e-6)
    odom_tf = h.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
    assert odom_tf.header.frame_id == 'odom'
    assert odom_tf.child_frame_id == 'base_link'


def _stamp_rate(msgs):
    t = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs]
    return (len(t) - 1) / (t[-1] - t[0])


def test_rates(h):
    # Rates are computed from header stamps (publish time) so that probe-side
    # scheduling jitter does not make the test flaky.
    assert h.spin_until(lambda: h.odoms and h.scans, 3.0)
    h.spin_for(0.3)
    n_odom, n_scan = len(h.odoms), len(h.scans)
    h.spin_for(2.5)
    odom_hz = _stamp_rate(h.odoms[n_odom:])
    scan_hz = _stamp_rate(h.scans[n_scan:])
    assert 15.0 <= odom_hz <= 25.0, odom_hz
    assert 7.0 <= scan_hz <= 13.0, scan_hz


def test_cmd_vel_and_watchdog(h):
    assert h.spin_until(lambda: h.odoms, 3.0)
    h.spin_for(0.2)
    x0 = h.odoms[-1].pose.pose.position.x
    seen = h.drive(0.2, 0.0, 1.5)
    assert seen, 'no odom while driving'
    dx = h.odoms[-1].pose.pose.position.x - x0
    assert 0.15 <= dx <= 0.45, dx
    assert seen[-1].twist.twist.linear.x == pytest.approx(0.2, abs=0.05)

    # Stop publishing: watchdog (0.5 s) must halt the robot.
    h.spin_for(1.0)
    n = len(h.odoms)
    xa = h.odoms[-1].pose.pose.position.x
    h.spin_for(0.5)
    assert len(h.odoms) > n
    xb = h.odoms[-1].pose.pose.position.x
    assert abs(xb - xa) < 0.01
    assert abs(h.odoms[-1].twist.twist.linear.x) < 0.01


def test_rotation(h):
    assert h.spin_until(lambda: h.odoms, 3.0)
    h.spin_for(0.2)
    yaw0 = yaw_of(h.odoms[-1])
    h.drive(0.0, 0.5, 1.0)
    dyaw = bm.wrap_angle(yaw_of(h.odoms[-1]) - yaw0)
    assert 0.3 <= dyaw <= 0.7, dyaw


def test_camera_enable_disable(h):
    h.spin_for(1.0)
    assert not h.images, 'camera frames received while disabled'
    h.cam_pub.publish(Bool(data=True))
    assert h.spin_until(lambda: h.images, 2.0), 'no camera frame after enabling'
    img = h.images[-1]
    assert img.format == 'jpeg'
    assert img.header.frame_id == 'camera_optical_frame'
    bgr = cv2.imdecode(np.frombuffer(bytes(img.data), np.uint8), cv2.IMREAD_COLOR)
    assert bgr.shape == (240, 320, 3)
    patch = bgr[2:20, 2:30].reshape(-1, 3).mean(axis=0)  # B, G, R
    assert patch[2] > 150 and patch[2] > patch[0] + 80 and patch[2] > patch[1] + 80, patch

    h.cam_pub.publish(Bool(data=False))
    h.spin_for(0.5)  # let the message be processed and in-flight frames drain
    n = len(h.images)
    h.spin_for(1.0)
    assert len(h.images) == n, 'frames still arriving after disabling'


def test_camera_disabled_by_default_param_override():
    harness = Harness(['camera_enabled:=true'])
    try:
        assert harness.spin_until(lambda: harness.images, 2.0)
    finally:
        harness.close()


def test_bridge_status(h):
    assert h.spin_until(lambda: h.status, 3.0), 'no bridge_status received'
    assert h.status[-1].data == 'mock'


def test_robot_pose(h):
    assert h.spin_until(lambda: h.poses and h.odoms, 3.0)
    h.spin_for(0.3)
    p = h.poses[-1]
    assert p.header.frame_id == 'odom'
    o = h.odoms[-1]
    assert p.pose.position.x == pytest.approx(o.pose.pose.position.x, abs=0.05)
    assert p.pose.position.y == pytest.approx(o.pose.pose.position.y, abs=0.05)
