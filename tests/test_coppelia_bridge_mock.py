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


class _SlowBackend:
    """Wraps the mock backend: each exchange takes 0.4 s and records overlaps with shutdown."""

    def __init__(self, inner):
        self.inner = inner
        self.in_exchange = False
        self.exchanges = 0
        self.overlap = False
        self.shutdown_after_exchange = None
        self.camera_pose = inner.camera_pose
        self.host = 'mock'
        self.started_simulation = False

    def exchange(self, *args, **kwargs):
        self.in_exchange = True
        self.exchanges += 1
        time.sleep(0.4)
        self.in_exchange = False
        return self.inner.exchange(*args, **kwargs)

    def shutdown(self, *args, **kwargs):
        self.overlap = self.overlap or self.in_exchange
        self.shutdown_after_exchange = not self.in_exchange
        return self.inner.shutdown(*args, **kwargs)

    def __getattr__(self, name):
        return getattr(self.inner, name)


def test_shutdown_waits_for_inflight_exchange(h):
    """Regression: on Ctrl-C the main thread ran shutdown_backend() while an executor worker
    was inside an exchange; closing the ZMQ socket under a pending recv() aborted the bridge
    in libzmq. shutdown_backend() must wait for the in-flight call and block new ones."""
    import threading
    assert h.spin_until(lambda: h.odoms, 3.0)
    slow = _SlowBackend(h.bridge.backend)
    h.bridge.backend = slow
    worker = threading.Thread(target=h.bridge.on_cycle)   # what an executor thread does
    worker.start()
    time.sleep(0.1)
    assert slow.in_exchange
    t0 = time.monotonic()
    h.bridge.shutdown_backend()
    waited = time.monotonic() - t0
    worker.join(2.0)
    assert not slow.overlap
    assert slow.shutdown_after_exchange is True
    assert waited >= 0.2, f'shutdown did not wait for the exchange ({waited:.2f} s)'
    # Callbacks firing after the shutdown must not touch the backend.
    n = slow.exchanges
    h.bridge.on_cycle()
    h.bridge.on_status_timer()
    assert slow.exchanges == n
    assert h.bridge.backend is None


class _RepeatScanBackend:
    """Returns the same scan (same scan_seq) on every exchange, as remoteExchange does when
    two exchanges land in the same simulation step."""

    camera_pose = None
    started_simulation = False

    def __init__(self):
        self.want_scan = []

    def exchange(self, wheels, want_scan, want_camera, jpeg_quality, running):
        from vr_robot_pkg.coppelia_bridge_node import make_state
        self.want_scan.append(want_scan)
        return make_state((0, 0, 0, 0, 0, 0, 1), (0, 0, 0), True,
                          scan=[1.0] * bm.SCAN_SAMPLES, scan_seq=7)

    def shutdown(self, *_args, **_kwargs):
        pass


def test_repeated_scan_does_not_restart_scan_period(h):
    """Regression: a repeated scan (unchanged scan_seq, not published) restarted the scan
    period, delaying the next new scan and lowering the /myRobot/scan rate."""
    assert h.spin_until(lambda: h.odoms, 3.0)
    stub = _RepeatScanBackend()
    with h.bridge.backend_lock:
        h.bridge.backend = stub
        h.bridge.last_scan_seq = None
        h.bridge.last_scan_time = -1e9
        h.bridge._cycle()                 # new seq 7: published, period restarts
        published_at = h.bridge.last_scan_time
        time.sleep(h.bridge.scan_period)
        h.bridge._cycle()                 # seq 7 again: not published
        assert h.bridge.last_scan_time == published_at
        h.bridge._cycle()                 # still asks for a scan
    assert stub.want_scan == [True, True, True]


def test_shutdown_cleans_up_when_disconnected(h, monkeypatch):
    """Regression: after a transient disconnect (backend None) the bridge exited without
    restoring CoppeliaSim's display or stopping the simulation it had started."""
    from vr_robot_pkg import coppelia_bridge_node as cbn
    calls = []
    monkeypatch.setattr(cbn.CoppeliaBackend, 'shutdown',
                        lambda self, stop, restore, close_main=True:
                        calls.append((self.host, stop, restore)))
    with h.bridge.backend_lock:
        h.bridge.backend = None
        h.bridge.connected = False
        h.bridge.mock = False
        h.bridge.host = '192.0.2.1'
        h.bridge.sim_started_by_bridge = True
        h.bridge.display_to_restore = True
    h.bridge.shutdown_backend()
    assert calls == [('192.0.2.1', True, True)]
    # The fixture's own shutdown must not try to reach the fake host.
    h.bridge.mock, h.bridge.sim_started_by_bridge, h.bridge.display_to_restore = True, False, None


def test_abort_stops_connect_before_any_remote_call():
    """Shutdown sets the abort event: a connect in a worker thread must not start another
    Remote API call (each can block for the full timeout while the lock is held)."""
    import threading
    from vr_robot_pkg.coppelia_bridge_node import CoppeliaBackend, SimDisconnected

    class Log:
        def warning(self, *_a, **_k):
            pass
        info = warning
    abort = threading.Event()
    abort.set()
    backend = CoppeliaBackend('192.0.2.1', 23000, '/myRobot', 10000, Log(), abort)
    t0 = time.monotonic()
    with pytest.raises(SimDisconnected):
        backend.connect(start_simulation=True)
    assert time.monotonic() - t0 < 1.0
    assert backend.started_simulation is False
    assert backend.client is None
