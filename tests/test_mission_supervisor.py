"""In-process integration tests for the mission supervisor node.

The supervisor runs in one SingleThreadedExecutor together with a probe node and, optionally,
mock Nav2 action servers and mock slam_toolbox services (no Nav2, slam_toolbox or CoppeliaSim).
"""
import json
import math
import os
import struct
import time

import pytest

rclpy = pytest.importorskip('rclpy')
np = pytest.importorskip('numpy')

try:
    from vr_robot_pkg.mission_supervisor_node import MissionSupervisorNode
except Exception as exc:  # pragma: no cover
    pytest.skip(f'supervisor node not importable: {exc}', allow_module_level=True)

from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.task import Future
from slam_toolbox.srv import DeserializePoseGraph, Reset, SerializePoseGraph
from std_msgs.msg import String

from vr_robot_pkg import keepout_mask as km
from vr_robot_pkg import map_store as ms
from vr_robot_pkg import mission_logic as ml

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
SPEC = km.GridSpec()   # supervisor defaults: 0.05 m, origin (-2.5, -4.25), 310x210
ROBOT = (1.0, 2.0, 0.5)


def f32(v):
    """Value as stored in a float32 message field (Point32)."""
    return struct.unpack('f', struct.pack('f', v))[0]


class MockNav2(Node):
    """Mock Nav2 action servers and slam_toolbox services."""

    def __init__(self):
        super().__init__('mock_nav2')
        self.cbg = ReentrantCallbackGroup()
        self.behavior = {'pose': ['succeed'], 'through': ['succeed']}
        self.remaining = 2                 # feedback of NavigateThroughPoses in 'hold'
        self.pose_goals, self.through_goals = [], []   # list of lists of PoseStamped
        self.cancels = 0
        # Like bt_navigator's navigator muxer: a goal on one action is aborted while a goal
        # on the other action is still running.
        self.active = {'pose': 0, 'through': 0}
        self.mux_rejections = 0
        self.deserialize_delay = 0.0
        self.serialize_calls, self.deserialize_calls, self.reset_calls = [], [], []
        kw = dict(goal_callback=lambda g: GoalResponse.ACCEPT,
                  cancel_callback=self._on_cancel, callback_group=self.cbg)
        self.srv_pose = ActionServer(self, NavigateToPose, '/navigate_to_pose',
                                     self._exec_pose, **kw)
        self.srv_through = ActionServer(self, NavigateThroughPoses, '/navigate_through_poses',
                                        self._exec_through, **kw)
        self.create_service(SerializePoseGraph, '/slam_toolbox/serialize_map',
                            self._serialize, callback_group=self.cbg)
        self.create_service(DeserializePoseGraph, '/slam_toolbox/deserialize_map',
                            self._deserialize, callback_group=self.cbg)
        self.create_service(Reset, '/slam_toolbox/reset', self._reset,
                            callback_group=self.cbg)

    def _on_cancel(self, _goal):
        self.cancels += 1
        return CancelResponse.ACCEPT

    def _next(self, key):
        lst = self.behavior[key]
        return lst.pop(0) if len(lst) > 1 else lst[0]

    async def _sleep(self, s):
        fut = Future()
        timer = None

        def cb():
            timer.cancel()
            if not fut.done():
                fut.set_result(None)
        timer = self.create_timer(s, cb, callback_group=self.cbg)
        await fut
        self.destroy_timer(timer)

    async def _run(self, key, goal_handle, result_cls, make_feedback):
        other = 'through' if key == 'pose' else 'pose'
        if self.active[other]:
            self.mux_rejections += 1
            goal_handle.abort()
            res = result_cls()
            res.error_msg = 'another navigator is processing'
            return res
        self.active[key] += 1
        try:
            return await self._run_goal(key, goal_handle, result_cls, make_feedback)
        finally:
            self.active[key] -= 1

    async def _run_goal(self, key, goal_handle, result_cls, make_feedback):
        beh = self._next(key)
        if beh == 'hold':
            last_fb = 0.0
            while not goal_handle.is_cancel_requested:
                now = time.monotonic()
                if now - last_fb >= 0.05:
                    last_fb = now
                    goal_handle.publish_feedback(make_feedback())
                await self._sleep(0.02)
        elif beh == 'succeed':
            for _ in range(15):
                if goal_handle.is_cancel_requested:
                    break
                await self._sleep(0.02)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result_cls()
        res = result_cls()
        if beh == 'abort':
            res.error_msg = 'mock failure'
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return res

    async def _exec_pose(self, goal_handle):
        self.pose_goals.append([goal_handle.request.pose])
        return await self._run('pose', goal_handle, NavigateToPose.Result,
                               NavigateToPose.Feedback)

    async def _exec_through(self, goal_handle):
        self.through_goals.append(list(goal_handle.request.poses))

        def fb():
            m = NavigateThroughPoses.Feedback()
            m.number_of_poses_remaining = self.remaining
            return m
        return await self._run('through', goal_handle, NavigateThroughPoses.Result, fb)

    def _serialize(self, req, resp):
        self.serialize_calls.append(req.filename)
        for ext in ('.posegraph', '.data'):
            with open(req.filename + ext, 'w') as f:
                f.write('dummy')
        resp.result = 0
        return resp

    async def _deserialize(self, req, resp):
        self.deserialize_calls.append(req)
        if self.deserialize_delay:
            await self._sleep(self.deserialize_delay)
        return resp

    def _reset(self, req, resp):
        self.reset_calls.append(req)
        resp.result = 0
        return resp

    def close(self):
        self.srv_pose.destroy()
        self.srv_through.destroy()
        self.destroy_node()


class Harness:
    def __init__(self, tmp_path, params=(), mocks=True):
        self.tmp = tmp_path
        args = ['--ros-args', '-p', f'maps_dir:={tmp_path}']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args)
        self.sup = self.probe = self.mock = self.ex = None
        try:
            self.sup = MissionSupervisorNode()
            self.probe = rclpy.create_node('probe')
            self.mock = MockNav2() if mocks else None
            self.ex = SingleThreadedExecutor()
            for n in (self.sup, self.probe, self.mock):
                if n is not None:
                    self.ex.add_node(n)
            pr = self.probe
            self.mission, self.map_states, self.masks = [], [], []
            pr.create_subscription(String, '/hmi/mission_state',
                                   lambda m: self.mission.append(json.loads(m.data)), LATCHED)
            pr.create_subscription(String, '/hmi/map_state',
                                   lambda m: self.map_states.append(json.loads(m.data)), LATCHED)
            pr.create_subscription(OccupancyGrid, '/keepout_filter_mask',
                                   self.masks.append, LATCHED)
            self.mask_copies = []
            pr.create_subscription(OccupancyGrid, '/hmi/keepout_mask',
                                   self.mask_copies.append, 10)
            self.map_copies = []
            pr.create_subscription(OccupancyGrid, '/hmi/map', self.map_copies.append, 10)
            self.goal_pub = pr.create_publisher(PoseStamped, '/hmi/goal', 10)
            self.path_pub = pr.create_publisher(Path, '/hmi/path', 10)
            self.mcmd_pub = pr.create_publisher(String, '/hmi/mission_cmd', 10)
            self.map_cmd_pub = pr.create_publisher(String, '/hmi/map_cmd', 10)
            self.stroke_pub = pr.create_publisher(PolygonStamped, '/hmi/keepout_stroke', 10)
            self.test_poly_pub = pr.create_publisher(PolygonStamped, '/keepout_mask_test', 10)
            self.teleop_pub = pr.create_publisher(Twist, '/cmd_vel_teleop', 10)
            self.odom_pub = pr.create_publisher(Odometry, '/myRobot/odom', 10)
            self.map_pub = pr.create_publisher(OccupancyGrid, '/map', LATCHED)
            self.set_odom(*ROBOT)
            self.publish_map()
            ok = self.spin_until(lambda: self.sup.last_map is not None and self.mission
                                 and self.map_states and self.masks, 5.0)
            assert ok, 'harness startup: supervisor did not receive the inputs'
            if mocks:
                ok = self.spin_until(
                    lambda: self.sup.nav_to_pose.server_is_ready()
                    and self.sup.nav_through.server_is_ready()
                    and self.sup.serialize_cli.service_is_ready()
                    and self.sup.deserialize_cli.service_is_ready()
                    and self.sup.reset_cli.service_is_ready(), 8.0)
                assert ok, 'harness startup: mock servers not discovered'
        except Exception:
            self.close()
            raise

    # ------------------------------------------------------------------ spinning
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.02)

    def spin_until(self, predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return bool(predicate())

    # ------------------------------------------------------------------ publishing
    def send(self, pub, msg):
        assert self.spin_until(lambda: pub.get_subscription_count() > 0, 5.0), \
            f'no subscriber on {pub.topic_name}'
        pub.publish(msg)

    def set_odom(self, x, y, yaw, v=0.0, w=0.0):
        m = Odometry()
        m.twist.twist.linear.x, m.twist.twist.angular.z = float(v), float(w)
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        qx, qy, qz, qw = ml.yaw_to_quaternion(yaw)
        m.pose.pose.orientation.x, m.pose.pose.orientation.y = qx, qy
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
        self.send(self.odom_pub, m)
        ok = self.spin_until(lambda: self.sup.odom_pose is not None
                             and abs(self.sup.odom_pose[0] - x) < 1e-6
                             and abs(self.sup.odom_pose[1] - y) < 1e-6, 3.0)
        assert ok, 'supervisor did not get the odometry'

    def publish_map(self):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution = 0.05
        m.info.width, m.info.height = 4, 3
        m.info.origin.position.x = m.info.origin.position.y = -1.0
        m.info.origin.orientation.w = 1.0
        m.data = [0, 100, -1, 0, 100, 0, 0, -1, 0, 0, 100, 100]
        self.send(self.map_pub, m)

    def goal(self, x, y, quat=(0.0, 0.0, 0.0, 1.0)):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = quat
        self.send(self.goal_pub, m)

    def path(self, pts):
        m = Path()
        m.header.frame_id = 'map'
        for x, y in pts:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        self.send(self.path_pub, m)

    def mission_cmd(self, text):
        self.send(self.mcmd_pub, String(data=text))

    def map_cmd(self, text):
        self.send(self.map_cmd_pub, String(data=text))

    @staticmethod
    def polygon(pts):
        m = PolygonStamped()
        m.header.frame_id = 'map'
        m.polygon.points = [Point32(x=float(x), y=float(y), z=float(z)) for x, y, z in pts]
        return m

    def stroke(self, pts):
        self.send(self.stroke_pub, self.polygon(pts))

    def teleop(self, v, w=0.0):
        m = Twist()
        m.linear.x, m.angular.z = v, w
        self.send(self.teleop_pub, m)

    # ------------------------------------------------------------------ observation
    def wait_mission(self, pred, timeout=5.0):
        return self.spin_until(lambda: bool(self.mission) and pred(self.mission[-1]), timeout)

    def wait_state(self, state, timeout=5.0, **extra):
        ok = self.wait_mission(
            lambda s: s['state'] == state and all(s[k] == v for k, v in extra.items()), timeout)
        assert ok, f'never reached {state} {extra}; last states: {self.mission[-3:]}'
        return self.mission[-1]

    def wait_map(self, pred, timeout=5.0):
        ok = self.spin_until(lambda: bool(self.map_states) and pred(self.map_states[-1]),
                             timeout)
        assert ok, f'map state condition not met; last: {self.map_states[-3:]}'
        return self.map_states[-1]

    def mask_array(self):
        m = self.masks[-1]
        return np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)

    def wait_mask(self, pred, timeout=5.0):
        ok = self.spin_until(lambda: bool(self.masks) and pred(self.mask_array()), timeout)
        assert ok, f'mask condition not met; cells set: {int((self.mask_array() == 100).sum())}'
        return self.mask_array()

    def close(self):
        try:
            if self.ex is not None:
                self.ex.shutdown()
            if self.mock is not None:
                self.mock.close()
            if self.sup is not None:
                self.sup.destroy_node()
            if self.probe is not None:
                self.probe.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


@pytest.fixture
def make_h(tmp_path):
    made = []

    def factory(params=(), mocks=True):
        h = Harness(tmp_path, params, mocks)
        made.append(h)
        return h
    yield factory
    for h in made:
        h.close()


def make_map_files(tmp_path, name='house'):
    for ext in ('.posegraph', '.data'):
        (tmp_path / (name + ext)).write_text('dummy')


def expected_stroke_mask(strokes, dilation=0.21):
    mask = km.empty_mask(SPEC)
    for s in strokes:
        km.rasterize_stroke(mask, SPEC, s, dilation=dilation)
    return mask


# ---------------------------------------------------------------------- 1
def test_initial_state(make_h):
    h = make_h()
    s = h.mission[-1]
    assert set(s) == {'state', 'mode', 'remaining', 'message'}
    assert (s['state'], s['mode'], s['remaining']) == ('IDLE', None, 0)
    m = h.map_states[-1]
    assert m['state'] == 'READY' and m['map'] is None and m['maps'] == []
    msg = h.masks[-1]
    assert (msg.info.width, msg.info.height) == (310, 210)
    assert msg.info.resolution == pytest.approx(0.05)
    assert msg.info.origin.position.x == pytest.approx(-2.5)
    assert msg.info.origin.position.y == pytest.approx(-4.25)
    assert msg.header.frame_id == 'map'
    assert len(msg.data) == 310 * 210 and not any(msg.data)


# ---------------------------------------------------------------------- 2
def test_goal_succeeds(make_h):
    h = make_h()
    h.goal(3.0, 1.5)
    h.wait_state('NAVIGATING', mode='goal', remaining=1)
    assert h.spin_until(lambda: h.mock.pose_goals, 5.0)
    pose = h.mock.pose_goals[0][0]
    assert pose.header.frame_id == 'map'
    assert (pose.pose.position.x, pose.pose.position.y) == pytest.approx((3.0, 1.5))
    h.wait_state('SUCCEEDED')


# ---------------------------------------------------------------------- 3
def test_goal_without_orientation_faces_travel_direction(make_h):
    h = make_h()
    h.goal(4.0, 6.0, quat=(0.0, 0.0, 0.0, 0.0))
    assert h.spin_until(lambda: h.mock.pose_goals, 5.0)
    q = h.mock.pose_goals[0][0].pose.orientation
    yaw = ml.quaternion_to_yaw(q.x, q.y, q.z, q.w)
    assert yaw == pytest.approx(math.atan2(6.0 - 2.0, 4.0 - 1.0), abs=1e-6)


# ---------------------------------------------------------------------- 4
def test_path_waypoints(make_h):
    h = make_h()
    h.mock.behavior['through'] = ['hold']
    h.path([(1.0 + 0.05 * i, 2.0) for i in range(41)])
    h.wait_state('NAVIGATING', mode='path')
    assert h.spin_until(lambda: h.mock.through_goals, 5.0)
    wps = [(p.pose.position.x, p.pose.position.y) for p in h.mock.through_goals[0]]
    assert len(wps) >= 2
    assert wps[-1] == pytest.approx((3.0, 2.0))
    for a, b in zip(wps, wps[1:]):
        assert math.dist(a, b) >= 0.3 - 1e-6
    assert all(math.dist(w, (1.0, 2.0)) >= 0.3 - 1e-6 for w in wps)


# ---------------------------------------------------------------------- 5
def test_goal_aborted(make_h):
    h = make_h()
    h.mock.behavior['pose'] = ['abort']
    h.goal(3.0, 2.0)
    s = h.wait_state('FAILED')
    assert 'mock failure' in s['message']


# ---------------------------------------------------------------------- 6
def test_pause_resume_path(make_h):
    h = make_h()
    h.mock.behavior['through'] = ['hold']
    h.mock.remaining = 2
    h.path([(1.0 + 0.05 * i, 2.0) for i in range(41)])
    h.wait_state('NAVIGATING', mode='path', remaining=2)
    assert h.spin_until(lambda: h.mock.through_goals, 5.0)
    original = [(p.pose.position.x, p.pose.position.y) for p in h.mock.through_goals[0]]
    assert len(original) > 2
    h.mission_cmd('PAUSE')
    s = h.wait_state('PAUSED')
    assert s['remaining'] == 2
    assert h.spin_until(lambda: h.mock.cancels >= 1, 5.0)
    h.mission_cmd('RESUME')
    assert h.spin_until(lambda: len(h.mock.through_goals) >= 2, 5.0)
    resumed = [(p.pose.position.x, p.pose.position.y) for p in h.mock.through_goals[1]]
    assert resumed == pytest.approx(original[-2:])
    assert len(resumed) == 2
    h.wait_state('NAVIGATING', mode='path')


# ---------------------------------------------------------------------- 7
def test_pause_resume_goal(make_h):
    h = make_h()
    h.mock.behavior['pose'] = ['hold']
    h.goal(3.0, 1.5)
    h.wait_state('NAVIGATING', mode='goal')
    assert h.spin_until(lambda: h.mock.pose_goals, 5.0)
    h.mission_cmd('PAUSE')
    s = h.wait_state('PAUSED')
    assert s['remaining'] == 1
    assert h.spin_until(lambda: h.mock.cancels >= 1, 5.0)
    h.mission_cmd('RESUME')
    assert h.spin_until(lambda: len(h.mock.pose_goals) >= 2, 5.0)
    a, b = h.mock.pose_goals[0][0], h.mock.pose_goals[1][0]
    assert (b.pose.position.x, b.pose.position.y) == pytest.approx(
        (a.pose.position.x, a.pose.position.y))
    assert b.pose.orientation.w == pytest.approx(a.pose.orientation.w)
    assert b.pose.orientation.z == pytest.approx(a.pose.orientation.z)
    h.wait_state('NAVIGATING', mode='goal')


# ---------------------------------------------------------------------- 8
def test_cancel_while_navigating(make_h):
    h = make_h()
    h.mock.behavior['pose'] = ['hold']
    h.goal(3.0, 1.5)
    h.wait_state('NAVIGATING')
    assert h.spin_until(lambda: h.mock.pose_goals, 5.0)
    h.mission_cmd('CANCEL')
    h.wait_state('CANCELED')
    assert h.spin_until(lambda: h.mock.cancels >= 1, 5.0)
    h.mission_cmd('RESUME')
    h.spin_for(0.5)
    assert h.mission[-1]['state'] == 'CANCELED'
    assert len(h.mock.pose_goals) == 1


def test_cancel_while_paused(make_h):
    h = make_h()
    h.mock.behavior['pose'] = ['hold']
    h.goal(3.0, 1.5)
    h.wait_state('NAVIGATING')
    h.mission_cmd('PAUSE')
    h.wait_state('PAUSED')
    h.mission_cmd('CANCEL')
    h.wait_state('CANCELED')


# ---------------------------------------------------------------------- 9
def test_preemption(make_h):
    h = make_h()
    h.mock.behavior['pose'] = ['hold', 'succeed']
    h.goal(3.0, 1.5)
    h.wait_state('NAVIGATING')
    assert h.spin_until(lambda: len(h.mock.pose_goals) >= 1, 5.0)
    h.goal(-1.0, 0.5)
    assert h.spin_until(lambda: len(h.mock.pose_goals) >= 2 and h.mock.cancels >= 1, 5.0)
    p2 = h.mock.pose_goals[1][0].pose.position
    assert (p2.x, p2.y) == pytest.approx((-1.0, 0.5))
    h.wait_state('SUCCEEDED')
    h.spin_for(0.5)
    assert h.mission[-1]['state'] == 'SUCCEEDED'
    assert all(s['state'] != 'CANCELED' for s in h.mission)


# ---------------------------------------------------------------------- 10
def test_teleop_auto_pause(make_h):
    h = make_h()
    h.mock.behavior['pose'] = ['hold']
    h.goal(3.0, 1.5)
    h.wait_state('NAVIGATING')
    h.teleop(0.0, 0.0)
    h.spin_for(0.5)
    assert h.mission[-1]['state'] == 'NAVIGATING'
    h.teleop(0.2)
    s = h.wait_state('PAUSED')
    assert 'teleop' in s['message']


# ---------------------------------------------------------------------- 11
def test_nav2_missing(make_h):
    h = make_h(mocks=False)
    h.goal(3.0, 1.5)
    s = h.wait_state('FAILED')
    assert 'not available' in s['message']


# ---------------------------------------------------------------------- 12
def test_unknown_mission_command(make_h):
    h = make_h()
    h.mission_cmd('DANCE')
    assert h.wait_mission(lambda s: 'unknown' in s['message'])
    assert h.mission[-1]['state'] == 'IDLE'


def test_keepout_mask_volatile_copy(make_h):
    h = make_h(params=['keepout_republish_period:=0.2'])
    h.stroke([(0.5, 0.5, 0.1)])
    h.wait_mask(lambda m: (m == 100).any())
    # The copy of the stroke mask arrives with the same data as the latched mask.
    assert h.spin_until(
        lambda: h.mask_copies and h.mask_copies[-1].data == h.masks[-1].data
        and any(v == 100 for v in h.mask_copies[-1].data), 5.0)
    # Periodic republish: more copies without a new stroke, /keepout_filter_mask untouched.
    n_copies, n_masks = len(h.mask_copies), len(h.masks)
    assert h.spin_until(lambda: len(h.mask_copies) >= n_copies + 2, 5.0)
    assert len(h.masks) == n_masks
    assert h.mask_copies[-1].data == h.masks[-1].data


def test_map_volatile_copy(make_h):
    h = make_h(params=['map_republish_period:=0.2'])
    # The map published by the harness arrives on /hmi/map with identical data.
    assert h.spin_until(lambda: h.map_copies, 5.0)
    assert list(h.map_copies[-1].data) == list(h.sup.last_map.data)
    assert h.map_copies[-1].info.width == 4 and h.map_copies[-1].info.height == 3
    # Periodic republish without new /map messages.
    n = len(h.map_copies)
    assert h.spin_until(lambda: len(h.map_copies) >= n + 2, 5.0)
    assert list(h.map_copies[-1].data) == list(h.sup.last_map.data)


# ---------------------------------------------------------------------- 13
def test_keepout_stroke(make_h):
    h = make_h()
    h.stroke([(0.5, 0.5, 0.1)])
    radius = f32(0.1) + 0.21
    expected = km.empty_mask(SPEC)
    n = km.rasterize_disc(expected, SPEC, 0.5, 0.5, radius)
    assert n > 0
    mask = h.wait_mask(lambda m: (m == 100).any())
    assert int((mask == 100).sum()) == n
    assert np.array_equal(mask, expected)

    h.stroke([(1.5, 1.5, 0.1)])
    mask2 = h.wait_mask(lambda m: int((m == 100).sum()) > n)
    assert int((mask2 == 100).sum()) == 2 * n   # the two discs do not overlap

    h.stroke([])
    h.wait_mask(lambda m: not m.any())


# ---------------------------------------------------------------------- 14
def test_keepout_test_polygon(make_h):
    h = make_h()
    h.send(h.test_poly_pub, h.polygon([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]))
    mask = h.wait_mask(lambda m: (m == 100).any())
    assert int((mask == 100).sum()) == 400
    h.send(h.test_poly_pub, h.polygon([]))
    h.wait_mask(lambda m: not m.any())


# ---------------------------------------------------------------------- 15
def test_save_map(make_h, tmp_path):
    h = make_h()
    h.stroke([(0.5, 0.5, 0.1)])
    h.wait_mask(lambda m: m.any())
    h.map_cmd('SAVE house')
    st = h.wait_map(lambda s: s['map'] == 'house' and s['state'] == 'READY')
    assert st['ok'] is True and st['maps'] == ['house']
    assert h.mock.serialize_calls == [str(tmp_path / 'house')]
    for ext in ('.posegraph', '.data', '.keepout.json', '.pgm', '.yaml'):
        assert os.path.isfile(tmp_path / ('house' + ext)), ext
    strokes = ms.load_strokes(str(tmp_path), 'house')
    assert strokes == [[(0.5, 0.5, f32(0.1))]]
    assert 'image: house.pgm' in (tmp_path / 'house.yaml').read_text()


# ---------------------------------------------------------------------- 16
def test_load_map(make_h, tmp_path):
    make_map_files(tmp_path)
    ms.save_strokes(str(tmp_path), 'house', [[(0.5, 0.5, 0.1)]])
    h = make_h()
    h.mock.behavior['pose'] = ['hold']
    h.goal(3.0, 1.5)
    h.wait_state('NAVIGATING')
    assert h.spin_until(lambda: h.mock.pose_goals, 5.0)
    h.map_cmd('LOAD house')
    assert h.spin_until(lambda: h.mock.deserialize_calls, 5.0)
    req = h.mock.deserialize_calls[0]
    assert req.filename == str(tmp_path / 'house')
    assert req.match_type == 2
    assert req.initial_pose.x == pytest.approx(1.0)
    assert req.initial_pose.y == pytest.approx(2.0)
    assert req.initial_pose.theta == pytest.approx(0.5, abs=1e-6)
    s = h.wait_state('CANCELED')
    assert s['message'] == 'map loaded'
    st = h.wait_map(lambda s: s['map'] == 'house' and s['state'] == 'READY')
    assert st['ok'] is True
    mask = h.wait_mask(lambda m: m.any())
    assert np.array_equal(mask, expected_stroke_mask([[(0.5, 0.5, 0.1)]]))


# ---------------------------------------------------------------------- 17
def test_load_missing_map(make_h):
    h = make_h()
    h.map_cmd('LOAD nothere')
    st = h.wait_map(lambda s: s['ok'] is False)
    assert 'no saved map' in st['message']
    h.spin_for(0.3)
    assert h.mock.deserialize_calls == []


# ---------------------------------------------------------------------- 18
@pytest.mark.parametrize('cmd', ['SAVE bad/name', 'FLY'])
def test_invalid_map_command(make_h, cmd):
    h = make_h()
    h.map_cmd(cmd)
    st = h.wait_map(lambda s: s['ok'] is False)
    assert st['message']
    h.spin_for(0.3)
    assert not (h.mock.serialize_calls or h.mock.deserialize_calls or h.mock.reset_calls)


# ---------------------------------------------------------------------- 19
def test_reset_map(make_h):
    h = make_h()
    h.stroke([(0.5, 0.5, 0.1)])
    h.wait_mask(lambda m: m.any())
    h.map_cmd('RESET')
    assert h.spin_until(lambda: h.mock.reset_calls, 5.0)
    assert h.mock.reset_calls[0].pause_new_measurements is False
    st = h.wait_map(lambda s: 'reset' in s['message'] and s['state'] == 'READY')
    assert st['map'] is None and st['ok'] is True
    h.wait_mask(lambda m: not m.any())


# ---------------------------------------------------------------------- 20
def test_startup_auto_load(make_h, tmp_path):
    make_map_files(tmp_path)
    h = make_h(params=['initial_map:=house'])
    assert h.spin_until(lambda: h.mock.deserialize_calls, 8.0), 'no auto-load request'
    assert h.mock.deserialize_calls[0].filename == str(tmp_path / 'house')
    st = h.wait_map(lambda s: s['map'] == 'house' and s['state'] == 'READY')
    assert st['ok'] is True


# ---------------------------------------------------------------------- 21
def test_map_service_missing(make_h):
    h = make_h(mocks=False)
    h.map_cmd('SAVE house')
    st = h.wait_map(lambda s: s['ok'] is False)
    assert 'not available' in st['message']


# ---------------------------------------------------------------------- regressions
@pytest.mark.parametrize('first,second', [('goal', 'path'), ('path', 'goal')])
def test_switch_between_goal_and_path(make_h, first, second):
    """Regression: the new goal was sent right after the cancel, while the old navigator
    was still running, and bt_navigator aborted it (mission FAILED)."""
    h = make_h()
    key = {'goal': 'pose', 'path': 'through'}
    h.mock.behavior = {key[first]: ['hold'], key[second]: ['succeed']}
    send = {'goal': lambda: h.goal(3.0, 1.5),
            'path': lambda: h.path([(1.0, 2.0), (2.0, 2.0), (3.0, 2.0)])}
    send[first]()
    h.wait_state('NAVIGATING', mode=first)
    assert h.spin_until(lambda: h.mock.active[key[first]] == 1, 5.0)
    send[second]()
    h.wait_state('SUCCEEDED', mode=second, timeout=8.0)
    assert h.mock.mux_rejections == 0
    assert h.mock.cancels >= 1
    assert all(s['state'] != 'FAILED' for s in h.mission)


def test_load_waits_for_the_robot_to_stop(make_h, tmp_path):
    """Regression: LOAD sent the odom pose of the request time, but slam_toolbox applies it
    to the next scan; a moving robot left map -> odom offset for the rest of the session."""
    make_map_files(tmp_path)
    h = make_h()
    h.set_odom(1.0, 2.0, 0.5, v=0.3)
    h.map_cmd('LOAD house')
    st = h.wait_map(lambda s: 'waiting for the robot to stop' in s['message'])
    assert st['state'] == 'BUSY'
    h.spin_for(1.0)
    assert h.mock.deserialize_calls == []
    h.set_odom(1.5, 2.25, 0.75)          # stopped here
    assert h.spin_until(lambda: h.mock.deserialize_calls, 3.0)
    req = h.mock.deserialize_calls[0]
    assert (req.initial_pose.x, req.initial_pose.y) == pytest.approx((1.5, 2.25))
    assert req.initial_pose.theta == pytest.approx(0.75, abs=1e-6)
    h.wait_map(lambda s: s['map'] == 'house' and s['state'] == 'READY' and s['ok'])


def test_load_fails_if_the_robot_keeps_moving(make_h, tmp_path):
    make_map_files(tmp_path)
    h = make_h(params=['stop_timeout:=1.0', 'state_period:=0.2'])
    h.set_odom(1.0, 2.0, 0.5, w=0.5)
    h.map_cmd('LOAD house')
    st = h.wait_map(lambda s: s['state'] == 'READY' and s['ok'] is False, timeout=4.0)
    assert 'did not stop' in st['message']
    assert h.mock.deserialize_calls == []


def test_late_load_answer_is_applied(make_h, tmp_path):
    """Regression: after a service timeout the late slam_toolbox answer was dropped, so the
    supervisor's map name and keepout no longer matched the map SLAM had loaded."""
    make_map_files(tmp_path)
    ms.save_strokes(str(tmp_path), 'house', [[(0.5, 0.5, 0.1)]])
    h = make_h(params=['service_timeout:=0.3', 'state_period:=0.1'])
    h.mock.deserialize_delay = 1.5
    h.map_cmd('LOAD house')
    st = h.wait_map(lambda s: s['ok'] is False and 'did not answer' in s['message'], 3.0)
    assert st['state'] == 'READY' and st['map'] is None
    st = h.wait_map(lambda s: s['map'] == 'house' and s['ok'] is True, 4.0)
    assert '1 keepout strokes' in st['message']
    h.wait_mask(lambda m: m.any())


def test_initial_map_timer_stops_after_load(make_h, tmp_path):
    make_map_files(tmp_path)
    h = make_h(params=['initial_map:=house'])
    assert h.spin_until(lambda: h.mock.deserialize_calls, 8.0)
    assert h.sup.initial_map_timer.is_canceled()


def test_map_failure_after_success_does_not_crash(make_h, tmp_path):
    """Regression: set_map_state logged at INFO and WARNING from one call site; rclpy raises
    ValueError on the second severity, which killed the node (e.g. a bad LOAD after a SAVE)."""
    h = make_h()
    h.map_cmd('SAVE house')
    h.wait_map(lambda s: s['map'] == 'house' and s['ok'] is True)
    h.map_cmd('LOAD nothere')
    st = h.wait_map(lambda s: s['ok'] is False)
    assert 'no saved map' in st['message']
