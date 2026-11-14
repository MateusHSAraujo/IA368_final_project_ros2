"""Mission supervisor: the single ROS node the Unity HMI talks to (Stage 5).

ROS-TCP-Connector cannot call ROS actions or services, so Unity uses plain topics and this
node turns them into:

* Nav2 actions: ``/hmi/goal`` -> NavigateToPose, ``/hmi/path`` -> NavigateThroughPoses
  (thinned to waypoints), ``/hmi/mission_cmd`` PAUSE / RESUME / CANCEL (Nav2 has no native
  pause: PAUSE cancels and keeps the remaining poses). A nonzero ``/cmd_vel_teleop`` command
  auto-pauses an active mission. State on ``/hmi/mission_state`` (JSON).
* The Nav2 keepout filter mask: ``/hmi/keepout_stroke`` brush strokes (dilated by the robot
  half-width) are rasterized into ``/keepout_filter_mask`` (reliable + transient_local, which
  ROS-TCP-Endpoint cannot publish). ``/keepout_mask_test`` polygons are kept for debugging.
* Saved maps: ``/hmi/map_cmd`` SAVE / LOAD / RESET through the slam_toolbox services; the
  keepout strokes are saved with the map. State on ``/hmi/map_state`` (JSON).

Odometry is CoppeliaSim ground truth and ``map -> odom`` starts at identity, so a saved map
frame is the world frame and the robot pose in a loaded map is simply its odom pose.

Single-threaded and fully asynchronous: no callback ever blocks on a future.
"""
import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PolygonStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import DeserializePoseGraph, Reset, SerializePoseGraph
from std_msgs.msg import String

from vr_robot_pkg import keepout_mask as km
from vr_robot_pkg import map_store as ms
from vr_robot_pkg import mission_logic as ml

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

MAP_READY, MAP_BUSY = 'READY', 'BUSY'


class MissionSupervisorNode(Node):
    def __init__(self):
        super().__init__('mission_supervisor')
        p = self.declare_parameter
        self.frame_id = p('frame_id', 'map').value
        self.spec = km.GridSpec(
            resolution=float(p('resolution', 0.05).value),
            origin_x=float(p('origin_x', -2.5).value),
            origin_y=float(p('origin_y', -4.25).value),
            width=int(p('width', 310).value),
            height=int(p('height', 210).value))
        self.dilation = float(p('robot_half_width', 0.21).value)
        self.waypoint_spacing = float(p('waypoint_spacing', 0.3).value)
        self.teleop_threshold = float(p('teleop_threshold', 0.01).value)
        self.maps_dir = ms.resolve_maps_dir(str(p('maps_dir', '').value or ''))
        self.initial_map = str(p('initial_map', '').value or '')
        self.service_timeout = float(p('service_timeout', 30.0).value)
        slam_ns = p('slam_namespace', '/slam_toolbox').value.rstrip('/')

        # Publishers. The state topics are latched for ROS tools; ROS-TCP-Endpoint subscribes
        # as volatile and gets the 1 Hz republish.
        self.mask_pub = self.create_publisher(OccupancyGrid, '/keepout_filter_mask', LATCHED)
        self.mission_pub = self.create_publisher(String, '/hmi/mission_state', LATCHED)
        self.map_state_pub = self.create_publisher(String, '/hmi/map_state', LATCHED)

        # Inputs
        self.create_subscription(PoseStamped, '/hmi/goal', self.on_goal, 10)
        self.create_subscription(Path, '/hmi/path', self.on_path, 10)
        self.create_subscription(String, '/hmi/mission_cmd', self.on_mission_cmd, 10)
        self.create_subscription(PolygonStamped, '/hmi/keepout_stroke', self.on_stroke, 10)
        self.create_subscription(PolygonStamped, '/keepout_mask_test', self.on_test_polygon, 10)
        self.create_subscription(String, '/hmi/map_cmd', self.on_map_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self.on_teleop, 10)
        self.create_subscription(Odometry, p('odom_topic', '/myRobot/odom').value,
                                 self.on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)

        self.nav_to_pose = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.nav_through = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.serialize_cli = self.create_client(SerializePoseGraph, f'{slam_ns}/serialize_map')
        self.deserialize_cli = self.create_client(DeserializePoseGraph,
                                                  f'{slam_ns}/deserialize_map')
        self.reset_cli = self.create_client(Reset, f'{slam_ns}/reset')

        # Mission state
        self.state = ml.IDLE
        self.mode = None
        self.message = ''
        self.poses = []           # PoseStamped list of the active mission
        self.remaining = 0        # poses still to visit (Nav2 feedback)
        self.resume_poses = []    # stored by PAUSE
        self.goal_handle = None
        self.generation = 0       # invalidates callbacks of replaced/canceled goals

        # Keepout state
        self.mask = km.empty_mask(self.spec)
        self.strokes = []         # persisted with the map: lists of (x, y, r)

        # Map state
        self.map_busy = False
        self.map_op = 0           # invalidates late service responses after a timeout
        self.map_busy_since = 0.0
        self.map_name = None
        self.map_ok = True
        self.map_message = ''
        self.last_map = None
        self.odom_pose = None     # (x, y, yaw) in odom == world frame
        self.initial_map_done = not self.initial_map
        self.last_initial_log = 0.0
        self._last_mission_json = None
        self._last_map_json = None

        self.create_timer(float(p('state_period', 1.0).value), self.on_state_timer)
        if not self.initial_map_done:
            self.create_timer(1.0, self.try_initial_map)
        self.get_logger().info(f'Maps directory: {self.maps_dir}')
        self.publish_mask()
        self.publish_mission_state(force=True)
        self.publish_map_state(force=True)

    # ---------------------------------------------------------------- state publishing
    def publish_mission_state(self, force=False):
        if self.state == ml.NAVIGATING:
            remaining = self.remaining
        elif self.state == ml.PAUSED:
            remaining = len(self.resume_poses)
        else:
            remaining = 0
        text = ml.mission_state_json(self.state, self.mode, remaining, self.message)
        if force or text != self._last_mission_json:
            self._last_mission_json = text
            self.mission_pub.publish(String(data=text))

    def set_mission(self, state, message, mode='keep'):
        if state != self.state or message != self.message:
            self.get_logger().info(f'Mission {self.state} -> {state}: {message}')
        self.state = state
        self.message = message
        if mode != 'keep':
            self.mode = mode
        self.publish_mission_state()

    def publish_map_state(self, force=False):
        text = json.dumps({
            'state': MAP_BUSY if self.map_busy else MAP_READY,
            'map': self.map_name,
            'maps': ms.list_maps(self.maps_dir),
            'ok': self.map_ok,
            'message': self.map_message,
        })
        if force or text != self._last_map_json:
            self._last_map_json = text
            self.map_state_pub.publish(String(data=text))

    def set_map_state(self, ok, message, busy=None):
        (self.get_logger().info if ok else self.get_logger().warning)(f'Map: {message}')
        self.map_ok = ok
        self.map_message = message
        if busy is not None:
            self.map_busy = busy
        self.publish_map_state()

    def on_state_timer(self):
        if self.map_busy and time.monotonic() - self.map_busy_since > self.service_timeout:
            self.map_op += 1
            self.set_map_state(False, 'slam_toolbox did not answer (timed out)', busy=False)
        self.publish_mission_state(force=True)
        self.publish_map_state(force=True)

    # ---------------------------------------------------------------- inputs
    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.odom_pose = (p.position.x, p.position.y, ml.quaternion_to_yaw(q.x, q.y, q.z, q.w))

    def on_map(self, msg):
        self.last_map = msg

    def on_teleop(self, msg):
        if self.state != ml.NAVIGATING:
            return
        if abs(msg.linear.x) > self.teleop_threshold or abs(msg.angular.z) > self.teleop_threshold:
            self.pause('teleop')

    # ---------------------------------------------------------------- missions
    def _pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id   # stamp 0 = latest transform
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        qx, qy, qz, qw = ml.yaw_to_quaternion(yaw)
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        return ps

    def on_goal(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        if math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2) < 1e-6:
            # No orientation given: face the direction of travel.
            yaw = (math.atan2(y - self.odom_pose[1], x - self.odom_pose[0])
                   if self.odom_pose else 0.0)
        else:
            yaw = ml.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.start_mission(ml.MODE_GOAL, [self._pose(x, y, yaw)],
                           f'goal ({x:.2f}, {y:.2f})')

    def on_path(self, msg):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        start = self.odom_pose[:2] if self.odom_pose else None
        wps = ml.path_waypoints(pts, self.waypoint_spacing, start)
        if not wps:
            self.set_mission(self.state, 'empty path ignored')
            return
        self.start_mission(ml.MODE_PATH, [self._pose(*w) for w in wps],
                           f'path with {len(wps)} waypoints')

    def on_mission_cmd(self, msg):
        cmd = ml.parse_mission_cmd(msg.data)
        if cmd == 'PAUSE':
            self.pause('operator')
        elif cmd == 'RESUME':
            self.resume()
        elif cmd == 'CANCEL':
            self.cancel('canceled by operator')
        else:
            self.set_mission(self.state, f'unknown mission command {msg.data!r}')

    def _invalidate_goal(self):
        """Forget the active goal and cancel it in Nav2 (its callbacks become stale)."""
        self.generation += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def start_mission(self, mode, poses, message):
        self._invalidate_goal()
        self.resume_poses = []
        client, action = ((self.nav_to_pose, '/navigate_to_pose') if mode == ml.MODE_GOAL
                          else (self.nav_through, '/navigate_through_poses'))
        if not client.server_is_ready():
            self.poses, self.remaining = [], 0
            self.set_mission(ml.FAILED, f'Nav2 action server {action} not available', mode)
            return
        if mode == ml.MODE_GOAL:
            goal = NavigateToPose.Goal(pose=poses[0])
        else:
            goal = NavigateThroughPoses.Goal(poses=poses)
        gen = self.generation
        self.poses, self.remaining = list(poses), len(poses)
        self.set_mission(ml.NAVIGATING, message, mode)
        future = client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_feedback(gen, fb))
        future.add_done_callback(lambda f: self._on_goal_response(gen, f))

    def _on_goal_response(self, gen, future):
        handle = future.result()
        if gen != self.generation:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()   # replaced/paused/canceled before acceptance
            return
        if handle is None or not handle.accepted:
            self.set_mission(ml.FAILED, 'goal rejected by Nav2')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(gen, f))

    def _on_feedback(self, gen, feedback_msg):
        if gen != self.generation or self.state != ml.NAVIGATING:
            return
        if self.mode == ml.MODE_PATH:
            n = int(feedback_msg.feedback.number_of_poses_remaining)
            if n > 0 and n != self.remaining:
                self.remaining = min(n, len(self.poses))
                self.publish_mission_state()

    def _on_result(self, gen, future):
        if gen != self.generation:
            return
        self.goal_handle = None
        res = future.result()
        status = res.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.set_mission(ml.SUCCEEDED, 'goal reached')
        elif status == GoalStatus.STATUS_CANCELED:
            self.set_mission(ml.CANCELED, 'canceled by Nav2')
        else:
            err = res.result.error_msg or f'error code {res.result.error_code}'
            self.set_mission(ml.FAILED, f'navigation failed: {err}')

    def pause(self, reason):
        if self.state != ml.NAVIGATING:
            self.set_mission(self.state, 'nothing to pause')
            return
        self._invalidate_goal()
        self.resume_poses = ml.remaining_poses(self.poses, self.remaining)
        self.set_mission(ml.PAUSED, f'paused ({reason})')

    def resume(self):
        if self.state != ml.PAUSED or not self.resume_poses:
            self.set_mission(self.state, 'nothing to resume')
            return
        poses = self.resume_poses
        # Path mode stays 'path' even with one waypoint left, so Unity keeps showing the path.
        self.start_mission(self.mode, poses, f'resumed: {len(poses)} pose(s) left')

    def cancel(self, message):
        if self.state not in (ml.NAVIGATING, ml.PAUSED):
            self.set_mission(self.state, 'nothing to cancel')
            return
        self._invalidate_goal()
        self.resume_poses = []
        self.set_mission(ml.CANCELED, message)

    # ---------------------------------------------------------------- keepout
    def on_stroke(self, msg):
        pts = [(float(pt.x), float(pt.y), float(pt.z)) for pt in msg.polygon.points]
        if not pts:
            self.clear_keepout()
            return
        self.strokes.append(pts)
        km.rasterize_stroke(self.mask, self.spec, pts, dilation=self.dilation)
        self.publish_mask()

    def on_test_polygon(self, msg):
        pts = [(pt.x, pt.y) for pt in msg.polygon.points]
        if not pts:
            self.clear_keepout()
            return
        km.rasterize_polygon(self.mask, self.spec, pts)   # debug only: not saved with maps
        self.publish_mask()

    def clear_keepout(self):
        self.strokes = []
        self.mask = km.empty_mask(self.spec)
        self.publish_mask()

    def rebuild_mask(self):
        self.mask = km.empty_mask(self.spec)
        for pts in self.strokes:
            km.rasterize_stroke(self.mask, self.spec, pts, dilation=self.dilation)
        self.publish_mask()

    def publish_mask(self):
        msg = OccupancyGrid()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.info.map_load_time = now
        msg.info.resolution = self.spec.resolution
        msg.info.width = self.spec.width
        msg.info.height = self.spec.height
        msg.info.origin.position.x = self.spec.origin_x
        msg.info.origin.position.y = self.spec.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = km.to_occupancy_data(self.mask)
        self.mask_pub.publish(msg)
        self.get_logger().info(
            f'Keepout mask: {int((self.mask == km.KEEPOUT).sum())} cells, '
            f'{len(self.strokes)} strokes')

    # ---------------------------------------------------------------- maps
    def on_map_cmd(self, msg):
        try:
            cmd, name = ms.parse_map_cmd(msg.data)
        except ValueError as exc:
            self.set_map_state(False, str(exc))
            return
        if self.map_busy:
            self.set_map_state(False, f'{cmd} ignored: another map operation is running')
            return
        {'SAVE': self.save_map, 'LOAD': self.load_map, 'RESET': self.reset_map}[cmd](name)

    def _begin_map_op(self, message):
        self.map_op += 1
        self.map_busy_since = time.monotonic()
        self.set_map_state(True, message, busy=True)
        return self.map_op

    def _service_ready(self, client, what):
        if client.service_is_ready():
            return True
        self.set_map_state(False, f'{what}: slam_toolbox service {client.srv_name} not available')
        return False

    def save_map(self, name):
        if not self._service_ready(self.serialize_cli, f'SAVE {name}'):
            return
        try:
            os.makedirs(self.maps_dir, exist_ok=True)
        except OSError as exc:
            self.set_map_state(False, f'SAVE {name}: cannot create {self.maps_dir}: {exc}')
            return
        op = self._begin_map_op(f'saving {name}')
        req = SerializePoseGraph.Request(filename=ms.map_base(self.maps_dir, name))
        self.serialize_cli.call_async(req).add_done_callback(
            lambda f: self._on_saved(op, name, f))

    def _on_saved(self, op, name, future):
        if op != self.map_op:
            return
        res = future.result()
        if res is None or res.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            self.set_map_state(False, f'SAVE {name}: slam_toolbox could not write the map',
                               busy=False)
            return
        notes = []
        try:
            ms.save_strokes(self.maps_dir, name, self.strokes)
        except (OSError, ValueError) as exc:
            notes.append(f'keepout not saved: {exc}')
        if self.last_map is not None:
            m = self.last_map
            q = m.info.origin.orientation
            try:
                ms.write_map_image(self.maps_dir, name, m.info.width, m.info.height,
                                   m.info.resolution,
                                   (m.info.origin.position.x, m.info.origin.position.y,
                                    ml.quaternion_to_yaw(q.x, q.y, q.z, q.w)), m.data)
            except (OSError, ValueError) as exc:
                notes.append(f'image not saved: {exc}')
        self.map_name = name
        msg = f'saved {name}' + (f' ({"; ".join(notes)})' if notes else '')
        self.set_map_state(not notes, msg, busy=False)

    def load_map(self, name):
        if not ms.map_exists(self.maps_dir, name):
            self.set_map_state(False, f'LOAD {name}: no saved map with that name')
            return False
        if self.odom_pose is None:
            self.set_map_state(False, f'LOAD {name}: no odometry yet')
            return False
        if not self._service_ready(self.deserialize_cli, f'LOAD {name}'):
            return False
        if self.state in (ml.NAVIGATING, ml.PAUSED):
            self.cancel('map loaded')
        op = self._begin_map_op(f'loading {name}')
        x, y, yaw = self.odom_pose
        req = DeserializePoseGraph.Request(
            filename=ms.map_base(self.maps_dir, name),
            match_type=DeserializePoseGraph.Request.START_AT_GIVEN_POSE,
            initial_pose=Pose2D(x=float(x), y=float(y), theta=float(yaw)))
        self.deserialize_cli.call_async(req).add_done_callback(
            lambda f: self._on_loaded(op, name, f))
        return True

    def _on_loaded(self, op, name, future):
        if op != self.map_op:
            return
        if future.result() is None:
            self.set_map_state(False, f'LOAD {name}: slam_toolbox call failed', busy=False)
            return
        self.map_name = name
        try:
            self.strokes = ms.load_strokes(self.maps_dir, name)
        except ValueError as exc:
            self.strokes = []
            self.rebuild_mask()
            self.set_map_state(False, f'loaded {name}, keepout not restored: {exc}', busy=False)
            return
        self.rebuild_mask()
        self.set_map_state(True, f'loaded {name} ({len(self.strokes)} keepout strokes)',
                           busy=False)

    def reset_map(self, _name=None):
        if not self._service_ready(self.reset_cli, 'RESET'):
            return
        if self.state in (ml.NAVIGATING, ml.PAUSED):
            self.cancel('map reset')
        op = self._begin_map_op('resetting the map')
        self.reset_cli.call_async(Reset.Request(pause_new_measurements=False)).add_done_callback(
            lambda f: self._on_reset(op, f))

    def _on_reset(self, op, future):
        if op != self.map_op:
            return
        if future.result() is None:
            self.set_map_state(False, 'RESET: slam_toolbox call failed', busy=False)
            return
        self.map_name = None
        self.clear_keepout()
        self.set_map_state(True, 'map reset: mapping from scratch', busy=False)

    def try_initial_map(self):
        """Startup auto-load: wait for odometry, the slam_toolbox service and a first /map
        (slam_toolbox has processed scans), then LOAD once."""
        if self.initial_map_done or self.map_busy:
            return
        waiting = [what for what, ok in (
            ('odometry', self.odom_pose is not None),
            ('slam_toolbox', self.deserialize_cli.service_is_ready()),
            ('/map', self.last_map is not None)) if not ok]
        if waiting:
            if time.monotonic() - self.last_initial_log > 10.0:
                self.last_initial_log = time.monotonic()
                self.get_logger().info(
                    f'Initial map {self.initial_map}: waiting for {", ".join(waiting)}')
            return
        self.initial_map_done = True
        self.load_map(self.initial_map)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
