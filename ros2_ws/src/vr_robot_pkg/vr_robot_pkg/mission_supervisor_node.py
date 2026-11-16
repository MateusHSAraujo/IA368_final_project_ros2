"""Mission supervisor: the single ROS node the Unity HMI talks to (Stage 5).

ROS-TCP-Connector cannot call ROS actions or services, so Unity uses plain topics and this
node turns them into:

* Nav2 actions: ``/hmi/goal`` -> NavigateToPose, ``/hmi/path`` -> NavigateThroughPoses
  (thinned to waypoints), ``/hmi/mission_cmd`` PAUSE / RESUME / CANCEL (Nav2 has no native
  pause: PAUSE cancels and keeps the remaining poses). A nonzero ``/cmd_vel_teleop`` command
  auto-pauses an active mission, and sets the ``teleop`` flag of
  ``/hmi/mission_state`` while teleop commands keep arriving. State on ``/hmi/mission_state`` (JSON). A new goal is
  sent only after the previous one has ended in Nav2: bt_navigator aborts a goal on one
  action while the other is still running, and a cancel ends a pending goal on the same
  action too.
* The Nav2 keepout filter mask: ``/hmi/keepout_stroke`` brush strokes (dilated by the robot
  half-width) are rasterized into ``/keepout_filter_mask`` (reliable + transient_local, which
  ROS-TCP-Endpoint cannot publish). ``/keepout_mask_test`` polygons are kept for debugging.
* Saved maps: ``/hmi/map_cmd`` SAVE / LOAD / RESET through the slam_toolbox services; the
  keepout strokes are saved with the map. State on ``/hmi/map_state`` (JSON).

Odometry is CoppeliaSim ground truth and ``map -> odom`` starts at identity, so a saved map
frame is the world frame and the robot pose in a loaded map is simply its odom pose.
slam_toolbox pins that pose to the next scan it processes, so LOAD waits until the robot
has stopped (otherwise ``map -> odom`` would keep the motion in between as an offset).

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
        self.teleop_active_timeout = float(p('teleop_active_timeout', 0.5).value)
        self.last_teleop_time = None
        self.maps_dir = ms.resolve_maps_dir(str(p('maps_dir', '').value or ''))
        self.initial_map = str(p('initial_map', '').value or '')
        self.service_timeout = float(p('service_timeout', 30.0).value)
        # Longest wait for Nav2 to end a canceled goal before the next goal is sent anyway.
        self.cancel_timeout = float(p('cancel_timeout', 5.0).value)
        # LOAD waits until the odometry speed stays below these for settle_time.
        self.still_linear = float(p('still_linear_speed', 0.01).value)
        self.still_angular = float(p('still_angular_speed', 0.01).value)
        self.settle_time = float(p('settle_time', 0.5).value)
        self.stop_timeout = float(p('stop_timeout', 10.0).value)
        slam_ns = p('slam_namespace', '/slam_toolbox').value.rstrip('/')

        # Publishers. The state topics are latched for ROS tools; ROS-TCP-Endpoint subscribes
        # as volatile and gets the 1 Hz republish.
        self.mask_pub = self.create_publisher(OccupancyGrid, '/keepout_filter_mask', LATCHED)
        # Volatile copy for clients that cannot receive transient_local (ROS-TCP-Endpoint).
        self.mask_copy_pub = self.create_publisher(OccupancyGrid, '/hmi/keepout_mask', 1)
        # Volatile copy of /map (slam_toolbox only publishes it when the map changes).
        self.map_copy_pub = self.create_publisher(OccupancyGrid, '/hmi/map', 1)
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
        self.inflight = set()     # tokens of goals sent to Nav2 whose result has not arrived
        self.next_token = 0
        self.pending_goal = None  # (generation, client, goal) waiting for the old goal to end
        self.pending_since = 0.0

        # Keepout state
        self.mask = km.empty_mask(self.spec)
        self.mask_msg = None      # last built mask message, republished periodically
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
        self.still_since = None   # monotonic time since the robot is stopped (None = moving)
        self.pending_load = None  # (map_op, name) of a LOAD waiting for the robot to stop
        self.maps = ms.list_maps(self.maps_dir)   # refreshed on map commands and SAVE
        self.initial_map_done = not self.initial_map
        self.last_initial_log = 0.0
        self._last_mission_json = None
        self._last_map_json = None

        self.create_timer(float(p('state_period', 1.0).value), self.on_state_timer)
        self.create_timer(0.1, self.on_teleop_timer)
        # Periodic republish of the volatile mask copy only (never /keepout_filter_mask).
        republish = float(p('keepout_republish_period', 2.0).value)
        if republish > 0.0:
            self.create_timer(republish, self.republish_mask_copy)
        map_republish = float(p('map_republish_period', 5.0).value)
        if map_republish > 0.0:
            self.create_timer(map_republish, self.republish_map_copy)
        self.initial_map_timer = (None if self.initial_map_done
                                  else self.create_timer(1.0, self.try_initial_map))
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
        text = ml.mission_state_json(self.state, self.mode, remaining, self.message,
                                    teleop=self.teleop_active())
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
            'maps': self.maps,
            'ok': self.map_ok,
            'message': self.map_message,
        })
        if force or text != self._last_map_json:
            self._last_map_json = text
            self.map_state_pub.publish(String(data=text))

    def set_map_state(self, ok, message, busy=None):
        # Two call sites: rclpy raises ValueError when one call site changes severity.
        if ok:
            self.get_logger().info(f'Map: {message}')
        else:
            self.get_logger().warning(f'Map: {message}')
        self.map_ok = ok
        self.map_message = message
        if busy is not None:
            self.map_busy = busy
        self.publish_map_state()

    def on_state_timer(self):
        now = time.monotonic()
        if self.pending_load is not None:
            self._try_send_load()
            if self.pending_load is not None and now - self.map_busy_since > self.stop_timeout:
                name = self.pending_load[1]
                self.pending_load = None
                self.set_map_state(False, f'LOAD {name}: the robot did not stop within '
                                   f'{self.stop_timeout:.0f} s', busy=False)
        elif self.map_busy and now - self.map_busy_since > self.service_timeout:
            # Not invalidated: slam_toolbox may still finish, and its late answer then
            # updates map_name and the keepout strokes to match what SLAM is running.
            self.set_map_state(False, f'slam_toolbox did not answer within '
                               f'{self.service_timeout:.0f} s (a late answer is still applied)',
                               busy=False)
        if self.pending_goal is not None and now - self.pending_since > self.cancel_timeout:
            self.get_logger().warning(
                f'Nav2 did not end the previous goal within {self.cancel_timeout:.0f} s: '
                'sending the new goal anyway')
            self._dispatch_goal(force=True)
        self.publish_mission_state(force=True)
        self.publish_map_state(force=True)

    # ---------------------------------------------------------------- inputs
    def on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.odom_pose = (p.position.x, p.position.y, ml.quaternion_to_yaw(q.x, q.y, q.z, q.w))
        t = msg.twist.twist
        moving = (math.hypot(t.linear.x, t.linear.y) > self.still_linear
                  or abs(t.angular.z) > self.still_angular)
        if moving:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = time.monotonic()
        if self.pending_load is not None:
            self._try_send_load()

    def on_map(self, msg):
        self.last_map = msg
        self.map_copy_pub.publish(msg)

    def republish_map_copy(self):
        if self.last_map is not None:
            self.map_copy_pub.publish(self.last_map)

    def teleop_active(self):
        return (self.last_teleop_time is not None
                and time.monotonic() - self.last_teleop_time < self.teleop_active_timeout)

    def on_teleop_timer(self):
        self.publish_mission_state()

    def on_teleop(self, msg):
        moving = (abs(msg.linear.x) > self.teleop_threshold
                  or abs(msg.angular.z) > self.teleop_threshold)
        if moving:
            self.last_teleop_time = time.monotonic()
            self.publish_mission_state()
        if moving and self.state == ml.NAVIGATING:
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
        self.pending_goal = None
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
        self.poses, self.remaining = list(poses), len(poses)
        self.set_mission(ml.NAVIGATING, message, mode)
        self.pending_goal = (self.generation, client, goal)
        self.pending_since = time.monotonic()
        self._dispatch_goal()

    def _dispatch_goal(self, force=False):
        """Send the pending goal once every earlier goal has ended in Nav2 (or on force)."""
        if self.pending_goal is None or (self.inflight and not force):
            return
        gen, client, goal = self.pending_goal
        self.pending_goal = None
        if gen != self.generation:
            return
        token = self.next_token
        self.next_token += 1
        self.inflight.add(token)
        future = client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_feedback(gen, fb))
        future.add_done_callback(lambda f: self._on_goal_response(gen, token, f))

    def _goal_ended(self, token):
        self.inflight.discard(token)
        self._dispatch_goal()

    def _on_goal_response(self, gen, token, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._goal_ended(token)
            if gen == self.generation:
                self.set_mission(ml.FAILED, 'goal rejected by Nav2')
            return
        # Track the result even for a stale goal: the next goal waits until it has ended.
        handle.get_result_async().add_done_callback(lambda f: self._on_result(gen, token, f))
        if gen != self.generation:
            handle.cancel_goal_async()   # replaced/paused/canceled before acceptance
            return
        self.goal_handle = handle

    def _on_feedback(self, gen, feedback_msg):
        if gen != self.generation or self.state != ml.NAVIGATING:
            return
        if self.mode == ml.MODE_PATH:
            n = int(feedback_msg.feedback.number_of_poses_remaining)
            if n > 0 and n != self.remaining:
                self.remaining = min(n, len(self.poses))
                self.publish_mission_state()

    def _on_result(self, gen, token, future):
        self._goal_ended(token)
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
        self.mask_msg = msg
        self.mask_pub.publish(msg)
        self.mask_copy_pub.publish(msg)
        self.get_logger().info(
            f'Keepout mask: {int((self.mask == km.KEEPOUT).sum())} cells, '
            f'{len(self.strokes)} strokes')

    def republish_mask_copy(self):
        if self.mask_msg is not None:
            self.mask_copy_pub.publish(self.mask_msg)

    # ---------------------------------------------------------------- maps
    def on_map_cmd(self, msg):
        self.maps = ms.list_maps(self.maps_dir)
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
        self.maps = ms.list_maps(self.maps_dir)
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
        self.pending_load = (op, name)
        self._try_send_load()
        if self.pending_load is not None:
            self.set_map_state(True, f'LOAD {name}: waiting for the robot to stop')
        return True

    def _robot_settled(self):
        return (self.still_since is not None
                and time.monotonic() - self.still_since >= self.settle_time)

    def _try_send_load(self):
        """Send the pending LOAD once the robot is stopped: slam_toolbox applies the given
        pose to the next scan it processes, not to the robot pose at request time."""
        op, name = self.pending_load
        if op != self.map_op:
            self.pending_load = None
            return
        if not self._robot_settled():
            return
        self.pending_load = None
        self.map_busy_since = time.monotonic()
        x, y, yaw = self.odom_pose
        req = DeserializePoseGraph.Request(
            filename=ms.map_base(self.maps_dir, name),
            match_type=DeserializePoseGraph.Request.START_AT_GIVEN_POSE,
            initial_pose=Pose2D(x=float(x), y=float(y), theta=float(yaw)))
        self.deserialize_cli.call_async(req).add_done_callback(
            lambda f: self._on_loaded(op, name, f))

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
        self.initial_map_timer.cancel()
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
