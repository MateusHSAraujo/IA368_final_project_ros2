"""CoppeliaSim <-> ROS 2 bridge for the Pioneer P3-DX (/myRobot).

One node and one ZMQ Remote API client (single thread) replace the five class nodes
(lidar, tf, ground truth, velocity, camera). It publishes:

* ``/myRobot/scan``                  sensor_msgs/LaserScan (frame ``laser_link``)
* ``/myRobot/odom``                  nav_msgs/Odometry (ground truth, ``odom`` -> ``base_link``)
* ``/tf``                            ``odom`` -> ``base_link``
* ``/tf_static``                     ``base_link`` -> ``laser_link``, ``camera_link``, ``camera_optical_frame``
* ``/camera/image_raw/compressed``   sensor_msgs/CompressedImage (JPEG), only while enabled
* ``/hmi/robot_pose``                geometry_msgs/PoseStamped in ``map`` (``odom`` until SLAM is up)
* ``/myRobot/bridge_status``         std_msgs/String

and subscribes to ``cmd_vel_topic`` (geometry_msgs/Twist, with a watchdog) and
``/camera/enable`` (std_msgs/Bool).

While simulating, the ZMQ Remote API answers only ~1-2 requests per simulation step.
The bridge therefore makes ONE request per cycle: ``remoteExchange`` in the laser script
installed by ``scripts/coppelia/install_fast_laser.py`` sets the wheel velocities and
returns pose, velocity, the latest scan and (when asked) a JPEG camera frame. Without
that script (or while the simulation is stopped) it falls back to individual calls.

``odom`` is the CoppeliaSim world frame (ground truth), and slam_toolbox starts with
``map`` -> ``odom`` = identity, so ``map`` matches the CoppeliaSim/Unity origin.

With ``mock_mode:=true`` the robot is simulated in-process (unicycle kinematics in a
rectangular room), so the stack and the tests run without CoppeliaSim.
"""

from __future__ import annotations

import math
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
import zmq
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

from vr_robot_pkg import bridge_math as bm
from vr_robot_pkg.network import detect_windows_host_ip

# Rotation from the CoppeliaSim vision sensor frame (x left, y up, z forward) to the
# ROS camera body frame (x forward, y left, z up).
SENSOR_TO_CAMERA_LINK = (-0.5, -0.5, -0.5, 0.5)
# Standard rotation from camera_link to the optical frame (z forward, x right, y down).
CAMERA_LINK_TO_OPTICAL = (-0.5, 0.5, -0.5, 0.5)
# frontCamera pose relative to base_link (Ground_truth) as created in scene_house.ttt;
# used in mock mode and as a fallback if the sensor is missing in the scene.
DEFAULT_CAMERA_POSE = (0.2002, 0.0, 0.2187, 0.5417, 0.5417, 0.4545, 0.4545)
EXCHANGE_FUNCTION = 'remoteExchange'


def quat_multiply(a, b):
    """Hamilton product a * b of (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class SimDisconnected(Exception):
    """Raised when the Remote API connection is lost or not available."""


def make_state(pose, twist, running, scan=None, scan_seq=None, jpeg=None):
    """One robot sample: pose (x, y, z, qx, qy, qz, qw) and twist (vx, vy, wz) in the world frame."""
    return {'pose': tuple(float(v) for v in pose), 'twist': tuple(float(v) for v in twist),
            'running': running, 'scan': scan, 'scan_seq': scan_seq, 'jpeg': jpeg}


class CoppeliaBackend:
    """Access to /myRobot through the ZMQ Remote API. Every call may raise SimDisconnected."""

    def __init__(self, host: str, port: int, robot_path: str, timeout_ms: int, logger,
                 abort: threading.Event | None = None):
        self.host = host
        self.port = port
        self.robot_path = robot_path
        self.timeout_ms = timeout_ms
        self.log = logger
        self.client = None
        self.sim = None
        self.started_simulation = False
        self.camera_pose = DEFAULT_CAMERA_POSE
        self.exchange_script = None
        self.wheels_sent = None
        self.left = self.right = None
        # Set by the node on shutdown: no new Remote API call is started, so a callback
        # holding the backend lock finishes after at most one pending call.
        self.abort = abort if abort is not None else threading.Event()

    def _check_abort(self) -> None:
        if self.abort.is_set():
            raise SimDisconnected('bridge shutting down')

    # -- connection -------------------------------------------------------------------
    def connect(self, start_simulation: bool) -> None:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        client = RemoteAPIClient(host=self.host, port=self.port)
        client.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        client.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        client.socket.setsockopt(zmq.LINGER, 0)
        self.client = client

        def call(fn, *args):
            self._check_abort()
            return fn(*args)
        try:
            sim = call(client.require, 'sim')
            self.sim = sim
            self.robot = call(sim.getObject, self.robot_path)
            self.gt = call(sim.getObject, self.robot_path + '/Ground_truth')
            self.left = call(sim.getObject, self.robot_path + '/leftMotor')
            self.right = call(sim.getObject, self.robot_path + '/rightMotor')
            self.camera = call(sim.getObject, self.robot_path + '/frontCamera', {'noError': True})
            if self.camera != -1:
                self.camera_pose = tuple(
                    float(v) for v in call(sim.getObjectPose, self.camera, self.gt))
            else:
                self.log.warning(f'{self.robot_path}/frontCamera not found: camera disabled')
            self._find_exchange_script(call)
            if (start_simulation
                    and call(sim.getSimulationState) == sim.simulation_stopped):
                # Set first: if the call times out or connect fails afterwards, the
                # simulation may still have started and must be stopped on exit.
                self.started_simulation = True
                call(sim.startSimulation)
                self.log.info('Simulation started by the bridge')
        except Exception:
            self.close()
            raise

    def _find_exchange_script(self, call) -> None:
        sim = self.sim
        laser = call(sim.getObject, self.robot_path + '/LaserScanner_2D', {'noError': True})
        script = call(sim.getScript, sim.scripttype_childscript, laser) if laser != -1 else -1
        text = (call(sim.getScriptStringParam, script, sim.scriptstringparam_text)
                if script != -1 else '')
        if f'function {EXCHANGE_FUNCTION}' in text:
            self.exchange_script = script
        else:
            self.log.warning(
                f'The laser script has no {EXCHANGE_FUNCTION}(): using individual Remote API '
                'calls, which limits the bridge to a few Hz while simulating. Run '
                'scripts/coppelia/install_fast_laser.py on the scene.')

    def close(self) -> None:
        client, self.client, self.sim = self.client, None, None
        if client is None:
            return
        try:
            client.socket.close(linger=0)
            client.context.term()
        except Exception:
            pass
        # RemoteAPIClient.__del__ closes again later; closing twice is harmless in pyzmq.

    def _guard(self, fn, *args):
        if self.sim is None:
            raise SimDisconnected('not connected')
        self._check_abort()
        try:
            return fn(*args)
        except Exception as exc:  # zmq.Again (timeout), zmq errors, CoppeliaSim errors
            self.close()
            raise SimDisconnected(str(exc)) from exc

    # -- data -------------------------------------------------------------------------
    def simulation_running(self) -> bool:
        state = self._guard(self.sim.getSimulationState)
        return state != self.sim.simulation_stopped

    def exchange(self, wheels, want_scan: bool, want_camera: bool, jpeg_quality: int,
                 running: bool):
        """Send the wheel velocities (left, right) and return a state dict (see make_state)."""
        if self.sim is None:
            raise SimDisconnected('not connected')
        self._check_abort()
        if self.exchange_script is not None and running:
            try:
                out = self.sim.callScriptFunction(
                    EXCHANGE_FUNCTION, self.exchange_script, float(wheels[0]), float(wheels[1]),
                    bool(want_scan), bool(want_camera and self.camera != -1), int(jpeg_quality))
            except zmq.ZMQError as exc:  # includes zmq.Again (timeout)
                self.close()
                raise SimDisconnected(str(exc)) from exc
            except Exception as exc:
                message = str(exc)
                if 'not initialized' in message or 'already ended' in message:
                    return self._direct(wheels, False, False, jpeg_quality, running=False)
                self.log.warning(f'{EXCHANGE_FUNCTION}() failed ({message}); using individual calls')
                self.exchange_script = None
                return self._direct(wheels, want_scan, want_camera, jpeg_quality, running)
            self.wheels_sent = tuple(wheels)
            lin, ang = out['lin'], out['ang']
            jpeg = out.get('jpeg')
            return make_state(out['pose'], (lin[0], lin[1], ang[2]), True,
                              scan=out.get('scan'), scan_seq=out.get('scanSeq'),
                              jpeg=bytes(jpeg) if jpeg else None)
        return self._direct(wheels, want_scan, want_camera, jpeg_quality, running)

    def _direct(self, wheels, want_scan, want_camera, jpeg_quality, running):
        """Fallback with individual Remote API calls (fast while the simulation is stopped)."""
        sim = self.sim
        if running and tuple(wheels) != self.wheels_sent:
            self.set_wheels(*wheels)
        pose = self._guard(sim.getObjectPose, self.gt, -1)
        lin, ang = self._guard(sim.getObjectVelocity, self.gt)
        scan = None
        if want_scan and running:
            raw = self._guard(sim.getFloatArrayProperty, self.robot, 'signal.distSignal',
                              {'noError': True})
            scan = list(raw) if raw else None
        jpeg = None
        if want_camera and self.camera != -1:
            self._guard(sim.handleVisionSensor, self.camera)
            img, res = self._guard(sim.getVisionSensorImg, self.camera)
            jpeg = bm.encode_jpeg(bm.flip_rows(img, int(res[0]), int(res[1])), jpeg_quality)
        return make_state(pose, (lin[0], lin[1], ang[2]), running, scan=scan, jpeg=jpeg)

    def set_wheels(self, left: float, right: float) -> None:
        self._guard(self.sim.setJointTargetVelocity, self.left, float(left))
        self._guard(self.sim.setJointTargetVelocity, self.right, float(right))
        self.wheels_sent = (left, right)

    def set_display(self, enabled: bool) -> bool:
        """Set CoppeliaSim's display (window rendering); return the previous state."""
        previous = bool(self._guard(self.sim.getBoolParam, self.sim.boolparam_display_enabled))
        if previous != enabled:
            self._guard(self.sim.setBoolParam, self.sim.boolparam_display_enabled, enabled)
        return previous

    def shutdown(self, stop_simulation: bool, restore_display=None,
                 close_main: bool = True) -> None:
        """Stop the wheels, restore the display and optionally stop the simulation.

        Runs on a fresh connection: a Ctrl-C usually lands while the main socket waits for
        a reply, which leaves that REQ socket unusable. ``close_main=False`` leaves the main
        socket alone (another thread may still be blocked on it). Works on a backend that
        never connected (the motor handles are then looked up).
        """
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        if close_main:
            self.close()
        else:
            self.client, self.sim = None, None
        client = None
        try:
            client = RemoteAPIClient(host=self.host, port=self.port)
            client.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            client.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            client.socket.setsockopt(zmq.LINGER, 0)
            sim = client.require('sim')
            for handle, name in ((self.left, 'leftMotor'), (self.right, 'rightMotor')):
                if handle is None:
                    handle = sim.getObject(f'{self.robot_path}/{name}', {'noError': True})
                if handle != -1:
                    sim.setJointTargetVelocity(handle, 0.0)
            if restore_display is not None:
                sim.setBoolParam(sim.boolparam_display_enabled, bool(restore_display))
            if stop_simulation:
                sim.stopSimulation()
                self.log.info('Simulation stopped by the bridge')
        except Exception as exc:
            self.log.warning(f'Cleanup in CoppeliaSim failed: {exc}')
        finally:
            if client is not None:
                self.client = client
                self.sim = None
                self.close()


class MockBackend:
    """In-process unicycle robot in a rectangular room (bridge_math.MockWorld)."""

    def __init__(self, wheel_radius: float, wheel_base: float):
        self.world = bm.MockWorld()
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.r = wheel_radius
        self.L = wheel_base
        self.last = time.monotonic()
        self.camera_pose = DEFAULT_CAMERA_POSE
        self.started_simulation = False
        self.scan_seq = 0

    def connect(self, start_simulation: bool) -> None:
        self.last = time.monotonic()

    def close(self) -> None:
        pass

    def _step(self) -> None:
        now = time.monotonic()
        dt = min(now - self.last, 0.2)
        self.last = now
        x, y, yaw = bm.integrate_unicycle(self.x, self.y, self.yaw, self.v, self.w, dt)
        if self.world.is_free(x, y, radius=0.25):
            self.x, self.y, self.yaw = x, y, yaw
        else:  # bumped into something: rotate in place only
            self.yaw = yaw

    def simulation_running(self) -> bool:
        return True

    def exchange(self, wheels, want_scan, want_camera, jpeg_quality, running):
        self._step()
        self.set_wheels(*wheels)
        scan = None
        if want_scan:
            self.scan_seq += 1
            scan = self.world.scan(self.x, self.y, self.yaw)
        jpeg = None
        if want_camera:
            frame = bm.mock_camera_frame(320, 240, time.monotonic())
            jpeg = bm.encode_jpeg(bm.flip_rows(frame, 320, 240), jpeg_quality)
        pose = (self.x, self.y, 0.0) + bm.quaternion_from_yaw(self.yaw)
        twist = (self.v * math.cos(self.yaw), self.v * math.sin(self.yaw), self.w)
        return make_state(pose, twist, True, scan=scan, scan_seq=self.scan_seq, jpeg=jpeg)

    def set_wheels(self, left: float, right: float) -> None:
        self.v = self.r * (left + right) / 2.0
        self.w = self.r * (right - left) / self.L

    def set_display(self, enabled: bool) -> bool:
        return enabled

    def shutdown(self, stop_simulation: bool, restore_display=None,
                 close_main: bool = True) -> None:
        self.v = self.w = 0.0


class CoppeliaBridgeNode(Node):
    def __init__(self):
        super().__init__('coppelia_bridge')
        p = self.declare_parameter
        self.host_param = p('coppelia_host', '').value
        self.port = int(p('coppelia_port', 23000).value)
        self.mock = bool(p('mock_mode', False).value)
        self.robot_path = p('robot_path', '/myRobot').value
        self.start_sim = bool(p('start_simulation', True).value)
        self.stop_sim_on_exit = bool(p('stop_simulation_on_exit', True).value)
        # CoppeliaSim's window rendering of a large scene halves the simulation speed
        # (scene_house: ~0.44x vs ~0.87x real time). Unity is the visualization, so the
        # display is turned off while the bridge runs and restored on exit.
        self.coppelia_display = bool(p('coppelia_display', False).value)
        # One Remote API exchange per cycle; while simulating the effective rate is bounded
        # by the simulation step rate (one exchange per step).
        cycle_rate = float(p('cycle_rate', 20.0).value)
        self.scan_period = 1.0 / float(p('scan_rate', 10.0).value)
        self.camera_period = 1.0 / float(p('camera_rate', 10.0).value)
        self.robot_pose_period = 1.0 / float(p('robot_pose_rate', 10.0).value)
        self.camera_enabled = bool(p('camera_enabled', False).value)
        self.jpeg_quality = int(p('camera_jpeg_quality', 70).value)
        self.publish_raw = bool(p('publish_raw_image', False).value)
        cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel').value
        self.cmd_timeout = float(p('cmd_vel_timeout', 0.5).value)
        self.wheel_radius = float(p('wheel_radius', bm.WHEEL_RADIUS).value)
        self.wheel_base = float(p('wheel_base', bm.WHEEL_BASE).value)
        # The first simulation steps of a large scene can take several seconds.
        self.timeout_ms = int(p('remote_api_timeout_ms', 10000).value)
        self.reconnect_period = float(p('reconnect_period', 2.0).value)
        # Resolved once: detection runs a subprocess (wslinfo) and the host does not change.
        self.host = None if self.mock else (
            self.host_param or detect_windows_host_ip() or '127.0.0.1')

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/myRobot/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/myRobot/odom', 10)
        self.jpeg_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 2)
        self.raw_pub = self.create_publisher(Image, '/camera/image_raw', 2) if self.publish_raw else None
        self.pose_pub = self.create_publisher(PoseStamped, '/hmi/robot_pose', 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/myRobot/bridge_status', latched)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        # No spin_thread: tf2_ros would spin this whole node in a second executor (timers
        # included). The MultiThreadedExecutor serves the TF callbacks in the default group.
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions run in their own callback group so they are not starved by the
        # cycle timer (with a MultiThreadedExecutor, see main()); they only store values.
        inputs = ReentrantCallbackGroup()
        self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10, callback_group=inputs)
        self.create_subscription(Bool, '/camera/enable', self.on_camera_enable, 10,
                                 callback_group=inputs)

        # State
        self.backend = None
        self.connected = False
        self.sim_running = False
        # Kept across reconnects: stop the simulation on exit only if this node started it.
        self.sim_started_by_bridge = False
        self.display_to_restore = None
        self.status = ''
        self.last_connect_attempt = -1e9
        self.cmd = (0.0, 0.0)
        self.cmd_stamp = None
        self.last_scan_time = -1e9
        self.last_scan_seq = None
        self.last_camera_time = -1e9
        self.last_robot_pose_time = -1e9
        self.scan_msg = self._make_scan_template()
        self.stats = {'cycles': 0, 'exchange_s': 0.0, 'max_s': 0.0, 'since': time.monotonic()}
        self.stats_period = float(p('stats_period', 10.0).value)
        # The backend's ZMQ socket is not thread-safe. The timers below never overlap (same
        # mutually exclusive group), but on Ctrl-C the main thread runs shutdown_backend()
        # while a worker thread may still be inside an exchange: closing the socket under a
        # pending recv() aborts the process in libzmq (signaler.cpp assertion), skipping the
        # cleanup. Every backend use holds this lock; `closing` stops new uses.
        self.backend_lock = threading.Lock()
        self.closing = False
        self.abort = threading.Event()   # set with closing; checked before every Remote API call

        # Both timers use the backend: never run them concurrently.
        backend_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / cycle_rate, self.on_cycle, callback_group=backend_group)
        self.create_timer(1.0, self.on_status_timer, callback_group=backend_group)
        self.set_status('mock' if self.mock else 'disconnected')
        self.try_connect()

    # -- connection management ----------------------------------------------------------
    def set_status(self, status: str) -> None:
        if status != self.status:
            self.status = status
            self.status_pub.publish(String(data=status))
            self.get_logger().info(f'Bridge status: {status}')

    def try_connect(self) -> None:
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_period:
            return
        self.last_connect_attempt = now
        # A local reference: shutdown_backend() may clear self.backend from the main thread.
        if self.mock:
            backend = MockBackend(self.wheel_radius, self.wheel_base)
        else:
            backend = CoppeliaBackend(self.host, self.port, self.robot_path, self.timeout_ms,
                                      self.get_logger(), self.abort)
        self.backend = backend
        try:
            backend.connect(self.start_sim)
        except Exception as exc:
            self.sim_started_by_bridge |= backend.started_simulation
            if self.closing:
                return
            self.backend = None
            self.get_logger().warning(
                f'CoppeliaSim not available ({exc}); retrying every {self.reconnect_period:.0f} s. '
                'If CoppeliaSim is open but does not answer, click "Connectivity >> ZMQ remote '
                'API server@addonScript: abort execution" in CoppeliaSim.',
                throttle_duration_sec=30.0)
            return
        self.sim_started_by_bridge |= backend.started_simulation
        if self.closing:
            return
        self.connected = True
        if not self.mock:
            try:
                previous = backend.set_display(self.coppelia_display)
            except SimDisconnected as exc:
                self.on_disconnect(exc)
                return
            if self.display_to_restore is None:
                self.display_to_restore = previous
            self.get_logger().info(f'Connected to CoppeliaSim at {backend.host}:{self.port}')
        self.publish_static_transforms()
        self.update_sim_state()

    def on_disconnect(self, exc: Exception) -> None:
        if self.closing:
            return   # shutdown_backend() owns the backend and its socket from now on
        if self.connected:
            self.get_logger().error(f'Lost the CoppeliaSim connection: {exc}')
        self.connected = False
        self.sim_running = False
        if self.backend is not None:
            self.backend.close()
        self.backend = None
        self.set_status('disconnected')

    def update_sim_state(self, running=None) -> None:
        if running is None:
            try:
                running = self.backend.simulation_running()
            except SimDisconnected as exc:
                self.on_disconnect(exc)
                return
        self.sim_running = running
        if self.mock:
            self.set_status('mock')
        else:
            self.set_status('connected' if running else 'waiting-for-simulation')

    # -- static transforms --------------------------------------------------------------
    def _transform(self, parent, child, xyz, quat, stamp) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
            float(xyz[0]), float(xyz[1]), float(xyz[2]))
        q = bm.normalize_quaternion(quat)
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        return t

    def publish_static_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()
        cam = self.backend.camera_pose
        cam_link_q = quat_multiply(cam[3:7], SENSOR_TO_CAMERA_LINK)
        self.static_tf_pub.sendTransform([
            self._transform('base_link', 'laser_link', bm.LASER_OFFSET, bm.LASER_QUATERNION, stamp),
            self._transform('base_link', 'camera_link', cam[0:3], cam_link_q, stamp),
            self._transform('camera_link', 'camera_optical_frame', (0, 0, 0),
                            CAMERA_LINK_TO_OPTICAL, stamp),
        ])

    # -- callbacks --------------------------------------------------------------------
    def on_cmd_vel(self, msg: Twist) -> None:
        # Stamp first: current_wheels() reads cmd then cmd_stamp from another thread.
        self.cmd_stamp = time.monotonic()
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_camera_enable(self, msg: Bool) -> None:
        if msg.data != self.camera_enabled:
            self.camera_enabled = msg.data
            self.get_logger().info(f'Camera {"enabled" if msg.data else "disabled"}')

    def on_status_timer(self) -> None:
        # Also republished every second: Unity (ROS-TCP-Connector) subscribes with volatile
        # QoS and would miss a status that is only published on change.
        self.status_pub.publish(String(data=self.status))
        with self.backend_lock:
            if self.closing:
                return
            if not self.connected:
                self.try_connect()
            elif not self.sim_running:
                self.update_sim_state()  # cheap while stopped; detects a (re)started simulation

    def current_wheels(self):
        v, w = self.cmd
        if self.cmd_stamp is None or time.monotonic() - self.cmd_stamp > self.cmd_timeout:
            v = w = 0.0
        return bm.diff_drive_ik(v, w, self.wheel_radius, self.wheel_base)

    def on_cycle(self) -> None:
        with self.backend_lock:
            if not self.closing:
                self._cycle()

    def _cycle(self) -> None:
        if not self.connected:
            return
        now = time.monotonic()
        want_scan = self.sim_running and now - self.last_scan_time >= self.scan_period * 0.9
        want_camera = self.camera_enabled and now - self.last_camera_time >= self.camera_period * 0.9
        try:
            state = self.backend.exchange(self.current_wheels(), want_scan, want_camera,
                                          self.jpeg_quality, self.sim_running)
        except SimDisconnected as exc:
            self.on_disconnect(exc)
            return
        self.record_cycle(time.monotonic() - now)
        if state['running'] != self.sim_running:
            self.update_sim_state(state['running'])

        stamp = self.get_clock().now().to_msg()
        x, y, _z, qx, qy, qz, qw = state['pose']
        # Planar robot: base_link on the floor plane, yaw only.
        yaw = bm.yaw_from_quaternion(qx, qy, qz, qw)
        q = bm.quaternion_from_yaw(yaw)
        self.tf_pub.sendTransform(self._transform('odom', 'base_link', (x, y, 0.0), q, stamp))
        self.publish_odom(stamp, x, y, yaw, q, state['twist'])
        if now - self.last_robot_pose_time >= self.robot_pose_period * 0.9:
            self.last_robot_pose_time = now
            self.publish_robot_pose(stamp, x, y, q)
        # Only a new scan restarts the scan period: a repeated one (same simulation step)
        # is asked for again on the next cycle.
        if state['scan'] is not None and (state['scan_seq'] is None
                                          or state['scan_seq'] != self.last_scan_seq):
            self.last_scan_time = now
            self.last_scan_seq = state['scan_seq']
            self.publish_scan(stamp, state['scan'])
        if state['jpeg'] is not None:
            self.last_camera_time = now
            self.publish_camera(stamp, state['jpeg'])

    def record_cycle(self, exchange_s: float) -> None:
        st = self.stats
        st['cycles'] += 1
        st['exchange_s'] += exchange_s
        st['max_s'] = max(st['max_s'], exchange_s)
        elapsed = time.monotonic() - st['since']
        if self.stats_period > 0 and elapsed >= self.stats_period:
            self.get_logger().info(
                f'Bridge cycle {st["cycles"] / elapsed:.1f} Hz, '
                f'mean exchange {1000 * st["exchange_s"] / st["cycles"]:.0f} ms, '
                f'max {1000 * st["max_s"]:.0f} ms')
            self.stats = {'cycles': 0, 'exchange_s': 0.0, 'max_s': 0.0, 'since': time.monotonic()}

    # -- publishing -------------------------------------------------------------------
    def publish_odom(self, stamp, x, y, yaw, q, twist) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        bvx, bvy, bwz = bm.world_twist_to_body(twist[0], twist[1], twist[2], yaw)
        odom.twist.twist.linear.x = bvx
        odom.twist.twist.linear.y = bvy
        odom.twist.twist.angular.z = bwz
        self.odom_pub.publish(odom)

    def publish_robot_pose(self, stamp, x, y, q) -> None:
        msg = PoseStamped()
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            msg.header.stamp = tf.header.stamp
            msg.header.frame_id = 'map'
            msg.pose.position.x = tf.transform.translation.x
            msg.pose.position.y = tf.transform.translation.y
            msg.pose.orientation = tf.transform.rotation
        except Exception:
            # SLAM not running yet: odom == CoppeliaSim world == map at start.
            msg.header.stamp = stamp
            msg.header.frame_id = 'odom'
            msg.pose.position.x = x
            msg.pose.position.y = y
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = q
        self.pose_pub.publish(msg)

    def _make_scan_template(self) -> LaserScan:
        scan = LaserScan()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = bm.SCAN_ANGLE_MIN
        scan.angle_increment = bm.SCAN_ANGLE_INCREMENT
        scan.angle_max = bm.scan_angle_max()
        scan.range_min = bm.SCAN_RANGE_MIN
        scan.range_max = bm.SCAN_RANGE_MAX
        scan.scan_time = self.scan_period
        return scan

    def publish_scan(self, stamp, raw) -> None:
        if len(raw) != bm.SCAN_SAMPLES:
            self.get_logger().warning(
                f'Unexpected scan length {len(raw)} (expected {bm.SCAN_SAMPLES})',
                throttle_duration_sec=10.0)
            return
        msg = self.scan_msg
        msg.header.stamp = stamp
        msg.ranges = bm.convert_ranges(raw)
        self.scan_pub.publish(msg)

    def publish_camera(self, stamp, jpeg: bytes) -> None:
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_optical_frame'
        msg.format = 'jpeg'
        msg.data = jpeg
        self.jpeg_pub.publish(msg)
        if self.raw_pub is not None:
            bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return
            raw = Image()
            raw.header = msg.header
            raw.height, raw.width = bgr.shape[0], bgr.shape[1]
            raw.encoding = 'rgb8'
            raw.step = raw.width * 3
            raw.data = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes()
            self.raw_pub.publish(raw)

    # -- shutdown ---------------------------------------------------------------------
    def shutdown_backend(self) -> None:
        """Clean up in CoppeliaSim. Called from the main thread after spin() returns, while
        executor worker threads may still be running a callback: wait for the in-flight
        backend call before touching the socket. `abort` stops a callback from starting
        another Remote API call, so the wait is bounded by one call's timeout."""
        self.closing = True
        self.abort.set()
        acquired = self.backend_lock.acquire(timeout=self.timeout_ms / 1000.0 + 2.0)
        if not acquired:
            self.get_logger().warning(
                'A CoppeliaSim call is still pending: cleaning up without closing its socket')
        try:
            backend = self.backend
            if (backend is None and not self.mock
                    and (self.sim_started_by_bridge or self.display_to_restore is not None)):
                # Disconnected (e.g. one timed-out exchange) when the node stopped: the
                # simulation and the display still need restoring, over a fresh client.
                backend = CoppeliaBackend(self.host, self.port, self.robot_path,
                                          self.timeout_ms, self.get_logger())
            if backend is not None:
                started = self.sim_started_by_bridge or backend.started_simulation
                backend.shutdown(self.stop_sim_on_exit and started,
                                 self.display_to_restore, close_main=acquired)
                self.backend = None
            self.connected = False
        finally:
            if acquired:
                self.backend_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaBridgeNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # A second Ctrl-C (ros2 launch forwards SIGINT) must not abort the cleanup that
        # stops the robot, restores CoppeliaSim's display and stops the simulation.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown_backend()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
