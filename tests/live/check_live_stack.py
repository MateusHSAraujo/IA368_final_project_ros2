#!/usr/bin/env python3
"""Live end-to-end check of the running stack (bridge + SLAM + Nav2 + keepout + camera).

Not collected by pytest: it needs CoppeliaSim and `ros2 launch vr_robot_pkg
vr_robot_bringup.launch.py` running. Each step prints PASS/FAIL with numbers.

    python3 -u tests/live/check_live_stack.py rates drive camera
    python3 -u tests/live/check_live_stack.py nav --goal 2.5 0.6 --goal 3.3 -1.5
    python3 -u tests/live/check_live_stack.py keepout --zone 3.0 0.5 0.4 --goal 2.5 -1.0
    python3 -u tests/live/check_live_stack.py map --save /tmp/map.png

Mission supervisor (Stage 5) steps, driven only through the /hmi/* topics Unity uses:

    python3 -u tests/live/check_live_stack.py hmi_goal hmi_path hmi_pause hmi_keepout hmi_map \\
        --goal 2.4 -1.5 --goal 3.3 -1.7 --zone 2.4 -0.4 0.3
    python3 -u tests/live/check_live_stack.py hmi_map --map-name my_map

hmi_goal / hmi_pause use the first --goal (hmi_pause the farthest --goal from the robot, so
there is something left to drive); hmi_path goes through all --goal points; hmi_keepout takes
--zone x y r (a brush stroke of radius r); hmi_map takes --map-name (default live_check).
"""

import argparse
import json
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Checker(Node):
    def __init__(self):
        super().__init__('live_stack_checker')
        self.odom, self.scans, self.frames, self.status = [], [], [], None
        self.maps, self.costmap = [], None
        self.create_subscription(Odometry, '/myRobot/odom', self.odom.append, 50)
        self.create_subscription(LaserScan, '/myRobot/scan', self.scans.append, 20)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.frames.append, 20)
        self.create_subscription(String, '/myRobot/bridge_status', lambda m: setattr(self, 'status', m.data), LATCHED)
        self.create_subscription(OccupancyGrid, '/map', self.maps.append, LATCHED)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'costmap', m), LATCHED)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cam_pub = self.create_publisher(Bool, '/camera/enable', 10)
        self.zone_pub = self.create_publisher(PolygonStamped, '/keepout_mask_test', 10)
        self.mission_log, self.map_log, self.mask = [], [], None
        self.create_subscription(String, '/hmi/mission_state', lambda m: self._log(self.mission_log, m), LATCHED)
        self.create_subscription(String, '/hmi/map_state', lambda m: self._log(self.map_log, m), LATCHED)
        self.create_subscription(OccupancyGrid, '/keepout_filter_mask', lambda m: setattr(self, 'mask', m), LATCHED)
        self.hmi_goal_pub = self.create_publisher(PoseStamped, '/hmi/goal', 10)
        self.hmi_path_pub = self.create_publisher(Path, '/hmi/path', 10)
        self.hmi_cmd_pub = self.create_publisher(String, '/hmi/mission_cmd', 10)
        self.hmi_stroke_pub = self.create_publisher(PolygonStamped, '/hmi/keepout_stroke', 10)
        self.hmi_map_pub = self.create_publisher(String, '/hmi/map_cmd', 10)
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.results = []

    @staticmethod
    def _log(log, msg):
        try:
            log.append(json.loads(msg.data))
        except ValueError:
            pass

    def spin_for(self, seconds, every=None):
        end = time.time() + seconds
        while time.time() < end:
            if every:
                every()
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_subscribers(self, publisher, timeout=5.0):
        """Messages published before discovery matches a subscriber are lost."""
        end = time.time() + timeout
        while publisher.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return publisher.get_subscription_count() > 0

    def report(self, name, ok, detail):
        self.results.append(ok)
        print(f'[{"PASS" if ok else "FAIL"}] {name}: {detail}', flush=True)

    # -- steps ---------------------------------------------------------------------------
    def check_rates(self):
        end = time.time() + 5.0
        while self.status is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.report('bridge status', self.status == 'connected', f'status={self.status}')
        self.odom.clear(), self.scans.clear()
        self.spin_for(6.0)
        o = (len(self.odom) - 1) / (stamp_s(self.odom[-1]) - stamp_s(self.odom[0])) if len(self.odom) > 2 else 0
        s = (len(self.scans) - 1) / (stamp_s(self.scans[-1]) - stamp_s(self.scans[0])) if len(self.scans) > 2 else 0
        self.report('odom rate', o >= 8, f'{o:.1f} Hz')
        self.report('scan rate', s >= 5, f'{s:.1f} Hz')
        if self.scans:
            r = np.array(self.scans[-1].ranges)
            finite = r[np.isfinite(r)]
            self.report('scan content', len(r) == 452 and finite.size > 50,
                        f'{len(r)} beams, {finite.size} hits, min {finite.min():.2f} m' if finite.size else 'no hits')

    def pose(self):
        p = self.odom[-1].pose.pose
        return p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w)

    def check_drive(self, v=0.2, w=0.0, seconds=2.0):
        self.wait_for_subscribers(self.cmd_pub)
        self.spin_for(0.5)
        x0, y0, a0 = self.pose()
        t0 = time.time()
        self.spin_for(seconds, every=lambda: self.cmd_pub.publish(Twist(linear=self._vec(v), angular=self._vec(0, 0, w))))
        self.spin_for(1.0)  # watchdog stops the robot
        x1, y1, a1 = self.pose()
        dist = math.hypot(x1 - x0, y1 - y0)
        rot = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        self.report('drive', dist > 0.3 * v * seconds or abs(rot) > 0.3 * abs(w) * seconds,
                    f'cmd v={v} w={w} for {seconds:.1f} s wall -> moved {dist:.2f} m, turned {math.degrees(rot):.0f} deg')
        moved_after = self.odom[-1].twist.twist.linear.x
        self.report('watchdog stop', abs(moved_after) < 0.05, f'speed after stop {moved_after:.3f} m/s')

    @staticmethod
    def _vec(x=0.0, y=0.0, z=0.0):
        from geometry_msgs.msg import Vector3
        return Vector3(x=float(x), y=float(y), z=float(z))

    def check_camera(self, save=None):
        self.wait_for_subscribers(self.cam_pub)
        self.frames.clear()
        self.cam_pub.publish(Bool(data=True))
        self.spin_for(4.0)
        self.cam_pub.publish(Bool(data=True))
        n = len(self.frames)
        rate = (n - 1) / (stamp_s(self.frames[-1]) - stamp_s(self.frames[0])) if n > 2 else 0
        img = cv2.imdecode(np.frombuffer(bytes(self.frames[-1].data), np.uint8), cv2.IMREAD_COLOR) if n else None
        self.report('camera', img is not None and rate > 3,
                    f'{rate:.1f} Hz, {len(self.frames[-1].data) if n else 0} bytes, shape {None if img is None else img.shape}')
        if img is not None and save:
            cv2.imwrite(save, img)
            print(f'       saved {save}')
        self.cam_pub.publish(Bool(data=False))
        self.spin_for(1.0)
        self.frames.clear()
        self.spin_for(1.5)
        self.report('camera off', len(self.frames) == 0, f'{len(self.frames)} frames after disable')

    def _goal(self, x, y, yaw=0.0):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = float(x), float(y)
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return g

    def _run_action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=10):
            return None, 'action server not available'
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return None, 'goal rejected'
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=timeout)
        if not res.done():
            handle.cancel_goal_async()
            return None, f'timeout after {timeout} s'
        return res.result(), ''

    def check_nav(self, goals, through=False, timeout=180):
        self.spin_for(0.5)
        t0 = time.time()
        if through:
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self._goal(*g) for g in goals]
            result, err = self._run_action(ActionClient(self, NavigateThroughPoses, '/navigate_through_poses'), goal, timeout)
            label = f'navigate_through_poses {goals}'
        else:
            goal = NavigateToPose.Goal()
            goal.pose = self._goal(*goals[0])
            result, err = self._run_action(ActionClient(self, NavigateToPose, '/navigate_to_pose'), goal, timeout)
            label = f'navigate_to_pose {goals[0]}'
        self.spin_for(0.5)
        x, y, _ = self.pose()
        gx, gy = goals[-1][0], goals[-1][1]
        status = result.status if result else None
        self.report(label, status == 4 and math.hypot(x - gx, y - gy) < 0.5,
                    f'status={status} {err} final=({x:.2f}, {y:.2f}) error={math.hypot(x - gx, y - gy):.2f} m '
                    f'in {time.time() - t0:.0f} s')

    def check_keepout(self, cx, cy, half, goal):
        poly = PolygonStamped()
        poly.header.frame_id = 'map'
        poly.polygon.points = [Point32(x=float(cx + dx), y=float(cy + dy), z=0.0)
                               for dx, dy in ((-half, -half), (half, -half), (half, half), (-half, half))]
        if not self.wait_for_subscribers(self.zone_pub):
            self.report('keepout', False, 'keepout_mask_node is not subscribed to /keepout_mask_test')
            return
        self.zone_pub.publish(poly)
        self.spin_for(3.0)
        cost = self._cost_at(cx, cy)
        self.report('keepout in global costmap', cost is not None and cost >= 99, f'cost at zone center = {cost}')
        g = ComputePathToPose.Goal()
        g.goal = self._goal(*goal)
        result, err = self._run_action(ActionClient(self, ComputePathToPose, '/compute_path_to_pose'), g, 20)
        path = result.result.path.poses if result else []
        inside = [p for p in path if abs(p.pose.position.x - cx) < half - 0.03 and abs(p.pose.position.y - cy) < half - 0.03]
        self.report('plan avoids keepout', bool(path) and not inside,
                    f'{len(path)} poses, {len(inside)} inside the zone {err}')
        clear = PolygonStamped()
        clear.header.frame_id = 'map'
        self.zone_pub.publish(clear)
        self.spin_for(1.0)

    def _cost_at(self, x, y):
        m = self.costmap
        if m is None:
            return None
        i = int((x - m.info.origin.position.x) / m.info.resolution)
        j = int((y - m.info.origin.position.y) / m.info.resolution)
        if 0 <= i < m.info.width and 0 <= j < m.info.height:
            return m.data[j * m.info.width + i]
        return None

    def check_map(self, save=None):
        self.spin_for(2.0)
        if not self.maps:
            self.report('map', False, 'no /map received')
            return
        m = self.maps[-1]
        data = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        known = int((data >= 0).sum())
        self.report('map', known > 0, f'{m.info.width}x{m.info.height} cells, origin '
                    f'({m.info.origin.position.x:.2f}, {m.info.origin.position.y:.2f}), '
                    f'{known * m.info.resolution ** 2:.1f} m^2 known')
        if save:
            img = np.full(data.shape, 205, np.uint8)
            img[data == 0] = 254
            img[data >= 65] = 0
            cv2.imwrite(save, np.flipud(img))
            print(f'       saved {save}')

    # -- HMI (mission supervisor) steps ---------------------------------------------------
    def _wait_log(self, log, pred, timeout, start=0):
        """Spin until a log entry at index >= start satisfies pred. Returns (index, entry) or (None, None)."""
        end, i = time.time() + timeout, start
        while True:
            while i < len(log):
                if pred(log[i]):
                    return i, log[i]
                i += 1
            if time.time() >= end:
                return None, None
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_mission(self, state, timeout, start=0, mode=None, msg_has=None):
        return self._wait_log(
            self.mission_log, lambda m: m.get('state') == state and (mode is None or m.get('mode') == mode)
            and (msg_has is None or msg_has in m.get('message', '')), timeout, start)

    def _hmi_ready(self, *pubs):
        """Subscribers of every publisher matched (and one mission_state received)."""
        ok = all(self.wait_for_subscribers(p) for p in pubs)
        self.spin_for(1.0)
        if not self.odom:
            self.spin_for(2.0)
        return ok

    def _final_error(self, goal, t0):
        x, y, _ = self.pose()
        err = math.hypot(x - goal[0], y - goal[1])
        return err, f'final=({x:.2f}, {y:.2f}) error={err:.2f} m in {time.time() - t0:.0f} s'

    def check_hmi_goal(self, goal, timeout=300):
        if not self._hmi_ready(self.hmi_goal_pub):
            self.report('hmi_goal', False, 'mission_supervisor is not subscribed to /hmi/goal')
            return
        mark, t0 = len(self.mission_log), time.time()
        self.hmi_goal_pub.publish(self._goal(*goal))
        i, _ = self._wait_mission('NAVIGATING', 20, mark, mode='goal')
        self.report('hmi_goal NAVIGATING (mode goal)', i is not None, f'goal {goal}')
        j, last = self._wait_mission('SUCCEEDED', timeout, (i if i is not None else mark) + 1)
        self.spin_for(0.5)
        err, detail = self._final_error(goal, t0)
        self.report(f'hmi_goal SUCCEEDED {goal}', j is not None and err < 0.5,
                    f'state={self.mission_log[-1] if self.mission_log else None} {detail}')

    def check_hmi_path(self, goals, timeout=420):
        if not self._hmi_ready(self.hmi_path_pub):
            self.report('hmi_path', False, 'mission_supervisor is not subscribed to /hmi/path')
            return
        x0, y0, _ = self.pose()
        pts, (px, py) = [], (x0, y0)
        for gx, gy in [(g[0], g[1]) for g in goals]:
            n = max(1, int(math.hypot(gx - px, gy - py) / 0.05))
            pts += [(px + (gx - px) * k / n, py + (gy - py) * k / n) for k in range(1, n + 1)]
            px, py = gx, gy
        path = Path()
        path.header.frame_id = 'map'
        for k, (x, y) in enumerate(pts):
            ps = self._goal(x, y)
            nx, ny = pts[min(k + 1, len(pts) - 1)]
            yaw = math.atan2(ny - y, nx - x) if (nx, ny) != (x, y) else 0.0
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(ps)
        mark, t0 = len(self.mission_log), time.time()
        self.hmi_path_pub.publish(path)
        i, _ = self._wait_mission('NAVIGATING', 20, mark, mode='path')
        self.report('hmi_path NAVIGATING (mode path)', i is not None, f'{len(pts)} poses via {goals}')
        j, _ = self._wait_mission('SUCCEEDED', timeout, (i if i is not None else mark) + 1)
        self.spin_for(0.5)
        err, detail = self._final_error(goals[-1], t0)
        self.report('hmi_path SUCCEEDED', j is not None and err < 0.5,
                    f'state={self.mission_log[-1] if self.mission_log else None} {detail}')

    def _speed(self):
        t = self.odom[-1].twist.twist.linear
        return math.hypot(t.x, t.y)

    def _start_mission_and_move(self, goal):
        """Send /hmi/goal, wait for NAVIGATING and >= 0.2 m of motion. Returns the NAVIGATING log index."""
        x0, y0, _ = self.pose()
        mark = len(self.mission_log)
        self.hmi_goal_pub.publish(self._goal(*goal))
        i, _ = self._wait_mission('NAVIGATING', 20, mark, mode='goal')
        end = time.time() + 60
        while i is not None and math.hypot(self.pose()[0] - x0, self.pose()[1] - y0) < 0.2 and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        moved = math.hypot(self.pose()[0] - x0, self.pose()[1] - y0)
        return i, moved

    def check_hmi_pause(self, goals):
        if not self._hmi_ready(self.hmi_goal_pub, self.hmi_cmd_pub, self.teleop_pub):
            self.report('hmi_pause', False, 'mission_supervisor is not subscribed (goal/mission_cmd/cmd_vel_teleop)')
            return
        x, y, _ = self.pose()
        goal = max(goals, key=lambda g: math.hypot(g[0] - x, g[1] - y))  # something left to drive
        # PAUSE / RESUME
        i, moved = self._start_mission_and_move(goal)
        self.report('hmi_pause mission running', i is not None and moved >= 0.2, f'goal {goal}, moved {moved:.2f} m')
        mark = len(self.mission_log)
        self.hmi_cmd_pub.publish(String(data='PAUSE'))
        j, _ = self._wait_mission('PAUSED', 5, mark)
        self.report('hmi_pause PAUSED', j is not None, 'state after PAUSE')
        self.spin_for(2.0)
        self.report('hmi_pause robot stopped', self._speed() < 0.05, f'speed {self._speed():.3f} m/s')
        mark, t0 = len(self.mission_log), time.time()
        self.hmi_cmd_pub.publish(String(data='RESUME'))
        j, _ = self._wait_mission('NAVIGATING', 10, mark)
        self.report('hmi_pause RESUME -> NAVIGATING', j is not None, 'state after RESUME')
        k, _ = self._wait_mission('SUCCEEDED', 300, (j if j is not None else mark) + 1)
        self.spin_for(0.5)
        err, detail = self._final_error(goal, t0)
        self.report('hmi_pause resumed mission SUCCEEDED', k is not None and err < 0.5, detail)
        # teleop auto-pause, then CANCEL (go back to the farthest goal from the new position)
        x, y, _ = self.pose()
        goal = max(goals, key=lambda g: math.hypot(g[0] - x, g[1] - y))
        i, moved = self._start_mission_and_move(goal)
        self.report('hmi_pause mission running (teleop)', i is not None and moved >= 0.2, f'goal {goal}, moved {moved:.2f} m')
        mark = len(self.mission_log)
        self.spin_for(0.5, every=lambda: (self.teleop_pub.publish(Twist(linear=self._vec(0.1))), time.sleep(0.1)))
        j, st = self._wait_mission('PAUSED', 5, mark, msg_has='teleop')
        self.report('hmi_pause teleop -> PAUSED', j is not None, f'state={st}')
        mark = len(self.mission_log)
        self.hmi_cmd_pub.publish(String(data='CANCEL'))
        j, st = self._wait_mission('CANCELED', 10, mark)
        self.report('hmi_pause CANCEL -> CANCELED', j is not None, f'state={st}')
        self.teleop_pub.publish(Twist())
        self.cmd_pub.publish(Twist())
        self.spin_for(1.0)

    def _mask_near(self, cx, cy, radius):
        """Number of 100-cells of the latest keepout mask within `radius` of (cx, cy), and the total of 100-cells."""
        m = self.mask
        if m is None:
            return 0, 0
        d = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        jj, ii = np.nonzero(d == 100)
        xs = m.info.origin.position.x + (ii + 0.5) * m.info.resolution
        ys = m.info.origin.position.y + (jj + 0.5) * m.info.resolution
        return int((np.hypot(xs - cx, ys - cy) <= radius).sum()), int(jj.size)

    def check_hmi_keepout(self, cx, cy, r):
        if not self._hmi_ready(self.hmi_stroke_pub):
            self.report('hmi_keepout', False, 'mission_supervisor is not subscribed to /hmi/keepout_stroke')
            return
        stroke = PolygonStamped()
        stroke.header.frame_id = 'map'
        stroke.polygon.points = [Point32(x=float(cx), y=float(cy), z=float(r))]
        self.hmi_stroke_pub.publish(stroke)
        end, near, cost = time.time() + 10, 0, None
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            near, _ = self._mask_near(cx, cy, r + 0.21)
            cost = self._cost_at(cx, cy)
            if near > 0 and cost is not None and cost >= 99:
                break
        self.report('hmi_keepout mask', near > 0, f'{near} cells == 100 within {r + 0.21:.2f} m of ({cx}, {cy})')
        self.report('hmi_keepout in global costmap', cost is not None and cost >= 99, f'cost at stroke center = {cost}')
        clear = PolygonStamped()
        clear.header.frame_id = 'map'
        self.hmi_stroke_pub.publish(clear)
        end, total = time.time() + 10, -1
        while time.time() < end and total != 0:
            rclpy.spin_once(self, timeout_sec=0.05)
            total = self._mask_near(cx, cy, 0.0)[1]
        self.report('hmi_keepout cleared', self.mask is not None and total == 0, f'{total} cells == 100 after empty stroke')

    def _map_cmd(self, cmd, pred, timeout):
        """Publish a map command, wait for BUSY (if seen) then a READY state satisfying pred."""
        mark = len(self.map_log)
        self.hmi_map_pub.publish(String(data=cmd))
        b, _ = self._wait_log(self.map_log, lambda m: m.get('state') == 'BUSY', 10, mark)
        return self._wait_log(self.map_log, lambda m: m.get('state') == 'READY' and pred(m), timeout,
                              (b + 1) if b is not None else mark)

    def check_hmi_map(self, name):
        if not self._hmi_ready(self.hmi_map_pub):
            self.report('hmi_map', False, 'mission_supervisor is not subscribed to /hmi/map_cmd')
            return
        t0 = time.time()
        i, st = self._map_cmd(f'SAVE {name}', lambda m: m.get('ok') is True and name in m.get('maps', []), 60)
        self.report(f'hmi_map SAVE {name}', i is not None, f'{time.time() - t0:.0f} s, state={st or self.map_log[-1:]}')
        self.maps.clear()
        t0 = time.time()
        i, st = self._map_cmd(f'LOAD {name}', lambda m: m.get('ok') is True and m.get('map') == name, 60)
        self.report(f'hmi_map LOAD {name}', i is not None, f'{time.time() - t0:.0f} s, state={st or self.map_log[-1:]}')
        end = time.time() + 15
        while not self.maps and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        m = self.maps[-1] if self.maps else None
        self.report('hmi_map /map after LOAD', m is not None and m.info.width * m.info.height > 0,
                    f'{m.info.width}x{m.info.height} cells' if m else 'no /map within 15 s')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('steps', nargs='+', choices=['rates', 'drive', 'camera', 'nav', 'through', 'keepout', 'map',
                                              'hmi_goal', 'hmi_path', 'hmi_pause', 'hmi_keepout', 'hmi_map'])
    parser.add_argument('--goal', nargs=2, type=float, action='append', default=[])
    parser.add_argument('--zone', nargs=3, type=float, help='keepout square: center x, center y, half size')
    parser.add_argument('--map-name', default='live_check', help='map name for hmi_map')
    parser.add_argument('--save', help='image path for camera/map')
    args = parser.parse_args()
    rclpy.init()
    node = Checker()
    try:
        for step in args.steps:
            if step == 'rates':
                node.check_rates()
            elif step == 'drive':
                node.check_drive()
            elif step == 'camera':
                node.check_camera(args.save)
            elif step == 'nav':
                for g in args.goal:
                    node.check_nav([g])
            elif step == 'through':
                node.check_nav(args.goal, through=True)
            elif step == 'keepout':
                node.check_keepout(*args.zone, goal=args.goal[0])
            elif step == 'map':
                node.check_map(args.save)
            elif step == 'hmi_goal':
                node.check_hmi_goal(args.goal[0])
            elif step == 'hmi_path':
                node.check_hmi_path(args.goal)
            elif step == 'hmi_pause':
                node.check_hmi_pause(args.goal)
            elif step == 'hmi_keepout':
                node.check_hmi_keepout(*args.zone)
            elif step == 'hmi_map':
                node.check_hmi_map(args.map_name)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    print(f'{sum(node.results)}/{len(node.results)} checks passed')
    sys.exit(0 if all(node.results) else 1)


if __name__ == '__main__':
    main()
