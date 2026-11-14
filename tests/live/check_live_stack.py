#!/usr/bin/env python3
"""Live end-to-end check of the running stack (bridge + SLAM + Nav2 + keepout + camera).

Not collected by pytest: it needs CoppeliaSim and `ros2 launch vr_robot_pkg
vr_robot_bringup.launch.py` running. Each step prints PASS/FAIL with numbers.

    python3 -u tests/live/check_live_stack.py rates drive camera
    python3 -u tests/live/check_live_stack.py nav --goal 2.5 0.6 --goal 3.3 -1.5
    python3 -u tests/live/check_live_stack.py keepout --zone 3.0 0.5 0.4 --goal 2.5 -1.0
    python3 -u tests/live/check_live_stack.py map --save /tmp/map.png
"""

import argparse
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
        self.results = []

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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('steps', nargs='+', choices=['rates', 'drive', 'camera', 'nav', 'through', 'keepout', 'map'])
    parser.add_argument('--goal', nargs=2, type=float, action='append', default=[])
    parser.add_argument('--zone', nargs=3, type=float, help='keepout square: center x, center y, half size')
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
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
    print(f'{sum(node.results)}/{len(node.results)} checks passed')
    sys.exit(0 if all(node.results) else 1)


if __name__ == '__main__':
    main()
