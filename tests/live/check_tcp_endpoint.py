#!/usr/bin/env python3
"""Live check of ROS-TCP-Endpoint (port 10000) speaking the ROS-TCP-Connector protocol.

Acts like Unity: connects, subscribes to a few topics and decodes what arrives, then
publishes /camera/enable and a teleop Twist through the endpoint. Needs the stack running.

    python3 -u tests/live/check_tcp_endpoint.py [--host 127.0.0.1] [--port 10000] [--no-teleop]

Wire format (both directions): [u32 len][destination][u32 len][payload]. Destinations
starting with "__" are system commands (JSON payload); others are topics whose payload
is the CDR-serialized message.
"""

import argparse
import json
import socket
import struct
import sys
import time

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool


def send(sock, destination, payload: bytes):
    d = destination.encode()
    sock.sendall(struct.pack('<I', len(d)) + d + struct.pack('<I', len(payload)) + payload)


def send_command(sock, command, params):
    # The endpoint strips the last character of system command payloads: the Unity
    # connector terminates the JSON with a NUL byte.
    send(sock, command, json.dumps(params).encode() + b'\0')


def recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('endpoint closed the connection')
        buf += chunk
    return buf


def recv_message(sock):
    (n,) = struct.unpack('<I', recv_exact(sock, 4))
    destination = recv_exact(sock, n).decode()
    (m,) = struct.unpack('<I', recv_exact(sock, 4))
    return destination, recv_exact(sock, m)


def check_teleop(sock):
    """Act like Unity's joystick: publish /cmd_vel_teleop (angular.z = 0.5) at 10 Hz for 2 s through
    the endpoint and measure the odom angular velocity over the last second.
    NOTE: a nonzero teleop command pauses an active mission (mission_supervisor auto-pause)."""
    send_command(sock, '__publish', {'topic': '/cmd_vel_teleop', 'message_name': 'geometry_msgs/Twist',
                                     'queue_size': 1, 'latch': False})
    rclpy.init()
    node = rclpy.create_node('tcp_endpoint_teleop_check')
    odom = []
    node.create_subscription(Odometry, '/myRobot/odom', odom.append, 50)
    try:
        # A fresh node needs ~2-3 s of DDS discovery on the full stack before odom arrives; this
        # wait also gives the endpoint time to create its /cmd_vel_teleop publisher.
        end = time.time() + 10.0
        while time.time() < end and not odom:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not odom:
            print('[FAIL] /cmd_vel_teleop via endpoint: no /myRobot/odom within 10 s')
            return False
        twist = serialize_message(Twist(angular=Vector3(z=0.5)))
        t0, next_pub, mark = time.time(), time.time(), None
        while time.time() - t0 < 2.0:
            now = time.time()
            if now >= next_pub:
                send(sock, '/cmd_vel_teleop', twist)
                next_pub += 0.1
            if mark is None and now - t0 >= 1.0:
                mark = len(odom)
            rclpy.spin_once(node, timeout_sec=0.02)
        w = [abs(m.twist.twist.angular.z) for m in odom[mark:]] if mark is not None else []
        mean_w = sum(w) / len(w) if w else 0.0
        ok = bool(w) and mean_w > 0.25
        print(f'[{"PASS" if ok else "FAIL"}] /cmd_vel_teleop via endpoint: mean |w|={mean_w:.3f} rad/s '
              f'over {len(w)} odom samples (cmd 0.5, need > 0.25)')
        send(sock, '/cmd_vel_teleop', serialize_message(Twist()))
        print('[INFO] sent zero Twist on /cmd_vel_teleop through the endpoint')
        return ok
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=10000)
    parser.add_argument('--no-teleop', action='store_true',
                        help='skip the /cmd_vel_teleop check (it pauses an active mission)')
    args = parser.parse_args()
    topics = {'/hmi/robot_pose': ('geometry_msgs/PoseStamped', PoseStamped),
              '/map': ('nav_msgs/OccupancyGrid', OccupancyGrid)}
    sock = socket.create_connection((args.host, args.port), timeout=10)
    for topic, (name, _) in topics.items():
        send_command(sock, '__subscribe', {'topic': topic, 'message_name': name})
    send_command(sock, '__publish', {'topic': '/camera/enable', 'message_name': 'std_msgs/Bool',
                                     'queue_size': 1, 'latch': False})
    got, end = {}, time.time() + 20
    while time.time() < end and len(got) < len(topics):
        destination, payload = recv_message(sock)
        if destination in topics and destination not in got:
            got[destination] = deserialize_message(payload, topics[destination][1])
        elif destination.startswith('__'):
            print(f'system message {destination}: {payload[:120]!r}')
    pose = got.get('/hmi/robot_pose')
    grid = got.get('/map')
    print(f'[{"PASS" if pose else "FAIL"}] /hmi/robot_pose via endpoint: '
          + (f'frame={pose.header.frame_id} x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}' if pose else 'none'))
    print(f'[{"PASS" if grid else "FAIL"}] /map via endpoint: '
          + (f'{grid.info.width}x{grid.info.height} @ {grid.info.resolution:.2f} m' if grid else 'none'))
    time.sleep(0.5)  # let the endpoint create the publisher
    send(sock, '/camera/enable', serialize_message(Bool(data=False)))
    print('[INFO] published /camera/enable=false through the endpoint')
    teleop_ok = True if args.no_teleop else check_teleop(sock)
    sock.close()
    sys.exit(0 if pose and grid and teleop_ok else 1)


if __name__ == '__main__':
    main()
