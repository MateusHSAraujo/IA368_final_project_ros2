#!/usr/bin/env python3
"""Live check of ROS-TCP-Endpoint (port 10000) speaking the ROS-TCP-Connector protocol.

Acts like Unity: connects, subscribes to a few topics and decodes what arrives, then
publishes /camera/enable through the endpoint. Needs the stack running.

    python3 -u tests/live/check_tcp_endpoint.py [--host 127.0.0.1] [--port 10000]

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

from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=10000)
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
    sock.close()
    sys.exit(0 if pose and grid else 1)


if __name__ == '__main__':
    main()
