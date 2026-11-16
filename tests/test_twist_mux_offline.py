"""twist_mux configuration checks and a live run of the real twist_mux node.

Part A is static (YAML/text only). Part B starts the real twist_mux executable on a
private DDS domain (so it never clashes with a live stack on domain 0) and checks
the arbitration between Nav2 (cmd_vel_nav_safe) and teleop (cmd_vel_teleop).
"""
import os
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'vr_robot_pkg'
MUX_YAML = PKG / 'config' / 'twist_mux.yaml'
NAV2_YAML = PKG / 'config' / 'nav2_params_jazzy.yaml'
LAUNCH = PKG / 'launch' / 'vr_robot_bringup.launch.py'
DOMAIN = 87
BRIDGE_WATCHDOG = 0.5


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _mux_params():
    return _load(MUX_YAML)['twist_mux']['ros__parameters']


# ----------------------------------------------------------------- A. static

def test_collision_monitor_output_feeds_mux_navigation_input():
    nav2 = _load(NAV2_YAML)
    out = nav2['collision_monitor']['ros__parameters']['cmd_vel_out_topic']
    assert out == _mux_params()['topics']['navigation']['topic']


def test_mux_priorities_and_timeouts():
    p = _mux_params()
    nav, tele = p['topics']['navigation'], p['topics']['teleop']
    assert tele['priority'] > nav['priority']
    assert tele['topic'] == 'cmd_vel_teleop'
    assert p['use_stamped'] is False
    for t in (nav, tele):
        assert 0 < t['timeout'] <= BRIDGE_WATCHDOG


def test_launch_runs_twist_mux_remapped_to_cmd_vel():
    text = LAUNCH.read_text()
    assert re.search(r"package\s*=\s*['\"]twist_mux['\"]", text)
    assert re.search(r"\(\s*['\"]cmd_vel_out['\"]\s*,\s*['\"]/cmd_vel['\"]\s*\)", text)


# ------------------------------------------------------------------- B. live

MUX_EXE = '/opt/ros/jazzy/lib/twist_mux/twist_mux'
if not os.path.exists(MUX_EXE):
    MUX_EXE = shutil.which('twist_mux')

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402


class Rig:
    def __init__(self, ctx):
        self.node = rclpy.create_node('twist_mux_test_client', context=ctx)
        self.exec = SingleThreadedExecutor(context=ctx)
        self.exec.add_node(self.node)
        self.nav_pub = self.node.create_publisher(Twist, 'cmd_vel_nav_safe', 10)
        self.tele_pub = self.node.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.rx = []  # (time, linear.x)
        self.node.create_subscription(
            Twist, '/cmd_vel', lambda m: self.rx.append((time.monotonic(), m.linear.x)), 10)

    def spin_for(self, duration, nav=None, teleop=None, rate=10.0):
        """Spin for `duration` s, publishing nav/teleop linear.x at `rate` Hz."""
        end = time.monotonic() + duration
        next_pub = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_pub:
                for pub, val in ((self.nav_pub, nav), (self.tele_pub, teleop)):
                    if val is not None:
                        msg = Twist()
                        msg.linear.x = val
                        pub.publish(msg)
                next_pub = now + 1.0 / rate
            self.exec.spin_once(timeout_sec=0.01)


@pytest.fixture(scope='module')
def rig():
    if MUX_EXE is None:
        pytest.skip('twist_mux executable not installed')
    ctx = Context()
    rclpy.init(context=ctx, domain_id=DOMAIN)
    env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
    # The node keeps its default name: the YAML parameters are under 'twist_mux'.
    proc = subprocess.Popen(
        [MUX_EXE, '--ros-args', '--params-file', str(MUX_YAML),
         '-r', 'cmd_vel_out:=/cmd_vel'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    r = Rig(ctx)
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            r.exec.spin_once(timeout_sec=0.1)
            if (r.nav_pub.get_subscription_count() > 0
                    and r.tele_pub.get_subscription_count() > 0):
                break
        else:
            pytest.fail('twist_mux did not appear on the private domain')
        time.sleep(0.5)  # let the /cmd_vel subscription match the mux publisher
        yield r
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        r.exec.shutdown()
        r.node.destroy_node()
        rclpy.shutdown(context=ctx)


def test_nav_passes_through(rig):
    rig.spin_for(1.0, nav=1.0)  # also lets the mux settle
    rig.rx.clear()
    rig.spin_for(1.0, nav=1.0)
    assert len(rig.rx) >= 5
    assert all(v == 1.0 for _, v in rig.rx)


def test_teleop_overrides_nav(rig):
    rig.spin_for(0.3, nav=1.0)
    rig.rx.clear()
    rig.spin_for(1.5, nav=1.0, teleop=2.0)
    vals = [v for _, v in rig.rx]
    assert len(vals) >= 5
    assert vals[-1] == 2.0
    assert set(vals[2:]) == {2.0}, vals


def test_nav_resumes_after_teleop_stops(rig):
    rig.spin_for(0.6, nav=1.0, teleop=2.0)
    assert rig.rx and rig.rx[-1][1] == 2.0
    last_teleop = time.monotonic()
    rig.rx.clear()
    # nav keeps publishing; teleop stopped
    rig.spin_for(1.2, nav=1.0)
    nav_times = [t for t, v in rig.rx if v == 1.0]
    assert nav_times, 'nav output never resumed'
    first_nav = nav_times[0] - last_teleop
    assert first_nav >= 0.3, f'nav resumed too early ({first_nav:.2f}s)'
    assert first_nav <= 1.2, f'nav resumed too late ({first_nav:.2f}s)'
    assert not any(v == 2.0 for t, v in rig.rx if t > nav_times[0])
