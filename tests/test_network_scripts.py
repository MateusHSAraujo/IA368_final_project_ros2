"""Tests for the network helper scripts and vr_robot_pkg.network (no ROS needed)."""
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'vr_robot_pkg'))

from vr_robot_pkg import network  # noqa: E402

SCRIPT = REPO / 'scripts' / 'get_windows_host_ip.sh'
PS1 = REPO / 'scripts' / 'setup_adb_reverse.ps1'
IPV4 = re.compile(r'^(\d{1,3}\.){3}\d{1,3}$')

ROUTE_SAMPLE = (
    'Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU\tWindow\tIRTT\n'
    'eth0\t00000000\t01E017AC\t0003\t0\t0\t0\t00000000\t0\t0\t0\n'
    'eth0\t00E017AC\t00000000\t0001\t0\t0\t0\t00F0FFFF\t0\t0\t0\n'
)
ROUTE_NO_DEFAULT = (
    'Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU\tWindow\tIRTT\n'
    'eth0\t00E017AC\t00000000\t0001\t0\t0\t0\t00F0FFFF\t0\t0\t0\n'
)


def _on_wsl():
    if Path('/proc/sys/fs/binfmt_misc/WSLInterop').exists():
        return True
    try:
        return 'microsoft' in Path('/proc/version').read_text().lower()
    except OSError:
        return False


needs_wsl = pytest.mark.skipif(not _on_wsl(), reason='not running under WSL')


def run_script(*args, env_extra=None, clear_host=False):
    env = dict(os.environ)
    if clear_host:
        env.pop('COPPELIA_HOST', None)
    env.update(env_extra or {})
    return subprocess.run(['bash', str(SCRIPT), *args], capture_output=True,
                          text=True, env=env, timeout=30)


@pytest.mark.parametrize('name', ['get_windows_host_ip.sh', 'sync_to_windows.sh'])
def test_bash_syntax(name):
    result = subprocess.run(['bash', '-n', str(REPO / 'scripts' / name)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


@needs_wsl
def test_script_prints_ipv4():
    result = run_script(clear_host=True)
    assert result.returncode == 0, result.stderr
    assert IPV4.match(result.stdout.strip())


def test_script_override():
    result = run_script(env_extra={'COPPELIA_HOST': '10.1.2.3'})
    assert result.returncode == 0
    assert result.stdout.strip() == '10.1.2.3'


def test_script_check_closed_port():
    result = run_script('--check', '1', env_extra={'COPPELIA_HOST': '127.0.0.1'})
    assert result.returncode == 2


def test_script_bad_option():
    assert run_script('--bogus').returncode == 3


def test_parse_default_gateway():
    assert network.parse_default_gateway(ROUTE_SAMPLE) == '172.23.224.1'
    assert network.parse_default_gateway(ROUTE_NO_DEFAULT) is None
    assert network.parse_default_gateway('') is None


def test_detect_override():
    assert network.detect_windows_host_ip(env={'COPPELIA_HOST': '10.0.0.5'}) == '10.0.0.5'


@needs_wsl
def test_detect_matches_script():
    result = run_script(clear_host=True)
    assert result.returncode == 0, result.stderr
    assert network.detect_windows_host_ip(env={}) == result.stdout.strip()


def test_is_tcp_reachable_closed_port():
    assert network.is_tcp_reachable('127.0.0.1', 1, timeout=0.5) is False


@pytest.mark.skipif(shutil.which('powershell.exe') is None, reason='powershell.exe not available')
def test_powershell_script_parses(tmp_path):
    temp = Path('/mnt/c/Users/mateu/AppData/Local/Temp')
    if not temp.is_dir():
        pytest.skip('Windows temp directory not reachable')
    copy = temp / 'ia368_setup_adb_reverse_parse.ps1'
    shutil.copy(PS1, copy)
    try:
        win_path = subprocess.run(['wslpath', '-w', str(copy)], capture_output=True,
                                  text=True, check=True).stdout.strip()
        cmd = ("$e=$null; [void][System.Management.Automation.Language.Parser]::ParseFile("
               f"'{win_path}', [ref]$null, [ref]$e); $e.Count")
        result = subprocess.run(['powershell.exe', '-NoProfile', '-Command', cmd],
                                capture_output=True, text=True, timeout=60)
    finally:
        copy.unlink(missing_ok=True)
    assert result.stdout.strip() == '0', result.stdout + result.stderr
