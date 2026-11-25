"""Network helpers to locate the Windows host (CoppeliaSim) from WSL2.

This module mirrors ``scripts/get_windows_host_ip.sh`` in pure Python (stdlib
only, no ROS imports). Resolution order:

1. ``COPPELIA_HOST`` environment variable (manual override).
2. ``127.0.0.1`` outside WSL (native Linux: CoppeliaSim runs on the same machine).
3. ``127.0.0.1`` when WSL runs in mirrored networking mode.
4. The default gateway of the WSL network (NAT mode), read from
   ``/proc/net/route``: this is the Windows host.
"""

import os
import socket
import subprocess
from typing import Mapping, Optional

_ROUTE_FILE = '/proc/net/route'
_VERSION_FILE = '/proc/version'


def parse_default_gateway(proc_net_route_text: str) -> Optional[str]:
    """Return the default gateway IPv4 from the text of /proc/net/route."""
    for line in proc_net_route_text.splitlines()[1:]:
        fields = line.split()
        if len(fields) < 3:
            continue
        destination, gateway = fields[1], fields[2]
        if destination != '00000000':
            continue
        try:
            value = int(gateway, 16)
        except ValueError:
            continue
        if value == 0:
            continue
        # The kernel prints the address as a little-endian hex word.
        return '.'.join(str((value >> shift) & 0xFF) for shift in (0, 8, 16, 24))
    return None


def is_wsl_kernel(proc_version_text: str) -> bool:
    """Return True if the text of /proc/version names a WSL (Microsoft) kernel."""
    return 'microsoft' in proc_version_text.lower()


def _running_in_wsl() -> bool:
    try:
        with open(_VERSION_FILE, 'r', encoding='utf-8') as version_file:
            return is_wsl_kernel(version_file.read())
    except OSError:
        return False


def _networking_mode() -> str:
    try:
        result = subprocess.run(
            ['wslinfo', '--networking-mode'],
            capture_output=True, text=True, timeout=2.0, check=False)
    except (OSError, subprocess.SubprocessError):
        return ''
    return result.stdout.strip()


def detect_windows_host_ip(env: Optional[Mapping[str, str]] = None) -> Optional[str]:
    """Return the CoppeliaSim host IPv4 (the Windows host from WSL2), or None if unknown."""
    if env is None:
        env = os.environ
    override = env.get('COPPELIA_HOST')
    if override:
        return override
    if not _running_in_wsl():
        return '127.0.0.1'
    if _networking_mode() == 'mirrored':
        return '127.0.0.1'
    try:
        with open(_ROUTE_FILE, 'r', encoding='utf-8') as route_file:
            return parse_default_gateway(route_file.read())
    except OSError:
        return None


def is_tcp_reachable(host: str, port: int, timeout: float = 2.0) -> bool:
    """Return True if a TCP connection to host:port succeeds within timeout."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False
