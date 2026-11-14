"""ROS-free logic for the mission supervisor (Nav2 goal sequencing).

No rclpy or ROS message imports. The yaw <-> quaternion helpers are the ones
already defined in ``bridge_math`` (re-exported here under the mission names),
so both sides share a single implementation. Note that importing
``bridge_math`` also imports numpy and cv2.
"""
from __future__ import annotations

import json
import math

from vr_robot_pkg.bridge_math import quaternion_from_yaw as yaw_to_quaternion
from vr_robot_pkg.bridge_math import yaw_from_quaternion as quaternion_to_yaw

IDLE, NAVIGATING, PAUSED, SUCCEEDED, FAILED, CANCELED = (
    'IDLE', 'NAVIGATING', 'PAUSED', 'SUCCEEDED', 'FAILED', 'CANCELED')
MODE_GOAL, MODE_PATH = 'goal', 'path'
STATES = (IDLE, NAVIGATING, PAUSED, SUCCEEDED, FAILED, CANCELED)
MODES = (MODE_GOAL, MODE_PATH)
COMMANDS = ('PAUSE', 'RESUME', 'CANCEL')

_EPS = 1e-9


def _dist(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _heading(a, b) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def path_waypoints(points, min_spacing=0.3, start=None):
    """Thin a painted path into Nav2 waypoints.

    points: sequence of (x, y) in meters (map frame), in drawing order.
    start: optional (x, y) robot position; points closer than min_spacing to it
        at the beginning are skipped.
    Greedy: the reference is `start` if given, else the first point (which is
    then kept). A point is kept when its distance to the last kept point (or the
    reference) is >= min_spacing. The final input point is always the last
    waypoint: kept points that are closer than min_spacing to it are dropped
    and it is appended (so the last waypoint is the final point, and no
    consecutive waypoints end up closer than min_spacing). If the final point
    is itself closer than min_spacing to `start` and nothing else was kept,
    [final] is returned anyway.

    Returns a list of (x, y, yaw). The yaw of waypoint i points towards
    waypoint i+1; the last waypoint uses the direction from the previous one.
    A single waypoint uses the direction from `start` if given (and not
    coincident), else 0.0. Empty input -> [].
    """
    pts = [(float(p[0]), float(p[1])) for p in points]
    if not pts:
        return []
    final = pts[-1]
    if start is None:
        kept = [pts[0]]
        last = pts[0]
        candidates = pts[1:-1]
    else:
        start = (float(start[0]), float(start[1]))
        kept = []
        last = start
        candidates = pts[:-1]
    for p in candidates:
        if _dist(p, last) >= min_spacing - _EPS:
            kept.append(p)
            last = p
    # Drop kept points that would sit too close to the final point, then append it.
    while kept and _dist(kept[-1], final) < min_spacing - _EPS:
        kept.pop()
    kept.append(final)

    waypoints = []
    for i, (x, y) in enumerate(kept):
        if i + 1 < len(kept):
            yaw = _heading((x, y), kept[i + 1])
        elif len(kept) > 1:
            yaw = _heading(kept[i - 1], (x, y))
        elif start is not None and _dist(start, (x, y)) > _EPS:
            yaw = _heading(start, (x, y))
        else:
            yaw = 0.0
        waypoints.append((x, y, yaw))
    return waypoints


def remaining_poses(poses, n):
    """Poses still to visit after Nav2 feedback said n remain: poses[-n:], with n
    clamped to [1, len(poses)]. Empty poses -> []. n None -> all poses."""
    poses = list(poses)
    if not poses:
        return []
    if n is None:
        return poses
    n = min(max(int(n), 1), len(poses))
    return poses[-n:]


def parse_mission_cmd(text):
    """'pause ' -> 'PAUSE'. Returns one of 'PAUSE', 'RESUME', 'CANCEL' or None."""
    cmd = (text or '').strip().upper()
    return cmd if cmd in COMMANDS else None


def mission_state_json(state, mode=None, remaining=0, message=''):
    """JSON string with exactly the keys state, mode, remaining, message (in that order).

    state must be in STATES and mode None, 'goal' or 'path' (ValueError otherwise).
    """
    if state not in STATES:
        raise ValueError(f'unknown mission state: {state!r}')
    if mode is not None and mode not in MODES:
        raise ValueError(f'unknown mission mode: {mode!r}')
    return json.dumps({
        'state': state,
        'mode': mode,
        'remaining': int(remaining),
        'message': str(message),
    })
