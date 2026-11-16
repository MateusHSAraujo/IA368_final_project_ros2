import json
import math

import pytest

from vr_robot_pkg import mission_logic as ml


def _line(x0, x1, step, y=0.0):
    n = round((x1 - x0) / step)
    return [(x0 + i * step, y) for i in range(n + 1)]


def _consecutive_ok(wps, min_spacing):
    for a, b in zip(wps, wps[1:]):
        assert math.hypot(b[0] - a[0], b[1] - a[1]) >= min_spacing - 1e-9


# --- path_waypoints ---------------------------------------------------------

def test_path_waypoints_empty():
    assert ml.path_waypoints([]) == []


def test_path_waypoints_straight_line():
    pts = _line(0.0, 1.0, 0.05)
    wps = ml.path_waypoints(pts, min_spacing=0.3)
    xs = [w[0] for w in wps]
    assert wps[-1][:2] == pytest.approx(pts[-1])
    assert xs == pytest.approx([0.0, 0.3, 0.6, 1.0])
    _consecutive_ok(wps, 0.3)
    # Every waypoint except the last is at least min_spacing from the final point.
    for w in wps[:-1]:
        assert math.hypot(pts[-1][0] - w[0], pts[-1][1]) >= 0.3 - 1e-9


def test_path_waypoints_yaw_along_x_is_zero():
    wps = ml.path_waypoints(_line(0.0, 1.0, 0.05))
    for w in wps:
        assert w[2] == pytest.approx(0.0)


def test_path_waypoints_yaw_along_y_is_half_pi():
    pts = [(0.0, 0.05 * i) for i in range(21)]
    wps = ml.path_waypoints(pts)
    assert wps[0][2] == pytest.approx(math.pi / 2)
    assert wps[-1][2] == pytest.approx(math.pi / 2)


def test_path_waypoints_start_skips_leading_points():
    start = (0.2, 0.0)
    wps = ml.path_waypoints(_line(0.0, 1.0, 0.05), min_spacing=0.3, start=start)
    assert wps[0][0] == pytest.approx(0.5)
    assert wps[-1][:2] == pytest.approx((1.0, 0.0))
    for w in wps:
        assert math.hypot(w[0] - start[0], w[1] - start[1]) >= 0.3 - 1e-9
    _consecutive_ok(wps, 0.3)


def test_path_waypoints_tiny_path_near_start_returns_final():
    pts = [(0.0, 0.0), (0.05, 0.0), (0.1, 0.0)]
    wps = ml.path_waypoints(pts, min_spacing=0.3, start=(0.0, 0.0))
    assert len(wps) == 1
    assert wps[0][:2] == pytest.approx((0.1, 0.0))


def test_path_waypoints_single_point_no_start():
    assert ml.path_waypoints([(2.0, 3.0)]) == [(2.0, 3.0, 0.0)]


def test_path_waypoints_single_point_uses_start_direction():
    wps = ml.path_waypoints([(0.0, 2.0)], start=(0.0, -1.0))
    assert len(wps) == 1
    assert wps[0][2] == pytest.approx(math.pi / 2)


def test_path_waypoints_single_point_coincident_with_start_yaw_zero():
    wps = ml.path_waypoints([(1.0, 1.0)], start=(1.0, 1.0))
    assert wps == [(1.0, 1.0, 0.0)]


def test_path_waypoints_l_shape_yaw_changes_at_corner():
    horizontal = _line(0.0, 1.0, 0.1, y=0.0)
    vertical = [(1.0, 0.1 * i) for i in range(1, 11)]
    pts = horizontal + vertical
    wps = ml.path_waypoints(pts, min_spacing=0.3)
    assert wps[0][2] == pytest.approx(0.0)
    assert wps[-1][:2] == pytest.approx((1.0, 1.0))
    assert wps[-1][2] == pytest.approx(math.pi / 2)
    yaws = [w[2] for w in wps]
    assert len(set(round(y, 6) for y in yaws)) > 1
    _consecutive_ok(wps, 0.3)


def test_path_waypoints_all_consecutive_spaced():
    # A zig-zag that doubles back, to exercise the final-point trimming.
    pts = _line(0.0, 1.0, 0.07) + _line(1.0, 0.5, 0.07)[1:]
    for spacing in (0.2, 0.3, 0.5):
        wps = ml.path_waypoints(pts, min_spacing=spacing)
        assert wps[-1][:2] == pytest.approx(pts[-1])
        _consecutive_ok(wps, spacing)


# --- remaining_poses --------------------------------------------------------

POSES = ['a', 'b', 'c', 'd']


def test_remaining_poses_n_zero_clamps_to_last():
    assert ml.remaining_poses(POSES, 0) == ['d']


def test_remaining_poses_n_larger_than_len_returns_all():
    assert ml.remaining_poses(POSES, 10) == POSES


def test_remaining_poses_n_two_returns_last_two():
    assert ml.remaining_poses(POSES, 2) == ['c', 'd']


def test_remaining_poses_empty():
    assert ml.remaining_poses([], 3) == []


def test_remaining_poses_none_returns_all():
    assert ml.remaining_poses(POSES, None) == POSES


# --- parse_mission_cmd ------------------------------------------------------

@pytest.mark.parametrize('text, expected', [
    ('pause ', 'PAUSE'),
    ('  Resume\n', 'RESUME'),
    ('CANCEL', 'CANCEL'),
    ('cancel', 'CANCEL'),
])
def test_parse_mission_cmd_known(text, expected):
    assert ml.parse_mission_cmd(text) == expected


def test_parse_mission_cmd_unknown_and_empty():
    assert ml.parse_mission_cmd('go') is None
    assert ml.parse_mission_cmd('') is None
    assert ml.parse_mission_cmd('   ') is None


# --- mission_state_json -----------------------------------------------------

def test_mission_state_json_round_trip_and_key_order():
    text = ml.mission_state_json(ml.NAVIGATING, mode=ml.MODE_PATH, remaining=3.0,
                                 message='on way')
    assert list(json.loads(text).keys()) == ['state', 'mode', 'remaining', 'message',
                                             'teleop']
    assert json.loads(text) == {'state': 'NAVIGATING', 'mode': 'path',
                                'remaining': 3, 'message': 'on way',
                                'teleop': False}
    assert text.index('"state"') < text.index('"mode"') < text.index('"remaining"') \
        < text.index('"message"') < text.index('"teleop"')


def test_mission_state_json_teleop_flag():
    on = json.loads(ml.mission_state_json(ml.IDLE, teleop=True))
    off = json.loads(ml.mission_state_json(ml.IDLE, teleop=False))
    assert on['teleop'] is True and off['teleop'] is False
    assert list(on.keys())[-1] == 'teleop'


def test_mission_state_json_defaults_and_null_mode():
    data = json.loads(ml.mission_state_json(ml.IDLE))
    assert data == {'state': 'IDLE', 'mode': None, 'remaining': 0, 'message': '',
                    'teleop': False}


def test_mission_state_json_invalid_state():
    with pytest.raises(ValueError):
        ml.mission_state_json('RUNNING')


def test_mission_state_json_invalid_mode():
    with pytest.raises(ValueError):
        ml.mission_state_json(ml.IDLE, mode='route')


def test_states_constant_complete():
    assert set(ml.STATES) == {'IDLE', 'NAVIGATING', 'PAUSED', 'SUCCEEDED',
                              'FAILED', 'CANCELED'}


# --- yaw <-> quaternion -----------------------------------------------------

@pytest.mark.parametrize('yaw', [-3.0, -math.pi / 2, -0.5, 0.0, 0.7, math.pi / 2, 3.1])
def test_yaw_quaternion_round_trip(yaw):
    q = ml.yaw_to_quaternion(yaw)
    assert len(q) == 4
    assert q[0] == pytest.approx(0.0) and q[1] == pytest.approx(0.0)
    assert math.hypot(*q) == pytest.approx(1.0)
    assert ml.quaternion_to_yaw(*q) == pytest.approx(yaw)
