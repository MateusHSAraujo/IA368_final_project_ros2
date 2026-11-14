"""Offline tests for vr_robot_pkg.map_store (no rclpy, no ROS graph)."""

import os
import sys
import types

import pytest

from vr_robot_pkg import map_store as ms


def _make_pair(maps_dir, name):
    os.makedirs(maps_dir, exist_ok=True)
    for ext in ('.posegraph', '.data'):
        with open(os.path.join(maps_dir, name + ext), 'w') as handle:
            handle.write('x')


# valid_name ---------------------------------------------------------------

@pytest.mark.parametrize('name', ['map1', 'Kitchen_2', 'a-b', 'A', 'x' * 40])
def test_valid_name_accepts_good_names(name):
    assert ms.valid_name(name)


@pytest.mark.parametrize('name', [
    '', 'x' * 41, 'has space', 'a/b', 'a.b', '..', '.', 'semi;colon',
    'trailing\n', None, 42, b'map1',
])
def test_valid_name_rejects_bad_names(name):
    assert not ms.valid_name(name)


# resolve_maps_dir -----------------------------------------------------------

def test_resolve_override_expands_home(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    assert ms.resolve_maps_dir('~/my_maps') == os.path.join(str(tmp_path),
                                                            'my_maps')


def test_resolve_override_is_absolute(tmp_path):
    result = ms.resolve_maps_dir(str(tmp_path / 'maps'))
    assert result == str(tmp_path / 'maps')


def test_resolve_share_dir_maps_next_to_ros2_ws(tmp_path):
    repo = tmp_path / 'repo'
    share = repo / 'ros2_ws' / 'install' / 'vr_robot_pkg' / 'share' / \
        'vr_robot_pkg'
    share.mkdir(parents=True)
    assert ms.resolve_maps_dir(share_dir=str(share)) == str(repo / 'maps')


def test_resolve_share_dir_without_ros2_ws_falls_back(tmp_path):
    share = tmp_path / 'other' / 'install' / 'vr_robot_pkg' / 'share'
    share.mkdir(parents=True)
    expected = os.path.abspath(os.path.expanduser(ms.FALLBACK_DIR))
    assert ms.resolve_maps_dir(share_dir=str(share)) == expected


def test_resolve_install_not_under_ros2_ws_falls_back(tmp_path):
    # 'install' exists but its parent is not named ros2_ws.
    share = tmp_path / 'ros2_ws_copy' / 'install_old' / 'install' / 'pkg'
    share.mkdir(parents=True)
    expected = os.path.abspath(os.path.expanduser(ms.FALLBACK_DIR))
    assert ms.resolve_maps_dir(share_dir=str(share)) == expected


def test_resolve_uses_ament_lookup(monkeypatch, tmp_path):
    repo = tmp_path / 'repo'
    share = repo / 'ros2_ws' / 'install' / 'vr_robot_pkg' / 'share'
    share.mkdir(parents=True)

    fake_packages = types.ModuleType('ament_index_python.packages')
    fake_packages.get_package_share_directory = lambda pkg: str(share)
    fake_root = types.ModuleType('ament_index_python')
    monkeypatch.setitem(sys.modules, 'ament_index_python', fake_root)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages',
                        fake_packages)

    assert ms.resolve_maps_dir() == str(repo / 'maps')


def test_resolve_ament_failure_falls_back(monkeypatch):
    def boom(pkg):
        raise RuntimeError('package not found')

    fake_packages = types.ModuleType('ament_index_python.packages')
    fake_packages.get_package_share_directory = boom
    fake_root = types.ModuleType('ament_index_python')
    monkeypatch.setitem(sys.modules, 'ament_index_python', fake_root)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages',
                        fake_packages)

    expected = os.path.abspath(os.path.expanduser(ms.FALLBACK_DIR))
    assert ms.resolve_maps_dir() == expected


# map_base / keepout_path -----------------------------------------------------

def test_map_base_joins_name(tmp_path):
    assert ms.map_base(str(tmp_path), 'room') == os.path.join(str(tmp_path),
                                                              'room')


def test_map_base_rejects_invalid_name(tmp_path):
    with pytest.raises(ValueError):
        ms.map_base(str(tmp_path), '../escape')


def test_keepout_path(tmp_path):
    assert ms.keepout_path(str(tmp_path), 'room') == os.path.join(
        str(tmp_path), 'room.keepout.json')


# list_maps / map_exists ------------------------------------------------------

def test_list_maps_only_complete_pairs_sorted(tmp_path):
    _make_pair(str(tmp_path), 'zeta')
    _make_pair(str(tmp_path), 'alpha')
    # Incomplete: missing .data
    (tmp_path / 'partial.posegraph').write_text('x')
    # Incomplete: missing .posegraph
    (tmp_path / 'onlydata.data').write_text('x')
    # Invalid names are ignored even with a complete-looking pair.
    _make_pair(str(tmp_path), 'bad.name')
    # Map image and strokes do not make a map on their own.
    (tmp_path / 'imgonly.pgm').write_text('x')
    (tmp_path / 'imgonly.yaml').write_text('x')
    assert ms.list_maps(str(tmp_path)) == ['alpha', 'zeta']


def test_list_maps_missing_dir(tmp_path):
    assert ms.list_maps(str(tmp_path / 'missing')) == []


def test_map_exists(tmp_path):
    _make_pair(str(tmp_path), 'room')
    assert ms.map_exists(str(tmp_path), 'room')
    assert not ms.map_exists(str(tmp_path), 'nope')
    assert not ms.map_exists(str(tmp_path), '../room')


# strokes ---------------------------------------------------------------------

STROKES = [
    [(0.5, -1.25, 0.3), (0.75, -1.0, 0.3)],
    [(2.0, 3.5, 0.1)],
]


def test_strokes_round_trip_preserves_floats(tmp_path):
    maps_dir = str(tmp_path / 'maps')
    path = ms.save_strokes(maps_dir, 'room', STROKES)
    assert path == ms.keepout_path(maps_dir, 'room')
    loaded = ms.load_strokes(maps_dir, 'room')
    assert loaded == [
        [(0.5, -1.25, 0.3), (0.75, -1.0, 0.3)],
        [(2.0, 3.5, 0.1)],
    ]
    assert all(isinstance(v, float) for s in loaded for p in s for v in p)


def test_save_strokes_creates_directory(tmp_path):
    maps_dir = str(tmp_path / 'new' / 'nested')
    ms.save_strokes(maps_dir, 'room', [])
    assert os.path.isdir(maps_dir)
    assert ms.load_strokes(maps_dir, 'room') == []


def test_save_strokes_file_layout(tmp_path):
    ms.save_strokes(str(tmp_path), 'room', [[(1, 2, 3)]])
    with open(ms.keepout_path(str(tmp_path), 'room')) as handle:
        import json
        data = json.load(handle)
    assert data == {'version': 1, 'strokes': [[[1.0, 2.0, 3.0]]]}


def test_save_strokes_leaves_no_temp_files(tmp_path):
    ms.save_strokes(str(tmp_path), 'room', STROKES)
    ms.save_strokes(str(tmp_path), 'room', STROKES[:1])
    assert sorted(os.listdir(tmp_path)) == ['room.keepout.json']


def test_save_strokes_rejects_bad_point(tmp_path):
    with pytest.raises(ValueError):
        ms.save_strokes(str(tmp_path), 'room', [[(1.0, 2.0)]])
    assert os.listdir(tmp_path) == []


def test_save_strokes_invalid_name(tmp_path):
    with pytest.raises(ValueError):
        ms.save_strokes(str(tmp_path), 'bad/name', [])


def test_load_strokes_missing_file(tmp_path):
    assert ms.load_strokes(str(tmp_path), 'room') == []


def test_load_strokes_malformed_json(tmp_path):
    (tmp_path / 'room.keepout.json').write_text('{"version": 1, "strokes"')
    with pytest.raises(ValueError):
        ms.load_strokes(str(tmp_path), 'room')


@pytest.mark.parametrize('payload', [
    '[]',
    '{"version": 2, "strokes": []}',
    '{"strokes": []}',
    '{"version": true, "strokes": []}',
    '{"version": 1, "strokes": {}}',
    '{"version": 1, "strokes": [5]}',
    '{"version": 1, "strokes": [[[1, 2]]]}',
    '{"version": 1, "strokes": [[["a", 2, 3]]]}',
    '{"version": 1, "strokes": [[[true, 2, 3]]]}',
])
def test_load_strokes_wrong_structure(tmp_path, payload):
    (tmp_path / 'room.keepout.json').write_text(payload)
    with pytest.raises(ValueError):
        ms.load_strokes(str(tmp_path), 'room')


def test_load_strokes_wrong_version_message(tmp_path):
    (tmp_path / 'room.keepout.json').write_text(
        '{"version": 7, "strokes": []}')
    with pytest.raises(ValueError, match='version'):
        ms.load_strokes(str(tmp_path), 'room')


# parse_map_cmd ----------------------------------------------------------------

@pytest.mark.parametrize('text, expected', [
    ('SAVE room1', ('SAVE', 'room1')),
    ('LOAD room1', ('LOAD', 'room1')),
    ('RESET', ('RESET', None)),
    ('save room1', ('SAVE', 'room1')),
    ('Load Kitchen_2', ('LOAD', 'Kitchen_2')),
    ('reset', ('RESET', None)),
    ('   SAVE    room1  ', ('SAVE', 'room1')),
    ('\tLOAD\troom-1\n', ('LOAD', 'room-1')),
    ('  RESET  ', ('RESET', None)),
])
def test_parse_map_cmd_valid(text, expected):
    assert ms.parse_map_cmd(text) == expected


@pytest.mark.parametrize('text', [
    '', '   ', None, 5,
    'DELETE room', 'HELLO', 'SAVE', 'LOAD', '   save   ',
    'SAVE room1 extra', 'LOAD a b c',
    'RESET now', 'RESET room1',
    'SAVE bad/name', 'LOAD ..', 'SAVE ' + 'x' * 41, 'SAVE a.b',
])
def test_parse_map_cmd_errors(text):
    with pytest.raises(ValueError):
        ms.parse_map_cmd(text)


def test_parse_map_cmd_invalid_name_message():
    with pytest.raises(ValueError, match='letters, digits'):
        ms.parse_map_cmd('SAVE bad/name')


def test_parse_map_cmd_empty_message():
    with pytest.raises(ValueError, match='empty'):
        ms.parse_map_cmd('')


# --------------------------------------------------------------------------- map image

def test_write_map_image_trinary_and_flipped(tmp_path):
    # 3x2 grid, row 0 = lowest y: [occupied, free, unknown] / [free, 50, occupied]
    data = [100, 0, -1, 0, 50, 100]
    yaml_path = ms.write_map_image(str(tmp_path), 'tiny', 3, 2, 0.05, (-1.0, 2.0, 0.0), data)
    raw = (tmp_path / 'tiny.pgm').read_bytes()
    header, pixels = raw.rsplit(b'255\n', 1)
    assert header.startswith(b'P5\n') and b'3 2\n' in header
    # PGM top row = highest y = grid row 1
    assert list(pixels) == [254, 205, 0, 0, 254, 205]
    text = open(yaml_path).read()
    assert 'image: tiny.pgm' in text and 'mode: trinary' in text
    assert 'origin: [-1.0, 2.0, 0.0]' in text and 'resolution: 0.05' in text


def test_write_map_image_size_mismatch(tmp_path):
    with pytest.raises(ValueError):
        ms.write_map_image(str(tmp_path), 'bad', 3, 2, 0.05, (0, 0, 0), [0] * 5)
