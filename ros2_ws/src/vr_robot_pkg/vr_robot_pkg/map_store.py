"""Saved SLAM map storage for the VR robot mission supervisor.

Pure module: standard library only, no rclpy. A saved map named NAME inside
a maps directory D consists of:

- ``D/NAME.posegraph`` and ``D/NAME.data``: written by slam_toolbox's
  serialize service (base path ``D/NAME``, no extension). Required for a
  map to be listed.
- ``D/NAME.pgm`` and ``D/NAME.yaml``: optional map image.
- ``D/NAME.keepout.json``: no-go zone strokes written by this module.
"""

import json
import os
import re
import tempfile

NAME_PATTERN = r'^[A-Za-z0-9_-]{1,40}$'
FALLBACK_DIR = '~/.ros/vr_robot_maps'

_NAME_RE = re.compile(NAME_PATTERN)
_PACKAGE = 'vr_robot_pkg'
_WORKSPACE_DIR = 'ros2_ws'
_INSTALL_DIR = 'install'
_STROKES_VERSION = 1


def valid_name(name):
    """Return True when ``name`` is a str matching NAME_PATTERN."""
    if not isinstance(name, str):
        return False
    return _NAME_RE.fullmatch(name) is not None


def _workspace_maps_dir(share_dir):
    """Return ``<workspace>/maps`` for a share dir inside ``ros2_ws/install``.

    Returns None when no ancestor is ``install`` whose parent is ``ros2_ws``.
    """
    path = os.path.abspath(share_dir)
    while True:
        parent = os.path.dirname(path)
        if os.path.basename(path) == _INSTALL_DIR and \
                os.path.basename(parent) == _WORKSPACE_DIR:
            return os.path.join(os.path.dirname(parent), 'maps')
        if parent == path:
            return None
        path = parent


def _package_share_dir():
    """Return the installed share directory of this package, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(_PACKAGE)
    except Exception:  # noqa: BLE001 - any failure means "unknown"
        return None


def resolve_maps_dir(override='', share_dir=None):
    """Return the absolute directory for maps. Does NOT create it.

    Order: explicit ``override``; then the maps folder of the workspace that
    owns the package install (``<ws>/maps`` for ``<ws>/install/...``); then
    the expanded ``FALLBACK_DIR``.
    """
    if override:
        return os.path.abspath(os.path.expanduser(override))

    if share_dir is None:
        share_dir = _package_share_dir()
    if share_dir:
        maps_dir = _workspace_maps_dir(share_dir)
        if maps_dir is not None:
            return maps_dir

    return os.path.abspath(os.path.expanduser(FALLBACK_DIR))


def map_base(maps_dir, name):
    """Return ``maps_dir/name`` (base path without extension)."""
    if not valid_name(name):
        raise ValueError(
            f'invalid map name {name!r}: use 1-40 characters from '
            'letters, digits, underscore and hyphen')
    return os.path.join(maps_dir, name)


def keepout_path(maps_dir, name):
    """Return the path of the keepout strokes file for a map."""
    return map_base(maps_dir, name) + '.keepout.json'


def _has_complete_pair(maps_dir, name):
    base = os.path.join(maps_dir, name)
    return (os.path.isfile(base + '.posegraph')
            and os.path.isfile(base + '.data'))


def list_maps(maps_dir):
    """Return sorted names having both .posegraph and .data files."""
    try:
        entries = os.listdir(maps_dir)
    except (FileNotFoundError, NotADirectoryError):
        return []

    names = set()
    for entry in entries:
        if not entry.endswith('.posegraph'):
            continue
        name = entry[:-len('.posegraph')]
        if valid_name(name) and _has_complete_pair(maps_dir, name):
            names.add(name)
    return sorted(names)


def map_exists(maps_dir, name):
    """Return True when the map has a valid name and a complete file pair."""
    return valid_name(name) and _has_complete_pair(maps_dir, name)


def _format_strokes(strokes):
    """Validate and convert strokes to a list of lists of [x, y, r] floats."""
    formatted = []
    for s_index, stroke in enumerate(strokes):
        points = []
        for p_index, point in enumerate(stroke):
            try:
                x, y, r = point
                values = [float(x), float(y), float(r)]
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f'stroke {s_index}, point {p_index} must be (x, y, r) '
                    f'numbers: {exc}') from exc
            points.append(values)
        formatted.append(points)
    return formatted


def save_strokes(maps_dir, name, strokes):
    """Atomically write the keepout strokes of a map. Returns the file path.

    ``strokes`` is a list of strokes, each a list of (x, y, r) floats.
    """
    path = keepout_path(maps_dir, name)
    payload = {'version': _STROKES_VERSION,
               'strokes': _format_strokes(strokes)}
    text = json.dumps(payload)

    os.makedirs(maps_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=maps_dir, prefix='.keepout-',
                                    suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(tmp_path, 0o644)   # mkstemp creates 0600; the file is committed and shared
        os.replace(tmp_path, path)
    except BaseException:
        try:
            os.remove(tmp_path)
        except OSError:
            pass
        raise
    return path


def _parse_strokes(data):
    if not isinstance(data, dict):
        raise ValueError('keepout file must contain a JSON object')
    if data.get('version') != _STROKES_VERSION or \
            isinstance(data.get('version'), bool):
        raise ValueError(
            f'unsupported keepout file version {data.get("version")!r} '
            f'(expected {_STROKES_VERSION})')
    strokes = data.get('strokes')
    if not isinstance(strokes, list):
        raise ValueError('keepout file "strokes" must be a list')

    result = []
    for s_index, stroke in enumerate(strokes):
        if not isinstance(stroke, list):
            raise ValueError(f'stroke {s_index} must be a list')
        points = []
        for p_index, point in enumerate(stroke):
            if not isinstance(point, list) or len(point) != 3 or \
                    any(isinstance(v, bool) or not isinstance(v, (int, float))
                        for v in point):
                raise ValueError(
                    f'stroke {s_index}, point {p_index} must be [x, y, r] '
                    'numbers')
            points.append(tuple(float(v) for v in point))
        result.append(points)
    return result


def load_strokes(maps_dir, name):
    """Return the keepout strokes of a map as lists of (x, y, r) tuples.

    A missing file gives []. Malformed JSON, a wrong structure or an
    unsupported version raise ValueError.
    """
    path = keepout_path(maps_dir, name)
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            text = handle.read()
    except FileNotFoundError:
        return []

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f'malformed JSON in {path}: {exc}') from exc
    return _parse_strokes(data)


def write_map_image(maps_dir, name, width, height, resolution, origin, data,
                    occupied_thresh=0.65, free_thresh=0.25):
    """Write ``NAME.pgm`` + ``NAME.yaml`` in nav2 map_server trinary format.

    ``data`` is OccupancyGrid data (row-major, row 0 = lowest y, values -1 or
    0..100); ``origin`` is (x, y, yaw). Occupied (>= occupied_thresh*100) is
    black (0), free (<= free_thresh*100) white (254), anything else unknown
    (205). PGM rows run top-down, so the grid is flipped vertically. Used
    instead of slam_toolbox's save_map, whose map_saver times out waiting for
    /map. Returns the yaml path.
    """
    width, height = int(width), int(height)
    if len(data) != width * height:
        raise ValueError(f'map data has {len(data)} cells, expected '
                         f'{width}x{height}')
    occ_min = occupied_thresh * 100.0
    free_max = free_thresh * 100.0
    pixels = bytearray(width * height)
    for row in range(height):
        src = (height - 1 - row) * width
        dst = row * width
        for col in range(width):
            v = data[src + col]
            if v < 0:
                pixels[dst + col] = 205
            elif v >= occ_min:
                pixels[dst + col] = 0
            elif v <= free_max:
                pixels[dst + col] = 254
            else:
                pixels[dst + col] = 205
    base = map_base(maps_dir, name)
    os.makedirs(maps_dir, exist_ok=True)
    with open(base + '.pgm', 'wb') as handle:
        handle.write(f'P5\n# CREATOR: vr_robot_pkg map_store\n{width} {height}\n255\n'
                     .encode('ascii'))
        handle.write(bytes(pixels))
    ox, oy, oyaw = (float(v) for v in origin)
    with open(base + '.yaml', 'w', encoding='utf-8') as handle:
        handle.write(f'image: {name}.pgm\nmode: trinary\nresolution: {round(float(resolution), 6)}\n'
                     f'origin: [{ox}, {oy}, {oyaw}]\nnegate: 0\n'
                     f'occupied_thresh: {occupied_thresh}\nfree_thresh: {free_thresh}\n')
    return base + '.yaml'


_COMMANDS_WITH_NAME = ('SAVE', 'LOAD')


def parse_map_cmd(text):
    """Parse a Unity command: 'SAVE <name>', 'LOAD <name>' or 'RESET'.

    Returns ``(cmd, name)`` with ``cmd`` upper-case and ``name`` None for
    RESET. Raises ValueError with a readable message on any error.
    """
    if not isinstance(text, str):
        raise ValueError('map command must be a string')
    parts = text.split()
    if not parts:
        raise ValueError('empty map command')

    cmd = parts[0].upper()
    args = parts[1:]

    if cmd in _COMMANDS_WITH_NAME:
        if not args:
            raise ValueError(f'{cmd} needs a map name: {cmd} <name>')
        if len(args) > 1:
            raise ValueError(
                f'{cmd} takes exactly one map name, got {len(args)} '
                'arguments')
        name = args[0]
        if not valid_name(name):
            raise ValueError(
                f'invalid map name {name!r}: use 1-40 characters from '
                'letters, digits, underscore and hyphen')
        return cmd, name

    if cmd == 'RESET':
        if args:
            raise ValueError('RESET takes no arguments')
        return cmd, None

    raise ValueError(
        f'unknown map command {parts[0]!r}: expected SAVE <name>, '
        'LOAD <name> or RESET')
