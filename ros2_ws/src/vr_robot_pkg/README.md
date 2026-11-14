# vr_robot_pkg

ROS 2 Jazzy package of the IA368 final project (ROS 2 side, WSL2). It bridges CoppeliaSim to ROS 2, builds the Nav2 keepout mask, and brings up SLAM, Nav2 and the ROS-TCP-Endpoint that Unity connects to.

This file is also the **interface contract** for the Unity side (ROS-TCP-Connector, Stages 5 and 6). Sections marked **RESERVED** are fixed contracts whose ROS-side implementation does not exist yet.

## Overview

`ros2 launch vr_robot_pkg vr_robot_bringup.launch.py` starts:

| Component | Role |
|---|---|
| `coppelia_bridge` | CoppeliaSim ZMQ Remote API (Windows host, port 23000) <-> ROS: scan, odom, TF, camera, `cmd_vel`. |
| `keepout_mask_node` | Publishes `/keepout_filter_mask` (transient_local); accepts debug polygons on `/keepout_mask_test`. |
| `costmap_filter_info_server` + `lifecycle_manager_costmap_filters` | Nav2 Keepout Filter plumbing (started 3 s late on purpose). |
| slam_toolbox (online async) | Mapping; publishes `/map` and `map -> odom`. |
| Nav2 `navigation_launch.py` | Started after `nav2_delay` (8 s). |
| ROS-TCP-Endpoint (v0.7.0, protocol ROS2) | Listens on `0.0.0.0:10000`; Unity connects here. |
| RViz | Optional. |

Bridge behaviour: starts the CoppeliaSim simulation if it is stopped and stops it on exit (only if it started it). It turns CoppeliaSim window rendering off while running (about 0.44x -> 0.87x real time in scene_house) and restores it on exit; `coppelia_display:=true` keeps rendering on.

`mock_mode:=true` simulates the robot in-process (6x6 m room): same topics, no CoppeliaSim needed.

## Quick start

```bash
git clone --recurse-submodules git@github.com:MateusHSAraujo/IA368_final_project_ros2.git
# or: git submodule update --init   (ROS-TCP-Endpoint, branch main-ros2, v0.7.0)
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
# Live (CoppeliaSim running on Windows with the prepared scene):
ros2 launch vr_robot_pkg vr_robot_bringup.launch.py
# Offline:
ros2 launch vr_robot_pkg vr_robot_bringup.launch.py mock_mode:=true
```

Note: for this ament_python package, `config/` and `launch/` files are COPIED at build time. Rebuild after editing them.

## Scene preparation (CoppeliaSim)

Required in `scene_house.ttt`: `/myRobot` (Pioneer P3-DX) with `/myRobot/frontCamera`, and the fast laser.

| Script (`scripts/coppelia/`) | Effect |
|---|---|
| `install_fast_laser.py` | Installs two 92-degree depth vision sensors (same 452-sample output as the original laser). Its script provides `remoteExchange()`, the single batched Remote API call the bridge uses (CoppeliaSim answers only ~1-2 Remote API requests per simulation step while simulating). |
| `prepare_scene_house.py` | Makes house shapes respondable, enables real-time mode, installs the laser. |

Without `remoteExchange` the bridge falls back to individual calls (a few Hz).

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `mock_mode` | `false` | In-process simulated robot. |
| `coppelia_host` | `''` | CoppeliaSim host. Empty = auto-detect (WSL default gateway); also env `COPPELIA_HOST`. |
| `camera_enabled` | `false` | Camera streaming at start (normally toggled by `/camera/enable`). |
| `coppelia_display` | `false` | Keep CoppeliaSim window rendering on. |
| `use_slam` | `true` | slam_toolbox. |
| `use_nav2` | `true` | Nav2 navigation stack. |
| `nav2_delay` | `8.0` | Seconds before Nav2 starts. |
| `use_tcp_endpoint` | `true` | ROS-TCP-Endpoint. |
| `tcp_port` | `10000` | Endpoint port. |
| `use_rviz` | `false` | RViz. |

## Interface contract

### Frames

`map` (slam_toolbox) -> `odom` (= CoppeliaSim world; `map -> odom` starts as identity, so map is approximately the CoppeliaSim/Unity origin, and may drift slightly as SLAM corrects) -> `base_link` (`/myRobot/Ground_truth`, on the floor plane, yaw only) -> `laser_link` (0.0778, 0, 0.2498), `camera_link` (about (0.20, 0, 0.219), tilted 10 deg down) -> `camera_optical_frame`.

Robot: Pioneer P3-DX, wheel radius 0.0975 m, wheelbase 0.331 m, half-width about 0.21 m. Start pose (2.9, 2.125), yaw 0.

### ROS <-> Unity conversion (shared origin)

| Quantity | Conversion |
|---|---|
| Position | `x_ros = z_u`, `y_ros = -x_u`, `z_ros = y_u` (i.e. `x_u = -y_ros`, `y_u = z_ros`, `z_u = x_ros`) |
| Quaternion | Unity `(x,y,z,w) = (qy, -qz, -qx, qw)` of the ROS quaternion |
| Yaw-only pose | ROS `q = (0,0,sin(t/2),cos(t/2))` -> Unity `(0, -sin(t/2), 0, cos(t/2))` = rotation of `-t` about Unity +Y. So `yaw_unity_about_Y = -yaw_ros` (in Unity's left-handed convention, Euler Y in degrees = `-yaw_ros_deg`). |

### Topics available now

"Unity" column = what Unity does with it. Message names in ROS-TCP-Connector use `pkg/Type` (e.g. `geometry_msgs/PoseStamped`).

| Topic | Type | Frame | Unity | Notes |
|---|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | `map` | subscribe | slam_toolbox, republished about every 5 s; transient_local. **Size and origin change while mapping (grid grows): re-create the table texture on every message.** Values -1 unknown, 0 free, 100 occupied. Row-major, row 0 = lowest y. |
| `/hmi/robot_pose` | `geometry_msgs/PoseStamped` | `map` (frame_id `odom` until SLAM publishes `map->odom`: check `frame_id`) | subscribe | 10 Hz. For the table mini-avatar (the 3D twin robot already moves through the Stage 1 ZMQ pose stream). |
| `/plan` | `nav_msgs/Path` | `map` | subscribe | Nav2 global plan, only while navigating. |
| `/myRobot/scan` | `sensor_msgs/LaserScan` | `laser_link` | subscribe | 452 beams, -90 deg to +90.4 deg, step 0.4 deg, `inf` = no hit. 7-8 Hz. |
| `/myRobot/odom` | `nav_msgs/Odometry` | `odom` -> `base_link` | subscribe | Ground truth. 12-15 Hz. |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | `camera_optical_frame` | subscribe | JPEG 320x240, about 8 Hz, about 4-8 KB, **only while enabled**. |
| `/camera/enable` | `std_msgs/Bool` | - | publish | `true` when the teleop panel opens, `false` when it closes. Camera is disabled by default. |
| `/cmd_vel` | `geometry_msgs/Twist` | - | publish (Stage 3 testing only) | `linear.x` m/s, `angular.z` rad/s. Bridge watchdog stops the robot 0.5 s after the last message: publish at >= 10 Hz. Nav2 also publishes here while navigating (collision_monitor). Stage 6 inserts `twist_mux`; Unity must then publish on `/cmd_vel_teleop` instead. |
| `/myRobot/bridge_status` | `std_msgs/String` | - | subscribe | `connected` / `waiting-for-simulation` / `disconnected` / `mock`. Republished every 1 s (also transient_local). |
| `/keepout_filter_mask` | `nav_msgs/OccupancyGrid` | `map` | subscribe (optional) | transient_local. Fixed grid: resolution 0.05, origin (-2.5, -4.25), 310 x 210 cells (house bounding box); 100 = keepout. Published by `keepout_mask_node` now; the Stage 5 supervisor will own it. **Unity must NOT publish it** (ROS-TCP-Endpoint cannot publish transient_local). |
| `/keepout_mask_test` | `geometry_msgs/PolygonStamped` | `map` | debug only | Input of `keepout_mask_node`: polygon in meters, cumulative; empty polygon clears. `Point32` fields must be floats. |

Nav2 actions `/navigate_to_pose` and `/navigate_through_poses` exist, but Unity cannot call ROS actions through ROS-TCP-Connector: use the reserved `/hmi` topics below.

Keepout note: Nav2 does not inflate keepout cells, so the robot body (half-width 0.21 m) can overlap a zone edge. The Stage 5 supervisor will dilate painted strokes by the robot radius.

### RESERVED for Stages 5/6 (contract fixed now, ROS side NOT implemented yet)

| Topic | Type | Direction | Contract |
|---|---|---|---|
| `/hmi/goal` | `geometry_msgs/PoseStamped` (`map`) | Unity -> supervisor | Point-and-click goal -> NavigateToPose. |
| `/hmi/path` | `nav_msgs/Path` (`map`) | Unity -> supervisor | Painted path; the supervisor subsamples to waypoints (>= 0.3 m apart) -> NavigateThroughPoses. |
| `/hmi/mission_cmd` | `std_msgs/String` | Unity -> supervisor | `PAUSE` / `RESUME` / `CANCEL`. Nav2 has no native pause: the supervisor cancels and stores the remaining poses. |
| `/hmi/mission_state` | `std_msgs/String` (JSON) | supervisor -> Unity | On change and at 1 Hz: `{"state": "IDLE"\|"NAVIGATING"\|"PAUSED"\|"SUCCEEDED"\|"FAILED"\|"CANCELED", "mode": "goal"\|"path"\|null, "remaining": n, "message": "..."}` |
| `/hmi/keepout_stroke` | `geometry_msgs/PolygonStamped` (`map`) | Unity -> supervisor | One painted stroke = ordered points (`x`, `y` in meters, `z` = brush radius in meters). Cumulative; a message with zero points clears all zones. The supervisor rasterizes into `/keepout_filter_mask` (dilated by robot half-width). |
| `/cmd_vel_teleop` | `geometry_msgs/Twist` | Unity -> `twist_mux` | Higher priority than Nav2. >= 10 Hz while the deadman (grip) is held; stop publishing to release. The supervisor auto-pauses an active mission when teleop starts. |

## Networking

| Client | Setup |
|---|---|
| Unity Editor (Windows) | ROS IP `127.0.0.1`, port `10000`; reaches WSL through WSL2 localhost forwarding (verified with `Test-NetConnection`). |
| Quest 3 (USB) | Run `scripts/setup_adb_reverse.ps1` on Windows (adb reverse tcp 10000, 23005, 23006). The APK also uses `127.0.0.1:10000`. `-PortProxy` (admin) adds a netsh portproxy if localhost forwarding fails. Mirrored to the Windows repo `scripts/` by `scripts/sync_to_windows.sh`. |
| ROS side -> CoppeliaSim | Windows host port 23000 (auto-detected as the WSL default gateway; `scripts/get_windows_host_ip.sh`). |

ROS-TCP-Connector handshake: endpoint v0.7.0, protocol ROS2.

## Performance and limits (scene_house, display off)

| Item | Value |
|---|---|
| Bridge cycle | 13-19 Hz (one exchange per sim step) |
| `/myRobot/odom` | 12-15 Hz |
| `/myRobot/scan` | 7-8 Hz |
| Camera (when enabled) | 7-8 Hz |
| Simulation speed | about 0.7-0.87x real time (below real time: plan for it in timing and timeouts) |
| Nav goals verified | between dining room, living room and corridor, e.g. (2.4,-1.5), (3.3,-1.7), (6.2,0.7), (2.5,0.6); NavigateThroughPoses works |

## Troubleshooting

| Symptom | Fix |
|---|---|
| CoppeliaSim not answering (bridge status `disconnected`/stuck) | The add-on hung. In the CoppeliaSim window click **"Connectivity >> ZMQ remote API server@addonScript: abort execution"** (only the user can). |
| Bridge runs at a few Hz | `remoteExchange()` missing: run `scripts/coppelia/install_fast_laser.py` (or `prepare_scene_house.py`). |
| `ros2 topic list` stale/odd | `ros2 daemon stop`, then retry. |
| Edited config/launch not taking effect | Rebuild (`colcon build --symlink-install`): those files are copied. |
| Unity cannot connect | Check endpoint is up (`ss -ltn \| grep 10000`), `127.0.0.1:10000`, and the Quest `adb reverse` script. |

## Verification

```bash
pytest tests/        # 54 offline tests: math, network scripts, keepout mask, bridge in mock mode
# Live (stack running against CoppeliaSim):
python3 -u tests/live/check_live_stack.py rates drive camera keepout nav through map \
    --zone 2.4 -0.4 0.4 --goal 2.4 -1.5 --goal 3.3 -1.7 --goal 6.2 0.7
python3 -u tests/live/check_tcp_endpoint.py   # acts like Unity through port 10000
```
