# IA368 — Final Project, ROS 2 side (WSL2)

You are the **WSL agent (Claude Code on WSL2, Ubuntu 24.04)**. This repository (`IA368_final_project_ros2`, `git@github.com:MateusHSAraujo/IA368_final_project_ros2.git`) holds the ROS 2 side of the project: ROS 2 Jazzy, Nav2, slam_toolbox, the CoppeliaSim–ROS 2 bridge and the mission supervisor (Stages 2, 3, 5 and 6 of the plan).

## Read first
- `IMPLEMENTATION_PLAN.md`: stage-by-stage plan. Section 2.4 lists the open architecture items for Stages 3–6 (e.g. ROS-TCP-Connector is not an Action client; Nav2 has no native Pause/Resume; the Keepout Filter needs `costmap_filter_info_server` and a `transient_local` mask).
- `PROJECT_PROPOSAL.md`: overview.
- `STAGE1_UNITY_PLAN.md`: what the Windows agent built in Stage 1 (CoppeliaSim → Unity converter) and its status.
- `STAGE4_VR_LOCOMOTION_PLAN.md`: what the Windows agent built in Stage 4 part 1 (VR locomotion, VR menu, start flow) and its status. Not needed for the ROS work.

These four files are **symlinks into `windows_workspace/`** (the Windows repository `IA368_final_project`, mounted at `/mnt/c/Users/mateu/unicamp/IA368/projeto_final`). There is a single copy of each document: if you edit them, the change lands in the Windows repository. Do not commit the Windows repository from WSL (Git over `/mnt/c` is slow and mixes line endings/permissions); tell the user which documents you changed so they are committed on Windows.

## Language
Everything committed is in **English**: code, comments, log strings, documentation, file and folder names, and commit messages.

## Division of work
- **Windows agent (Claude Code on Windows)**, repo `IA368_final_project`: Unity project (`My project/`), CoppeliaSim scripts (`coppelia_scripts/`), scenes (`scene_house/*.ttt`), PowerShell scripts and the planning documents. Stage 1 is done.
- **You (WSL)**: `ros2_ws/`, `tests/`, `scripts/*.sh` (and `scripts/*.ps1` authored here, mirrored to Windows by `scripts/sync_to_windows.sh`).
- `sync_to_windows.sh` copies **only** `scripts/*.ps1`. Never copy documents, C#, Lua or scenes to Windows: Windows is their master copy.

## Interfaces with the Windows side
- **CoppeliaSim EDU 4.10 runs on Windows.** ZMQ Remote API on port **23000** (Python client `coppeliasim-zmqremoteapi-client`); this is what the ROS nodes use. From WSL, reach the Windows host IP (Stage 2: `scripts/get_windows_host_ip.sh`) or use WSL mirrored networking.
- **Scene streamer (Stage 1, for Unity):** REP **23005** and PUB **23006**, documented in `windows_workspace/coppelia_scripts/README_COPPELIA_SETUP.md`. The ROS side does not need it.
- **Unity ↔ ROS 2:** ROS-TCP-Endpoint on port **10000** (Stages 3–6).
- **Frames:** Unity and CoppeliaSim/ROS share the origin. ROS ↔ Unity: `x_ros = z_u, y_ros = -x_u, z_ros = y_u`; quaternion Unity = `(qy, -qz, -qx, qw)` of the ROS/CoppeliaSim quaternion.
- **Robot:** Pioneer P3-DX `/myRobot`, `r = 0.0975 m`, `L = 0.331 m`, joints `/leftMotor` and `/rightMotor`, `/myRobot/scan`, `/myRobot/odom`, `/camera/image_raw`. The scene with the robot is `windows_workspace/scene_house/p3dx_nav2.ttt`; the house without the robot is `windows_workspace/scene_house/scene_house.ttt`.
- Course reference code to reuse: `/home/mateus/ia368/IA368ii/IA368_ws` (see section 0 of the plan).

## Operating CoppeliaSim through the Remote API (practical lessons)
- Always set a receive timeout (`client.socket.setsockopt(zmq.RCVTIMEO, 5000..20000)`); without it the client blocks forever. Run Python with `-u`.
- If the add-on hangs, CoppeliaSim shows **"Connectivity >> ZMQ remote API server@addonScript: abort execution"**. Only the user can click it: detect the timeout and ask them. Check liveness with a simple call and a 5 s timeout; the fps in the window title is not reliable.
- Every script that calls `sim.setStepping(True)` or `sim.startSimulation()` must call `sim.stopSimulation()` and `sim.setStepping(False)` in a `finally` block.
- Never pull large meshes/textures through the Remote API (CBOR serialization in Lua freezes the add-on).
- `sim.getShapeViz(h, i)` does not fail for out-of-range indices; get the count from `#sim.getIntArrayProperty(h, 'meshes')`.
- ROS nodes must tolerate CoppeliaSim being absent (retry loop or `mock_mode:=true`) so the offline tests keep working.
- **Throughput while simulating: only ~1–2 Remote API requests are answered per simulation step** (~20/s). Batch: the bridge makes one `sim.callScriptFunction('remoteExchange', ...)` per cycle (function in the laser script installed by `scripts/coppelia/install_fast_laser.py`). While stopped, ~400 calls/s.
- Simulation speed in `scene_house.ttt`: the class laser sweep (452 proximity rays/step in Lua) and CoppeliaSim's own window rendering each halve it. Use the depth-camera laser and keep the display off (`sim.boolparam_display_enabled`, the bridge does it). Measure with `sim.getSimulationTime()` vs wall time.
- `sim.getVisionSensorImg` rows are bottom-up (flip vertically only); `sim.saveImage(img, res, 0, '.jpg', q)` returns an upright JPEG buffer.
- No `sim.executeScriptString` in 4.10: to run Lua for debugging, create a temporary customization script (`sim.createScript`), wait ~1 s for init, `sim.callScriptFunction`, then remove it.
- A Ctrl-C during a pending request leaves the REQ socket unusable: do cleanup over a fresh client.

## Status
- Stages 2 and 3 done (2026-11-13), validated live against `scene_house.ttt`. Contract for Stages 5–6: `ros2_ws/src/vr_robot_pkg/README.md`.
- Stage 5 ROS side done (2026-11-14): `mission_supervisor_node` (the only node Unity talks to: `/hmi/goal`, `/hmi/path`, `/hmi/mission_cmd`, `/hmi/keepout_stroke`, `/hmi/map_cmd` → `/hmi/mission_state`, `/hmi/map_state`), replacing `keepout_mask_node`. Saved maps in `maps/` (reference `scene_house`); `map:=scene_house` loads it at startup. Next on this side: twist_mux (Stage 6, `sudo apt install ros-jazzy-twist-mux`).
- `scene_house.ttt` was saved on Windows (2026-11-13) with `/myRobot`, `/myRobot/frontCamera`, respondable house shapes, real-time mode and the depth-camera laser v2. Scene changes made through the Remote API live only in memory until the user saves the scene in CoppeliaSim.

## ROS-side practical lessons
- `ament_python` + `--symlink-install`: Python modules and config/launch files are **copied** into `install/`; rebuild after every change before a live run (pytest imports `src/` through `tests/conftest.py`, so it can pass on stale installs).
- Processes started in the background from a non-interactive shell inherit SIGINT as ignored: start `ros2 launch` through a wrapper that restores `SIG_DFL`, stop it with one SIGINT to the launch process, and never `pkill -f` a pattern contained in your own command line.
- `tf2_ros.TransformListener(spin_thread=True)` spins the whole node in a second executor (timers included): don't use it in nodes with blocking timers.
- Publishers created right before publishing lose messages until discovery matches: wait for `get_subscription_count() > 0`. A stale ROS daemon hides topics: `ros2 daemon stop`.
- Live checks: `tests/live/check_live_stack.py` (incl. the `hmi_*` steps) and `tests/live/check_tcp_endpoint.py` (not collected by pytest).
- In a shell, `a && b && cmd &` backgrounds the whole `&&` list as one subshell, so `$!` is not the launch PID: start `ros2 launch` as its own statement before recording `$!`. A stray live stack makes the mock-mode tests fail (same topics, DDS domain 0).
- Saved maps: slam_toolbox `serialize_map`/`deserialize_map` (START_AT_GIVEN_POSE at the odom pose) work in async mode; ground-truth odom makes the saved map frame the world frame. slam_toolbox runs with `use_scan_matching: false` and `do_loop_closing: false` (a bad match during a long exploration had shifted `map -> odom` by 2.2 m / 19°); keep them off so `map` == world == Unity. `/slam_toolbox/save_map` fails (its map_saver times out on `/map`): `map_store.write_map_image` writes the PGM/YAML instead.

## Verification
`colcon build --symlink-install` in `ros2_ws`, `pytest tests/` (offline, incl. the bridge in mock mode), mock action servers for Nav2 actions, and the live checks above when CoppeliaSim is available. ROS 2 Jazzy: `source /opt/ros/jazzy/setup.bash`.
