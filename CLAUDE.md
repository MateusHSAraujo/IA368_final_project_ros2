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

## Verification
`colcon build` in `ros2_ws`, `pytest tests/`, mock action servers for Nav2 actions. ROS 2 Jazzy: `source /opt/ros/jazzy/setup.bash`.
