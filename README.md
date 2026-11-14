# IA368 — Final Project: Mixed Reality HMI and Digital Twin for a Mobile Robot (ROS 2 side)

**Course:** Design and Development of Intelligent Autonomous Robots (IA368) — FEEC / Unicamp
**Author:** Mateus Henrique Silva Araújo

---

## Overview

Development of an **immersive Virtual/Mixed Reality Human-Machine Interface (HMI) (Meta Quest 3, via Unity)** for tactical planning, monitoring and first-person teleoperation of a **Pioneer P3-DX differential-drive mobile robot** simulated in **CoppeliaSim EDU**, integrated with the **ROS 2 (Jazzy)** and **Nav2** stack.

The system runs on a Digital Twin architecture split into two decoupled layers:
1. **Simulated Physical World layer (ZeroMQ):** procedurally rebuilds the CoppeliaSim scene in Unity and synchronizes the poses of moving bodies in real time. Implemented in the Windows repository (Stage 1, done).
2. **Mixed Reality HMI and Control layer (ROS 2):** on-demand VR Spatial Menu with a **Holographic Tactical Table** (minimap with *Point-and-Click*, *Path Painting* for `NavigateThroughPoses` and *No-Go Zone painting* for the Costmap Keepout Filter) and **First-Person Teleoperation** (`/camera/image_raw` + joystick with priority through `twist_mux`).

---

## Two repositories

| Repository | Machine | Contents |
|---|---|---|
| **`IA368_final_project_ros2`** (this one) | WSL2, `/home/mateus/ia368/projeto_final` | ROS 2 workspace, ROS-side tests, Linux scripts |
| [`IA368_final_project`](https://github.com/MateusHSAraujo/IA368_final_project) | Windows, `C:\Users\mateu\unicamp\IA368\projeto_final` | Unity project, CoppeliaSim scripts and scenes, **planning documents** |

The Windows folder is reachable from here through the `windows_workspace` symlink (not versioned). The planning documents at the root of this repository are symlinks into it, so there is a single copy: changes made through them belong to the Windows repository.

## Repository structure

```text
├── CLAUDE.md                # Instructions for the WSL agent (Claude Code)
├── PROJECT_PROPOSAL.md      -> windows_workspace/PROJECT_PROPOSAL.md     (symlink)
├── IMPLEMENTATION_PLAN.md   -> windows_workspace/IMPLEMENTATION_PLAN.md  (symlink)
├── STAGE1_UNITY_PLAN.md     -> windows_workspace/STAGE1_UNITY_PLAN.md    (symlink)
├── STAGE4_VR_LOCOMOTION_PLAN.md -> windows_workspace/STAGE4_VR_LOCOMOTION_PLAN.md (symlink)
├── ros2_ws/src/
│   ├── vr_robot_pkg/        # Bridge, mission supervisor, configs, bringup launch; README = ROS interface contract
│   └── ROS-TCP-Endpoint/    # git submodule (Unity-Technologies, main-ros2)
├── maps/                    # Saved SLAM maps (pose graph, map image, keepout strokes); scene_house = reference map of the house
├── scripts/
│   ├── get_windows_host_ip.sh   # Windows host IP for CoppeliaSim (--check PORT)
│   ├── setup_adb_reverse.ps1    # Quest 3 adb reverse / portproxy (mirrored to Windows)
│   ├── sync_to_windows.sh       # Copies scripts/*.ps1 to the Windows repo
│   └── coppelia/                # prepare_scene_house.py, install_fast_laser.py (scene tools)
└── tests/                   # Offline pytest suite; tests/live/ = checks against the running stack
```

## Status and quick start

Stages 2 and 3 are done, and the ROS side of Stage 5 is done: the Pioneer P3-DX in `scene_house.ttt` runs with SLAM, Nav2 (with keepout filter), a compressed front camera and ROS-TCP-Endpoint on port 10000. The mission supervisor handles point-and-click and path missions with pause/resume, keepout brush zones, and saved maps that can be loaded from Unity or at startup with `map:=scene_house`. See [`ros2_ws/src/vr_robot_pkg/README.md`](ros2_ws/src/vr_robot_pkg/README.md) for the interface contract used by Unity.

```bash
git submodule update --init
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch vr_robot_pkg vr_robot_bringup.launch.py          # CoppeliaSim on Windows with scene_house.ttt open
ros2 launch vr_robot_pkg vr_robot_bringup.launch.py map:=scene_house  # same, starting with the saved house map on /map
ros2 launch vr_robot_pkg vr_robot_bringup.launch.py mock_mode:=true   # without CoppeliaSim
cd .. && pytest tests/
```

For architecture details, network topology and the stage plan, see:
* [Project Proposal](PROJECT_PROPOSAL.md)
* [Implementation Plan](IMPLEMENTATION_PLAN.md)
