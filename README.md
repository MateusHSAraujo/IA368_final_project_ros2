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
├── VR_for_Robotics_Google_Gemini.pdf   # Reference material (in Portuguese)
├── ros2_ws/                 # ROS 2 Jazzy workspace (vr_robot_pkg and ROS-TCP-Endpoint)
├── scripts/                 # Automation scripts (sync_to_windows.sh, network helpers)
└── tests/                   # Offline automated tests in Python (pytest)
```

For architecture details, network topology and the stage plan, see:
* [Project Proposal](PROJECT_PROPOSAL.md)
* [Implementation Plan](IMPLEMENTATION_PLAN.md)
