# ADR-003: ROS 1 Noetic

**Status:** Accepted · supersedes the earlier ROS 2 plan (`_archive/2026-09-30-ros2-nav2-plan/`)

## Context
The course **requires ROS 1**. Noetic is the only ROS 1 release with Python 3 and current Unity support. It runs only on Ubuntu 20.04.

## Decision
Use **ROS 1 Noetic** on **Ubuntu 20.04**, inside WSL2 ([ADR-007](ADR-007-windows-wsl-environment.md)).

## Consequences
- Noetic reached **end of life on 31 May 2025**. Binaries are still hosted on `packages.ros.org`, but there are no further fixes. That is acceptable for a closed simulation project with no internet-facing parts.
- Installation must use the new `ros-apt-source` key package; older `apt-key` guides fail ([setup guide §3](setup-windows-wsl.md#3-install-ros-1-noetic)).
- In P0, freeze the installed package versions: `dpkg -l 'ros-noetic-*' > infra/ros-packages.lock`.
- Unity's bridge supports ROS 1: `ROS-TCP-Endpoint` (catkin, `main` branch, pinned to `v0.7.0`).
- Nodes use `rospy`, `actionlib` and `dynamic_reconfigure`, all mature in Noetic.

## Alternatives rejected
| Option | Why not |
|--------|---------|
| ROS 2 (Humble/Jazzy) + Nav2 | Not allowed by the course |
| ROS 1 Melodic | Older, Python 2, Ubuntu 18.04 |
