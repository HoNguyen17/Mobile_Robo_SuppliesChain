# ADR-007: Unity on Windows + ROS in WSL2

**Status:** Accepted · supersedes the earlier "Ubuntu dual-boot on every machine" decision

## Context
The team develops on Windows, and the Unity project already works there. ROS Noetic needs Ubuntu 20.04. The Unity↔ROS link is plain TCP, so the two sides do not have to share an OS.

## Decision

| Runs on **Windows** | Runs in **WSL2 Ubuntu 20.04** |
|---------------------|-------------------------------|
| Unity 2021.1.11f1 (URP) | ROS Noetic, `ros_tcp_endpoint`, `move_base`, our nodes |
| IDE / git, Docker Desktop (engine) | `docker compose` + MySQL (via WSL integration), headless runner, RViz (via WSLg) |

- **Networking:** WSL `networkingMode=nat` + `localhostForwarding=true`, so Unity connects to `127.0.0.1:10000`. Mirrored mode was tried and rejected: the Hyper-V firewall dropped Windows → WSL connections even with an explicit allow rule (tested 2026-09-30).
- **Code location:** one git clone on Windows. WSL symlinks `ros/src` into `~/catkin_ws/src`; build output stays on the Linux disk.
- **Clock:** Unity publishes `/clock`, and ROS uses sim time ([ADR-009](ADR-009-simulation-clock.md)). This avoids clock differences between Windows and WSL.

Step-by-step install: [setup-windows-wsl.md](setup-windows-wsl.md).

## Consequences
- No dual boot, and everyone uses the same setup.
- Localhost forwarding works on both Windows 10 and 11. If it fails on a machine, the fallback is to set the WSL IP in Unity each session (setup guide, Appendix A).
- Files under `/mnt/c` need LF line endings and `metadata` mount options (handled in the setup guide).
- Latency across the bridge is local loopback (well under 1 ms), which does not matter at 5–30 Hz sensor rates.

## Alternatives rejected
| Option | Why not |
|--------|---------|
| Everything on Ubuntu (dual boot) | Requires rebooting; Unity on Linux is less stable; the team works on Windows |
| ROS inside a Docker container on Windows | Extra layer for networking and GUI (RViz); WSLg already handles GUI |
| A VM (VirtualBox/VMware) | Slower, with manual network bridging |
