# Auto_SupplyChain

Autonomous mobile-robot supply-chain system. Start with `docs/README.md` (ROS 1 Noetic in WSL2 + Unity on Windows, single robot).

## Environment

- Unity 2021.1.11f1 (URP, project `WarehouseProjectURP`) on Windows. ROS 1 Noetic in the WSL2 distro `Ubuntu-20.04`, catkin workspace `~/nav_ws` with symlinks to `ros1/*`. Unity and ROS talk over TCP `127.0.0.1:10000`.
- On Windows the shell is Git Bash, not WSL. Run ROS and the Python tests inside WSL:
  `wsl -d Ubuntu-20.04 --cd "<repo windows path>" -e bash -c "source /opt/ros/noetic/setup.bash && source ~/nav_ws/devel/setup.bash && <command>"`
  Windows Python gives false test failures (`signal.SIGKILL` and file locking in the `ab_batch` tests, `:` in `PYTHONPATH`).
- Full install steps: `README.md` and `docs/setup-windows-wsl.md`.

## Commands (inside WSL, from the repo root)

- **Tests** (plain `unittest`, no pytest; no ROS master or Unity needed). All tests should pass.
  - `cd ros1/turtlebot_control && python3 -m unittest discover -s test -v`
  - `cd ros1 && PYTHONPATH="warehouse_mission/src:warehouse_eval/src" python3 -m unittest discover -s warehouse_mission/test -v`
  - `cd ros1 && PYTHONPATH="warehouse_mission/src:warehouse_eval/src" python3 -m unittest discover -s warehouse_eval/test -v`
- **Build:** `cd ~/nav_ws && catkin_make`
- **Run:** Unity menu `Robotics > Warehouse > 1. Launch ROS System & RViz`, press Play, `2. Start Mission Node`, `3. Clean Stop All ROS`. Or `start_ros.bat`, Play, `start_mission.bat`, `stop_ros.bat`. D\* Lite instead of A\*: menu `Warehouse > 1D. Launch ROS System with D* Lite (WSL)`. Manual: `roslaunch warehouse_bringup bringup.launch`.
- **A/B planner benchmark:** `README-BENCHMARK.md` (Unity menu `Warehouse > A-B Test`, or `ros1/turtlebot_control/scripts/ab_batch.py`).
- **Database (optional):** `cd infra && cp .env.example .env && docker compose up -d`.

## Conventions

- ROS nodes are Python with `rospy`; logic lives in pure-Python modules without ROS imports so it is testable anywhere.
- Standard message types only, no custom `.msg`.
- Topics and parameters follow `docs/architecture.md` §5 and the interface contract in `README.md`.
- Coordinates are in robot metres, frame `map`, simulation time (`/clock`).
- Use the words defined in `CONTEXT.md` in code, issues, commits and tests.
- Follow the patterns already in the repo instead of generic approaches. Verify against the current code before making claims.
- Reference files by path relative to the repo root.
- New ADRs go in `docs/` as `ADR-NNN-<slug>.md`, continuing the numbering.

## Do not touch

- `_archive/`, `docs/_archive/`: superseded work.
- `WarehouseProjectHDRP/`, `com.unity.robotics.warehouse.*`, `.yamato/`, `Documentation/`: upstream Unity material; the packages may be hidden dependencies.
- `WarehouseProjectURP/Library/`, `Temp/`, `Logs/`: generated, git-ignored.
- Ask before deleting or restructuring files.
- `start_ros.bat`, `stop_ros.bat` and Unity menu 3 run `killall -9 ... python3`, which kills every python3 process in the WSL distro, including a running A/B batch. Never use them during a batch, and do not use `killall` or `pkill` in new code.

## Agent skills

### Issue tracker

Issues live in GitHub Issues on `HoNguyen17/Mobile_Robo_SuppliesChain`, driven by the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Domain docs

Single-context: one `CONTEXT.md` at the repo root, ADRs as `docs/ADR-NNN-*.md`. See `docs/agents/domain.md`.
