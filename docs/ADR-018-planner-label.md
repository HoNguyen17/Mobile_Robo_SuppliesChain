# ADR-018: The Planner Label in Mission Run Summaries

**Status:** Accepted (2026-11-09) · label only, no behaviour · closes open issue 14

## Context
`mission_node.py` created its `MetricsCollector` with `planner_name="astar"`, so every run summary said `astar`, also when the mission ran on `dstar_planner` (D\* Lite). The mission node is normally started with `python3 .../mission_node.py` by `start_mission.bat` and by the Unity menu "2. Start Mission Node", not with `roslaunch`, so a launch-file argument of `mission.launch` never reaches it.

## Decision
- The label is read in this order: the node parameter `~planner_name`, then the global parameter `/nav/planner_name`, then `astar`.
- The launch file that starts a planner sets the global parameter: `bringup.launch` sets `astar` **only when it starts the A\* planner** (`if="$(arg nav)"`); `bringup_dstar.launch` sets `dstar`. It must be conditional: `bringup_dstar.launch` includes `bringup.launch` with `nav:=false`, and an unconditional `astar` in the included file would override the `dstar` set before the include.
- The node does not detect the planner by itself. If a planner is started by hand, set `/nav/planner_name` or `_planner_name:=...`.

## Consequences
- Mission summaries from `bringup_dstar.launch` say `dstar`, from `bringup.launch` say `astar`. The A/B benchmark (`ab_run.py`) is unaffected: it detects the planner from the running nodes.
- A stale `/nav/planner_name` cannot survive a restart, because the parameter lives on the ROS master, which `start_ros.bat` and Unity menu 3 kill.
- Tests: `test_launch_files.py` (each launch file sets the label of the planner it starts, `bringup.launch` only under `nav`, `bringup_dstar.launch` includes it with `nav:=false`), `test_mission_node_source.py` (the node reads the parameters in that order and no longer hardcodes the name).
