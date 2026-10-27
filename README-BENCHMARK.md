# A/B Benchmark & D* Lite Guide: UC6 Warehouse Robot

This document provides a comprehensive operational and technical manual for benchmarking the default A* global planner against the incremental D* Lite global planner in the UC6 Warehouse Robot project.

---

## 1. What This Is

The UC6 Warehouse Robot system supports two interchangeable global planners:
1. **A\* (`astar_planner.py`):** The baseline global planner. Re-computes paths from scratch using an inflated 8-connected grid with Bresenham line-of-sight smoothing.
2. **D\* Lite (`dstar_planner.py`):** An opt-in incremental global planner. Dynamically repairs rhs/g search trees when obstacles are detected, reusing prior searches when the goal remains unchanged.

The **A/B Benchmark Suite** is an autonomous test harness designed to execute repeatable, rigorous comparison trials between A* and D* Lite under identical simulation scenarios in Unity URP, collecting telemetry, computing non-parametric statistics, and generating publication-ready reports.

### File Manifest

| File Path | Component Role |
|---|---|
| `ros1/turtlebot_control/scripts/astar_planner.py` | Baseline A* global planner ROS node (updated with `expand_lidar_hit`, `~filter_static_hits`, adaptive `~blocked_use_plan_radius`, and `~obstacle_memory_s` for 100% perception/trigger parity). |
| `ros1/turtlebot_control/scripts/dstar_planner.py` | D* Lite global planner ROS node (subscribes to `/map`, `/robot/pose`, `/scan`, `/move_base_simple/goal`, `/nav/replan`; publishes `/planned_path`, `/costmap`, `/nav/leg_result`). |
| `ros1/turtlebot_control/scripts/dstar_lite.py` | Pure-Python D* Lite incremental graph search engine with line-of-sight smoothing and adaptive inflation. |
| `ros1/turtlebot_control/launch/navigation_dstar.launch` | Navigation launch file starting `dstar_planner` and `path_follower`. |
| `ros1/warehouse_bringup/launch/bringup_dstar.launch` | Master standalone launch file combining `bringup.launch nav:=false` with `navigation_dstar.launch`. |
| `WarehouseProjectURP/Assets/Scripts/Ros/Editor/DstarLaunchMenu.cs` | Unity Editor menu item (`Warehouse > 1D. Launch ROS System with D* Lite (WSL)`) for interactive testing. |
| `WarehouseProjectURP/Assets/Scripts/Ros/Editor/AbBenchmarkMenu.cs` | Unity Editor menu integration (`Warehouse > A-B Test`) driving automated benchmarking from the Editor. |
| `ros1/turtlebot_control/scripts/ab_batch.py` | Supervisory batch orchestrator managing ROS lifecycle, subprocess supervision, watchdog gates, and report dispatch. |
| `ros1/turtlebot_control/scripts/ab_run.py` | Single-stack test runner managing `/sim/reset_scenario` resets, settle immobility checks, goal dispatch, and 41-column CSV capture. |
| `ros1/turtlebot_control/scripts/ab_report.py` | Offline report generator generating `report.html` (interactive SVG charts) and `report.md` comparison tables. |
| `ros1/turtlebot_control/test/test_ab_tools.py` | Unit tests for `ab_run.py` metrics collection and `ab_report.py` table/footnote generation. |
| `ros1/turtlebot_control/test/test_ab_batch.py` | Unit tests for `ab_batch.py` process group isolation, XML-RPC readiness gating, and cleanup routines. |
| `ros1/turtlebot_control/test/test_launch_files.py` | Unit tests verifying node topology, argument passing, and parameter parity across launch files. |

---

## 2. Prerequisites & Environment Setup

### System Requirements
- **Host OS:** Windows 10/11 with WSL2.
- **WSL Distribution:** Ubuntu 20.04 (`Ubuntu-20.04`).
  > **Note on Distro Name:** The distribution name is defined as a constant in `AbBenchmarkMenu.cs`:
  > ```csharp
  > private const string WslDistro = "Ubuntu-20.04";
  > ```
  > (line 15 of `WarehouseProjectURP/Assets/Scripts/Ros/Editor/AbBenchmarkMenu.cs`). It is also hardcoded in `DstarLaunchMenu.cs` (line 17) and `RosBridgeSetupMenu.cs` (lines 94, 107, 120). If your WSL distribution is named differently in `wsl -l -v`, update this constant and strings accordingly.
- **ROS Version:** ROS 1 Noetic installed at `/opt/ros/noetic`.
- **Catkin Workspace:** `~/nav_ws` built with `catkin_make`. All menu commands source this environment:
  `"source /opt/ros/noetic/setup.bash && source ~/nav_ws/devel/setup.bash"` [verified in code: `AbBenchmarkMenu.cs` line 134, `DstarLaunchMenu.cs` line 17, `RosBridgeSetupMenu.cs` line 94].
- **Workspace Packages:** Symlinked packages in `~/nav_ws/src`:
  - `turtlebot_control` (`<repo>/ros1/turtlebot_control`)
  - `warehouse_bringup` (`<repo>/ros1/warehouse_bringup`)
  - `warehouse_mission` (`<repo>/ros1/warehouse_mission`)
  - `warehouse_eval` (`<repo>/ros1/warehouse_eval`)
  - `ROS-TCP-Endpoint` (v0.7.0 branch)
- **Unity Editor:** Version **2021.1.11f1** with project `WarehouseProjectURP` (`Assets/Scenes/Warehouse.unity`).
- **Python Version:** Python 3.8 inside WSL (`python3`, `python-is-python3`, `psmisc`).

For complete step-by-step installation commands, refer to `PROJECT.md` Section 5 ("How to Build and Run") and `ros1/turtlebot_control/RUN_STEPS.txt`.

---

## 3. Quick Start Guides

### Quick Start A: Live Interactive D* Lite Demo
1. In the Unity Editor, open scene `Assets/Scenes/Warehouse.unity`.
2. Click menu `Warehouse > 1D. Launch ROS System with D* Lite (WSL)` (or `Robotics > Warehouse > 1D...`). A terminal window opens launching `bringup_dstar.launch` and RViz.
3. Press **Play** in Unity. Wait 90 seconds for `/clock` time synchronization and TCP endpoint connection.
4. Click menu `Robotics > Warehouse > 2. Start Mission Node (WSL)`.
5. The robot will plan using D* Lite and drive autonomously toward shelf locations.
6. Stop after the first `"Arrived at the goal"` message appears in the terminal or Unity console.
7. Click `Robotics > Warehouse > 3. Clean Stop All ROS (WSL)` to terminate the stack.

### Quick Start B: One-Click Autonomous Benchmark
1. Open Unity scene `Warehouse.unity` and press **Play**.
2. Wait 90 seconds for `/clock` synchronization.
3. Click Unity menu `Warehouse > A-B Test > 4 Run Both + Report (5 each)`.
4. The benchmark will autonomously execute 5 runs of A* followed by 5 runs of D* Lite (~9 minutes total). Logs stream in real time to the Unity console.
5. Upon completion, `report.html` opens automatically in your default Windows browser.
6. To halt execution at any time, click `Warehouse > A-B Test > 6 Stop Batch`.

---

## 4. Standing Operational Rules (Before Every Batch)

Adhere strictly to these operational safeguards before and during benchmark runs:

1. **Fresh Play 90 s Warm-up Rule:** After pressing Play in Unity, wait at least 90 seconds before clicking any benchmark menu command. This ensures Unity's TCP socket endpoint (`ROSConnection`), background asset loading, and `/clock` time synchronization are fully established. Triggering too early risks socket handshake failures or physics stalls.
2. **Pause and Error Pause OFF:** Ensure Unity's **Pause** and **Error Pause** (Console options) are disabled. Any paused simulation tick blocks `/clock` and causes timeout failures in `ab_run.py`.
3. **Do NOT type O, 0, or B in Game View:** In `ScenarioLoader.cs`, `Update()` monitors keypresses for `KeyCode.O`, `KeyCode.Alpha0`, `KeyCode.Keypad0`, and `KeyCode.B`, calling `ToggleObstacleInFrontOfRobot()`, which spawns a 3.0 m tall dynamic box 2.5 m ahead of the robot.
4. **Never Drag `Dynamic_Draggable_Obstacle` Manually:** In `ScenarioLoader.cs`, `Start()` spawns it at `(2.5, 1.5, 10.0)`. Moving it manually introduces uncontrolled spatial variance.
5. **Do NOT Edit or Add `.cs` Files During a Run:** Editing C# scripts triggers an immediate Unity compilation and domain reload, halting Play mode and severing TCP sockets.
6. **Clean Stop First:** Never launch Menu 1D and an A/B batch at the same time. Menu 1D runs standalone without process group tracking. If switching between 1D and benchmark batches, always click `Robotics > Warehouse > 3. Clean Stop All ROS (WSL)` first.

---

## 5. Menu Reference & CLI Reference

### Unity Menu Reference

#### D* Lite Standalone Menu (`Warehouse` and `Robotics/Warehouse`)
- `1D. Launch ROS System with D* Lite (WSL)`:
  ```bash
  wsl.exe -d Ubuntu-20.04 bash -c "source /opt/ros/noetic/setup.bash && source ~/nav_ws/devel/setup.bash && roslaunch warehouse_bringup bringup_dstar.launch"
  ```
  [verified in code: `DstarLaunchMenu.cs` lines 14–20]

#### A/B Benchmark Menu (`Warehouse > A-B Test`)
All benchmark menu items invoke commands via `wsl.exe` using `BuildWslCommandLine`:
```bash
wsl.exe -d Ubuntu-20.04 --cd "<repo>" -e bash -c "source /opt/ros/noetic/setup.bash && source ~/nav_ws/devel/setup.bash && python3 -u ros1/turtlebot_control/scripts/ab_batch.py <extraArgs>"
```
[verified in code: `AbBenchmarkMenu.cs` lines 131–136]

- `1 Dry Run (A* then D*, 3 each)`:
  Runs `ab_batch.py --planner both --dry` (3 runs per planner, dry_ session prefix, skips report).
- `2 Run A* Batch (5)`:
  Runs `ab_batch.py --planner astar --repeats 5` (5 runs of A*, opens report on completion).
- `3 Run D* Lite Batch (5)`:
  Runs `ab_batch.py --planner dstar --repeats 5` (5 runs of D* Lite, opens report on completion).
- `4 Run Both + Report (5 each)`:
  Runs `ab_batch.py --planner both --repeats 5` (5 runs of A* then 5 runs of D* Lite, opens report on completion).
- `5 Open Latest Report`:
  Reads `~/ab_runs/latest_report.txt`, resolves the Windows path via `wslpath -w`, and opens `report.html` in the default browser.
- `6 Stop Batch`:
  Runs `ab_batch.py --stop`, signalling the active batch process group via graceful SIGINT/SIGTERM escalation.
- `7 Open Latest Session Folder`:
  Reads `~/ab_runs/latest_session.txt`, resolves the Windows path via `wslpath -w`, and opens the session directory in Windows File Explorer.

#### Utility Menu (`Robotics > Warehouse`)
- `3. Clean Stop All ROS (WSL)`:
  ```bash
  wsl.exe -d Ubuntu-20.04 bash -c "killall -9 rosmaster roslaunch python3 rviz gzserver 2>/dev/null"
  ```
  [verified in code: `RosBridgeSetupMenu.cs` lines 117–124]

---

### Command-Line Interface (`ab_batch.py`)

The supervisory orchestrator can be executed directly from WSL:

```bash
python3 ros1/turtlebot_control/scripts/ab_batch.py --help
```

```
usage: ab_batch.py [-h] [--planner {astar,dstar,both}] [--repeats REPEATS]
                   [--scenario SCENARIO] [--goal X Y] [--settle SETTLE]
                   [--timeout TIMEOUT] [--rviz] [--out OUT] [--dry] [--stop]
                   [--order ORDER]

A/B Benchmark Batch Orchestrator for A* vs D* Lite

optional arguments:
  -h, --help            show this help message and exit
  --planner {astar,dstar,both}
                        Planner to benchmark (default: both)
  --repeats REPEATS     Number of repetitions per planner (default: 5)
  --scenario SCENARIO   Scenario name (default: S-04)
  --goal X Y            Goal coordinates (default: -3.75 -3.13)
  --settle SETTLE       Settle time in sim seconds after reset (default: 7.0)
  --timeout TIMEOUT     Timeout in sim seconds per run (default: 120.0)
  --rviz                Launch RViz (default: False)
  --out OUT             Base output directory (default: ~/ab_runs)
  --dry                 Dry run: 3 repeats, session prefixed dry_, no report
  --stop                Stop a running batch
  --order ORDER         Execution order for planners (default: astar,dstar)
```

#### CLI Execution Examples
- **Full Benchmark (A\* then D\* Lite, 5 runs each, with report):**
  ```bash
  python3 ros1/turtlebot_control/scripts/ab_batch.py --planner both --repeats 5
  ```
- **A\* Only Benchmark (5 runs):**
  ```bash
  python3 ros1/turtlebot_control/scripts/ab_batch.py --planner astar --repeats 5
  ```
- **D\* Lite Only Benchmark (5 runs):**
  ```bash
  python3 ros1/turtlebot_control/scripts/ab_batch.py --planner dstar --repeats 5
  ```
- **Dry-Run Validation (3 runs each, fast smoke check, no report):**
  ```bash
  python3 ros1/turtlebot_control/scripts/ab_batch.py --planner both --dry
  ```
- **Graceful Process Termination:**
  ```bash
  python3 ros1/turtlebot_control/scripts/ab_batch.py --stop
  ```

---

## 6. Output Artifacts & Data Layout

### Session Directory Structure
Every batch execution creates an isolated, timestamped folder under `~/ab_runs/<timestamp>/` (e.g. `~/ab_runs/20261025_103721/`):

```
~/ab_runs/20261025_103721/
├── astar_runs.csv        # 41-column telemetry for A* runs
├── dstar_runs.csv        # 41-column telemetry for D* Lite runs
├── batch.log             # Interleaved supervisor, runner, and node output log
├── bringup_astar.log     # Stdout/stderr from A* bringup stack
├── bringup_dstar.log     # Stdout/stderr from D* Lite bringup stack
├── meta_astar.json       # Git state, rosparams, and file hashes for A*
├── meta_dstar.json       # Git state, rosparams, and file hashes for D*
├── handoff.txt           # Consolidated self-contained bundle (<400 KB)
└── report/
    ├── report.html       # Interactive HTML comparison report with SVG charts
    └── report.md         # Markdown summary tables with speedup ratios
```

### Pointer Files in `~/ab_runs/`
- `~/ab_runs/latest_session.txt`: Contains the absolute path to the most recent session folder.
- `~/ab_runs/latest_report.txt`: Contains the absolute path to the most recent `report.html`.
- `~/ab_runs/batch.pid`: Contains the process ID of the currently active batch (deleted on exit).

### How to View Reports
- **From Unity:** Menu `Warehouse > A-B Test > 5 Open Latest Report`.
- **From Windows:** Open File Explorer at `\\wsl$\Ubuntu-20.04\home\<user>\ab_runs\` or run:
  ```powershell
  wsl -d Ubuntu-20.04 bash -c "explorer.exe \$(wslpath -w \$(cat ~/ab_runs/latest_report.txt))"
  ```

### CSV Telemetry Columns (41 Columns)
Defined in `CSV_COLUMNS` (`ab_run.py` lines 22–65):

| Column | Type | Description |
|---|---|---|
| `run_id` | str | Unique run identifier (`<label>_<timestamp>`). |
| `label` | str | Benchmark series label (`astar` or `dstar`). |
| `planner` | str | Active planner node name (`astar_planner` or `dstar_planner`). |
| `scenario` | str | Scenario identifier (`S-04`). |
| `goal_x`, `goal_y` | float | Target coordinates dispatched to `/move_base_simple/goal`. |
| `run_index` | int | 1-based iteration counter within the series. |
| `outcome` | str | Run status: `succeeded`, `timeout`, `aborted`, or `invalid`. |
| `sim_time_to_goal_s` | float | Simulation seconds elapsed from goal publish to leg completion. |
| `wall_time_s` | float | Real-world wall-clock seconds elapsed for the leg. |
| `driven_m` | float | Cumulative distance traversed by the robot odometry pose. |
| `straight_m` | float | Euclidean distance from start pose to nominal target goal. |
| `displacement_m` | float | Straight-line distance from first recorded pose to last recorded pose. |
| `path_ratio` | float | Path efficiency ratio: `driven_m / displacement_m` ($\ge 1.0$). |
| `stops` | int | Count of complete robot stops ($v_{lin} = 0$). |
| `collisions` | int | Total collision contact events reported on `/sim/collision`. |
| `min_clearance_m` | float | Minimum clearance distance to obstacles recorded by LiDAR. |
| `plans` | int | Total plan calculations executed during the leg (initial + replans). |
| `early_replans` | int | Replan operations triggered dynamically by obstacle scan check. |
| `first_plan_ms` | float | Initial planning computation time in milliseconds. |
| `first_plan_rebuilt` | bool | True if first plan required graph rebuilding (`rebuilt=True`). |
| `replan_ms_median` | float | Median computation latency of replan operations in the run. |
| `replan_ms_p95` | float | 95th percentile computation latency of replans in the run. |
| `replan_ms_max` | float | Maximum computation latency of replans in the run. |
| `expanded_total` | int | Total graph vertices expanded during search. |
| `rebuilds` | int | Count of full graph rebuild events during the run. |
| `level_max` | int | Maximum search tree depth or inflation level reached. |
| `started_at`, `ended_at` | str | ISO-8601 UTC timestamps for run start and end. |
| `settle_plans` | int | Spurious plans generated during post-reset settle window. |
| `invalid_reason` | str | Error description if run flagged invalid (`settle movement`, etc.). |
| `first_plan_path_ready_ms` | float | Time to publish path before costmap generation (D* Lite). |
| `replan_path_ready_median_ms` | float | Median path-ready latency across replans (D* Lite). |
| `replan_path_ready_p95_ms` | float | 95th percentile path-ready latency across replans (D* Lite). |
| `settle_diag` | str | Downsampled settle trajectory telemetry JSON. |
| `settle_retry` | int | Number of retry reset attempts required during settle (0 or 1). |
| `max_xte_m` | float | Maximum cross-track error perpendicular to active planned polyline. |
| `mean_xte_m` | float | Mean cross-track error perpendicular to active planned polyline. |
| `heading_osc` | int | Sign reversals of angular velocity $w_z$ with $|w_z| > 0.2$ rad/s. |
| `path_switches` | int | Total `/planned_path` publications received during the leg. |
| `plan_ms_total` | float | Sum of milliseconds across all `Planned:` lines in the leg. |
| `plan_share_pct` | float | Fraction of simulation run time spent planning: `100 * plan_ms_total / 1000 / sim_time_to_goal_s`. |

### Environment Metadata (`meta_*.json`)
- `git_commit`: Active git commit SHA at batch launch.
- `git_status`: Short porcelain status or timeout fallback.
- `git_diff_stat`: Diff stat summary of uncommitted modifications.
- `file_sha256`: SHA256 integrity hashes of key navigation and benchmarking source files.

---

## 7. Reading the Results & Statistical Guidelines

### Report Footnotes
1. **Footnote 1 (`FOOTNOTE_TIMER`):** Total computation timer inside the `Planned: ...` log line. A* publishes costmap and path before reading the timer. D* Lite logs `path_ready` immediately after path publication, before costmap generation. Total `ms` must be compared for like-for-like algorithmic fairness.
2. **Footnote 2 (`FOOTNOTE_REUSE`):** D* Lite reuses its search graph across successive goals; only `rebuilt=True` first plans represent cold-start searches.
3. **Footnote 3 (`FOOTNOTE_MOTION`):** Cross-track error (XTE) measures perpendicular deviation from the active `/planned_path`. Because every replan originates at the robot's current pose, XTE resets toward 0 upon replanning; XTE is comparable only between planners with similar `path_switches`. `path_ratio` is `driven_m / displacement_m` ($\ge 1.0$).
4. **Footnote 4 (`FOOTNOTE_P95`):** Displays median of run medians alongside worst-case tail performance across runs (`p95_largest` convention).
5. **Footnote 5 (`FOOTNOTE_FIRST_PLAN`):** First-plan latencies are explicitly distinguished as `First Plan ms (cold)` (`rebuilt=True`) or `First Plan ms (warm)` (`rebuilt=False`).
6. **Footnote 6 (`FOOTNOTE_PLANNER_TIME`):** Cumulative ms computation timer across all `Planned:` lines. Only plans started by the dynamic-obstacle check run synchronously inside `on_scan`; first plans and goal changes run in `on_goal`, and lookahead replan requests run in `on_replan` in separate subscriber callback threads.

### Statistical Interpretation & Sample Size ($n=5$ Caution)
With $n=5$ runs per planner, individual outliers can distort sample means. Differences that fall within the run-to-run min-max spread should be treated as not established. Pooling multiple batches ($n=10$ per planner) yields rigorous statistical confidence.

### Pooled Benchmark Results Table (S-04 Crossing NPC, n=10 per planner)
Data pooled from Batch 1 (`~/ab_runs/20261025_071408/`) and Batch 2 (`~/ab_runs/20261025_103721/`) [verified in data]:

| Metric | A* (`astar_planner`) | D* Lite (`dstar_planner`) | Ratio (A*/D*) | Exact Permutation p-value | Significance |
|---|:---:|:---:|:---:|:---:|:---:|
| **Outcome (Success Rate)** | 10/10 (100%) | 10/10 (100%) | 1.00x | - | Succeeded |
| **Sim Time to Goal (s)** | 40.35 s [37.66, 43.56] | 40.02 s [37.27, 41.81] | 1.01x | p = 0.760 | No difference |
| **Driven Distance (m)** | 9.050 m [8.921, 9.315] | 8.958 m [8.892, 9.193] | 1.01x | p = 0.239 | No difference |
| **Path Ratio** | 1.056 [1.040, 1.076] | 1.045 [1.037, 1.071] | 1.01x | p = 0.253 | No difference |
| **Min Clearance (m)** | 0.227 m [0.148, 0.266] | 0.185 m [0.150, 0.213] | 1.19x | **p = 0.029** | Suggestive (~3–6 cm less clearance for D*) |
| **Early Replans** | 4.0 [1, 6] (mean 3.6) | 7.5 [6, 8] (mean 7.2) | 0.50x | **p < 0.001** | Significant (D* replans 2x more often) |
| **Path Switches** | 14.0 [9, 16] (mean 13.2) | 17.0 [13, 20] (mean 16.8) | 0.79x | **p = 0.004** | Significant |
| **Heading Oscillations** | 3.0 [1, 7] (mean 3.5) | 4.0 [3, 5] (mean 4.0) | 0.88x | p = 0.558 | No difference |
| **Replan ms (Median of Medians)**| **205.8 ms** [188.0, 224.0] | **19.5 ms** [18.0, 26.5] | **10.55x** | **p < 0.001** | **Decisive (D* 10x faster replan)** |
| **Plan ms Total per Run (s)** | **3.21 s** [1.96, 3.43] (mean 2.96 s) | **0.57 s** [0.49, 0.97] (mean 0.64 s) | **4.65x** | **p < 0.001** | **Decisive (D* uses 4.6x less CPU time)** |
| **Plan Share % of Sim Time** | **7.83%** [5.20%, 8.52%] (mean 7.31%) | **1.43%** [1.18%, 2.34%] (mean 1.58%) | **4.63x** | **p < 0.001** | **Decisive (1.6% vs 7.3% CPU share)** |
| **First Plan Cold (ms)** | 236.0 ms [220.0, 244.0] (n=10) | 474.0 & 503.0 ms (n=2) | 0.49x | - | D* cold build 2.0x–2.1x slower |
| **First Plan Warm (ms)** | N/A | 81.5 ms & 88.0 ms (n=8) | - | - | D* warm start 2.8x faster than A* cold |

### 7.4 100% Fair Parity Benchmark (Batch 20261026_214123, n=5 per planner)

In Batches 1 and 2 above, `astar_planner.py` retained static LiDAR returns (~1000 cells), dilating walls and forcing A* southwards ($y \approx -2.18\text{ m}$), away from the crossing worker ($y = -1.65\text{ m}$). This masked early replans in A* (3.6 vs 7.2) and created an artificial clearance difference.

With `astar_planner.py` updated to include `expand_lidar_hit(..., filter_static=True)`, adaptive check radius `~blocked_use_plan_radius`, and `~obstacle_memory_s=6.0` (commit `3a1ab03`), both planners were evaluated under identical perception conditions:

| Metric | A* (`astar_planner`) | D* Lite (`dstar_planner`) | Ratio (A*/D*) | Status / Key Insight |
|---|:---:|:---:|:---:|---|
| **Dynamic Obstacles (First Plan)** | 43 cells (n=5) | 43 cells (n=5) | 1.00x | **Identical perception** [verified in log] |
| **Success Rate** | 5/5 (100%) | 5/5 (100%) | 1.00x | All 10 runs succeeded |
| **Sim Time to Goal (s)** | 39.3 s [38.1, 41.7] (mean 39.66) | 39.4 s [38.8, 42.0] (mean 39.88) | 1.00x | Identical transit duration |
| **Driven Distance (m)** | 8.96 m [8.91, 9.16] (mean 9.00) | 9.02 m [8.93, 9.26] (mean 9.04) | 0.99x | Identical corridor trajectory |
| **Min Clearance (m)** | 0.22 m [0.19, 0.23] (mean 0.210) | 0.18 m [0.16, 0.20] (mean 0.183) | 1.21x | Realistic corridor clearance |
| **Early Replans** | **6.0 [6.0, 8.0] (mean 6.4)** | **7.0 [4.0, 10.0] (mean 6.6)** | **0.97x** | **Early-replan gap eliminated (6.4 vs 6.6)** |
| **Path Switches** | 16.0 [15.0, 20.0] (mean 16.6) | 17.0 [12.0, 18.0] (mean 15.4) | 1.08x | Consistent replan response |
| **First Plan ms (cold)** | 279.0 ms [208.0, 346.0] | 571.0 ms (n=1) | 0.49x | D* cold start 2.0x slower |
| **First Plan ms (warm)** | N/A | 82.0 ms [66.0, 125.0] (n=4) | - | D* warm start 3.4x faster than A* cold |
| **Replan ms (Median of Medians)**| **188.5 ms** [185.5, 195.0] | **24.5 ms** [22.0, 28.0] | **7.69x** | **Decisive (D* 7.7x faster replanning)** |
| **Plan ms Total per Run (s)** | **3.17 s** [3.07, 4.04] (mean 3.41 s) | **0.53 s** [0.43, 0.94] (mean 0.60 s) | **5.94x** | **Decisive (D* uses 5.9x less CPU time)** |
| **Plan Share % of Sim Time** | **8.1%** [7.4%, 10.1%] | **1.4%** [1.0%, 2.4%] | **5.7x** | **1.4% vs 8.1% CPU share** |

---

## 8. Running the Unit Tests

The test suite requires no active ROS master or running Unity Editor. Execute from WSL:

```bash
cd <repo>/ros1/turtlebot_control && python3 -m unittest discover -s test
```

**Expected Result:**
```
Ran 251 tests in ~30s
OK
```
[verified in code: `test_launch_files.py`, `test_ab_tools.py`, `test_ab_batch.py`, `test_dstar_lite.py`, `test_grid_planner.py`, `test_path_tracker.py`]

---

## 9. Troubleshooting & FAQ

### 1. False "Unity not connected" immediately after pressing Play
- **Cause:** TCP socket handshake between Unity `ROSConnection` and WSL takes several seconds to bind.
- **Fix:** Wait at least 90 seconds after entering Play mode before triggering benchmark commands.

### 2. Simulation clock stall when clicking menu too soon
- **Cause:** Unity physics loop and `/clock` time publishing stall if initialized while assets are streaming.
- **Fix:** Keep Play mode active, wait 90 seconds, and ensure **Error Pause** is disabled in Unity Console.

### 3. Settle Retry triggered by stale goal replan
- **Cause:** Neither planner handles `/nav/cancel`; when Unity resets the robot to `(5.01, -0.05)`, obstacle hits near the old corridor path trigger an early replan, rotating the robot during settle.
- **Fix:** `ab_run.py` includes autonomous self-healing: it retries reset on attempt 2, cancelling velocity commands and stabilizing settle.

### 4. `git_status` taking 35–38 seconds
- **Cause:** Cold WSL I/O across the Windows `/mnt/c` filesystem mount can take over 35 seconds for git status queries.
- **Fix:** `ab_batch.py` captures git status once per batch with a 120-second timeout before starting ROS bringup, logging duration to `batch.log`.

### 5. Benign "Not registered to publish topic '/scan'" warnings
- **Cause:** Transient handshake race condition during `ROSConnection` initialization at `sim_time: 0.000000`.
- **Fix:** Harmless; safe to ignore. Disappears immediately once topics register.

### 6. WSL distribution name mismatch
- **Cause:** Menu commands target `Ubuntu-20.04`. If your distro is named `Ubuntu` or `Ubuntu-22.04`, launch fails.
- **Fix:** Edit `private const string WslDistro = "Ubuntu-20.04";` in `AbBenchmarkMenu.cs` (line 15) and command strings in `DstarLaunchMenu.cs` (line 17) and `RosBridgeSetupMenu.cs` (lines 94, 107, 120).

### 7. Two planners running simultaneously after launching Menu 1D
- **Cause:** Menu 1D runs standalone and does not terminate background processes when a batch starts.
- **Fix:** Click `Robotics > Warehouse > 3. Clean Stop All ROS (WSL)` before starting an A/B benchmark batch.

### 8. Unity domain recompile during a batch
- **Cause:** Modifying or generating `.cs` files in the Assets folder while Play mode is running forces Unity to recompile scripts, exiting Play mode.
- **Fix:** Never edit or create C# scripts while an A/B batch is in progress.

---

## 10. Known Limitations & Architectural Notes

1. **Evaluated Scenario:** Automated benchmarking has been executed exclusively on Scenario `S-04` (crossing dynamic NPC). Scenarios S-00 through S-03 and S-05 remain available for future testing.
2. **Fixed Goal Snapping vs Multi-Goal Missions:** D* Lite demonstrates high warm-start efficiency (~80 ms) when the goal is unchanged. In multi-waypoint missions where targets change at every leg, D* Lite rebuilds its search graph from scratch (`rebuilt=True`, 427–571 ms).
3. **Hardcoded Planner Label in `mission_node.py`:** `mission_node.py` line 44 hardcodes parameter `planner_name` to `"astar"`. When D* Lite is active under Menu 1D, mission summary logs output `planner: astar`. This is purely an informational string label; navigation executes via D* Lite.
4. **Absence of `/nav/cancel` Handling:** Neither `astar_planner.py` nor `dstar_planner.py` subscribes to `/nav/cancel`. Stale paths persist in node memory until a new goal is received.
