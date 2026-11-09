# A/B Navigation Benchmark Report

Benchmarking `astar_planner` vs `dstar_planner` side-by-side.

## Scenario: S-04

### Overview
- **A***: 5 runs total, 0 invalid. Outcomes: 5 succeeded, 0 aborted, 0 timeout.
- **D***: 5 runs total, 0 invalid. Outcomes: 5 succeeded, 0 aborted, 0 timeout.

| Metric | A* | D* | Ratio (A*/D*) |
| :--- | :--- | :--- | :--- |
| Success Rate | 5/5 (n=5) | 5/5 (n=5) | 1.00x |
| Sim Time to Goal | 39.3 s [38.1, 41.7] (n=5) | 39.4 s [38.8, 42.0] (n=5) | 1.00x |
| Driven Distance | 8.96 m [8.91, 9.16] (n=5) | 9.02 m [8.93, 9.26] (n=5) | 0.99x |
| Path Ratio | 1.04 [1.04, 1.07] (n=5) | 1.05 [1.04, 1.08] (n=5) | 0.99x |
| Stops | 0.0 [0.0, 0.0] (n=5) | 0.0 [0.0, 0.0] (n=5) | - |
| Min Clearance | 0.22 m [0.19, 0.23] (n=5) | 0.18 m [0.16, 0.20] (n=5) | 1.21x |
| Collisions | 0 total (0 runs) (n=5) | 0 total (0 runs) (n=5) | - |
| Max XTE | 0.127 m [0.101, 0.139] (n=5) | 0.115 m [0.097, 0.120] (n=5) | 1.10x |
| Mean XTE | 0.018 m [0.014, 0.018] (n=5) | 0.014 m [0.009, 0.015] (n=5) | 1.29x |
| Heading Oscillations | 3.0 [3.0, 5.0] (n=5) | 5.0 [3.0, 5.0] (n=5) | 0.60x |
| Path Switches | 16.0 [15.0, 20.0] (n=5) | 17.0 [12.0, 18.0] (n=5) | 0.94x |
| First Plan ms (cold) | 279.0 ms [208.0, 346.0] (n=5) | 571.0 ms [571.0, 571.0] (n=1, indicative only) | 0.49x |
| First Plan ms (warm) | - | 82.0 ms [66.0, 125.0] (n=4, indicative only) | - |
| Replan ms (median) | 188.5 ms (median of run medians; worst-run p95: 270.3, worst-run max: 322.0) | 24.5 ms (median of run medians; worst-run p95: 69.0, worst-run max: 89.0) | 7.69x |
| ↳ Replan Path Ready ms (D* only) | - | 17.0 ms (median of run medians; worst-run p95: 61.5) | - |
| Planner time per run (s) | 3.17 s [3.07, 4.04] (n=5) | 0.53 s [0.43, 0.94] (n=5) | 5.94x |
| Planner time share of run (%) | 8.1 % [7.4, 10.1] (n=5) | 1.4 % [1.0, 2.4] (n=5) | 5.93x |

### Permutation test (exact, two-sided, on means)

| Metric | A* mean | D* mean | A* - D* | p |
| :--- | :--- | :--- | :--- | :--- |
| Sim Time to Goal (s) | 39.66 (n=5) | 39.88 (n=5) | -0.23 | 0.786 |
| Driven Distance (m) | 8.997 (n=5) | 9.043 (n=5) | -0.047 | 0.540 |
| Path Ratio | 1.048 (n=5) | 1.054 (n=5) | -0.005 | 0.571 |
| Stops | 0.00 (n=5) | 0.00 (n=5) | 0.00 | 1.000 |
| Min Clearance (m) | 0.210 (n=5) | 0.183 (n=5) | 0.026 | 0.056 |
| Max XTE (m) | 0.124 (n=5) | 0.111 (n=5) | 0.013 | 0.135 |
| Mean XTE (m) | 0.016 (n=5) | 0.013 (n=5) | 0.004 | 0.079 |
| Heading Oscillations | 3.80 (n=5) | 4.20 (n=5) | -0.40 | 1.000 |
| Path Switches | 16.60 (n=5) | 15.40 (n=5) | 1.20 | 0.532 |
| Plans per Run | 16.60 (n=5) | 15.40 (n=5) | 1.20 | 0.532 |
| Early Replans | 6.40 (n=5) | 6.60 (n=5) | -0.20 | 1.000 |
| Replan ms (per-run median) | 189.3 (n=5) | 25.0 (n=5) | 164.3 | 0.008 |
| Planner Time per Run (s) | 3.411 (n=5) | 0.602 (n=5) | 2.809 | 0.008 |
| Planner Time Share of Run (%) | 8.62 (n=5) | 1.52 (n=5) | 7.10 | 0.008 |

---
### Footnotes
1. ms is the timer inside the Planned log line. A* publishes the costmap and the path before reading it. D* publishes the path first and reads path_ready right after it, before the costmap is published. Compare ms; path_ready is not like for like.
2. D* reuses its search state when the goal repeats, so only rebuilt=True first plans are cold.
3. max_xte_m and mean_xte_m: distance from each recorded pose to the polyline of the /planned_path that was active at that time. Each plan starts at the robot pose, so XTE falls toward 0 at every replan and is only comparable between planners with a similar path_switches; heading_osc, stops and path_ratio are the veering evidence. heading_osc: sign changes of angular.z with |w| > 0.2 rad/s during the leg. path_switches: number of /planned_path messages during the leg (counts the first path too). path_ratio is driven_m / displacement_m (>= 1.0), where displacement_m is the straight-line distance from the first recorded pose to the last recorded pose of the leg. straight_m is the distance from start to nominal goal; planners snap blocked or unreachable goals to the nearest collision-free cell, so displacement_m reflects actual endpoint distance.
4. Ratio (A*/D*) above 1 means D* has the lower value for time and count metrics.
5. n per planner and [min, max] ranges: with n=5 treat differences inside the run-to-run spread as not established.
6. Sum of the ms timer of all Planned lines in the leg; only plans started by the dynamic-obstacle check run inside the /scan callback [verified in code: astar_planner.py line 181 'self.plan_for_goal(self.latest_goal)', dstar_planner.py line 234 'self.plan_for_goal(self.latest_goal)'], first plans, goal changes and lookahead replan requests run in other callbacks [verified in code: astar_planner.py line 228 'self.plan_for_goal(msg)', astar_planner.py line 233 'self.plan_for_goal(self.latest_goal)', dstar_planner.py line 294 'self.plan_for_goal(msg)', dstar_planner.py line 299 'self.plan_for_goal(self.latest_goal)']; a ROS Python node runs each subscriber callback in its own thread [verified in code: path_tracker.py line 3-4 'ROS callbacks run on their own threads'], so only the scan-triggered part delays scan processing.
7. Permutation test: exact and two-sided, on the difference of means. Every way of splitting the pooled runs into two groups of the same sizes is enumerated, and p is the share of splits whose absolute difference of means is at least the observed one (ties count). With n=5 per planner the smallest possible p is 2/252 = 0.0079. No correction for the number of metrics tested; with n=5 read p as indicative.
