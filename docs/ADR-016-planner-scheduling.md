# ADR-016: Planner Scheduling and /nav/cancel

**Status:** Accepted (2026-11-09) · applies to `astar_planner` and `dstar_planner` · prepares milestone 3 (dynamic obstacles)

## Context
Three things were wrong in both planner nodes (`ros1/turtlebot_control/scripts/`), found in the audit of 2026-11-09 and listed as open issues 10, 18 and 19:

1. **No lock around planning.** `on_goal`, `on_replan` and `on_scan` each called `plan_for_goal` on their own rospy callback thread, so two plans could overlap and publish in the wrong order (seen in a log at sim time 134.97). The obstacle dictionary had a lock; the plan state (`current_path_corners`, `current_path_radius`, `last_auto_replan_time`) did not.
2. **A replan inside `on_scan` blocked scan processing.** The dynamic-obstacle trigger planned inline, so the scan callback stayed busy for the whole search (A\* about 190 ms, D\* up to 770 ms in clutter). For moving obstacles that is the callback that must stay fast.
3. **The planners ignored `/nav/cancel`.** Only `path_follower` listened. After a cancel the planner kept `latest_goal` and `current_path_corners`, so the next lidar hit near the old path started a replan, published a path, and the follower (which accepts any new path) drove off. `ab_run.py` works around it with a retry. Evidence: batches `20261025_071408` and `20261026_194405` each have an A\* run 3 with "robot moved > 0.3 m during settle".

## Decision
- A pure-Python `PlanScheduler` (`scripts/plan_scheduler.py`, no ROS imports) runs the plans on **one worker thread, one plan at a time**.
- The callbacks only post requests and return: `on_goal` posts `GOAL`, `on_replan` posts `REPLAN`, the `on_scan` trigger posts `SCAN_TRIGGER`. Nothing else calls `plan_for_goal`.
- **At most one request waits.** A new request replaces the waiting one if its priority is the same or higher (`GOAL` > `REPLAN` > `SCAN_TRIGGER`); a lower one is dropped because the waiting plan already covers it. The planner's own 1 s throttle for `SCAN_TRIGGER` stays.
- **Stale results are not published.** A running plan cannot be interrupted, but its ticket reports `cancelled` after a `/nav/cancel` or after a newer goal arrived. `plan_for_goal` checks it after the search and before publishing the costmap or the path, and `abort()` does not send `no_path` for a cancelled plan.
- **Both planners subscribe to `/nav/cancel`.** `on_cancel` cancels the scheduler and resets `latest_goal`, `current_path_corners` and `current_path_radius`. The obstacle memory (6 s) is kept. A goal that arrives later plans normally.
- An exception in a plan is logged (`Planning failed with an unexpected error`) and the worker keeps going.
- The scheduler thread stops on ROS shutdown.

## Consequences
- Overlapping plans are impossible by construction; the scan callback no longer blocks while planning.
- **Latency changes.** A replan now starts after the callback returns and after any plan in progress, so the first reaction to a new obstacle can be a few milliseconds later and a second request waits for the first search to end. The A\*/D\* numbers measured before this change (`docs/benchmark-data/20261026_214123/`) are not guaranteed to hold; the milestone 3 baseline is measured after it.
- The follower and the mission node need no change: `/nav/cancel` was already sent on leg timeouts and by `ab_run.py`, and the follower already reported `cancelled`.
- `ab_run.py`'s retry for "settle movement" stays; it is now a safety net, not the only protection.
- Tests: `test_plan_scheduler.py` (scheduler rules and threads), `test_planner_nodes.py` (both real nodes against a fake rospy: off-thread planning, cancel in flight, newer goal, stale path not republished after a cancel, a failing plan), `test_planner_scheduling_source.py` (no callback plans directly, both subscribe to `/nav/cancel`, `on_cancel` resets the leg state, the cancelled check comes before the first publish).
