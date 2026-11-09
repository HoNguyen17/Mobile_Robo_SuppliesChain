# ADR-020: The Unattended Matrix Runner, Its Validity Rules, and Draft Thresholds

**Status:** Accepted for the runner (2026-11-09). **The thresholds in section "Draft thresholds" are a DRAFT and are not approved.** · milestone 3 · follows [ADR-019](ADR-019-dynamic-scenario-spec.md)

## Context
The milestone 3 evidence is a matrix: scenarios x planners x N = 20 valid runs, about 20 minutes per cell and 3 to 4 hours for the five core scenarios. It has to run without the person at the keyboard, survive the usual failures (a run that is invalid, a stack that does not start, a PC that goes to sleep), and end with a report. `ab_batch.py` already starts and supervises one planner stack and one `ab_run.py`; it does not know about several scenarios, retries or resuming.

## Decisions
1. **`ros1/turtlebot_control/scripts/ab_matrix.py` runs one `ab_batch.py` per attempt.** A cell is one (scenario, planner). The stack start, the checks of nodes and parameters, the watchdog and the cleanup stay in `ab_batch.py` (new options `--seed-base`, `--first-index`, `--no-report`). One attempt costs one stack start (about 2 minutes). The launcher is `start_matrix.bat` (Unity in Play first); the default is `S-00c S-04 S-06 S-07 S-08` x `astar dstar` x N = 20, seed base 1000.
2. **Order.** The first planner rotates from scenario to scenario (A\*, D\* / D\*, A\* / ...), so neither planner is always measured first. Run k of every cell uses seed `1000 + k` (ADR-019), so run k of both planners meets the same worker timing.
3. **Validity and top-ups.** A run is invalid for the reasons `ab_run.py` already knows (`settle movement`, `pose off start`, `ack timeout`, `ack not ok`, `publisher not connected`, `planner node mismatch`, `no Planned line`) plus `no worker seen` (a seeded run whose workers never appeared on `/sim/npc_poses`: a Unity without the ADR-019 code). Invalid runs are not counted. After each attempt the runner counts valid runs and, if fewer than N, starts another attempt for the missing number with fresh run indices (and therefore fresh seeds), up to `--max-attempts` (default 3). A cell that still lacks runs is reported as incomplete and the exit code is 2. Every invalid run, and every attempt that produced no runs, is written to `invalid_runs.csv` with its reason, run index, seed and session folder.
4. **Per-cell timeout.** 2 minutes of stack start, 2 minutes of slack and, per run, the scenario's `leg_timeout_s` + 7 s settle + 15 s. When it expires the runner sends SIGINT to the `ab_batch.py` process group (it cleans up its stack), then SIGTERM after 90 s, then SIGKILL after 15 s more. If a ROS master still answers afterwards the matrix aborts (a leftover stack would corrupt the next cell). Two failed stack starts in a row abort the matrix ("is Unity in Play mode?"). No `killall` or `pkill` anywhere (CLAUDE.md); a test checks the source.
5. **Resume.** `matrix_config.json` and `matrix_state.json` are written in the matrix folder; the CSVs of the attempts are the truth for what is done. `--resume DIR` skips complete cells and continues the rest (attempts already used count against `--max-attempts`; `--max-attempts N` raises it). Ctrl-C (or `--stop --resume DIR`) stops the current attempt cleanly and still writes the report.
6. **Preflight checklist, printed at every start.** FAIL stops the matrix unless `--force`; WARN is shown; INFO lists the manual steps.
   - FAIL: not in WSL; stray ROS or benchmark processes; a ROS master already answering; less than 1 GB free in WSL or 2 GB on C:; running on battery; the PC sleeps on AC sooner than the matrix needs; stale or invalid `scenarios.yaml` / `scenarios.json`.
   - WARN: little disk space; sleep timeout on AC longer than 0 but long enough; lid close does something; PowerShell not reachable so the power state is unknown; a scenario with `layout_checked: false`.
   - INFO: estimated duration and finish time, power plan name, "Unity is in Play and its Console has no red errors", "the C# was compiled and the EditMode tests passed once", "do not touch Unity; never use `start_ros.bat`, `stop_ros.bat` or Unity menu 3".
   - The Windows checks call `powershell.exe` from WSL and parse the numbers, not the words, so they work in any Windows language. The runner **checks** the sleep setting; it does not prevent sleep.
7. **Output** in `~/ab_matrix/<timestamp>/` (`dry_` prefix for `--dry`): `matrix_report.md` (the automatic final report), `matrix.log`, `invalid_runs.csv`, `matrix_config.json` (settings, git commit, hashes of `scenarios.yaml` and `scenarios.json`), `matrix_state.json`, `cells/<scenario>_<planner>/attempt<k>/` (the `ab_batch` sessions with their CSVs, logs and `meta_*.json`), `data/<scenario>/<planner>_runs.csv` (all attempts merged), `report/<scenario>/` (`ab_report.py` with its permutation tests). `~/ab_matrix/latest_matrix.txt` names the newest folder.
8. **The report** has one row per cell (valid/N, invalid, success, runs with a collision, worker collisions, worker gap minimum and median, time to goal median and range, early replans, timeouts), the draft-rule flags, the invalid runs by reason, every attempt, and the protocol. Banners say when cells are incomplete, the matrix stopped early, layouts are unchecked, or it was a dry run.

## Draft thresholds (NOT approved; for the user to approve after the baseline)
These rules only decide which scenarios get improvement work. They are written down now so that the baseline is read against rules fixed in advance; they will be recalibrated against the baseline data, and no number below is final.

| Rule | Draft value | Applies to | Why |
|---|---|---|---|
| Success rate | ≥ 95 % (19 of 20) | S-04, S-06, S-07, S-08 | `docs/prd.md` asks 85 % for S-04 only |
| Worker collisions (`collisions_npc`) | 0 in N runs | S-04, S-06, S-07, S-08 | `docs/prd.md` allows 1 in 20, which hides a true rate of about 5 %; 0 of 20 bounds it below 14 % |
| Worker gap (`npc_min_gap_m`) | ≥ 0.15 m in every run; median ≥ 0.30 m | the same | the planner's own margin at 0.35 m inflation is about 0.09 m; S-01 uses 0.15 m |
| Time to goal | median ≤ 1.3 x the S-00c median of the same planner | S-04, S-06 | not a detour limit for S-07b |
| No deadlock | no timeout | the same | the CSV cannot tell a deadlock from a slow run: a timeout is a suspect, to be looked at |
| Stress scenarios (S-07b, S-09, S-10, S-11) | reported, not gated | | reaction-limited cases may be physically unavoidable |

Until the thresholds are approved, the minimum in `docs/prd.md` (S-04: success ≥ 85 %, ≤ 1 collision in 20) is the only gate. `ab_matrix.py` flags cells with these draft rules (`DRAFT_*` constants); approving or changing them is a change of those constants plus a row in the threshold change log of `docs/test-plan.md`.

## Consequences
- The three-to-four-hour matrix needs no one, but it needs the PC awake, Unity in Play and nobody else using the WSL distro. The preflight checks what it can; it cannot check that Unity was recompiled (the `no worker seen` rule catches that after the first run).
- Top-ups cost a stack start each, and a run that is invalid in one planner's cell and not in the other's gets a different seed, so a few runs are not paired.
- Planner time is not comparable with earlier batches: the planners now plan off the callbacks ([ADR-016](ADR-016-planner-scheduling.md)).
- Tests: `test_ab_matrix.py` (cell order, top-ups, timeout, abort rules, resume, dry mode, preflight with a fake shell, report, no `killall` in the source and in `start_matrix.bat`), `test_seeded_batch.py`. The real preflight was run once on the author's machine; the runner itself has not been run against Unity.
