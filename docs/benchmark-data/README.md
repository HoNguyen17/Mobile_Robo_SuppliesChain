# Benchmark data

Raw output of A/B benchmark sessions, copied out of WSL (`~/ab_runs/`) so the numbers in `README-BENCHMARK.md` can be reproduced from a clone. Copies only: the originals stay in `~/ab_runs/`.

## `20261026_214123/`: parity batch (S-04, n=5 per planner)

Run on commit `3a1ab03` (A\* and D\* Lite with identical obstacle perception), clean git status. Both planners 5/5 succeeded, 0 invalid runs.

| File | What it is |
|---|---|
| `astar_runs.csv`, `dstar_runs.csv` | 41-column per-run telemetry (columns in `README-BENCHMARK.md` §6) |
| `meta_astar.json`, `meta_dstar.json` | git commit, file hashes, rosparams, CLI args |
| `report.md` | the report as `ab_batch.py` wrote it on 2026-10-26 (no permutation test yet) |
| `report_with_permutation.md` | the same CSVs through the current `ab_report.py`, which adds the exact permutation test table |
| `handoff.txt` | the consolidated bundle with personal paths and the host name replaced by `<repo>`, `/home/<user>`, `/mnt/c/Users/<user>` and `<host>` |

Not copied: `bringup_*.log`, `batch.log`, `report.html`.

## Regenerate the report and the p-values

From the repo root, inside WSL (stdlib only, no ROS needed):

```bash
python3 ros1/turtlebot_control/scripts/ab_report.py --out /tmp/ab_report \
    docs/benchmark-data/20261026_214123/astar_runs.csv \
    docs/benchmark-data/20261026_214123/dstar_runs.csv
```

Then read `/tmp/ab_report/report.md`. The permutation test is `permutation_test_means()` in `ros1/turtlebot_control/scripts/ab_report.py`.
