#!/usr/bin/env python3
"""A/B Benchmark Reporting Tool for A* vs D* Lite.

Generates side-by-side Markdown and self-contained HTML comparison reports
from benchmark CSV runs. Pure Python, stdlib only, zero ROS dependencies.
Python 3.8 compatible.
"""

import argparse
import csv
import datetime
import html
import math
import os
import statistics
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

FOOTNOTE_TIMER = (
    "ms is the timer inside the Planned log line. A* publishes the costmap and the path "
    "before reading it. D* publishes the path first and reads path_ready right after it, "
    "before the costmap is published. Compare ms; path_ready is not like for like."
)

FOOTNOTE_REUSE = (
    "D* reuses its search state when the goal repeats, so only rebuilt=True first plans are cold."
)

FOOTNOTE_MOTION = (
    "max_xte_m and mean_xte_m: distance from each recorded pose to the polyline of the "
    "/planned_path that was active at that time. Each plan starts at the robot pose, so "
    "XTE falls toward 0 at every replan and is only comparable between planners with a "
    "similar path_switches; heading_osc, stops and path_ratio are the veering evidence. "
    "heading_osc: sign changes of angular.z with |w| > 0.2 rad/s during the leg. "
    "path_switches: number of /planned_path messages during the leg (counts the first path too). "
    "path_ratio is driven_m / displacement_m (>= 1.0), where displacement_m is the straight-line "
    "distance from the first recorded pose to the last recorded pose of the leg. "
    "straight_m is the distance from start to nominal goal; planners snap blocked or unreachable "
    "goals to the nearest collision-free cell, so displacement_m reflects actual endpoint distance."
)

FOOTNOTE_RATIO = (
    "Ratio (A*/D*) above 1 means D* has the lower value for time and count metrics."
)

FOOTNOTE_SPREAD = (
    "n per planner and [min, max] ranges: with n=5 treat differences inside the run-to-run spread as not established."
)

FOOTNOTE_PLANNER_TIME = (
    "Sum of the ms timer of all Planned lines in the leg; "
    "only plans started by the dynamic-obstacle check run inside the /scan callback [verified in code: "
    "astar_planner.py line 181 'self.plan_for_goal(self.latest_goal)', "
    "dstar_planner.py line 234 'self.plan_for_goal(self.latest_goal)'], "
    "first plans, goal changes and lookahead replan requests run in other callbacks [verified in code: "
    "astar_planner.py line 228 'self.plan_for_goal(msg)', "
    "astar_planner.py line 233 'self.plan_for_goal(self.latest_goal)', "
    "dstar_planner.py line 294 'self.plan_for_goal(msg)', "
    "dstar_planner.py line 299 'self.plan_for_goal(self.latest_goal)']; "
    "a ROS Python node runs each subscriber callback in its own thread [verified in code: "
    "path_tracker.py line 3-4 'ROS callbacks run on their own threads'], "
    "so only the scan-triggered part delays scan processing."
)


def _to_float(val: Any) -> Optional[float]:
    if val is None:
        return None
    s = str(val).strip()
    if not s:
        return None
    try:
        f = float(s)
        if math.isnan(f) or math.isinf(f):
            return None
        return f
    except (ValueError, TypeError):
        return None


def _to_int(val: Any) -> Optional[int]:
    if val is None:
        return None
    s = str(val).strip()
    if not s:
        return None
    try:
        return int(float(s))
    except (ValueError, TypeError):
        return None


def _to_bool(val: Any) -> Optional[bool]:
    if val is None:
        return None
    s = str(val).strip().lower()
    if s in ("true", "1", "yes"):
        return True
    if s in ("false", "0", "no"):
        return False
    return None


def _calc_stats(values: Sequence[float]) -> Optional[Dict[str, Any]]:
    clean = [float(v) for v in values if v is not None and not math.isnan(v) and not math.isinf(v)]
    if not clean:
        return None
    return {
        "median": float(statistics.median(clean)),
        "min": float(min(clean)),
        "max": float(max(clean)),
        "n": len(clean),
    }


def _calc_ratio(a_med: Optional[float], d_med: Optional[float]) -> str:
    if a_med is not None and d_med is not None and d_med != 0.0:
        ratio = a_med / d_med
        return "{:.2f}x".format(ratio)
    return "-"


def load_runs_from_csv(csv_path: str) -> List[Dict[str, Any]]:
    """Load run records from a CSV file into a list of row dictionaries."""
    runs = []  # type: List[Dict[str, Any]]
    if not os.path.exists(csv_path):
        return runs
    with open(csv_path, "r", encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row:
                runs.append(dict(row))
    return runs


def compute_planner_metrics(runs: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Compute benchmark metrics for a single planner from its run rows.

    Invalid runs are counted with reasons and excluded from all statistical metrics.
    """
    total_runs = len(runs)
    invalid_runs = []  # type: List[Dict[str, Any]]
    valid_runs = []  # type: List[Dict[str, Any]]

    for r in runs:
        outcome = str(r.get("outcome", "")).strip().lower()
        if outcome == "invalid":
            invalid_runs.append(r)
        else:
            valid_runs.append(r)

    # Count invalid reasons
    invalid_reasons = {}  # type: Dict[str, int]
    for r in invalid_runs:
        reason = str(r.get("invalid_reason", "")).strip() or "unspecified"
        invalid_reasons[reason] = invalid_reasons.get(reason, 0) + 1

    # Count valid outcomes
    outcomes = {"succeeded": 0, "aborted": 0, "timeout": 0}  # type: Dict[str, int]
    for r in valid_runs:
        out = str(r.get("outcome", "")).strip().lower()
        if out in outcomes:
            outcomes[out] += 1
        else:
            outcomes[out] = outcomes.get(out, 0) + 1

    valid_n = len(valid_runs)
    is_indicative = valid_n < 5

    # Determine planner name
    planner_name = "unknown"
    if runs:
        planner_name = str(runs[0].get("planner") or runs[0].get("label") or "unknown").strip().lower()
    is_dstar = "dstar" in planner_name

    # Succeeded-only runs for time, driven distance, path ratio, and planner time/share
    succeeded_runs = [r for r in valid_runs if str(r.get("outcome", "")).strip().lower() == "succeeded"]
    time_vals = [_to_float(r.get("sim_time_to_goal_s")) for r in succeeded_runs]
    driven_vals = [_to_float(r.get("driven_m")) for r in succeeded_runs]
    ratio_vals = [_to_float(r.get("path_ratio")) for r in succeeded_runs]
    plan_time_s_vals = [
        (_to_float(r.get("plan_ms_total")) / 1000.0)
        for r in succeeded_runs
        if _to_float(r.get("plan_ms_total")) is not None
    ]
    plan_share_pct_vals = [
        _to_float(r.get("plan_share_pct"))
        for r in succeeded_runs
        if _to_float(r.get("plan_share_pct")) is not None
    ]

    # All valid runs for stops, clearance, and motion-quality metrics
    stops_vals = [_to_float(r.get("stops")) for r in valid_runs]
    clear_vals = [_to_float(r.get("min_clearance_m")) for r in valid_runs]

    max_xte_vals = [_to_float(r.get("max_xte_m")) for r in valid_runs]
    mean_xte_vals = [_to_float(r.get("mean_xte_m")) for r in valid_runs]
    heading_osc_vals = [_to_float(r.get("heading_osc")) for r in valid_runs]
    path_switches_vals = [_to_float(r.get("path_switches")) for r in valid_runs]

    coll_vals = [_to_int(r.get("collisions")) for r in valid_runs]
    valid_coll = [c for c in coll_vals if c is not None]
    total_collisions = sum(valid_coll) if valid_coll else 0
    runs_with_collisions = sum(1 for c in valid_coll if c > 0)

    # First plan ms
    first_plan_all_vals = [_to_float(r.get("first_plan_ms")) for r in valid_runs]
    first_plan_all = _calc_stats([v for v in first_plan_all_vals if v is not None])

    first_plan_rebuilt_true = None  # type: Optional[Dict[str, Any]]
    first_plan_rebuilt_false = None  # type: Optional[Dict[str, Any]]

    if is_dstar:
        true_vals = [
            _to_float(r.get("first_plan_ms"))
            for r in valid_runs
            if _to_bool(r.get("first_plan_rebuilt")) is True
        ]
        false_vals = [
            _to_float(r.get("first_plan_ms"))
            for r in valid_runs
            if _to_bool(r.get("first_plan_rebuilt")) is False
        ]
        first_plan_rebuilt_true = _calc_stats([v for v in true_vals if v is not None])
        first_plan_rebuilt_false = _calc_stats([v for v in false_vals if v is not None])
    else:
        # For A*, all first plans are one group (effectively cold / rebuilt)
        first_plan_rebuilt_true = first_plan_all
        first_plan_rebuilt_false = None

    # Replan ms: median of per-run replan_ms_median, largest p95 and largest max
    replan_med_vals = [_to_float(r.get("replan_ms_median")) for r in valid_runs]
    replan_med_clean = [v for v in replan_med_vals if v is not None]
    replan_p95_vals = [_to_float(r.get("replan_ms_p95")) for r in valid_runs]
    replan_p95_clean = [v for v in replan_p95_vals if v is not None]
    replan_max_vals = [_to_float(r.get("replan_ms_max")) for r in valid_runs]
    replan_max_clean = [v for v in replan_max_vals if v is not None]

    replan_stats = None  # type: Optional[Dict[str, Any]]
    if replan_med_clean:
        replan_stats = {
            "median": float(statistics.median(replan_med_clean)),
            "p95_largest": float(max(replan_p95_clean)) if replan_p95_clean else None,
            "max_largest": float(max(replan_max_clean)) if replan_max_clean else None,
            "n": len(replan_med_clean),
        }

    # Replan path_ready ms (D* only)
    replan_pr_stats = None  # type: Optional[Dict[str, Any]]
    if is_dstar:
        pr_med_vals = [_to_float(r.get("replan_path_ready_median_ms")) for r in valid_runs]
        pr_med_clean = [v for v in pr_med_vals if v is not None]
        pr_p95_vals = [_to_float(r.get("replan_path_ready_p95_ms")) for r in valid_runs]
        pr_p95_clean = [v for v in pr_p95_vals if v is not None]
        if pr_med_clean:
            replan_pr_stats = {
                "median": float(statistics.median(pr_med_clean)),
                "p95_largest": float(max(pr_p95_clean)) if pr_p95_clean else None,
                "n": len(pr_med_clean),
            }

    return {
        "planner": planner_name,
        "is_dstar": is_dstar,
        "total_runs": total_runs,
        "valid_runs": valid_n,
        "succeeded_runs": len(succeeded_runs),
        "invalid_runs": len(invalid_runs),
        "invalid_reasons": invalid_reasons,
        "outcomes": outcomes,
        "is_indicative": is_indicative,
        "sim_time_s": _calc_stats([v for v in time_vals if v is not None]),
        "driven_m": _calc_stats([v for v in driven_vals if v is not None]),
        "path_ratio": _calc_stats([v for v in ratio_vals if v is not None]),
        "stops": _calc_stats([v for v in stops_vals if v is not None]),
        "min_clearance_m": _calc_stats([v for v in clear_vals if v is not None]),
        "max_xte_m": _calc_stats([v for v in max_xte_vals if v is not None]),
        "mean_xte_m": _calc_stats([v for v in mean_xte_vals if v is not None]),
        "heading_osc": _calc_stats([v for v in heading_osc_vals if v is not None]),
        "path_switches": _calc_stats([v for v in path_switches_vals if v is not None]),
        "collisions": {
            "total": total_collisions,
            "runs_with_collisions": runs_with_collisions,
        },
        "first_plan_ms_all": first_plan_all,
        "first_plan_ms_rebuilt_true": first_plan_rebuilt_true,
        "first_plan_ms_rebuilt_false": first_plan_rebuilt_false,
        "replan_ms": replan_stats,
        "replan_path_ready_ms": replan_pr_stats,
        "plan_time_s": _calc_stats([v for v in plan_time_s_vals if v is not None]),
        "plan_share_pct": _calc_stats([v for v in plan_share_pct_vals if v is not None]),
    }


def _format_stat(stat: Optional[Dict[str, Any]], unit: str = "", digits: int = 1) -> str:
    if not stat or stat.get("n", 0) == 0:
        return "-"
    med = stat["median"]
    mi = stat["min"]
    ma = stat["max"]
    n = stat.get("n", 0)
    fmt = "{:." + str(digits) + "f}"
    unit_str = (" " + unit) if unit else ""
    val_str = (fmt + "{unit} [" + fmt + ", " + fmt + "]").format(med, mi, ma, unit=unit_str)
    if n < 5:
        return "{} (n={}, indicative only)".format(val_str, n)
    return "{} (n={})".format(val_str, n)


def _format_success_rate(m: Optional[Dict[str, Any]]) -> str:
    if not m:
        return "-"
    val_n = m.get("valid_runs", 0)
    if val_n == 0:
        return "-"
    succ_n = m.get("succeeded_runs", 0)
    tag = " (n={}, indicative only)".format(val_n) if val_n < 5 else " (n={})".format(val_n)
    return "{}/{}{}".format(succ_n, val_n, tag)


def _format_collisions(m: Optional[Dict[str, Any]]) -> str:
    if not m:
        return "-"
    val_n = m.get("valid_runs", 0)
    if val_n == 0:
        return "-"
    tag = " (n={}, indicative only)".format(val_n) if val_n < 5 else " (n={})".format(val_n)
    return "{total} total ({runs} runs){tag}".format(
        total=m["collisions"]["total"],
        runs=m["collisions"]["runs_with_collisions"],
        tag=tag
    )


def _format_replan(rp: Optional[Dict[str, Any]]) -> str:
    if not rp or rp.get("n", 0) == 0:
        return "-"
    n = rp["n"]
    tag = " (n={}, indicative only)".format(n) if n < 5 else ""
    p95_s = "{:.1f}".format(rp["p95_largest"]) if rp.get("p95_largest") is not None else "-"
    max_s = "{:.1f}".format(rp["max_largest"]) if rp.get("max_largest") is not None else "-"
    return "{:.1f} ms (median of run medians; worst-run p95: {}, worst-run max: {}){}".format(
        rp["median"], p95_s, max_s, tag
    )


def _format_replan_pr(pr: Optional[Dict[str, Any]]) -> str:
    if not pr or pr.get("n", 0) == 0:
        return "-"
    n = pr["n"]
    tag = " (n={}, indicative only)".format(n) if n < 5 else ""
    p95_s = "{:.1f}".format(pr["p95_largest"]) if pr.get("p95_largest") is not None else "-"
    return "{:.1f} ms (median of run medians; worst-run p95: {}){}".format(
        pr["median"], p95_s, tag
    )


def generate_markdown_report(scenarios_data: Dict[str, Dict[str, Any]]) -> str:
    """Generate Markdown comparison report for all scenarios."""
    lines = []  # type: List[str]
    lines.append("# A/B Navigation Benchmark Report")
    lines.append("")
    lines.append("Benchmarking `astar_planner` vs `dstar_planner` side-by-side.")
    lines.append("")

    for scenario in sorted(scenarios_data.keys()):
        pdata = scenarios_data[scenario]
        astar = pdata.get("astar")
        dstar = pdata.get("dstar")

        lines.append("## Scenario: {}".format(scenario))
        lines.append("")

        # Overview / Run counts
        lines.append("### Overview")
        for label, m in [("A*", astar), ("D*", dstar)]:
            if m:
                ind = " (n={}, indicative only)".format(m["valid_runs"]) if m["is_indicative"] else ""
                reasons_str = ""
                if m["invalid_reasons"]:
                    reasons_str = " (reasons: " + ", ".join(
                        "{}: {}".format(k, v) for k, v in sorted(m["invalid_reasons"].items())
                    ) + ")"
                out = m["outcomes"]
                lines.append(
                    "- **{}**{}: {} runs total, {} invalid{}. Outcomes: {} succeeded, {} aborted, {} timeout.".format(
                        label, ind, m["total_runs"], m["invalid_runs"], reasons_str,
                        out.get("succeeded", 0), out.get("aborted", 0), out.get("timeout", 0)
                    )
                )
            else:
                lines.append("- **{}**: No runs recorded.".format(label))
        lines.append("")

        # Metrics comparison table
        lines.append("| Metric | A* | D* | Ratio (A*/D*) |")
        lines.append("| :--- | :--- | :--- | :--- |")

        # Success rate
        a_sr = _format_success_rate(astar)
        d_sr = _format_success_rate(dstar)
        a_r = (astar["succeeded_runs"] / float(astar["valid_runs"])) if (astar and astar["valid_runs"] > 0) else None
        d_r = (dstar["succeeded_runs"] / float(dstar["valid_runs"])) if (dstar and dstar["valid_runs"] > 0) else None
        r_sr = _calc_ratio(a_r, d_r)
        lines.append("| Success Rate | {} | {} | {} |".format(a_sr, d_sr, r_sr))

        # Sim time to goal
        a_time = _format_stat(astar.get("sim_time_s") if astar else None, unit="s", digits=1)
        d_time = _format_stat(dstar.get("sim_time_s") if dstar else None, unit="s", digits=1)
        r_time = _calc_ratio(
            astar["sim_time_s"]["median"] if (astar and astar.get("sim_time_s")) else None,
            dstar["sim_time_s"]["median"] if (dstar and dstar.get("sim_time_s")) else None,
        )
        lines.append("| Sim Time to Goal | {} | {} | {} |".format(a_time, d_time, r_time))

        # Driven distance
        a_drv = _format_stat(astar.get("driven_m") if astar else None, unit="m", digits=2)
        d_drv = _format_stat(dstar.get("driven_m") if dstar else None, unit="m", digits=2)
        r_drv = _calc_ratio(
            astar["driven_m"]["median"] if (astar and astar.get("driven_m")) else None,
            dstar["driven_m"]["median"] if (dstar and dstar.get("driven_m")) else None,
        )
        lines.append("| Driven Distance | {} | {} | {} |".format(a_drv, d_drv, r_drv))

        # Path ratio
        a_pr = _format_stat(astar.get("path_ratio") if astar else None, digits=2)
        d_pr = _format_stat(dstar.get("path_ratio") if dstar else None, digits=2)
        r_pr = _calc_ratio(
            astar["path_ratio"]["median"] if (astar and astar.get("path_ratio")) else None,
            dstar["path_ratio"]["median"] if (dstar and dstar.get("path_ratio")) else None,
        )
        lines.append("| Path Ratio | {} | {} | {} |".format(a_pr, d_pr, r_pr))

        # Stops
        a_stp = _format_stat(astar.get("stops") if astar else None, digits=1)
        d_stp = _format_stat(dstar.get("stops") if dstar else None, digits=1)
        r_stp = _calc_ratio(
            astar["stops"]["median"] if (astar and astar.get("stops")) else None,
            dstar["stops"]["median"] if (dstar and dstar.get("stops")) else None,
        )
        lines.append("| Stops | {} | {} | {} |".format(a_stp, d_stp, r_stp))

        # Min clearance
        a_clr = _format_stat(astar.get("min_clearance_m") if astar else None, unit="m", digits=2)
        d_clr = _format_stat(dstar.get("min_clearance_m") if dstar else None, unit="m", digits=2)
        r_clr = _calc_ratio(
            astar["min_clearance_m"]["median"] if (astar and astar.get("min_clearance_m")) else None,
            dstar["min_clearance_m"]["median"] if (dstar and dstar.get("min_clearance_m")) else None,
        )
        lines.append("| Min Clearance | {} | {} | {} |".format(a_clr, d_clr, r_clr))

        # Collisions
        a_col = _format_collisions(astar)
        d_col = _format_collisions(dstar)
        lines.append("| Collisions | {} | {} | - |".format(a_col, d_col))

        # Motion quality metrics (TASK D)
        a_mxte = _format_stat(astar.get("max_xte_m") if astar else None, unit="m", digits=3)
        d_mxte = _format_stat(dstar.get("max_xte_m") if dstar else None, unit="m", digits=3)
        r_mxte = _calc_ratio(
            astar["max_xte_m"]["median"] if (astar and astar.get("max_xte_m")) else None,
            dstar["max_xte_m"]["median"] if (dstar and dstar.get("max_xte_m")) else None,
        )
        lines.append("| Max XTE | {} | {} | {} |".format(a_mxte, d_mxte, r_mxte))

        a_xte = _format_stat(astar.get("mean_xte_m") if astar else None, unit="m", digits=3)
        d_xte = _format_stat(dstar.get("mean_xte_m") if dstar else None, unit="m", digits=3)
        r_xte = _calc_ratio(
            astar["mean_xte_m"]["median"] if (astar and astar.get("mean_xte_m")) else None,
            dstar["mean_xte_m"]["median"] if (dstar and dstar.get("mean_xte_m")) else None,
        )
        lines.append("| Mean XTE | {} | {} | {} |".format(a_xte, d_xte, r_xte))

        a_osc = _format_stat(astar.get("heading_osc") if astar else None, digits=1)
        d_osc = _format_stat(dstar.get("heading_osc") if dstar else None, digits=1)
        r_osc = _calc_ratio(
            astar["heading_osc"]["median"] if (astar and astar.get("heading_osc")) else None,
            dstar["heading_osc"]["median"] if (dstar and dstar.get("heading_osc")) else None,
        )
        lines.append("| Heading Oscillations | {} | {} | {} |".format(a_osc, d_osc, r_osc))

        a_ps = _format_stat(astar.get("path_switches") if astar else None, digits=1)
        d_ps = _format_stat(dstar.get("path_switches") if dstar else None, digits=1)
        r_ps = _calc_ratio(
            astar["path_switches"]["median"] if (astar and astar.get("path_switches")) else None,
            dstar["path_switches"]["median"] if (dstar and dstar.get("path_switches")) else None,
        )
        lines.append("| Path Switches | {} | {} | {} |".format(a_ps, d_ps, r_ps))

        # First plan ms (cold)
        a_fp_cold = _format_stat(astar.get("first_plan_ms_all") if astar else None, unit="ms", digits=1)
        d_fp_cold = _format_stat(dstar.get("first_plan_ms_rebuilt_true") if dstar else None, unit="ms", digits=1)
        r_fp_cold = _calc_ratio(
            astar["first_plan_ms_all"]["median"] if (astar and astar.get("first_plan_ms_all")) else None,
            dstar["first_plan_ms_rebuilt_true"]["median"] if (dstar and dstar.get("first_plan_ms_rebuilt_true")) else None,
        )
        lines.append("| First Plan ms (cold) | {} | {} | {} |".format(a_fp_cold, d_fp_cold, r_fp_cold))

        # First plan ms (warm)
        d_fp_warm = _format_stat(dstar.get("first_plan_ms_rebuilt_false") if dstar else None, unit="ms", digits=1)
        lines.append("| First Plan ms (warm) | - | {} | - |".format(d_fp_warm))

        # Replan ms
        a_rp_str = _format_replan(astar.get("replan_ms") if astar else None)
        d_rp_str = _format_replan(dstar.get("replan_ms") if dstar else None)
        r_rp = _calc_ratio(
            astar["replan_ms"]["median"] if (astar and astar.get("replan_ms")) else None,
            dstar["replan_ms"]["median"] if (dstar and dstar.get("replan_ms")) else None,
        )
        lines.append("| Replan ms (median) | {} | {} | {} |".format(a_rp_str, d_rp_str, r_rp))

        # Replan path_ready ms (D* only, secondary row)
        d_pr_str = _format_replan_pr(dstar.get("replan_path_ready_ms") if dstar else None)
        lines.append("| ↳ Replan Path Ready ms (D* only) | - | {} | - |".format(d_pr_str))

        # Planner time per run (s)
        a_pts = _format_stat(astar.get("plan_time_s") if astar else None, unit="s", digits=2)
        d_pts = _format_stat(dstar.get("plan_time_s") if dstar else None, unit="s", digits=2)
        r_pts = _calc_ratio(
            astar["plan_time_s"]["median"] if (astar and astar.get("plan_time_s")) else None,
            dstar["plan_time_s"]["median"] if (dstar and dstar.get("plan_time_s")) else None,
        )
        lines.append("| Planner time per run (s) | {} | {} | {} |".format(a_pts, d_pts, r_pts))

        # Planner time share of run (%)
        a_psp = _format_stat(astar.get("plan_share_pct") if astar else None, unit="%", digits=1)
        d_psp = _format_stat(dstar.get("plan_share_pct") if dstar else None, unit="%", digits=1)
        r_psp = _calc_ratio(
            astar["plan_share_pct"]["median"] if (astar and astar.get("plan_share_pct")) else None,
            dstar["plan_share_pct"]["median"] if (dstar and dstar.get("plan_share_pct")) else None,
        )
        lines.append("| Planner time share of run (%) | {} | {} | {} |".format(a_psp, d_psp, r_psp))
        lines.append("")

    lines.append("---")
    lines.append("### Footnotes")
    lines.append("1. " + FOOTNOTE_TIMER)
    lines.append("2. " + FOOTNOTE_REUSE)
    lines.append("3. " + FOOTNOTE_MOTION)
    lines.append("4. " + FOOTNOTE_RATIO)
    lines.append("5. " + FOOTNOTE_SPREAD)
    lines.append("6. " + FOOTNOTE_PLANNER_TIME)
    lines.append("")

    return "\n".join(lines)


def _render_svg_bar_chart(title: str, unit: str, series: List[Dict[str, Any]]) -> str:
    """Render inline SVG bar chart with min-max whiskers. Zero external links."""
    width = 380
    height = 230
    margin_top = 40
    margin_bottom = 40
    margin_left = 60
    margin_right = 30

    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    plot_bottom = height - margin_bottom

    # Find maximum value for scaling
    max_val = 1.0
    for s in series:
        if s.get("max") is not None and s["max"] > max_val:
            max_val = float(s["max"])
        elif s.get("median") is not None and s["median"] > max_val:
            max_val = float(s["median"])
    max_y = max_val * 1.25

    def scale_y(val: float) -> float:
        return plot_bottom - (val / max_y) * plot_h

    svg_parts = []
    svg_parts.append('<svg viewBox="0 0 {} {}" width="{}" height="{}">'.format(width, height, width, height))

    # Grid lines and y-axis ticks
    for i in range(4):
        tick_val = (max_y / 3.0) * i
        y_pos = scale_y(tick_val)
        svg_parts.append('<line x1="{}" y1="{:.1f}" x2="{}" y2="{:.1f}" stroke="#e5e7eb" stroke-dasharray="3,3" />'.format(
            margin_left, y_pos, width - margin_right, y_pos
        ))
        svg_parts.append('<text x="{}" y="{:.1f}" text-anchor="end" font-size="10" fill="#6b7280">{:.0f}</text>'.format(
            margin_left - 8, y_pos + 4, tick_val
        ))

    # Title
    svg_parts.append('<text x="{}" y="24" text-anchor="middle" font-size="13" font-weight="bold" fill="#111827">{}</text>'.format(
        width // 2, html.escape(title)
    ))

    # Baseline
    svg_parts.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#9ca3af" stroke-width="1.5" />'.format(
        margin_left, plot_bottom, width - margin_right, plot_bottom
    ))

    # Render bars and whiskers
    n_series = len(series)
    bar_width = 46
    for idx, s in enumerate(series):
        x_center = margin_left + (idx + 1) * (plot_w / (n_series + 1))
        lbl = html.escape(str(s.get("label", "")))
        color = s.get("color", "#3b82f6")

        if s.get("median") is None:
            svg_parts.append('<text x="{:.1f}" y="{:.1f}" text-anchor="middle" font-size="11" fill="#9ca3af">N/A</text>'.format(
                x_center, plot_bottom - 20
            ))
            svg_parts.append('<text x="{:.1f}" y="{}" text-anchor="middle" font-size="11" font-weight="500" fill="#4b5563">{}</text>'.format(
                x_center, height - 15, lbl
            ))
            continue

        med = float(s["median"])
        y_med = scale_y(med)
        bar_x = x_center - (bar_width / 2.0)
        bar_h = max(1.0, plot_bottom - y_med)

        # Bar rect
        svg_parts.append('<rect x="{:.1f}" y="{:.1f}" width="{}" height="{:.1f}" fill="{}" rx="4" />'.format(
            bar_x, y_med, bar_width, bar_h, color
        ))

        # Whisker
        top_y = y_med
        if s.get("min") is not None and s.get("max") is not None:
            y_min = scale_y(float(s["min"]))
            y_max = scale_y(float(s["max"]))
            top_y = y_max
            # Vertical stem
            svg_parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" stroke="#1f2937" stroke-width="2" />'.format(
                x_center, y_max, x_center, y_min
            ))
            # Caps
            svg_parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" stroke="#1f2937" stroke-width="2" />'.format(
                x_center - 7, y_max, x_center + 7, y_max
            ))
            svg_parts.append('<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" stroke="#1f2937" stroke-width="2" />'.format(
                x_center - 7, y_min, x_center + 7, y_min
            ))

        # Value label
        val_lbl = "{:.1f}{}".format(med, unit)
        lbl_y = max(35.0, top_y - 6.0)
        svg_parts.append('<text x="{:.1f}" y="{:.1f}" text-anchor="middle" font-size="11" font-weight="600" fill="#111827">{}</text>'.format(
            x_center, lbl_y, val_lbl
        ))

        # X-axis label
        svg_parts.append('<text x="{:.1f}" y="{}" text-anchor="middle" font-size="11" font-weight="500" fill="#4b5563">{}</text>'.format(
            x_center, height - 15, lbl
        ))

    svg_parts.append('</svg>')
    return "".join(svg_parts)


def generate_html_report(scenarios_data: Dict[str, Dict[str, Any]]) -> str:
    """Generate self-contained HTML comparison report with inline CSS and SVG bar charts.

    Contains zero external resources (no http:// or https:// links, no xmlns on svg tags).
    """
    html_parts = []
    html_parts.append('<!DOCTYPE html>')
    html_parts.append('<html lang="en">')
    html_parts.append('<head>')
    html_parts.append('<meta charset="utf-8">')
    html_parts.append('<title>A/B Navigation Benchmark Report</title>')
    html_parts.append('<style>')
    html_parts.append("""
body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    line-height: 1.5;
    color: #1f2937;
    background-color: #f9fafb;
    margin: 0;
    padding: 24px;
}
.container {
    max-width: 1200px;
    margin: 0 auto;
}
header {
    margin-bottom: 32px;
    border-bottom: 1px solid #e5e7eb;
    padding-bottom: 16px;
}
h1 {
    font-size: 28px;
    margin: 0 0 8px 0;
    color: #111827;
}
.subtitle {
    color: #4b5563;
    font-size: 14px;
}
.scenario-card {
    background: #ffffff;
    border: 1px solid #e5e7eb;
    border-radius: 8px;
    padding: 24px;
    margin-bottom: 32px;
    box-shadow: 0 1px 3px rgba(0,0,0,0.05);
}
h2 {
    font-size: 20px;
    margin: 0 0 16px 0;
    color: #1f2937;
}
.overview-box {
    background: #f3f4f6;
    border-radius: 6px;
    padding: 16px;
    margin-bottom: 24px;
}
.overview-item {
    margin-bottom: 8px;
    font-size: 14px;
}
.overview-item:last-child {
    margin-bottom: 0;
}
.charts-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(360px, 1fr));
    gap: 20px;
    margin-bottom: 24px;
}
.chart-card {
    background: #f9fafb;
    border: 1px solid #e5e7eb;
    border-radius: 6px;
    padding: 12px;
    display: flex;
    justify-content: center;
}
table {
    width: 100%;
    border-collapse: collapse;
    margin-top: 16px;
    font-size: 14px;
}
th, td {
    padding: 10px 14px;
    text-align: left;
    border-bottom: 1px solid #e5e7eb;
}
th {
    background-color: #f9fafb;
    font-weight: 600;
    color: #374151;
}
tr:hover {
    background-color: #f9fafb;
}
.ratio-col {
    font-weight: 600;
    color: #2563eb;
}
.footnotes-box {
    margin-top: 32px;
    padding-top: 16px;
    border-top: 1px solid #e5e7eb;
    font-size: 13px;
    color: #4b5563;
}
.footnotes-box ol {
    margin: 8px 0 0 0;
    padding-left: 20px;
}
.footnotes-box li {
    margin-bottom: 6px;
}
""")
    html_parts.append('</style>')
    html_parts.append('</head>')
    html_parts.append('<body>')
    html_parts.append('<div class="container">')

    html_parts.append('<header>')
    html_parts.append('<h1>A/B Navigation Benchmark Report</h1>')
    html_parts.append('<div class="subtitle">Benchmarking <code>astar_planner</code> vs <code>dstar_planner</code> side-by-side.</div>')
    html_parts.append('</header>')

    for scenario in sorted(scenarios_data.keys()):
        pdata = scenarios_data[scenario]
        astar = pdata.get("astar")
        dstar = pdata.get("dstar")

        html_parts.append('<div class="scenario-card">')
        html_parts.append('<h2>Scenario: {}</h2>'.format(html.escape(scenario)))

        # Overview Box
        html_parts.append('<div class="overview-box">')
        for label, m in [("A*", astar), ("D*", dstar)]:
            if m:
                ind = " (n={}, indicative only)".format(m["valid_runs"]) if m["is_indicative"] else ""
                reasons_str = ""
                if m["invalid_reasons"]:
                    reasons_str = " (reasons: " + ", ".join(
                        "{}: {}".format(html.escape(k), v) for k, v in sorted(m["invalid_reasons"].items())
                    ) + ")"
                out = m["outcomes"]
                html_parts.append(
                    '<div class="overview-item"><strong>{}</strong>{}: {} runs total, {} invalid{}. Outcomes: {} succeeded, {} aborted, {} timeout.</div>'.format(
                        label, ind, m["total_runs"], m["invalid_runs"], reasons_str,
                        out.get("succeeded", 0), out.get("aborted", 0), out.get("timeout", 0)
                    )
                )
            else:
                html_parts.append('<div class="overview-item"><strong>{}</strong>: No runs recorded.</div>'.format(label))
        html_parts.append('</div>')

        # Charts Grid
        html_parts.append('<div class="charts-grid">')

        # Chart 1: Sim Time to Goal
        c1_series = [
            {
                "label": "A*",
                "color": "#3b82f6",
                "median": astar["sim_time_s"]["median"] if (astar and astar.get("sim_time_s")) else None,
                "min": astar["sim_time_s"]["min"] if (astar and astar.get("sim_time_s")) else None,
                "max": astar["sim_time_s"]["max"] if (astar and astar.get("sim_time_s")) else None,
            },
            {
                "label": "D*",
                "color": "#10b981",
                "median": dstar["sim_time_s"]["median"] if (dstar and dstar.get("sim_time_s")) else None,
                "min": dstar["sim_time_s"]["min"] if (dstar and dstar.get("sim_time_s")) else None,
                "max": dstar["sim_time_s"]["max"] if (dstar and dstar.get("sim_time_s")) else None,
            },
        ]
        html_parts.append('<div class="chart-card">{}</div>'.format(
            _render_svg_bar_chart("Sim Time to Goal", "s", c1_series)
        ))

        # Chart 2: First-plan ms (cold)
        c2_series = [
            {
                "label": "A*",
                "color": "#3b82f6",
                "median": astar["first_plan_ms_all"]["median"] if (astar and astar.get("first_plan_ms_all")) else None,
                "min": astar["first_plan_ms_all"]["min"] if (astar and astar.get("first_plan_ms_all")) else None,
                "max": astar["first_plan_ms_all"]["max"] if (astar and astar.get("first_plan_ms_all")) else None,
            },
            {
                "label": "D*",
                "color": "#10b981",
                "median": dstar["first_plan_ms_rebuilt_true"]["median"] if (dstar and dstar.get("first_plan_ms_rebuilt_true")) else None,
                "min": dstar["first_plan_ms_rebuilt_true"]["min"] if (dstar and dstar.get("first_plan_ms_rebuilt_true")) else None,
                "max": dstar["first_plan_ms_rebuilt_true"]["max"] if (dstar and dstar.get("first_plan_ms_rebuilt_true")) else None,
            },
        ]
        html_parts.append('<div class="chart-card">{}</div>'.format(
            _render_svg_bar_chart("First-plan ms (cold)", "ms", c2_series)
        ))

        # Chart 3: Replan ms
        c3_series = [
            {
                "label": "A*",
                "color": "#3b82f6",
                "median": astar["replan_ms"]["median"] if (astar and astar.get("replan_ms")) else None,
                "min": astar["replan_ms"]["median"] if (astar and astar.get("replan_ms")) else None,
                "max": astar["replan_ms"]["max_largest"] if (astar and astar.get("replan_ms")) else None,
            },
            {
                "label": "D*",
                "color": "#10b981",
                "median": dstar["replan_ms"]["median"] if (dstar and dstar.get("replan_ms")) else None,
                "min": dstar["replan_ms"]["median"] if (dstar and dstar.get("replan_ms")) else None,
                "max": dstar["replan_ms"]["max_largest"] if (dstar and dstar.get("replan_ms")) else None,
            },
        ]
        html_parts.append('<div class="chart-card">{}</div>'.format(
            _render_svg_bar_chart("Replan ms", "ms", c3_series)
        ))

        html_parts.append('</div>')

        # Comparison Table
        html_parts.append('<table>')
        html_parts.append('<thead>')
        html_parts.append('<tr><th>Metric</th><th>A*</th><th>D*</th><th>Ratio (A*/D*)</th></tr>')
        html_parts.append('</thead>')
        html_parts.append('<tbody>')

        # Success rate
        a_sr = _format_success_rate(astar)
        d_sr = _format_success_rate(dstar)
        a_r = (astar["succeeded_runs"] / float(astar["valid_runs"])) if (astar and astar["valid_runs"] > 0) else None
        d_r = (dstar["succeeded_runs"] / float(dstar["valid_runs"])) if (dstar and dstar["valid_runs"] > 0) else None
        r_sr = _calc_ratio(a_r, d_r)
        html_parts.append('<tr><td>Success Rate</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(
            html.escape(a_sr), html.escape(d_sr), r_sr
        ))

        # Sim time
        a_time = html.escape(_format_stat(astar.get("sim_time_s") if astar else None, unit="s", digits=1))
        d_time = html.escape(_format_stat(dstar.get("sim_time_s") if dstar else None, unit="s", digits=1))
        r_time = _calc_ratio(
            astar["sim_time_s"]["median"] if (astar and astar.get("sim_time_s")) else None,
            dstar["sim_time_s"]["median"] if (dstar and dstar.get("sim_time_s")) else None,
        )
        html_parts.append('<tr><td>Sim Time to Goal</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_time, d_time, r_time))

        # Driven distance
        a_drv = html.escape(_format_stat(astar.get("driven_m") if astar else None, unit="m", digits=2))
        d_drv = html.escape(_format_stat(dstar.get("driven_m") if dstar else None, unit="m", digits=2))
        r_drv = _calc_ratio(
            astar["driven_m"]["median"] if (astar and astar.get("driven_m")) else None,
            dstar["driven_m"]["median"] if (dstar and dstar.get("driven_m")) else None,
        )
        html_parts.append('<tr><td>Driven Distance</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_drv, d_drv, r_drv))

        # Path ratio
        a_pr = html.escape(_format_stat(astar.get("path_ratio") if astar else None, digits=2))
        d_pr = html.escape(_format_stat(dstar.get("path_ratio") if dstar else None, digits=2))
        r_pr = _calc_ratio(
            astar["path_ratio"]["median"] if (astar and astar.get("path_ratio")) else None,
            dstar["path_ratio"]["median"] if (dstar and dstar.get("path_ratio")) else None,
        )
        html_parts.append('<tr><td>Path Ratio</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_pr, d_pr, r_pr))

        # Stops
        a_stp = html.escape(_format_stat(astar.get("stops") if astar else None, digits=1))
        d_stp = html.escape(_format_stat(dstar.get("stops") if dstar else None, digits=1))
        r_stp = _calc_ratio(
            astar["stops"]["median"] if (astar and astar.get("stops")) else None,
            dstar["stops"]["median"] if (dstar and dstar.get("stops")) else None,
        )
        html_parts.append('<tr><td>Stops</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_stp, d_stp, r_stp))

        # Min clearance
        a_clr = html.escape(_format_stat(astar.get("min_clearance_m") if astar else None, unit="m", digits=2))
        d_clr = html.escape(_format_stat(dstar.get("min_clearance_m") if dstar else None, unit="m", digits=2))
        r_clr = _calc_ratio(
            astar["min_clearance_m"]["median"] if (astar and astar.get("min_clearance_m")) else None,
            dstar["min_clearance_m"]["median"] if (dstar and dstar.get("min_clearance_m")) else None,
        )
        html_parts.append('<tr><td>Min Clearance</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_clr, d_clr, r_clr))

        # Collisions
        a_col = _format_collisions(astar)
        d_col = _format_collisions(dstar)
        html_parts.append('<tr><td>Collisions</td><td>{}</td><td>{}</td><td class="ratio-col">-</td></tr>'.format(
            html.escape(a_col), html.escape(d_col)
        ))

        # Motion quality metrics (TASK D)
        a_mxte = html.escape(_format_stat(astar.get("max_xte_m") if astar else None, unit="m", digits=3))
        d_mxte = html.escape(_format_stat(dstar.get("max_xte_m") if dstar else None, unit="m", digits=3))
        r_mxte = _calc_ratio(
            astar["max_xte_m"]["median"] if (astar and astar.get("max_xte_m")) else None,
            dstar["max_xte_m"]["median"] if (dstar and dstar.get("max_xte_m")) else None,
        )
        html_parts.append('<tr><td>Max XTE</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_mxte, d_mxte, r_mxte))

        a_xte = html.escape(_format_stat(astar.get("mean_xte_m") if astar else None, unit="m", digits=3))
        d_xte = html.escape(_format_stat(dstar.get("mean_xte_m") if dstar else None, unit="m", digits=3))
        r_xte = _calc_ratio(
            astar["mean_xte_m"]["median"] if (astar and astar.get("mean_xte_m")) else None,
            dstar["mean_xte_m"]["median"] if (dstar and dstar.get("mean_xte_m")) else None,
        )
        html_parts.append('<tr><td>Mean XTE</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_xte, d_xte, r_xte))

        a_osc = html.escape(_format_stat(astar.get("heading_osc") if astar else None, digits=1))
        d_osc = html.escape(_format_stat(dstar.get("heading_osc") if dstar else None, digits=1))
        r_osc = _calc_ratio(
            astar["heading_osc"]["median"] if (astar and astar.get("heading_osc")) else None,
            dstar["heading_osc"]["median"] if (dstar and dstar.get("heading_osc")) else None,
        )
        html_parts.append('<tr><td>Heading Oscillations</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_osc, d_osc, r_osc))

        a_ps = html.escape(_format_stat(astar.get("path_switches") if astar else None, digits=1))
        d_ps = html.escape(_format_stat(dstar.get("path_switches") if dstar else None, digits=1))
        r_ps = _calc_ratio(
            astar["path_switches"]["median"] if (astar and astar.get("path_switches")) else None,
            dstar["path_switches"]["median"] if (dstar and dstar.get("path_switches")) else None,
        )
        html_parts.append('<tr><td>Path Switches</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_ps, d_ps, r_ps))

        # First plan ms (cold)
        a_fp_cold = _format_stat(astar.get("first_plan_ms_all") if astar else None, unit="ms", digits=1)
        d_fp_cold = _format_stat(dstar.get("first_plan_ms_rebuilt_true") if dstar else None, unit="ms", digits=1)
        r_fp_cold = _calc_ratio(
            astar["first_plan_ms_all"]["median"] if (astar and astar.get("first_plan_ms_all")) else None,
            dstar["first_plan_ms_rebuilt_true"]["median"] if (dstar and dstar.get("first_plan_ms_rebuilt_true")) else None,
        )
        html_parts.append('<tr><td>First Plan ms (cold)</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(
            html.escape(a_fp_cold), html.escape(d_fp_cold), r_fp_cold
        ))

        # First plan ms (warm)
        d_fp_warm = _format_stat(dstar.get("first_plan_ms_rebuilt_false") if dstar else None, unit="ms", digits=1)
        html_parts.append('<tr><td>First Plan ms (warm)</td><td>-</td><td>{}</td><td class="ratio-col">-</td></tr>'.format(
            html.escape(d_fp_warm)
        ))

        # Replan ms
        a_rp_str = _format_replan(astar.get("replan_ms") if astar else None)
        d_rp_str = _format_replan(dstar.get("replan_ms") if dstar else None)
        r_rp = _calc_ratio(
            astar["replan_ms"]["median"] if (astar and astar.get("replan_ms")) else None,
            dstar["replan_ms"]["median"] if (dstar and dstar.get("replan_ms")) else None,
        )
        html_parts.append('<tr><td>Replan ms (median)</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(
            html.escape(a_rp_str), html.escape(d_rp_str), r_rp
        ))

        # Replan path ready (D* only)
        d_pr_str = _format_replan_pr(dstar.get("replan_path_ready_ms") if dstar else None)
        html_parts.append('<tr><td>&nbsp;&nbsp;↳ Replan Path Ready ms (D* only)</td><td>-</td><td>{}</td><td class="ratio-col">-</td></tr>'.format(
            html.escape(d_pr_str)
        ))

        # Planner time per run (s)
        a_pts = html.escape(_format_stat(astar.get("plan_time_s") if astar else None, unit="s", digits=2))
        d_pts = html.escape(_format_stat(dstar.get("plan_time_s") if dstar else None, unit="s", digits=2))
        r_pts = _calc_ratio(
            astar["plan_time_s"]["median"] if (astar and astar.get("plan_time_s")) else None,
            dstar["plan_time_s"]["median"] if (dstar and dstar.get("plan_time_s")) else None,
        )
        html_parts.append('<tr><td>Planner time per run (s)</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_pts, d_pts, r_pts))

        # Planner time share of run (%)
        a_psp = html.escape(_format_stat(astar.get("plan_share_pct") if astar else None, unit="%", digits=1))
        d_psp = html.escape(_format_stat(dstar.get("plan_share_pct") if dstar else None, unit="%", digits=1))
        r_psp = _calc_ratio(
            astar["plan_share_pct"]["median"] if (astar and astar.get("plan_share_pct")) else None,
            dstar["plan_share_pct"]["median"] if (dstar and dstar.get("plan_share_pct")) else None,
        )
        html_parts.append('<tr><td>Planner time share of run (%)</td><td>{}</td><td>{}</td><td class="ratio-col">{}</td></tr>'.format(a_psp, d_psp, r_psp))

        html_parts.append('</tbody>')
        html_parts.append('</table>')

        html_parts.append('</div>')

    # Footnotes
    html_parts.append('<div class="footnotes-box">')
    html_parts.append('<strong>Footnotes</strong>')
    html_parts.append('<ol>')
    html_parts.append('<li>{}</li>'.format(html.escape(FOOTNOTE_TIMER)))
    html_parts.append('<li>{}</li>'.format(html.escape(FOOTNOTE_REUSE)))
    html_parts.append('<li>{}</li>'.format(html.escape(FOOTNOTE_MOTION)))
    html_parts.append('<li>{}</li>'.format(html.escape(FOOTNOTE_RATIO)))
    html_parts.append('<li>{}</li>'.format(html.escape(FOOTNOTE_SPREAD)))
    html_parts.append('<li>{}</li>'.format(html.escape(FOOTNOTE_PLANNER_TIME)))
    html_parts.append('</ol>')
    html_parts.append('</div>')

    html_parts.append('</div>')
    html_parts.append('</body>')
    html_parts.append('</html>')

    return "\n".join(html_parts)


def generate_report_from_csvs(files: Sequence[str], out_dir: str) -> Tuple[str, str]:
    """Load runs from specified CSV files, aggregate metrics, and generate report files."""
    all_runs = []  # type: List[Dict[str, Any]]
    for fpath in files:
        all_runs.extend(load_runs_from_csv(fpath))

    # Group runs by scenario and planner
    scenarios = {}  # type: Dict[str, Dict[str, List[Dict[str, Any]]]]
    for r in all_runs:
        scen = str(r.get("scenario") or "Default").strip()
        pl = str(r.get("planner") or r.get("label") or "unknown").strip().lower()
        pl_key = "dstar" if "dstar" in pl else "astar"
        scenarios.setdefault(scen, {}).setdefault(pl_key, []).append(r)

    # Compute metrics
    scenarios_data = {}  # type: Dict[str, Dict[str, Any]]
    for scen, p_dict in scenarios.items():
        scenarios_data[scen] = {}
        for pl_key, r_list in p_dict.items():
            scenarios_data[scen][pl_key] = compute_planner_metrics(r_list)

    md_content = generate_markdown_report(scenarios_data)
    html_content = generate_html_report(scenarios_data)

    os.makedirs(out_dir, exist_ok=True)
    md_path = os.path.join(out_dir, "report.md")
    html_path = os.path.join(out_dir, "report.html")

    with open(md_path, "w", encoding="utf-8") as f:
        f.write(md_content)

    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html_content)

    return md_path, html_path


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point for benchmark report generation."""
    parser = argparse.ArgumentParser(description="Generate A/B benchmark reports from CSV runs")
    parser.add_argument("--dir", default=None, help="Directory to search for CSV files")
    parser.add_argument("--out", default=".", help="Output directory for reports")
    parser.add_argument("csv_files", nargs="*", help="CSV files to process")

    args = parser.parse_args(argv)

    files = list(args.csv_files)
    if args.dir:
        dir_path = os.path.expanduser(args.dir)
        if os.path.isdir(dir_path):
            for fname in sorted(os.listdir(dir_path)):
                if fname.endswith(".csv"):
                    files.append(os.path.join(dir_path, fname))

    if not files:
        print("Error: No CSV files provided or found.", file=sys.stderr)
        return 1

    out_dir = os.path.expanduser(args.out)
    md_path, html_path = generate_report_from_csvs(files, out_dir)
    print("Report generated successfully:")
    print("  Markdown: {}".format(md_path))
    print("  HTML:     {}".format(html_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
