#!/usr/bin/env python3
"""One-click unattended benchmark matrix: scenarios x planners x N runs (ADR-020).

Start Unity (Play), then run `start_matrix.bat` (or this script inside WSL). The runner

  1. prints a preflight checklist (WSL, stray ROS processes, disk, Windows power plan, scenario files) and refuses to
     start on a FAIL;
  2. runs every (scenario, planner) cell as one `ab_batch.py` attempt with the seeded protocol (ADR-019), a per-cell
     timeout, and the planner order alternating from scenario to scenario;
  3. tops up invalid runs with fresh run indices until N valid runs exist or `--max-attempts` is used up, and logs every
     invalid run with its reason in `invalid_runs.csv`;
  4. can be stopped with Ctrl-C and continued with `--resume <dir>`;
  5. writes `matrix_report.md` (plus `ab_report.py` reports per scenario) when it ends.

Never use start_ros.bat, stop_ros.bat or Unity menu 3 while it runs: they `killall python3`.
Python 3.8, standard library only.
"""

import argparse
import csv
import datetime
import glob
import hashlib
import json
import math
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import time
from typing import Any, Callable, Dict, List, NamedTuple, Optional, Sequence, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import ab_batch  # noqa: E402
import ab_run  # noqa: E402

DEFAULT_SCENARIOS = ["S-00c", "S-04", "S-06", "S-07", "S-08"]
DEFAULT_PLANNERS = ["astar", "dstar"]
DEFAULT_OUT_ROOT = "~/ab_matrix"
STACK_START_S = 120.0   # roslaunch + Unity handshake + verification, per attempt (measured: about 2 min)
RUN_S = 50.0            # one run: 7 s settle + about 40 s leg + reset (measured at sim speed 1.00x)
RETRY_ALLOWANCE = 1.1   # share added for top-ups when estimating
SETTLE_S = 7.0

# Draft rules for deciding which scenarios need improvement work (ADR-020, NOT approved yet).
DRAFT_SUCCESS_RATE = 0.95
DRAFT_GAP_M = 0.15
DRAFT_GAP_SHARE = 0.10
DRAFT_TIME_FACTOR = 1.3


class Cell(NamedTuple):
    scenario: str
    planner: str


def cell_key(cell: Cell) -> str:
    return "%s|%s" % (cell.scenario, cell.planner)


def cell_dirname(cell: Cell) -> str:
    return "%s_%s" % (cell.scenario, cell.planner)


def build_cells(scenarios: Sequence[str], planners: Sequence[str], alternate: bool = True) -> List[Cell]:
    """Cell order. With alternate, the first planner rotates from scenario to scenario, so neither planner is always
    measured first (the order effect that confounded the first benchmark)."""
    cells = []  # type: List[Cell]
    pl = list(planners)
    for i, scenario in enumerate(scenarios):
        k = (i % len(pl)) if alternate else 0
        for planner in pl[k:] + pl[:k]:
            cells.append(Cell(scenario, planner))
    return cells


def estimate_seconds(n_cells: int, repeats: int) -> float:
    return n_cells * (STACK_START_S + repeats * RUN_S * RETRY_ALLOWANCE)


def cell_timeout_s(repeats: int, leg_timeout_s: float, override: Optional[float] = None) -> float:
    """Wall-clock limit of one attempt: stack start, plus every run at its worst case, plus slack."""
    if override is not None:
        return float(override)
    return STACK_START_S + 120.0 + repeats * (leg_timeout_s + SETTLE_S + 15.0)


# ---------------- reading what the attempts wrote ----------------

def _int(value: Any) -> Optional[int]:
    try:
        return int(float(str(value).strip()))
    except (TypeError, ValueError):
        return None


def _float(value: Any) -> Optional[float]:
    try:
        text = str(value).strip()
        if text == "":
            return None
        v = float(text)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def read_cell_rows(cell_dir: str, planner: str) -> List[Dict[str, Any]]:
    """All rows of all attempts of a cell, each tagged with `_attempt` and `_session`."""
    rows = []  # type: List[Dict[str, Any]]
    pattern = os.path.join(cell_dir, "attempt*", "*", "%s_runs.csv" % planner)
    for path in sorted(glob.glob(pattern)):
        attempt_dir = os.path.basename(os.path.dirname(os.path.dirname(path)))
        attempt = _int(attempt_dir.replace("attempt", "")) or 0
        try:
            with open(path, "r", encoding="utf-8", newline="") as f:
                for row in csv.DictReader(f):
                    row["_attempt"] = attempt
                    row["_session"] = os.path.dirname(path)
                    rows.append(row)
        except OSError:
            continue
    return rows


def is_valid_row(row: Dict[str, Any]) -> bool:
    return str(row.get("outcome", "")).strip().lower() not in ("invalid", "")


def count_valid(rows: Sequence[Dict[str, Any]]) -> int:
    return len({_int(r.get("run_index")) for r in rows if is_valid_row(r)})


def next_attempt_plan(rows: Sequence[Dict[str, Any]], target_n: int) -> Optional[Tuple[int, int]]:
    """(first_index, repeats) of the next attempt, or None when the cell has N valid runs.
    A top-up uses fresh indices (so fresh seeds) after the highest index used so far."""
    missing = target_n - count_valid(rows)
    if missing <= 0:
        return None
    used = [i for i in (_int(r.get("run_index")) for r in rows) if i is not None]
    return (max(used) + 1 if used else 1), missing


# ---------------- preflight ----------------

class Check(NamedTuple):
    name: str
    status: str   # PASS, WARN, FAIL, INFO
    detail: str


def parse_hex_settings(text: str) -> List[int]:
    """The 0x... values of a `powercfg /query` output, in order. Locale independent."""
    return [int(h, 16) for h in re.findall(r"0x[0-9a-fA-F]+", text)]


def parse_ac_dc(text: str) -> Optional[Tuple[int, int]]:
    """(AC value, DC value) from the last two index lines of `powercfg /query <scheme> <sub> <setting>`."""
    values = parse_hex_settings(text)
    if len(values) < 2:
        return None
    return values[-2], values[-1]


def parse_battery(text: str) -> Optional[bool]:
    """True = on mains power (or no battery), False = running on battery, None = unknown."""
    m = re.search(r"BATTERY:(\w+)", text)
    if not m:
        return None
    if m.group(1) == "none":
        return True
    status = _int(m.group(1))
    if status is None:
        return None
    return status not in (1, 4, 5)  # discharging, low, critical


STRAY_PATTERN = re.compile(
    r"(roslaunch|rosmaster|roscore|rosout|default_server_endpoint|astar_planner|dstar_planner|path_follower|mission_node"
    r"|ab_run\.py|ab_batch\.py|ab_matrix\.py|rviz|nav_ws|turtlebot|warehouse)"
)


def find_stray_processes(ps_lines: Sequence[str], own_pids: Sequence[int]) -> List[str]:
    """Lines of `ps -eo pid=,args=` that look like ROS or benchmark processes other than ours."""
    stray = []  # type: List[str]
    for line in ps_lines:
        parts = line.strip().split(None, 1)
        if len(parts) != 2:
            continue
        pid = _int(parts[0])
        if pid is None or pid in own_pids:
            continue
        args = parts[1]
        if os.path.basename(args.split()[0]) in ("ps", "pgrep", "grep"):
            continue
        if STRAY_PATTERN.search(args):
            stray.append(line.strip()[:160])
    return stray


def _shell(cmd: Sequence[str], timeout: float = 20.0) -> Tuple[int, str]:
    try:
        res = subprocess.run(list(cmd), capture_output=True, text=True, timeout=timeout)
        return res.returncode, (res.stdout or "") + (res.stderr or "")
    except (OSError, subprocess.SubprocessError) as exc:
        return 127, str(exc)


def _powershell(command: str, shell: Callable[..., Tuple[int, str]]) -> Optional[str]:
    rc, out = shell(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command])
    return out if rc == 0 else None


def run_preflight(
    config: Dict[str, Any],
    out_root: str,
    runner: Any,
    shell: Callable[..., Tuple[int, str]] = _shell,
    own_pids: Optional[Sequence[int]] = None,
    disk_usage: Callable[[str], Any] = shutil.disk_usage,
    scenario_check: Optional[Callable[[], List[str]]] = None,
) -> List[Check]:
    """The checklist. FAIL stops the matrix (unless --force), WARN is shown, INFO lists the manual steps."""
    checks = []  # type: List[Check]
    est = estimate_seconds(len(build_cells(config["scenarios"], config["planners"])), config["repeats"])
    est_min = est / 60.0
    finish = datetime.datetime.now() + datetime.timedelta(seconds=est)
    checks.append(Check("Estimated time", "INFO", "about %.0f min (%.1f h) for %d cells x N=%d, until about %s" % (
        est_min, est_min / 60.0, len(build_cells(config["scenarios"], config["planners"])), config["repeats"],
        finish.strftime("%H:%M"))))

    # 1. WSL
    distro = os.environ.get("WSL_DISTRO_NAME", "")
    try:
        with open("/proc/version", "r", encoding="utf-8") as f:
            in_wsl = "microsoft" in f.read().lower()
    except OSError:
        in_wsl = False
    checks.append(Check("WSL", "PASS" if in_wsl else "FAIL",
                        ("running in WSL %s" % distro).strip() if in_wsl else "not inside WSL: run this in Ubuntu-20.04"))

    # 2. stray processes and a ROS master
    rc, out = shell(["ps", "-eo", "pid=,args="])
    pids = list(own_pids) if own_pids is not None else [os.getpid(), os.getppid()]
    stray = find_stray_processes(out.splitlines(), pids) if rc == 0 else []
    if rc != 0:
        checks.append(Check("Stray processes", "WARN", "could not list processes"))
    elif stray:
        checks.append(Check("Stray processes", "FAIL", "%d found, e.g. %s. Stop them first (Ctrl-C the old run, or "
                            "ab_batch.py --stop)." % (len(stray), stray[0])))
    else:
        checks.append(Check("Stray processes", "PASS", "no roslaunch, rosmaster, planner, ab_run or other ROS python3 running"))
    master_up = runner.is_ros_master_up()
    checks.append(Check("ROS master", "FAIL" if master_up else "PASS",
                        "a master is already answering on :11311 (the matrix starts its own)" if master_up else "none running"))

    # 3. disk
    probe = out_root
    while probe and not os.path.exists(probe):
        probe = os.path.dirname(probe)
    free_gb = disk_usage(probe or "/").free / 1e9
    checks.append(Check("WSL disk", "FAIL" if free_gb < 1.0 else ("WARN" if free_gb < 5.0 else "PASS"),
                        "%.1f GB free where results are written" % free_gb))
    text = _powershell("(Get-PSDrive C).Free", shell)
    win_free = _float((text or "").strip().splitlines()[0]) if text and text.strip() else None
    if win_free is None:
        checks.append(Check("Windows C: disk", "WARN", "could not read the free space of C:"))
    else:
        gb = win_free / 1e9
        checks.append(Check("Windows C: disk", "FAIL" if gb < 2.0 else ("WARN" if gb < 10.0 else "PASS"),
                            "%.1f GB free (the WSL disk image and Unity logs grow here)" % gb))

    # 4. Windows power: plan, mains, sleep, lid
    plan_txt = _powershell("powercfg /getactivescheme", shell)
    plan = re.search(r"\(([^)]+)\)", plan_txt or "")
    checks.append(Check("Power plan", "INFO" if plan else "WARN", plan.group(1) if plan else "could not read the power plan"))

    bat_txt = _powershell("$b=Get-CimInstance Win32_Battery; if($b){'BATTERY:'+@($b)[0].BatteryStatus}else{'BATTERY:none'}", shell)
    mains = parse_battery(bat_txt or "")
    if mains is None:
        checks.append(Check("Plugged in", "WARN", "could not read the battery state"))
    elif not mains:
        checks.append(Check("Plugged in", "FAIL", "running on battery: plug in the charger"))
    else:
        checks.append(Check("Plugged in", "PASS", "on mains power (or no battery)"))

    sleep_txt = _powershell("powercfg /query SCHEME_CURRENT SUB_SLEEP STANDBYIDLE", shell)
    sleep = parse_ac_dc(sleep_txt or "")
    if sleep is None:
        checks.append(Check("Sleep on AC", "WARN", "could not read the sleep timeout: set Sleep to Never while plugged in"))
    elif sleep[0] == 0:
        checks.append(Check("Sleep on AC", "PASS", "never"))
    elif sleep[0] < est:
        checks.append(Check("Sleep on AC", "FAIL", "the PC sleeps after %.0f min on AC but the matrix needs about %.0f min: "
                            "Settings > System > Power > Sleep = Never" % (sleep[0] / 60.0, est_min)))
    else:
        checks.append(Check("Sleep on AC", "WARN", "the PC sleeps after %.0f min on AC (the matrix needs about %.0f min)" % (
            sleep[0] / 60.0, est_min)))

    lid_txt = _powershell("powercfg /query SCHEME_CURRENT SUB_BUTTONS LIDACTION", shell)
    lid = parse_ac_dc(lid_txt or "")
    if lid is not None and lid[0] != 0:
        checks.append(Check("Lid close on AC", "WARN", "closing the lid does something (index %d): set it to Do nothing" % lid[0]))

    # 5. scenario files
    if scenario_check is not None:
        problems = scenario_check()
        checks.append(Check("Scenario files", "FAIL" if problems else "PASS",
                            "; ".join(problems) if problems else "scenarios.yaml is valid and scenarios.json is up to date"))
    for name in config.get("unchecked_layouts", []):
        checks.append(Check("Layout " + name, "WARN", "layout_checked is false: workers may start inside a rack (step U3)"))

    checks.append(Check("Unity (manual)", "INFO",
                        "Play is pressed and the Console has no red errors; the U1 compile and EditMode tests passed once "
                        "after the last C# change. A Unity without the new C# makes every seeded run 'no worker seen'."))
    checks.append(Check("Hands off (manual)", "INFO",
                        "do not touch Unity, drag objects or press O/0/B; never use start_ros.bat, stop_ros.bat or Unity menu 3 "
                        "(they killall python3)"))
    return checks


def format_preflight(checks: Sequence[Check]) -> str:
    lines = ["=== Preflight checklist ==="]
    for c in checks:
        lines.append("[%-4s] %-18s %s" % (c.status, c.name, c.detail))
    fails = [c for c in checks if c.status == "FAIL"]
    warns = [c for c in checks if c.status == "WARN"]
    lines.append("Result: %d FAIL, %d WARN" % (len(fails), len(warns)))
    return "\n".join(lines)


def scenario_problems(spec_module: Any) -> List[str]:
    """yaml validity and scenarios.json freshness, using warehouse_eval.scenario_spec."""
    try:
        config = spec_module.load_config()
        text = spec_module.dumps(spec_module.export(config))
    except (OSError, ValueError, KeyError) as exc:
        return ["scenarios.yaml: %s" % exc]
    try:
        with open(spec_module.DEFAULT_JSON, "r", encoding="utf-8", newline="") as f:
            current = f.read().replace("\r\n", "\n")
    except OSError:
        return ["Assets/StreamingAssets/scenarios.json is missing"]
    if current != text:
        return ["scenarios.json is stale: run python3 -m warehouse_eval.scenario_spec --write"]
    return []


# ---------------- statistics and the report ----------------

def _median(values: Sequence[float]) -> Optional[float]:
    return statistics.median(values) if values else None


def summarize_cell(rows: Sequence[Dict[str, Any]], target_n: int) -> Dict[str, Any]:
    valid = [r for r in rows if is_valid_row(r)]
    invalid = [r for r in rows if not is_valid_row(r)]
    succeeded = [r for r in valid if str(r.get("outcome", "")).strip().lower() == "succeeded"]
    gaps = [g for g in (_float(r.get("npc_min_gap_m")) for r in valid) if g is not None]
    times = [t for t in (_float(r.get("sim_time_to_goal_s")) for r in succeeded) if t is not None]
    coll = [_int(r.get("collisions")) or 0 for r in valid]
    coll_npc = [_int(r.get("collisions_npc")) or 0 for r in valid]
    return {
        "planned": target_n,
        "valid": len(valid),
        "invalid": len(invalid),
        "succeeded": len(succeeded),
        "timeouts": sum(1 for r in valid if str(r.get("outcome", "")).strip().lower() == "timeout"),
        "aborted": sum(1 for r in valid if str(r.get("outcome", "")).strip().lower() == "aborted"),
        "success_rate": (len(succeeded) / len(valid)) if valid else None,
        "runs_with_collision": sum(1 for c in coll if c > 0),
        "collisions": sum(coll),
        "collisions_npc": sum(coll_npc),
        "runs_with_npc_collision": sum(1 for c in coll_npc if c > 0),
        "gap_n": len(gaps),
        "gap_min": min(gaps) if gaps else None,
        "gap_median": _median(gaps),
        "gap_below_share": (sum(1 for g in gaps if g < DRAFT_GAP_M) / len(gaps)) if gaps else None,
        "time_median": _median(times),
        "time_min": min(times) if times else None,
        "time_max": max(times) if times else None,
        "early_replans_median": _median([v for v in (_float(r.get("early_replans")) for r in valid) if v is not None]),
        "stops_median": _median([v for v in (_float(r.get("stops")) for r in valid) if v is not None]),
        "path_ratio_median": _median([v for v in (_float(r.get("path_ratio")) for r in succeeded) if v is not None]),
    }


def draft_flags(scenario: str, planner: str, summ: Dict[str, Any], control_time: Optional[float]) -> List[str]:
    """Which draft rules (ADR-020) a cell trips. Not approved thresholds: they only point at where to look."""
    flags = []  # type: List[str]
    if summ["valid"] == 0:
        return ["no valid runs"]
    if summ["success_rate"] is not None and summ["success_rate"] < DRAFT_SUCCESS_RATE:
        flags.append("success %.0f%% < %.0f%%" % (100 * summ["success_rate"], 100 * DRAFT_SUCCESS_RATE))
    if summ["collisions_npc"] > 0:
        flags.append("%d worker collision(s) in %d run(s)" % (summ["collisions_npc"], summ["runs_with_npc_collision"]))
    if summ["gap_below_share"] is not None and summ["gap_below_share"] >= DRAFT_GAP_SHARE:
        flags.append("worker gap < %.2f m in %.0f%% of runs" % (DRAFT_GAP_M, 100 * summ["gap_below_share"]))
    if control_time and summ["time_median"] and summ["time_median"] > DRAFT_TIME_FACTOR * control_time:
        flags.append("median time %.1f s > %.1f x control %.1f s" % (summ["time_median"], DRAFT_TIME_FACTOR, control_time))
    if summ["timeouts"] > 0:
        flags.append("%d timeout(s): possible deadlock" % summ["timeouts"])
    return flags


def _fmt(value: Optional[float], digits: int = 2, unit: str = "") -> str:
    return "—" if value is None else ("%.*f%s" % (digits, value, unit))


def build_report(
    config: Dict[str, Any],
    cell_rows: Dict[str, List[Dict[str, Any]]],
    state: Dict[str, Any],
    invalid_log: Sequence[Dict[str, Any]],
) -> str:
    """The Markdown summary of a matrix. Pure: everything comes in as data."""
    cells = build_cells(config["scenarios"], config["planners"], config.get("alternate", True))
    summ = {cell_key(c): summarize_cell(cell_rows.get(cell_key(c), []), config["repeats"]) for c in cells}
    out = []  # type: List[str]
    out.append("# Benchmark matrix report")
    out.append("")
    out.append("Created %s · git `%s`%s · seed base %s · N=%d per cell%s" % (
        config.get("created", "?"), str(config.get("git_commit", "?"))[:12], " (dirty tree)" if config.get("git_dirty") else "",
        config.get("seed_base"), config["repeats"], " · DRY RUN" if config.get("dry") else ""))
    out.append("")
    incomplete = [c for c in cells if summ[cell_key(c)]["valid"] < config["repeats"]]
    if state.get("aborted"):
        out.append("> **The matrix stopped early:** %s" % state["aborted"])
        out.append("")
    if incomplete:
        out.append("> **Incomplete cells (fewer than N valid runs):** " + ", ".join(
            "%s/%s (%d of %d)" % (c.scenario, c.planner, summ[cell_key(c)]["valid"], config["repeats"]) for c in incomplete))
        out.append("")
    unchecked = config.get("unchecked_layouts", [])
    if unchecked:
        out.append("> **Layout not checked in Unity (U3):** %s. Worker geometry is unverified for these scenarios." % ", ".join(unchecked))
        out.append("")

    out.append("## Results per cell")
    out.append("")
    out.append("| Scenario | Planner | Valid / N | Invalid | Success | Runs with collision | Worker collisions | Worker gap min / median (m) | "
               "Time to goal median [min, max] (s) | Early replans (median) | Timeouts |")
    out.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for c in cells:
        s = summ[cell_key(c)]
        rate = "—" if s["success_rate"] is None else "%d/%d (%.0f%%)" % (s["succeeded"], s["valid"], 100 * s["success_rate"])
        tmm = "—" if s["time_median"] is None else "%.1f [%.1f, %.1f]" % (s["time_median"], s["time_min"], s["time_max"])
        out.append("| %s | %s | %d / %d | %d | %s | %d | %d | %s / %s | %s | %s | %d |" % (
            c.scenario, c.planner, s["valid"], config["repeats"], s["invalid"], rate, s["runs_with_collision"],
            s["collisions_npc"], _fmt(s["gap_min"]), _fmt(s["gap_median"]), tmm, _fmt(s["early_replans_median"], 1), s["timeouts"]))
    out.append("")

    out.append("## Draft rule flags (ADR-020, thresholds NOT approved)")
    out.append("")
    out.append("A cell is listed when it trips a draft rule: success < %.0f %%, any worker collision, worker gap < %.2f m in "
               "≥ %.0f %% of runs, median time > %.1f x the S-00c median of the same planner, or a timeout (a deadlock suspect: "
               "the CSV cannot tell a deadlock from a slow run). These only point at where improvement work may be needed." % (
                   100 * DRAFT_SUCCESS_RATE, DRAFT_GAP_M, 100 * DRAFT_GAP_SHARE, DRAFT_TIME_FACTOR))
    out.append("")
    any_flag = False
    for c in cells:
        control = summ.get(cell_key(Cell("S-00c", c.planner)), {}).get("time_median") if c.scenario != "S-00c" else None
        flags = draft_flags(c.scenario, c.planner, summ[cell_key(c)], control)
        if flags and c.scenario != "S-00c":
            any_flag = True
            out.append("- **%s / %s:** %s" % (c.scenario, c.planner, "; ".join(flags)))
    if not any_flag:
        out.append("- No cell trips a draft rule.")
    out.append("")

    out.append("## Invalid runs")
    out.append("")
    if not invalid_log:
        out.append("None.")
    else:
        reasons = {}  # type: Dict[str, int]
        for item in invalid_log:
            reasons[str(item.get("reason", ""))] = reasons.get(str(item.get("reason", "")), 0) + 1
        out.append("%d invalid run(s) or failed attempt(s). By reason: %s. Every one is in `invalid_runs.csv`." % (
            len(invalid_log), ", ".join("%s: %d" % (k or "unspecified", v) for k, v in sorted(reasons.items()))))
    out.append("")

    out.append("## Attempts")
    out.append("")
    out.append("| Cell | Attempt | Runs (first index, count) | Exit | Timed out | Stack failed |")
    out.append("|---|---|---|---|---|---|")
    for c in cells:
        for a in state.get("attempts", {}).get(cell_key(c), []):
            out.append("| %s/%s | %d | %s, %s | %s | %s | %s |" % (
                c.scenario, c.planner, a.get("attempt", 0), a.get("first_index"), a.get("repeats"), a.get("exit_code"),
                "yes" if a.get("timed_out") else "no", "yes" if a.get("stack_failed") else "no"))
    out.append("")
    out.append("## Protocol")
    out.append("")
    out.append("Seeded reset (ADR-019): run k of a cell resets `<scenario>?seed=%s + k`, so the same run index meets the same worker "
               "timing for both planners. A top-up of invalid runs uses new indices and therefore new seeds. The planner order "
               "alternates from scenario to scenario%s. Per-scenario reports from `ab_report.py` are in `report/<scenario>/`." % (
                   config.get("seed_base"), "" if config.get("alternate", True) else " (alternation was OFF)"))
    out.append("")
    return "\n".join(out)


# ---------------- the matrix ----------------

class Matrix:
    def __init__(self, config: Dict[str, Any], out_dir: str, runner: Any, clock: Callable[[], float] = time.time,
                 goals: Optional[Dict[str, Tuple[float, float]]] = None, leg_timeouts: Optional[Dict[str, float]] = None,
                 ab_batch_path: Optional[str] = None, spec_module: Any = None) -> None:
        self.config = config
        self.out_dir = out_dir
        self.runner = runner
        self.clock = clock
        self.goals = goals or {}
        self.leg_timeouts = leg_timeouts or {}
        self.ab_batch_path = ab_batch_path or os.path.join(SCRIPT_DIR, "ab_batch.py")
        self.spec_module = spec_module
        self.interrupted = False
        self.active_pid = None  # type: Optional[int]
        self.logger = None  # type: Optional[ab_batch.TeeLogger]
        self.state = self._load_state()
        self.cells = build_cells(config["scenarios"], config["planners"], config.get("alternate", True))

    # -- files --
    def path(self, *parts: str) -> str:
        return os.path.join(self.out_dir, *parts)

    def _load_state(self) -> Dict[str, Any]:
        try:
            with open(os.path.join(self.out_dir, "matrix_state.json"), "r", encoding="utf-8") as f:
                state = json.load(f)
            if isinstance(state, dict):
                state.setdefault("attempts", {})
                return state
        except (OSError, ValueError):
            pass
        return {"attempts": {}, "aborted": None}

    def save_state(self) -> None:
        tmp = self.path("matrix_state.json.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.state, f, indent=2)
        os.replace(tmp, self.path("matrix_state.json"))

    def log(self, msg: str) -> None:
        if self.logger:
            self.logger.log(msg)
        else:
            print(msg)

    def cell_dir(self, cell: Cell) -> str:
        return self.path("cells", cell_dirname(cell))

    def rows(self, cell: Cell) -> List[Dict[str, Any]]:
        return read_cell_rows(self.cell_dir(cell), cell.planner)

    # -- one attempt --
    def _attempt_cmd(self, cell: Cell, attempt_dir: str, first_index: int, repeats: int) -> List[str]:
        goal = self.goals.get(cell.scenario, (-3.75, -3.13))
        cmd = [sys.executable, self.ab_batch_path, "--planner", cell.planner, "--scenario", cell.scenario,
               "--goal", str(goal[0]), str(goal[1]), "--repeats", str(repeats),
               "--settle", str(SETTLE_S), "--timeout", str(self.leg_timeouts.get(cell.scenario, 90.0)),
               "--out", attempt_dir, "--seed-base", str(self.config["seed_base"]),
               "--first-index", str(first_index), "--no-report"]
        if self.config.get("dry"):
            cmd.append("--dry")
        return cmd

    def _wait_for(self, proc: Any, timeout_s: float) -> Tuple[Optional[int], bool]:
        """(exit code, timed_out). On the timeout or an interrupt the attempt is stopped: SIGINT, then SIGTERM, then SIGKILL."""
        t0 = self.clock()
        while True:
            code = proc.poll()
            if code is not None:
                return code, False
            timed_out = self.clock() - t0 > timeout_s
            if timed_out or self.interrupted:
                self.log("%s: stopping the attempt (SIGINT)..." % ("per-cell timeout" if timed_out else "interrupted"))
                self._stop_proc(proc)
                return proc.poll(), timed_out
            self.runner.sleep(1.0)

    def _stop_proc(self, proc: Any) -> None:
        for sig, wait_s in ((signal.SIGINT, 90.0), (signal.SIGTERM, 15.0), (signal.SIGKILL, 5.0)):
            if not self.runner.is_pid_alive(proc.pid):
                return
            self.runner.kill_pg(proc.pid, sig)
            t0 = self.clock()
            while self.clock() - t0 < wait_s and self.runner.is_pid_alive(proc.pid):
                self.runner.sleep(1.0)

    def run_cell(self, cell: Cell) -> bool:
        """Runs attempts until the cell has N valid runs. Returns False after a failed stack start."""
        n_target = self.config["repeats"]
        key = cell_key(cell)
        attempts = self.state["attempts"].setdefault(key, [])
        last_stack_failed = False
        while not self.interrupted:
            rows = self.rows(cell)
            plan = next_attempt_plan(rows, n_target)
            if plan is None:
                break
            if len(attempts) >= self.config["max_attempts"]:
                self.log("%s: %d of %d valid runs after %d attempts: giving up on this cell" % (
                    key, count_valid(rows), n_target, len(attempts)))
                break
            first_index, repeats = plan
            attempt_no = len(attempts) + 1
            attempt_dir = os.path.join(self.cell_dir(cell), "attempt%d" % attempt_no)
            os.makedirs(attempt_dir, exist_ok=True)
            timeout = cell_timeout_s(repeats, self.leg_timeouts.get(cell.scenario, 90.0), self.config.get("cell_timeout_s"))
            self.log("%s: attempt %d, runs %d..%d, cell timeout %.0f min" % (
                key, attempt_no, first_index, first_index + repeats - 1, timeout / 60.0))
            started = datetime.datetime.now().isoformat()
            env = dict(os.environ)
            env["PYTHONUNBUFFERED"] = "1"
            with open(os.path.join(attempt_dir, "ab_batch.out"), "w", encoding="utf-8") as out_f:
                proc = self.runner.popen(self._attempt_cmd(cell, attempt_dir, first_index, repeats), stdout=out_f,
                                         stderr=subprocess.STDOUT, preexec_fn=os.setsid if hasattr(os, "setsid") else None, env=env)
                self.active_pid = proc.pid
                code, timed_out = self._wait_for(proc, timeout)
                self.active_pid = None
            ended = datetime.datetime.now().isoformat()
            new_rows = [r for r in self.rows(cell) if r["_attempt"] == attempt_no]
            stack_failed = not new_rows and code != 0
            attempts.append({"attempt": attempt_no, "first_index": first_index, "repeats": repeats, "exit_code": code,
                             "timed_out": timed_out, "stack_failed": stack_failed, "started": started, "ended": ended,
                             "interrupted": self.interrupted})
            self._log_invalid(cell, attempt_no, new_rows, stack_failed, timed_out, code)
            self.save_state()
            if timed_out and self.runner.is_ros_master_up():
                self.state["aborted"] = "a ROS master is still up after the timeout of %s: stop it (ab_batch.py --stop, or reboot WSL) " \
                                        "and resume with --resume" % key
                self.save_state()
                self.interrupted = True
                return False
            last_stack_failed = stack_failed
            self.log("%s: attempt %d done: exit %s, %d valid of %d runs so far%s" % (
                key, attempt_no, code, count_valid(self.rows(cell)), n_target, ", STACK FAILED" if stack_failed else ""))
            if self.config.get("dry"):
                break
        return not last_stack_failed

    def _log_invalid(self, cell: Cell, attempt_no: int, rows: Sequence[Dict[str, Any]], stack_failed: bool,
                     timed_out: bool, code: Optional[int]) -> None:
        path = self.path("invalid_runs.csv")
        new_file = not os.path.exists(path)
        with open(path, "a", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(["scenario", "planner", "attempt", "run_index", "seed", "reason", "session"])
            for r in rows:
                if not is_valid_row(r):
                    w.writerow([cell.scenario, cell.planner, attempt_no, r.get("run_index"), r.get("seed"),
                                r.get("invalid_reason") or "unspecified", r.get("_session")])
            if stack_failed:
                w.writerow([cell.scenario, cell.planner, attempt_no, "", "", "attempt produced no runs (ab_batch exit %s%s)" % (
                    code, ", timed out" if timed_out else ""), ""])
            elif timed_out:
                w.writerow([cell.scenario, cell.planner, attempt_no, "", "", "cell timeout: the attempt was stopped", ""])

    def read_invalid_log(self) -> List[Dict[str, Any]]:
        try:
            with open(self.path("invalid_runs.csv"), "r", encoding="utf-8", newline="") as f:
                return list(csv.DictReader(f))
        except OSError:
            return []

    # -- the whole matrix --
    def run(self) -> int:
        os.makedirs(self.out_dir, exist_ok=True)
        self.logger = ab_batch.TeeLogger(self.path("matrix.log"))
        try:
            consecutive_stack_failures = 0
            for i, cell in enumerate(self.cells):
                if self.interrupted:
                    break
                self.log("\n=== Cell %d / %d: %s ===" % (i + 1, len(self.cells), cell_key(cell)))
                ok = self.run_cell(cell)
                consecutive_stack_failures = 0 if ok else consecutive_stack_failures + 1
                if consecutive_stack_failures >= 2 and not self.interrupted:
                    self.state["aborted"] = "the ROS stack failed to start twice in a row: is Unity in Play mode?"
                    self.save_state()
                    self.log("ABORT: " + self.state["aborted"])
                    break
            self.save_state()
            self.finalize()
        finally:
            if self.logger:
                self.logger.close()
        complete = all(count_valid(self.rows(c)) >= self.config["repeats"] for c in self.cells)
        if self.state.get("aborted") or self.interrupted:
            return 1
        return 0 if complete else 2

    def finalize(self) -> None:
        """Merged CSVs, ab_report per scenario, matrix_report.md."""
        cell_rows = {cell_key(c): self.rows(c) for c in self.cells}
        for scenario in self.config["scenarios"]:
            csvs = []
            for planner in self.config["planners"]:
                rows = cell_rows.get(cell_key(Cell(scenario, planner)), [])
                if not rows:
                    continue
                path = self.path("data", scenario, "%s_runs.csv" % planner)
                os.makedirs(os.path.dirname(path), exist_ok=True)
                rows = sorted(rows, key=lambda r: _int(r.get("run_index")) or 0)
                with open(path, "w", encoding="utf-8", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=ab_run.CSV_COLUMNS, extrasaction="ignore")
                    w.writeheader()
                    w.writerows(rows)
                csvs.append(path)
            if csvs and not self.config.get("dry"):
                report_dir = self.path("report", scenario)
                res = self.runner.run([sys.executable, os.path.join(SCRIPT_DIR, "ab_report.py"), *csvs, "--out", report_dir],
                                      capture_output=True, text=True)
                if res.returncode != 0:
                    self.log("ab_report failed for %s (exit %s)" % (scenario, res.returncode))
        report = build_report(self.config, cell_rows, self.state, self.read_invalid_log())
        with open(self.path("matrix_report.md"), "w", encoding="utf-8") as f:
            f.write(report)
        self.log("\n" + report)
        self.log("Report: %s" % self.path("matrix_report.md"))
        try:
            with open(os.path.join(os.path.dirname(self.out_dir), "latest_matrix.txt"), "w", encoding="utf-8") as f:
                f.write(self.out_dir + "\n")
        except OSError:
            pass


# ---------------- command line ----------------

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Unattended benchmark matrix: scenarios x planners x N (ADR-020)")
    p.add_argument("--scenarios", default=",".join(DEFAULT_SCENARIOS), help="Comma separated (default: %s)" % ",".join(DEFAULT_SCENARIOS))
    p.add_argument("--planners", default=",".join(DEFAULT_PLANNERS), help="Comma separated (default: astar,dstar)")
    p.add_argument("--repeats", type=int, default=20, help="Valid runs wanted per cell (default: 20)")
    p.add_argument("--seed-base", type=int, default=1000, help="Run k of every cell uses seed base + k (default: 1000)")
    p.add_argument("--max-attempts", type=int, default=3, help="Attempts per cell, the first plus top-ups of invalid runs (default: 3)")
    p.add_argument("--cell-timeout-s", type=float, default=None, help="Wall-clock limit of one attempt (default: computed from N)")
    p.add_argument("--no-alternate", action="store_true", help="Do not rotate the planner order between scenarios")
    p.add_argument("--out", default=DEFAULT_OUT_ROOT, help="Root of the matrix directories (default: ~/ab_matrix)")
    p.add_argument("--resume", default=None, metavar="DIR", help="Continue the matrix in DIR (its configuration is reused)")
    p.add_argument("--dry", action="store_true", help="3 runs per cell, no top-ups, no ab_report (a check of the whole chain)")
    p.add_argument("--preflight-only", action="store_true", help="Print the checklist and exit")
    p.add_argument("--force", action="store_true", help="Start although the preflight has FAILs")
    p.add_argument("--stop", action="store_true", help="Ask the running matrix in --resume DIR to stop (SIGINT)")
    return p.parse_args(argv)


def _sha256(path: str) -> Optional[str]:
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return None


def new_config(args: argparse.Namespace, spec_module: Any, runner: Any) -> Dict[str, Any]:
    scenarios = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    planners = [p.strip().lower() for p in args.planners.split(",") if p.strip()]
    for p in planners:
        if p not in ("astar", "dstar"):
            raise ValueError("unknown planner %s" % p)
    cfg = spec_module.load_config()
    unchecked = []  # type: List[str]
    for s in scenarios:
        if not spec_module.is_spec_scenario(cfg, s):
            raise ValueError("scenario %s is not a spec-driven scenario of scenarios.yaml (no workers: list)" % s)
        if not cfg["scenarios"][s].get("layout_checked", False):
            unchecked.append(s)
    status = runner.git_status_short(timeout=60.0) if hasattr(runner, "git_status_short") else ""
    return {
        "scenarios": scenarios, "planners": planners, "repeats": 3 if args.dry else args.repeats, "seed_base": args.seed_base,
        "max_attempts": 1 if args.dry else args.max_attempts, "cell_timeout_s": args.cell_timeout_s,
        "alternate": not args.no_alternate, "dry": bool(args.dry), "unchecked_layouts": unchecked,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "git_commit": runner.git_commit_hash(timeout=60.0) if hasattr(runner, "git_commit_hash") else "",
        "git_dirty": bool(str(status).strip()) and "unavailable" not in str(status).lower(),
        "scenarios_yaml_sha256": _sha256(spec_module.DEFAULT_YAML), "scenarios_json_sha256": _sha256(spec_module.DEFAULT_JSON),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    spec = ab_run.load_scenario_spec()
    runner = ab_batch.Runner()

    if args.stop:
        if not args.resume:
            print("--stop needs --resume DIR", file=sys.stderr)
            return 1
        try:
            with open(os.path.join(os.path.expanduser(args.resume), "matrix.pid"), "r", encoding="utf-8") as f:
                pid = int(f.read().strip())
        except (OSError, ValueError):
            print("No running matrix found in %s" % args.resume)
            return 0
        runner.kill_pg(pid, signal.SIGINT)
        print("Sent SIGINT to matrix PID %d; it stops the current attempt and writes its report." % pid)
        return 0

    out_root = os.path.expanduser(args.out)
    if args.resume:
        out_dir = os.path.expanduser(args.resume)
        with open(os.path.join(out_dir, "matrix_config.json"), "r", encoding="utf-8") as f:
            config = json.load(f)
        config.setdefault("alternate", True)
        if "--max-attempts" in (argv if argv is not None else sys.argv[1:]):
            config["max_attempts"] = args.max_attempts
    else:
        config = new_config(args, spec, runner)
        out_dir = os.path.join(out_root, ("dry_" if args.dry else "") + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))

    checks = run_preflight(config, out_root, runner, scenario_check=lambda: scenario_problems(spec))
    print(format_preflight(checks))
    if args.preflight_only:
        return 1 if any(c.status == "FAIL" for c in checks) else 0
    if any(c.status == "FAIL" for c in checks) and not args.force:
        print("Preflight failed. Fix the FAIL lines, or start anyway with --force.", file=sys.stderr)
        return 1

    cfg = spec.load_config()
    goals = {s: spec.scenario_goal(cfg, s) for s in config["scenarios"]}
    leg_timeouts = {s: float(cfg["scenarios"][s].get("leg_timeout_s", 90.0)) for s in config["scenarios"]}
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "matrix_config.json"), "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)
    pid_path = os.path.join(out_dir, "matrix.pid")
    with open(pid_path, "w", encoding="utf-8") as f:
        f.write(str(os.getpid()))

    matrix = Matrix(config, out_dir, runner, goals=goals, leg_timeouts=leg_timeouts, spec_module=spec)

    def on_signal(_signum: int, _frame: Any) -> None:
        matrix.interrupted = True

    old_int = signal.signal(signal.SIGINT, on_signal)
    old_term = signal.signal(signal.SIGTERM, on_signal)
    try:
        print("Matrix directory: %s  (resume with --resume %s)" % (out_dir, out_dir))
        return matrix.run()
    finally:
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        try:
            os.remove(pid_path)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
