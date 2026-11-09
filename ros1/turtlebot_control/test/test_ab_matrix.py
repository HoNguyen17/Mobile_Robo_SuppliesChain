"""ab_matrix.py: cell order, top-ups, timeouts, resume, preflight, report (ADR-020). Pure Python, no ROS, no Unity."""
import ast
import csv
import json
import os
import signal
import sys
import tempfile
import types
import unittest
from typing import Any, Callable, Dict, List, Optional

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
if TEST_DIR not in sys.path:
    sys.path.insert(0, TEST_DIR)

import support  # noqa: F401,E402 (adds scripts/ to sys.path)
import ab_matrix  # noqa: E402
import ab_run  # noqa: E402

SCRIPTS = os.path.normpath(os.path.join(TEST_DIR, "..", "scripts"))
REPO = os.path.normpath(os.path.join(TEST_DIR, "..", "..", ".."))


def row(run_index, outcome="succeeded", reason="", **kw):
    r = {c: "" for c in ab_run.CSV_COLUMNS}
    r.update({"run_index": run_index, "outcome": outcome, "invalid_reason": reason, "seed": 1000 + run_index,
              "collisions": 0, "collisions_npc": 0, "npc_min_gap_m": 0.5, "sim_time_to_goal_s": 40.0, "early_replans": 2})
    r.update(kw)
    return r


class FakeProc:
    def __init__(self, pid: int, code: Optional[int]) -> None:
        self.pid = pid
        self._code = code

    def poll(self) -> Optional[int]:
        return self._code


class FakeRunner:
    """Stands in for ab_batch.Runner. `behavior(cell_info) -> rows | None` decides what an ab_batch attempt writes;
    None = the attempt hangs until it is killed."""

    def __init__(self, behavior: Callable[[Dict[str, Any]], Any]) -> None:
        self.behavior = behavior
        self.now = 1000.0
        self.calls = []  # type: List[Dict[str, Any]]
        self.killed = []  # type: List[Any]
        self.alive = set()
        self.master_up = False
        self.reports = []  # type: List[List[str]]
        self.next_pid = 5000
        self.procs = {}  # type: Dict[int, FakeProc]
        self.on_popen = None  # type: Optional[Callable[[Dict[str, Any]], None]]

    def clock(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds

    def is_ros_master_up(self) -> bool:
        return self.master_up

    def is_pid_alive(self, pid: int) -> bool:
        return pid in self.alive

    def kill_pg(self, pid: int, sig: int) -> None:
        self.killed.append((pid, sig))
        self.alive.discard(pid)
        self.procs[pid]._code = 130

    def git_commit_hash(self, timeout: float = 0.0) -> str:
        return "abcdef1234567890"

    def git_status_short(self, timeout: float = 0.0) -> str:
        return ""

    def run(self, cmd, **kw):
        self.reports.append(list(cmd))
        return types.SimpleNamespace(returncode=0, stdout="", stderr="")

    def popen(self, cmd, stdout=None, stderr=None, preexec_fn=None, **kw):
        a = cmd
        info = {
            "planner": a[a.index("--planner") + 1], "scenario": a[a.index("--scenario") + 1],
            "repeats": int(a[a.index("--repeats") + 1]), "first_index": int(a[a.index("--first-index") + 1]),
            "seed_base": int(a[a.index("--seed-base") + 1]), "out": a[a.index("--out") + 1], "cmd": list(cmd),
            "attempt": int(os.path.basename(a[a.index("--out") + 1]).replace("attempt", "")),
            "n_calls": len(self.calls),
        }
        self.calls.append(info)
        pid = self.next_pid
        self.next_pid += 1
        if self.on_popen:
            self.on_popen(info)
        rows = self.behavior(info)
        if rows is None:
            proc = FakeProc(pid, None)
            self.alive.add(pid)
        else:
            session = os.path.join(info["out"], "20261109_120000")
            os.makedirs(session, exist_ok=True)
            if rows:
                with open(os.path.join(session, "%s_runs.csv" % info["planner"]), "w", encoding="utf-8", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=ab_run.CSV_COLUMNS)
                    w.writeheader()
                    w.writerows(rows)
            proc = FakeProc(pid, 0 if rows else 1)
        self.procs[pid] = proc
        return proc


def all_valid(info):
    return [row(i) for i in range(info["first_index"], info["first_index"] + info["repeats"])]


def make_config(**kw):
    cfg = {"scenarios": ["S-04", "S-06"], "planners": ["astar", "dstar"], "repeats": 5, "seed_base": 1000, "max_attempts": 3,
           "cell_timeout_s": None, "alternate": True, "dry": False, "unchecked_layouts": [], "created": "2026-11-09T12:00:00",
           "git_commit": "abcdef1234567890", "git_dirty": False}
    cfg.update(kw)
    return cfg


def make_matrix(behavior, tmpdir, **cfg):
    runner = FakeRunner(behavior)
    m = ab_matrix.Matrix(make_config(**cfg), tmpdir, runner, clock=runner.clock,
                         goals={"S-04": (-3.75, -3.13), "S-06": (-3.75, -3.13)}, leg_timeouts={"S-04": 90.0, "S-06": 90.0})
    return m, runner


class PureHelpersTest(unittest.TestCase):
    def test_the_first_planner_alternates_from_scenario_to_scenario(self):
        cells = ab_matrix.build_cells(["A", "B", "C"], ["astar", "dstar"])
        self.assertEqual([(c.scenario, c.planner) for c in cells],
                         [("A", "astar"), ("A", "dstar"), ("B", "dstar"), ("B", "astar"), ("C", "astar"), ("C", "dstar")])
        fixed = ab_matrix.build_cells(["A", "B"], ["astar", "dstar"], alternate=False)
        self.assertEqual([c.planner for c in fixed], ["astar", "dstar", "astar", "dstar"])

    def test_estimate_and_timeout(self):
        self.assertAlmostEqual(ab_matrix.estimate_seconds(10, 20) / 3600.0, 10 * (120 + 20 * 50 * 1.1) / 3600.0)
        self.assertEqual(ab_matrix.cell_timeout_s(20, 90.0), 120 + 120 + 20 * (90 + 7 + 15))
        self.assertEqual(ab_matrix.cell_timeout_s(20, 90.0, override=600.0), 600.0)

    def test_next_attempt_plan_tops_up_with_fresh_indices(self):
        self.assertEqual(ab_matrix.next_attempt_plan([], 20), (1, 20))
        rows = [row(i) for i in range(1, 21)]
        self.assertIsNone(ab_matrix.next_attempt_plan(rows, 20))
        rows[3] = row(4, "invalid", "settle movement")
        rows[9] = row(10, "invalid", "pose off start")
        self.assertEqual(ab_matrix.next_attempt_plan(rows, 20), (21, 2))
        self.assertEqual(ab_matrix.next_attempt_plan([row(1, "invalid", "x")], 3), (2, 3))

    def test_a_timeout_or_aborted_run_is_a_valid_run_but_invalid_is_not(self):
        rows = [row(1, "timeout"), row(2, "aborted"), row(3, "invalid", "x"), row(4)]
        self.assertEqual(ab_matrix.count_valid(rows), 3)

    def test_parsers(self):
        self.assertEqual(ab_matrix.parse_ac_dc("Power Setting Index: 0x00000000\n Current AC Power Setting Index: 0x00000384\n"
                                              " Current DC Power Setting Index: 0x00000258\n"), (900, 600))
        self.assertIsNone(ab_matrix.parse_ac_dc("nothing"))
        self.assertTrue(ab_matrix.parse_battery("BATTERY:none"))
        self.assertTrue(ab_matrix.parse_battery("BATTERY:2"))
        self.assertFalse(ab_matrix.parse_battery("BATTERY:1"))
        self.assertIsNone(ab_matrix.parse_battery("garbage"))

    def test_stray_processes(self):
        ps = ["  1 /sbin/init", " 200 /usr/bin/python3 /usr/bin/networkd-dispatcher", " 300 python3 /opt/ros/noetic/bin/roslaunch x y",
              " 301 /usr/bin/python3 /home/u/nav_ws/src/ros_tcp_endpoint/default_server_endpoint.py", " 400 python3 ab_matrix.py",
              " 500 ps -eo pid=,args=", " 600 bash -c something"]
        found = ab_matrix.find_stray_processes(ps, own_pids=[400])
        self.assertEqual(len(found), 2)
        self.assertTrue(all("roslaunch" in f or "default_server_endpoint" in f for f in found))


class PreflightTest(unittest.TestCase):
    CFG = make_config(repeats=20)

    def shell(self, battery="BATTERY:2", sleep_ac=0, free_c="500000000000", plan="Power Scheme GUID: x  (Balanced)",
              ps="  1 /sbin/init", lid_ac=0, fail_power=False):
        def run(cmd, timeout=20.0):
            if cmd[0] == "ps":
                return 0, ps
            if fail_power:
                return 127, "no powershell"
            text = cmd[-1]
            if "Get-PSDrive" in text:
                return 0, free_c + "\n"
            if "getactivescheme" in text:
                return 0, plan
            if "Win32_Battery" in text:
                return 0, battery
            if "STANDBYIDLE" in text:
                return 0, "Current AC Power Setting Index: 0x%08x\nCurrent DC Power Setting Index: 0x00000258\n" % sleep_ac
            if "LIDACTION" in text:
                return 0, "Current AC Power Setting Index: 0x%08x\nCurrent DC Power Setting Index: 0x00000001\n" % lid_ac
            return 1, ""
        return run

    def preflight(self, runner=None, **kw):
        shell_kw = {k: kw.pop(k) for k in list(kw) if k in ("battery", "sleep_ac", "free_c", "plan", "ps", "lid_ac", "fail_power")}
        runner = runner or FakeRunner(all_valid)
        cfg = kw.pop("cfg", self.CFG)
        usage = kw.pop("disk", lambda p: types.SimpleNamespace(free=50e9))
        with tempfile.TemporaryDirectory() as tmp:
            return {c.name: c for c in ab_matrix.run_preflight(
                cfg, tmp, runner, shell=self.shell(**shell_kw), own_pids=[1], disk_usage=usage,
                scenario_check=kw.pop("scenario_check", lambda: []))}

    def test_a_healthy_machine_has_no_fail_and_lists_the_manual_steps(self):
        checks = self.preflight()
        self.assertEqual([c.name for c in checks.values() if c.status == "FAIL"], [])  # tests run inside WSL (CLAUDE.md)
        for name in ("Estimated time", "Unity (manual)", "Hands off (manual)"):
            self.assertEqual(checks[name].status, "INFO")
        self.assertEqual(checks["Plugged in"].status, "PASS")
        self.assertEqual(checks["Sleep on AC"].status, "PASS")
        self.assertEqual(checks["Power plan"].detail, "Balanced")

    def test_battery_is_a_fail(self):
        self.assertEqual(self.preflight(battery="BATTERY:1")["Plugged in"].status, "FAIL")

    def test_a_sleep_timeout_shorter_than_the_matrix_is_a_fail_and_a_long_one_a_warn(self):
        self.assertEqual(self.preflight(sleep_ac=900)["Sleep on AC"].status, "FAIL")
        self.assertEqual(self.preflight(sleep_ac=10 * 3600 * 10)["Sleep on AC"].status, "WARN")

    def test_a_lid_action_is_a_warn(self):
        self.assertEqual(self.preflight(lid_ac=1)["Lid close on AC"].status, "WARN")

    def test_stray_ros_processes_and_a_running_master_are_fails(self):
        self.assertEqual(self.preflight(ps="  9 python3 /opt/ros/noetic/bin/roslaunch warehouse_bringup bringup.launch")["Stray processes"].status, "FAIL")
        r = FakeRunner(all_valid)
        r.master_up = True
        self.assertEqual(self.preflight(runner=r)["ROS master"].status, "FAIL")

    def test_disk_space(self):
        low = lambda p: types.SimpleNamespace(free=0.5e9)
        mid = lambda p: types.SimpleNamespace(free=3e9)
        self.assertEqual(self.preflight(disk=low)["WSL disk"].status, "FAIL")
        self.assertEqual(self.preflight(disk=mid)["WSL disk"].status, "WARN")
        self.assertEqual(self.preflight(free_c="1000000000")["Windows C: disk"].status, "FAIL")

    def test_when_powershell_is_not_reachable_the_power_checks_warn_but_do_not_fail(self):
        checks = self.preflight(fail_power=True)
        for name in ("Windows C: disk", "Plugged in", "Sleep on AC"):
            self.assertEqual(checks[name].status, "WARN", name)

    def test_scenario_problems_and_unchecked_layouts(self):
        checks = self.preflight(scenario_check=lambda: ["scenarios.json is stale"])
        self.assertEqual(checks["Scenario files"].status, "FAIL")
        cfg = make_config(unchecked_layouts=["S-06"])
        self.assertEqual(self.preflight(cfg=cfg)["Layout S-06"].status, "WARN")

    def test_format(self):
        text = ab_matrix.format_preflight(list(self.preflight().values()))
        self.assertIn("=== Preflight checklist ===", text)
        self.assertIn("Result:", text)

    def test_the_real_scenario_files_pass_their_check(self):
        support_spec = ab_run.load_scenario_spec()
        self.assertEqual(ab_matrix.scenario_problems(support_spec), [])


class MatrixRunTest(unittest.TestCase):
    def test_every_cell_runs_once_with_the_seeded_arguments_and_the_report_is_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            m, r = make_matrix(all_valid, tmp)
            self.assertEqual(m.run(), 0)
            self.assertEqual([(c["scenario"], c["planner"]) for c in r.calls],
                             [("S-04", "astar"), ("S-04", "dstar"), ("S-06", "dstar"), ("S-06", "astar")])
            c = r.calls[0]["cmd"]
            self.assertEqual(c[c.index("--seed-base") + 1], "1000")
            self.assertEqual(c[c.index("--first-index") + 1], "1")
            self.assertEqual(c[c.index("--repeats") + 1], "5")
            self.assertEqual(c[c.index("--timeout") + 1], "90.0")
            self.assertIn("--no-report", c)
            self.assertNotIn("--dry", c)
            self.assertTrue(os.path.exists(os.path.join(tmp, "matrix_report.md")))
            merged = os.path.join(tmp, "data", "S-04", "astar_runs.csv")
            with open(merged, "r", encoding="utf-8", newline="") as f:
                self.assertEqual(len(list(csv.DictReader(f))), 5)
            self.assertEqual(len(r.reports), 2)  # ab_report per scenario
            with open(os.path.join(tmp, "matrix_state.json"), "r", encoding="utf-8") as f:
                self.assertEqual(len(json.load(f)["attempts"]), 4)

    def test_invalid_runs_are_topped_up_with_fresh_indices_and_logged(self):
        def behavior(info):
            if info["attempt"] == 1:
                return [row(1), row(2, "invalid", "settle movement"), row(3), row(4, "invalid", "no worker seen"), row(5)]
            return all_valid(info)
        with tempfile.TemporaryDirectory() as tmp:
            m, r = make_matrix(behavior, tmp, scenarios=["S-04"], planners=["astar"])
            self.assertEqual(m.run(), 0)
            self.assertEqual(len(r.calls), 2)
            second = r.calls[1]["cmd"]
            self.assertEqual(second[second.index("--first-index") + 1], "6")
            self.assertEqual(second[second.index("--repeats") + 1], "2")
            with open(os.path.join(tmp, "invalid_runs.csv"), "r", encoding="utf-8", newline="") as f:
                log = list(csv.DictReader(f))
            self.assertEqual([(x["run_index"], x["seed"], x["reason"]) for x in log],
                             [("2", "1002", "settle movement"), ("4", "1004", "no worker seen")])
            self.assertIn("2 invalid run(s)", open(os.path.join(tmp, "matrix_report.md"), encoding="utf-8").read())

    def test_a_cell_that_never_reaches_n_valid_runs_gives_up_after_max_attempts_and_the_exit_code_says_so(self):
        always_bad = lambda info: [row(i, "invalid", "pose off start") for i in range(info["first_index"], info["first_index"] + info["repeats"])]
        with tempfile.TemporaryDirectory() as tmp:
            m, r = make_matrix(always_bad, tmp, scenarios=["S-04"], planners=["astar"], repeats=3)
            self.assertEqual(m.run(), 2)
            self.assertEqual(len(r.calls), 3)
            self.assertEqual([c["first_index"] for c in r.calls], [1, 4, 7])
            report = open(os.path.join(tmp, "matrix_report.md"), encoding="utf-8").read()
            self.assertIn("Incomplete cells", report)
            self.assertIn("S-04/astar (0 of 3)", report)

    def test_a_hanging_attempt_is_stopped_at_the_cell_timeout_and_logged(self):
        with tempfile.TemporaryDirectory() as tmp:
            m, r = make_matrix(lambda info: None, tmp, scenarios=["S-04"], planners=["astar"], repeats=2,
                               cell_timeout_s=600.0, max_attempts=1)
            self.assertEqual(m.run(), 2)
            self.assertEqual(r.killed[0][1], signal.SIGINT)
            self.assertGreaterEqual(r.now - 1000.0, 600.0)
            with open(os.path.join(tmp, "invalid_runs.csv"), "r", encoding="utf-8", newline="") as f:
                reasons = [x["reason"] for x in csv.DictReader(f)]
            self.assertTrue(any("timed out" in x or "timeout" in x for x in reasons), reasons)
            state = json.load(open(os.path.join(tmp, "matrix_state.json"), encoding="utf-8"))
            self.assertTrue(state["attempts"]["S-04|astar"][0]["timed_out"])

    def test_a_master_that_survives_a_timeout_aborts_the_matrix(self):
        with tempfile.TemporaryDirectory() as tmp:
            m, r = make_matrix(lambda info: None, tmp, scenarios=["S-04", "S-06"], planners=["astar"], repeats=2, cell_timeout_s=60.0)
            r.master_up = True
            self.assertEqual(m.run(), 1)
            self.assertEqual(len(r.calls), 1)
            self.assertIn("still up", m.state["aborted"])

    def test_two_failed_stack_starts_in_a_row_abort_the_matrix(self):
        with tempfile.TemporaryDirectory() as tmp:
            m, r = make_matrix(lambda info: [], tmp, scenarios=["S-04", "S-06", "S-08"], planners=["astar"], max_attempts=1)
            self.assertEqual(m.run(), 1)
            self.assertEqual(len(r.calls), 2)
            self.assertIn("Play mode", m.state["aborted"])
            self.assertIn("The matrix stopped early", open(os.path.join(tmp, "matrix_report.md"), encoding="utf-8").read())

    def test_resume_skips_complete_cells_and_continues_the_rest(self):
        with tempfile.TemporaryDirectory() as tmp:
            holder = {}

            def interrupt_on_second_cell(info):
                if info["n_calls"] == 1:
                    holder["m"].interrupted = True

            def behavior(info):
                return all_valid(info) if info["n_calls"] == 0 else None

            m, r = make_matrix(behavior, tmp, scenarios=["S-04", "S-06"], planners=["astar"], repeats=2)
            holder["m"] = m
            r.on_popen = interrupt_on_second_cell
            self.assertEqual(m.run(), 1)
            self.assertEqual(len(r.calls), 2)

            m2, r2 = make_matrix(all_valid, tmp, scenarios=["S-04", "S-06"], planners=["astar"], repeats=2)
            self.assertEqual(m2.run(), 0)
            self.assertEqual([c["scenario"] for c in r2.calls], ["S-06"])  # S-04 was complete
            self.assertEqual(r2.calls[0]["attempt"], 2)
            self.assertTrue(r2.calls[0]["out"].endswith("attempt2"))

    def test_dry_mode_passes_dry_runs_one_attempt_and_makes_no_ab_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            m, r = make_matrix(all_valid, tmp, scenarios=["S-04"], planners=["astar"], dry=True, repeats=3, max_attempts=1)
            self.assertEqual(m.run(), 0)
            self.assertIn("--dry", r.calls[0]["cmd"])
            self.assertEqual(r.reports, [])


class ReportTest(unittest.TestCase):
    def rows_for(self, **kw):
        return [row(i, **kw) for i in range(1, 6)]

    def build(self, cell_rows, cfg=None, state=None, invalid=()):
        cfg = cfg or make_config(scenarios=["S-00c", "S-04"], planners=["astar", "dstar"])
        return ab_matrix.build_report(cfg, cell_rows, state or {"attempts": {}}, list(invalid))

    def test_summary_numbers(self):
        rows = [row(1, collisions=1, collisions_npc=1, npc_min_gap_m=0.05), row(2, "timeout"), row(3, npc_min_gap_m=0.4), row(4, "invalid", "x")]
        s = ab_matrix.summarize_cell(rows, 5)
        self.assertEqual((s["valid"], s["invalid"], s["succeeded"], s["timeouts"]), (3, 1, 2, 1))
        self.assertAlmostEqual(s["success_rate"], 2 / 3)
        self.assertEqual((s["collisions"], s["collisions_npc"], s["runs_with_npc_collision"]), (1, 1, 1))
        self.assertAlmostEqual(s["gap_min"], 0.05)
        self.assertAlmostEqual(s["gap_below_share"], 1 / 3)

    def test_flags_follow_the_draft_rules(self):
        clean = ab_matrix.summarize_cell(self.rows_for(), 5)
        self.assertEqual(ab_matrix.draft_flags("S-04", "astar", clean, 40.0), [])
        bad = ab_matrix.summarize_cell([row(1, collisions_npc=1, collisions=1), row(2, "timeout"), row(3, npc_min_gap_m=0.05, sim_time_to_goal_s=90.0)], 5)
        text = "; ".join(ab_matrix.draft_flags("S-04", "astar", bad, 40.0))
        for part in ("success", "worker collision", "worker gap", "median time", "timeout"):
            self.assertIn(part, text)
        self.assertEqual(ab_matrix.draft_flags("S-04", "astar", ab_matrix.summarize_cell([], 5), None), ["no valid runs"])

    def test_the_report_has_a_row_per_cell_and_says_the_thresholds_are_a_draft(self):
        cell_rows = {"S-00c|astar": self.rows_for(), "S-00c|dstar": self.rows_for(), "S-04|astar": self.rows_for(collisions_npc=1, collisions=1),
                     "S-04|dstar": self.rows_for()}
        text = self.build(cell_rows)
        self.assertEqual(text.count("| S-04 |"), 2)
        self.assertIn("NOT approved", text)
        self.assertIn("**S-04 / astar:**", text)
        self.assertNotIn("**S-04 / dstar:**", text)

    def test_banners_for_incomplete_cells_unchecked_layouts_and_a_dry_run(self):
        cfg = make_config(scenarios=["S-04"], planners=["astar"], unchecked_layouts=["S-04"], dry=True)
        text = self.build({"S-04|astar": self.rows_for()[:2]}, cfg=cfg)
        self.assertIn("Incomplete cells", text)
        self.assertIn("Layout not checked in Unity (U3):** S-04", text)
        self.assertIn("DRY RUN", text)

    def test_invalid_runs_are_summarised_by_reason(self):
        text = self.build({}, invalid=[{"reason": "no worker seen"}, {"reason": "no worker seen"}, {"reason": "settle movement"}])
        self.assertIn("3 invalid run(s) or failed attempt(s)", text)
        self.assertIn("no worker seen: 2", text)


class SourceRulesTest(unittest.TestCase):
    def test_no_killall_or_pkill_anywhere_in_the_runner(self):
        with open(os.path.join(SCRIPTS, "ab_matrix.py"), "r", encoding="utf-8") as f:
            tree = ast.parse(f.read())
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                self.assertNotIn(node.value.strip(), ("killall", "pkill", "taskkill"))

    def test_the_launcher_starts_the_runner_and_kills_nothing(self):
        with open(os.path.join(REPO, "start_matrix.bat"), "r", encoding="utf-8") as f:
            bat = f.read().lower()
        # the echo lines only warn about stop_ros.bat; what counts is what the file runs
        commands = " ".join(l for l in bat.splitlines() if not l.strip().startswith(("echo", "rem", "title")))
        self.assertIn("ab_matrix.py", commands)
        self.assertIn("%*", commands)
        for word in ("killall", "pkill", "taskkill", "stop_ros", "start_ros"):
            self.assertNotIn(word, commands)
        self.assertIn("ubuntu-20.04", commands)


if __name__ == "__main__":
    unittest.main()
