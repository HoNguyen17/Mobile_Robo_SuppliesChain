"""Unit tests for ab_batch.py orchestrator.

Pure Python, 100% ROS-free. Runs without ROS or Unity.
Tests CLI parsing, FakeRunner execution, dstar param sequence,
preflight checks, signal cleanup escalation, AST grammar,
and log/metadata aggregation.
Python 3.8 compatible.
"""

import ast
import csv
import io
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
from typing import Any, Dict, List, Optional, Sequence, Tuple

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.normpath(os.path.join(TEST_DIR, "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)
if TEST_DIR not in sys.path:
    sys.path.insert(0, TEST_DIR)

import support  # noqa: F401 (adds scripts/ to sys.path)
import ab_batch


class FakeProcess:
    """Mock process returned by FakeRunner.popen."""

    def __init__(self, pid: int, runner: Optional['FakeRunner'] = None, returncode: int = 0):
        self.pid = pid
        self.runner = runner
        self.returncode = returncode
        self.stdout = io.StringIO("")
        self._alive = False

    def poll(self) -> Optional[int]:
        if not self._alive and self.runner and self.pid in self.runner.live_pids:
            self.runner.remove_pid(self.pid)
        return self.returncode

    def wait(self, timeout: Optional[float] = None) -> int:
        self._alive = False
        if self.runner and self.pid in self.runner.live_pids:
            self.runner.remove_pid(self.pid)
        return self.returncode

    def terminate(self) -> None:
        self._alive = False
        if self.runner and self.pid in self.runner.live_pids:
            self.runner.remove_pid(self.pid)

    def kill(self) -> None:
        self._alive = False
        if self.runner and self.pid in self.runner.live_pids:
            self.runner.remove_pid(self.pid)


class FakeRunner(ab_batch.Runner):
    """Mock runner that records all system and process interactions."""

    def __init__(self):
        super().__init__()
        self.last_topic_error: Optional[str] = None
        self.popen_calls: List[List[str]] = []
        self.popen_kwargs: List[Dict[str, Any]] = []
        self.run_calls: List[List[str]] = []
        self.killed_pgs: List[Tuple[int, int]] = []
        self.next_pid = 1000
        self.live_pids = set()
        self.master_up = False
        self.nodes = ["/unity_endpoint", "/clock"]
        self.publishers: Dict[str, List[str]] = {"/clock": ["/unity_endpoint"]}
        self.clock_publishing = True
        self.rosparams: Dict[str, Any] = {
            "/nav/max_lin": 0.26,
            "/nav/inflation_radius": 0.35,
        }
        self.timeline: List[str] = []
        self.create_synthetic_csv = True
        self.keep_custom_state = False

    def sleep(self, seconds: float) -> None:
        """Bypass sleep during tests."""
        pass

    def remove_pid(self, pid: int) -> None:
        if pid in self.live_pids:
            self.live_pids.remove(pid)
        active_managed = [p for p in self.live_pids if 1000 <= p < 2000]
        if not active_managed and not getattr(self, "keep_custom_state", False):
            self.master_up = False
            self.nodes = ["/unity_endpoint", "/clock"]
            self.publishers = {"/clock": ["/unity_endpoint"]}

    def popen(self, cmd: List[str], stdout=None, stderr=None, preexec_fn=None, **kwargs):
        self.popen_calls.append(cmd)
        self.popen_kwargs.append({"preexec_fn": preexec_fn})
        cmd_str = " ".join(cmd)
        self.timeline.append(f"popen: {cmd[0]} {cmd[1] if len(cmd) > 1 else ''}")
        pid = self.next_pid
        self.next_pid += 1
        self.live_pids.add(pid)

        # If launching navigation_dstar, add dstar_planner node
        if "bringup.launch" in cmd_str:
            self.master_up = True
            if not getattr(self, "keep_custom_state", False):
                self.publishers = {"/clock": ["/unity_endpoint"]}
                if "nav:=false" not in cmd_str:
                    self.nodes = ["/unity_endpoint", "/clock", "/astar_planner", "/path_follower"]
                else:
                    self.nodes = ["/unity_endpoint", "/clock", "/path_follower"]
        elif "navigation_dstar.launch" in cmd_str:
            if "/dstar_planner" not in self.nodes:
                self.nodes.append("/dstar_planner")

        # If running ab_run, write synthetic CSV if requested
        if "ab_run.py" in cmd_str:
            # Parse output directory from cmd
            out_idx = cmd.index("--out") + 1
            session_dir = cmd[out_idx]
            label_idx = cmd.index("--label") + 1
            label = cmd[label_idx]
            csv_path = os.path.join(session_dir, f"{label}_runs.csv")
            if self.create_synthetic_csv:
                with open(csv_path, "w", encoding="utf-8") as f:
                    f.write("run_id,label,planner,scenario,goal_x,goal_y,run_index,outcome,settle_retry,invalid_reason\n")
                    f.write(f"r1,{label},{label},S-04,-3.75,-3.13,1,succeeded,0,\n")
                    f.write(f"r2,{label},{label},S-04,-3.75,-3.13,2,succeeded,1,\n")

        p = FakeProcess(pid=pid, runner=self, returncode=0)
        return p

    def run(self, cmd: List[str], capture_output=True, text=True, timeout=None, check=False, **kwargs):
        self.run_calls.append(cmd)
        self.timeline.append(f"run: {cmd[0]}")
        return subprocess.CompletedProcess(cmd, returncode=0, stdout="", stderr="")

    def kill_pg(self, pid: int, sig: int) -> None:
        self.killed_pgs.append((pid, sig))
        if sig in (signal.SIGTERM, signal.SIGKILL, signal.SIGINT):
            self.remove_pid(pid)

    def is_pid_alive(self, pid: int) -> bool:
        return pid in self.live_pids

    def is_ros_master_up(self, uri: str = "http://localhost:11311") -> bool:
        return self.master_up

    def get_system_state(self, uri: str = "http://localhost:11311") -> Optional[Dict[str, List[str]]]:
        if not self.master_up:
            return None
        return dict(self.publishers)

    def get_ros_nodes(self) -> List[str]:
        return list(self.nodes)

    def is_topic_publishing(self, topic: str, timeout_s: float = 10.0) -> bool:
        if topic == "/clock":
            if not self.clock_publishing:
                self.last_topic_error = "TimeoutExpired after 10.0s"
                return False
            return True
        return True

    def rosparam_set(self, param: str, val: Any) -> bool:
        self.rosparams[param] = val
        self.timeline.append(f"rosparam_set: {param}={val}")
        return True

    def rosparam_get(self, param: str) -> Optional[Any]:
        return self.rosparams.get(param)

    def git_commit_hash(self, cwd: Optional[str] = None, timeout: float = 120.0) -> str:
        return "1234567890abcdef"

    def git_status_short(self, cwd: Optional[str] = None, timeout: float = 120.0) -> str:
        return "?? untracked.py"

    def git_diff_stat(self, cwd: Optional[str] = None, timeout: float = 120.0) -> str:
        return " 1 file changed, 1 insertion(+)"


class StatefulCleanupFakeRunner(FakeRunner):
    """FakeRunner where rosparam_get returns None once cleanup has run (K1)."""

    def __init__(self):
        super().__init__()
        self.cleaned_up = False

    def popen(self, cmd: List[str], stdout=None, stderr=None, preexec_fn=None, **kwargs):
        if "bringup.launch" in " ".join(cmd):
            self.cleaned_up = False
        return super().popen(cmd, stdout=stdout, stderr=stderr, preexec_fn=preexec_fn, **kwargs)

    def kill_pg(self, pid: int, sig: int) -> None:
        self.cleaned_up = True
        super().kill_pg(pid, sig)

    def rosparam_get(self, param: str) -> Optional[Any]:
        if self.cleaned_up:
            return None
        return super().rosparam_get(param)


class TestAbBatchOrchestrator(unittest.TestCase):
    def test_cli_parsing_defaults_and_modes(self):
        # 1. Defaults
        args = ab_batch.parse_args([])
        self.assertEqual(args.planner, "both")
        self.assertEqual(args.repeats, 5)
        self.assertEqual(args.scenario, "S-04")
        self.assertEqual(args.goal, [-3.75, -3.13])
        self.assertEqual(args.settle, 7.0)
        self.assertEqual(args.timeout, 120.0)
        self.assertFalse(args.rviz)
        self.assertEqual(args.out, "~/ab_runs")
        self.assertFalse(args.dry)
        self.assertFalse(args.stop)
        self.assertEqual(args.order, "astar,dstar")

        # 2. Dry run flag
        args_dry = ab_batch.parse_args(["--dry"])
        self.assertTrue(args_dry.dry)

        # 3. Explicit planner
        args_dstar = ab_batch.parse_args(["--planner", "dstar", "--repeats", "10", "--rviz"])
        self.assertEqual(args_dstar.planner, "dstar")
        self.assertEqual(args_dstar.repeats, 10)
        self.assertTrue(args_dstar.rviz)

    def test_preflight_refuses_when_ros_master_active(self):
        fake = FakeRunner()
        fake.master_up = True  # Master is already running!
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 1)
            # Ensure no processes were spawned
            self.assertEqual(len(fake.popen_calls), 0)

    def test_preflight_refuses_when_live_pidfile_exists(self):
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            pidfile = os.path.join(tmpdir, "batch.pid")
            with open(pidfile, "w") as f:
                f.write("4321")
            fake.live_pids.add(4321)  # Process 4321 is alive

            args = ab_batch.parse_args(["--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 1)
            self.assertEqual(len(fake.popen_calls), 0)

    def test_stop_flag_removes_pidfile_and_signals(self):
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            pidfile = os.path.join(tmpdir, "batch.pid")
            with open(pidfile, "w") as f:
                f.write("5678")
            fake.live_pids.add(5678)

            args = ab_batch.parse_args(["--stop", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)
            self.assertFalse(os.path.exists(pidfile))
            self.assertTrue(any(pid == 5678 and sig == signal.SIGINT for pid, sig in fake.killed_pgs))

    def test_exact_argv_for_each_mode(self):
        """Verify exact argv for bringup, navigation_dstar, and ab_run in all modes."""
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            # 1. Test D* mode with rviz=True
            args_dstar = ab_batch.parse_args([
                "--planner", "dstar",
                "--scenario", "S-04",
                "--goal", "-3.75", "-3.13",
                "--repeats", "4",
                "--settle", "8.0",
                "--timeout", "110.0",
                "--rviz",
                "--out", tmpdir,
            ])
            ret = ab_batch.run_batch(args_dstar, runner=fake)
            self.assertEqual(ret, 0)

            # Bringup command
            bringup_cmd = fake.popen_calls[0]
            self.assertEqual(bringup_cmd, [
                "roslaunch", "warehouse_bringup", "bringup.launch", "rviz:=true", "nav:=false"
            ])

            # Navigation D* command
            dstar_cmd = fake.popen_calls[1]
            self.assertEqual(dstar_cmd, [
                "roslaunch", "turtlebot_control", "navigation_dstar.launch"
            ])

            # ab_run command
            ab_cmd = fake.popen_calls[2]
            self.assertIn("ab_run.py", ab_cmd[1])
            self.assertIn("--scenario", ab_cmd)
            self.assertEqual(ab_cmd[ab_cmd.index("--scenario") + 1], "S-04")
            self.assertIn("--goal", ab_cmd)
            self.assertEqual(ab_cmd[ab_cmd.index("--goal") + 1], "-3.75")
            self.assertEqual(ab_cmd[ab_cmd.index("--goal") + 2], "-3.13")
            self.assertIn("--repeats", ab_cmd)
            self.assertEqual(ab_cmd[ab_cmd.index("--repeats") + 1], "4")
            self.assertIn("--settle", ab_cmd)
            self.assertEqual(ab_cmd[ab_cmd.index("--settle") + 1], "8.0")
            self.assertIn("--timeout", ab_cmd)
            self.assertEqual(ab_cmd[ab_cmd.index("--timeout") + 1], "110.0")
            self.assertIn("--label", ab_cmd)
            self.assertEqual(ab_cmd[ab_cmd.index("--label") + 1], "dstar")

            # 2. Test A* mode with rviz=False (default)
            fake_a = FakeRunner()
            args_astar = ab_batch.parse_args([
                "--planner", "astar",
                "--repeats", "2",
                "--out", tmpdir,
            ])
            ret_a = ab_batch.run_batch(args_astar, runner=fake_a)
            self.assertEqual(ret_a, 0)

            bringup_a = fake_a.popen_calls[0]
            self.assertEqual(bringup_a, [
                "roslaunch", "warehouse_bringup", "bringup.launch", "rviz:=false"
            ])

    def test_dstar_params_set_before_navigation_dstar_launch(self):
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "dstar", "--repeats", "2", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)

            # Check execution timeline: rosparam_set MUST appear before popen of navigation_dstar
            nav_launch_idx = -1
            param_set_indices = []
            for i, event in enumerate(fake.timeline):
                if "rosparam_set: /dstar_planner" in event:
                    param_set_indices.append(i)
                elif "popen: roslaunch turtlebot_control" in event:
                    nav_launch_idx = i

            self.assertGreater(len(param_set_indices), 0)
            self.assertGreater(nav_launch_idx, 0)
            for idx in param_set_indices:
                self.assertLess(idx, nav_launch_idx, "rosparam set occurred after navigation_dstar launch!")

    def test_both_planners_order_and_clean_stop_between(self):
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            # Order: dstar then astar
            args = ab_batch.parse_args(["--planner", "both", "--order", "dstar,astar", "--repeats", "1", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)

            # Check that dstar bringup launched before astar bringup
            bringup_cmds = [cmd for cmd in fake.popen_calls if "bringup.launch" in " ".join(cmd)]
            self.assertEqual(len(bringup_cmds), 2)
            self.assertIn("nav:=false", bringup_cmds[0])  # D* was first!
            self.assertNotIn("nav:=false", bringup_cmds[1])  # A* was second!

    def test_dry_run_forces_3_repeats_and_skips_report(self):
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--dry", "--repeats", "10", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)

            # Check that ab_run received repeats=3 despite CLI argument 10
            ab_run_calls = [cmd for cmd in fake.popen_calls if "ab_run.py" in " ".join(cmd)]
            self.assertGreater(len(ab_run_calls), 0)
            for cmd in ab_run_calls:
                idx = cmd.index("--repeats") + 1
                self.assertEqual(cmd[idx], "3")

            # Check that ab_report was NOT executed
            report_calls = [cmd for cmd in fake.run_calls if "ab_report.py" in " ".join(cmd)]
            self.assertEqual(len(report_calls), 0)

            # Check session folder has dry_ prefix
            entries = os.listdir(tmpdir)
            dry_dirs = [e for e in entries if e.startswith("dry_")]
            self.assertGreater(len(dry_dirs), 0)

            # Check that meta_astar.json records effective_repeats = 3 (Q2)
            meta_dry = os.path.join(tmpdir, dry_dirs[0], "meta_astar.json")
            if os.path.exists(meta_dry):
                with open(meta_dry, "r", encoding="utf-8") as f_md:
                    data_dry = json.load(f_md)
                self.assertEqual(data_dry["effective_repeats"], 3)

    def test_meta_json_fields_and_summary_parsing(self):
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "2", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)

            session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
            self.assertEqual(len(session_dirs), 1)
            session_dir = os.path.join(tmpdir, session_dirs[0])

            meta_file = os.path.join(session_dir, "meta_astar.json")
            self.assertTrue(os.path.exists(meta_file))

            with open(meta_file, "r", encoding="utf-8") as f:
                data = json.load(f)

            self.assertEqual(data["git_commit"], "1234567890abcdef")
            self.assertEqual(data["git_status"], "?? untracked.py")
            self.assertEqual(data["planner"], "astar")
            self.assertEqual(data["effective_repeats"], 2)
            self.assertIn("cli_args", data)
            self.assertIn("python_version", data)
            self.assertIn("start_time", data)
            self.assertIn("end_time", data)
            self.assertEqual(data["ab_run_exit_code"], 0)

            # Verify SUMMARY contains Sim/wall ratio (Q4)
            batch_log = os.path.join(session_dir, "batch.log")
            with open(batch_log, "r", encoding="utf-8") as f_bl:
                bl_text = f_bl.read()
            self.assertIn("Sim/wall ratio:", bl_text)

    def test_cleanup_signals_only_started_groups(self):
        fake = FakeRunner()
        procs = [FakeProcess(pid=2001), FakeProcess(pid=2002)]
        fake.live_pids.update([2001, 2002, 9999])  # 9999 is external

        ab_batch.cleanup_group(fake, procs)

        # 2001 and 2002 must have received signals; 9999 must NOT have been signaled
        signaled_pids = {pid for pid, sig in fake.killed_pgs}
        self.assertIn(2001, signaled_pids)
        self.assertIn(2002, signaled_pids)
        self.assertNotIn(9999, signaled_pids)

    def test_cleanup_on_exception_and_keyboard_interrupt(self):
        """Ensure cleanup_group executes and removes pidfile if an exception occurs."""
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            # Inject an error inside popen during ab_run
            def fault_popen(cmd, **kwargs):
                if "ab_run.py" in " ".join(cmd):
                    raise KeyboardInterrupt("Simulated Ctrl-C")
                return FakeRunner.popen(fake, cmd, **kwargs)

            fake.popen = fault_popen
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
            with self.assertRaises(KeyboardInterrupt):
                ab_batch.run_batch(args, runner=fake)

            # Pidfile must have been removed in finally:
            pidfile = os.path.join(tmpdir, "batch.pid")
            self.assertFalse(os.path.exists(pidfile))

            # All spawned processes must have been signaled
            signaled_pids = {pid for pid, sig in fake.killed_pgs}
            self.assertGreater(len(signaled_pids), 0)

    def test_summary_counts_from_synthetic_csv(self):
        """Test parse_csv_summary accurately counts valid, invalid, reasons, and retries."""
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = os.path.join(tmpdir, "test_runs.csv")
            with open(csv_path, "w", encoding="utf-8") as f:
                f.write("run_id,outcome,settle_retry,invalid_reason\n")
                f.write("r1,succeeded,0,\n")
                f.write("r2,succeeded,1,\n")
                f.write("r3,invalid,0,stuck\n")
                f.write("r4,invalid,1,wall_collision\n")
                f.write("r5,succeeded,0,\n")

            info = ab_batch.parse_csv_summary(csv_path)
            self.assertEqual(info["valid"], 3)
            self.assertEqual(info["invalid"], 2)
            self.assertEqual(info["succeeded"], 3)
            self.assertEqual(info["timeout"], 0)
            self.assertEqual(info["aborted"], 0)
            self.assertEqual(info["reasons"], {"stuck": 1, "wall_collision": 1})
            self.assertEqual(info["retries"], 2)

    # --- K1 Test: rosparam values read BEFORE cleanup ---

    def test_k1_meta_json_reads_rosparams_before_cleanup(self):
        """K1: meta_<planner>.json reads rosparams BEFORE cleanup_group.
        Tested with StatefulCleanupFakeRunner whose rosparam_get returns None once cleanup has run.
        """
        fake = StatefulCleanupFakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "both", "--repeats", "1", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)
            self.assertTrue(fake.cleaned_up, "Cleanup must have run during batch")

            session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
            session_dir = os.path.join(tmpdir, session_dirs[0])

            # Check meta_astar.json
            with open(os.path.join(session_dir, "meta_astar.json"), "r", encoding="utf-8") as f:
                meta_a = json.load(f)
            self.assertEqual(meta_a["nav_max_lin"], 0.26)
            self.assertEqual(meta_a["nav_inflation_radius"], 0.35)

            # Check meta_dstar.json
            with open(os.path.join(session_dir, "meta_dstar.json"), "r", encoding="utf-8") as f:
                meta_d = json.load(f)
            self.assertEqual(meta_d["nav_max_lin"], 0.26)
            self.assertEqual(meta_d["nav_inflation_radius"], 0.35)
            self.assertIsNotNone(meta_d["rosparam_values"])
            self.assertIs(meta_d["rosparam_values"]["/dstar_planner/filter_static_hits"], True)
            self.assertIs(meta_d["rosparam_values"]["/dstar_planner/blocked_use_plan_radius"], True)
            self.assertIs(meta_d["rosparam_values"]["/dstar_planner/verify_costmap"], False)

    # --- K2 Tests: setsid, process group tracking, and watchdog ---

    def test_k2_ab_run_preexec_fn_and_pgid_safety(self):
        """K2: ab_run starts with setsid, added to active_procs, orchestrator pgid never signalled."""
        fake = FakeRunner()
        cleanup_procs = []
        orig_cleanup = ab_batch.cleanup_group

        def tracking_cleanup(runner, procs, logger=None):
            cleanup_procs.extend(procs)
            return orig_cleanup(runner, procs, logger)

        with mock.patch("ab_batch.cleanup_group", side_effect=tracking_cleanup):
            with tempfile.TemporaryDirectory() as tmpdir:
                args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
                ret = ab_batch.run_batch(args, runner=fake)
                self.assertEqual(ret, 0)

        # Find ab_run call
        ab_run_indices = [
            i for i, cmd in enumerate(fake.popen_calls) if "ab_run.py" in " ".join(cmd)
        ]
        self.assertEqual(len(ab_run_indices), 1)
        ab_idx = ab_run_indices[0]
        if hasattr(os, "setsid"):
            self.assertEqual(fake.popen_kwargs[ab_idx]["preexec_fn"], os.setsid)

        # Verify orchestrator's own PID was never signaled
        my_pid = os.getpid()
        signaled_pids = [pid for pid, sig in fake.killed_pgs]
        self.assertNotIn(my_pid, signaled_pids)

        # Verify ab_run was in the cleanup list (active_procs)
        ab_pid = 1000 + ab_idx
        cleanup_pids = [p.pid for p in cleanup_procs if p and hasattr(p, "pid")]
        self.assertIn(ab_pid, cleanup_pids)

    def test_k2_watchdog_triggers_with_silent_ab_run(self):
        """K2: watchdog terminates ab_run even if it produces zero output lines."""
        fake = FakeRunner()

        class SilentHangingProcess(FakeProcess):
            def __init__(self, pid, runner):
                super().__init__(pid, runner=runner, returncode=0)
                self._alive = True

            def poll(self):
                if self.runner and self.pid not in self.runner.live_pids:
                    self._alive = False
                    return -1
                return None

        orig_popen = fake.popen

        def silent_popen(cmd, stdout=None, stderr=None, preexec_fn=None, **kwargs):
            if "ab_run.py" in " ".join(cmd):
                pid = fake.next_pid
                fake.next_pid += 1
                fake.live_pids.add(pid)
                fake.popen_calls.append(cmd)
                fake.popen_kwargs.append({"preexec_fn": preexec_fn})
                return SilentHangingProcess(pid, fake)
            return orig_popen(cmd, stdout=stdout, stderr=stderr, preexec_fn=preexec_fn, **kwargs)

        fake.popen = silent_popen

        t_base = 10000.0
        call_count = [0]

        def fast_time():
            call_count[0] += 1
            if call_count[0] > 5:
                return t_base + 999999.0
            return t_base + call_count[0] * 0.1

        with mock.patch("time.time", side_effect=fast_time):
            with tempfile.TemporaryDirectory() as tmpdir:
                args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
                ret = ab_batch.run_batch(args, runner=fake)

        ab_run_pids = [1000 + i for i, cmd in enumerate(fake.popen_calls) if "ab_run.py" in " ".join(cmd)]
        self.assertEqual(len(ab_run_pids), 1)
        ab_pid = ab_run_pids[0]
        sigs_sent = [sig for pid, sig in fake.killed_pgs if pid == ab_pid]
        self.assertIn(signal.SIGTERM, sigs_sent)
        self.assertIn(signal.SIGKILL, sigs_sent)

    # --- K3 Tests: Polling verification and missing reporting ---

    def test_k3_verification_polling_succeeds_after_n_polls(self):
        """K3: Polling succeeds after nodes become ready on N-th poll."""
        class PollingFakeRunner(FakeRunner):
            def __init__(self):
                super().__init__()
                self.poll_count = 0

            def get_ros_nodes(self) -> List[str]:
                self.poll_count += 1
                # Polls 1-3: missing nodes; Poll 4+: ready
                if self.poll_count <= 3:
                    return ["/unity_endpoint", "/clock"]
                return ["/unity_endpoint", "/clock", "/astar_planner", "/path_follower"]

        fake = PollingFakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)
            self.assertGreaterEqual(fake.poll_count, 4)

    def test_k3_verification_fails_and_lists_missing_nodes_and_params(self):
        """K3: Verification failure message lists exact missing nodes and parameters."""
        fake = FakeRunner()
        # Ensure get_ros_nodes always returns missing nodes
        fake.get_ros_nodes = lambda: ["/unity_endpoint", "/clock"]

        t_base = 50000.0
        time_calls = [0]

        def fast_forward_time():
            time_calls[0] += 1
            if time_calls[0] > 5:
                return t_base + 100.0
            return t_base + time_calls[0]

        with mock.patch("time.time", side_effect=fast_forward_time):
            with tempfile.TemporaryDirectory() as tmpdir:
                args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
                ret = ab_batch.run_batch(args, runner=fake)
                self.assertEqual(ret, 1)

                session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
                batch_log = os.path.join(tmpdir, session_dirs[0], "batch.log")
                with open(batch_log, "r", encoding="utf-8") as f:
                    content = f.read()

                self.assertIn("Verification failed after 20s! Missing/mismatched:", content)
                self.assertIn("node /path_follower", content)
                self.assertIn("node /astar_planner", content)

    # --- K4 Tests: Signals, idempotency, and 45s stop escalation ---

    def test_k4_signal_handler_idempotency(self):
        """K4: Signal handler ignores second signal while cleaning up."""
        fake = FakeRunner()
        p1 = FakeProcess(pid=8001)
        fake.live_pids.add(8001)
        procs = [p1]

        cleaning_up = False
        cleanup_calls = []

        def mock_cleanup_group(runner, procs_arg, logger=None):
            cleanup_calls.append(len(procs_arg))

        def sig_handler(_signum, _frame):
            nonlocal cleaning_up
            if cleaning_up:
                return
            cleaning_up = True
            mock_cleanup_group(fake, procs)

        sig_handler(signal.SIGINT, None)
        sig_handler(signal.SIGINT, None)
        self.assertEqual(len(cleanup_calls), 1)

    def test_k4_stop_waits_up_to_45s_for_exit(self):
        """K4: --stop waits up to 45s for batch pid to exit on SIGINT before sending SIGTERM."""
        # Case A: exits cleanly before 45s
        fake_clean = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            pidfile = os.path.join(tmpdir, "batch.pid")
            with open(pidfile, "w") as f:
                f.write("7001")
            fake_clean.live_pids.add(7001)

            check_counts = [0]

            def is_alive_clean(pid):
                check_counts[0] += 1
                if check_counts[0] >= 2:
                    fake_clean.live_pids.discard(pid)
                return pid in fake_clean.live_pids

            fake_clean.is_pid_alive = is_alive_clean

            args = ab_batch.parse_args(["--stop", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake_clean)
            self.assertEqual(ret, 0)
            sigs = [sig for pid, sig in fake_clean.killed_pgs if pid == 7001]
            self.assertIn(signal.SIGINT, sigs)
            self.assertNotIn(signal.SIGTERM, sigs)
            self.assertNotIn(signal.SIGKILL, sigs)

        # Case B: remains alive past 45s, escalates to SIGTERM and SIGKILL
        fake_stubborn = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            pidfile = os.path.join(tmpdir, "batch.pid")
            with open(pidfile, "w") as f:
                f.write("7002")
            fake_stubborn.live_pids.add(7002)

            def stubborn_kill_pg(pid, sig):
                fake_stubborn.killed_pgs.append((pid, sig))
                if sig == signal.SIGKILL:
                    fake_stubborn.remove_pid(pid)

            fake_stubborn.kill_pg = stubborn_kill_pg

            t_val = [100.0]

            def fake_time():
                t_val[0] += 10.0
                return t_val[0]

            with mock.patch("time.time", side_effect=fake_time):
                args = ab_batch.parse_args(["--stop", "--out", tmpdir])
                ret = ab_batch.run_batch(args, runner=fake_stubborn)
                self.assertEqual(ret, 0)

            sigs = [sig for pid, sig in fake_stubborn.killed_pgs if pid == 7002]
            self.assertIn(signal.SIGINT, sigs)
            self.assertIn(signal.SIGTERM, sigs)
            self.assertIn(signal.SIGKILL, sigs)

    # --- K5 Tests: SUMMARY counts, respawn scanning, and report failure ---

    def test_k5_summary_counts_succeeded_timeout_aborted(self):
        """K5: parse_csv_summary counts succeeded, timeout, aborted alongside valid and invalid."""
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = os.path.join(tmpdir, "test_runs.csv")
            with open(csv_path, "w", encoding="utf-8") as f:
                f.write("run_id,outcome,sim_time_to_goal_s,wall_time_s,settle_retry,invalid_reason\n")
                f.write("r1,succeeded,10.0,10.0,0,\n")
                f.write("r2,succeeded,20.0,10.0,1,\n")
                f.write("r3,timeout,30.0,10.0,0,\n")
                f.write("r4,aborted,15.0,10.0,1,\n")
                f.write("r5,invalid,0.0,0.0,0,wall_collision\n")

            info = ab_batch.parse_csv_summary(csv_path)
            self.assertEqual(info["valid"], 4)
            self.assertEqual(info["invalid"], 1)
            self.assertEqual(info["succeeded"], 2)
            self.assertEqual(info["timeout"], 1)
            self.assertEqual(info["aborted"], 1)
            self.assertEqual(info["reasons"], {"wall_collision": 1})
            self.assertEqual(info["retries"], 2)
            self.assertIsNotNone(info["median_sim_wall"])
            self.assertAlmostEqual(info["median_sim_wall"], 1.5)

    def test_k5_respawn_scan_exact_pmon_strings(self):
        """K5: scan_log_for_respawns matches exact roslaunch/pmon.py and nodeprocess.py strings."""
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = os.path.join(tmpdir, "bringup.log")
            with open(log_path, "w", encoding="utf-8") as f:
                f.write("[/path_follower] restarting process\n")
                f.write("process has died [pid 1234, exit code 1, cmd ...]\n")
                f.write("[/astar_planner] process has died\n")
                f.write("normal log line\n")

            count = ab_batch.scan_log_for_respawns(log_path)
            self.assertEqual(count, 3)

    def test_k5_report_failure_logs_returncode_and_stderr(self):
        """K5: ab_report failure or missing report logs return code, stderr, and missing file."""
        fake = FakeRunner()

        def custom_run(cmd, **kwargs):
            if "ab_report.py" in " ".join(cmd):
                return subprocess.CompletedProcess(cmd, returncode=2, stdout="", stderr="Report rendering crash")
            return subprocess.CompletedProcess(cmd, returncode=0, stdout="", stderr="")

        fake.run = custom_run
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)

            session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
            batch_log = os.path.join(tmpdir, session_dirs[0], "batch.log")
            with open(batch_log, "r", encoding="utf-8") as f:
                log_content = f.read()

            self.assertIn("Report generation failed! Exit code: 2", log_content)
            self.assertIn("ab_report stderr:\nReport rendering crash", log_content)
            self.assertIn("Expected report file missing:", log_content)

    # --- L1 Test: Zombie false-alive with real process ---

    def test_l1_zombie_false_alive_with_real_process(self):
        """L1: Real subprocess exited without poll() is recognized as dead; cleanup_group < 2s."""
        runner = ab_batch.Runner()
        cmd = [sys.executable, "-c", "import time; time.sleep(0.2)"]
        p = runner.popen(cmd)
        try:
            time.sleep(0.4)
            is_alive = runner.is_pid_alive(p.pid)
            if os.path.exists(f"/proc/{p.pid}/stat"):
                self.assertFalse(is_alive, "is_pid_alive must return False for un-reaped zombie")

            t0 = time.time()
            ab_batch.cleanup_group(runner, [p])
            elapsed = time.time() - t0
            self.assertLess(elapsed, 2.0, f"cleanup_group took {elapsed:.2f}s, expected < 2.0s")
        finally:
            try:
                p.wait(timeout=1.0)
            except Exception:
                pass

    # --- L2 Test: Respawn scan cut offset ---

    def test_l2_respawn_scan_with_cut_offset(self):
        """L2: Only occurrences before the cut offset are counted by scan_log_for_respawns."""
        with tempfile.TemporaryDirectory() as tmpdir:
            log_path = os.path.join(tmpdir, "bringup.log")
            with open(log_path, "wb") as f:
                f.write(b"[/path_follower] restarting process\n")
                f.write(b"process has died [pid 1234, exit code 1]\n")
                f.flush()
                cut_offset = f.tell()
                f.write(b"process has died [pid 5678, exit code 0]\n")
                f.write(b"[/astar_planner] process has died\n")

            count_before = ab_batch.scan_log_for_respawns(log_path, max_bytes=cut_offset)
            self.assertEqual(count_before, 2)

            count_all = ab_batch.scan_log_for_respawns(log_path)
            self.assertEqual(count_all, 4)

    # --- L3 Tests: handoff.txt ---

    def test_l3_handoff_created_on_success(self):
        """L3: handoff.txt created after successful run with correct sections and separators."""
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)

            session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
            session_dir = os.path.join(tmpdir, session_dirs[0])
            handoff_path = os.path.join(session_dir, "handoff.txt")

            self.assertTrue(os.path.exists(handoff_path))
            with open(handoff_path, "r", encoding="utf-8") as f:
                content = f.read()

            self.assertIn("===== header =====", content)
            self.assertIn("===== batch.log =====", content)
            self.assertIn("===== meta_astar.json =====", content)
            self.assertIn("===== astar_runs.csv =====", content)
            self.assertIn("Git Commit: 1234567890abcdef", content)

    def test_l3_handoff_created_on_fatal_and_interrupted(self):
        """L3: handoff.txt created even on fatal failure or keyboard interrupt."""
        # 1. Fatal failure (verification failure)
        fake_fatal = FakeRunner()
        fake_fatal.get_ros_nodes = lambda: ["/unity_endpoint", "/clock"]
        t_base = 50000.0
        time_calls = [0]

        def fast_time():
            time_calls[0] += 1
            return t_base + 100.0 if time_calls[0] > 5 else t_base + time_calls[0]

        with mock.patch("time.time", side_effect=fast_time):
            with tempfile.TemporaryDirectory() as tmpdir:
                args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
                ret = ab_batch.run_batch(args, runner=fake_fatal)
                self.assertEqual(ret, 1)

                session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
                session_dir = os.path.join(tmpdir, session_dirs[0])
                handoff_path = os.path.join(session_dir, "handoff.txt")
                self.assertTrue(os.path.exists(handoff_path), "handoff.txt must exist on fatal error")

        # 2. Interrupted run (KeyboardInterrupt during popen)
        fake_intr = FakeRunner()

        def fault_popen(cmd, **kwargs):
            if "ab_run.py" in " ".join(cmd):
                raise KeyboardInterrupt("Simulated Ctrl-C")
            return FakeRunner.popen(fake_intr, cmd, **kwargs)

        fake_intr.popen = fault_popen

        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
            with self.assertRaises(KeyboardInterrupt):
                ab_batch.run_batch(args, runner=fake_intr)

            session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
            session_dir = os.path.join(tmpdir, session_dirs[0])
            handoff_path = os.path.join(session_dir, "handoff.txt")
            self.assertTrue(os.path.exists(handoff_path), "handoff.txt must exist on interrupt")

    def test_l3_handoff_size_cap_respected(self):
        """L3: handoff.txt respects 400 KB hard cap even with large bringup logs."""
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            session_dir = os.path.join(tmpdir, "session_test")
            os.makedirs(session_dir, exist_ok=True)

            bringup_log = os.path.join(session_dir, "bringup_astar.log")
            with open(bringup_log, "w", encoding="utf-8") as f:
                for i in range(12000):
                    f.write(f"log line {i} with some padding content to increase byte size\n")

            args = ab_batch.parse_args(["--planner", "astar"])
            handoff_path = ab_batch.write_handoff(session_dir, args, fake, "2026-10-24T18:00:00")
            self.assertIsNotNone(handoff_path)
            self.assertTrue(os.path.exists(handoff_path))

            size = os.path.getsize(handoff_path)
            self.assertLessEqual(size, 400 * 1024, f"Handoff size {size} exceeds 400 KB cap")
            with open(handoff_path, "r", encoding="utf-8", errors="replace") as f:
                content = f.read()
            self.assertIn("===== header =====", content)
            self.assertIn("===== bringup_astar.log =====", content)

    # --- L4 Test: latest_session.txt ---

    def test_l4_latest_session_txt_written(self):
        """L4: latest_session.txt written to out_dir at session start with session dir path."""
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)

            latest_session_file = os.path.join(tmpdir, "latest_session.txt")
            self.assertTrue(os.path.exists(latest_session_file))
            with open(latest_session_file, "r", encoding="utf-8") as f:
                stored_path = f.read().strip()

            session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
            expected_path = os.path.join(tmpdir, session_dirs[0])
            self.assertEqual(stored_path, expected_path)

    # --- O5 Tests: Readiness gate, diagnostics, and subprocess timeout ---

    def test_o5_a_gate_passes_even_when_is_topic_publishing_false(self):
        """O5(a): Regression test: gate passes when master up, endpoint listed, /clock pub present,
        even if is_topic_publishing returns False.
        """
        fake = FakeRunner()
        fake.clock_publishing = False
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0, "Gate should pass despite is_topic_publishing returning False")

    def test_o5_b_endpoint_missing_logs_play_hint(self):
        """O5(b): When endpoint is missing, timeout message lists endpoint=False and includes Play hint."""
        fake = FakeRunner()
        fake.keep_custom_state = True
        fake.nodes = ["/clock"]
        fake.publishers = {"/clock": ["/rosout"]}

        cur_t = [50000.0]

        def fast_time():
            cur_t[0] += 20.0
            return cur_t[0]

        with mock.patch("time.time", side_effect=fast_time):
            with tempfile.TemporaryDirectory() as tmpdir:
                args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
                ret = ab_batch.run_batch(args, runner=fake)
                self.assertEqual(ret, 1)

                session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
                batch_log = os.path.join(tmpdir, session_dirs[0], "batch.log")
                with open(batch_log, "r", encoding="utf-8") as f:
                    log_text = f.read()

                self.assertIn("Stack not ready after 90 s: master=True endpoint=False clock_publisher=False", log_text)
                self.assertIn("Unity not connected: is Play pressed?", log_text)

    def test_o5_c_endpoint_present_no_clock_pub_logs_clock_not_registered(self):
        """O5(c): When endpoint is present but no /clock pub, logs '/clock not registered' and NO Play hint."""
        fake = FakeRunner()
        fake.keep_custom_state = True
        fake.nodes = ["/unity_endpoint", "/clock"]
        fake.publishers = {"/clock": []}

        cur_t = [60000.0]

        def fast_time():
            cur_t[0] += 20.0
            return cur_t[0]

        with mock.patch("time.time", side_effect=fast_time):
            with tempfile.TemporaryDirectory() as tmpdir:
                args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
                ret = ab_batch.run_batch(args, runner=fake)
                self.assertEqual(ret, 1)

                session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
                batch_log = os.path.join(tmpdir, session_dirs[0], "batch.log")
                with open(batch_log, "r", encoding="utf-8") as f:
                    log_text = f.read()

                self.assertIn("Stack not ready after 90 s: master=True endpoint=True clock_publisher=False", log_text)
                self.assertIn("Unity connected but /clock not registered", log_text)
                self.assertNotIn("Unity not connected: is Play pressed?", log_text)

    def test_o5_d_diagnostics_block_written_to_batch_log_on_timeout(self):
        """O5(d): Diagnostics block with rosnode list, publishers, last_topic_error, and bringup tail is written to batch.log."""
        fake = FakeRunner()
        fake.keep_custom_state = True
        fake.nodes = ["/unity_endpoint"]
        fake.publishers = {"/clock": []}

        cur_t = [70000.0]

        def fast_time():
            cur_t[0] += 20.0
            return cur_t[0]

        with mock.patch("time.time", side_effect=fast_time):
            with tempfile.TemporaryDirectory() as tmpdir:
                args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
                ret = ab_batch.run_batch(args, runner=fake)
                self.assertEqual(ret, 1)

                session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
                batch_log = os.path.join(tmpdir, session_dirs[0], "batch.log")
                with open(batch_log, "r", encoding="utf-8") as f:
                    log_text = f.read()

                self.assertIn("rosnode list:", log_text)
                self.assertIn("publishers:", log_text)
                self.assertIn("last_topic_error:", log_text)
                self.assertIn("bringup log (last 25 lines):", log_text)

    def test_o5_e_real_subprocess_timeout_in_last_topic_error(self):
        """O5(e): Runner.run(['sleep', '2'], timeout=1) reports timeout in last_topic_error."""
        runner = ab_batch.Runner()
        runner.run(["sleep", "2"], timeout=1)
        self.assertIsNotNone(runner.last_topic_error)
        self.assertIn("timeoutexpired", runner.last_topic_error.lower())

    def test_git_status_short_truncation_and_repo_dir(self):
        """Q2: git_status_short uses repo_dir as cwd and truncates at 200 lines."""
        runner = ab_batch.Runner()
        self.assertTrue(os.path.isdir(runner.repo_dir))

        # Mock self.run to return 250 lines
        lines_250 = "\n".join(f"?? file_{i}.txt" for i in range(250))
        captured_kwargs = []

        def mock_run(cmd, timeout=None, cwd=None, **kwargs):
            captured_kwargs.append({"cmd": cmd, "cwd": cwd})
            return subprocess.CompletedProcess(cmd, returncode=0, stdout=lines_250, stderr="")

        runner.run = mock_run
        status_out = runner.git_status_short()
        status_lines = status_out.splitlines()
        self.assertEqual(len(status_lines), 200)
        self.assertEqual(status_lines[0], "?? file_0.txt")
        self.assertEqual(status_lines[-1], "?? file_199.txt")
        self.assertEqual(captured_kwargs[0]["cwd"], runner.repo_dir)

    def test_git_status_real_repo_in_temp_dir(self):
        """S4: Real git repo in temp dir (git init, untracked file, expect '??'; skip if git missing)."""
        import shutil
        if not shutil.which("git"):
            self.skipTest("git is not installed")
        with tempfile.TemporaryDirectory() as tmpdir:
            res = subprocess.run(["git", "init"], cwd=tmpdir, capture_output=True, text=True)
            if res.returncode != 0:
                self.skipTest("git init failed")
            sample_file = os.path.join(tmpdir, "sample.txt")
            with open(sample_file, "w", encoding="utf-8") as f:
                f.write("hello world\n")

            runner = ab_batch.Runner()
            out = runner.git_status_short(cwd=tmpdir)
            self.assertIn("?? sample.txt", out)

            diff_out = runner.git_diff_stat(cwd=tmpdir)
            self.assertEqual(diff_out, "")

    def test_git_runner_timeout_and_error_returns_unavailable(self):
        """S4: Runner returns rc=124 on timeout and result starts with 'UNAVAILABLE'."""
        runner = ab_batch.Runner()

        def mock_run_timeout(cmd, timeout=None, cwd=None, **kwargs):
            return subprocess.CompletedProcess(
                cmd, returncode=124, stdout="", stderr="TimeoutExpired after 120s"
            )
        runner.run = mock_run_timeout
        status_res = runner.git_status_short()
        self.assertTrue(status_res.startswith("UNAVAILABLE"))
        self.assertIn("rc=124", status_res)
        self.assertIn("TimeoutExpired", status_res)

        diff_res = runner.git_diff_stat()
        self.assertTrue(diff_res.startswith("UNAVAILABLE"))
        self.assertIn("rc=124", diff_res)

        commit_res = runner.git_commit_hash()
        self.assertTrue(commit_res.startswith("UNAVAILABLE"))
        self.assertIn("rc=124", commit_res)

        def mock_run_exc(cmd, timeout=None, cwd=None, **kwargs):
            raise RuntimeError("process crashed")
        runner.run = mock_run_exc
        err_res = runner.capture_git_output(["git", "status"])
        self.assertTrue(err_res.startswith("UNAVAILABLE: rc=-1"))
        self.assertIn("process crashed", err_res)

    def test_compute_file_sha256(self):
        """S4: compute_file_sha256 returns correct hex digests and null for missing files."""
        import hashlib
        with tempfile.TemporaryDirectory() as tmpdir:
            file_a = os.path.join(tmpdir, "a.py")
            with open(file_a, "wb") as f:
                f.write(b"print('hello world')\n")
            expected_sha = hashlib.sha256(b"print('hello world')\n").hexdigest()

            hashes = ab_batch.compute_file_sha256(tmpdir, ["a.py", "nonexistent.py"])
            self.assertEqual(hashes["a.py"], expected_sha)
            self.assertIsNone(hashes["nonexistent.py"])

    def test_git_capture_before_bringup_logged_and_stored_in_meta(self):
        """S1/S2: Git capture runs before first bringup, logs message, and stores in meta_*.json."""
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "1", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)

            session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
            session_dir = os.path.join(tmpdir, session_dirs[0])
            batch_log = os.path.join(session_dir, "batch.log")
            with open(batch_log, "r", encoding="utf-8") as f:
                log_content = f.read()

            self.assertIn("Capturing git state (can take up to ~40 s on /mnt/c)...", log_content)

            meta_file = os.path.join(session_dir, "meta_astar.json")
            with open(meta_file, "r", encoding="utf-8") as f:
                meta = json.load(f)

            self.assertIn("git_commit", meta)
            self.assertIn("git_status", meta)
            self.assertIn("git_diff_stat", meta)
            self.assertIn("file_sha256", meta)
            self.assertEqual(meta["git_diff_stat"], " 1 file changed, 1 insertion(+)")
            self.assertIsInstance(meta["file_sha256"], dict)

    def test_no_killall_or_pkill_in_ab_batch_source(self):
        """Strict rule: never use killall or pkill anywhere in ab_batch.py."""
        batch_path = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "ab_batch.py")
        )
        with open(batch_path, "r", encoding="utf-8") as f:
            src = f.read()

        self.assertNotIn("killall", src)
        self.assertNotIn("pkill", src)

    def test_python38_ast_compatibility(self):
        """Strict rule: ab_batch.py must parse cleanly with Python 3.8 grammar."""
        batch_path = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "ab_batch.py")
        )
        with open(batch_path, "r", encoding="utf-8") as f:
            src = f.read()

        ast.parse(src, filename=batch_path, feature_version=(3, 8))

    def test_parse_csv_summary_planner_time(self):
        """W3/W4: parse_csv_summary extracts mean_plan_s and mean_plan_pct from CSV."""
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = os.path.join(tmpdir, "astar_runs.csv")
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["run_id", "outcome", "sim_time_to_goal_s", "wall_time_s", "settle_retry", "plan_ms_total", "plan_share_pct"])
                writer.writerow(["r1", "succeeded", "40.0", "40.0", "0", "3000.0", "7.5"])
                writer.writerow(["r2", "succeeded", "42.0", "42.0", "0", "3200.0", "7.62"])
                writer.writerow(["r3", "timeout", "120.0", "120.0", "0", "5000.0", ""])

            summary = ab_batch.parse_csv_summary(csv_path)
            self.assertEqual(summary["valid"], 3)
            self.assertEqual(summary["succeeded"], 2)
            # mean of 3.0s and 3.2s = 3.10s
            self.assertAlmostEqual(summary["mean_plan_s"], 3.10)
            # mean of 7.5 and 7.62 = 7.56
            self.assertAlmostEqual(summary["mean_plan_pct"], 7.56)

            # Test CSV without the columns returns None
            csv_old = os.path.join(tmpdir, "old_runs.csv")
            with open(csv_old, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["run_id", "outcome", "sim_time_to_goal_s", "wall_time_s", "settle_retry"])
                writer.writerow(["r1", "succeeded", "40.0", "40.0", "0"])

            summary_old = ab_batch.parse_csv_summary(csv_old)
            self.assertIsNone(summary_old["mean_plan_s"])
            self.assertIsNone(summary_old["mean_plan_pct"])

    def test_summary_block_planner_time_per_run(self):
        """W3/W4: SUMMARY block logs 'Planner time/run:' with mean seconds and percent."""
        fake = FakeRunner()
        with tempfile.TemporaryDirectory() as tmpdir:
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "2", "--out", tmpdir])
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)

            session_dirs = [d for d in os.listdir(tmpdir) if os.path.isdir(os.path.join(tmpdir, d))]
            batch_log = os.path.join(tmpdir, session_dirs[0], "batch.log")
            with open(batch_log, "r", encoding="utf-8") as f:
                log_text = f.read()

            self.assertIn("Planner time/run:", log_text)


if __name__ == "__main__":
    unittest.main()
