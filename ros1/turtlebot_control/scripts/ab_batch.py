#!/usr/bin/env python3
"""A/B Benchmark Batch Orchestrator for A* vs D* Lite.

Manages the lifecycle of ROS launch stacks, parameter configuration,
benchmark execution, process supervision, metadata logging, and report generation.
Python 3.8 compatible, standard library only.
"""

import argparse
import ast
import csv
import datetime
import hashlib
import json
import math
import os
import queue
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union


class Runner:
    """Wraps subprocess and process signals so tests can inject a mock runner."""

    def __init__(self) -> None:
        self.last_topic_error: Optional[str] = None
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.repo_dir = os.path.normpath(os.path.join(base_dir, "..", "..", ".."))
        if "ROS_ROOT" not in os.environ or "roslaunch" not in os.environ.get("PATH", ""):
            cmd = "source /opt/ros/noetic/setup.bash 2>/dev/null && source ~/nav_ws/devel/setup.bash 2>/dev/null && env -0"
            try:
                out = subprocess.check_output(["bash", "-c", cmd])
                for entry in out.split(b"\0"):
                    if b"=" in entry:
                        k, v = entry.split(b"=", 1)
                        k_str = k.decode("utf-8", "replace")
                        v_str = v.decode("utf-8", "replace")
                        if k_str.startswith("ROS_") or k_str in (
                            "PATH", "PYTHONPATH", "LD_LIBRARY_PATH", "CMAKE_PREFIX_PATH", "PKG_CONFIG_PATH"
                        ):
                            os.environ[k_str] = v_str
            except Exception:
                pass

    def popen(
        self,
        cmd: List[str],
        stdout: Any = None,
        stderr: Any = None,
        preexec_fn: Any = None,
        **kwargs: Any,
    ) -> Any:
        return subprocess.Popen(
            cmd,
            stdout=stdout,
            stderr=stderr,
            preexec_fn=preexec_fn,
            **kwargs,
        )

    def run(
        self,
        cmd: List[str],
        capture_output: bool = True,
        text: bool = True,
        timeout: Optional[float] = None,
        check: bool = False,
        **kwargs: Any,
    ) -> subprocess.CompletedProcess:
        try:
            return subprocess.run(
                cmd,
                capture_output=capture_output,
                text=text,
                timeout=timeout,
                check=check,
                **kwargs,
            )
        except subprocess.TimeoutExpired as exc:
            self.last_topic_error = f"TimeoutExpired after {timeout}s"
            if check:
                raise
            return subprocess.CompletedProcess(
                cmd,
                returncode=124,
                stdout=exc.stdout if exc.stdout else "",
                stderr=exc.stderr if exc.stderr else f"TimeoutExpired after {timeout}s",
            )

    def sleep(self, seconds: float) -> None:
        """Sleep for specified seconds."""
        time.sleep(seconds)

    def kill_pg(self, pid: int, sig: int) -> None:
        """Send signal to process group of pid."""
        try:
            pgid = os.getpgid(pid)
            os.killpg(pgid, sig)
        except (ProcessLookupError, PermissionError):
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass

    def is_pid_alive(self, pid: int) -> bool:
        if pid <= 0:
            return False
        try:
            os.kill(pid, 0)
        except (ProcessLookupError, PermissionError):
            return False
        try:
            with open(f"/proc/{pid}/stat", "r") as f:
                s = f.read()
            r = s.rfind(")")
            return not (r != -1 and s[r + 1:].split()[0] == "Z")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            return not os.path.exists("/proc")
        return True

    def is_ros_master_up(self, uri: str = "http://localhost:11311") -> bool:
        """Check if a ROS master answers at uri."""
        orig_timeout = socket.getdefaulttimeout()
        try:
            socket.setdefaulttimeout(1.0)
            server = xmlrpc.client.ServerProxy(uri)
            server.getUri("probe")
            return True
        except Exception:
            return False
        finally:
            socket.setdefaulttimeout(orig_timeout)

    def get_system_state(self, uri: str = "http://localhost:11311") -> Optional[Dict[str, List[str]]]:
        """Call master getSystemState over XML-RPC and return publishers dict {topic: [nodes]} or None."""
        orig_timeout = socket.getdefaulttimeout()
        try:
            socket.setdefaulttimeout(2.0)
            server = xmlrpc.client.ServerProxy(uri)
            res = server.getSystemState("ab_batch")
            if isinstance(res, (list, tuple)) and len(res) >= 3 and res[0] == 1:
                system_state = res[2]
                if isinstance(system_state, (list, tuple)) and len(system_state) >= 1:
                    raw_pubs = system_state[0]
                    pubs_dict: Dict[str, List[str]] = {}
                    for item in raw_pubs:
                        if isinstance(item, (list, tuple)) and len(item) >= 2:
                            pubs_dict[str(item[0])] = [str(n) for n in item[1]]
                    return pubs_dict
        except Exception:
            pass
        finally:
            socket.setdefaulttimeout(orig_timeout)
        return None

    def get_ros_nodes(self) -> List[str]:
        """Query active node names via rosnode list."""
        try:
            res = self.run(["rosnode", "list"], timeout=10.0)
            if res.returncode == 0:
                return [line.strip() for line in res.stdout.splitlines() if line.strip()]
        except Exception:
            pass
        return []

    def is_topic_publishing(self, topic: str, timeout_s: float = 10.0) -> bool:
        """Check if topic is actively publishing messages."""
        self.last_topic_error = None
        try:
            res = self.run(["rostopic", "echo", "-n", "1", topic], timeout=timeout_s)
            if res.returncode == 0:
                if bool(res.stdout.strip()):
                    return True
                self.last_topic_error = "empty output"
                return False
            if not self.last_topic_error:
                err_text = res.stderr.strip() if res.stderr else "no stderr"
                self.last_topic_error = f"returncode {res.returncode}: {err_text}"
            return False
        except subprocess.TimeoutExpired:
            self.last_topic_error = f"TimeoutExpired after {timeout_s}s"
            return False
        except Exception as exc:
            self.last_topic_error = str(exc)
            return False

    def rosparam_set(self, param: str, val: Any) -> bool:
        """Set a parameter via rosparam set."""
        val_str = "true" if val is True else ("false" if val is False else str(val))
        res = self.run(["rosparam", "set", param, val_str], timeout=10.0)
        return res.returncode == 0

    def rosparam_get(self, param: str) -> Optional[Any]:
        """Get parameter value via rosparam get."""
        try:
            res = self.run(["rosparam", "get", param], timeout=10.0)
            if res.returncode == 0:
                s = res.stdout.strip()
                if s.lower() == "true":
                    return True
                if s.lower() == "false":
                    return False
                try:
                    return float(s)
                except ValueError:
                    return s
        except Exception:
            pass
        return None

    def capture_git_output(
        self,
        cmd: List[str],
        cwd: Optional[str] = None,
        timeout: float = 120.0,
        max_lines: int = 200,
    ) -> str:
        """Execute a git command with timeout and return up to max_lines lines.

        On timeout or error, returns 'UNAVAILABLE: rc=<rc> <first 200 chars of stderr>', never ''.
        """
        if cwd is None:
            cwd = getattr(self, "repo_dir", None)
        try:
            res = self.run(cmd, timeout=timeout, cwd=cwd)
            if res.returncode == 0:
                lines = res.stdout.splitlines()[:max_lines]
                return "\n".join(lines).strip()
            stderr_snippet = (res.stderr or "").strip()[:200]
            err_part = f" {stderr_snippet}" if stderr_snippet else ""
            return f"UNAVAILABLE: rc={res.returncode}{err_part}"
        except Exception as exc:
            err_str = str(exc).strip()[:200]
            err_part = f" {err_str}" if err_str else ""
            return f"UNAVAILABLE: rc=-1{err_part}"

    def git_commit_hash(self, cwd: Optional[str] = None, timeout: float = 120.0) -> str:
        """Get current git commit hash."""
        if cwd is None:
            cwd = getattr(self, "repo_dir", None)
        try:
            res = self.run(["git", "rev-parse", "HEAD"], timeout=timeout, cwd=cwd)
            if res.returncode == 0:
                return res.stdout.strip()
            stderr_snippet = (res.stderr or "").strip()[:200]
            err_part = f" {stderr_snippet}" if stderr_snippet else ""
            return f"UNAVAILABLE: rc={res.returncode}{err_part}"
        except Exception as exc:
            err_str = str(exc).strip()[:200]
            err_part = f" {err_str}" if err_str else ""
            return f"UNAVAILABLE: rc=-1{err_part}"

    def git_status_short(self, cwd: Optional[str] = None, timeout: float = 120.0) -> str:
        """Get git status --short output, first 200 lines."""
        return self.capture_git_output(["git", "status", "--short"], cwd=cwd, timeout=timeout)

    def git_diff_stat(self, cwd: Optional[str] = None, timeout: float = 120.0) -> str:
        """Get git diff --stat output, first 200 lines."""
        return self.capture_git_output(["git", "diff", "--stat"], cwd=cwd, timeout=timeout)


META_TRACKED_FILES = [
    "ros1/turtlebot_control/scripts/ab_batch.py",
    "ros1/turtlebot_control/scripts/ab_run.py",
    "ros1/turtlebot_control/scripts/ab_report.py",
    "ros1/turtlebot_control/scripts/dstar_planner.py",
    "ros1/turtlebot_control/scripts/dstar_lite.py",
    "ros1/turtlebot_control/scripts/astar_planner.py",
    "ros1/turtlebot_control/scripts/path_follower.py",
    "ros1/turtlebot_control/launch/navigation_dstar.launch",
    "WarehouseProjectURP/Assets/Scripts/Ros/ScenarioLoader.cs",
]


def compute_file_sha256(
    repo_dir: str, rel_paths: Optional[Sequence[str]] = None
) -> Dict[str, Optional[str]]:
    """Compute SHA-256 hash for specified files relative to repo_dir.

    Returns null/None for missing files.
    """
    if rel_paths is None:
        rel_paths = META_TRACKED_FILES
    result: Dict[str, Optional[str]] = {}
    for rel_path in rel_paths:
        full_path = os.path.join(repo_dir, rel_path)
        if not os.path.isfile(full_path):
            result[rel_path] = None
            continue
        try:
            h = hashlib.sha256()
            with open(full_path, "rb") as f:
                while True:
                    chunk = f.read(65536)
                    if not chunk:
                        break
                    h.update(chunk)
            result[rel_path] = h.hexdigest()
        except Exception:
            result[rel_path] = None
    return result


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse CLI arguments for ab_batch."""
    parser = argparse.ArgumentParser(description="A/B Benchmark Batch Orchestrator for A* vs D* Lite")
    parser.add_argument(
        "--planner",
        choices=["astar", "dstar", "both"],
        default="both",
        help="Planner to benchmark (default: both)",
    )
    parser.add_argument("--repeats", type=int, default=5, help="Number of repetitions per planner (default: 5)")
    parser.add_argument("--scenario", default="S-04", help="Scenario name (default: S-04)")
    parser.add_argument(
        "--goal",
        nargs=2,
        type=float,
        default=[-3.75, -3.13],
        metavar=("X", "Y"),
        help="Goal coordinates (default: -3.75 -3.13)",
    )
    parser.add_argument("--settle", type=float, default=7.0, help="Settle time in sim seconds after reset (default: 7.0)")
    parser.add_argument("--timeout", type=float, default=120.0, help="Timeout in sim seconds per run (default: 120.0)")
    parser.add_argument("--rviz", action="store_true", default=False, help="Launch RViz (default: False)")
    parser.add_argument("--out", default="~/ab_runs", help="Base output directory (default: ~/ab_runs)")
    parser.add_argument(
        "--dry",
        action="store_true",
        default=False,
        help="Dry run: 3 repeats, session prefixed dry_, no report",
    )
    parser.add_argument("--stop", action="store_true", default=False, help="Stop a running batch")
    parser.add_argument("--order", default="astar,dstar", help="Execution order for planners (default: astar,dstar)")
    return parser.parse_args(argv)


class TeeLogger:
    """Tees log messages to console and batch.log."""

    def __init__(self, log_path: str):
        self.log_path = log_path
        self._f = open(log_path, "a", encoding="utf-8")

    def log(self, msg: str) -> None:
        print(msg)
        sys.stdout.flush()
        self._f.write(msg + "\n")
        self._f.flush()

    def close(self) -> None:
        if self._f and not self._f.closed:
            self._f.close()


def stop_batch(out_dir: str, runner: Runner) -> int:
    """Stop active batch process found via pidfile. Waits up to 45s before SIGTERM."""
    pid_file = os.path.join(out_dir, "batch.pid")
    if not os.path.exists(pid_file):
        print("No active batch found.")
        return 0

    try:
        with open(pid_file, "r", encoding="utf-8") as f:
            pid_str = f.read().strip()
        pid = int(pid_str)
    except Exception:
        print(f"Malformed pidfile {pid_file}. Removing.")
        try:
            os.remove(pid_file)
        except OSError:
            pass
        return 0

    if runner.is_pid_alive(pid):
        print(f"Stopping batch (PID {pid})...")
        runner.kill_pg(pid, signal.SIGINT)

        # Wait up to 45s for the batch process to exit cleanly on SIGINT
        t_stop_start = time.time()
        while time.time() - t_stop_start < 45.0:
            if not runner.is_pid_alive(pid):
                break
            runner.sleep(1.0)

        # If still alive after 45s, escalate to SIGTERM
        if runner.is_pid_alive(pid):
            print(f"Batch PID {pid} still running after 45s; escalating to SIGTERM...")
            runner.kill_pg(pid, signal.SIGTERM)
            t_term = time.time()
            while time.time() - t_term < 5.0:
                if not runner.is_pid_alive(pid):
                    break
                runner.sleep(0.5)

            if runner.is_pid_alive(pid):
                print(f"Batch PID {pid} still running; sending SIGKILL...")
                runner.kill_pg(pid, signal.SIGKILL)
    else:
        print(f"PID {pid} is not running. Removing stale pidfile.")

    try:
        if os.path.exists(pid_file):
            os.remove(pid_file)
    except OSError:
        pass
    return 0


def cleanup_group(runner: Runner, procs: List[Any], logger: Optional[TeeLogger] = None) -> None:
    """Clean up spawned process groups with SIGINT, SIGTERM, SIGKILL escalation."""
    for p in procs:
        if p and hasattr(p, "pid") and runner.is_pid_alive(p.pid):
            runner.kill_pg(p.pid, signal.SIGINT)

    t0 = time.time()
    while time.time() - t0 < 10.0:
        if all(not runner.is_pid_alive(p.pid) for p in procs if p and hasattr(p, "pid")):
            break
        runner.sleep(0.5)

    for p in procs:
        if p and hasattr(p, "pid") and runner.is_pid_alive(p.pid):
            runner.kill_pg(p.pid, signal.SIGTERM)

    t1 = time.time()
    while time.time() - t1 < 5.0:
        if all(not runner.is_pid_alive(p.pid) for p in procs if p and hasattr(p, "pid")):
            break
        runner.sleep(0.5)

    for p in procs:
        if p and hasattr(p, "pid") and runner.is_pid_alive(p.pid):
            runner.kill_pg(p.pid, signal.SIGKILL)

    t_master = time.time()
    while time.time() - t_master < 15.0:
        if not runner.is_ros_master_up():
            break
        runner.sleep(0.5)

    if runner.is_ros_master_up():
        msg = "Leftover warning: ROS master is still answering after cleanup!"
        if logger:
            logger.log(msg)
        else:
            print(msg)


def scan_log_for_respawns(log_path: str, max_bytes: Optional[int] = None) -> int:
    """Scan bringup log for process deaths and respawns matching roslaunch/pmon.py exact text:
    - pmon.py:571 / nodeprocess.py:400: 'process has died'
    - pmon.py:615: 'restarting process'
    If max_bytes is provided, scan only up to that byte offset to avoid shutdown noise.
    """
    count = 0
    if not os.path.exists(log_path):
        return 0
    try:
        with open(log_path, "rb") as f:
            raw = f.read(max_bytes) if max_bytes is not None else f.read()
        text = raw.decode("utf-8", errors="replace")
        for line in text.splitlines():
            if "process has died" in line or "restarting process" in line:
                count += 1
    except Exception:
        pass
    return count


def parse_csv_summary(csv_path: str) -> Dict[str, Any]:
    """Parse CSV run results for counts of outcomes, reasons, settle retries, and median sim/wall ratio."""
    n_valid = 0
    n_invalid = 0
    n_succeeded = 0
    n_timeout = 0
    n_aborted = 0
    invalid_reasons: Dict[str, int] = {}
    settle_retries = 0
    sim_wall_ratios: List[float] = []
    plan_s_list: List[float] = []
    plan_pct_list: List[float] = []

    if not os.path.exists(csv_path):
        return {
            "valid": 0,
            "invalid": 0,
            "succeeded": 0,
            "timeout": 0,
            "aborted": 0,
            "reasons": {},
            "retries": 0,
            "median_sim_wall": None,
            "mean_plan_s": None,
            "mean_plan_pct": None,
        }

    with open(csv_path, "r", encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row:
                continue
            outcome = str(row.get("outcome", "")).strip().lower()
            retry_val = row.get("settle_retry", "0")
            try:
                if int(retry_val) == 1:
                    settle_retries += 1
            except ValueError:
                pass

            if outcome == "invalid":
                n_invalid += 1
                reason = str(row.get("invalid_reason", "")).strip() or "unspecified"
                invalid_reasons[reason] = invalid_reasons.get(reason, 0) + 1
            else:
                n_valid += 1
                if outcome == "succeeded":
                    n_succeeded += 1
                    st_val = row.get("sim_time_to_goal_s")
                    wt_val = row.get("wall_time_s")
                    try:
                        if st_val is not None and wt_val is not None:
                            st = float(str(st_val).strip())
                            wt = float(str(wt_val).strip())
                            if wt > 0.0:
                                sim_wall_ratios.append(st / wt)
                    except (ValueError, TypeError):
                        pass

                    pt_val = row.get("plan_ms_total")
                    pp_val = row.get("plan_share_pct")
                    try:
                        if pt_val is not None and str(pt_val).strip():
                            plan_s_list.append(float(str(pt_val).strip()) / 1000.0)
                    except (ValueError, TypeError):
                        pass
                    try:
                        if pp_val is not None and str(pp_val).strip():
                            plan_pct_list.append(float(str(pp_val).strip()))
                    except (ValueError, TypeError):
                        pass
                elif outcome == "timeout":
                    n_timeout += 1
                elif outcome == "aborted":
                    n_aborted += 1

    median_sim_wall = None
    if sim_wall_ratios:
        import statistics
        median_sim_wall = float(statistics.median(sim_wall_ratios))

    mean_plan_s = None
    mean_plan_pct = None
    if plan_s_list:
        import statistics
        mean_plan_s = float(statistics.mean(plan_s_list))
    if plan_pct_list:
        import statistics
        mean_plan_pct = float(statistics.mean(plan_pct_list))

    return {
        "valid": n_valid,
        "invalid": n_invalid,
        "succeeded": n_succeeded,
        "timeout": n_timeout,
        "aborted": n_aborted,
        "reasons": invalid_reasons,
        "retries": settle_retries,
        "median_sim_wall": median_sim_wall,
        "mean_plan_s": mean_plan_s,
        "mean_plan_pct": mean_plan_pct,
    }


def _format_bringup_log(fpath: str, max_first: int = 15, max_last: int = 60) -> str:
    """Format bringup log taking first N lines, death/restart lines, and last M lines without duplication."""
    try:
        with open(fpath, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()
    except Exception:
        return ""
    n = len(lines)
    if n == 0:
        return ""
    indices = set()
    for i in range(min(max_first, n)):
        indices.add(i)
    for i, line in enumerate(lines):
        if "process has died" in line or "restarting process" in line:
            indices.add(i)
    for i in range(max(0, n - max_last), n):
        indices.add(i)
    return "".join(lines[i] for i in sorted(indices))


def write_handoff(
    session_dir: str,
    args: argparse.Namespace,
    runner: Runner,
    start_time: str,
    logger: Optional[TeeLogger] = None,
) -> Optional[str]:
    """Write <session>/handoff.txt consolidating all artifacts of the run.
    Best effort: never raises. Hard cap: 400 KB.
    """
    try:
        if not os.path.isdir(session_dir):
            return None

        handoff_path = os.path.join(session_dir, "handoff.txt")

        # 1. Header section
        header_text = (
            "===== header =====\n"
            f"Time: {datetime.datetime.now().isoformat()}\n"
            f"Start Time: {start_time}\n"
            f"Git Commit: {runner.git_commit_hash()}\n"
            f"CLI Args: {vars(args)}\n\n"
        )

        # 2. batch.log section in full
        if logger and hasattr(logger, "_f") and logger._f and not logger._f.closed:
            try:
                logger._f.flush()
            except Exception:
                pass
        batch_log_path = os.path.join(session_dir, "batch.log")
        batch_log_content = ""
        if os.path.exists(batch_log_path):
            try:
                with open(batch_log_path, "r", encoding="utf-8", errors="replace") as f:
                    batch_log_content = f.read()
            except Exception:
                pass
        batch_section = f"===== batch.log =====\n{batch_log_content}\n"

        # 3. Every meta_*.json in full
        meta_sections = []
        try:
            for mf in sorted(os.listdir(session_dir)):
                if mf.startswith("meta_") and mf.endswith(".json"):
                    with open(os.path.join(session_dir, mf), "r", encoding="utf-8", errors="replace") as f:
                        meta_sections.append(f"===== {mf} =====\n{f.read()}\n")
        except Exception:
            pass
        meta_text = "\n".join(meta_sections)

        # 4. Every *_runs.csv in full
        csv_sections = []
        try:
            for cf in sorted(os.listdir(session_dir)):
                if cf.endswith("_runs.csv"):
                    with open(os.path.join(session_dir, cf), "r", encoding="utf-8", errors="replace") as f:
                        csv_sections.append(f"===== {cf} =====\n{f.read()}\n")
        except Exception:
            pass
        csv_text = "\n".join(csv_sections)

        # 5. Bringup logs
        bringup_files = []
        try:
            bringup_files = sorted(
                [f for f in os.listdir(session_dir) if f.startswith("bringup_") and f.endswith(".log")]
            )
        except Exception:
            pass

        max_cap = 400 * 1024  # 409600 bytes
        base_text = header_text + batch_section + (meta_text + "\n" if meta_text else "") + (csv_text + "\n" if csv_text else "")
        base_bytes = len(base_text.encode("utf-8", errors="replace"))

        shrink_tiers = [(15, 60), (10, 30), (5, 15), (0, 5), (0, 0)]
        bringup_text = ""
        for max_first, max_last in shrink_tiers:
            b_sections = []
            for bf in bringup_files:
                fpath = os.path.join(session_dir, bf)
                formatted = _format_bringup_log(fpath, max_first, max_last)
                b_sections.append(f"===== {bf} =====\n{formatted}\n")
            candidate_bringup = "\n".join(b_sections)
            candidate_bytes = len(candidate_bringup.encode("utf-8", errors="replace"))
            if base_bytes + candidate_bytes <= max_cap or (max_first == 0 and max_last == 0):
                bringup_text = candidate_bringup
                break

        full_content = base_text + (bringup_text if bringup_text else "")
        encoded = full_content.encode("utf-8", errors="replace")
        if len(encoded) > max_cap:
            encoded = encoded[:max_cap]

        with open(handoff_path, "wb") as f_out:
            f_out.write(encoded)

        return handoff_path
    except Exception:
        return None


def run_batch(args: argparse.Namespace, runner: Optional[Runner] = None) -> int:
    """Main batch orchestration routine."""
    if runner is None:
        runner = Runner()

    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)

    if args.stop:
        return stop_batch(out_dir, runner)

    pid_file = os.path.join(out_dir, "batch.pid")

    # 1. Preflight
    if os.path.exists(pid_file):
        try:
            with open(pid_file, "r", encoding="utf-8") as f:
                existing_pid = int(f.read().strip())
            if runner.is_pid_alive(existing_pid):
                print(f"Error: A live batch is already running (PID {existing_pid}). Run Clean Stop first.", file=sys.stderr)
                return 1
        except Exception:
            pass

    if runner.is_ros_master_up():
        print("Error: A ROS master is already answering. Run Clean Stop first.", file=sys.stderr)
        return 1

    # Write pidfile
    try:
        with open(pid_file, "w", encoding="utf-8") as f:
            f.write(str(os.getpid()))
    except Exception as exc:
        print(f"Error writing pidfile: {exc}", file=sys.stderr)
        return 1

    # Prepare session dir
    batch_start_iso = datetime.datetime.now().isoformat()
    now_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_name = f"dry_{now_str}" if args.dry else now_str
    session_dir = os.path.join(out_dir, session_name)
    os.makedirs(session_dir, exist_ok=True)

    # Write latest_session.txt (L4)
    latest_session_file = os.path.join(out_dir, "latest_session.txt")
    try:
        with open(latest_session_file, "w", encoding="utf-8") as f_ls:
            f_ls.write(session_dir + "\n")
    except Exception:
        pass

    logger = TeeLogger(os.path.join(session_dir, "batch.log"))
    logger.log(f"=== A/B Benchmark Batch Started at {batch_start_iso} ===")
    logger.log(f"Session directory: {session_dir}")
    logger.log(f"Output directory:  {out_dir}")

    # Determine execution list
    if args.planner == "both":
        raw_order = [p.strip().lower() for p in args.order.split(",") if p.strip()]
        planners_to_run = [p for p in raw_order if p in ("astar", "dstar")]
        if not planners_to_run:
            planners_to_run = ["astar", "dstar"]
    else:
        planners_to_run = [args.planner.lower()]

    repeats = 3 if args.dry else args.repeats
    rviz_val = "true" if args.rviz else "false"

    base_script_dir = os.path.dirname(os.path.abspath(__file__))
    ab_run_path = os.path.join(base_script_dir, "ab_run.py")
    ab_report_path = os.path.join(base_script_dir, "ab_report.py")

    planner_results: Dict[str, Dict[str, Any]] = {}
    fatal_error = False

    active_procs: List[Any] = []
    cleaning_up = False

    def sig_handler(_signum: int, _frame: Any) -> None:
        nonlocal cleaning_up
        if cleaning_up:
            return
        cleaning_up = True
        logger.log(f"\nInterrupted by signal {_signum}! Cleaning up...")
        handoff_path = write_handoff(session_dir, args, runner, batch_start_iso, logger)
        if handoff_path:
            logger.log(f"Handoff: {handoff_path}")
            print(f"Handoff: {handoff_path}")
        cleanup_group(runner, active_procs, logger)
        if bringup_f and not bringup_f.closed:
            bringup_f.close()
        if os.path.exists(pid_file):
            try:
                os.remove(pid_file)
            except OSError:
                pass
        if logger:
            logger.close()
        sys.exit(130)

    orig_sigint = signal.signal(signal.SIGINT, sig_handler)
    orig_sigterm = signal.signal(signal.SIGTERM, sig_handler)

    bringup_f = None
    batch_completed = False

    logger.log("Capturing git state (can take up to ~40 s on /mnt/c)...")
    batch_git_commit = runner.git_commit_hash(timeout=120.0)
    batch_git_status = runner.git_status_short(timeout=120.0)
    batch_git_diff_stat = runner.git_diff_stat(timeout=120.0)
    batch_file_sha256 = compute_file_sha256(runner.repo_dir)

    try:
        for idx, planner in enumerate(planners_to_run):
            logger.log(f"\n------------------------------------------------------------")
            logger.log(f"Starting Planner {idx + 1} / {len(planners_to_run)}: {planner.upper()}")
            logger.log(f"------------------------------------------------------------")

            start_iso = datetime.datetime.now().isoformat()
            bringup_log_path = os.path.join(session_dir, f"bringup_{planner}.log")
            active_procs = []

            # 2. Start bringup
            bringup_f = open(bringup_log_path, "w", encoding="utf-8")
            bringup_cmd = ["roslaunch", "warehouse_bringup", "bringup.launch", f"rviz:={rviz_val}"]
            if planner == "dstar":
                bringup_cmd.append("nav:=false")

            logger.log(f"Launching bringup: {' '.join(bringup_cmd)}")
            p_bringup = runner.popen(
                bringup_cmd,
                stdout=bringup_f,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid if hasattr(os, "setsid") else None,
            )
            active_procs.append(p_bringup)

            # 3. Wait up to 90s for master up, unity_endpoint node, and /clock publisher (O1)
            logger.log("Waiting for ROS stack & Unity (90s wall limit)...")
            t_wait_start = time.time()
            stack_ready = False
            is_master_up = False
            has_endpoint = False
            has_clock_pub = False
            current_nodes: List[str] = []
            publishers_dict: Optional[Dict[str, List[str]]] = None

            while time.time() - t_wait_start < 90.0:
                is_master_up = runner.is_ros_master_up()
                if is_master_up:
                    current_nodes = runner.get_ros_nodes()
                    has_endpoint = any("unity_endpoint" in n for n in current_nodes)
                    publishers_dict = runner.get_system_state()
                    clock_pubs = publishers_dict.get("/clock", []) if publishers_dict else []
                    has_clock_pub = any("unity_endpoint" in p for p in clock_pubs)
                    if has_endpoint and has_clock_pub:
                        stack_ready = True
                        break
                runner.sleep(1.0)

            if not stack_ready:
                logger.log(f"Stack not ready after 90 s: master={is_master_up} endpoint={has_endpoint} clock_publisher={has_clock_pub}")
                if not has_endpoint:
                    logger.log("Unity not connected: is Play pressed?")
                else:
                    logger.log("Unity connected but /clock not registered")
                logger.log(f"rosnode list: {current_nodes}")
                logger.log(f"publishers: {publishers_dict}")
                logger.log(f"last_topic_error: {runner.last_topic_error}")
                if bringup_f and not bringup_f.closed:
                    bringup_f.flush()
                tail_lines = []
                if os.path.exists(bringup_log_path):
                    try:
                        with open(bringup_log_path, "r", encoding="utf-8", errors="replace") as f_tail:
                            tail_lines = [l.rstrip() for l in f_tail.readlines()[-25:]]
                    except Exception:
                        pass
                logger.log("bringup log (last 25 lines):\n" + "\n".join(tail_lines))
                cleanup_group(runner, active_procs, logger)
                if bringup_f and not bringup_f.closed:
                    bringup_f.close()
                fatal_error = True
                break

            # Soft check for active topic publishing after gate passes (O2, never fails batch)
            clock_active = runner.is_topic_publishing("/clock", timeout_s=10.0)
            if clock_active:
                logger.log("Soft-check: /clock is actively publishing.")
            else:
                logger.log(f"Soft-check: /clock not publishing yet ({runner.last_topic_error}); proceeding as ab_run awaits live data.")

            # 4. dstar only: configure rosparam and start navigation_dstar.launch
            if planner == "dstar":
                logger.log("Configuring D* Lite rosparams...")
                runner.rosparam_set("/dstar_planner/filter_static_hits", True)
                runner.rosparam_set("/dstar_planner/blocked_use_plan_radius", True)
                runner.rosparam_set("/dstar_planner/verify_costmap", False)

                dstar_cmd = ["roslaunch", "turtlebot_control", "navigation_dstar.launch"]
                logger.log(f"Launching D* navigation: {' '.join(dstar_cmd)}")
                p_dstar = runner.popen(
                    dstar_cmd,
                    stdout=bringup_f,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid if hasattr(os, "setsid") else None,
                )
                active_procs.append(p_dstar)

            # 5. Verify nodes & params (polling up to 20s wall, 1s step)
            logger.log("Verifying node configuration and parameters (up to 20s)...")
            t_verify_start = time.time()
            verified = False
            last_missing_msgs = []

            while time.time() - t_verify_start < 20.0:
                current_nodes = runner.get_ros_nodes()
                has_follower = any("path_follower" in n for n in current_nodes)
                has_astar = any("astar_planner" in n for n in current_nodes)
                has_dstar = any("dstar_planner" in n for n in current_nodes)
                missing = []

                if not has_follower:
                    missing.append("node /path_follower")

                if planner == "astar":
                    if not has_astar:
                        missing.append("node /astar_planner")
                    if has_dstar:
                        missing.append("unexpected node /dstar_planner")
                    if not missing:
                        verified = True
                        break
                elif planner == "dstar":
                    if not has_dstar:
                        missing.append("node /dstar_planner")
                    if has_astar:
                        missing.append("unexpected node /astar_planner")
                    p1 = runner.rosparam_get("/dstar_planner/filter_static_hits")
                    p2 = runner.rosparam_get("/dstar_planner/blocked_use_plan_radius")
                    p3 = runner.rosparam_get("/dstar_planner/verify_costmap")
                    if p1 is not True:
                        missing.append(f"param /dstar_planner/filter_static_hits={p1} (expected True)")
                    if p2 is not True:
                        missing.append(f"param /dstar_planner/blocked_use_plan_radius={p2} (expected True)")
                    if p3 is not False:
                        missing.append(f"param /dstar_planner/verify_costmap={p3} (expected False)")
                    if not missing:
                        verified = True
                        break

                last_missing_msgs = missing
                runner.sleep(1.0)

            if not verified:
                err_msg = f"Verification failed after 20s! Missing/mismatched: {', '.join(last_missing_msgs)}. Current nodes: {current_nodes}"
                logger.log(err_msg)
                cleanup_group(runner, active_procs, logger)
                if bringup_f and not bringup_f.closed:
                    bringup_f.close()
                fatal_error = True
                break

            logger.log("Nodes and parameters verified successfully.")

            # 6. Run ab_run.py
            cmd_ab_run = [
                sys.executable,
                ab_run_path,
                "--scenario",
                args.scenario,
                "--goal",
                str(args.goal[0]),
                str(args.goal[1]),
                "--repeats",
                str(repeats),
                "--settle",
                str(args.settle),
                "--timeout",
                str(args.timeout),
                "--out",
                session_dir,
                "--label",
                planner,
            ]
            logger.log(f"Executing benchmark: {' '.join(cmd_ab_run)}")

            watchdog_s = repeats * (args.timeout + args.settle + 60.0)
            t_run_start = time.time()

            env_ab = dict(os.environ)
            env_ab["PYTHONUNBUFFERED"] = "1"
            p_ab = runner.popen(
                cmd_ab_run,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid if hasattr(os, "setsid") else None,
                env=env_ab,
            )
            active_procs.append(p_ab)

            line_queue: queue.Queue = queue.Queue()

            def _reader_thread(pipe: Any, q: Any) -> None:
                try:
                    for l in iter(pipe.readline, ""):
                        q.put(l)
                except Exception:
                    pass
                finally:
                    try:
                        pipe.close()
                    except Exception:
                        pass

            t_reader = threading.Thread(target=_reader_thread, args=(p_ab.stdout, line_queue))
            t_reader.daemon = True
            t_reader.start()

            ab_exit_code = -1
            while True:
                try:
                    line = line_queue.get(timeout=0.2)
                    logger.log(line.rstrip())
                except queue.Empty:
                    pass

                poll_code = p_ab.poll()
                if poll_code is not None:
                    while not line_queue.empty():
                        try:
                            rem_line = line_queue.get_nowait()
                            logger.log(rem_line.rstrip())
                        except queue.Empty:
                            break
                    ab_exit_code = poll_code
                    break

                if time.time() - t_run_start > watchdog_s:
                    logger.log(f"Watchdog triggered (exceeded {watchdog_s}s)! Terminating ab_run...")
                    runner.kill_pg(p_ab.pid, signal.SIGTERM)
                    runner.sleep(2.0)
                    runner.kill_pg(p_ab.pid, signal.SIGKILL)
                    ab_exit_code = -1
                    break

            end_iso = datetime.datetime.now().isoformat()
            logger.log(f"ab_run exited with code {ab_exit_code}")

            # Read rosparam values while ROS stack and master are still active (K1)
            rosparam_values = {}
            if planner == "dstar":
                rosparam_values = {
                    "/dstar_planner/filter_static_hits": runner.rosparam_get("/dstar_planner/filter_static_hits"),
                    "/dstar_planner/blocked_use_plan_radius": runner.rosparam_get("/dstar_planner/blocked_use_plan_radius"),
                    "/dstar_planner/verify_costmap": runner.rosparam_get("/dstar_planner/verify_costmap"),
                }
            nav_max_lin = runner.rosparam_get("/nav/max_lin")
            nav_inflation_radius = runner.rosparam_get("/nav/inflation_radius")

            # Flush bringup log and record cut offset before cleanup (L2)
            cut_offset: Optional[int] = None
            if bringup_f and not bringup_f.closed:
                bringup_f.flush()
            if os.path.exists(bringup_log_path):
                cut_offset = os.path.getsize(bringup_log_path)

            # 7. Cleanup started processes
            logger.log("Stopping ROS stack...")
            cleanup_group(runner, active_procs, logger)
            if bringup_f and not bringup_f.closed:
                bringup_f.close()

            # 8. Scan bringup log
            respawns = scan_log_for_respawns(bringup_log_path, max_bytes=cut_offset)
            logger.log(f"Bringup log respawns/crashes: {respawns}")

            # 9. Write meta_<planner>.json
            csv_path = os.path.join(session_dir, f"{planner}_runs.csv")
            summary_info = parse_csv_summary(csv_path)

            meta = {
                "git_commit": batch_git_commit,
                "git_status": batch_git_status,
                "git_diff_stat": batch_git_diff_stat,
                "file_sha256": batch_file_sha256,
                "planner": planner,
                "effective_repeats": repeats,
                "rviz": args.rviz,
                "cli_args": vars(args),
                "rosparam_values": rosparam_values,
                "nav_max_lin": nav_max_lin,
                "nav_inflation_radius": nav_inflation_radius,
                "python_version": sys.version,
                "start_time": start_iso,
                "end_time": end_iso,
                "ab_run_exit_code": ab_exit_code,
                "respawn_count": respawns,
            }
            meta_path = os.path.join(session_dir, f"meta_{planner}.json")
            with open(meta_path, "w", encoding="utf-8") as f_meta:
                json.dump(meta, f_meta, indent=2)

            planner_results[planner] = {
                "valid": summary_info["valid"],
                "invalid": summary_info["invalid"],
                "succeeded": summary_info["succeeded"],
                "timeout": summary_info["timeout"],
                "aborted": summary_info["aborted"],
                "reasons": summary_info["reasons"],
                "retries": summary_info["retries"],
                "respawns": respawns,
                "median_sim_wall": summary_info["median_sim_wall"],
                "mean_plan_s": summary_info["mean_plan_s"],
                "mean_plan_pct": summary_info["mean_plan_pct"],
                "csv": csv_path,
            }

            # Pause between planners
            if idx + 1 < len(planners_to_run):
                logger.log("Pausing 5 seconds before next planner...")
                runner.sleep(5.0)

        batch_completed = True
    finally:
        cleanup_group(runner, active_procs, logger)
        signal.signal(signal.SIGINT, orig_sigint)
        signal.signal(signal.SIGTERM, orig_sigterm)
        if bringup_f and not bringup_f.closed:
            bringup_f.close()
        if os.path.exists(pid_file):
            try:
                os.remove(pid_file)
            except OSError:
                pass
        if not batch_completed or fatal_error:
            handoff_path = write_handoff(session_dir, args, runner, batch_start_iso, logger)
            if handoff_path:
                print(f"Handoff: {handoff_path}")
            logger.close()

    if fatal_error:
        return 1

    # Generate Report (after both planners, skipped for dry runs)
    report_html_path: Optional[str] = None
    if not args.dry and planner_results:
        report_dir = os.path.join(session_dir, "report")
        csv_files = [res["csv"] for res in planner_results.values() if os.path.exists(res["csv"])]
        if csv_files:
            cmd_report = [sys.executable, ab_report_path, *csv_files, "--out", report_dir]
            logger.log(f"Generating report: {' '.join(cmd_report)}")
            res = runner.run(cmd_report, capture_output=True, text=True)
            report_html = os.path.join(report_dir, "report.html")
            if res.returncode == 0 and os.path.exists(report_html):
                report_html_path = report_html
                latest_report_file = os.path.join(out_dir, "latest_report.txt")
                with open(latest_report_file, "w", encoding="utf-8") as lr:
                    lr.write(report_html + "\n")
                logger.log(f"Report generated: {report_html}")
            else:
                logger.log(f"Report generation failed! Exit code: {res.returncode}")
                if res.stderr:
                    logger.log(f"ab_report stderr:\n{res.stderr.strip()}")
                if not os.path.exists(report_html):
                    logger.log(f"Expected report file missing: {report_html}")

    # SUMMARY block
    logger.log("\n================ SUMMARY ================")
    all_have_valid = True
    for pl in planners_to_run:
        info = planner_results.get(pl, {
            "valid": 0, "invalid": 0, "succeeded": 0, "timeout": 0, "aborted": 0,
            "reasons": {}, "retries": 0, "respawns": 0, "median_sim_wall": None,
            "mean_plan_s": None, "mean_plan_pct": None,
        })
        logger.log(f"Planner: {pl}")
        logger.log(f"  Valid runs:      {info['valid']}")
        logger.log(f"    Succeeded:     {info['succeeded']}")
        logger.log(f"    Timeout:       {info['timeout']}")
        logger.log(f"    Aborted:       {info['aborted']}")
        logger.log(f"  Invalid runs:    {info['invalid']}")
        reasons_str = ", ".join(f"{k}: {v}" for k, v in info['reasons'].items()) if info['reasons'] else "none"
        logger.log(f"  Invalid reasons: {reasons_str}")
        logger.log(f"  Settle retries:  {info['retries']}")
        logger.log(f"  Respawns:        {info['respawns']}")
        med_sw = info.get("median_sim_wall")
        sw_str = f"{med_sw:.2f}x" if med_sw is not None else "N/A"
        logger.log(f"  Sim/wall ratio:  {sw_str}")
        mean_s = info.get("mean_plan_s")
        mean_pct = info.get("mean_plan_pct")
        if mean_s is not None and mean_pct is not None:
            pt_str = f"{mean_s:.2f}s ({mean_pct:.1f}%)"
        elif mean_s is not None:
            pt_str = f"{mean_s:.2f}s"
        else:
            pt_str = "N/A"
        logger.log(f"  Planner time/run: {pt_str}")
        if info['valid'] == 0:
            all_have_valid = False

    if report_html_path:
        logger.log(f"Report: {report_html_path}")
    elif args.dry:
        logger.log("Report: skipped (--dry)")
    else:
        logger.log("Report: none")

    handoff_path = write_handoff(session_dir, args, runner, batch_start_iso, logger)
    if handoff_path:
        logger.log(f"Handoff: {handoff_path}")

    logger.log("=========================================\n")
    logger.close()

    if handoff_path:
        print(f"Handoff: {handoff_path}")

    return 0 if all_have_valid else 1


def main() -> None:
    args = parse_args()
    code = run_batch(args)
    sys.exit(code)


if __name__ == "__main__":
    main()
