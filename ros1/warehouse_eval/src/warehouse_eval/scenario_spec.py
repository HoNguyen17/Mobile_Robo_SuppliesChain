"""Scenario specification for the dynamic-obstacle scenarios (ADR-019). Pure Python, no ROS.

`config/scenarios.yaml` is the single source. `export()` resolves it into plain data (ROS metres, absolute
waypoints) that Unity reads from `Assets/StreamingAssets/scenarios.json`. The same maths as Unity's `NpcPath.cs`
is kept here (`position_at`, `seeded_unit`) so it is tested without Unity; the C# test vectors are the same.

    python3 -m warehouse_eval.scenario_spec --write   # regenerate scenarios.json
    python3 -m warehouse_eval.scenario_spec --check   # exit 1 when scenarios.json is stale
"""

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import yaml

SCHEMA_VERSION = 1
MODES = ("once", "pingpong")
TRIGGERS = ("reset", "robot_departs")
FOREVER = -1.0

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.normpath(os.path.join(_HERE, "..", "..", "..", ".."))
DEFAULT_YAML = os.path.join(REPO_DIR, "ros1", "warehouse_eval", "config", "scenarios.yaml")
DEFAULT_JSON = os.path.join(REPO_DIR, "WarehouseProjectURP", "Assets", "StreamingAssets", "scenarios.json")


# ---------------- reset message: "S-06?seed=7" ----------------

def parse_reset_id(text: str) -> Tuple[str, Optional[int]]:
    """Split a /sim/reset_scenario string into (scenario id, seed). No "?" means no seed (the old protocol)."""
    raw = str(text).strip()
    if "?" not in raw:
        return raw, None
    scenario, _, query = raw.partition("?")
    scenario = scenario.strip()
    parts = [p for p in query.split("&") if p]
    if not scenario or len(parts) != 1 or not parts[0].startswith("seed="):
        raise ValueError("reset message must be 'ID' or 'ID?seed=N', got %r" % raw)
    value = parts[0][len("seed="):]
    if not value.isdigit():
        raise ValueError("seed must be a non-negative integer, got %r" % value)
    return scenario, int(value)


def reset_message(scenario: str, seed: Optional[int]) -> str:
    """The string ab_run publishes. Without a seed it is the bare ID, byte for byte the old message."""
    if seed is None:
        return scenario
    if seed < 0:
        raise ValueError("seed must be >= 0")
    return "%s?seed=%d" % (scenario, seed)


# ---------------- seeded offsets ----------------

def seeded_unit(seed: int, salt: int) -> float:
    """Deterministic value in [0, 1) from (seed, salt). The same 32-bit integer hash is in NpcPath.cs."""
    x = (seed * 0x9E3779B1 + salt * 0x85EBCA6B + 0x165667B1) & 0xFFFFFFFF
    x ^= x >> 15
    x = (x * 0x2C1B3C6D) & 0xFFFFFFFF
    x ^= x >> 12
    x = (x * 0x297A2D39) & 0xFFFFFFFF
    x ^= x >> 15
    return x / 4294967296.0


def worker_start_delay_s(worker: Dict[str, Any], seed: int, index: int) -> float:
    return float(worker["start_delay_s"]) + float(worker["start_jitter_s"]) * seeded_unit(seed, index)


# ---------------- worker path (mirror of NpcPath.cs) ----------------

def _cycle(worker: Dict[str, Any]) -> List[Dict[str, float]]:
    wps = worker["waypoints"]
    seq = list(wps)
    if worker["mode"] == "pingpong":
        seq = list(wps) + list(reversed(wps[1:-1]))
    return seq


def cycle_duration_s(worker: Dict[str, Any]) -> float:
    """Seconds for one pass (once) or one full cycle (pingpong). Infinite when a worker waits for ever."""
    seq = _cycle(worker)
    speed = float(worker["speed_mps"])
    closed = worker["mode"] == "pingpong"
    total = 0.0
    for i, wp in enumerate(seq):
        if not closed and i == len(seq) - 1:
            break  # `once` holds the last waypoint, whatever its dwell
        if wp["dwell_s"] < 0.0:
            return math.inf
        nxt = seq[(i + 1) % len(seq)]
        total += wp["dwell_s"] + math.hypot(nxt["x"] - wp["x"], nxt["y"] - wp["y"]) / speed
    return total


def position_at(worker: Dict[str, Any], t: float) -> Tuple[float, float]:
    """(x, y) in ROS metres, t seconds after the worker started. `once` holds the last waypoint afterwards."""
    seq = _cycle(worker)
    speed = float(worker["speed_mps"])
    closed = worker["mode"] == "pingpong"
    t = max(0.0, float(t))
    if closed:
        period = cycle_duration_s(worker)
        if math.isfinite(period) and period > 0.0:
            t = math.fmod(t, period)
    for i, wp in enumerate(seq):
        last = i == len(seq) - 1
        dwell = wp["dwell_s"]
        if last and not closed:
            return wp["x"], wp["y"]
        if dwell < 0.0 or t < dwell:
            return wp["x"], wp["y"]
        t -= dwell
        nxt = seq[(i + 1) % len(seq)]
        leg = math.hypot(nxt["x"] - wp["x"], nxt["y"] - wp["y"]) / speed
        if t < leg:
            k = t / leg if leg > 0.0 else 1.0
            return wp["x"] + (nxt["x"] - wp["x"]) * k, wp["y"] + (nxt["y"] - wp["y"]) * k
        t -= leg
    return seq[0]["x"], seq[0]["y"]


# ---------------- yaml -> export ----------------

def load_config(path: str = DEFAULT_YAML) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "scenarios" not in data:
        raise ValueError("%s has no 'scenarios'" % path)
    return data


def route_of(config: Dict[str, Any]) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    route = config["route"]
    return (float(route["start"][0]), float(route["start"][1])), (float(route["goal"][0]), float(route["goal"][1]))


def scenario_goal(config: Dict[str, Any], scenario: str) -> Tuple[float, float]:
    """The goal of a scenario: its own `goal` or the route goal."""
    cfg = config["scenarios"].get(scenario) or {}
    goal = cfg.get("goal") or config["route"]["goal"]
    return float(goal[0]), float(goal[1])


def is_spec_scenario(config: Dict[str, Any], scenario: str) -> bool:
    cfg = config["scenarios"].get(scenario)
    return isinstance(cfg, dict) and "workers" in cfg


def resolve_waypoint(config: Dict[str, Any], wp: Dict[str, Any]) -> Dict[str, float]:
    dwell = float(wp.get("dwell_s", 0.0))
    if "x" in wp and "y" in wp:
        x, y = float(wp["x"]), float(wp["y"])
    elif "f" in wp:
        (sx, sy), (gx, gy) = route_of(config)
        dx, dy = gx - sx, gy - sy
        length = math.hypot(dx, dy)
        f = float(wp["f"])
        off = float(wp.get("offset_m", 0.0))
        nx, ny = -dy / length, dx / length  # left of the travel direction
        x, y = sx + f * dx + off * nx, sy + f * dy + off * ny
    else:
        raise ValueError("waypoint needs x and y, or f: %r" % (wp,))
    return {"x": round(x, 4), "y": round(y, 4), "dwell_s": dwell}


def resolve_worker(config: Dict[str, Any], worker: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "name": str(worker["name"]),
        "speed_mps": float(worker["speed_mps"]),
        "radius_m": float(worker["radius_m"]),
        "mode": str(worker.get("mode", "once")),
        "start_trigger": str(worker.get("start_trigger", "robot_departs")),
        "start_delay_s": float(worker.get("start_delay_s", 0.0)),
        "start_jitter_s": float(worker.get("start_jitter_s", 0.0)),
        "waypoints": [resolve_waypoint(config, wp) for wp in worker["waypoints"]],
    }


def validate(config: Dict[str, Any]) -> List[str]:
    """Problems found in the dynamic scenarios; an empty list means the file is usable."""
    problems = []  # type: List[str]
    try:
        (sx, sy), (gx, gy) = route_of(config)
        if math.hypot(gx - sx, gy - sy) < 1.0:
            problems.append("route start and goal are less than 1 m apart")
    except (KeyError, TypeError, ValueError, IndexError):
        return ["route.start and route.goal are required"]
    for sid, cfg in config["scenarios"].items():
        if not isinstance(cfg, dict) or "workers" not in cfg:
            continue
        if not isinstance(cfg["workers"], list):
            problems.append("%s: workers must be a list" % sid)
            continue
        if "layout_checked" not in cfg:
            problems.append("%s: layout_checked is required" % sid)
        names = set()
        for i, w in enumerate(cfg["workers"]):
            where = "%s worker %d" % (sid, i)
            try:
                rw = resolve_worker(config, w)
            except (KeyError, TypeError, ValueError) as exc:
                problems.append("%s: %s" % (where, exc))
                continue
            if rw["name"] in names:
                problems.append("%s: duplicate name %s" % (where, rw["name"]))
            names.add(rw["name"])
            if rw["mode"] not in MODES:
                problems.append("%s: mode %s not in %s" % (where, rw["mode"], MODES))
            if rw["start_trigger"] not in TRIGGERS:
                problems.append("%s: start_trigger %s not in %s" % (where, rw["start_trigger"], TRIGGERS))
            if not 0.05 <= rw["speed_mps"] <= 1.5:
                problems.append("%s: speed_mps %.2f outside 0.05..1.5" % (where, rw["speed_mps"]))
            if not 0.05 <= rw["radius_m"] <= 0.5:
                problems.append("%s: radius_m %.2f outside 0.05..0.5" % (where, rw["radius_m"]))
            if rw["start_delay_s"] < 0.0 or rw["start_jitter_s"] < 0.0:
                problems.append("%s: start_delay_s and start_jitter_s must be >= 0" % where)
            if len(rw["waypoints"]) < 2:
                problems.append("%s: at least 2 waypoints" % where)
            for wp in rw["waypoints"]:
                if not (math.isfinite(wp["x"]) and math.isfinite(wp["y"]) and abs(wp["x"]) < 30 and abs(wp["y"]) < 30):
                    problems.append("%s: waypoint (%s, %s) is not a plausible map position" % (where, wp["x"], wp["y"]))
                if wp["dwell_s"] < 0.0 and wp["dwell_s"] != FOREVER:
                    problems.append("%s: dwell_s must be >= 0 or -1" % where)
            if rw["mode"] == "pingpong" and any(wp["dwell_s"] < 0.0 for wp in rw["waypoints"]):
                problems.append("%s: a pingpong worker cannot wait for ever" % where)
    return problems


def export(config: Dict[str, Any]) -> Dict[str, Any]:
    """The data Unity reads. Only scenarios with a `workers` key are spec-driven."""
    problems = validate(config)
    if problems:
        raise ValueError("scenarios.yaml is invalid:\n  " + "\n  ".join(problems))
    scenarios = []
    for sid, cfg in config["scenarios"].items():
        if not isinstance(cfg, dict) or "workers" not in cfg:
            continue
        scenarios.append({
            "id": sid,
            "name": str(cfg.get("name", sid)),
            "workers": [resolve_worker(config, w) for w in cfg["workers"]],
        })
    return {"schema": SCHEMA_VERSION, "scenarios": scenarios}


def dumps(data: Dict[str, Any]) -> str:
    return json.dumps(data, indent=2) + "\n"


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Export scenarios.yaml to Unity's scenarios.json")
    parser.add_argument("--yaml", default=DEFAULT_YAML)
    parser.add_argument("--json", default=DEFAULT_JSON)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    text = dumps(export(load_config(args.yaml)))
    if args.write:
        os.makedirs(os.path.dirname(args.json), exist_ok=True)
        with open(args.json, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print("wrote %s" % args.json)
        return 0
    try:
        with open(args.json, "r", encoding="utf-8", newline="") as f:
            current = f.read()
    except OSError:
        current = None
    if current != text:
        print("%s is stale: run python3 -m warehouse_eval.scenario_spec --write" % args.json, file=sys.stderr)
        return 1
    print("scenarios.json is up to date")
    return 0


if __name__ == "__main__":
    sys.exit(main())
