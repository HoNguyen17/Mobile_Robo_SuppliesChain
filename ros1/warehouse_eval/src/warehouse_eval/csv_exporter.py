"""CSV Exporter for UC6 Warehouse Robot Evaluation.

Conforms to docs/test-plan.md §5.
"""

import os
import csv
import math
from typing import List, Dict, Any
from warehouse_mission.models import RunSummary


class CsvExporter:
    """Exports run summaries and evaluation results to CSV."""

    RUNS_HEADER = [
        "run_id",
        "scenario_id",
        "task_id",
        "category",
        "planner",
        "outcome",
        "fail_reason",
        "collisions",
        "replans",
        "recoveries",
        "min_clearance_m",
        "path_length_m",
        "baseline_m",
        "path_ratio",
        "duration_s",
        "dropoff_mean_speed_mps",
        "started_at",
        "ended_at",
    ]

    def __init__(self, output_dir: str = "results"):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

    def export_runs(
        self, scenario_id: str, runs: List[RunSummary], category: str = "Standard"
    ) -> str:
        """Export list of runs to <scenario>_runs.csv."""
        filepath = os.path.join(self.output_dir, f"{scenario_id}_runs.csv")
        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(self.RUNS_HEADER)
            for idx, r in enumerate(runs, start=1):
                clr = f"{r.min_clearance_m:.3f}" if not math.isnan(r.min_clearance_m) else ""
                pr = f"{r.path_ratio:.3f}" if not math.isnan(r.path_ratio) else ""
                spd = f"{r.dropoff_mean_speed_mps:.3f}" if not math.isnan(r.dropoff_mean_speed_mps) else ""

                writer.writerow(
                    [
                        idx,
                        r.scenario_id,
                        r.task_id,
                        category,
                        r.planner,
                        r.outcome,
                        r.fail_reason,
                        r.collisions,
                        r.replans,
                        r.recoveries,
                        clr,
                        f"{r.path_length_m:.2f}",
                        f"{r.baseline_m:.2f}",
                        pr,
                        f"{r.duration_s:.1f}",
                        spd,
                        r.started_at,
                        r.ended_at,
                    ]
                )
        return filepath

    def export_summary(
        self, scenario_id: str, runs: List[RunSummary], criteria: Dict[str, Any]
    ) -> str:
        """Evaluate runs against acceptance criteria and export <scenario>_summary.csv."""
        filepath = os.path.join(self.output_dir, f"{scenario_id}_summary.csv")
        n = len(runs)
        if n == 0:
            return filepath

        successes = sum(1 for r in runs if r.outcome == "success")
        success_rate = successes / n
        total_collisions = sum(r.collisions for r in runs)

        valid_clearances = [
            r.min_clearance_m for r in runs if not math.isnan(r.min_clearance_m)
        ]
        worst_clearance = min(valid_clearances) if valid_clearances else float("nan")

        valid_ratios = [r.path_ratio for r in runs if not math.isnan(r.path_ratio)]
        worst_ratio = max(valid_ratios) if valid_ratios else float("nan")

        target_sr = criteria.get("success_rate", 0.90)
        target_col = criteria.get("max_collisions", 0)
        target_clr = criteria.get("min_clearance_m", None)
        target_pr = criteria.get("max_path_ratio", None)

        sr_pass = success_rate >= target_sr
        col_pass = total_collisions <= target_col
        clr_pass = True if target_clr is None or math.isnan(worst_clearance) else worst_clearance >= target_clr
        pr_pass = True if target_pr is None or math.isnan(worst_ratio) else worst_ratio <= target_pr

        overall_pass = sr_pass and col_pass and clr_pass and pr_pass

        header = [
            "scenario_id",
            "runs_count",
            "success_rate_pct",
            "target_success_rate_pct",
            "success_rate_status",
            "total_collisions",
            "target_collisions",
            "collision_status",
            "worst_clearance_m",
            "target_clearance_m",
            "clearance_status",
            "worst_path_ratio",
            "target_path_ratio",
            "path_ratio_status",
            "overall_status",
        ]

        row = [
            scenario_id,
            n,
            f"{success_rate * 100:.1f}%",
            f"{target_sr * 100:.1f}%",
            "PASS" if sr_pass else "FAIL",
            total_collisions,
            target_col,
            "PASS" if col_pass else "FAIL",
            f"{worst_clearance:.3f}" if not math.isnan(worst_clearance) else "N/A",
            f"{target_clr:.2f}" if target_clr is not None else "N/A",
            "PASS" if clr_pass else "FAIL",
            f"{worst_ratio:.3f}" if not math.isnan(worst_ratio) else "N/A",
            f"{target_pr:.2f}" if target_pr is not None else "N/A",
            "PASS" if pr_pass else "FAIL",
            "PASS" if overall_pass else "FAIL",
        ]

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerow(row)

        return filepath
