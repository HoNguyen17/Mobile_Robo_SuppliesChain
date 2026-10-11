"""Headless Scenario Runner for UC6 Warehouse Robot.

Conforms to docs/test-plan.md §4.
"""

import os
import yaml
import time
import json
import logging
from typing import List, Optional

from warehouse_mission.models import RunSummary
from warehouse_eval.csv_exporter import CsvExporter

logger = logging.getLogger(__name__)


class ScenarioRunner:
    """Automates batch execution of warehouse scenarios."""

    def __init__(self, config_path: Optional[str] = None, output_dir: str = "results"):
        self.output_dir = output_dir
        self.exporter = CsvExporter(output_dir=self.output_dir)
        self.config = {}

        if config_path and os.path.exists(config_path):
            with open(config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
                if data and "scenarios" in data:
                    self.config = data["scenarios"]

    def get_scenario_config(self, scenario_id: str) -> dict:
        return self.config.get(
            scenario_id,
            {
                "name": scenario_id,
                "category": "Standard",
                "leg_timeout_s": 90.0,
                "criteria": {"success_rate": 0.90, "max_collisions": 0, "max_path_ratio": 1.3},
            },
        )

    def process_and_export(
        self, scenario_id: str, runs: List[RunSummary]
    ) -> (str, str):
        """Export runs and summary evaluation files."""
        cfg = self.get_scenario_config(scenario_id)
        runs_csv = self.exporter.export_runs(scenario_id, runs, category=cfg.get("category", "Standard"))
        summary_csv = self.exporter.export_summary(scenario_id, runs, criteria=cfg.get("criteria", {}))
        return runs_csv, summary_csv
