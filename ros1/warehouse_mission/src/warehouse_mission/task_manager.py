"""Task manager for UC6 Warehouse Robot.

Handles task retrieval and run reporting with MySQL 8 and local JSON fallback.
Conforms to docs/architecture.md §5, §11 and docs/data-model.md.
"""

import os
import json
import time
import glob
import logging
from typing import Optional, Tuple
from dataclasses import asdict

from warehouse_mission.models import (
    Task,
    Pose2D,
    RunSummary,
    CATEGORY_STANDARD,
)

logger = logging.getLogger(__name__)

FALLBACK_BUFFER_DIR = os.path.expanduser("~/.warehouse/pending")


class TaskManager:
    """Manages tasks from MySQL or fallback local JSON storage."""

    def __init__(
        self,
        db_host: str = "127.0.0.1",
        db_port: int = 3306,
        db_user: str = "warehouse",
        db_pass: str = "warehouse",
        db_name: str = "warehouse",
        buffer_dir: str = FALLBACK_BUFFER_DIR,
    ):
        self.db_host = db_host
        self.db_port = db_port
        self.db_user = db_user
        self.db_pass = db_pass
        self.db_name = db_name
        self.buffer_dir = buffer_dir

        os.makedirs(self.buffer_dir, exist_ok=True)
        self._db_conn = None

    def _get_db_connection(self):
        """Attempt to establish a MySQL connection using pymysql or mysql.connector."""
        try:
            import pymysql
            return pymysql.connect(
                host=self.db_host,
                port=self.db_port,
                user=self.db_user,
                password=self.db_pass,
                database=self.db_name,
                autocommit=True,
                connect_timeout=2,
            )
        except Exception:
            return None

    def _get_default_task(self, scenario_id: str = "S-00") -> Task:
        """Bundled default task per architecture.md §11 (target shelf to HomePoint)."""
        return Task(
            task_id=1,
            item_id=1,
            category=CATEGORY_STANDARD,
            shelf_pose=Pose2D(x=-3.75, y=-3.13, theta=0.0),
            dropoff_pose=Pose2D(x=6.32, y=5.87, theta=0.0),
            scenario_id=scenario_id,
        )

    def get_next_task(self, scenario_id: str = "S-00") -> Tuple[Task, bool]:
        """Fetch next pending task.

        Returns (task, from_fallback).
        """
        # Try uploading any buffered runs first
        self.flush_pending_runs()

        conn = self._get_db_connection()
        if not conn:
            logger.info("MySQL unreachable, using bundled default task (from_fallback=True)")
            return self._get_default_task(scenario_id), True

        try:
            with conn.cursor() as cur:
                # Query a pending task
                query = (
                    "SELECT t.task_id, t.item_id, c.name, "
                    "s.pose_x, s.pose_y, s.pose_theta, "
                    "t.dropoff_x, t.dropoff_y, t.dropoff_theta "
                    "FROM tasks t "
                    "JOIN items i ON t.item_id = i.item_id "
                    "JOIN categories c ON i.category_id = c.category_id "
                    "JOIN shelf_slots s ON i.slot_id = s.slot_id "
                    "WHERE t.status = 'pending' AND t.scenario_id = %s "
                    "ORDER BY t.task_id ASC LIMIT 1 FOR UPDATE"
                )
                cur.execute(query, (scenario_id,))
                row = cur.fetchone()
                if row:
                    task_id, item_id, cat, sx, sy, sth, dx, dy, dth = row
                    cur.execute(
                        "UPDATE tasks SET status = 'in_progress' WHERE task_id = %s",
                        (task_id,),
                    )
                    task = Task(
                        task_id=task_id,
                        item_id=item_id,
                        category=cat,
                        shelf_pose=Pose2D(float(sx), float(sy), float(sth)),
                        dropoff_pose=Pose2D(float(dx), float(dy), float(dth)),
                        scenario_id=scenario_id,
                    )
                    return task, False
        except Exception as e:
            logger.warning("Error fetching task from DB: %s", e)
        finally:
            try:
                conn.close()
            except Exception:
                pass

        logger.info("No DB tasks pending, returning default task")
        return self._get_default_task(scenario_id), True

    def report_run(self, summary: RunSummary) -> bool:
        """Report run results to MySQL, or buffer locally in JSON if DB unreachable."""
        conn = self._get_db_connection()
        if not conn:
            return self._buffer_run(summary)

        try:
            with conn.cursor() as cur:
                # Insert run summary
                insert_run = (
                    "INSERT INTO runs (task_id, outcome, fail_reason, collisions, "
                    "min_clearance_m, path_ratio, duration_s, start_x, start_y, "
                    "planner, dropoff_mean_speed_mps) "
                    "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
                )
                clr = None if summary.min_clearance_m != summary.min_clearance_m else summary.min_clearance_m
                pr = None if summary.path_ratio != summary.path_ratio else summary.path_ratio
                spd = None if summary.dropoff_mean_speed_mps != summary.dropoff_mean_speed_mps else summary.dropoff_mean_speed_mps

                cur.execute(
                    insert_run,
                    (
                        summary.task_id,
                        summary.outcome,
                        summary.fail_reason or None,
                        summary.collisions,
                        clr,
                        pr,
                        summary.duration_s,
                        summary.start_x,
                        summary.start_y,
                        summary.planner,
                        spd,
                    ),
                )
                run_id = cur.lastrowid

                # Insert events
                if summary.events:
                    insert_ev = (
                        "INSERT INTO run_events (run_id, event_type, payload) VALUES (%s, %s, %s)"
                    )
                    for ev in summary.events:
                        cur.execute(
                            insert_ev,
                            (run_id, ev.event_type, json.dumps(ev.payload)),
                        )

                # Mark task done
                cur.execute("UPDATE tasks SET status = 'done' WHERE task_id = %s", (summary.task_id,))
                return True
        except Exception as e:
            logger.warning("Failed to insert run into DB: %s, buffering to file", e)
            return self._buffer_run(summary)
        finally:
            try:
                conn.close()
            except Exception:
                pass

    def _buffer_run(self, summary: RunSummary) -> bool:
        """Buffer run summary to a local JSON file."""
        ts = int(time.time())
        filename = f"run_{summary.task_id}_{ts}.json"
        path = os.path.join(self.buffer_dir, filename)
        try:
            data = asdict(summary)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            logger.info("Buffered run summary to %s", path)
            return True
        except Exception as e:
            logger.error("Failed to buffer run summary to file: %s", e)
            return False

    def flush_pending_runs(self) -> int:
        """Attempt to upload buffered JSON files to DB when connected."""
        files = glob.glob(os.path.join(self.buffer_dir, "run_*.json"))
        if not files:
            return 0

        conn = self._get_db_connection()
        if not conn:
            return 0

        flushed = 0
        try:
            for filepath in files:
                with open(filepath, "r", encoding="utf-8") as f:
                    data = json.load(f)
                # Reconstruct summary
                summary = RunSummary(
                    task_id=data.get("task_id", 1),
                    scenario_id=data.get("scenario_id", "S-00"),
                    planner=data.get("planner", "astar"),
                    outcome=data.get("outcome", "success"),
                    fail_reason=data.get("fail_reason", ""),
                    collisions=data.get("collisions", 0),
                    min_clearance_m=data.get("min_clearance_m", 0.0),
                    path_length_m=data.get("path_length_m", 0.0),
                    baseline_m=data.get("baseline_m", 0.0),
                    duration_s=data.get("duration_s", 0.0),
                    start_x=data.get("start_x", 0.0),
                    start_y=data.get("start_y", 0.0),
                    dropoff_mean_speed_mps=data.get("dropoff_mean_speed_mps", 0.0),
                )
                if self.report_run(summary):
                    os.remove(filepath)
                    flushed += 1
        except Exception as e:
            logger.warning("Error flushing pending runs: %s", e)
        finally:
            try:
                conn.close()
            except Exception:
                pass
        return flushed
