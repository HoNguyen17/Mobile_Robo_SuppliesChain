"""Mission orchestrator state machine for UC6 Warehouse Robot.

Conforms to docs/architecture.md §6 and §7.
"""

from typing import Optional, Callable
import logging

from warehouse_mission.models import (
    Task,
    RunSummary,
    LEG_SHELF,
    LEG_DROPOFF,
    OUTCOME_SUCCESS,
    OUTCOME_FAIL,
    FAIL_TIMEOUT,
    FAIL_NAV_ABORTED,
    FAIL_ITEM_ERROR,
    CATEGORY_STANDARD,
)
from warehouse_mission.motion_profile import MotionProfileManager
from warehouse_mission.metrics_collector import MetricsCollector
from warehouse_mission.task_manager import TaskManager

logger = logging.getLogger(__name__)

# State constants
STATE_IDLE = "Idle"
STATE_TO_SHELF = "ToShelf"
STATE_AT_SHELF = "AtShelf"
STATE_TO_DROPOFF = "ToDropoff"
STATE_AT_DROPOFF = "AtDropoff"
STATE_REPORT = "Report"


class MissionOrchestrator:
    """Manages the episode state machine from task dispatch to run reporting."""

    def __init__(
        self,
        task_manager: TaskManager,
        motion_profile: MotionProfileManager,
        metrics_collector: MetricsCollector,
        send_goal_fn: Optional[Callable] = None,
        cancel_nav_fn: Optional[Callable] = None,
        attach_item_fn: Optional[Callable] = None,
        release_item_fn: Optional[Callable] = None,
        leg_timeout_s: float = 90.0,
        dwell_time_s: float = 3.0,
    ):
        self.task_manager = task_manager
        self.motion_profile = motion_profile
        self.metrics_collector = metrics_collector

        self.send_goal_fn = send_goal_fn or (lambda pose: None)
        self.cancel_nav_fn = cancel_nav_fn or (lambda: None)
        self.attach_item_fn = attach_item_fn or (lambda item_id: True)
        self.release_item_fn = release_item_fn or (lambda: True)

        self.leg_timeout_s = leg_timeout_s
        self.dwell_time_s = dwell_time_s

        self.state = STATE_IDLE
        self.current_task: Optional[Task] = None
        self.leg_start_time: float = 0.0
        self.dwell_start_time: float = 0.0

        self.final_outcome: str = OUTCOME_SUCCESS
        self.final_fail_reason: str = ""
        self.latest_summary: Optional[RunSummary] = None

    def step(self, current_x: float, current_y: float, sim_time_s: float):
        """Execute one step of the state machine clock."""
        if self.state == STATE_IDLE:
            # Ready for next task
            pass

        elif self.state == STATE_TO_SHELF:
            if sim_time_s - self.leg_start_time > self.leg_timeout_s:
                logger.warning("Shelf leg timed out (%.1f s)", sim_time_s - self.leg_start_time)
                self.cancel_nav_fn()
                self._abort_run(OUTCOME_FAIL, FAIL_TIMEOUT, sim_time_s)

        elif self.state == STATE_AT_SHELF:
            if sim_time_s - self.dwell_start_time >= self.dwell_time_s:
                # Dwell complete, attach item and transition to dropoff leg
                ok = self.attach_item_fn(self.current_task.item_id)
                if not ok:
                    logger.error("Failed to attach item %d", self.current_task.item_id)
                    self._abort_run(OUTCOME_FAIL, FAIL_ITEM_ERROR, sim_time_s)
                    return

                # Apply category profile for drop-off leg
                warn = self.motion_profile.apply_profile(self.current_task.category)
                if warn:
                    self.metrics_collector.events.append(warn)

                self.metrics_collector.mark_leg(LEG_DROPOFF)
                self.send_goal_fn(self.current_task.dropoff_pose)
                self.leg_start_time = sim_time_s
                self.state = STATE_TO_DROPOFF
                logger.info("Item attached, heading to dropoff: %s", self.current_task.dropoff_pose)

        elif self.state == STATE_TO_DROPOFF:
            if sim_time_s - self.leg_start_time > self.leg_timeout_s:
                logger.warning("Dropoff leg timed out (%.1f s)", sim_time_s - self.leg_start_time)
                self.cancel_nav_fn()
                self._abort_run(OUTCOME_FAIL, FAIL_TIMEOUT, sim_time_s)

        elif self.state == STATE_AT_DROPOFF:
            if sim_time_s - self.dwell_start_time >= self.dwell_time_s:
                self.release_item_fn()
                logger.info("Item released at dropoff, completing run")
                self.final_outcome = OUTCOME_SUCCESS
                self.final_fail_reason = ""
                self._report_and_reset(sim_time_s)

    def dispatch_next_task(
        self, current_x: float, current_y: float, sim_time_s: float, scenario_id: str = "S-00"
    ) -> bool:
        """Start a new task if idle."""
        if self.state != STATE_IDLE:
            logger.warning("Cannot dispatch: state is %s", self.state)
            return False

        task, _ = self.task_manager.get_next_task(scenario_id)
        if not task:
            logger.warning("No task available")
            return False

        self.current_task = task
        # Apply Standard profile for shelf leg
        self.motion_profile.apply_profile(CATEGORY_STANDARD)
        self.metrics_collector.start_run(task, current_x, current_y, sim_time_s)
        self.metrics_collector.mark_leg(LEG_SHELF)

        self.send_goal_fn(task.shelf_pose)
        self.leg_start_time = sim_time_s
        self.state = STATE_TO_SHELF
        logger.info("Dispatched task %d: heading to shelf %s", task.task_id, task.shelf_pose)
        return True

    def on_nav_result(self, outcome: str, reason: str, sim_time_s: float):
        """Callback when /nav/leg_result message is received."""
        logger.info("Nav result received in state %s: outcome=%s reason=%s", self.state, outcome, reason)
        if self.state == STATE_TO_SHELF:
            if outcome == "succeeded":
                self.state = STATE_AT_SHELF
                self.dwell_start_time = sim_time_s
                logger.info("Arrived at shelf, starting %0.1fs dwell", self.dwell_time_s)
            else:
                self._abort_run(OUTCOME_FAIL, FAIL_NAV_ABORTED, sim_time_s)

        elif self.state == STATE_TO_DROPOFF:
            if outcome == "succeeded":
                self.state = STATE_AT_DROPOFF
                self.dwell_start_time = sim_time_s
                logger.info("Arrived at dropoff, starting %0.1fs dwell", self.dwell_time_s)
            else:
                self._abort_run(OUTCOME_FAIL, FAIL_NAV_ABORTED, sim_time_s)

    def _abort_run(self, outcome: str, fail_reason: str, sim_time_s: float):
        """Abort current run and trigger reporting."""
        self.final_outcome = outcome
        self.final_fail_reason = fail_reason
        self.state = STATE_REPORT
        self._report_and_reset(sim_time_s)

    def _report_and_reset(self, sim_time_s: float):
        """Generate summary and report to task manager."""
        summary = self.metrics_collector.finish_run(
            outcome=self.final_outcome,
            fail_reason=self.final_fail_reason,
            sim_time_s=sim_time_s,
        )
        self.latest_summary = summary
        self.task_manager.report_run(summary)
        logger.info("Run finished: outcome=%s duration=%.1fs", summary.outcome, summary.duration_s)
        self.state = STATE_IDLE
        self.current_task = None
