"""PlanScheduler: runs the planners' plans one at a time, off the ROS callback threads.

No ROS imports (ADR-016). Before, `on_goal`, `on_replan` and `on_scan` each called
`plan_for_goal` on their own callback thread, so two plans could overlap, a plan inside
`on_scan` blocked scan processing, and a cancelled leg could still get a path.

Rules:
- One worker thread runs one plan at a time.
- At most one request waits. A new request replaces the waiting one when its priority is
  the same or higher (GOAL > REPLAN > SCAN_TRIGGER); a lower one is dropped, because the
  waiting plan already covers it.
- `cancel()` drops the waiting request and marks the running plan as cancelled, so the
  planner must not publish its result.
- A new GOAL makes every earlier request stale for the same reason: its result is for an
  old goal.
- An exception in a plan is reported to `on_error` and the worker keeps going.
"""
import threading
from typing import Any, Callable, Optional

GOAL = "goal"
REPLAN = "replan"
SCAN_TRIGGER = "scan_trigger"

PRIORITY = {SCAN_TRIGGER: 0, REPLAN: 1, GOAL: 2}


class Ticket:
    """One request as the plan function sees it."""

    def __init__(self, scheduler: "PlanScheduler", reason: str, payload: Any,
                 generation: int, goal_seq: int) -> None:
        self._scheduler = scheduler
        self.reason = reason
        self.payload = payload
        self.generation = generation
        self.goal_seq = goal_seq

    @property
    def cancelled(self) -> bool:
        """True when the result must be dropped: a cancel or a newer goal came after this request."""
        return self._scheduler._is_stale(self)


class PlanScheduler:
    def __init__(
        self,
        run: Callable[[Ticket], None],
        on_error: Optional[Callable[[BaseException], None]] = None,
        name: str = "plan-scheduler",
    ) -> None:
        self._run = run
        self._on_error = on_error
        self._cond = threading.Condition()
        self._pending: Optional[Ticket] = None
        self._running = False
        self._generation = 0  # +1 at every cancel()
        self._goal_seq = 0  # +1 at every GOAL request
        self._stopped = False
        self._thread = threading.Thread(target=self._loop, name=name, daemon=True)
        self._thread.start()

    # ---------------- requests ----------------
    def post(self, reason: str, payload: Any = None) -> bool:
        """Ask for a plan. True if the request is now waiting, False if a waiting request covers it."""
        if reason not in PRIORITY:
            raise ValueError("unknown plan reason: %r" % (reason,))
        with self._cond:
            if reason == GOAL:
                self._goal_seq += 1
            pending = self._pending
            if pending is not None and PRIORITY[reason] < PRIORITY[pending.reason]:
                return False
            self._pending = Ticket(self, reason, payload, self._generation, self._goal_seq)
            self._cond.notify_all()
            return True

    def cancel(self) -> bool:
        """Drop the waiting request and mark the running plan as cancelled. True if something was dropped."""
        with self._cond:
            dropped = self._pending is not None or self._running
            self._pending = None
            self._generation += 1
            self._cond.notify_all()
            return dropped

    # ---------------- state ----------------
    @property
    def busy(self) -> bool:
        with self._cond:
            return self._running or self._pending is not None

    def wait_idle(self, timeout: float = 5.0) -> bool:
        """Block until nothing is running or waiting. False on timeout."""
        with self._cond:
            return self._cond.wait_for(lambda: not self._running and self._pending is None, timeout)

    def stop(self, timeout: float = 2.0) -> None:
        with self._cond:
            self._stopped = True
            self._pending = None
            self._cond.notify_all()
        self._thread.join(timeout)

    def _is_stale(self, ticket: Ticket) -> bool:
        with self._cond:
            return ticket.generation != self._generation or ticket.goal_seq != self._goal_seq

    # ---------------- worker ----------------
    def _loop(self) -> None:
        while True:
            with self._cond:
                self._cond.wait_for(lambda: self._stopped or self._pending is not None)
                if self._stopped:
                    return
                ticket = self._pending
                self._pending = None
                self._running = True
            try:
                if not ticket.cancelled:
                    self._run(ticket)
            except Exception as exc:  # the worker must survive a failed plan
                if self._on_error is not None:
                    try:
                        self._on_error(exc)
                    except Exception:
                        pass
            finally:
                with self._cond:
                    self._running = False
                    self._cond.notify_all()
