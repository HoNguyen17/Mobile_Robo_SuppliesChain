"""Unit tests for plan_scheduler: one plan at a time, coalescing, cancel, errors."""
import threading
import unittest

import support  # noqa: F401  (puts scripts/ on sys.path)
from plan_scheduler import GOAL, REPLAN, SCAN_TRIGGER, PlanScheduler

WAIT = 5.0  # s: generous, the tests wait on events, not on time


class Recorder:
    """A plan function that records its tickets and can be held until released."""

    def __init__(self, hold_first=False):
        self.tickets = []
        self.started = threading.Event()
        self.release = threading.Event()
        self.lock = threading.Lock()
        self.active = 0
        self.max_active = 0
        self.hold_first = hold_first

    def __call__(self, ticket):
        with self.lock:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
            first = not self.tickets
            self.tickets.append(ticket)
        self.started.set()
        if self.hold_first and first:
            self.release.wait(WAIT)
        with self.lock:
            self.active -= 1

    @property
    def reasons(self):
        return [t.reason for t in self.tickets]


class PlanSchedulerTest(unittest.TestCase):
    def make(self, hold_first=False, on_error=None):
        rec = Recorder(hold_first)
        sched = PlanScheduler(rec, on_error=on_error)
        self.addCleanup(sched.stop)
        self.addCleanup(rec.release.set)
        return sched, rec

    def test_a_posted_request_runs_with_its_reason_and_payload(self):
        sched, rec = self.make()
        self.assertTrue(sched.post(GOAL, "goal-1"))
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(rec.reasons, [GOAL])
        self.assertEqual(rec.tickets[0].payload, "goal-1")
        self.assertFalse(rec.tickets[0].cancelled)

    def test_unknown_reason_is_refused(self):
        sched, _ = self.make()
        with self.assertRaises(ValueError):
            sched.post("whenever")

    def test_plans_never_overlap(self):
        sched, rec = self.make()
        for i in range(50):
            sched.post(GOAL if i % 3 == 0 else REPLAN, i)
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(rec.max_active, 1)

    def test_requests_posted_from_many_threads_never_overlap(self):
        sched, rec = self.make()

        def spam():
            for _ in range(30):
                sched.post(SCAN_TRIGGER)
                sched.post(REPLAN)

        threads = [threading.Thread(target=spam) for _ in range(6)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(rec.max_active, 1)
        self.assertGreaterEqual(len(rec.tickets), 1)

    def test_requests_while_busy_coalesce_into_one_waiting_plan(self):
        sched, rec = self.make(hold_first=True)
        sched.post(GOAL, "g")
        self.assertTrue(rec.started.wait(WAIT))
        self.assertTrue(sched.post(REPLAN))
        self.assertTrue(sched.post(REPLAN))  # same priority: the newer one replaces the waiting one
        rec.release.set()
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(rec.reasons, [GOAL, REPLAN])

    def test_a_lower_priority_request_is_dropped_by_a_waiting_higher_one(self):
        sched, rec = self.make(hold_first=True)
        sched.post(REPLAN)
        self.assertTrue(rec.started.wait(WAIT))
        self.assertTrue(sched.post(GOAL, "g"))
        self.assertFalse(sched.post(SCAN_TRIGGER))
        self.assertFalse(sched.post(REPLAN))
        rec.release.set()
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(rec.reasons, [REPLAN, GOAL])
        self.assertEqual(rec.tickets[1].payload, "g")

    def test_a_higher_priority_request_replaces_a_waiting_lower_one(self):
        sched, rec = self.make(hold_first=True)
        sched.post(REPLAN)
        self.assertTrue(rec.started.wait(WAIT))
        self.assertTrue(sched.post(SCAN_TRIGGER))
        self.assertTrue(sched.post(REPLAN))
        self.assertTrue(sched.post(GOAL, "g"))
        rec.release.set()
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(rec.reasons, [REPLAN, GOAL])

    def test_cancel_drops_the_waiting_request(self):
        sched, rec = self.make(hold_first=True)
        sched.post(GOAL, "g1")
        self.assertTrue(rec.started.wait(WAIT))
        sched.post(REPLAN)
        self.assertTrue(sched.cancel())
        rec.release.set()
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(rec.reasons, [GOAL])  # the waiting REPLAN never ran

    def test_cancel_marks_the_running_plan_as_cancelled(self):
        sched, rec = self.make(hold_first=True)
        sched.post(GOAL, "g1")
        self.assertTrue(rec.started.wait(WAIT))
        ticket = rec.tickets[0]
        self.assertFalse(ticket.cancelled)
        sched.cancel()
        self.assertTrue(ticket.cancelled)

    def test_cancel_with_nothing_to_cancel_returns_false(self):
        sched, rec = self.make()
        self.assertFalse(sched.cancel())

    def test_a_request_after_a_cancel_runs_normally(self):
        sched, rec = self.make()
        sched.cancel()
        sched.post(GOAL, "g2")
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(rec.reasons, [GOAL])
        self.assertFalse(rec.tickets[0].cancelled)

    def test_a_new_goal_makes_the_running_plan_stale(self):
        sched, rec = self.make(hold_first=True)
        sched.post(GOAL, "old")
        self.assertTrue(rec.started.wait(WAIT))
        old = rec.tickets[0]
        sched.post(GOAL, "new")
        self.assertTrue(old.cancelled)
        rec.release.set()
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual([t.payload for t in rec.tickets], ["old", "new"])
        self.assertFalse(rec.tickets[1].cancelled)

    def test_a_replan_for_the_current_goal_is_not_stale(self):
        sched, rec = self.make(hold_first=True)
        sched.post(GOAL, "g")
        self.assertTrue(rec.started.wait(WAIT))
        sched.post(REPLAN)
        self.assertFalse(rec.tickets[0].cancelled)
        rec.release.set()
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(rec.reasons, [GOAL, REPLAN])
        self.assertFalse(rec.tickets[1].cancelled)

    def test_a_failing_plan_is_reported_and_the_worker_goes_on(self):
        errors = []
        ran = []

        def run(ticket):
            ran.append(ticket.payload)
            if ticket.payload == "bad":
                raise RuntimeError("boom")

        sched = PlanScheduler(run, on_error=errors.append)
        self.addCleanup(sched.stop)
        sched.post(GOAL, "bad")
        self.assertTrue(sched.wait_idle(WAIT))
        sched.post(GOAL, "good")
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(ran, ["bad", "good"])
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], RuntimeError)

    def test_a_failing_error_handler_does_not_kill_the_worker(self):
        ran = []

        def run(ticket):
            ran.append(ticket.payload)
            raise RuntimeError("boom")

        def bad_handler(_exc):
            raise ValueError("handler broke too")

        sched = PlanScheduler(run, on_error=bad_handler)
        self.addCleanup(sched.stop)
        sched.post(GOAL, 1)
        self.assertTrue(sched.wait_idle(WAIT))
        sched.post(GOAL, 2)
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertEqual(ran, [1, 2])

    def test_wait_idle_times_out_while_a_plan_is_held(self):
        sched, rec = self.make(hold_first=True)
        sched.post(GOAL, "g")
        self.assertTrue(rec.started.wait(WAIT))
        self.assertTrue(sched.busy)
        self.assertFalse(sched.wait_idle(0.05))
        rec.release.set()
        self.assertTrue(sched.wait_idle(WAIT))
        self.assertFalse(sched.busy)

    def test_stop_ends_the_worker_thread(self):
        sched, rec = self.make()
        sched.stop()
        self.assertFalse(sched._thread.is_alive())


if __name__ == "__main__":
    unittest.main()
