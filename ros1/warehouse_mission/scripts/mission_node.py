#!/usr/bin/env python3
"""ROS Node: mission

Coordinates task execution, profiles, metrics and reporting for UC6 Warehouse Robot.
Conforms to docs/architecture.md §3, §5, §6.
"""

import rospy
import json
from std_msgs.msg import String, Empty, Int32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock

from warehouse_mission.models import Pose2D
from warehouse_mission.task_manager import TaskManager
from warehouse_mission.motion_profile import MotionProfileManager
from warehouse_mission.metrics_collector import MetricsCollector
from warehouse_mission.mission_orchestrator import MissionOrchestrator, STATE_IDLE


class MissionNode:
    """ROS node wrapper for the UC6 warehouse mission state machine."""

    def __init__(self):
        rospy.init_node("mission")

        import argparse
        parser = argparse.ArgumentParser(description="Warehouse Mission Node")
        parser.add_argument("--scenario", "-s", default=None, help="Scenario ID (e.g. S-00)")
        parser.add_argument("--auto-start", "-a", action="store_true", default=False, help="Auto-dispatch task")
        args, _ = parser.parse_known_args()

        cli_scenario = args.scenario
        cli_auto = args.auto_start or (cli_scenario is not None)

        self.scenario_id = cli_scenario if cli_scenario else rospy.get_param("~scenario_id", "S-00")
        self.dwell_time_s = float(rospy.get_param("~dwell_time_s", 3.0))
        self.leg_timeout_s = float(rospy.get_param("~leg_timeout_s", 90.0))
        self.auto_start = bool(rospy.get_param("~auto_start", cli_auto))

        self.current_pose = None
        self.sim_time_s = 0.0
        self.has_clock = False
        self.has_pose = False

        # Component instances
        self.task_mgr = TaskManager()
        self.motion_mgr = MotionProfileManager()
        self.metrics_coll = MetricsCollector(planner_name="astar")

        # Publishers
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_cancel = rospy.Publisher("/nav/cancel", Empty, queue_size=1)
        self.pub_attach = rospy.Publisher("/sim/attach_item", Int32, queue_size=1)
        self.pub_release = rospy.Publisher("/sim/release_item", Empty, queue_size=1)

        # Build orchestrator
        self.orchestrator = MissionOrchestrator(
            task_manager=self.task_mgr,
            motion_profile=self.motion_mgr,
            metrics_collector=self.metrics_coll,
            send_goal_fn=self.send_goal,
            cancel_nav_fn=self.cancel_nav,
            attach_item_fn=self.attach_item,
            release_item_fn=self.release_item,
            leg_timeout_s=self.leg_timeout_s,
            dwell_time_s=self.dwell_time_s,
        )

        # Subscribers
        rospy.Subscriber("/clock", Clock, self.on_clock)
        rospy.Subscriber("/robot/pose", PoseStamped, self.on_robot_pose)
        rospy.Subscriber("/scan", LaserScan, self.on_scan)
        rospy.Subscriber("/sim/collision", String, self.on_collision)
        rospy.Subscriber("/planned_path", Path, self.on_path)
        rospy.Subscriber("/nav/leg_result", String, self.on_leg_result)
        rospy.Subscriber("/nav/event", String, self.on_nav_event)
        rospy.Subscriber("/mission/start", String, self.on_mission_start)

        rospy.loginfo("Mission node initialized: scenario=%s auto_start=%s", self.scenario_id, self.auto_start)

    def on_clock(self, msg: Clock):
        self.sim_time_s = msg.clock.to_sec()
        self.has_clock = True

    def on_robot_pose(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.current_pose = (x, y)
        self.has_pose = True
        self.metrics_coll.on_pose(x, y, self.sim_time_s)

    def on_scan(self, msg: LaserScan):
        self.metrics_coll.on_scan(msg.ranges)

    def on_collision(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.metrics_coll.on_collision(
                other_tag=data.get("other_tag", "Obstacle"),
                sim_time_s=float(data.get("sim_time_s", self.sim_time_s)),
                x=float(data.get("x", 0.0)),
                y=float(data.get("y", 0.0)),
            )
        except Exception as e:
            rospy.logwarn("Failed to parse /sim/collision: %s", e)

    def on_path(self, msg: Path):
        self.metrics_coll.on_planned_path(self.sim_time_s)

    def on_leg_result(self, msg: String):
        try:
            data = json.loads(msg.data)
            outcome = data.get("outcome", "aborted")
            reason = data.get("reason", "")
            self.orchestrator.on_nav_result(outcome, reason, self.sim_time_s)
        except Exception as e:
            rospy.logwarn("Failed to parse /nav/leg_result: %s", e)

    def on_nav_event(self, msg: String):
        try:
            data = json.loads(msg.data)
            ev_type = data.get("type", "recovery")
            self.metrics_coll.on_nav_event(ev_type, self.sim_time_s, data)
        except Exception as e:
            rospy.logwarn("Failed to parse /nav/event: %s", e)

    def on_mission_start(self, msg: String):
        scen = msg.data.strip() or self.scenario_id
        if self.has_pose:
            self.orchestrator.dispatch_next_task(
                self.current_pose[0], self.current_pose[1], self.sim_time_s, scen
            )

    def send_goal(self, pose: Pose2D):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.from_sec(self.sim_time_s) if self.has_clock else rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = pose.x
        goal.pose.position.y = pose.y
        goal.pose.orientation.w = 1.0
        self.pub_goal.publish(goal)
        rospy.loginfo("Published navigation goal: (%.2f, %.2f)", pose.x, pose.y)

    def cancel_nav(self):
        self.pub_cancel.publish(Empty())
        rospy.loginfo("Published /nav/cancel")

    def attach_item(self, item_id: int) -> bool:
        msg = Int32()
        msg.data = item_id
        self.pub_attach.publish(msg)
        rospy.loginfo("Published /sim/attach_item: %d", item_id)
        return True

    def release_item(self) -> bool:
        self.pub_release.publish(Empty())
        rospy.loginfo("Published /sim/release_item")
        return True

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.has_clock and self.has_pose:
                if self.auto_start and self.orchestrator.state == STATE_IDLE:
                    self.orchestrator.dispatch_next_task(
                        self.current_pose[0], self.current_pose[1], self.sim_time_s, self.scenario_id
                    )
                self.orchestrator.step(self.current_pose[0], self.current_pose[1], self.sim_time_s)
            else:
                rospy.loginfo_throttle(
                    4.0,
                    "Mission standby: waiting for Unity simulation (clock=%s, pose=%s)...",
                    self.has_clock,
                    self.has_pose,
                )
            rate.sleep()


if __name__ == "__main__":
    node = MissionNode()
    node.spin()
