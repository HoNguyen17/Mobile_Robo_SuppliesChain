"""Category motion profiles manager for UC6 Warehouse Robot.

Conforms to docs/architecture.md §9.
"""

import os
import yaml
import logging
from typing import Dict, Any, Optional

from warehouse_mission.models import (
    CATEGORY_STANDARD,
    CATEGORY_HEAVY,
    CATEGORY_FRAGILE,
    EVENT_WARNING,
    RunEvent,
)

logger = logging.getLogger(__name__)

DEFAULT_PROFILES = {
    CATEGORY_STANDARD: {
        "max_lin": 0.26,
        "inflation_radius": 0.35,
        "description": "Waffle Pi nominal top speed for standard cargo",
    },
    CATEGORY_HEAVY: {
        "max_lin": 0.15,
        "inflation_radius": 0.35,
        "description": "Reduced speed with caution for heavy cargo",
    },
    CATEGORY_FRAGILE: {
        "max_lin": 0.10,
        "inflation_radius": 0.50,
        "description": "Slow speed with wide safety margin for fragile cargo",
    },
}


class MotionProfileManager:
    """Manages speed and inflation profiles for different cargo categories."""

    def __init__(self, config_path: Optional[str] = None):
        self.profiles = dict(DEFAULT_PROFILES)
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    if data and "profiles" in data:
                        for cat, vals in data["profiles"].items():
                            if cat in self.profiles:
                                self.profiles[cat].update(vals)
            except Exception as e:
                logger.warning("Failed to load motion_profiles.yaml from %s: %s", config_path, e)

        self.current_category = CATEGORY_STANDARD

    def get_profile(self, category: str) -> Dict[str, Any]:
        """Return profile dict for given category, defaulting to Fragile if unknown."""
        return self.profiles.get(category, self.profiles[CATEGORY_FRAGILE])

    def apply_profile(self, category: str, set_param_fn=None) -> Optional[RunEvent]:
        """Apply speed and inflation parameters for given category.

        If set_param_fn is None, attempts to use rospy.set_param.
        If applying fails, reverts to Fragile and returns a warning RunEvent.
        """
        warning_event = None
        target_category = category

        if target_category not in self.profiles:
            warning_event = RunEvent(
                event_type=EVENT_WARNING,
                sim_time_s=0.0,
                payload={"msg": f"Unknown category '{category}', reverting to Fragile"},
            )
            target_category = CATEGORY_FRAGILE

        profile = self.profiles[target_category]

        def _do_set(cat: str, prof: Dict[str, Any]) -> bool:
            try:
                if set_param_fn:
                    set_param_fn("/nav/max_lin", float(prof["max_lin"]))
                    set_param_fn("/nav/inflation_radius", float(prof["inflation_radius"]))
                else:
                    import rospy
                    rospy.set_param("/nav/max_lin", float(prof["max_lin"]))
                    rospy.set_param("/nav/inflation_radius", float(prof["inflation_radius"]))
                self.current_category = cat
                return True
            except Exception as ex:
                logger.error("Failed to set ROS params for %s: %s", cat, ex)
                return False

        if not _do_set(target_category, profile):
            # Revert to Fragile per architecture.md §9 Rule 4
            fallback_prof = self.profiles[CATEGORY_FRAGILE]
            _do_set(CATEGORY_FRAGILE, fallback_prof)
            warning_event = RunEvent(
                event_type=EVENT_WARNING,
                sim_time_s=0.0,
                payload={"msg": f"Failed to apply {category}, reverted to Fragile"},
            )

        return warning_event
