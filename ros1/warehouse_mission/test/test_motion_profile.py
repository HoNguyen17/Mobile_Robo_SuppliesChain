import unittest
from warehouse_mission.motion_profile import MotionProfileManager
from warehouse_mission.models import CATEGORY_STANDARD, CATEGORY_HEAVY, CATEGORY_FRAGILE, EVENT_WARNING


class TestMotionProfile(unittest.TestCase):
    def setUp(self):
        self.mgr = MotionProfileManager()

    def test_default_profiles_values(self):
        std = self.mgr.get_profile(CATEGORY_STANDARD)
        self.assertEqual(std["max_lin"], 0.26)
        self.assertEqual(std["inflation_radius"], 0.35)

        hvy = self.mgr.get_profile(CATEGORY_HEAVY)
        self.assertEqual(hvy["max_lin"], 0.15)

        fra = self.mgr.get_profile(CATEGORY_FRAGILE)
        self.assertEqual(fra["max_lin"], 0.10)
        self.assertEqual(fra["inflation_radius"], 0.50)

    def test_apply_profile_mock(self):
        params = {}

        def mock_set(param_name, val):
            params[param_name] = val

        warn = self.mgr.apply_profile(CATEGORY_HEAVY, set_param_fn=mock_set)
        self.assertIsNone(warn)
        self.assertEqual(params["/nav/max_lin"], 0.15)
        self.assertEqual(params["/nav/inflation_radius"], 0.35)

    def test_unknown_category_reverts_to_fragile(self):
        params = {}

        def mock_set(param_name, val):
            params[param_name] = val

        warn = self.mgr.apply_profile("Unknown", set_param_fn=mock_set)
        self.assertIsNotNone(warn)
        self.assertEqual(warn.event_type, EVENT_WARNING)
        self.assertEqual(params["/nav/max_lin"], 0.10)
        self.assertEqual(params["/nav/inflation_radius"], 0.50)


if __name__ == "__main__":
    unittest.main()
