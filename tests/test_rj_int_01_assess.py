"""Pure acceptance checks. These do NOT exercise ROS 2 or Webots."""
import unittest

from scripts.rj_int_01_assess import assess


class IntegrationAssessmentTests(unittest.TestCase):
    def _evidence(self):
        return {
            "joint_states": [
                {"phase": "startup", "angle_rad": 0.0, "effort_nm": 0.0},
                {"phase": "move", "angle_rad": 0.14, "effort_nm": 0.06},
                {"phase": "timeout", "angle_rad": 0.11, "effort_nm": -0.02},
            ],
            "safe_targets": [
                {"phase": "move", "value": 0.6},
                {"phase": "timeout", "value": 0.0},
            ],
            "safety_states": [{"phase": "timeout", "value": "DAMPING_ONLY"}],
        }

    def test_complete_observations_are_accepted(self):
        result = assess(self._evidence())
        self.assertTrue(result["passed"], result["failures"])

    def test_empty_observations_are_rejected(self):
        self.assertFalse(assess({})["passed"])

    def test_no_joint_motion_is_rejected(self):
        evidence = self._evidence()
        evidence["joint_states"][1]["angle_rad"] = 0.0
        evidence["joint_states"][2]["angle_rad"] = 0.0
        self.assertFalse(assess(evidence)["passed"])

    def test_missing_safety_timeout_is_rejected(self):
        evidence = self._evidence()
        evidence["safety_states"] = []
        self.assertFalse(assess(evidence)["passed"])

    def test_nonfinite_telemetry_is_rejected(self):
        evidence = self._evidence()
        evidence["joint_states"][1]["effort_nm"] = float("nan")
        self.assertFalse(assess(evidence)["passed"])


if __name__ == "__main__":
    unittest.main()
