"""Pure acceptance checks. These do NOT exercise ROS 2 or Webots."""
import unittest

from scripts.rj_int_01_assess import assess


class IntegrationAssessmentTests(unittest.TestCase):
    def _evidence(self):
        return {
            "joint_states": [
                {"phase": "startup", "angle_rad": 0.0, "effort_nm": 0.0, "elapsed_wall_s": 1.0},
                {"phase": "move", "angle_rad": 0.14, "effort_nm": 0.06, "elapsed_wall_s": 2.0},
                {"phase": "move", "angle_rad": 0.595, "effort_nm": 0.01, "elapsed_wall_s": 8.0},
                {"phase": "timeout", "angle_rad": 0.59, "effort_nm": -0.02, "elapsed_wall_s": 8.2},
            ],
            "safe_targets": [
                {"phase": "move", "value": 0.6, "elapsed_wall_s": 2.0},
                {"phase": "timeout", "value": 0.6, "elapsed_wall_s": 8.1},
                {"phase": "timeout", "value": 0.0, "elapsed_wall_s": 9.0},
            ],
            "safety_states": [
                {"phase": "timeout", "value": "NORMAL", "elapsed_wall_s": 8.1},
                {"phase": "timeout", "value": "DAMPING_ONLY", "elapsed_wall_s": 9.0},
            ],
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

    def test_torque_above_the_actuator_limit_is_rejected(self):
        evidence = self._evidence()
        from scripts.rj_int_01_assess import _committed_actuator_limit_nm
        evidence["joint_states"][1]["effort_nm"] = _committed_actuator_limit_nm() + 1.0
        result = assess(evidence)
        self.assertFalse(result["passed"])
        self.assertTrue(any("actuator limit" in item for item in result["failures"]))

    def test_joint_that_does_not_settle_is_rejected(self):
        evidence = self._evidence()
        evidence["joint_states"][2]["angle_rad"] = 0.2
        result = assess(evidence)
        self.assertFalse(result["passed"])
        self.assertTrue(any("settle" in item for item in result["failures"]))

    def test_late_timeout_degradation_is_rejected(self):
        evidence = self._evidence()
        evidence["safe_targets"][-1]["elapsed_wall_s"] = 12.0
        evidence["safety_states"][-1]["elapsed_wall_s"] = 12.0
        result = assess(evidence)
        self.assertFalse(result["passed"])
        self.assertTrue(any("timeout bound" in item for item in result["failures"]))


if __name__ == "__main__":
    unittest.main()
