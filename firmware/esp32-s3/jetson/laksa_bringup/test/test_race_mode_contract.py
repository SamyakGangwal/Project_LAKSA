#!/usr/bin/env python3

"""Static gates for operator-free race mode in drive_supervisor."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RaceModeContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.supervisor = (ROOT / "scripts/drive_supervisor_node.py").read_text()
        cls.drive = (ROOT / "config/drive_supervisor.yaml").read_text()

    def test_operator_is_required_unless_explicitly_disabled(self):
        self.assertIn('"require_operator": True', self.supervisor)
        self.assertNotIn("require_operator: false", self.drive)

    def test_controller_staleness_only_gates_when_operator_required(self):
        self.assertIn("if self._require_operator:\n"
                      "            checks.insert(0, (self._last_joy_ns, self._joy_timeout_ns, \"XBOX_STALE\"))",
                      self.supervisor)
        self.assertIn("if not joy_fresh and self._require_operator:", self.supervisor)

    def test_manual_mode_still_brakes_without_a_controller(self):
        self.assertIn("elif not joy_fresh and (self._require_operator or not self._autonomous):",
                      self.supervisor)

    def test_hardware_estop_latches(self):
        self.assertIn('"hardware_estop_topic": "/laksa/estop_hw"', self.supervisor)
        self.assertIn('self._latch_estop("hardware emergency stop")', self.supervisor)

    def test_only_the_hardware_estop_latches(self):
        self.assertEqual(self.supervisor.count("self._latch_estop("), 1)

    def test_other_autonomy_gates_remain(self):
        for reason in ("ESP32_STATE_STALE", "OBSTACLE_SOURCE_STALE: LiDAR scan", "ODOM_STALE",
                       "OBSTACLE_SOURCE_STALE: ZED obstacle cloud", "explorer reported BLOCKED"):
            self.assertIn(reason, self.supervisor)


if __name__ == "__main__":
    unittest.main()
