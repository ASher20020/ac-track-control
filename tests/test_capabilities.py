from __future__ import annotations

import unittest

from actc.capabilities import (
    acceleration_capability_mps2,
    braking_capability_mps2,
    planning_braking_capability_mps2,
)


class LongitudinalCapabilityTests(unittest.TestCase):
    def test_acceleration_capability_decreases_with_speed(self) -> None:
        low_speed = acceleration_capability_mps2(40.0 / 3.6)
        mid_speed = acceleration_capability_mps2(100.0 / 3.6)
        high_speed = acceleration_capability_mps2(180.0 / 3.6)
        self.assertGreater(low_speed, mid_speed)
        self.assertGreater(mid_speed, high_speed)
        self.assertAlmostEqual(6.2, mid_speed)
        self.assertAlmostEqual(3.05, high_speed)

    def test_braking_capability_stays_high_at_high_speed(self) -> None:
        low_speed = braking_capability_mps2(50.0 / 3.6)
        mid_speed = braking_capability_mps2(100.0 / 3.6)
        high_speed = braking_capability_mps2(180.0 / 3.6)
        self.assertGreaterEqual(low_speed, 15.0)
        self.assertGreaterEqual(mid_speed, 15.0)
        self.assertGreater(high_speed, 16.7)

    def test_planning_braking_uses_conservative_cap(self) -> None:
        speed_mps = 180.0 / 3.6
        planning = planning_braking_capability_mps2(speed_mps)
        self.assertGreaterEqual(planning, 16.5)
        self.assertLessEqual(planning, 17.0)


if __name__ == "__main__":
    unittest.main()
