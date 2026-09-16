from __future__ import annotations

import unittest
from pathlib import Path

from actc.speed_calibration import (
    BASE_SPEED_SCALE_BY_BAND,
    build_speed_calibration_schedule,
)
from tools.auto_start import build_command


class SpeedCalibrationTests(unittest.TestCase):
    def test_schedule_is_monotonic_for_target_bands(self) -> None:
        schedule = build_speed_calibration_schedule()
        self.assertEqual(10, len(schedule))
        previous = BASE_SPEED_SCALE_BY_BAND
        for candidate in schedule:
            current = candidate.speed_scale_by_band
            self.assertGreaterEqual(current[3], previous[3])
            self.assertGreaterEqual(current[6], previous[6])
            for index in (0, 1, 2, 4, 5, 7, 8):
                self.assertEqual(previous[index], current[index])
            previous = current
        self.assertAlmostEqual(1.310, previous[3])
        self.assertAlmostEqual(1.335, previous[6])

    def test_final_lap_increases_both_target_bands(self) -> None:
        schedule = build_speed_calibration_schedule()
        before = schedule[-2].speed_scale_by_band
        final = schedule[-1].speed_scale_by_band
        self.assertAlmostEqual(before[3] + 0.005, final[3])
        self.assertAlmostEqual(before[6] + 0.005, final[6])

    def test_auto_start_forwards_speed_calibration_flag(self) -> None:
        command = build_command(
            Path("lap.csv"),
            lateral_controller="lmpc",
            resume_tuning=Path("tuning.json"),
            mpc_model=Path("model.json"),
            autotune=False,
            speed_calibration=True,
        )
        self.assertIn("--speed-calibration", command)


if __name__ == "__main__":
    unittest.main()
