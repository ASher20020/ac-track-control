from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from actc.cli import lap_log_writer, _write_lateral_scan_plan
from actc.lateral_scan import (
    LapScanMetrics,
    build_lateral_refine_schedule,
    build_lateral_scan_schedule,
    compute_lap_scan_metrics,
    select_refinement_confirmations,
)
from actc.mpc import MpcConfig
from tools.auto_start import build_command


class LateralScanTests(unittest.TestCase):
    def test_schedule_has_twenty_unique_candidates(self) -> None:
        schedule = build_lateral_scan_schedule()
        self.assertEqual(20, len(schedule))
        self.assertEqual(20, len({candidate.as_dict()["design_index"] for candidate in schedule}))

    def test_candidate_apply_changes_only_scan_weights(self) -> None:
        config = MpcConfig()
        original_frame = config.coordinate_frame
        original_horizon = config.horizon
        candidate = build_lateral_scan_schedule()[0]

        candidate.apply(config)

        self.assertEqual(original_frame, config.coordinate_frame)
        self.assertEqual(original_horizon, config.horizon)
        self.assertEqual(candidate.q_lateral, config.q_lateral)
        self.assertEqual(candidate.q_heading, config.q_heading)
        self.assertEqual(candidate.q_yaw_rate, config.q_yaw_rate)
        self.assertEqual(
            candidate.q_lateral_velocity,
            config.q_lateral_velocity,
        )
        self.assertEqual(candidate.r_steer, config.r_steer)
        self.assertEqual(candidate.rd_steer, config.rd_steer)

    def test_scan_metrics_include_yaw_and_sideslip(self) -> None:
        metrics = compute_lap_scan_metrics(
            lateral_errors_m=[0.1, 0.2, 0.3],
            heading_errors_rad=[0.02, -0.03, 0.04],
            yaw_rate_errors_rad_s=[0.01, -0.02, 0.03],
            sideslip_abs_rad=[0.01, 0.02, 0.03],
            raw_steer_commands=[0.0, 0.01, 0.02, 0.01],
            dt=0.02,
        )
        self.assertGreater(metrics.composite_score, 0.0)
        self.assertGreater(metrics.yaw_rate_error_p95_rad_s, 0.0)
        self.assertGreater(metrics.sideslip_p95_rad, 0.0)
        self.assertGreater(metrics.raw_steer_jerk_rms, 0.0)

    def test_auto_start_forwards_lateral_scan_flag(self) -> None:
        command = build_command(
            Path("lap.csv"),
            lateral_controller="lmpc",
            resume_tuning=Path("tuning.json"),
            mpc_model=Path("model.json"),
            autotune=False,
            lateral_scan=True,
        )
        self.assertIn("--lateral-scan", command)
        self.assertNotIn("--autotune", command)

    def test_auto_start_forwards_lateral_refine_flag(self) -> None:
        command = build_command(
            Path("lap.csv"),
            lateral_controller="lmpc",
            resume_tuning=Path("tuning.json"),
            mpc_model=Path("model.json"),
            autotune=False,
            lateral_refine=True,
        )
        self.assertIn("--lateral-refine", command)
        self.assertNotIn("--lateral-scan", command)

    def test_auto_start_forwards_line_offset_envelope(self) -> None:
        command = build_command(
            Path("lap.csv"),
            lateral_controller="lmpc",
            resume_tuning=Path("tuning.json"),
            mpc_model=Path("model.json"),
            line_offset_envelope=Path("offsets.json"),
        )
        index = command.index("--line-offset-envelope")
        self.assertEqual("offsets.json", command[index + 1])

    def test_refine_schedule_has_warmup_six_explore_four_confirm(self) -> None:
        schedule = build_lateral_refine_schedule()
        self.assertEqual(11, len(schedule))
        self.assertIsNotNone(schedule[0])
        self.assertEqual(100, schedule[0].design_index)
        self.assertEqual(
            [101, 102, 103, 104, 105, 106],
            [
                candidate.design_index
                for candidate in schedule[1:7]
            ],
        )
        self.assertTrue(all(candidate is None for candidate in schedule[7:]))

    def test_refine_selection_uses_abba_confirmation(self) -> None:
        metrics = {
            index: LapScanMetrics(
                lateral_error_rms_m=float(index),
                heading_error_p95_rad=0.1,
                yaw_rate_error_rms_rad_s=0.1,
                yaw_rate_error_p95_rad_s=0.2,
                sideslip_rms_rad=0.01,
                sideslip_p95_rad=0.03,
                raw_steer_step_p95=0.005,
                raw_steer_jerk_rms=0.1,
                shake_score=0.03,
                sideslip_score_rad=0.02,
                composite_score=float(index),
            )
            for index in range(101, 107)
        }
        confirmations = select_refinement_confirmations(metrics)
        self.assertEqual(
            [101, 102, 102, 101],
            [candidate.design_index for candidate in confirmations],
        )

    def test_lap_log_includes_scan_metrics(self) -> None:
        with TemporaryDirectory() as directory:
            path = Path(directory) / "lap.csv"
            handle, writer = lap_log_writer(path)
            try:
                fields = set(writer.fieldnames)
            finally:
                handle.close()
        self.assertIn("yaw_rate_ref_rad_s", fields)
        self.assertIn("yaw_rate_error_rad_s", fields)
        self.assertIn("sideslip_rad", fields)
        self.assertIn("longitudinal_preview_accel", fields)
        self.assertIn("longitudinal_preview_min_accel", fields)

    def test_refine_plan_serializes_unresolved_confirmation_slots(self) -> None:
        with TemporaryDirectory() as directory:
            path = Path(directory) / "lap.csv"
            _write_lateral_scan_plan(
                path,
                build_lateral_refine_schedule(),
            )
            payload = (Path(directory) / "lap_scan_plan.json").read_text(
                encoding="utf-8"
            )
        self.assertIn('"candidates"', payload)
        self.assertIn("null", payload)


if __name__ == "__main__":
    unittest.main()
