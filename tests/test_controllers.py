from __future__ import annotations

import math
import unittest
from pathlib import Path
from types import SimpleNamespace

from actc.controllers import (
    HeadingController,
    HeadingControllerConfig,
    SpeedController,
    SpeedControllerConfig,
    wrap_pi,
)
from actc.lap import StanleyConfig, StanleyController
from actc.longitudinal import (
    LongitudinalMpcConfig,
    LongitudinalMpcController,
)
from actc.cli import (
    _load_mpc_parameters,
    _load_tuning_state,
    _build_longitudinal_preview,
    _braking_limited_speed_kmh,
    _filter_target_speed,
    _next_target_speed,
    _next_lap_tuning,
    _next_lmpc_tuning,
    _speed_band,
    _speed_scale_kmh,
    _scaled_path_speed_kmh,
    _tune_gain_maps,
)
from actc.mpc import MpcConfig
from actc.track import (
    TrackPath,
    _entry_lateral_accel_weights,
    _speed_limits_from_geometry,
    physics_heading_to_world,
    wrap_pi as track_wrap_pi,
)
from tools.auto_start import build_command


class SpeedControllerTests(unittest.TestCase):
    def test_requests_throttle_below_target(self) -> None:
        controller = SpeedController(SpeedControllerConfig(target_kmh=60.0))
        throttle, brake = controller.step(20.0, 0.01)
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)

    def test_requests_brake_above_target(self) -> None:
        controller = SpeedController(SpeedControllerConfig(target_kmh=60.0))
        throttle, brake = controller.step(80.0, 0.01)
        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)

    def test_releases_pedals_inside_deadband(self) -> None:
        controller = SpeedController(SpeedControllerConfig(target_kmh=60.0))
        for _ in range(100):
            controller.step(60.0, 0.01)
        throttle, brake = controller.step(60.1, 0.01)
        self.assertLess(throttle, 0.05)
        self.assertLess(brake, 0.05)

    def test_longitudinal_mpc_uses_previous_command_in_free_response(
        self,
    ) -> None:
        config = LongitudinalMpcConfig(
            horizon=30,
            dt=0.03,
            response_tau_s=0.10,
            q_speed=12.0,
            q_accel=3.0,
            r_accel=1.2,
            r_jerk=70.0,
            max_accel_mps2=3.0,
            max_brake_mps2=10.0,
            max_jerk_mps3=7.6,
            max_brake_jerk_mps3=20.0,
        )
        controller = LongitudinalMpcController(config)
        controller.reset(2.0)
        speed_mps = 50.0 / 3.6
        result = controller.step(
            speed_mps=speed_mps,
            acceleration_mps2=2.0,
            reference_speed_mps=[speed_mps] * 31,
            acceleration_limit_mps2=3.0,
            braking_limit_mps2=10.0,
            lateral_accel_g=0.0,
        )
        self.assertLess(result.acceleration_command_mps2, 1.9)

    def test_predicts_speed_and_starts_braking_early(self) -> None:
        controller = SpeedController(
            SpeedControllerConfig(
                target_kmh=30.0,
                prediction_time_s=0.6,
            )
        )
        for speed in (10.0, 15.0, 20.0, 25.0, 28.0):
            controller.step(speed, 0.1)
        throttle, brake = controller.step(29.0, 0.1)
        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)


class HeadingControllerTests(unittest.TestCase):
    def test_positive_error_produces_positive_steering(self) -> None:
        controller = HeadingController(
            HeadingControllerConfig(),
            target_heading_rad=math.radians(10.0),
        )
        steer = controller.step(0.0, 0.0, 0.01)
        self.assertGreater(steer, 0.0)

    def test_wrap_pi(self) -> None:
        self.assertAlmostEqual(wrap_pi(math.radians(370.0)), math.radians(10.0))
        self.assertAlmostEqual(wrap_pi(math.radians(-370.0)), math.radians(-10.0))


class TrackControllerGeometryTests(unittest.TestCase):
    def test_auto_start_uses_configured_braking_capacity(self) -> None:
        command = build_command(
            Path("lap.csv"),
            lateral_controller="lmpc",
            resume_tuning=Path("tuning.json"),
            mpc_model=Path("model.json"),
            braking_accel_mps2=8.34,
        )
        index = command.index("--braking-accel")
        self.assertEqual(command[index + 1], "8.34")
        speed_index = command.index("--max-speed-kmh")
        self.assertEqual(command[speed_index + 1], "280.0")
        lateral_index = command.index("--lateral-accel")
        self.assertEqual(command[lateral_index + 1], "6.867")
        horizon_index = command.index("--longitudinal-horizon")
        self.assertEqual(command[horizon_index + 1], "45")
        dt_index = command.index("--longitudinal-dt")
        self.assertEqual(command[dt_index + 1], "0.03")

    def test_ac_heading_maps_to_world_heading_by_positive_90_degrees(self) -> None:
        self.assertAlmostEqual(
            physics_heading_to_world(-math.pi / 2.0),
            0.0,
        )
        self.assertAlmostEqual(
            track_wrap_pi(physics_heading_to_world(0.0)),
            math.pi / 2.0,
        )

    def test_stanley_steers_right_when_car_is_left_of_path(self) -> None:
        path = TrackPath.from_points(
            [(float(x), 0.0) for x in range(20)],
            [20.0] * 20,
        )
        state = SimpleNamespace(
            position=(0.0, 0.0, 1.0),
            heading_rad=-math.pi / 2.0,
            speed_ms=5.0,
            yaw_rate_rad_s=0.0,
        )
        controller = StanleyController(
            StanleyConfig(
                cross_track_gain=1.0,
                steering_gain=1.0,
                steer_limit=1.0,
                steer_rate_per_s=10.0,
            )
        )
        info = controller.step(state, path, 0.1)
        self.assertGreater(info.lateral_error_m, 0.0)
        self.assertLess(info.steer, 0.0)

    def test_autotune_increases_speed_after_clean_slow_lap(self) -> None:
        tuned = _next_lap_tuning(
            lap_time_s=220.0,
            target_lap_s=180.0,
            max_lateral_error_m=1.0,
            max_heading_error_rad=0.05,
            p95_lateral_error_m=1.0,
            p95_heading_error_rad=0.05,
            band_p95_lateral_error_m=(1.0, 1.0, 1.0, 1.0),
            band_p95_heading_error_rad=(0.05, 0.05, 0.05, 0.05),
            band_max_tyres_out=(0, 0, 0, 0),
            band_steering_activity=(0.001, 0.001, 0.001, 0.001),
            band_sample_count=(100, 100, 100, 100),
            max_tyres_out=0,
            steering_activity=0.001,
            speed_scale_by_band=[1.0, 1.0, 1.0, 1.0],
            cross_track_gain=0.65,
            heading_gain=1.0,
            yaw_rate_gain=0.15,
            steering_filter_tau_s=0.10,
            braking_accel_by_band=[4.5, 4.5, 4.5, 4.5],
            speed_scale_step=0.05,
            max_speed_scale=1.45,
            braking_accel_step_mps2=0.30,
            max_braking_accel_mps2=8.34,
        )
        self.assertAlmostEqual(tuned[0][0], 1.12)
        self.assertAlmostEqual(tuned[0][-1], 1.04)
        self.assertAlmostEqual(tuned[5][0], 5.01)
        self.assertAlmostEqual(tuned[5][-1], 4.71)
        self.assertLess(tuned[0][0], 1.2)

    def test_autotune_reduces_speed_after_unstable_lap(self) -> None:
        tuned = _next_lap_tuning(
            lap_time_s=220.0,
            target_lap_s=180.0,
            max_lateral_error_m=6.5,
            max_heading_error_rad=0.2,
            p95_lateral_error_m=3.8,
            p95_heading_error_rad=0.2,
            band_p95_lateral_error_m=(3.8, 3.8, 3.8, 3.8),
            band_p95_heading_error_rad=(0.2, 0.2, 0.2, 0.2),
            band_max_tyres_out=(3, 3, 3, 3),
            band_steering_activity=(0.006, 0.006, 0.006, 0.006),
            band_sample_count=(100, 100, 100, 100),
            max_tyres_out=3,
            steering_activity=0.006,
            speed_scale_by_band=[1.2, 1.2, 1.2, 1.2],
            cross_track_gain=0.65,
            heading_gain=1.0,
            yaw_rate_gain=0.15,
            steering_filter_tau_s=0.10,
            braking_accel_by_band=[6.0, 6.0, 6.0, 6.0],
            speed_scale_step=0.05,
            max_speed_scale=1.45,
            braking_accel_step_mps2=0.30,
            max_braking_accel_mps2=8.34,
        )
        self.assertLess(tuned[0][0], 1.2)
        self.assertGreater(tuned[1], 0.65)
        self.assertLess(tuned[5][0], 6.0)

    def test_gain_maps_adapt_per_speed_band(self) -> None:
        config = StanleyConfig()
        _tune_gain_maps(
            config,
            band_p95_lateral_error_m=(3.0, 1.8, 1.0, 0.5),
            band_steering_activity=(0.003, 0.001, 0.0005, 0.0003),
            band_sample_count=(100, 100, 100, 100),
        )
        self.assertGreater(config.cross_track_gain_map[0], 1.10)
        self.assertGreater(config.cross_track_gain_map[1], 1.00)
        self.assertGreater(config.yaw_rate_gain_map[0], 1.20)
        self.assertGreater(config.filter_tau_map[0], 0.85)

    def test_lmpc_tuning_raises_low_speed_boundary_and_weights(self) -> None:
        config = MpcConfig(
            q_lateral=60.0,
            q_heading=50.0,
            q_yaw_rate=10.0,
        )
        speed_scales, braking = _next_lmpc_tuning(
            lap_time_s=220.0,
            target_lap_s=180.0,
            p95_lateral_error_m=0.9,
            p95_heading_error_rad=0.10,
            steering_activity=0.001,
            band_raw_steering_activity=(
                0.01,
                0.01,
                0.01,
                0.01,
            ),
            band_p95_lateral_error_m=(0.3, 0.3, 0.3, 0.3),
            band_p95_heading_error_rad=(0.04, 0.04, 0.04, 0.04),
            band_max_tyres_out=(0, 0, 0, 0),
            band_sample_count=(100, 100, 100, 100),
            band_p95_lateral_g=(0.5, 0.6, 0.7, 0.2),
            band_p95_longitudinal_g=(0.5, 0.6, 0.6, 0.5),
            speed_scale_by_band=[1.0, 1.3, 1.3, 1.3],
            braking_accel_by_band=[6.0, 6.0, 6.0, 6.0],
            base_lateral_accel_mps2=3.92,
            max_speed_scale=1.45,
            max_braking_accel_mps2=8.34,
            mpc_config=config,
        )
        self.assertGreater(speed_scales[0], 1.0)
        self.assertLessEqual(speed_scales[0], 1.45)
        self.assertGreater(braking[0], 6.0)
        self.assertGreater(config.q_lateral, 60.0)
        self.assertGreater(config.q_yaw_rate, 10.0)
        self.assertGreater(config.r_steer, 6.0)
        self.assertGreaterEqual(config.rd_steer, 320.0)
        self.assertGreater(config.r_steer_weight_map[3], 2.0)

    def test_lmpc_tuning_rolls_back_scale_below_one(self) -> None:
        config = MpcConfig()
        speed_scales, _ = _next_lmpc_tuning(
            lap_time_s=160.0,
            target_lap_s=180.0,
            p95_lateral_error_m=2.0,
            p95_heading_error_rad=0.20,
            steering_activity=0.002,
            band_raw_steering_activity=(0.002,) * 6,
            band_p95_lateral_error_m=(2.0, 0.2, 0.2, 0.2, 0.2, 0.2),
            band_p95_heading_error_rad=(0.2, 0.03, 0.03, 0.03, 0.03, 0.03),
            band_max_tyres_out=(1, 0, 0, 0, 0, 0),
            band_sample_count=(100, 100, 100, 100, 100, 100),
            band_p95_lateral_g=(0.7, 0.4, 0.4, 0.4, 0.4, 0.4),
            band_p95_longitudinal_g=(0.5,) * 6,
            speed_scale_by_band=[1.0] * 6,
            braking_accel_by_band=[6.0] * 6,
            base_lateral_accel_mps2=6.867,
            max_speed_scale=1.45,
            max_braking_accel_mps2=20.0,
            mpc_config=config,
        )
        self.assertLess(speed_scales[0], 1.0)
        self.assertGreaterEqual(speed_scales[0], 0.60)

    def test_lmpc_tuning_damps_high_speed_steering_activity(self) -> None:
        config = MpcConfig()
        _next_lmpc_tuning(
            lap_time_s=220.0,
            target_lap_s=180.0,
            p95_lateral_error_m=0.30,
            p95_heading_error_rad=0.04,
            steering_activity=0.0015,
            band_raw_steering_activity=(
                0.0010,
                0.0010,
                0.0010,
                0.0018,
            ),
            band_p95_lateral_error_m=(0.2, 0.2, 0.2, 0.2),
            band_p95_heading_error_rad=(0.03, 0.03, 0.03, 0.03),
            band_max_tyres_out=(0, 0, 0, 0),
            band_sample_count=(100, 100, 100, 100),
            band_p95_lateral_g=(0.4, 0.5, 0.6, 0.3),
            band_p95_longitudinal_g=(0.4, 0.5, 0.5, 0.3),
            speed_scale_by_band=[1.0, 1.0, 1.0, 1.0],
            braking_accel_by_band=[6.0, 6.0, 6.0, 6.0],
            base_lateral_accel_mps2=3.92,
            max_speed_scale=1.45,
            max_braking_accel_mps2=8.34,
            mpc_config=config,
        )
        self.assertGreater(config.r_steer, 8.0)
        self.assertGreater(config.rd_steer, 320.0)
        self.assertGreater(config.r_steer_weight_map[3], 2.25)
        self.assertGreater(config.rd_steer_weight_map[3], 2.5)

    def test_longitudinal_preview_is_rate_limited(self) -> None:
        path = TrackPath.from_points(
            [(float(index), 0.0) for index in range(300)],
            [120.0] * 300,
        )
        preview = _build_longitudinal_preview(
            path,
            0,
            target_speed_kmh=40.0,
            speed_mps=11.0,
            horizon=50,
            dt=0.02,
            acceleration_limit_mps2=2.0,
            braking_limit_mps2=5.0,
            speed_scale_by_band=[1.0, 1.0, 1.0, 1.0],
            steering=0.0,
            lateral_g=0.0,
        )
        max_step = 2.0 * 0.02 + 1e-9
        self.assertEqual(len(preview), 51)
        self.assertLessEqual(max(preview), 40.0 / 3.6 + 1e-9)
        self.assertTrue(
            all(
                following - current <= max_step
                for current, following in zip(preview, preview[1:])
            )
        )

    def test_longitudinal_preview_respects_fixed_limits(self) -> None:
        path = TrackPath.from_points(
            [(float(index), 0.0) for index in range(300)],
            [120.0] * 300,
            fixed_speed_limit=[True] * 300,
        )
        preview = _build_longitudinal_preview(
            path,
            0,
            target_speed_kmh=120.0,
            speed_mps=33.0,
            horizon=50,
            dt=0.02,
            acceleration_limit_mps2=3.0,
            braking_limit_mps2=6.0,
            speed_scale_by_band=[1.5, 1.5, 1.5, 1.5],
            steering=0.0,
            lateral_g=0.0,
        )
        self.assertLessEqual(
            max(preview),
            120.0 / 3.6 + 1e-9,
        )
        self.assertAlmostEqual(
            preview[-1],
            120.0 / 3.6,
            places=6,
        )
        self.assertTrue(
            all(
                following - current <= 3.0 * 0.02 + 1e-9
                for current, following in zip(preview, preview[1:])
            )
        )

    def test_longitudinal_preview_has_continuous_braking_intent(
        self,
    ) -> None:
        path = TrackPath.from_points(
            [(float(index), 0.0) for index in range(300)],
            [200.0] * 300,
        )
        preview = _build_longitudinal_preview(
            path,
            0,
            target_speed_kmh=200.0,
            speed_mps=250.0 / 3.6,
            horizon=45,
            dt=0.05,
            acceleration_limit_mps2=3.0,
            braking_limit_mps2=6.0,
            speed_scale_by_band=[1.0, 1.0, 1.0, 1.0],
            steering=0.0,
            lateral_g=0.0,
        )
        self.assertAlmostEqual(preview[0], 200.0 / 3.6, places=6)
        self.assertTrue(
            all(
                following <= current + 1e-9
                for current, following in zip(preview, preview[1:])
            )
        )

    def test_braking_lead_distance_moves_limit_earlier(self) -> None:
        speeds = [120.0] * 120 + [30.0] * 120
        path = TrackPath.from_points(
            [(float(index), 0.0) for index in range(240)],
            speeds,
        )
        without_lead = _braking_limited_speed_kmh(
            path,
            0,
            [1.0, 1.0, 1.0, 1.0],
            [3.0, 3.0, 3.0, 3.0],
            safety_factor=0.95,
            lead_distance_m=0.0,
        )
        with_lead = _braking_limited_speed_kmh(
            path,
            0,
            [1.0, 1.0, 1.0, 1.0],
            [3.0, 3.0, 3.0, 3.0],
            safety_factor=0.95,
            lead_distance_m=20.0,
        )
        self.assertGreater(with_lead, without_lead)

    def test_braking_lead_grows_with_speed(self) -> None:
        path = TrackPath.from_points(
            [(float(index), 0.0) for index in range(300)],
            [200.0] * 120 + [30.0] * 180,
        )
        fixed = _braking_limited_speed_kmh(
            path,
            0,
            [1.0, 1.0, 1.0, 1.0],
            [3.0, 3.0, 3.0, 3.0],
            safety_factor=0.95,
            lead_distance_m=0.0,
            lead_speed_gain_m_per_kmh=0.0,
        )
        speed_scaled = _braking_limited_speed_kmh(
            path,
            0,
            [1.0, 1.0, 1.0, 1.0],
            [3.0, 3.0, 3.0, 3.0],
            safety_factor=0.95,
            lead_distance_m=0.0,
            lead_speed_gain_m_per_kmh=0.10,
        )
        self.assertGreater(speed_scaled, fixed)

    def test_base_brake_shift_changes_profile_not_corner_speed(self) -> None:
        count = 240
        ds = [1.0] * count
        curvature = [0.0] * 100 + [0.01] * 40 + [0.0] * 100
        baseline = _speed_limits_from_geometry(
            ds=ds,
            curvature=curvature,
            max_speed_kmh=120.0,
            min_speed_kmh=20.0,
            lateral_accel_mps2=8.0,
            accel_mps2=3.0,
            brake_mps2=10.0,
            speed_smoothing_radius=0,
        )
        shifted = _speed_limits_from_geometry(
            ds=ds,
            curvature=curvature,
            max_speed_kmh=120.0,
            min_speed_kmh=20.0,
            lateral_accel_mps2=8.0,
            accel_mps2=3.0,
            brake_mps2=10.0,
            speed_smoothing_radius=0,
            brake_point_shift_base_m=5.0,
            brake_point_shift_speed_gain_m_per_kmh=0.2,
        )
        self.assertGreater(shifted[85], baseline[85])
        self.assertAlmostEqual(
            shifted[120],
            baseline[120],
            places=6,
        )

    def test_late_braking_preserves_fixed_speed_limits(self) -> None:
        path = TrackPath.from_points(
            [(float(index), 0.0) for index in range(240)],
            [120.0] * 240,
            fixed_speed_limit=[True] * 240,
        )
        shifted = path.with_late_braking(
            max_speed_kmh=280.0,
            min_speed_kmh=25.0,
            lateral_accel_mps2=8.0,
            accel_mps2=3.0,
            brake_mps2=16.0,
            brake_point_shift_base_m=5.0,
            brake_point_shift_speed_gain_m_per_kmh=0.2,
        )
        self.assertTrue(
            all(
                abs(value - 120.0) < 1e-9
                for value in shifted.speed_limit_kmh
            )
        )

    def test_entry_lateral_boost_only_affects_corner_entry(self) -> None:
        curvature = [0.0] * 80 + [0.008] * 40 + [0.0] * 80
        weights = _entry_lateral_accel_weights(curvature)
        self.assertGreater(weights[79], 0.0)
        self.assertLess(weights[110], 0.1)
        ds = [1.0] * len(curvature)
        baseline = _speed_limits_from_geometry(
            ds=ds,
            curvature=curvature,
            max_speed_kmh=160.0,
            min_speed_kmh=20.0,
            lateral_accel_mps2=7.0,
            accel_mps2=3.0,
            brake_mps2=16.0,
            speed_smoothing_radius=0,
        )
        boosted = _speed_limits_from_geometry(
            ds=ds,
            curvature=curvature,
            max_speed_kmh=160.0,
            min_speed_kmh=20.0,
            lateral_accel_mps2=7.0,
            accel_mps2=3.0,
            brake_mps2=16.0,
            speed_smoothing_radius=0,
            entry_lateral_accel_boost=0.20,
        )
        self.assertGreater(boosted[79], baseline[79])
        self.assertAlmostEqual(
            boosted[110],
            baseline[110],
            places=6,
        )

    def test_friction_ellipse_reduces_combined_limits(self) -> None:
        curvature = [0.0] * 100 + [0.008] * 80 + [0.0] * 100
        ds = [1.0] * len(curvature)
        baseline = _speed_limits_from_geometry(
            ds=ds,
            curvature=curvature,
            max_speed_kmh=160.0,
            min_speed_kmh=20.0,
            lateral_accel_mps2=7.0,
            accel_mps2=3.0,
            brake_mps2=14.0,
            speed_smoothing_radius=0,
        )
        ellipse = _speed_limits_from_geometry(
            ds=ds,
            curvature=curvature,
            max_speed_kmh=160.0,
            min_speed_kmh=20.0,
            lateral_accel_mps2=7.0,
            accel_mps2=3.0,
            brake_mps2=14.0,
            speed_smoothing_radius=0,
            use_friction_ellipse=True,
        )
        self.assertLessEqual(ellipse[120], baseline[120] + 1e-9)
        self.assertLessEqual(ellipse[50], baseline[50] + 1e-9)

    def test_path_resampling_preserves_closed_geometry(self) -> None:
        count = 120
        path = TrackPath.from_points(
            [
                (
                    50.0 * math.cos(2.0 * math.pi * index / count),
                    50.0 * math.sin(2.0 * math.pi * index / count),
                )
                for index in range(count)
            ],
            [80.0] * count,
            reference_points=None,
        )
        resampled = path.resampled(1.0)
        self.assertGreater(len(resampled.points), len(path.points))
        self.assertAlmostEqual(
            resampled.total_length_m,
            path.total_length_m,
            delta=1.0,
        )
        self.assertTrue(
            all(
                math.isfinite(x + z)
                for x, z in resampled.points
            )
        )

    def test_minimum_edge_clearance_projects_racing_line(self) -> None:
        count = 240
        points = [
            (
                50.0 * math.cos(2.0 * math.pi * index / count),
                50.0 * math.sin(2.0 * math.pi * index / count),
            )
            for index in range(count)
        ]
        path = TrackPath.from_points(
            points,
            [80.0] * count,
            reference_points=list(points),
            left_width_m=[10.0] * count,
            right_width_m=[1.0] * count,
            line_offset_m=[-0.5] * count,
        )
        projected = path.with_minimum_edge_clearance(
            0.8,
            max_speed_kmh=280.0,
            min_speed_kmh=25.0,
            lateral_accel_mps2=7.3575,
            accel_mps2=3.924,
            brake_mps2=16.0,
        )
        clearances = [
            min(projected.edge_clearance(index))
            for index in range(len(projected.points))
        ]
        self.assertGreaterEqual(min(clearances), 0.79)

    def test_minimum_edge_clearance_expands_from_very_narrow_edge(
        self,
    ) -> None:
        count = 240
        points = [
            (float(index), 0.0)
            for index in range(count)
        ]
        path = TrackPath.from_points(
            points,
            [80.0] * count,
            reference_points=list(points),
            left_width_m=[10.0] * count,
            right_width_m=[0.4] * count,
            line_offset_m=[0.0] * count,
        )
        projected = path.with_minimum_edge_clearance(
            0.8,
            max_speed_kmh=280.0,
            min_speed_kmh=25.0,
            lateral_accel_mps2=7.3575,
            accel_mps2=3.924,
            brake_mps2=16.0,
        )
        clearances = [
            min(projected.edge_clearance(index))
            for index in range(len(projected.points))
        ]
        self.assertGreaterEqual(min(clearances), 0.79)

    def test_longitudinal_preview_removes_braking_zone_bump(self) -> None:
        speeds = [120.0] * 240
        speeds[25:45] = [170.0] * 20
        speeds[45:75] = [80.0] * 30
        path = TrackPath.from_points(
            [(float(index), 0.0) for index in range(240)],
            speeds,
        )
        preview = _build_longitudinal_preview(
            path,
            0,
            target_speed_kmh=120.0,
            speed_mps=33.0,
            horizon=70,
            dt=0.05,
            acceleration_limit_mps2=3.0,
            braking_limit_mps2=7.0,
            speed_scale_by_band=[1.0, 1.0, 1.0, 1.0],
            steering=0.0,
            lateral_g=0.0,
        )
        bump_index = 10
        corner_index = 44
        self.assertLess(preview[bump_index], 170.0 / 3.6)
        for earlier, later in zip(
            preview[bump_index:corner_index],
            preview[bump_index + 1 : corner_index + 1],
        ):
            self.assertGreaterEqual(earlier + 1e-6, later)

    def test_scaled_path_speed_respects_absolute_cap(self) -> None:
        path = TrackPath.from_points(
            [(float(index), 0.0) for index in range(300)],
            [240.0] * 300,
        )
        self.assertEqual(
            _scaled_path_speed_kmh(
                path,
                0,
                [1.0, 1.2, 1.4, 1.6],
                max_speed_kmh=260.0,
            ),
            260.0,
        )

    def test_scaled_path_speed_respects_curvature_limit(self) -> None:
        count = 360
        radius = 100.0
        path = TrackPath.from_points(
            [
                (
                    radius * math.sin(2.0 * math.pi * index / count),
                    radius * math.cos(2.0 * math.pi * index / count),
                )
                for index in range(count)
            ],
            [240.0] * count,
        )
        lateral_limit = 6.0
        scaled = _scaled_path_speed_kmh(
            path,
            0,
            [1.0, 1.4, 1.6, 1.8],
            max_speed_kmh=280.0,
            max_lateral_accel_mps2=lateral_limit,
        )
        expected = (
            math.sqrt(lateral_limit / abs(path.curvature[0]))
            * 3.6
        )
        self.assertLessEqual(scaled, expected + 1e-6)

    def test_speed_scale_transitions_are_continuous(self) -> None:
        scales = [1.0, 1.32, 1.275, 1.165]
        before = _speed_scale_kmh(140.0 - 1e-6, scales)
        after = _speed_scale_kmh(140.0 + 1e-6, scales)
        self.assertAlmostEqual(before, after, places=6)
        transition_value = _speed_scale_kmh(139.0, scales)
        self.assertLess(transition_value, 1.275)
        self.assertGreater(transition_value, 1.165)

    def test_six_speed_bands_cover_high_speed_points(self) -> None:
        self.assertEqual(_speed_band(39.9), 0)
        self.assertEqual(_speed_band(40.0), 1)
        self.assertEqual(_speed_band(60.0), 2)
        self.assertEqual(_speed_band(80.0), 3)
        self.assertEqual(_speed_band(100.0), 4)
        self.assertEqual(_speed_band(120.0), 5)
        self.assertEqual(_speed_band(140.0), 6)
        self.assertEqual(_speed_band(180.0), 7)
        self.assertEqual(_speed_band(220.0), 8)
        self.assertEqual(_speed_band(80.0, 6), 2)
        self.assertEqual(_speed_band(140.0, 6), 3)
        self.assertEqual(_speed_band(180.0, 6), 4)
        self.assertEqual(_speed_band(220.0, 6), 5)
        scales = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.assertAlmostEqual(
            _speed_scale_kmh(180.0 - 1e-6, scales),
            _speed_scale_kmh(180.0 + 1e-6, scales),
            places=6,
        )

    def test_speed_envelope_only_increases_when_requested(self) -> None:
        path = TrackPath.from_points(
            [(float(index), 0.0) for index in range(20)],
            [80.0] * 20,
        )
        candidate = path.with_speed_envelope(
            [60.0, 100.0] * 10,
            blend=1.0,
            maximum_speed_kmh=280.0,
            only_increase=True,
        )
        self.assertEqual(candidate.speed_limit_kmh[0], 80.0)
        self.assertEqual(candidate.speed_limit_kmh[1], 100.0)

    def test_target_speed_filter_is_asymmetric(self) -> None:
        rising = _filter_target_speed(
            80.0,
            90.0,
            dt=0.02,
            rise_tau_s=0.35,
            fall_tau_s=0.10,
        )
        falling = _filter_target_speed(
            90.0,
            80.0,
            dt=0.02,
            rise_tau_s=0.35,
            fall_tau_s=0.10,
        )
        self.assertGreater(rising, 80.0)
        self.assertLess(falling, 90.0)
        self.assertLess(rising - 80.0, 90.0 - falling)

    def test_target_speed_rate_limit_is_not_attenuated_by_filter(
        self,
    ) -> None:
        commanded, filtered = _next_target_speed(
            current_kmh=50.0,
            filtered_kmh=50.0,
            requested_kmh=200.0,
            dt=0.02,
            rise_rate_kmh_s=18.0,
            fall_rate_kmh_s=60.0,
            rise_tau_s=0.15,
            fall_tau_s=0.0,
        )
        self.assertAlmostEqual(commanded, 50.36, places=9)
        self.assertGreater(filtered, 50.0)
        falling, _ = _next_target_speed(
            current_kmh=100.0,
            filtered_kmh=100.0,
            requested_kmh=0.0,
            dt=0.02,
            rise_rate_kmh_s=18.0,
            fall_rate_kmh_s=60.0,
            rise_tau_s=0.15,
            fall_tau_s=0.0,
        )
        self.assertAlmostEqual(falling, 98.8, places=9)

    def test_round_two_tuning_state_and_model_load(self) -> None:
        root = Path(__file__).resolve().parents[1]
        tuning = _load_tuning_state(
            root / "configs" / "lmpc_round2_start.json"
        )
        model = _load_mpc_parameters(
            root / "logs" / "autotune_10_laps_mpc_analysis.json"
        )
        self.assertEqual(
            tuning["speed_scale_by_band"],
            [1.0, 1.45, 1.45, 1.36],
        )
        self.assertIsNotNone(model)
        assert model is not None
        self.assertGreater(
            model.front_cornering_stiffness_n_rad,
            0.0,
        )

    def test_open_merge_does_not_bend_the_finish(self) -> None:
        path = TrackPath.from_points(
            [(float(x), 0.0) for x in range(200)],
            [60.0] * 200,
        )
        merged = path.plan_merge(
            car_x=0.0,
            car_z=4.0,
            car_heading_rad=0.0,
            merge_distance_m=20.0,
            close_loop=False,
        )
        self.assertAlmostEqual(merged.points[0][1], 4.0)
        self.assertAlmostEqual(merged.points[-1][1], path.points[-1][1])

    def test_start_merge_preserves_loop_boundary(self) -> None:
        path = TrackPath.from_points(
            [(float(x), 0.0) for x in range(200)],
            [60.0] * 200,
        )
        merged = path.plan_start_merge(
            car_x=100.0,
            car_z=4.0,
            car_heading_rad=0.0,
            merge_distance_m=20.0,
        )
        self.assertEqual(merged.points[0], path.points[0])
        self.assertEqual(merged.points[-1], path.points[-1])
        self.assertAlmostEqual(merged.points[0][1], 0.0)

    def test_start_merge_speed_is_not_scaled(self) -> None:
        path = TrackPath.from_points(
            [(float(x), 0.0) for x in range(300)],
            [100.0] * 300,
        )
        merged = path.plan_start_merge(
            car_x=50.0,
            car_z=0.0,
            car_heading_rad=0.0,
            merge_distance_m=20.0,
            merge_speed_kmh=30.0,
        )
        index = merged.nearest_index(50.0, 0.0)
        self.assertAlmostEqual(
            _scaled_path_speed_kmh(
                merged,
                index,
                [1.3, 1.3, 1.3, 1.3],
            ),
            30.0,
        )

    def test_start_speed_limit_preserves_reference_geometry(self) -> None:
        path = TrackPath.from_points(
            [(float(x), 0.0) for x in range(300)],
            [100.0] * 300,
        )
        limited = path.with_start_speed_limit(
            car_x=50.0,
            car_z=0.0,
            distance_m=30.0,
            speed_kmh=25.0,
        )
        self.assertEqual(limited.points, path.points)
        self.assertEqual(limited.curvature, path.curvature)
        index = limited.nearest_index(50.0, 0.0)
        self.assertTrue(limited.fixed_speed_limit[index])
        self.assertAlmostEqual(
            _scaled_path_speed_kmh(
                limited,
                index,
                [1.3, 1.3, 1.3, 1.3],
            ),
            25.0,
        )


if __name__ == "__main__":
    unittest.main()
