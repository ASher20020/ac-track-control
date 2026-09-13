from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

import numpy as np

from actc.mpc import (
    LinearMpcController,
    MpcConfig,
    MpcVehicleParameters,
    NonlinearMpcController,
    build_reference_sequence,
    vehicle_state_to_frenet,
)
from actc.track import TrackPath


def make_path() -> TrackPath:
    return TrackPath.from_points(
        [(float(index), 0.0) for index in range(80)],
        [45.0] * 80,
    )


def make_state() -> SimpleNamespace:
    return SimpleNamespace(
        position=(0.0, 0.0, 0.5),
        heading_rad=-math.pi / 2.0,
        local_velocity_m_s=(0.0, 0.0, 10.0),
        yaw_rate_rad_s=0.0,
        speed_ms=10.0,
        speed_kmh=36.0,
        steer=0.0,
    )


class MpcTests(unittest.TestCase):
    def test_reference_is_in_ego_frame(self) -> None:
        path = make_path()
        state = make_state()
        reference = build_reference_sequence(
            path,
            0,
            state,
            horizon=10,
            dt=0.02,
        )
        self.assertTrue((reference.local_points[:, 0] > 0.0).all())
        self.assertAlmostEqual(
            reference.local_points[0, 1],
            -0.5,
            places=6,
        )

    def test_frenet_error_uses_continuous_segment_projection(self) -> None:
        path = make_path()
        state = make_state()
        state.position = (3.5, 0.0, 0.2)
        index, frenet = vehicle_state_to_frenet(
            state,
            path,
            previous_index=2,
        )
        self.assertEqual(index, 3)
        self.assertAlmostEqual(frenet[0], 0.2, places=6)
        self.assertAlmostEqual(frenet[1], 0.0, places=6)

    def test_curved_reference_has_yaw_but_zero_lateral_velocity(
        self,
    ) -> None:
        points = [
            (
                50.0 * math.cos(2.0 * math.pi * index / 120.0),
                50.0 * math.sin(2.0 * math.pi * index / 120.0),
            )
            for index in range(120)
        ]
        path = TrackPath.from_points(points, [40.0] * 120)
        state = make_state()
        state.position = (points[0][0], 0.0, points[0][1])
        state.heading_rad = path.heading_rad[0] - math.pi / 2.0
        reference = build_reference_sequence(
            path,
            0,
            state,
            horizon=10,
            dt=0.02,
            parameters=MpcVehicleParameters(),
        )
        self.assertTrue((reference.states[:, 3] == 0.0).all())
        expected_yaw_rate = (
            reference.states[0, 2] * path.curvature[0]
        )
        self.assertAlmostEqual(
            reference.states[0, 4],
            expected_yaw_rate,
            places=9,
        )

    def test_linear_mpc_returns_finite_steering(self) -> None:
        path = make_path()
        state = make_state()
        controller = LinearMpcController(MpcConfig(horizon=20, dt=0.02))
        controller.calibrate(state, path)
        info = controller.step(state, path, 0.02)
        self.assertTrue(math.isfinite(info.steer))
        self.assertLessEqual(abs(info.steer), 1.0)
        self.assertEqual(controller.last_stats.status, "solved")

    def test_linear_mpc_applies_steering_rate_limit_after_compensation(
        self,
    ) -> None:
        path = make_path()
        state = make_state()
        state.position = (0.0, 0.0, 4.1)
        config = MpcConfig(horizon=20, dt=0.02)
        controller = LinearMpcController(config)
        controller.calibrate(state, path)
        previous = 0.0
        for _ in range(10):
            info = controller.step(state, path, config.dt)
            max_step = (
                config.steer_rate_limit_rad_s
                * config.dt
                / config.parameters.max_steer_rad
            )
            self.assertLessEqual(abs(info.steer - previous), max_step + 1e-9)
            previous = info.steer

    def test_steering_scale_is_speed_scheduled(self) -> None:
        controller = LinearMpcController(MpcConfig(horizon=20, dt=0.02))
        low_speed_scale = controller._steering_scale_from_speed(50.0)
        medium_speed_scale = controller._steering_scale_from_speed(120.0)
        high_speed_scale = controller._steering_scale_from_speed(160.0)
        self.assertAlmostEqual(low_speed_scale, 0.50, places=6)
        self.assertAlmostEqual(medium_speed_scale, 0.43, places=6)
        self.assertLess(high_speed_scale, medium_speed_scale)
        self.assertGreater(high_speed_scale, 0.34)

    def test_discrete_prediction_spectral_radius_is_bounded(self) -> None:
        controller = LinearMpcController(MpcConfig(horizon=20, dt=0.02))
        for speed_mps in range(3, 71, 2):
            a_matrix, _ = controller._stage_model_from_speed(
                float(speed_mps)
            )
            radius = max(abs(np.linalg.eigvals(a_matrix)))
            self.assertLessEqual(radius, 0.98 + 1e-9)

    def test_yaw_response_lag_scales_with_lateral_load(self) -> None:
        controller = LinearMpcController(
            MpcConfig(horizon=20, dt=0.02)
        )
        low_a, low_b = controller._stage_model_from_speed(
            25.0,
            lateral_accel_g=0.0,
        )
        high_a, high_b = controller._stage_model_from_speed(
            25.0,
            lateral_accel_g=0.8,
        )
        low_gain = np.linalg.solve(
            low_a[3:5, 3:5],
            -low_b[3:5, 0],
        )
        high_gain = np.linalg.solve(
            high_a[3:5, 3:5],
            -high_b[3:5, 0],
        )
        np.testing.assert_allclose(
            low_gain,
            high_gain,
            rtol=1e-6,
            atol=1e-6,
        )
        low_response = max(np.linalg.eigvals(low_a[3:5, 3:5]).real)
        high_response = max(
            np.linalg.eigvals(high_a[3:5, 3:5]).real
        )
        self.assertGreater(high_response, low_response)

    def test_nonlinear_mpc_with_short_horizon_smoke(self) -> None:
        path = make_path()
        state = make_state()
        controller = NonlinearMpcController(
            MpcConfig(horizon=4, dt=0.02, solve_time_budget_ms=1000.0)
        )
        controller.calibrate(state, path)
        info = controller.step(state, path, 0.02)
        self.assertTrue(math.isfinite(info.steer))
        self.assertLessEqual(abs(info.steer), 1.0)

    def test_preview_indices_are_continuous_across_lap_boundary(self) -> None:
        controller = LinearMpcController(MpcConfig(horizon=5, dt=0.02))
        indices = controller._continuous_indices(
            np.asarray([3375, 3376, 3377, 0, 1, 2]),
            3378,
        )
        self.assertEqual(indices, [3375, 3376, 3377, 3378, 3379, 3380])


if __name__ == "__main__":
    unittest.main()
