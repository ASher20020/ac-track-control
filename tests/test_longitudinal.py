from __future__ import annotations

import unittest

import numpy as np

from actc.longitudinal import (
    LongitudinalMpcConfig,
    LongitudinalMpcController,
    LongitudinalPedalConfig,
    LongitudinalPedalMapper,
    brake_command_for_deceleration,
    map_acceleration_to_pedals,
)
from actc.cli import (
    _apply_saved_longitudinal_parameters,
    _tune_longitudinal_mpc,
)


class LongitudinalMpcTests(unittest.TestCase):
    def test_accelerates_toward_higher_reference(self) -> None:
        controller = LongitudinalMpcController(
            LongitudinalMpcConfig(horizon=12)
        )
        result = controller.step(
            speed_mps=20.0,
            acceleration_mps2=0.0,
            reference_speed_mps=np.full(13, 30.0),
        )
        self.assertGreater(result.acceleration_command_mps2, 0.0)
        self.assertLess(result.acceleration_command_mps2, 4.0)

    def test_accelerates_from_rest(self) -> None:
        controller = LongitudinalMpcController(
            LongitudinalMpcConfig(
                horizon=50,
            )
        )
        result = controller.step(
            speed_mps=0.0,
            acceleration_mps2=0.0,
            reference_speed_mps=np.linspace(
                0.0,
                22.22,
                51,
            ),
            acceleration_limit_mps2=3.924,
            braking_limit_mps2=5.0,
        )
        self.assertGreater(result.acceleration_command_mps2, 0.0)
        self.assertLess(result.acceleration_command_mps2, 4.0)

    def test_compensates_for_coast_down_resistance(self) -> None:
        controller = LongitudinalMpcController(
            LongitudinalMpcConfig(
                horizon=50,
            )
        )
        result = None
        for _ in range(10):
            result = controller.step(
                speed_mps=25.0,
                acceleration_mps2=0.0,
                reference_speed_mps=np.full(51, 25.0),
                acceleration_limit_mps2=3.924,
                braking_limit_mps2=5.0,
            )
        assert result is not None
        self.assertGreater(result.acceleration_command_mps2, 0.5)

    def test_brakes_toward_lower_reference(self) -> None:
        controller = LongitudinalMpcController(
            LongitudinalMpcConfig(horizon=12)
        )
        result = controller.step(
            speed_mps=45.0,
            acceleration_mps2=0.0,
            reference_speed_mps=np.linspace(40.0, 25.0, 13),
        )
        self.assertLess(result.acceleration_command_mps2, 0.0)
        self.assertGreater(result.acceleration_command_mps2, -7.0)

    def test_first_command_respects_jerk_limit(self) -> None:
        config = LongitudinalMpcConfig(
            horizon=12,
            max_jerk_mps3=10.0,
        )
        controller = LongitudinalMpcController(config)
        result = controller.step(
            speed_mps=20.0,
            acceleration_mps2=0.0,
            reference_speed_mps=np.full(13, 40.0),
        )
        self.assertLessEqual(
            abs(result.acceleration_command_mps2),
            config.max_jerk_mps3 * config.dt + 1e-3,
        )

    def test_uses_speed_dependent_longitudinal_weights(self) -> None:
        controller = LongitudinalMpcController(
            LongitudinalMpcConfig(horizon=12)
        )
        controller.step(
            speed_mps=20.0,
            acceleration_mps2=0.0,
            reference_speed_mps=np.full(13, 25.0),
        )
        low_speed_index = controller._speed_weight_schedule_index
        low_speed_matrix = controller._p_matrix.copy()
        controller.step(
            speed_mps=220.0 / 3.6,
            acceleration_mps2=0.0,
            reference_speed_mps=np.full(13, 65.0),
        )
        self.assertGreater(
            controller._speed_weight_schedule_index,
            low_speed_index,
        )
        self.assertGreater(
            controller._p_matrix.diagonal().mean(),
            low_speed_matrix.diagonal().mean(),
        )

    def test_acceleration_maps_to_pedals(self) -> None:
        throttle, brake = map_acceleration_to_pedals(2.0, 20.0)
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)
        throttle, brake = map_acceleration_to_pedals(-4.0, 20.0)
        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)

    def test_pedal_mapper_limits_low_speed_throttle(self) -> None:
        mapper = LongitudinalPedalMapper()
        for _ in range(30):
            throttle, brake = mapper.step(
                acceleration_mps2=5.0,
                speed_mps=4.0,
                steering=0.0,
                rear_slip=0.0,
                dt=0.02,
            )
        self.assertGreater(throttle, 0.8)
        self.assertEqual(brake, 0.0)

    def test_pedal_mapper_allows_high_throttle_on_straight(self) -> None:
        mapper = LongitudinalPedalMapper(
            LongitudinalPedalConfig(
                throttle_rate_up_per_s=10.0,
            )
        )
        for _ in range(20):
            throttle, brake = mapper.step(
                acceleration_mps2=3.2,
                speed_mps=80.0 / 3.6,
                steering=0.0,
                rear_slip=0.0,
                dt=0.02,
            )
        self.assertGreater(throttle, 0.80)
        self.assertEqual(brake, 0.0)

    def test_small_deceleration_uses_coast_resistance_before_brake(
        self,
    ) -> None:
        mapper = LongitudinalPedalMapper(
            LongitudinalPedalConfig(
                throttle_rate_up_per_s=10.0,
            )
        )
        throttle, brake = mapper.step(
            acceleration_mps2=-0.4,
            speed_mps=80.0 / 3.6,
            steering=0.0,
            rear_slip=0.0,
            dt=0.02,
            drag_acceleration_mps2=0.8,
        )
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)

    def test_pedal_mapper_reduces_throttle_with_steering_and_slip(
        self,
    ) -> None:
        straight = LongitudinalPedalMapper()
        corner = LongitudinalPedalMapper()
        for _ in range(100):
            throttle_straight, _ = straight.step(
                acceleration_mps2=5.0,
                speed_mps=20.0,
                steering=0.0,
                rear_slip=0.0,
                dt=0.02,
            )
            throttle_corner, _ = corner.step(
                acceleration_mps2=5.0,
                speed_mps=20.0,
                steering=0.5,
                rear_slip=0.8,
                dt=0.02,
            )
        self.assertLess(throttle_corner, throttle_straight)

    def test_pedal_mapper_releases_throttle_smoothly(self) -> None:
        mapper = LongitudinalPedalMapper()
        mapper.throttle = 0.22
        throttle, brake = mapper.limit_pedals(
            throttle=0.0,
            brake=0.0,
            speed_mps=14.0,
            steering=0.0,
            rear_slip=0.0,
            acceleration_command_mps2=0.1,
            dt=0.02,
        )
        self.assertAlmostEqual(throttle, 0.17)
        self.assertEqual(brake, 0.0)

    def test_calibrated_brake_command_uses_measured_response(self) -> None:
        config = LongitudinalPedalConfig()
        low_speed_command = brake_command_for_deceleration(
            8.0,
            40.0 / 3.6,
            config,
        )
        high_speed_command = brake_command_for_deceleration(
            8.0,
            200.0 / 3.6,
            config,
        )
        self.assertGreater(low_speed_command, 0.0)
        self.assertLess(low_speed_command, 1.0)
        self.assertGreater(high_speed_command, 0.0)
        self.assertLess(high_speed_command, 1.0)
        self.assertNotAlmostEqual(
            low_speed_command,
            high_speed_command,
            places=3,
        )

    def test_longitudinal_tuning_updates_and_rebuilds_solver(self) -> None:
        config = LongitudinalMpcConfig(horizon=12)
        _apply_saved_longitudinal_parameters(
            config,
            {
                "q_speed": 24.0,
                "max_brake_mps2": 6.4,
            },
        )
        controller = LongitudinalMpcController(config)
        previous_q_speed = controller.config.q_speed
        _tune_longitudinal_mpc(
            controller,
            band_p95_speed_error_kmh=(5.0, 0.0, 0.0, 0.0),
            band_sample_count=(100, 0, 0, 0),
            band_p95_longitudinal_g=(0.5, 0.0, 0.0, 0.0),
            band_max_tyres_out=(0, 0, 0, 0),
            max_brake_limit_mps2=17.0,
        )
        self.assertGreater(controller.config.q_speed, previous_q_speed)
        self.assertIsNone(controller._solver)

    def test_high_but_usable_braking_does_not_reduce_limit(self) -> None:
        config = LongitudinalMpcConfig(
            horizon=12,
            max_brake_mps2=17.0,
        )
        controller = LongitudinalMpcController(config)
        _tune_longitudinal_mpc(
            controller,
            band_p95_speed_error_kmh=(1.0, 0.0, 0.0, 0.0),
            band_sample_count=(100, 0, 0, 0),
            band_p95_longitudinal_g=(1.20, 0.0, 0.0, 0.0),
            band_max_tyres_out=(0, 0, 0, 0),
            max_brake_limit_mps2=17.0,
        )
        self.assertEqual(
            controller.config.max_brake_mps2,
            17.0,
        )


if __name__ == "__main__":
    unittest.main()
