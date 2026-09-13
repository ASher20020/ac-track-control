from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class SpeedControllerConfig:
    target_kmh: float
    kp_accel: float = 0.080
    kp_brake: float = 0.060
    ki: float = 0.008
    integral_limit: float = 0.15
    deadband_kmh: float = 0.6
    throttle_rate_per_s: float = 1.5
    brake_rate_per_s: float = 2.0
    speed_rate_filter_tau_s: float = 0.20
    prediction_time_s: float = 0.0
    max_accel_kmh_s: float = 18.0
    gain_map_kmh: tuple[float, ...] = (0.0, 40.0, 80.0, 140.0, 220.0)
    accel_gain_map: tuple[float, ...] = (1.0, 1.0, 0.85, 0.75, 0.70)
    brake_gain_map: tuple[float, ...] = (0.85, 1.0, 1.10, 1.25, 1.35)
    accel_limit_map: tuple[float, ...] = (1.00, 1.00, 0.85, 0.70, 0.55)


class SpeedController:
    """Small PI controller that maps speed error to throttle and brake."""

    def __init__(self, config: SpeedControllerConfig) -> None:
        self.config = config
        self._accel_integral = 0.0
        self._brake_integral = 0.0
        self._last_speed_kmh: float | None = None
        self._speed_rate_kmh_s = 0.0
        self.throttle = 0.0
        self.brake = 0.0

    def reset(self) -> None:
        self._accel_integral = 0.0
        self._brake_integral = 0.0
        self._last_speed_kmh = None
        self._speed_rate_kmh_s = 0.0
        self.throttle = 0.0
        self.brake = 0.0

    def step(
        self,
        speed_kmh: float,
        dt: float,
        target_kmh: float | None = None,
    ) -> tuple[float, float]:
        cfg = self.config
        target = cfg.target_kmh if target_kmh is None else target_kmh
        accel_gain = _interpolate_map(
            speed_kmh,
            cfg.gain_map_kmh,
            cfg.accel_gain_map,
        )
        brake_gain = _interpolate_map(
            speed_kmh,
            cfg.gain_map_kmh,
            cfg.brake_gain_map,
        )
        accel_limit = cfg.max_accel_kmh_s * _interpolate_map(
            speed_kmh,
            cfg.gain_map_kmh,
            cfg.accel_limit_map,
        )
        if self._last_speed_kmh is not None and dt > 0.0:
            measured_rate = (speed_kmh - self._last_speed_kmh) / dt
            alpha = dt / (cfg.speed_rate_filter_tau_s + dt)
            self._speed_rate_kmh_s += alpha * (
                measured_rate - self._speed_rate_kmh_s
            )
        self._last_speed_kmh = speed_kmh

        predicted_speed = (
            speed_kmh + cfg.prediction_time_s * self._speed_rate_kmh_s
        )
        error = target - speed_kmh
        predicted_error = target - predicted_speed
        if abs(predicted_error) < cfg.deadband_kmh:
            predicted_error = 0.0

        if predicted_error >= 0.0:
            self._brake_integral = 0.0
            self._accel_integral += error * dt
            integral_term = clamp(
                cfg.ki * self._accel_integral,
                -cfg.integral_limit,
                cfg.integral_limit,
            )
            target_throttle = clamp(
                cfg.kp_accel * accel_gain * predicted_error + integral_term,
                0.0,
                1.0,
            )
            target_brake = 0.0
        else:
            self._accel_integral = 0.0
            self._brake_integral += -error * dt
            integral_term = clamp(
                cfg.ki * self._brake_integral,
                -cfg.integral_limit,
                cfg.integral_limit,
            )
            target_brake = clamp(
                cfg.kp_brake * brake_gain * (-predicted_error) + integral_term,
                0.0,
                1.0,
            )
            target_throttle = 0.0

        if self._speed_rate_kmh_s > accel_limit:
            target_throttle = 0.0

        self.throttle = _rate_limit(
            self.throttle,
            target_throttle,
            cfg.throttle_rate_per_s * dt,
        )
        self.brake = _rate_limit(
            self.brake,
            target_brake,
            cfg.brake_rate_per_s * dt,
        )
        return self.throttle, self.brake


def _rate_limit(current: float, target: float, max_delta: float) -> float:
    return current + clamp(target - current, -max_delta, max_delta)


def _interpolate_map(
    value: float,
    breakpoints: tuple[float, ...],
    values: tuple[float, ...],
) -> float:
    if not breakpoints or len(breakpoints) != len(values):
        return 1.0
    if value <= breakpoints[0]:
        return values[0]
    if value >= breakpoints[-1]:
        return values[-1]
    for index in range(1, len(breakpoints)):
        if value <= breakpoints[index]:
            lower = breakpoints[index - 1]
            upper = breakpoints[index]
            fraction = (value - lower) / max(upper - lower, 1e-9)
            return (
                values[index - 1]
                + fraction * (values[index] - values[index - 1])
            )
    return values[-1]


@dataclass
class HeadingControllerConfig:
    kp: float = 1.20
    kd: float = 0.25
    steer_limit: float = 1.0
    steer_rate_per_s: float = 2.5
    steer_sign: float = 1.0


class HeadingController:
    """Hold a target yaw heading.

    This is only a straight-line steering check. Track tracking should replace it
    with a Frenet/LQR or MPC controller once the input path is verified.
    """

    def __init__(
        self,
        config: HeadingControllerConfig,
        target_heading_rad: float | None = None,
    ) -> None:
        self.config = config
        self.target_heading_rad = target_heading_rad
        self.steer = 0.0

    def reset(self) -> None:
        self.steer = 0.0

    def step(self, heading_rad: float, yaw_rate_rad_s: float, dt: float) -> float:
        if self.target_heading_rad is None:
            self.target_heading_rad = heading_rad

        cfg = self.config
        heading_error = wrap_pi(self.target_heading_rad - heading_rad)
        raw_steer = cfg.steer_sign * (cfg.kp * heading_error - cfg.kd * yaw_rate_rad_s)
        target_steer = clamp(raw_steer, -cfg.steer_limit, cfg.steer_limit)
        self.steer = _rate_limit(
            self.steer,
            target_steer,
            cfg.steer_rate_per_s * dt,
        )
        return self.steer
