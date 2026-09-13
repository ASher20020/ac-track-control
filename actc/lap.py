from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_discrete_are

from .shared_memory import VehicleState
from .track import TrackPath, clamp, physics_heading_to_world, wrap_pi


def _lookup_map(
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
class PurePursuitConfig:
    wheelbase_m: float = 2.85
    max_steer_rad: float = 0.52
    lookahead_min_m: float = 12.0
    lookahead_gain_s: float = 1.20
    lookahead_max_m: float = 35.0
    merge_distance_m: float = 250.0
    cross_track_yaw_gain: float = 0.08
    max_yaw_correction_rad_s: float = 0.45
    heading_gain: float = 0.35
    yaw_rate_gain: float = 0.90
    curvature_weight: float = 0.25
    steering_gain: float = 0.90
    steer_rate_per_s: float = 3.0
    steer_limit: float = 0.70
    steering_sign: float = 1.0
    heading_sign: float = 1.0


@dataclass(frozen=True)
class LapControlInfo:
    path_index: int
    lookahead_index: int
    target_speed_kmh: float
    lateral_error_m: float
    heading_error_rad: float
    lookahead_m: float
    steer: float
    raw_steer: float | None = None


class PurePursuitController:
    def __init__(self, config: PurePursuitConfig | None = None) -> None:
        self.config = config or PurePursuitConfig()
        self.previous_index: int | None = None
        self.heading_offset_rad: float | None = None
        self.initial_lateral_error_m = 0.0
        self.elapsed_distance_m = 0.0
        self.steer = 0.0

    def calibrate(self, state: VehicleState, path: TrackPath) -> None:
        self.previous_index = path.nearest_index(
            state.position[0],
            state.position[2],
        )
        self.heading_offset_rad = 0.0
        self.initial_lateral_error_m = path.signed_lateral_error(
            self.previous_index,
            state.position[0],
            state.position[2],
        )
        self.elapsed_distance_m = 0.0
        self.steer = 0.0

    def step(
        self,
        state: VehicleState,
        path: TrackPath,
        dt: float,
    ) -> LapControlInfo:
        if self.heading_offset_rad is None:
            self.calibrate(state, path)
        assert self.heading_offset_rad is not None

        index = path.nearest_index(
            state.position[0],
            state.position[2],
            self.previous_index,
        )
        self.previous_index = index
        self.elapsed_distance_m += max(0.0, state.speed_ms) * dt

        speed_mps = max(0.0, state.speed_ms)
        lookahead_m = clamp(
            self.config.lookahead_min_m
            + self.config.lookahead_gain_s * speed_mps,
            self.config.lookahead_min_m,
            self.config.lookahead_max_m,
        )
        target_index = path.lookahead_index(index, lookahead_m)
        target_x, target_z = path.points[target_index]
        car_x, car_z = state.position[0], state.position[2]
        car_heading = physics_heading_to_world(state.heading_rad)
        heading_error = wrap_pi(path.heading_rad[index] - car_heading)
        lateral_error = path.signed_lateral_error(
            index,
            state.position[0],
            state.position[2],
        )
        merge_fraction = clamp(
            1.0 - self.elapsed_distance_m / self.config.merge_distance_m,
            0.0,
            1.0,
        )
        desired_offset = self.initial_lateral_error_m * merge_fraction * merge_fraction
        control_lateral_error = lateral_error - desired_offset

        distance = max(math.hypot(target_x - car_x, target_z - car_z), 1.0)
        yaw_correction = clamp(
            -self.config.cross_track_yaw_gain * control_lateral_error,
            -self.config.max_yaw_correction_rad_s,
            self.config.max_yaw_correction_rad_s,
        )

        curvature = path.curvature[index]
        curvature_angle = math.atan(self.config.wheelbase_m * curvature)
        curvature_steer = curvature_angle / self.config.max_steer_rad
        measured_yaw_rate = -state.yaw_rate_rad_s
        desired_yaw_rate = state.speed_ms * curvature + yaw_correction
        yaw_rate_error = desired_yaw_rate - measured_yaw_rate

        feedback = (
            self.config.heading_gain * heading_error
            + self.config.yaw_rate_gain * yaw_rate_error
        )
        target_steer = self.config.steering_sign * self.config.steering_gain * (
            feedback + self.config.curvature_weight * curvature_steer
        )
        target_steer = clamp(
            target_steer,
            -self.config.steer_limit,
            self.config.steer_limit,
        )
        max_delta = self.config.steer_rate_per_s * dt
        self.steer += clamp(
            target_steer - self.steer,
            -max_delta,
            max_delta,
        )
        self.steer = clamp(
            self.steer,
            -self.config.steer_limit,
            self.config.steer_limit,
        )

        return LapControlInfo(
            path_index=index,
            lookahead_index=target_index,
            target_speed_kmh=path.speed_limit_kmh[index],
            lateral_error_m=lateral_error,
            heading_error_rad=heading_error,
            lookahead_m=distance,
            steer=self.steer,
        )


@dataclass
class StanleyConfig:
    wheelbase_m: float = 2.85
    max_steer_rad: float = 0.52
    cross_track_gain: float = 0.65
    softening_speed_mps: float = 3.0
    heading_gain: float = 1.00
    curvature_weight: float = 1.0
    yaw_rate_gain: float = 0.15
    steering_gain: float = 1.0
    steer_rate_per_s: float = 1.5
    steering_filter_tau_s: float = 0.10
    steering_filter_window: int = 4
    steer_limit: float = 0.70
    speed_gain_map_kmh: tuple[float, ...] = (0.0, 40.0, 80.0, 140.0, 220.0)
    cross_track_gain_map: tuple[float, ...] = (1.10, 1.00, 1.05, 1.15, 1.25)
    heading_gain_map: tuple[float, ...] = (0.90, 0.95, 1.00, 1.05, 1.10)
    yaw_rate_gain_map: tuple[float, ...] = (1.20, 1.10, 1.00, 1.10, 1.25)
    filter_tau_map: tuple[float, ...] = (0.85, 0.95, 1.00, 1.10, 1.20)
    steering_sign: float = 1.0
    heading_sign: float = 1.0
    merge_distance_m: float = 150.0


class StanleyController:
    def __init__(self, config: StanleyConfig | None = None) -> None:
        self.config = config or StanleyConfig()
        self.previous_index: int | None = None
        self.heading_offset_rad: float | None = None
        self.initial_cross_track_error_m = 0.0
        self.elapsed_distance_m = 0.0
        self.steer = 0.0
        self._target_history: deque[float] = deque(
            maxlen=max(1, self.config.steering_filter_window)
        )

    def calibrate(self, state: VehicleState, path: TrackPath) -> None:
        center_index = path.nearest_index(
            state.position[0],
            state.position[2],
        )
        self.heading_offset_rad = 0.0
        car_heading = physics_heading_to_world(state.heading_rad)
        front_x = state.position[0] + self.config.wheelbase_m * math.cos(car_heading)
        front_z = state.position[2] + self.config.wheelbase_m * math.sin(car_heading)
        self.previous_index = path.nearest_index(front_x, front_z)
        self.initial_cross_track_error_m = self._cross_track_error(
            path,
            self.previous_index,
            front_x,
            front_z,
            car_heading,
        )
        self.elapsed_distance_m = 0.0
        self.steer = 0.0
        self._target_history.clear()

    @staticmethod
    def _cross_track_error(
        path: TrackPath,
        index: int,
        front_x: float,
        front_z: float,
        _car_heading: float,
    ) -> float:
        # Positive means that the front axle is left of the path.
        return path.signed_lateral_error(index, front_x, front_z)

    def step(
        self,
        state: VehicleState,
        path: TrackPath,
        dt: float,
    ) -> LapControlInfo:
        if self.heading_offset_rad is None:
            self.calibrate(state, path)
        assert self.heading_offset_rad is not None

        car_heading = physics_heading_to_world(state.heading_rad)
        front_x = state.position[0] + self.config.wheelbase_m * math.cos(car_heading)
        front_z = state.position[2] + self.config.wheelbase_m * math.sin(car_heading)
        index = path.nearest_index(front_x, front_z, self.previous_index)
        self.previous_index = index
        self.elapsed_distance_m += max(0.0, state.speed_ms) * dt

        cross_track_error = self._cross_track_error(
            path,
            index,
            front_x,
            front_z,
            car_heading,
        )
        merge_fraction = clamp(
            1.0 - self.elapsed_distance_m / self.config.merge_distance_m,
            0.0,
            1.0,
        )
        desired_offset = (
            self.initial_cross_track_error_m * merge_fraction * merge_fraction
        )
        control_error = cross_track_error - desired_offset
        heading_error = wrap_pi(path.heading_rad[index] - car_heading)
        speed_kmh = state.speed_ms * 3.6
        cross_track_gain = self.config.cross_track_gain * _lookup_map(
            speed_kmh,
            self.config.speed_gain_map_kmh,
            self.config.cross_track_gain_map,
        )
        heading_gain = self.config.heading_gain * _lookup_map(
            speed_kmh,
            self.config.speed_gain_map_kmh,
            self.config.heading_gain_map,
        )
        yaw_rate_gain = self.config.yaw_rate_gain * _lookup_map(
            speed_kmh,
            self.config.speed_gain_map_kmh,
            self.config.yaw_rate_gain_map,
        )
        filter_tau = self.config.steering_filter_tau_s * _lookup_map(
            speed_kmh,
            self.config.speed_gain_map_kmh,
            self.config.filter_tau_map,
        )

        cross_track_term = math.atan2(
            -cross_track_gain * control_error,
            max(state.speed_ms, self.config.softening_speed_mps),
        )
        curvature_angle = math.atan(
            self.config.wheelbase_m * path.curvature[index]
        )
        world_yaw_rate = -state.yaw_rate_rad_s
        desired_yaw_rate = state.speed_ms * path.curvature[index]
        yaw_rate_error = desired_yaw_rate - world_yaw_rate
        steering_angle = (
            heading_gain * heading_error
            + cross_track_term
            + self.config.curvature_weight * curvature_angle
            + yaw_rate_gain * yaw_rate_error
        )
        target_steer = self.config.steering_sign * self.config.steering_gain * (
            steering_angle / self.config.max_steer_rad
        )
        target_steer = clamp(
            target_steer,
            -self.config.steer_limit,
            self.config.steer_limit,
        )
        self._target_history.append(target_steer)
        smoothed_target = sum(self._target_history) / len(self._target_history)
        filter_alpha = dt / (
            max(filter_tau, 1e-3) + dt
        )
        filtered_steer = self.steer + filter_alpha * (
            smoothed_target - self.steer
        )
        max_delta = self.config.steer_rate_per_s * dt
        self.steer += clamp(
            filtered_steer - self.steer,
            -max_delta,
            max_delta,
        )
        self.steer = clamp(
            self.steer,
            -self.config.steer_limit,
            self.config.steer_limit,
        )

        return LapControlInfo(
            path_index=index,
            lookahead_index=index,
            target_speed_kmh=path.speed_limit_kmh[index],
            lateral_error_m=cross_track_error,
            heading_error_rad=heading_error,
            lookahead_m=0.0,
            steer=self.steer,
        )


@dataclass
class LqrConfig:
    wheelbase_m: float = 2.85
    max_steer_rad: float = 0.52
    q_lateral: float = 3.0
    q_lateral_rate: float = 0.8
    q_heading: float = 1.5
    q_heading_rate: float = 0.4
    r_steering: float = 1.0
    steering_gain: float = 1.0
    steering_sign: float = 1.0
    heading_sign: float = 1.0
    steer_rate_per_s: float = 3.0
    steer_limit: float = 0.70
    merge_distance_m: float = 150.0


class LqrController:
    """PythonRobotics-style LQR steering controller for Frenet tracking."""

    def __init__(self, config: LqrConfig | None = None) -> None:
        self.config = config or LqrConfig()
        self.previous_index: int | None = None
        self.heading_offset_rad: float | None = None
        self.initial_lateral_error_m = 0.0
        self.elapsed_distance_m = 0.0
        self.previous_lateral_error = 0.0
        self.previous_heading_error = 0.0
        self.steer = 0.0
        self._gain_cache: dict[tuple[float, int], np.ndarray] = {}

    def calibrate(self, state: VehicleState, path: TrackPath) -> None:
        self.previous_index = path.nearest_index(
            state.position[0],
            state.position[2],
        )
        self.heading_offset_rad = 0.0
        self.initial_lateral_error_m = self._lateral_error(
            state,
            path,
            self.previous_index,
        )
        self.elapsed_distance_m = 0.0
        self.previous_lateral_error = 0.0
        self.previous_heading_error = 0.0
        self.steer = 0.0

    def step(
        self,
        state: VehicleState,
        path: TrackPath,
        dt: float,
    ) -> LapControlInfo:
        if self.heading_offset_rad is None:
            self.calibrate(state, path)
        assert self.heading_offset_rad is not None

        index = path.nearest_index(
            state.position[0],
            state.position[2],
            self.previous_index,
        )
        self.previous_index = index
        self.elapsed_distance_m += max(0.0, state.speed_ms) * dt

        car_heading = physics_heading_to_world(state.heading_rad)
        lateral_error = self._lateral_error(state, path, index)
        merge_fraction = clamp(
            1.0 - self.elapsed_distance_m / self.config.merge_distance_m,
            0.0,
            1.0,
        )
        desired_offset = (
            self.initial_lateral_error_m * merge_fraction * merge_fraction
        )
        control_error = lateral_error - desired_offset
        heading_error = wrap_pi(car_heading - path.heading_rad[index])

        safe_dt = max(dt, 1e-3)
        lateral_rate = (
            control_error - self.previous_lateral_error
        ) / safe_dt
        heading_rate = (
            heading_error - self.previous_heading_error
        ) / safe_dt
        self.previous_lateral_error = control_error
        self.previous_heading_error = heading_error

        speed = max(1.0, state.speed_ms)
        k = self._gain(speed, dt)
        error_state = np.array(
            [[control_error], [lateral_rate], [heading_error], [heading_rate]],
            dtype=float,
        )
        feedforward = math.atan(self.config.wheelbase_m * path.curvature[index])
        feedback = float(-(k @ error_state)[0, 0])
        steering_angle = feedforward + feedback
        target_steer = self.config.steering_sign * self.config.steering_gain * (
            steering_angle / self.config.max_steer_rad
        )
        target_steer = clamp(
            target_steer,
            -self.config.steer_limit,
            self.config.steer_limit,
        )
        self.steer += clamp(
            target_steer - self.steer,
            -self.config.steer_rate_per_s * dt,
            self.config.steer_rate_per_s * dt,
        )
        self.steer = clamp(
            self.steer,
            -self.config.steer_limit,
            self.config.steer_limit,
        )

        return LapControlInfo(
            path_index=index,
            lookahead_index=index,
            target_speed_kmh=path.speed_limit_kmh[index],
            lateral_error_m=lateral_error,
            heading_error_rad=heading_error,
            lookahead_m=0.0,
            steer=self.steer,
        )

    def _gain(self, speed_mps: float, dt: float) -> np.ndarray:
        speed_key = round(speed_mps * 2.0) / 2.0
        dt_key = max(1, round(dt * 1000.0))
        key = (speed_key, dt_key)
        cached = self._gain_cache.get(key)
        if cached is not None:
            return cached

        model_dt = dt_key / 1000.0
        speed = max(3.0, speed_key)
        a_matrix = np.array(
            [
                [1.0, model_dt, 0.0, 0.0],
                [0.0, 1.0, speed, 0.0],
                [0.0, 0.0, 1.0, model_dt],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        b_matrix = np.array(
            [[0.0], [0.0], [0.0], [speed / self.config.wheelbase_m]],
            dtype=float,
        )
        q_matrix = np.diag(
            [
                self.config.q_lateral,
                self.config.q_lateral_rate,
                self.config.q_heading,
                self.config.q_heading_rate,
            ]
        )
        r_matrix = np.array([[self.config.r_steering]], dtype=float)
        riccati = solve_discrete_are(a_matrix, b_matrix, q_matrix, r_matrix)
        gain = np.linalg.solve(
            r_matrix + b_matrix.T @ riccati @ b_matrix,
            b_matrix.T @ riccati @ a_matrix,
        )
        self._gain_cache[key] = gain
        return gain

    @staticmethod
    def _lateral_error(
        state: VehicleState,
        path: TrackPath,
        index: int,
    ) -> float:
        reference_x, reference_z = path.points[index]
        dx = reference_x - state.position[0]
        dz = reference_z - state.position[2]
        distance = math.hypot(dx, dz)
        if distance < 1e-9:
            return 0.0
        north_angle = math.atan2(dz, dx)
        angle = wrap_pi(path.heading_rad[index] - north_angle)
        return distance if angle >= 0.0 else -distance
