from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import osqp
from scipy import sparse


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _coast_down_acceleration(
    speed_mps: float,
    config: "LongitudinalMpcConfig",
) -> float:
    if (
        not config.coast_down_accel_map_kmh
        or len(config.coast_down_accel_map_kmh)
        != len(config.coast_down_accel_map_mps2)
    ):
        return 0.0
    return float(
        np.interp(
            max(0.0, speed_mps) * 3.6,
            config.coast_down_accel_map_kmh,
            config.coast_down_accel_map_mps2,
        )
    )


@dataclass
class LongitudinalMpcConfig:
    horizon: int = 30
    dt: float = 0.05
    response_tau_s: float = 0.10
    q_speed: float = 18.0
    q_accel: float = 0.8
    r_accel: float = 0.6
    r_jerk: float = 25.0
    max_accel_mps2: float = 2.2
    max_brake_mps2: float = 5.6
    max_jerk_mps3: float = 5.0
    max_brake_jerk_mps3: float = 18.0
    reference_filter_up_tau_s: float = 0.35
    reference_filter_down_tau_s: float = 0.10
    lateral_weight_schedule_g: tuple[float, ...] = (
        0.0,
        0.3,
        0.6,
        0.9,
        1.2,
    )
    r_accel_lateral_scale: tuple[float, ...] = (
        1.0,
        1.10,
        1.30,
        1.55,
        1.80,
    )
    r_jerk_lateral_scale: tuple[float, ...] = (
        1.0,
        1.20,
        1.50,
        1.90,
        2.40,
    )
    speed_weight_schedule_kmh: tuple[float, ...] = (
        0.0,
        80.0,
        140.0,
        200.0,
        260.0,
    )
    r_accel_speed_scale: tuple[float, ...] = (
        1.0,
        1.05,
        1.15,
        1.30,
        1.45,
    )
    r_jerk_speed_scale: tuple[float, ...] = (
        1.0,
        1.15,
        1.40,
        1.75,
        2.00,
    )
    coast_down_accel_map_kmh: tuple[float, ...] = (
        0.0,
        40.0,
        80.0,
        120.0,
        160.0,
        200.0,
        240.0,
        280.0,
    )
    coast_down_accel_map_mps2: tuple[float, ...] = (
        0.35,
        0.65,
        0.75,
        0.85,
        1.05,
        1.35,
        1.75,
        2.20,
    )
    eps_abs: float = 1.0e-4
    eps_rel: float = 1.0e-4
    max_iter: int = 300
    polish: bool = False


@dataclass
class LongitudinalMpcResult:
    acceleration_command_mps2: float
    status: str
    solve_time_ms: float
    iterations: int


@dataclass
class LongitudinalPedalConfig:
    max_throttle: float = 1.0
    throttle_limit_map_kmh: tuple[float, ...] = (
        0.0,
        50.0,
        100.0,
        160.0,
        240.0,
    )
    throttle_limit_map: tuple[float, ...] = (
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
    )
    throttle_rate_up_per_s: float = 2.0
    throttle_rate_down_per_s: float = 2.5
    brake_rate_up_per_s: float = 2.0
    brake_rate_down_per_s: float = 3.0
    steering_throttle_reduction: float = 0.25
    slip_throttle_cut: float = 0.35
    unlimited_below_kmh: float = 20.0
    acceleration_deadband_mps2: float = 0.35
    coast_brake_enter_mps2: float = 0.50
    coast_brake_exit_mps2: float = 0.15
    coast_throttle_enter_mps2: float = 0.15
    braking_response_speed_kmh: tuple[float, ...] = (
        0.0,
        30.0,
        60.0,
        90.0,
        120.0,
        160.0,
        200.0,
        260.0,
    )
    braking_response_pedal: tuple[float, ...] = (
        0.0,
        0.15,
        0.30,
        0.45,
        0.60,
        0.75,
        0.90,
        1.00,
    )
    braking_response_decel_mps2: tuple[tuple[float, ...], ...] = (
        (0.0, 2.73, 4.50, 6.87, 9.29, 11.30, 13.10, 14.50),
        (0.0, 2.45, 4.28, 6.46, 8.52, 10.72, 12.37, 13.43),
        (0.0, 2.47, 4.59, 6.61, 9.21, 11.01, 12.86, 14.51),
        (0.0, 2.60, 4.33, 6.82, 9.05, 11.54, 13.47, 15.12),
        (0.0, 2.00, 4.41, 6.88, 9.49, 11.89, 14.57, 15.46),
        (0.0, 1.29, 3.95, 6.19, 8.93, 11.55, 13.85, 16.39),
        (0.0, 2.00, 3.23, 5.90, 8.77, 11.11, 13.95, 17.19),
        (0.0, 2.00, 3.23, 5.90, 8.77, 11.11, 13.95, 17.19),
    )


class LongitudinalPedalMapper:
    """Rate-limited pedal mapper with a conservative traction envelope."""

    def __init__(
        self,
        config: LongitudinalPedalConfig | None = None,
    ) -> None:
        self.config = config or LongitudinalPedalConfig()
        self.throttle = 0.0
        self.brake = 0.0
        self._pedal_mode = "coast"

    def reset(self) -> None:
        self.throttle = 0.0
        self.brake = 0.0
        self._pedal_mode = "coast"

    def step(
        self,
        *,
        acceleration_mps2: float,
        speed_mps: float,
        steering: float,
        rear_slip: float,
        dt: float,
        acceleration_command_mps2: float | None = None,
        drag_acceleration_mps2: float = 0.0,
    ) -> tuple[float, float]:
        desired_throttle, desired_brake = map_acceleration_to_pedals(
            acceleration_mps2,
            speed_mps,
            drag_acceleration_mps2=drag_acceleration_mps2,
        )
        propulsion_acceleration = (
            acceleration_mps2
            + max(0.0, drag_acceleration_mps2)
        )
        if self._pedal_mode == "brake":
            if propulsion_acceleration > -self.config.coast_brake_exit_mps2:
                self._pedal_mode = "coast"
        elif self._pedal_mode == "throttle":
            if propulsion_acceleration < self.config.coast_throttle_enter_mps2:
                self._pedal_mode = "coast"
        elif propulsion_acceleration < -self.config.coast_brake_enter_mps2:
            self._pedal_mode = "brake"
        elif propulsion_acceleration > self.config.coast_throttle_enter_mps2:
            self._pedal_mode = "throttle"

        if self._pedal_mode == "coast":
            desired_throttle = 0.0
            desired_brake = 0.0
        elif self._pedal_mode == "brake":
            desired_throttle = 0.0
            desired_brake = brake_command_for_deceleration(
                -propulsion_acceleration,
                speed_mps,
                self.config,
            )
        else:
            desired_throttle = clamp(
                propulsion_acceleration
                / max(
                    acceleration_capability_mps2(speed_mps),
                    1e-3,
                ),
                0.0,
                1.0,
            )
            desired_brake = 0.0
        return self.limit_pedals(
            throttle=desired_throttle,
            brake=desired_brake,
            speed_mps=speed_mps,
            steering=steering,
            rear_slip=rear_slip,
            acceleration_command_mps2=acceleration_mps2,
            dt=dt,
        )

    def limit_pedals(
        self,
        *,
        throttle: float,
        brake: float,
        speed_mps: float,
        steering: float,
        rear_slip: float,
        dt: float,
        acceleration_command_mps2: float | None = None,
        drag_acceleration_mps2: float = 0.0,
    ) -> tuple[float, float]:
        throttle_limit = self._throttle_limit(
            speed_mps,
            steering,
            rear_slip,
        )
        desired_throttle = min(max(0.0, throttle), throttle_limit)
        desired_brake = clamp(brake, 0.0, 1.0)
        self.throttle = _rate_limit(
            self.throttle,
            desired_throttle,
            (
                self.config.throttle_rate_up_per_s
                if desired_throttle >= self.throttle
                else self.config.throttle_rate_down_per_s
            )
            * max(dt, 1e-3),
        )
        self.brake = _rate_limit(
            self.brake,
            desired_brake,
            (
                self.config.brake_rate_up_per_s
                if desired_brake >= self.brake
                else self.config.brake_rate_down_per_s
            )
            * max(dt, 1e-3),
        )
        if self.throttle > 0.02 and self.brake > 0.02:
            if desired_throttle >= desired_brake:
                self.brake = 0.0
            else:
                self.throttle = 0.0
        return self.throttle, self.brake

    def _throttle_limit(
        self,
        speed_mps: float,
        steering: float,
        rear_slip: float,
    ) -> float:
        speed_kmh = max(0.0, speed_mps * 3.6)
        if speed_kmh < self.config.unlimited_below_kmh:
            return 1.0
        speed_limit = float(
            np.interp(
                speed_kmh,
                self.config.throttle_limit_map_kmh,
                self.config.throttle_limit_map,
            )
        )
        steering_scale = 1.0 - (
            self.config.steering_throttle_reduction
            * clamp(abs(steering), 0.0, 1.0)
        )
        slip_scale = 1.0
        if rear_slip > self.config.slip_throttle_cut:
            slip_scale = clamp(
                1.0
                - 0.55
                * (
                    rear_slip
                    - self.config.slip_throttle_cut
                )
                / max(1.0 - self.config.slip_throttle_cut, 1e-3),
                0.35,
                1.0,
            )
        return min(
            self.config.max_throttle,
            speed_limit * steering_scale * slip_scale,
        )


class LongitudinalMpcController:
    """Small incremental acceleration MPC solved as a condensed QP."""

    def __init__(self, config: LongitudinalMpcConfig | None = None) -> None:
        self.config = config or LongitudinalMpcConfig()
        self._build_weight_schedules()
        self._solver: osqp.OSQP | None = None
        self._previous_acceleration_command = 0.0
        self._q = np.zeros(self.config.horizon)

    def reset(self, acceleration_command_mps2: float = 0.0) -> None:
        self._previous_acceleration_command = float(
            acceleration_command_mps2
        )
        self._solver = None

    def coast_down_acceleration(self, speed_mps: float) -> float:
        return _coast_down_acceleration(speed_mps, self.config)

    def rebuild_solver(self) -> None:
        self._build_weight_schedules()

    def _build_weight_schedules(self) -> None:
        self._p_matrices: list[list[sparse.csc_matrix]] = []
        for accel_scale, jerk_scale in zip(
            self.config.r_accel_lateral_scale,
            self.config.r_jerk_lateral_scale,
        ):
            speed_matrices = []
            for speed_accel_scale, speed_jerk_scale in zip(
                self.config.r_accel_speed_scale,
                self.config.r_jerk_speed_scale,
            ):
                (
                    p_matrix,
                    self._a_matrix,
                    self._control_rows,
                    self._rate_rows,
                ) = self._build_problem_matrices(
                    accel_scale * speed_accel_scale,
                    jerk_scale * speed_jerk_scale,
                )
                speed_matrices.append(p_matrix)
            self._p_matrices.append(speed_matrices)
        self._weight_schedule_index = -1
        self._speed_weight_schedule_index = -1
        self._p_matrix = self._p_matrices[0][0]
        self._solver = None

    def _schedule_index(
        self,
        schedule: tuple[float, ...],
        value: float,
    ) -> int:
        if not schedule:
            return 0
        value = abs(float(value))
        if value <= schedule[0]:
            return 0
        for index in range(1, len(schedule)):
            if value <= schedule[index]:
                return index
        return len(schedule) - 1

    def _select_weight_schedule(
        self,
        lateral_accel_g: float,
        speed_mps: float,
    ) -> None:
        lateral_index = min(
            self._schedule_index(
                self.config.lateral_weight_schedule_g,
                lateral_accel_g,
            ),
            len(self._p_matrices) - 1,
        )
        speed_index = min(
            self._schedule_index(
                self.config.speed_weight_schedule_kmh,
                max(0.0, speed_mps) * 3.6,
            ),
            len(self._p_matrices[lateral_index]) - 1,
        )
        if (
            lateral_index == self._weight_schedule_index
            and speed_index == self._speed_weight_schedule_index
        ):
            return
        self._weight_schedule_index = lateral_index
        self._speed_weight_schedule_index = speed_index
        self._p_matrix = self._p_matrices[lateral_index][speed_index]
        self._solver = None

    def _build_prediction_matrices(
        self,
    ) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.config
        horizon = cfg.horizon
        alpha = cfg.dt / max(cfg.response_tau_s, 1e-3)
        alpha = clamp(alpha, 0.0, 1.0)
        speed_from_input = np.zeros((horizon + 1, horizon))
        acceleration_from_input = np.zeros((horizon + 1, horizon))
        for stage in range(horizon):
            for previous in range(stage + 1):
                acceleration_from_input[stage + 1, previous] = (
                    (1.0 - alpha)
                    * acceleration_from_input[stage, previous]
                    + (alpha if previous == stage else 0.0)
                )
                speed_from_input[stage + 1, previous] = (
                    speed_from_input[stage, previous]
                    + cfg.dt
                    * acceleration_from_input[stage + 1, previous]
                )
        return speed_from_input, acceleration_from_input

    def _build_problem_matrices(
        self,
        r_accel_scale: float = 1.0,
        r_jerk_scale: float = 1.0,
    ) -> tuple[sparse.csc_matrix, sparse.csc_matrix, int, int]:
        cfg = self.config
        self._speed_from_input, self._acceleration_from_input = (
            self._build_prediction_matrices()
        )
        self._increment_matrix = sparse.tril(
            np.ones((cfg.horizon, cfg.horizon)),
            format="csc",
        )
        self._speed_from_increment = (
            self._speed_from_input @ self._increment_matrix
        )
        self._acceleration_from_increment = (
            self._acceleration_from_input
            @ self._increment_matrix
        )
        p_matrix = 2.0 * (
            cfg.q_speed
            * self._speed_from_increment.T
            @ self._speed_from_increment
            + cfg.q_accel
            * self._acceleration_from_increment.T
            @ self._acceleration_from_increment
            + cfg.r_accel
            * r_accel_scale
            * self._increment_matrix.T
            @ self._increment_matrix
            + cfg.r_jerk
            * r_jerk_scale
            * sparse.eye(cfg.horizon, format="csc")
        )
        p_matrix += sparse.eye(cfg.horizon, format="csc") * 1e-6
        a_matrix = sparse.vstack(
            [
                self._increment_matrix,
                sparse.eye(cfg.horizon, format="csc"),
            ],
            format="csc",
        )
        return (
            sparse.csc_matrix(p_matrix),
            a_matrix,
            cfg.horizon,
            cfg.horizon,
        )

    def step(
        self,
        *,
        speed_mps: float,
        acceleration_mps2: float,
        reference_speed_mps: np.ndarray,
        acceleration_limit_mps2: float | None = None,
        braking_limit_mps2: float | None = None,
        lateral_accel_g: float = 0.0,
    ) -> LongitudinalMpcResult:
        self._select_weight_schedule(lateral_accel_g, speed_mps)
        cfg = self.config
        reference = np.asarray(reference_speed_mps, dtype=float)
        if reference.size < cfg.horizon + 1:
            reference = np.pad(
                reference,
                (0, cfg.horizon + 1 - reference.size),
                mode="edge",
            )
        reference = reference[: cfg.horizon + 1]
        max_acceleration = (
            cfg.max_accel_mps2
            if acceleration_limit_mps2 is None
            else min(
                cfg.max_accel_mps2,
                max(0.0, acceleration_limit_mps2),
            )
        )
        max_braking = (
            cfg.max_brake_mps2
            if braking_limit_mps2 is None
            else min(
                cfg.max_brake_mps2,
                max(0.0, braking_limit_mps2),
            )
        )
        previous_command_value = clamp(
            self._previous_acceleration_command,
            -max_braking,
            max_acceleration,
        )
        acceleration_affine = np.zeros(cfg.horizon + 1)
        speed_affine = np.zeros(cfg.horizon + 1)
        speed_affine[0] = speed_mps
        acceleration_affine[0] = acceleration_mps2
        alpha = clamp(
            cfg.dt / max(cfg.response_tau_s, 1e-3),
            0.0,
            1.0,
        )
        for stage in range(cfg.horizon):
            acceleration_affine[stage + 1] = (
                (1.0 - alpha) * acceleration_affine[stage]
                + alpha * previous_command_value
            )
            coast_drag = _coast_down_acceleration(
                speed_affine[stage],
                cfg,
            )
            speed_affine[stage + 1] = (
                speed_affine[stage]
                + cfg.dt
                * (acceleration_affine[stage + 1] - coast_drag)
            )
        acceleration_reference = np.gradient(
            reference,
            cfg.dt,
        )
        linear = 2.0 * (
            cfg.q_speed
            * self._speed_from_increment.T
            @ (speed_affine - reference)
            + cfg.q_accel
            * self._acceleration_from_increment.T
            @ (acceleration_affine - acceleration_reference)
        )

        control_lower = np.full(
            cfg.horizon,
            -max_braking,
        )
        control_upper = np.full(
            cfg.horizon,
            max_acceleration,
        )
        previous_command = np.full(
            cfg.horizon,
            previous_command_value,
        )
        increment_lower = np.full(
            cfg.horizon,
            -cfg.max_brake_jerk_mps3 * cfg.dt,
        )
        increment_upper = np.full(
            cfg.horizon,
            cfg.max_jerk_mps3 * cfg.dt,
        )
        lower = np.concatenate(
            [control_lower - previous_command, increment_lower]
        )
        upper = np.concatenate(
            [control_upper - previous_command, increment_upper]
        )

        if self._solver is None:
            self._solver = osqp.OSQP()
            self._solver.setup(
                P=self._p_matrix,
                q=linear,
                A=self._a_matrix,
                l=lower,
                u=upper,
                verbose=False,
                polishing=cfg.polish,
                eps_abs=cfg.eps_abs,
                eps_rel=cfg.eps_rel,
                max_iter=cfg.max_iter,
            )
        else:
            self._solver.update(q=linear, l=lower, u=upper)
        result = self._solver.solve(raise_error=False)
        status = str(result.info.status)
        valid_solution = (
            result.x is not None
            and np.all(np.isfinite(result.x))
            and status in {
                "solved",
                "solved inaccurate",
                "maximum iterations reached",
            }
        )
        if valid_solution:
            acceleration_command = clamp(
                previous_command_value + float(result.x[0]),
                -max_braking,
                max_acceleration,
            )
        else:
            acceleration_command = previous_command_value
            status = f"fallback:{status}"
            self._solver = None
        self._previous_acceleration_command = acceleration_command
        result_value = LongitudinalMpcResult(
            acceleration_command_mps2=acceleration_command,
            status=status,
            solve_time_ms=float(result.info.run_time * 1000.0),
            iterations=int(result.info.iter),
        )
        return result_value


def acceleration_capability_mps2(speed_mps: float) -> float:
    speed_kmh = max(0.0, speed_mps * 3.6)
    breakpoints = (0.0, 50.0, 100.0, 150.0, 220.0)
    values = (5.0, 4.0, 3.2, 2.8, 2.2)
    return float(np.interp(speed_kmh, breakpoints, values))


def braking_capability_mps2(speed_mps: float) -> float:
    speed_kmh = max(0.0, speed_mps * 3.6)
    breakpoints = (0.0, 60.0, 100.0, 160.0, 220.0, 280.0)
    values = (11.0, 12.5, 14.0, 15.5, 17.0, 17.5)
    return float(np.interp(speed_kmh, breakpoints, values))


def brake_command_for_deceleration(
    deceleration_mps2: float,
    speed_mps: float,
    config: LongitudinalPedalConfig,
) -> float:
    deceleration = max(0.0, float(deceleration_mps2))
    if deceleration <= 0.0:
        return 0.0
    speed_kmh = max(0.0, speed_mps * 3.6)
    speed_points = config.braking_response_speed_kmh
    pedal_points = config.braking_response_pedal
    response = config.braking_response_decel_mps2
    if (
        len(speed_points) != len(response)
        or len(pedal_points) < 2
        or any(len(row) != len(pedal_points) for row in response)
    ):
        return clamp(
            deceleration
            / max(braking_capability_mps2(speed_mps), 1e-3),
            0.0,
            1.0,
        )
    deceleration_by_pedal = [
        float(
            np.interp(
                speed_kmh,
                speed_points,
                [row[index] for row in response],
            )
        )
        for index in range(len(pedal_points))
    ]
    return clamp(
        float(
            np.interp(
                deceleration,
                deceleration_by_pedal,
                pedal_points,
            )
        ),
        0.0,
        1.0,
    )


def map_acceleration_to_pedals(
    acceleration_mps2: float,
    speed_mps: float,
    *,
    drag_acceleration_mps2: float = 0.0,
) -> tuple[float, float]:
    propulsion_acceleration = (
        acceleration_mps2 + max(0.0, drag_acceleration_mps2)
    )
    if propulsion_acceleration >= 0.0:
        throttle = propulsion_acceleration / max(
            acceleration_capability_mps2(speed_mps),
            1e-3,
        )
        return clamp(throttle, 0.0, 1.0), 0.0
    brake = -propulsion_acceleration / max(
        braking_capability_mps2(speed_mps),
        1e-3,
    )
    return 0.0, clamp(brake, 0.0, 1.0)


def _rate_limit(
    current: float,
    target: float,
    max_delta: float,
) -> float:
    return current + clamp(
        target - current,
        -max_delta,
        max_delta,
    )
