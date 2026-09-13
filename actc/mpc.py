from __future__ import annotations

import contextlib
import io
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import casadi as ca
import numpy as np
import osqp
from scipy import sparse

from .lap import LapControlInfo
from .preview import MpcPreviewPublisher
from .shared_memory import VehicleState
from .track import TrackPath, clamp, physics_heading_to_world, wrap_pi


def _interpolate_map(
    value: float,
    breakpoints: tuple[float, ...],
    values: tuple[float, ...],
) -> float:
    if not breakpoints or len(breakpoints) != len(values):
        return values[0] if values else 0.0
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


def _limit_discrete_spectral_radius(
    matrix: np.ndarray,
    limit: float,
) -> np.ndarray:
    spectral_radius = float(
        np.max(np.abs(np.linalg.eigvals(matrix)))
    )
    if spectral_radius <= limit:
        return matrix
    return matrix * (limit / spectral_radius)


@dataclass(frozen=True)
class MpcVehicleParameters:
    wheelbase_m: float = 2.85
    front_axle_to_cg_m: float = 1.20
    mass_kg: float = 1500.0
    yaw_inertia_kgm2: float = 2250.0
    front_cornering_stiffness_n_rad: float = 80000.0
    rear_cornering_stiffness_n_rad: float = 90000.0
    max_steer_rad: float = 0.52
    max_accel_mps2: float = 3.0
    max_brake_mps2: float = 8.34
    tire_friction_coefficient: float = 1.5
    cg_height_m: float = 0.55
    steering_scale_map_kmh: tuple[float, ...] = (
        0.0,
        80.0,
        120.0,
        180.0,
        240.0,
    )
    steering_scale_map: tuple[float, ...] = (
        0.50,
        0.50,
        0.43,
        0.34,
        0.34,
    )
    steer_lateral_gain_map_kmh: tuple[float, ...] = (
        0.0,
        70.0,
        110.0,
        150.0,
        200.0,
    )
    steer_lateral_gain_map: tuple[float, ...] = (
        49.5,
        49.5,
        44.2,
        25.8,
        25.8,
    )
    steer_yaw_gain_map_kmh: tuple[float, ...] = (
        0.0,
        70.0,
        110.0,
        150.0,
        200.0,
    )
    steer_yaw_gain_map: tuple[float, ...] = (
        30.8,
        30.8,
        35.1,
        46.0,
        46.0,
    )
    yaw_response_load_schedule_g: tuple[float, ...] = (
        0.0,
        0.2,
        0.4,
        0.6,
        0.8,
        1.2,
    )
    yaw_response_load_scale: tuple[float, ...] = (
        1.0,
        0.90,
        0.75,
        0.60,
        0.50,
        0.45,
    )
    direct_model_map_kmh: tuple[float, ...] = (
        0.0,
        45.0,
        70.0,
        100.0,
        130.0,
        180.0,
        240.0,
    )
    lateral_velocity_damping_map: tuple[float, ...] = (
        202.8,
        202.8,
        220.7,
        257.4,
        198.9,
        174.2,
        174.2,
    )
    lateral_yaw_coupling_map: tuple[float, ...] = (
        115.5,
        115.5,
        160.3,
        242.1,
        303.9,
        415.1,
        415.1,
    )
    yaw_velocity_coupling_map: tuple[float, ...] = (
        11.9,
        11.9,
        11.1,
        -40.6,
        -9.5,
        -19.9,
        -19.9,
    )
    yaw_rate_damping_map: tuple[float, ...] = (
        -142.9,
        -142.9,
        -183.6,
        -211.5,
        -211.8,
        -204.3,
        -204.3,
    )
    yaw_inertia_map_kmh: tuple[float, ...] = (
        0.0,
        80.0,
        120.0,
        180.0,
        240.0,
    )
    yaw_inertia_map_kgm2: tuple[float, ...] = (
        2800.0,
        2800.0,
        2100.0,
        1500.0,
        1500.0,
    )

    @property
    def rear_axle_to_cg_m(self) -> float:
        return self.wheelbase_m - self.front_axle_to_cg_m


@dataclass(frozen=True)
class MpcReferenceSequence:
    states: np.ndarray
    local_points: np.ndarray
    curvatures: np.ndarray
    indices: np.ndarray
    distances_m: np.ndarray


@dataclass
class MpcStats:
    solve_ms: float = 0.0
    assembly_ms: float = 0.0
    solver_ms: float = 0.0
    iterations: int = 0
    status: str = "unknown"


@dataclass
class MpcConfig:
    horizon: int = 20
    dt: float = 0.02
    q_lateral: float = 38.0
    q_heading: float = 32.0
    q_speed: float = 2.0
    q_lateral_velocity: float = 1.0
    q_yaw_rate: float = 7.0
    r_steer: float = 8.0
    r_accel: float = 0.4
    rd_steer: float = 320.0
    rd_accel: float = 8.0
    steer_limit: float = 0.52
    steer_rate_limit_rad_s: float = 2.0
    accel_limit_mps2: float = 3.0
    brake_limit_mps2: float = 8.34
    accel_rate_limit_mps3: float = 8.0
    solve_time_budget_ms: float = 18.0
    terminal_weight_scale: float = 4.0
    reference_preview_s: float = 0.08
    steering_delay_s: float = 0.02
    steering_lead_s: float = 0.02
    steering_filter_tau_s: float = 0.06
    steering_actuator_tau_s: float = 0.14
    steering_state_weight: float = 0.0
    use_steering_actuator_model: bool = True
    steering_rate_gain: float = 8.0
    steering_jerk_limit_rad_s3: float = 8.0
    steering_deadband: float = 0.003
    steering_hysteresis: float = 0.002
    speed_bucket_mps: float = 5.0
    max_discrete_spectral_radius: float = 0.98
    osqp_eps_abs: float = 1e-3
    osqp_eps_rel: float = 1e-3
    osqp_max_iter: int = 400
    osqp_warm_start_dual: bool = True
    weight_map_kmh: tuple[float, ...] = (
        0.0,
        40.0,
        80.0,
        140.0,
        220.0,
    )
    q_lateral_weight_map: tuple[float, ...] = (
        1.0,
        1.0,
        1.0,
        0.70,
        0.65,
    )
    q_heading_weight_map: tuple[float, ...] = (
        1.0,
        1.0,
        1.0,
        0.95,
        0.90,
    )
    q_yaw_rate_weight_map: tuple[float, ...] = (
        1.0,
        1.0,
        1.05,
        1.15,
        1.20,
    )
    r_steer_weight_map: tuple[float, ...] = (
        1.05,
        1.15,
        1.35,
        2.25,
        2.60,
    )
    rd_steer_weight_map: tuple[float, ...] = (
        1.05,
        1.20,
        1.50,
        2.50,
        3.00,
    )
    nlp_solver: str = "ipopt"
    parameters: MpcVehicleParameters = field(
        default_factory=MpcVehicleParameters
    )


def _lookup_vehicle_map(
    value: float,
    breakpoints: tuple[float, ...],
    values: tuple[float, ...],
) -> float:
    return _interpolate_map(value, breakpoints, values)


def _steady_state_reference(
    speed_mps: float,
    curvature: float,
    parameters: MpcVehicleParameters,
) -> tuple[float, float]:
    speed_mps = max(3.0, speed_mps)
    yaw_rate_ref = speed_mps * curvature
    del parameters
    return 0.0, float(yaw_rate_ref)


def build_reference_sequence(
    path: TrackPath,
    index: int,
    state: VehicleState,
    horizon: int,
    dt: float,
    parameters: MpcVehicleParameters | None = None,
    preview_time_s: float = 0.08,
    state_dimension: int = 5,
) -> MpcReferenceSequence:
    states = np.zeros((horizon, state_dimension), dtype=float)
    local_points = np.zeros((horizon, 2), dtype=float)
    curvatures = np.zeros(horizon, dtype=float)
    indices = np.zeros(horizon, dtype=np.int32)
    distances = np.zeros(horizon, dtype=float)
    predicted_speed_mps = max(3.0, min(60.0, state.speed_ms))
    car_heading = physics_heading_to_world(state.heading_rad)
    preview_distance_m = max(0.0, state.speed_ms * preview_time_s)
    distance_m = preview_distance_m

    for stage in range(horizon):
        current_index = path.lookahead_index(index, distance_m)
        speed_mps = max(
            2.0,
            path.speed_limit_kmh[current_index] / 3.6,
        )
        states[stage, 2] = speed_mps
        if parameters is not None:
            lateral_velocity_ref, yaw_rate_ref = _steady_state_reference(
                speed_mps,
                float(path.curvature[current_index]),
                parameters,
            )
            states[stage, 3] = lateral_velocity_ref
            states[stage, 4] = yaw_rate_ref
        point_x, point_z = path.points[current_index]
        dx = point_x - state.position[0]
        dz = point_z - state.position[2]
        local_points[stage, 0] = (
            math.cos(car_heading) * dx + math.sin(car_heading) * dz
        )
        local_points[stage, 1] = (
            -math.sin(car_heading) * dx + math.cos(car_heading) * dz
        )
        curvatures[stage] = path.curvature[current_index]
        indices[stage] = current_index
        distances[stage] = distance_m
        predicted_speed_mps = 0.5 * predicted_speed_mps + 0.5 * speed_mps
        distance_m += max(1.0, predicted_speed_mps) * dt

    return MpcReferenceSequence(
        states=states,
        local_points=local_points,
        curvatures=curvatures,
        indices=indices,
        distances_m=distances,
    )


def vehicle_state_to_frenet(
    state: VehicleState,
    path: TrackPath,
    previous_index: int | None,
) -> tuple[int, np.ndarray]:
    (
        index,
        _fraction,
        projected_x,
        projected_z,
        reference_heading,
        _curvature,
    ) = path.project(
        state.position[0],
        state.position[2],
        previous_index,
    )
    car_heading = physics_heading_to_world(state.heading_rad)
    tangent_x = math.cos(reference_heading)
    tangent_z = math.sin(reference_heading)
    lateral_error = (
        tangent_x * (state.position[2] - projected_z)
        - tangent_z * (state.position[0] - projected_x)
    )
    heading_error = wrap_pi(car_heading - reference_heading)
    vx = float(state.local_velocity_m_s[2])
    vy = -float(state.local_velocity_m_s[0])
    yaw_rate = -float(state.yaw_rate_rad_s)
    return index, np.array(
        [lateral_error, heading_error, vx, vy, yaw_rate],
        dtype=float,
    )


@dataclass(frozen=True)
class _LinearPredictionCache:
    speeds: np.ndarray
    stages: tuple[tuple[np.ndarray, np.ndarray], ...]
    sx: np.ndarray
    su: np.ndarray
    su_t_q: np.ndarray
    control_p_matrix: np.ndarray
    p_matrix: np.ndarray
    upper_rows: np.ndarray
    upper_cols: np.ndarray


class LinearMpcController:
    """Linear dynamic bicycle MPC solved as a condensed QP with OSQP."""

    def __init__(self, config: MpcConfig | None = None) -> None:
        self.config = config or MpcConfig()
        self.previous_index: int | None = None
        self.previous_control = np.zeros(2, dtype=float)
        self.previous_applied_steer = 0.0
        self.previous_applied_steer_rate = 0.0
        self.previous_filtered_steer = 0.0
        self.warm_start: np.ndarray | None = None
        self.last_stats = MpcStats()
        self._solver: osqp.OSQP | None = None
        self._solver_bucket: tuple[int, ...] | None = None
        self._q_bar: sparse.csc_matrix | None = None
        self._r_bar: sparse.csc_matrix | None = None
        self._rd_bar: sparse.csc_matrix | None = None
        self._increment_matrix: sparse.csc_matrix | None = None
        self._r_q_matrix: sparse.csc_matrix | None = None
        self._a_cons: sparse.csc_matrix | None = None
        self._lower_template: np.ndarray | None = None
        self._upper_template: np.ndarray | None = None
        self._prediction_cache: dict[
            tuple[tuple[int, ...], int],
            _LinearPredictionCache,
        ] = {}
        self._warm_start_dual: np.ndarray | None = None
        self._assembly_ms = 0.0
        self._solver_ms = 0.0
        self._last_predicted_states: np.ndarray | None = None
        self._preview_publisher = MpcPreviewPublisher(
            Path(__file__).resolve().parents[1]
            / "logs"
            / "mpc_preview.json"
        )

    def calibrate(self, state: VehicleState, path: TrackPath) -> None:
        self.previous_index, _ = vehicle_state_to_frenet(
            state,
            path,
            None,
        )
        self.previous_control[:] = 0.0
        self.previous_applied_steer = 0.0
        self.previous_applied_steer_rate = 0.0
        self.previous_filtered_steer = 0.0
        self.warm_start = None
        self._warm_start_dual = None

    def invalidate_plan_cache(self) -> None:
        self._prediction_cache.clear()
        self._q_bar = None
        self._r_bar = None
        self._rd_bar = None
        self._increment_matrix = None
        self._r_q_matrix = None
        self._solver_bucket = None

    def _state_size(self) -> int:
        return 6 if self.config.use_steering_actuator_model else 5

    def step(
        self,
        state: VehicleState,
        path: TrackPath,
        dt: float,
    ) -> LapControlInfo:
        del dt
        started = time.perf_counter()
        index, x0 = vehicle_state_to_frenet(
            state,
            path,
            self.previous_index,
        )
        if self.config.use_steering_actuator_model:
            x0 = np.concatenate(
                (x0, np.asarray([float(getattr(state, "steer", 0.0))]))
            )
        self.previous_index = index
        reference = build_reference_sequence(
            path,
            index,
            state,
            self.config.horizon,
            self.config.dt,
            self.config.parameters,
            self.config.reference_preview_s,
            state_dimension=self._state_size(),
        )
        control, status, iterations = self._solve_condensed_qp(
            x0,
            reference,
            abs(float(getattr(state, "acceleration_g", (0.0,))[0])),
        )
        if control is None:
            self.last_stats = MpcStats(
                solve_ms=(time.perf_counter() - started) * 1000.0,
                assembly_ms=self._assembly_ms,
                solver_ms=self._solver_ms,
                iterations=iterations,
                status=status,
            )
            steering = float(self.previous_control[0])
        else:
            steering = float(control[0])
            self.previous_control = control[:2].copy()
            self.last_stats = MpcStats(
                solve_ms=(time.perf_counter() - started) * 1000.0,
                assembly_ms=self._assembly_ms,
                solver_ms=self._solver_ms,
                iterations=iterations,
                status=status,
            )

        compensated_steering = self._compensate_steering(
            steering,
            state,
            self.config.dt,
            1.0,
        )
        self._publish_preview(state, path, reference)
        return LapControlInfo(
            path_index=index,
            lookahead_index=int(reference.indices[-1]),
            target_speed_kmh=float(reference.states[0, 2] * 3.6),
            lateral_error_m=float(x0[0]),
            heading_error_rad=float(x0[1]),
            lookahead_m=float(reference.distances_m[-1]),
            steer=clamp(
                compensated_steering,
                -1.0,
                1.0,
            ),
            raw_steer=clamp(
                steering,
                -1.0,
                1.0,
            ),
        )

    def _compensate_steering(
        self,
        requested_steer: float,
        state: VehicleState,
        dt: float,
        steer_scale: float,
    ) -> float:
        cfg = self.config
        params = cfg.parameters
        feedback_state = getattr(state, "steer", 0.0)
        if abs(feedback_state) <= 1.5:
            actual_steer = feedback_state * steer_scale
        else:
            actual_steer = feedback_state

        safe_dt = max(dt, 1e-3)
        filter_alpha = safe_dt / (
            max(cfg.steering_filter_tau_s, 1e-3) + safe_dt
        )
        filtered_steer = self.previous_filtered_steer + filter_alpha * (
            requested_steer - self.previous_filtered_steer
        )
        raw_rate = (
            filtered_steer - self.previous_filtered_steer
        ) / safe_dt
        self.previous_filtered_steer = filtered_steer
        lead = cfg.steering_lead_s * clamp(
            raw_rate,
            -cfg.steer_rate_limit_rad_s,
            cfg.steer_rate_limit_rad_s,
        )
        lead = clamp(lead, -0.03, 0.03)
        lag_correction = 0.15 * (requested_steer - actual_steer)
        compensated = filtered_steer + lead + lag_correction
        compensated = clamp(
            compensated,
            -steer_scale,
            steer_scale,
        )
        desired_rate = clamp(
            cfg.steering_rate_gain
            * (compensated - self.previous_applied_steer),
            -cfg.steer_rate_limit_rad_s,
            cfg.steer_rate_limit_rad_s,
        )
        max_rate_change = cfg.steering_jerk_limit_rad_s3 * safe_dt
        self.previous_applied_steer_rate += clamp(
            desired_rate - self.previous_applied_steer_rate,
            -max_rate_change,
            max_rate_change,
        )
        next_steer = (
            self.previous_applied_steer
            + self.previous_applied_steer_rate * safe_dt
        )
        if (
            (compensated - self.previous_applied_steer)
            * (compensated - next_steer)
            <= 0.0
        ):
            next_steer = compensated
            self.previous_applied_steer_rate = 0.0
        compensated = next_steer
        command_change = compensated - self.previous_applied_steer
        if abs(command_change) < cfg.steering_deadband:
            compensated = self.previous_applied_steer
        elif (
            abs(command_change) < cfg.steering_hysteresis
            and compensated * self.previous_applied_steer > 0.0
        ):
            compensated = self.previous_applied_steer
        self.previous_applied_steer = compensated
        return compensated

    def _steering_scale_from_speed(self, speed_kmh: float) -> float:
        params = self.config.parameters
        return max(
            1e-3,
            _interpolate_map(
                speed_kmh,
                params.steering_scale_map_kmh,
                params.steering_scale_map,
            ),
        )

    def _publish_preview(
        self,
        state: VehicleState,
        path: TrackPath,
        reference: MpcReferenceSequence,
    ) -> None:
        if self._last_predicted_states is None:
            return
        reference_points: list[tuple[float, float]] = []
        predicted_points: list[tuple[float, float]] = []
        for stage, reference_index in enumerate(reference.indices):
            index = int(reference_index) % len(path.points)
            point_x, point_z = path.points[index]
            heading = path.heading_rad[index]
            normal_x = -math.sin(heading)
            normal_z = math.cos(heading)
            reference_points.append((point_x, point_z))
            lateral_error = float(
                self._last_predicted_states[stage, 0]
            )
            predicted_points.append(
                (
                    point_x + lateral_error * normal_x,
                    point_z + lateral_error * normal_z,
                )
            )
        self._preview_publisher.publish(
            current=(state.position[0], state.position[2]),
            reference=reference_points,
            predicted=predicted_points,
            indices=self._continuous_indices(
                reference.indices,
                len(path.points),
            ),
            lateral_errors=[
                float(self._last_predicted_states[stage, 0])
                for stage in range(len(reference.indices))
            ],
            heading_errors=[
                float(self._last_predicted_states[stage, 1])
                for stage in range(len(reference.indices))
            ],
        )

    @staticmethod
    def _continuous_indices(
        indices: np.ndarray,
        point_count: int,
    ) -> list[int]:
        output: list[int] = []
        offset = 0
        previous: int | None = None
        for value in indices:
            index = int(value)
            if previous is not None:
                if index < previous - point_count // 2:
                    offset += point_count
                elif index > previous + point_count // 2:
                    offset -= point_count
            output.append(index + offset)
            previous = index
        return output

    def _solve_condensed_qp(
        self,
        x0: np.ndarray,
        reference: MpcReferenceSequence,
        lateral_accel_g: float = 0.0,
    ) -> tuple[np.ndarray | None, str, int]:
        cfg = self.config
        nu = 2
        nx = self._state_size()
        horizon = cfg.horizon
        assembly_started = time.perf_counter()
        bucket = tuple(
            int(round(float(speed) / cfg.speed_bucket_mps))
            for speed in reference.states[:, 2]
        )
        load_bucket = int(
            round(clamp(abs(lateral_accel_g), 0.0, 1.2) / 0.2)
        )
        cache_key = (bucket, load_bucket)
        prediction = self._prediction_cache.get(cache_key)
        if prediction is None:
            prediction = self._build_prediction_cache(
                bucket,
                load_bucket,
            )
            self._prediction_cache[cache_key] = prediction

        q_bar, r_bar, rd_bar, _ = (
            self._ensure_static_matrices()
        )
        del q_bar, r_bar
        reference_vector = reference.states.reshape(-1)
        affine = self._propagate_affine(
            prediction,
            reference.curvatures,
        )
        linear_term = prediction.sx @ x0 + affine[:, 0]
        previous_vector = np.tile(
            self.previous_control,
            horizon,
        )
        variable_count = 2 * horizon * nu
        q_vector = np.zeros(variable_count, dtype=float)
        q_vector[horizon * nu :] = (
            prediction.su_t_q @ (linear_term - reference_vector)
        )
        p_matrix = prediction.p_matrix
        p_upper = sparse.csc_matrix(
            (
                p_matrix[prediction.upper_rows, prediction.upper_cols],
                (prediction.upper_rows, prediction.upper_cols),
            ),
            shape=p_matrix.shape,
        )

        assert self._a_cons is not None
        assert self._lower_template is not None
        assert self._upper_template is not None
        lower_array = self._lower_template.copy()
        upper_array = self._upper_template.copy()
        lower_array[: horizon * nu] = previous_vector
        upper_array[: horizon * nu] = previous_vector
        self._assembly_ms = (
            time.perf_counter() - assembly_started
        ) * 1000.0
        solver_started = time.perf_counter()
        with contextlib.redirect_stdout(io.StringIO()):
            if self._solver is None:
                self._solver = osqp.OSQP()
                self._solver.setup(
                    P=p_upper,
                    q=q_vector,
                    A=self._a_cons,
                    l=lower_array,
                    u=upper_array,
                    verbose=False,
                    polishing=False,
                    adaptive_rho=True,
                    eps_abs=cfg.osqp_eps_abs,
                    eps_rel=cfg.osqp_eps_rel,
                    max_iter=cfg.osqp_max_iter,
                    warm_starting=True,
                )
                self._solver_bucket = bucket
            else:
                update_values: dict[str, np.ndarray] = {
                    "q": q_vector,
                    "l": lower_array,
                    "u": upper_array,
                }
                if bucket != self._solver_bucket:
                    update_values["Px"] = p_upper.data
                    self._solver_bucket = bucket
                self._solver.update(
                    **update_values,
                )
                if self.warm_start is not None:
                    self._solver.warm_start(
                        x=self.warm_start,
                        y=(
                            self._warm_start_dual
                            if cfg.osqp_warm_start_dual
                            else None
                        ),
                    )
            result = self._solver.solve()
        self._solver_ms = (
            time.perf_counter() - solver_started
        ) * 1000.0
        iterations = (
            int(result.info.iter)
            if hasattr(result.info, "iter")
            else 0
        )
        if result.x is None or not np.all(np.isfinite(result.x)):
            return None, str(result.info.status), iterations
        self.warm_start = result.x.copy()
        if getattr(result, "y", None) is not None:
            self._warm_start_dual = result.y.copy()
        absolute_controls = result.x[horizon * nu :]
        self._last_predicted_states = (
            linear_term[:, None]
            + prediction.su @ absolute_controls.reshape(-1, 1)
        ).T.reshape(horizon, nx)
        increment = result.x[:nu]
        return (
            self.previous_control + increment,
            str(result.info.status),
            iterations,
        )

    def _build_prediction_cache(
        self,
        bucket: tuple[int, ...],
        load_bucket: int,
    ) -> _LinearPredictionCache:
        cfg = self.config
        nx, nu, horizon = self._state_size(), 2, cfg.horizon
        speeds = np.asarray(bucket, dtype=float) * cfg.speed_bucket_mps
        stages = tuple(
            self._stage_model_from_speed(
                float(speed),
                lateral_accel_g=load_bucket * 0.2,
            )
            for speed in speeds
        )
        sx = np.zeros((horizon * nx, nx), dtype=float)
        su = np.zeros((horizon * nx, horizon * nu), dtype=float)
        phi = np.eye(nx)
        control_sensitivity = np.zeros(
            (nx, horizon * nu),
            dtype=float,
        )
        for stage, (a_matrix, b_matrix) in enumerate(stages):
            phi = a_matrix @ phi
            next_sensitivity = a_matrix @ control_sensitivity
            next_sensitivity[
                :,
                stage * nu : (stage + 1) * nu,
            ] = b_matrix
            control_sensitivity = next_sensitivity
            row = slice(stage * nx, (stage + 1) * nx)
            sx[row, :] = phi
            su[row, :] = control_sensitivity

        q_bar, r_bar, rd_bar = self._cost_matrices_for_speeds(
            speeds
        )
        su_t_q = np.asarray(su.T @ q_bar)
        control_p_matrix = np.asarray(
            su_t_q @ su + r_bar.toarray()
        )
        variable_count = 2 * horizon * nu
        p_matrix = np.zeros(
            (variable_count, variable_count),
            dtype=float,
        )
        p_matrix[: horizon * nu, : horizon * nu] = (
            rd_bar.toarray()
        )
        p_matrix[
            horizon * nu :,
            horizon * nu :,
        ] = control_p_matrix
        upper_rows, upper_cols = np.triu_indices(variable_count)
        return _LinearPredictionCache(
            speeds=speeds,
            stages=stages,
            sx=sx,
            su=su,
            su_t_q=su_t_q,
            control_p_matrix=control_p_matrix,
            p_matrix=p_matrix,
            upper_rows=upper_rows,
            upper_cols=upper_cols,
        )

    def _cost_matrices_for_speeds(
        self,
        speeds_mps: np.ndarray,
    ) -> tuple[
        sparse.csc_matrix,
        sparse.csc_matrix,
        sparse.csc_matrix,
    ]:
        cfg = self.config
        horizon = cfg.horizon
        q_diagonal: list[float] = []
        r_diagonal: list[float] = []
        rd_diagonal: list[float] = []
        for stage, speed_mps in enumerate(speeds_mps):
            speed_kmh = float(speed_mps) * 3.6
            q_lateral = cfg.q_lateral * _lookup_vehicle_map(
                speed_kmh,
                cfg.weight_map_kmh,
                cfg.q_lateral_weight_map,
            )
            q_heading = cfg.q_heading * _lookup_vehicle_map(
                speed_kmh,
                cfg.weight_map_kmh,
                cfg.q_heading_weight_map,
            )
            q_yaw_rate = cfg.q_yaw_rate * _lookup_vehicle_map(
                speed_kmh,
                cfg.weight_map_kmh,
                cfg.q_yaw_rate_weight_map,
            )
            terminal_scale = (
                cfg.terminal_weight_scale
                if stage == horizon - 1
                else 1.0
            )
            q_diagonal.extend(
                [
                    q_lateral * terminal_scale,
                    q_heading * terminal_scale,
                    cfg.q_speed * terminal_scale,
                    cfg.q_lateral_velocity * terminal_scale,
                    q_yaw_rate * terminal_scale,
                ]
            )
            if cfg.use_steering_actuator_model:
                q_diagonal.append(
                    cfg.steering_state_weight * terminal_scale
                )
            r_diagonal.extend(
                [
                    cfg.r_steer
                    * _lookup_vehicle_map(
                        speed_kmh,
                        cfg.weight_map_kmh,
                        cfg.r_steer_weight_map,
                    ),
                    cfg.r_accel,
                ]
            )
            rd_diagonal.extend(
                [
                    cfg.rd_steer
                    * _lookup_vehicle_map(
                        speed_kmh,
                        cfg.weight_map_kmh,
                        cfg.rd_steer_weight_map,
                    ),
                    cfg.rd_accel,
                ]
            )
        return (
            sparse.diags(q_diagonal, format="csc"),
            sparse.diags(r_diagonal, format="csc"),
            sparse.diags(rd_diagonal, format="csc"),
        )

    def _propagate_affine(
        self,
        prediction: _LinearPredictionCache,
        curvatures: np.ndarray,
    ) -> np.ndarray:
        cfg = self.config
        nx, horizon = self._state_size(), cfg.horizon
        affine = np.zeros((horizon * nx, 1), dtype=float)
        propagated = np.zeros(nx, dtype=float)
        for stage, (a_matrix, _b_matrix) in enumerate(
            prediction.stages
        ):
            disturbance = np.zeros(nx, dtype=float)
            disturbance[1] = (
                -prediction.speeds[stage]
                * float(curvatures[stage])
                * cfg.dt
            )
            propagated = a_matrix @ propagated + disturbance
            row = slice(stage * nx, (stage + 1) * nx)
            affine[row, 0] = propagated
        return affine

    def _ensure_static_matrices(
        self,
    ) -> tuple[
        sparse.csc_matrix,
        sparse.csc_matrix,
        sparse.csc_matrix,
        np.ndarray,
    ]:
        cfg = self.config
        nx, nu, horizon = self._state_size(), 2, cfg.horizon
        if self._q_bar is None:
            state_weights = np.array(
                [
                    cfg.q_lateral,
                    cfg.q_heading,
                    cfg.q_speed,
                    cfg.q_lateral_velocity,
                    cfg.q_yaw_rate,
                ],
                dtype=float,
            )
            if cfg.use_steering_actuator_model:
                state_weights = np.concatenate(
                    (state_weights, [cfg.steering_state_weight])
                )
            q_diag = np.tile(state_weights, horizon)
            q_diag[-nx:] *= cfg.terminal_weight_scale
            self._q_bar = sparse.diags(q_diag, format="csc")
            self._r_bar = sparse.kron(
                sparse.eye(horizon),
                sparse.diags([cfg.r_steer, cfg.r_accel]),
                format="csc",
            )
            self._rd_bar = sparse.kron(
                sparse.eye(horizon),
                sparse.diags([cfg.rd_steer, cfg.rd_accel]),
                format="csc",
            )
            increment_rows: list[sparse.csc_matrix] = []
            for stage in range(horizon):
                row = sparse.lil_matrix(
                    (nu, horizon * nu),
                    dtype=float,
                )
                row[
                    :,
                    : (stage + 1) * nu,
                ] = sparse.kron(
                    np.ones((1, stage + 1)),
                    sparse.eye(nu),
                    format="csc",
                )
                increment_rows.append(row.tocsc())
            self._increment_matrix = sparse.vstack(
                increment_rows,
                format="csc",
            )
            self._r_q_matrix = (
                self._increment_matrix.T @ self._r_bar
            ).tocsc()

            identity = sparse.eye(horizon * nu, format="csc")
            zero = sparse.csc_matrix(
                (horizon * nu, horizon * nu),
                dtype=float,
            )
            absolute_lower = list(
                [-cfg.steer_limit, -cfg.brake_limit_mps2] * horizon
            )
            absolute_upper = list(
                [cfg.steer_limit, cfg.accel_limit_mps2] * horizon
            )
            increment_lower = (
                [
                    -cfg.steer_rate_limit_rad_s,
                    -cfg.accel_rate_limit_mps3,
                ]
                * horizon
            )
            increment_upper = (
                [
                    cfg.steer_rate_limit_rad_s,
                    cfg.accel_rate_limit_mps3,
                ]
                * horizon
            )
            self._a_cons = sparse.vstack(
                [
                    sparse.hstack(
                        [-self._increment_matrix, identity],
                        format="csc",
                    ),
                    sparse.hstack(
                        [zero, identity],
                        format="csc",
                    ),
                    sparse.hstack(
                        [identity, zero],
                        format="csc",
                    ),
                ],
                format="csc",
            )
            self._lower_template = np.asarray(
                [0.0] * (horizon * nu)
                + absolute_lower
                + increment_lower
            )
            self._upper_template = np.asarray(
                [0.0] * (horizon * nu)
                + absolute_upper
                + increment_upper
            )

        assert self._q_bar is not None
        assert self._r_bar is not None
        assert self._rd_bar is not None
        assert self._increment_matrix is not None
        assert self._r_q_matrix is not None
        return (
            self._q_bar,
            self._r_bar,
            self._rd_bar,
            np.zeros(horizon * nu),
        )

    def _stage_model_from_speed(
        self,
        speed_mps: float,
        lateral_accel_g: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.config
        params = cfg.parameters
        dt = cfg.dt
        vx = max(3.0, speed_mps)
        m = params.mass_kg
        iz = max(
            1.0,
            _interpolate_map(
                vx * 3.6,
                params.yaw_inertia_map_kmh,
                params.yaw_inertia_map_kgm2,
            ),
        )
        a = params.front_axle_to_cg_m
        b = params.rear_axle_to_cg_m
        cf = params.front_cornering_stiffness_n_rad
        cr = params.rear_cornering_stiffness_n_rad

        a_cont = np.zeros((5, 5), dtype=float)
        a_cont[0, 1] = vx
        a_cont[0, 3] = 1.0
        a_cont[1, 4] = 1.0
        a_cont[3, 3] = -(cf + cr) / (m * vx)
        a_cont[3, 4] = (b * cr - a * cf) / (m * vx) - vx
        a_cont[4, 3] = (b * cr - a * cf) / (iz * vx)
        a_cont[4, 4] = -(a * a * cf + b * b * cr) / (iz * vx)
        speed_kmh = vx * 3.6
        if params.direct_model_map_kmh:
            a_cont[3, 3] = -_lookup_vehicle_map(
                speed_kmh,
                params.direct_model_map_kmh,
                params.lateral_velocity_damping_map,
            ) / vx
            a_cont[3, 4] = (
                _lookup_vehicle_map(
                    speed_kmh,
                    params.direct_model_map_kmh,
                    params.lateral_yaw_coupling_map,
                )
                / vx
                - vx
            )
            a_cont[4, 3] = _lookup_vehicle_map(
                speed_kmh,
                params.direct_model_map_kmh,
                params.yaw_velocity_coupling_map,
            ) / vx
            a_cont[4, 4] = _lookup_vehicle_map(
                speed_kmh,
                params.direct_model_map_kmh,
                params.yaw_rate_damping_map,
            ) / vx
        b_cont = np.zeros((5, 2), dtype=float)
        b_cont[2, 1] = 1.0
        b_cont[3, 0] = _lookup_vehicle_map(
            speed_kmh,
            params.steer_lateral_gain_map_kmh,
            params.steer_lateral_gain_map,
        )
        b_cont[4, 0] = _lookup_vehicle_map(
            speed_kmh,
            params.steer_yaw_gain_map_kmh,
            params.steer_yaw_gain_map,
        )
        yaw_response_scale = _lookup_vehicle_map(
            abs(lateral_accel_g),
            params.yaw_response_load_schedule_g,
            params.yaw_response_load_scale,
        )
        a_cont[4, :] *= yaw_response_scale
        b_cont[4, 0] *= yaw_response_scale
        if cfg.use_steering_actuator_model:
            actuator_tau = max(
                1e-3,
                cfg.steering_actuator_tau_s,
            )
            augmented_a = np.zeros((6, 6), dtype=float)
            augmented_a[:5, :5] = a_cont
            augmented_a[3, 5] = b_cont[3, 0]
            augmented_a[4, 5] = b_cont[4, 0]
            augmented_a[5, 5] = -1.0 / actuator_tau
            augmented_b = np.zeros((6, 2), dtype=float)
            augmented_b[:5, :] = b_cont
            augmented_b[3, 0] = 0.0
            augmented_b[4, 0] = 0.0
            augmented_b[5, 0] = 1.0 / actuator_tau
            a_cont = augmented_a
            b_cont = augmented_b
        a_matrix = np.eye(a_cont.shape[0]) + a_cont * dt
        a_matrix = _limit_discrete_spectral_radius(
            a_matrix,
            cfg.max_discrete_spectral_radius,
        )
        b_matrix = b_cont * dt
        return a_matrix, b_matrix


class NonlinearMpcController:
    """CasADi/IPOPT NMPC with a simple nonlinear tire and Frenet dynamics."""

    def __init__(
        self,
        config: MpcConfig | None = None,
        fallback: LinearMpcController | None = None,
    ) -> None:
        self.config = config or MpcConfig()
        self.fallback = fallback or LinearMpcController(self.config)
        self.previous_index: int | None = None
        self.previous_control = np.zeros(2, dtype=float)
        self.previous_applied_steer = 0.0
        self.previous_applied_steer_rate = 0.0
        self.previous_filtered_steer = 0.0
        self.previous_solution: np.ndarray | None = None
        self.last_stats = MpcStats()
        self._solver = None
        self._build_solver()

    def calibrate(self, state: VehicleState, path: TrackPath) -> None:
        self.previous_index, _ = vehicle_state_to_frenet(
            state,
            path,
            None,
        )
        self.previous_control[:] = 0.0
        self.previous_applied_steer = 0.0
        self.previous_applied_steer_rate = 0.0
        self.previous_filtered_steer = 0.0
        self.previous_solution = None
        self.fallback.calibrate(state, path)

    def step(
        self,
        state: VehicleState,
        path: TrackPath,
        dt: float,
    ) -> LapControlInfo:
        del dt
        started = time.perf_counter()
        index, x0 = vehicle_state_to_frenet(
            state,
            path,
            self.previous_index,
        )
        self.previous_index = index
        reference = build_reference_sequence(
            path,
            index,
            state,
            self.config.horizon,
            self.config.dt,
            self.config.parameters,
            self.config.reference_preview_s,
        )
        try:
            control, status = self._solve_nonlinear(x0, reference)
        except Exception:
            control, status = None, "solver-error"
        solve_ms = (time.perf_counter() - started) * 1000.0
        if (
            control is None
            or solve_ms > self.config.solve_time_budget_ms
        ):
            info = self.fallback.step(state, path, self.config.dt)
            self.last_stats = MpcStats(
                solve_ms=solve_ms,
                iterations=0,
                status=f"{status}-fallback",
            )
            return info

        self.previous_control = control.copy()
        compensated_steering = self._compensate_steering(
            float(control[0]),
            state,
            self.config.dt,
            self._steering_scale_from_speed(state.speed_kmh),
        )
        self.last_stats = MpcStats(
            solve_ms=solve_ms,
            iterations=0,
            status=status,
        )
        return LapControlInfo(
            path_index=index,
            lookahead_index=int(reference.indices[-1]),
            target_speed_kmh=float(reference.states[0, 2] * 3.6),
            lateral_error_m=float(x0[0]),
            heading_error_rad=float(x0[1]),
            lookahead_m=float(reference.distances_m[-1]),
            steer=clamp(
                compensated_steering
                / self._steering_scale_from_speed(state.speed_kmh),
                -1.0,
                1.0,
            ),
            raw_steer=clamp(
                float(control[0])
                / self._steering_scale_from_speed(state.speed_kmh),
                -1.0,
                1.0,
            ),
        )

    def _compensate_steering(
        self,
        requested_steer: float,
        state: VehicleState,
        dt: float,
        steer_scale: float,
    ) -> float:
        cfg = self.config
        params = cfg.parameters
        feedback_state = getattr(state, "steer", 0.0)
        if abs(feedback_state) <= 1.5:
            actual_steer = feedback_state * steer_scale
        else:
            actual_steer = feedback_state

        safe_dt = max(dt, 1e-3)
        filter_alpha = safe_dt / (
            max(cfg.steering_filter_tau_s, 1e-3) + safe_dt
        )
        filtered_steer = self.previous_filtered_steer + filter_alpha * (
            requested_steer - self.previous_filtered_steer
        )
        raw_rate = (
            filtered_steer - self.previous_filtered_steer
        ) / safe_dt
        self.previous_filtered_steer = filtered_steer
        lead = cfg.steering_lead_s * clamp(
            raw_rate,
            -cfg.steer_rate_limit_rad_s,
            cfg.steer_rate_limit_rad_s,
        )
        lead = clamp(lead, -0.03, 0.03)
        compensated = (
            filtered_steer
            + lead
            + 0.15 * (requested_steer - actual_steer)
        )
        compensated = clamp(
            compensated,
            -steer_scale,
            steer_scale,
        )
        desired_rate = clamp(
            cfg.steering_rate_gain
            * (compensated - self.previous_applied_steer),
            -cfg.steer_rate_limit_rad_s,
            cfg.steer_rate_limit_rad_s,
        )
        max_rate_change = cfg.steering_jerk_limit_rad_s3 * safe_dt
        self.previous_applied_steer_rate += clamp(
            desired_rate - self.previous_applied_steer_rate,
            -max_rate_change,
            max_rate_change,
        )
        next_steer = (
            self.previous_applied_steer
            + self.previous_applied_steer_rate * safe_dt
        )
        if (
            (compensated - self.previous_applied_steer)
            * (compensated - next_steer)
            <= 0.0
        ):
            next_steer = compensated
            self.previous_applied_steer_rate = 0.0
        compensated = next_steer
        command_change = compensated - self.previous_applied_steer
        if abs(command_change) < cfg.steering_deadband:
            compensated = self.previous_applied_steer
        elif (
            abs(command_change) < cfg.steering_hysteresis
            and compensated * self.previous_applied_steer > 0.0
        ):
            compensated = self.previous_applied_steer
        self.previous_applied_steer = compensated
        return compensated

    def _steering_scale_from_speed(self, speed_kmh: float) -> float:
        params = self.config.parameters
        return max(
            1e-3,
            _interpolate_map(
                speed_kmh,
                params.steering_scale_map_kmh,
                params.steering_scale_map,
            ),
        )

    def _build_solver(self) -> None:
        cfg = self.config
        nx, nu, horizon = 5, 2, cfg.horizon
        opti = ca.Opti()
        states = opti.variable(nx, horizon + 1)
        controls = opti.variable(nu, horizon)
        slacks = opti.variable(horizon + 1)
        x0 = opti.parameter(nx)
        reference = opti.parameter(nx, horizon)
        curvatures = opti.parameter(horizon)
        last_u = opti.parameter(nu)

        q_diag = np.array(
            [
                cfg.q_lateral,
                cfg.q_heading,
                cfg.q_speed,
                cfg.q_lateral_velocity,
                cfg.q_yaw_rate,
            ],
            dtype=float,
        )
        cost = ca.mtimes(slacks.T, slacks) * 1000.0
        for stage in range(horizon):
            state_error = states[:, stage] - reference[:, stage]
            control = controls[:, stage]
            previous = last_u if stage == 0 else controls[:, stage - 1]
            rate = control - previous
            stage_weight = (
                cfg.terminal_weight_scale
                if stage == horizon - 1
                else 1.0
            )
            cost += stage_weight * ca.dot(q_diag, state_error**2)
            cost += cfg.r_steer * control[0] ** 2
            cost += cfg.r_accel * control[1] ** 2
            cost += cfg.rd_steer * rate[0] ** 2
            cost += cfg.rd_accel * rate[1] ** 2
            opti.subject_to(
                opti.bounded(
                    -cfg.steer_rate_limit_rad_s * cfg.dt,
                    rate[0],
                    cfg.steer_rate_limit_rad_s * cfg.dt,
                )
            )
            opti.subject_to(
                opti.bounded(
                    -cfg.accel_rate_limit_mps3 * cfg.dt,
                    rate[1],
                    cfg.accel_rate_limit_mps3 * cfg.dt,
                )
            )
            next_state = self._rk4_step(
                states[:, stage],
                controls[:, stage],
                curvatures[stage],
            )
            opti.subject_to(states[:, stage + 1] == next_state)

        terminal_error = states[:, horizon] - reference[:, -1]
        cost += (
            cfg.terminal_weight_scale
            * ca.dot(q_diag, terminal_error**2)
        )

        opti.subject_to(states[:, 0] == x0)
        opti.subject_to(
            states[0, :] >= -8.0 - slacks.T
        )
        opti.subject_to(
            states[0, :] <= 8.0 + slacks.T
        )
        opti.subject_to(
            states[1, :] >= -1.5 - slacks.T
        )
        opti.subject_to(
            states[1, :] <= 1.5 + slacks.T
        )
        opti.subject_to(
            states[2, :] >= 0.5 - slacks.T
        )
        opti.subject_to(
            states[2, :] <= 80.0 + slacks.T
        )
        opti.subject_to(
            states[3, :] >= -25.0 - slacks.T
        )
        opti.subject_to(
            states[3, :] <= 25.0 + slacks.T
        )
        opti.subject_to(
            states[4, :] >= -3.0 - slacks.T
        )
        opti.subject_to(
            states[4, :] <= 3.0 + slacks.T
        )
        opti.subject_to(slacks >= 0.0)
        opti.subject_to(
            opti.bounded(
                -cfg.steer_limit,
                controls[0, :],
                cfg.steer_limit,
            )
        )
        opti.subject_to(
            opti.bounded(
                -cfg.brake_limit_mps2,
                controls[1, :],
                cfg.accel_limit_mps2,
            )
        )
        opti.minimize(cost)
        if cfg.nlp_solver == "ipopt":
            options = {
                "print_time": False,
                "ipopt": {
                    "print_level": 0,
                    "max_iter": 40,
                    "warm_start_init_point": "yes",
                    "sb": "yes",
                    "tol": 1e-3,
                },
            }
        else:
            options = {
                "print_time": False,
                "print_header": False,
                "print_iteration": False,
                "max_iter": 20,
                "hessian_approx": "gauss_newton",
                "regularize": True,
                "qpsol": "osqp",
                "qpsol_options": {
                    "print_time": False,
                    "verbose": False,
                },
            }
        opti.solver(cfg.nlp_solver, options)
        self._solver = (
            opti,
            states,
            controls,
            x0,
            reference,
            curvatures,
            last_u,
            slacks,
        )

    def _solve_nonlinear(
        self,
        x0: np.ndarray,
        reference: MpcReferenceSequence,
    ) -> tuple[np.ndarray | None, str]:
        if self._solver is None:
            return None, "solver-not-built"
        (
            opti,
            states,
            controls,
            x0_parameter,
            reference_parameter,
            curvature_parameter,
            last_u_parameter,
            slacks,
        ) = self._solver
        opti.set_value(x0_parameter, x0)
        opti.set_value(reference_parameter, reference.states.T)
        opti.set_value(curvature_parameter, reference.curvatures)
        opti.set_value(last_u_parameter, self.previous_control)

        if self.previous_solution is None:
            initial_states = np.tile(
                np.concatenate([np.zeros(2), [max(3.0, x0[2])], [0.0, 0.0]]),
                (self.config.horizon + 1, 1),
            ).T
            initial_states[:, 0] = x0
            initial_controls = np.zeros((2, self.config.horizon))
        else:
            initial_states = np.hstack(
                [
                    self.previous_solution["states"][:, 1:],
                    self.previous_solution["states"][:, -1:],
                ]
            )
            initial_states[:, 0] = x0
            initial_controls = np.hstack(
                [
                    self.previous_solution["controls"][:, 1:],
                    self.previous_solution["controls"][:, -1:],
                ]
            )
        opti.set_initial(states, initial_states)
        opti.set_initial(controls, initial_controls)
        opti.set_initial(slacks, np.zeros(self.config.horizon + 1))
        try:
            with (
                contextlib.redirect_stdout(io.StringIO()),
                contextlib.redirect_stderr(io.StringIO()),
            ):
                solution = opti.solve()
        except RuntimeError:
            return None, "ipopt-infeasible"
        self.previous_solution = {
            "states": np.asarray(solution.value(states)),
            "controls": np.asarray(solution.value(controls)),
        }
        return np.asarray(solution.value(controls))[:, 0], "ipopt-ok"

    def _rk4_step(
        self,
        state: ca.MX,
        control: ca.MX,
        curvature: ca.MX,
    ) -> ca.MX:
        dt = self.config.dt
        k1 = self._dynamics(state, control, curvature)
        k2 = self._dynamics(state + 0.5 * dt * k1, control, curvature)
        k3 = self._dynamics(state + 0.5 * dt * k2, control, curvature)
        k4 = self._dynamics(state + dt * k3, control, curvature)
        return state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0

    def _dynamics(
        self,
        state: ca.MX,
        control: ca.MX,
        curvature: ca.MX,
    ) -> ca.MX:
        lateral_error = state[0]
        heading_error = state[1]
        vx = ca.fmax(state[2], 0.5)
        vy = state[3]
        yaw_rate = state[4]
        steer = control[0]
        accel = control[1]
        params = self.config.parameters
        m = params.mass_kg
        iz = params.yaw_inertia_kgm2
        a = params.front_axle_to_cg_m
        b = params.rear_axle_to_cg_m
        wheelbase = params.wheelbase_m
        gravity = 9.81
        static_front = m * gravity * b / wheelbase
        static_rear = m * gravity * a / wheelbase
        load_transfer = (
            m
            * ca.fmax(
                -params.max_brake_mps2,
                ca.fmin(params.max_accel_mps2, accel),
            )
            * params.cg_height_m
            / wheelbase
        )
        front_load = ca.fmax(1000.0, static_front - load_transfer)
        rear_load = ca.fmax(1000.0, static_rear + load_transfer)
        slip_front = steer - ca.atan2(vy + a * yaw_rate, vx)
        slip_rear = -ca.atan2(vy - b * yaw_rate, vx)
        force_front = self._tire_force(slip_front, front_load)
        force_rear = self._tire_force(slip_rear, rear_load)
        lateral_displacement = (
            vx * ca.sin(heading_error)
            + vy * ca.cos(heading_error)
        )
        heading_rate = (
            yaw_rate
            - vx * curvature / ca.fmax(1.0 - lateral_error * curvature, 0.2)
        )
        vx_dot = accel + vy * yaw_rate
        vy_dot = (
            (force_front * ca.cos(steer) + force_rear) / m
            - vx * yaw_rate
        )
        yaw_accel = (
            a * force_front * ca.cos(steer) - b * force_rear
        ) / iz
        return ca.vertcat(
            lateral_displacement,
            heading_rate,
            vx_dot,
            vy_dot,
            yaw_accel,
        )

    def _tire_force(self, slip_angle: ca.MX, normal_load: ca.MX) -> ca.MX:
        mu = self.config.parameters.tire_friction_coefficient
        stiffness = 8.0
        shape = 1.8
        curvature = 0.8
        peak = mu * normal_load
        return peak * ca.sin(
            shape
            * ca.atan(
                stiffness * slip_angle
                - curvature
                * (
                    stiffness * slip_angle
                    - ca.atan(stiffness * slip_angle)
                )
            )
        )
