from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from actc.lap import (  # noqa: E402
    LqrConfig,
    LqrController,
    StanleyConfig,
    StanleyController,
)
from actc.mpc import (  # noqa: E402
    LinearMpcController,
    MpcConfig,
    NonlinearMpcController,
)
from actc.track import (  # noqa: E402
    AC_PHYSICS_TO_WORLD_HEADING_RAD,
    TrackPath,
    resolve_fast_lane,
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class VehicleParameters:
    wheelbase_m: float = 2.85
    front_axle_to_cg_m: float = 1.20
    mass_kg: float = 1500.0
    yaw_inertia_kgm2: float = 2250.0
    front_axle_stiffness_n_rad: float = 80000.0
    rear_axle_stiffness_n_rad: float = 80000.0
    max_steer_rad: float = 0.52
    max_steer_rate_rad_s: float = 1.5
    max_accel_mps2: float = 2.5
    max_brake_mps2: float = 4.0


@dataclass
class VehicleState:
    x: float
    z: float
    yaw: float
    vx: float
    vy: float
    yaw_rate: float
    steer: float


def simulate(
    path_file: Path,
    *,
    start_fraction: float,
    initial_offset_m: float,
    path_offset_m: float = 0.0,
    use_merge_plan: bool = False,
    max_speed_kmh: float,
    lateral_accel_mps2: float = 3.0,
    simulation_hz: int,
    max_seconds: float,
    controller_name: str = "lqr",
    cross_track_gain: float = 0.50,
    heading_gain: float = 0.70,
    steering_gain: float = 0.60,
    merge_distance_m: float = 150.0,
    q_lateral: float = 3.0,
    q_lateral_rate: float = 0.8,
    q_heading: float = 1.5,
    q_heading_rate: float = 0.4,
    r_steering: float = 1.0,
    solver_budget_ms: float = 18.0,
    solve_stride: int = 1,
) -> tuple[bool, dict[str, float], list[tuple[float, float]]]:
    params = VehicleParameters()
    path = TrackPath.load(
        path_file,
        centerline=True,
        lateral_offset_m=path_offset_m,
        max_speed_kmh=max_speed_kmh,
        min_speed_kmh=25.0,
        lateral_accel_mps2=lateral_accel_mps2,
    )
    count = len(path.points)
    start_index = int(clamp(start_fraction, 0.0, 0.999) * count)
    start_x, start_z = path.points[start_index]
    tangent = path.heading_rad[start_index]
    normal_x = -math.sin(tangent)
    normal_z = math.cos(tangent)
    state = VehicleState(
        x=start_x + initial_offset_m * normal_x,
        z=start_z + initial_offset_m * normal_z,
        yaw=tangent,
        vx=0.0,
        vy=0.0,
        yaw_rate=0.0,
        steer=0.0,
    )
    if use_merge_plan:
        path = path.plan_merge(
            car_x=state.x,
            car_z=state.z,
            car_heading_rad=state.yaw,
            merge_distance_m=60.0,
            merge_speed_kmh=10.0,
        )

    dt = 1.0 / simulation_hz
    if controller_name == "lqr":
        controller = LqrController(
            LqrConfig(
                wheelbase_m=params.wheelbase_m,
                max_steer_rad=params.max_steer_rad,
                q_lateral=q_lateral,
                q_lateral_rate=q_lateral_rate,
                q_heading=q_heading,
                q_heading_rate=q_heading_rate,
                r_steering=r_steering,
                steering_sign=1.0,
                heading_sign=1.0,
            )
        )
    elif controller_name == "lmpc":
        controller = LinearMpcController(
            MpcConfig(
                horizon=20,
                dt=dt,
                solve_time_budget_ms=solver_budget_ms,
            )
        )
    elif controller_name == "nmpc":
        controller = NonlinearMpcController(
            MpcConfig(
                horizon=20,
                dt=dt,
                solve_time_budget_ms=solver_budget_ms,
            )
        )
    else:
        controller = StanleyController(
            StanleyConfig(
                wheelbase_m=params.wheelbase_m,
                max_steer_rad=params.max_steer_rad,
                cross_track_gain=cross_track_gain,
                heading_gain=heading_gain,
                steering_gain=steering_gain,
                merge_distance_m=merge_distance_m,
                steering_sign=1.0,
                heading_sign=1.0,
            )
        )
    fake_state = make_controller_state(state)
    controller.calibrate(fake_state, path)

    elapsed = 0.0
    previous_index = start_index
    cumulative_progress = 0.0
    max_abs_error = 0.0
    sum_error_sq = 0.0
    error_samples = 0
    solver_times_ms: list[float] = []
    last_info = None
    trace: list[tuple[float, float]] = [(state.x, state.z)]

    while elapsed < max_seconds:
        fake_state = make_controller_state(state)
        if last_info is None or elapsed / dt % max(1, solve_stride) < 0.5:
            info = controller.step(fake_state, path, dt)
            last_info = info
            if hasattr(controller, "last_stats"):
                solver_times_ms.append(controller.last_stats.solve_ms)
        else:
            info = last_info
        index = info.path_index
        if index != previous_index:
            step = (index - previous_index) % count
            if step < count / 2:
                cumulative_progress += step * (
                    path.total_length_m / max(1, count)
                )
            previous_index = index

        lateral_error = path.signed_lateral_error(index, state.x, state.z)
        max_abs_error = max(max_abs_error, abs(lateral_error))
        sum_error_sq += lateral_error * lateral_error
        error_samples += 1

        target_speed = path.speed_limit_kmh[index] / 3.6
        speed_error = target_speed - state.vx
        acceleration = clamp(
            speed_error * 0.8,
            -params.max_brake_mps2,
            params.max_accel_mps2,
        )

        command_steer = info.steer
        requested_angle = command_steer * params.max_steer_rad
        max_delta = params.max_steer_rate_rad_s * dt
        state.steer += clamp(
            requested_angle - state.steer,
            -max_delta,
            max_delta,
        )
        state.steer = clamp(
            state.steer,
            -params.max_steer_rad,
            params.max_steer_rad,
        )
        integrate_step(state, acceleration, state.steer, params, dt)
        elapsed += dt
        trace.append((state.x, state.z))

        if (
            cumulative_progress > path.total_length_m * 0.98
            and elapsed > 60.0
        ):
            metrics = {
                "lap_time_s": elapsed,
                "distance_m": cumulative_progress,
                "max_abs_error_m": max_abs_error,
                "rms_error_m": math.sqrt(
                    sum_error_sq / max(1, error_samples)
                ),
                "final_speed_kmh": state.vx * 3.6,
                "mean_solve_ms": (
                    sum(solver_times_ms) / max(1, len(solver_times_ms))
                ),
                "max_solve_ms": max(solver_times_ms, default=0.0),
            }
            return True, metrics, trace

        if abs(lateral_error) > 9.0:
            break
        if not math.isfinite(state.x + state.z + state.yaw + state.vx):
            break

    metrics = {
        "lap_time_s": elapsed,
        "distance_m": cumulative_progress,
        "max_abs_error_m": max_abs_error,
        "rms_error_m": math.sqrt(sum_error_sq / max(1, error_samples)),
        "final_speed_kmh": state.vx * 3.6,
        "mean_solve_ms": (
            sum(solver_times_ms) / max(1, len(solver_times_ms))
        ),
        "max_solve_ms": max(solver_times_ms, default=0.0),
    }
    return False, metrics, trace


def make_controller_state(state: VehicleState) -> SimpleNamespace:
    return SimpleNamespace(
        position=(state.x, 0.0, state.z),
        speed_ms=state.vx,
        speed_kmh=state.vx * 3.6,
        heading_rad=state.yaw - AC_PHYSICS_TO_WORLD_HEADING_RAD,
        yaw_rate_rad_s=-state.yaw_rate,
        local_velocity_m_s=(-state.vy, 0.0, state.vx),
    )


def integrate_step(
    state: VehicleState,
    acceleration: float,
    steer: float,
    params: VehicleParameters,
    dt: float,
) -> None:
    speed_floor = 2.0
    vx = max(state.vx, speed_floor)
    alpha_f = math.atan2(
        state.vy + params.front_axle_to_cg_m * state.yaw_rate,
        vx,
    ) - steer
    alpha_r = math.atan2(
        state.vy
        - (params.wheelbase_m - params.front_axle_to_cg_m) * state.yaw_rate,
        vx,
    )
    fyf = -params.front_axle_stiffness_n_rad * alpha_f
    fyr = -params.rear_axle_stiffness_n_rad * alpha_r

    ax = acceleration + state.vy * state.yaw_rate
    ay = -state.vx * state.yaw_rate + (fyf + fyr) / params.mass_kg
    yaw_accel = (
        params.front_axle_to_cg_m * fyf
        - (params.wheelbase_m - params.front_axle_to_cg_m) * fyr
    ) / params.yaw_inertia_kgm2

    state.x += state.vx * math.cos(state.yaw) * dt
    state.z += state.vx * math.sin(state.yaw) * dt
    state.yaw += state.yaw_rate * dt
    state.vx += ax * dt
    state.vy += ay * dt
    state.yaw_rate += yaw_accel * dt
    state.vx = max(0.0, state.vx)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--track", default="tr_shanghai")
    parser.add_argument("--track-config", default="advanced")
    parser.add_argument("--start-fraction", type=float, default=0.912)
    parser.add_argument("--initial-offset", type=float, default=4.6)
    parser.add_argument("--path-offset", type=float, default=0.0)
    parser.add_argument("--use-merge-plan", action="store_true")
    parser.add_argument("--max-speed", type=float, default=75.0)
    parser.add_argument("--lateral-accel", type=float, default=3.0)
    parser.add_argument("--hz", type=int, default=50)
    parser.add_argument("--max-seconds", type=float, default=600.0)
    parser.add_argument(
        "--controller",
        choices=("stanley", "lqr", "lmpc", "nmpc"),
        default="lqr",
    )
    parser.add_argument("--cross-gain", type=float, default=0.50)
    parser.add_argument("--heading-gain", type=float, default=0.70)
    parser.add_argument("--steering-gain", type=float, default=0.60)
    parser.add_argument("--merge-distance", type=float, default=150.0)
    parser.add_argument("--q-lateral", type=float, default=3.0)
    parser.add_argument("--q-lateral-rate", type=float, default=0.8)
    parser.add_argument("--q-heading", type=float, default=1.5)
    parser.add_argument("--q-heading-rate", type=float, default=0.4)
    parser.add_argument("--r-steering", type=float, default=1.0)
    parser.add_argument("--solver-budget-ms", type=float, default=18.0)
    parser.add_argument("--solve-stride", type=int, default=1)
    args = parser.parse_args()

    path_file = resolve_fast_lane(args.track, args.track_config)
    success, metrics, trace = simulate(
        path_file,
        start_fraction=args.start_fraction,
        initial_offset_m=args.initial_offset,
        path_offset_m=args.path_offset,
        use_merge_plan=args.use_merge_plan,
        max_speed_kmh=args.max_speed,
        lateral_accel_mps2=args.lateral_accel,
        simulation_hz=args.hz,
        max_seconds=args.max_seconds,
        controller_name=args.controller,
        cross_track_gain=args.cross_gain,
        heading_gain=args.heading_gain,
        steering_gain=args.steering_gain,
        merge_distance_m=args.merge_distance,
        q_lateral=args.q_lateral,
        q_lateral_rate=args.q_lateral_rate,
        q_heading=args.q_heading,
        q_heading_rate=args.q_heading_rate,
        r_steering=args.r_steering,
        solver_budget_ms=args.solver_budget_ms,
        solve_stride=max(1, args.solve_stride),
    )
    print("success:", success)
    for key, value in metrics.items():
        print(f"{key}: {value:.3f}")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
