from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from actc.lap import StanleyConfig, StanleyController  # noqa: E402
from actc.mpc import (  # noqa: E402
    LinearMpcController,
    MpcConfig,
    MpcVehicleParameters,
    NonlinearMpcController,
)
from actc.track import TrackPath  # noqa: E402
from tools.simulate_lap import (  # noqa: E402
    VehicleParameters,
    VehicleState,
    clamp,
    integrate_step,
)


def make_oval_path(
    semi_major_m: float = 140.0,
    semi_minor_m: float = 80.0,
    count: int = 720,
    speed_kmh: float = 54.0,
) -> TrackPath:
    points = [
        (
            semi_major_m * math.cos(2.0 * math.pi * index / count),
            semi_minor_m * math.sin(2.0 * math.pi * index / count),
        )
        for index in range(count)
    ]
    speed_limits = [speed_kmh] * count
    return TrackPath.from_points(points, speed_limits)


def load_model(path: Path | None) -> MpcVehicleParameters | None:
    if path is None:
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    values = payload.get("model", payload)
    filtered = {}
    for key, value in values.items():
        if key not in MpcVehicleParameters.__dataclass_fields__:
            continue
        if isinstance(value, (int, float)):
            filtered[key] = float(value)
        elif isinstance(value, list):
            filtered[key] = tuple(float(item) for item in value)
    return MpcVehicleParameters(**filtered)


def make_controller(
    name: str,
    model: MpcVehicleParameters | None = None,
    overrides: dict[str, float] | None = None,
    horizon: int = 20,
) -> object:
    if name == "stanley":
        return StanleyController(
            StanleyConfig(
                cross_track_gain=0.65,
                heading_gain=1.0,
                steering_gain=1.0,
                steer_rate_per_s=1.5,
                steering_filter_tau_s=0.10,
            )
        )
    config = MpcConfig(horizon=horizon, dt=0.02)
    if model is not None:
        config.parameters = model
    for key, value in (overrides or {}).items():
        if hasattr(config, key):
            setattr(config, key, value)
    if name == "lmpc":
        return LinearMpcController(config)
    if name == "nmpc":
        return NonlinearMpcController(config)
    raise ValueError(name)


def controller_state(state: VehicleState) -> SimpleNamespace:
    return SimpleNamespace(
        position=(state.x, 0.0, state.z),
        speed_ms=state.vx,
        speed_kmh=state.vx * 3.6,
        heading_rad=state.yaw - math.pi / 2.0,
        yaw_rate_rad_s=-state.yaw_rate,
        local_velocity_m_s=(-state.vy, 0.0, state.vx),
        steer=state.steer,
    )


def run_controller(
    name: str,
    *,
    duration_s: float,
    solve_stride: int,
    model: MpcVehicleParameters | None = None,
    overrides: dict[str, float] | None = None,
    horizon: int = 20,
    speed_kmh: float = 54.0,
) -> dict[str, float | str]:
    path = make_oval_path(speed_kmh=speed_kmh)
    params = VehicleParameters()
    state = VehicleState(
        x=path.points[0][0],
        z=path.points[0][1],
        yaw=path.heading_rad[0],
        vx=0.0,
        vy=0.0,
        yaw_rate=0.0,
        steer=0.0,
    )
    controller = make_controller(name, model, overrides, horizon)
    fake_state = controller_state(state)
    controller.calibrate(fake_state, path)
    dt = 0.02
    elapsed = 0.0
    previous_index = 0
    progress_m = 0.0
    sum_error_sq = 0.0
    max_error = 0.0
    samples = 0
    solve_times: list[float] = []
    solver_iterations: list[int] = []
    steer_commands: list[float] = []
    last_info = None

    while elapsed < duration_s:
        fake_state = controller_state(state)
        if (
            last_info is None
            or int(round(elapsed / dt)) % max(1, solve_stride) == 0
        ):
            info = controller.step(fake_state, path, dt)
            last_info = info
            if hasattr(controller, "last_stats"):
                solve_times.append(float(controller.last_stats.solve_ms))
                solver_iterations.append(
                    int(controller.last_stats.iterations)
                )
        else:
            info = last_info

        step = (info.path_index - previous_index) % len(path.points)
        if step < len(path.points) / 2:
            progress_m += step * (
                path.total_length_m / len(path.points)
            )
        previous_index = info.path_index
        error = path.signed_lateral_error(
            info.path_index,
            state.x,
            state.z,
        )
        sum_error_sq += error * error
        max_error = max(max_error, abs(error))
        samples += 1

        target_speed = path.speed_limit_kmh[info.path_index] / 3.6
        acceleration = clamp(
            (target_speed - state.vx) * 0.8,
            -params.max_brake_mps2,
            params.max_accel_mps2,
        )
        requested_angle = info.steer * params.max_steer_rad
        state.steer += clamp(
            requested_angle - state.steer,
            -params.max_steer_rate_rad_s * dt,
            params.max_steer_rate_rad_s * dt,
        )
        integrate_step(state, acceleration, state.steer, params, dt)
        steer_commands.append(info.steer)
        elapsed += dt

        if progress_m > path.total_length_m * 0.999 and elapsed > 20.0:
            break
        if abs(error) > 20.0:
            break

    trim = min(100, len(steer_commands) // 4)
    active_steer = steer_commands[trim:]
    steer_delta = [
        abs(active_steer[index] - active_steer[index - 1])
        for index in range(1, len(active_steer))
    ]
    ordered_solve_times = sorted(solve_times)
    return {
        "controller": name,
        "completed": progress_m > path.total_length_m * 0.999,
        "sim_time_s": elapsed,
        "distance_m": progress_m,
        "max_error_m": max_error,
        "rms_error_m": math.sqrt(sum_error_sq / max(1, samples)),
        "steering_activity": statistics.mean(steer_delta)
        if steer_delta
        else 0.0,
        "mean_solve_ms": statistics.mean(solve_times) if solve_times else 0.0,
        "p50_solve_ms": (
            statistics.median(solve_times) if solve_times else 0.0
        ),
        "p95_solve_ms": (
            ordered_solve_times[int(0.95 * (len(solve_times) - 1))]
            if solve_times
            else 0.0
        ),
        "p99_solve_ms": (
            ordered_solve_times[int(0.99 * (len(solve_times) - 1))]
            if solve_times
            else 0.0
        ),
        "max_solve_ms": max(solve_times, default=0.0),
        "solve_count": len(solve_times),
        "mean_iterations": (
            statistics.mean(solver_iterations)
            if solver_iterations
            else 0.0
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--controllers",
        nargs="+",
        choices=("stanley", "lmpc", "nmpc"),
        default=("stanley", "lmpc", "nmpc"),
    )
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--mpc-model", type=Path, default=None)
    parser.add_argument("--lmpc-params", default="{}")
    parser.add_argument("--horizon", type=int, default=20)
    parser.add_argument("--speed-kmh", type=float, default=54.0)
    args = parser.parse_args()
    model = load_model(args.mpc_model)
    overrides = {
        str(key): float(value)
        for key, value in json.loads(args.lmpc_params).items()
    }
    strides = {"stanley": 1, "lmpc": 1, "nmpc": 10}
    for name in args.controllers:
        result = run_controller(
            name,
            duration_s=args.duration,
            solve_stride=strides[name],
            model=model,
            overrides=overrides,
            horizon=args.horizon,
            speed_kmh=args.speed_kmh,
        )
        print(
            " ".join(f"{key}={value}" for key, value in result.items())
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
