from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from actc.controllers import (  # noqa: E402
    SpeedController,
    SpeedControllerConfig,
)
from actc.longitudinal import (  # noqa: E402
    LongitudinalMpcConfig,
    LongitudinalMpcController,
    LongitudinalPedalMapper,
    acceleration_capability_mps2,
    braking_capability_mps2,
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def load_lap(path: Path, lap_number: int) -> pd.DataFrame:
    frame = pd.read_csv(path)
    frame = frame[frame["lap_number"] == lap_number].copy()
    if frame.empty:
        raise ValueError(f"lap {lap_number} not found in {path}")
    return frame.sort_values("timestamp").reset_index(drop=True)


def simulate(
    frame: pd.DataFrame,
    *,
    controller_name: str,
    kp_accel: float,
    kp_brake: float,
    ki: float,
    prediction_time_s: float,
    plant_tau_s: float,
    horizon: int,
    mpc_q_speed: float,
    mpc_q_accel: float,
    mpc_r_accel: float,
    mpc_r_jerk: float,
    mpc_max_jerk: float,
) -> tuple[pd.DataFrame, dict]:
    dt = float(np.median(np.diff(frame["timestamp"].to_numpy())))
    dt = clamp(dt, 0.01, 0.10)
    speed = float(frame["speed_kmh"].iloc[0]) / 3.6
    acceleration = float(frame["acceleration_g_z"].iloc[0]) * 9.81
    reference_speed = frame["target_speed_kmh"].to_numpy() / 3.6
    pi = SpeedController(
        SpeedControllerConfig(
            target_kmh=float(frame["target_speed_kmh"].iloc[0]),
            kp_accel=kp_accel,
            kp_brake=kp_brake,
            ki=ki,
            prediction_time_s=prediction_time_s,
        )
    )
    mpc = LongitudinalMpcController(
        LongitudinalMpcConfig(
            horizon=horizon,
            dt=dt,
            response_tau_s=plant_tau_s,
            q_speed=mpc_q_speed,
            q_accel=mpc_q_accel,
            r_accel=mpc_r_accel,
            r_jerk=mpc_r_jerk,
            max_jerk_mps3=mpc_max_jerk,
        )
    )
    pedal_mapper = LongitudinalPedalMapper()
    rows: list[dict] = []
    for index in range(len(frame)):
        target = float(reference_speed[index])
        if controller_name == "mpc":
            preview = reference_speed[index : index + horizon + 1]
            result = mpc.step(
                speed_mps=speed,
                acceleration_mps2=acceleration,
                reference_speed_mps=preview,
                acceleration_limit_mps2=(
                    acceleration_capability_mps2(speed)
                ),
                braking_limit_mps2=(
                    braking_capability_mps2(speed)
                ),
            )
            acceleration_request = result.acceleration_command_mps2
            solve_ms = result.solve_time_ms
            throttle, brake = pedal_mapper.step(
                acceleration_mps2=acceleration_request,
                speed_mps=speed,
                steering=0.0,
                rear_slip=0.0,
                dt=dt,
            )
            acceleration_request = (
                throttle * acceleration_capability_mps2(speed)
                - brake * braking_capability_mps2(speed)
            )
        else:
            throttle, brake = pi.step(
                speed * 3.6,
                dt,
                target * 3.6,
            )
            acceleration_request = (
                throttle * acceleration_capability_mps2(speed)
                - brake * braking_capability_mps2(speed)
            )
            solve_ms = 0.0
            acceleration_request = (
                throttle * acceleration_capability_mps2(speed)
                - brake * braking_capability_mps2(speed)
            )
        acceleration_request = clamp(
            acceleration_request,
            -braking_capability_mps2(speed),
            acceleration_capability_mps2(speed),
        )
        acceleration += (
            dt
            / max(plant_tau_s, 1e-3)
            * (acceleration_request - acceleration)
        )
        speed = max(0.0, speed + dt * acceleration)
        rows.append(
            {
                "time_s": index * dt,
                "reference_kmh": target * 3.6,
                "speed_kmh": speed * 3.6,
                "speed_error_kmh": (speed - target) * 3.6,
                "acceleration_g": acceleration / 9.81,
                "acceleration_request_g": acceleration_request / 9.81,
                "throttle": throttle,
                "brake": brake,
                "solve_ms": solve_ms,
            }
        )
    result_frame = pd.DataFrame(rows)
    speed_error = result_frame["speed_error_kmh"].to_numpy()
    throttle = result_frame["throttle"].to_numpy()
    brake = result_frame["brake"].to_numpy()
    metrics = {
        "controller": controller_name,
        "samples": len(result_frame),
        "speed_rmse_kmh": float(np.sqrt(np.mean(speed_error**2))),
        "speed_abs_p95_kmh": float(
            np.percentile(np.abs(speed_error), 95)
        ),
        "speed_abs_max_kmh": float(np.max(np.abs(speed_error))),
        "throttle_delta_mean": float(np.mean(np.abs(np.diff(throttle)))),
        "brake_delta_mean": float(np.mean(np.abs(np.diff(brake)))),
        "throttle_jerk_mean": float(
            np.mean(np.abs(np.diff(throttle, n=2)))
        ),
        "brake_jerk_mean": float(
            np.mean(np.abs(np.diff(brake, n=2)))
        ),
        "solve_ms_mean": float(result_frame["solve_ms"].mean()),
        "solve_ms_p95": float(
            np.percentile(result_frame["solve_ms"], 95)
        ),
        "solve_ms_max": float(result_frame["solve_ms"].max()),
    }
    return result_frame, metrics


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log",
        type=Path,
        default=(
            PROJECT_ROOT
            / "logs"
            / "lmpc_round2_v8_10_laps.csv"
        ),
    )
    parser.add_argument("--lap", type=int, default=7)
    parser.add_argument("--horizon", type=int, default=50)
    parser.add_argument("--plant-tau", type=float, default=0.10)
    parser.add_argument("--mpc-q-speed", type=float, default=18.0)
    parser.add_argument("--mpc-q-accel", type=float, default=0.8)
    parser.add_argument("--mpc-r-accel", type=float, default=0.6)
    parser.add_argument("--mpc-r-jerk", type=float, default=25.0)
    parser.add_argument("--mpc-max-jerk", type=float, default=7.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            PROJECT_ROOT
            / "logs"
            / "longitudinal_mpc_benchmark.json"
        ),
    )
    args = parser.parse_args()
    frame = load_lap(args.log, args.lap)
    variants = [
        (
            "pi_original",
            dict(
                kp_accel=0.13710594150236166,
                kp_brake=0.10282945612677125,
                ki=0.011256803381250007,
                prediction_time_s=0.07,
            ),
        ),
        (
            "pi_tuned",
            dict(
                kp_accel=0.18,
                kp_brake=0.15,
                ki=0.012,
                prediction_time_s=0.08,
            ),
        ),
        (
            "mpc",
            dict(
                kp_accel=0.0,
                kp_brake=0.0,
                ki=0.0,
                prediction_time_s=0.0,
            ),
        ),
    ]
    results = []
    for name, gains in variants:
        trace, metrics = simulate(
            frame,
            controller_name=name,
            plant_tau_s=args.plant_tau,
            horizon=args.horizon,
            mpc_q_speed=args.mpc_q_speed,
            mpc_q_accel=args.mpc_q_accel,
            mpc_r_accel=args.mpc_r_accel,
            mpc_r_jerk=args.mpc_r_jerk,
            mpc_max_jerk=args.mpc_max_jerk,
            **gains,
        )
        results.append(metrics)
        trace.to_csv(
            args.output.with_name(
                f"{args.output.stem}_{name}.csv"
            ),
            index=False,
        )
    payload = {
        "source_log": str(args.log),
        "lap": args.lap,
        "plant_tau_s": args.plant_tau,
        "results": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
