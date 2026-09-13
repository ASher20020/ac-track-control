from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


SPEED_BANDS = (
    (0.0, 40.0),
    (40.0, 80.0),
    (80.0, 140.0),
    (140.0, math.inf),
)


def as_float(row: dict[str, str], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return 0.0


def speed_band(speed_kmh: float) -> int:
    for index, (lower, upper) in enumerate(SPEED_BANDS):
        if lower <= speed_kmh < upper:
            return index
    return len(SPEED_BANDS) - 1


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * fraction))
    return ordered[index]


def load_clean_samples(
    path: Path,
    *,
    first_lap: int,
    max_steer_rad: float,
) -> tuple[list[dict[str, float]], list[dict[str, float]]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    samples: list[dict[str, float]] = []
    by_band: list[list[dict[str, float]]] = [[], [], [], []]
    previous: dict[str, str] | None = None
    previous_time = 0.0
    for row in rows:
        lap = int(as_float(row, "lap_number"))
        timestamp = as_float(row, "timestamp")
        if previous is None or lap != int(as_float(previous, "lap_number")):
            previous = row
            previous_time = timestamp
            continue

        dt = timestamp - previous_time
        speed_kmh = as_float(row, "speed_kmh")
        vx = as_float(row, "local_velocity_z")
        vy_previous = -as_float(previous, "local_velocity_x")
        vy = -as_float(row, "local_velocity_x")
        yaw_rate_previous = -as_float(
            previous,
            "yaw_rate_rad_s",
        )
        yaw_rate = -as_float(row, "yaw_rate_rad_s")
        steer = as_float(row, "steer") * max_steer_rad
        surface_grip = as_float(row, "surface_grip")
        tyres_out = int(as_float(row, "tyres_out"))

        valid = (
            lap >= first_lap
            and 0.005 <= dt <= 0.10
            and vx > 5.0
            and abs(vy) < 15.0
            and abs(yaw_rate) < 1.5
            and abs(steer) < max_steer_rad * 0.95
            and tyres_out == 0
            and surface_grip >= 0.85
        )
        if valid:
            sample = {
                "lap_number": float(lap),
                "speed_kmh": speed_kmh,
                "vx": vx,
                "vy": vy,
                "yaw_rate": yaw_rate,
                "steer": steer,
                "vy_dot": (vy - vy_previous) / dt,
                "yaw_accel": (yaw_rate - yaw_rate_previous) / dt,
                # AC local x points right; the bicycle model uses y left.
                "lateral_g": -as_float(row, "acceleration_g_x"),
                "normalized_position": as_float(
                    row,
                    "normalized_position",
                ),
                "target_speed_kmh": as_float(
                    row,
                    "target_speed_kmh",
                ),
                "lateral_error": as_float(row, "lateral_error_m"),
                "heading_error": as_float(
                    row,
                    "heading_error_rad",
                ),
                "cmd_steer": as_float(row, "cmd_steer"),
            }
            samples.append(sample)
            by_band[speed_band(speed_kmh)].append(sample)

        previous = row
        previous_time = timestamp
    return samples, [sample for band in by_band for sample in band]


def fit_vehicle_model(
    samples: list[dict[str, float]],
    *,
    mass_kg: float,
    front_axle_to_cg_m: float,
    wheelbase_m: float,
) -> dict[str, float]:
    if len(samples) < 500:
        raise RuntimeError("Not enough clean samples for identification.")
    a = front_axle_to_cg_m
    b = wheelbase_m - a
    data = np.asarray(
        [
            [
                sample["vx"],
                sample["vy"],
                sample["yaw_rate"],
                sample["steer"],
                sample["vy_dot"],
                sample["yaw_accel"],
            ]
            for sample in samples
        ],
        dtype=float,
    )
    vx = data[:, 0]
    vy = data[:, 1]
    yaw_rate = data[:, 2]
    steer = data[:, 3]
    vy_dot = data[:, 4]
    yaw_accel = data[:, 5]

    lateral_features = np.column_stack(
        (
            -vy / vx,
            yaw_rate / vx,
            steer,
        )
    )
    del vy_dot
    lateral_target = np.asarray(
        [sample["lateral_g"] for sample in samples],
        dtype=float,
    ) * 9.81
    coefficients, *_ = np.linalg.lstsq(
        lateral_features,
        lateral_target,
        rcond=None,
    )
    stiffness_sum_over_mass = float(coefficients[0])
    stiffness_moment_over_mass = float(coefficients[1])
    front_stiffness_over_mass = float(coefficients[2])
    front_stiffness = front_stiffness_over_mass * mass_kg
    rear_stiffness = (
        stiffness_sum_over_mass * mass_kg - front_stiffness
    )
    predicted_lateral = lateral_features @ coefficients
    lateral_rmse = math.sqrt(
        float(np.mean((predicted_lateral - lateral_target) ** 2))
    )

    yaw_feature = (
        (b * rear_stiffness - a * front_stiffness)
        * vy
        / vx
        - (a * a * front_stiffness + b * b * rear_stiffness)
        * yaw_rate
        / vx
        + a * front_stiffness * steer
    )
    inertia_over_one = float(
        np.dot(yaw_feature, yaw_accel)
        / max(np.dot(yaw_feature, yaw_feature), 1e-9)
    )
    yaw_inertia = 1.0 / max(inertia_over_one, 1e-9)
    predicted_yaw_accel = yaw_feature / yaw_inertia
    yaw_rmse = math.sqrt(
        float(np.mean((predicted_yaw_accel - yaw_accel) ** 2))
    )

    wheelbase = wheelbase_m
    understeer_gradient = (
        mass_kg
        / wheelbase
        * (
            a / max(front_stiffness, 1.0)
            - b / max(rear_stiffness, 1.0)
        )
    )
    return {
        "mass_kg": mass_kg,
        "front_axle_to_cg_m": a,
        "wheelbase_m": wheelbase_m,
        "front_cornering_stiffness_n_rad": front_stiffness,
        "rear_cornering_stiffness_n_rad": rear_stiffness,
        "yaw_inertia_kgm2": yaw_inertia,
        "understeer_gradient_rad_s2_m": understeer_gradient,
        "lateral_fit_rmse_mps2": lateral_rmse,
        "yaw_fit_rmse_rad_s2": yaw_rmse,
        "samples": float(len(samples)),
        "moment_consistency_mps2_per_rad": stiffness_moment_over_mass,
    }


def summarize_bands(
    samples: list[dict[str, float]],
) -> list[dict[str, float]]:
    output: list[dict[str, float]] = []
    for index, (lower, upper) in enumerate(SPEED_BANDS):
        band = [
            sample
            for sample in samples
            if speed_band(sample["speed_kmh"]) == index
        ]
        if not band:
            output.append(
                {
                    "band": float(index),
                    "lower_kmh": lower,
                    "upper_kmh": upper,
                    "samples": 0.0,
                }
            )
            continue
        steering = [abs(sample["cmd_steer"]) for sample in band]
        output.append(
            {
                "band": float(index),
                "lower_kmh": lower,
                "upper_kmh": upper,
                "samples": float(len(band)),
                "p95_lateral_error_m": percentile(
                    [abs(sample["lateral_error"]) for sample in band],
                    0.95,
                ),
                "p95_heading_error_rad": percentile(
                    [abs(sample["heading_error"]) for sample in band],
                    0.95,
                ),
                "mean_abs_lateral_g": float(
                    np.mean([abs(sample["lateral_g"]) for sample in band])
                ),
                "p95_abs_lateral_g": percentile(
                    [abs(sample["lateral_g"]) for sample in band],
                    0.95,
                ),
                "mean_target_gap_kmh": float(
                    np.mean(
                        [
                            sample["target_speed_kmh"]
                            - sample["speed_kmh"]
                            for sample in band
                        ]
                    )
                ),
                "mean_steer": float(np.mean(steering)),
                "p95_steer": percentile(steering, 0.95),
            }
        )
    return output


def summarize_positions(
    samples: list[dict[str, float]],
    *,
    bins: int,
) -> list[dict[str, float]]:
    grouped: list[list[dict[str, float]]] = [
        [] for _ in range(bins)
    ]
    for sample in samples:
        position = max(
            0.0,
            min(0.999999, sample["normalized_position"]),
        )
        grouped[int(position * bins)].append(sample)

    output: list[dict[str, float]] = []
    for index, group in enumerate(grouped):
        if not group:
            continue
        output.append(
            {
                "bin": float(index),
                "start_s": index / bins,
                "end_s": (index + 1) / bins,
                "samples": float(len(group)),
                "mean_speed_kmh": float(
                    np.mean([sample["speed_kmh"] for sample in group])
                ),
                "mean_target_kmh": float(
                    np.mean(
                        [
                            sample["target_speed_kmh"]
                            for sample in group
                        ]
                    )
                ),
                "p95_lateral_error_m": percentile(
                    [abs(sample["lateral_error"]) for sample in group],
                    0.95,
                ),
                "p95_heading_error_rad": percentile(
                    [abs(sample["heading_error"]) for sample in group],
                    0.95,
                ),
                "p95_lateral_g": percentile(
                    [abs(sample["lateral_g"]) for sample in group],
                    0.95,
                ),
            }
        )
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--first-lap", type=int, default=6)
    parser.add_argument("--bins", type=int, default=20)
    parser.add_argument("--mass-kg", type=float, default=1500.0)
    parser.add_argument("--front-axle-to-cg-m", type=float, default=1.2)
    parser.add_argument("--wheelbase-m", type=float, default=2.85)
    parser.add_argument("--max-steer-rad", type=float, default=0.52)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    samples, _ = load_clean_samples(
        args.log,
        first_lap=args.first_lap,
        max_steer_rad=args.max_steer_rad,
    )
    model = fit_vehicle_model(
        samples,
        mass_kg=args.mass_kg,
        front_axle_to_cg_m=args.front_axle_to_cg_m,
        wheelbase_m=args.wheelbase_m,
    )
    result = {
        "model": model,
        "bands": summarize_bands(samples),
        "positions": summarize_positions(samples, bins=args.bins),
    }
    output = args.output or args.log.with_name(
        f"{args.log.stem}_mpc_analysis.json"
    )
    output.write_text(
        json.dumps(result, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "model": model,
                "bands": result["bands"],
                "output": str(output),
            },
            indent=2,
            ensure_ascii=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
