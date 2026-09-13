from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


BANDS = (
    (40.0, 70.0),
    (70.0, 100.0),
    (100.0, 130.0),
    (130.0, 160.0),
    (160.0, 200.0),
)
MAP_SPEEDS_KMH = np.asarray(
    (0.0, 55.0, 85.0, 115.0, 145.0, 180.0, 240.0),
    dtype=float,
)


def load_arrays(path: Path) -> dict[str, np.ndarray]:
    columns: dict[str, list[float]] = {}
    with path.open("r", newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            for key, value in row.items():
                try:
                    columns.setdefault(key, []).append(float(value))
                except (TypeError, ValueError):
                    pass
    return {
        key: np.asarray(values, dtype=float)
        for key, values in columns.items()
    }


def fit_band(
    data: dict[str, np.ndarray],
    time_mask: np.ndarray,
    lower_kmh: float,
    upper_kmh: float,
    max_lateral_g: float,
) -> dict[str, float]:
    time_s = data["timestamp"]
    speed_kmh = data["speed_kmh"]
    vx = np.maximum(data["local_velocity_z"], 3.0)
    vy = -data["local_velocity_x"]
    yaw_rate = -data["yaw_rate_rad_s"]
    steer = data["steer"]
    lateral_g = np.abs(data["lateral_accel_g"])
    slip = data["wheel_slip_abs_max"]
    tyres_out = data["tyres_out"]
    surface_grip = data["surface_grip"]

    vy_dot = np.gradient(vy, time_s)
    yaw_accel = np.gradient(yaw_rate, time_s)
    valid = (
        time_mask
        & (speed_kmh >= lower_kmh)
        & (speed_kmh < upper_kmh)
        & (vx > 5.0)
        & (tyres_out == 0)
        & (slip < 0.25)
        & (lateral_g <= max_lateral_g)
        & (surface_grip >= 0.90)
        & (np.abs(steer) < 0.30)
    )
    indices = np.flatnonzero(valid)
    if len(indices) < 300:
        raise RuntimeError(
            f"Not enough samples in {lower_kmh:.0f}-{upper_kmh:.0f} km/h."
        )

    band_vx = vx[indices]
    band_vy = vy[indices]
    band_yaw = yaw_rate[indices]
    band_steer = steer[indices]
    band_vy_dot = vy_dot[indices]
    band_yaw_accel = yaw_accel[indices]

    lateral_features = np.column_stack(
        (
            -band_vy / band_vx,
            band_steer,
            np.ones(len(indices)),
        )
    )
    lateral_target = band_vy_dot + band_vx * band_yaw
    lateral_coefficients, *_ = np.linalg.lstsq(
        lateral_features,
        lateral_target,
        rcond=None,
    )
    lateral_prediction = lateral_features @ lateral_coefficients
    lateral_r2 = 1.0 - float(
        np.sum((lateral_prediction - lateral_target) ** 2)
        / max(
            np.sum(
                (lateral_target - float(np.mean(lateral_target))) ** 2
            ),
            1e-12,
        )
    )

    yaw_features = np.column_stack(
        (
            band_vy / band_vx,
            band_yaw / band_vx,
            band_steer,
            np.ones(len(indices)),
        )
    )
    yaw_coefficients, *_ = np.linalg.lstsq(
        yaw_features,
        band_yaw_accel,
        rcond=None,
    )
    yaw_prediction = yaw_features @ yaw_coefficients
    yaw_r2 = 1.0 - float(
        np.sum((yaw_prediction - band_yaw_accel) ** 2)
        / max(
            np.sum(
                (
                    band_yaw_accel
                    - float(np.mean(band_yaw_accel))
                )
                ** 2
            ),
            1e-12,
        )
    )

    c_lateral = float(lateral_coefficients[0])
    steer_lateral = float(lateral_coefficients[1])
    yaw_velocity = float(yaw_coefficients[0])
    yaw_damping = float(yaw_coefficients[1])
    steer_yaw = float(yaw_coefficients[2])
    mean_vx = float(np.mean(band_vx))
    state_matrix = np.asarray(
        (
            (-c_lateral / mean_vx, -mean_vx),
            (yaw_velocity / mean_vx, yaw_damping / mean_vx),
        )
    )
    input_vector = np.asarray((steer_lateral, steer_yaw))
    steady_state = np.linalg.solve(state_matrix, -input_vector)
    eigenvalues = np.linalg.eigvals(state_matrix)
    return {
        "lower_kmh": lower_kmh,
        "upper_kmh": upper_kmh,
        "speed_kmh": float(
            np.mean(speed_kmh[np.where(valid)[0]])
        ),
        "samples": float(len(indices)),
        "lateral_r2": lateral_r2,
        "yaw_r2": yaw_r2,
        "c_lateral": c_lateral,
        "steer_lateral": steer_lateral,
        "yaw_velocity": yaw_velocity,
        "yaw_damping": yaw_damping,
        "steer_yaw": steer_yaw,
        "steady_lateral_velocity_per_steer": float(steady_state[0]),
        "steady_yaw_rate_per_steer": float(steady_state[1]),
        "continuous_eigenvalues": [
            [float(value.real), float(value.imag)]
            for value in eigenvalues
        ],
        "mean_abs_lateral_g": float(np.mean(lateral_g[indices])),
        "p95_abs_lateral_g": float(np.percentile(lateral_g[indices], 95)),
    }


def interpolate_band_values(
    fits: list[dict[str, float]],
    key: str,
) -> np.ndarray:
    band_speeds = np.asarray(
        [fit["speed_kmh"] for fit in fits],
        dtype=float,
    )
    band_values = np.asarray(
        [fit[key] for fit in fits],
        dtype=float,
    )
    return np.interp(
        MAP_SPEEDS_KMH,
        band_speeds,
        band_values,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument(
        "--base-model",
        type=Path,
        default=(
            Path(__file__).resolve().parents[1]
            / "logs"
            / "lmpc_round2_identified_model_v6.json"
        ),
    )
    parser.add_argument("--identification-seconds", type=float, default=600.0)
    parser.add_argument("--max-lateral-g", type=float, default=0.80)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            Path(__file__).resolve().parents[1]
            / "logs"
            / "lmpc_calibrated_model_v10.json"
        ),
    )
    args = parser.parse_args()

    data = load_arrays(args.log)
    elapsed = data["timestamp"] - data["timestamp"][0]
    time_mask = elapsed < args.identification_seconds
    fits = [
        fit_band(
            data,
            time_mask,
            lower,
            upper,
            args.max_lateral_g,
        )
        for lower, upper in BANDS
    ]

    base = json.loads(
        args.base_model.read_text(encoding="utf-8")
    )
    model = dict(base["model"])
    model["direct_model_map_kmh"] = MAP_SPEEDS_KMH.tolist()
    model["lateral_velocity_damping_map"] = interpolate_band_values(
        fits,
        "c_lateral",
    ).tolist()
    model["lateral_yaw_coupling_map"] = [0.0] * len(MAP_SPEEDS_KMH)
    model["yaw_velocity_coupling_map"] = interpolate_band_values(
        fits,
        "yaw_velocity",
    ).tolist()
    model["yaw_rate_damping_map"] = interpolate_band_values(
        fits,
        "yaw_damping",
    ).tolist()
    model["steer_lateral_gain_map_kmh"] = MAP_SPEEDS_KMH.tolist()
    model["steer_lateral_gain_map"] = interpolate_band_values(
        fits,
        "steer_lateral",
    ).tolist()
    model["steer_yaw_gain_map_kmh"] = MAP_SPEEDS_KMH.tolist()
    model["steer_yaw_gain_map"] = interpolate_band_values(
        fits,
        "steer_yaw",
    ).tolist()

    output = {
        "model": model,
        "identification": {
            "source": str(args.log),
            "identification_seconds": args.identification_seconds,
            "max_lateral_g": args.max_lateral_g,
            "bands": fits,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )
    print(json.dumps(output["identification"], indent=2))
    print(f"model={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
