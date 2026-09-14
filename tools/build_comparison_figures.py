from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from tools.figure_theme import THEME, apply_dark_theme


PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIGURE_DIR = PROJECT_ROOT / "assets" / "figures"
DATA_DIR = PROJECT_ROOT / "assets" / "data"

COLORS = {
    **THEME,
    "human": THEME["blue"],
    "auto": THEME["orange"],
}


def find_manual_fastest_lap(
    frame: pd.DataFrame,
    minimum_duration_s: float,
    maximum_duration_s: float,
) -> tuple[float, pd.DataFrame]:
    previous = frame["normalized_position"].shift(1)
    crossings = np.flatnonzero(
        (
            (previous > 0.75)
            & (frame["normalized_position"] < 0.25)
        ).to_numpy()
    )
    candidates: list[tuple[float, pd.DataFrame]] = []
    for start, end in zip(crossings[:-1], crossings[1:]):
        lap = frame.iloc[start : end + 1].copy()
        duration = float(
            lap["timestamp"].iloc[-1] - lap["timestamp"].iloc[0]
        )
        if minimum_duration_s <= duration <= maximum_duration_s:
            candidates.append((duration, lap))
    if not candidates:
        raise RuntimeError("No complete manual lap found")
    return min(candidates, key=lambda item: item[0])


def find_auto_fastest_lap(
    frame: pd.DataFrame,
    minimum_duration_s: float,
    maximum_duration_s: float,
) -> tuple[float, pd.DataFrame]:
    laps: list[tuple[float, pd.DataFrame]] = []
    for _, lap in frame.groupby("lap_number", sort=True):
        duration = float(
            lap["timestamp"].iloc[-1] - lap["timestamp"].iloc[0]
        )
        start_position = float(lap["normalized_position"].iloc[0])
        end_position = float(lap["normalized_position"].iloc[-1])
        complete_loop = (
            (start_position > 0.75 and end_position < 0.25)
            or (start_position < 0.25 and end_position < 0.25)
        )
        if (
            complete_loop
            and minimum_duration_s <= duration <= maximum_duration_s
        ):
            laps.append((duration, lap.copy()))
    if not laps:
        raise RuntimeError("No complete automatic lap found")
    return min(laps, key=lambda item: item[0])


def binned_values(
    frame: pd.DataFrame,
    *,
    track_length_m: float,
    yaw_column: str,
    lateral_column: str,
    bin_m: float,
) -> np.ndarray:
    distance = (
        frame["normalized_position"].to_numpy(dtype=float) * track_length_m
    )
    edges = np.arange(0.0, track_length_m + bin_m, bin_m)
    rows: list[tuple[float, float, float, float]] = []
    for lower, upper in zip(edges[:-1], edges[1:]):
        mask = (distance >= lower) & (distance < upper)
        if not np.any(mask):
            continue
        rows.append(
            (
                0.5 * (lower + upper),
                float(np.median(frame.loc[mask, "speed_kmh"])),
                float(np.median(np.abs(frame.loc[mask, yaw_column]))),
                float(np.median(np.abs(frame.loc[mask, lateral_column]))),
            )
        )
    return np.asarray(rows, dtype=float)


def build_comparison(
    *,
    label: str,
    human_path: Path,
    auto_path: Path,
    track_length_m: float,
    output_path: Path,
    metrics_path: Path,
    bin_m: float,
) -> dict[str, object]:
    human = pd.read_csv(human_path).sort_values("timestamp")
    auto = pd.read_csv(auto_path).sort_values("timestamp")

    human_time, human_lap = find_manual_fastest_lap(
        human,
        minimum_duration_s=60.0,
        maximum_duration_s=300.0,
    )
    auto_time, auto_lap = find_auto_fastest_lap(
        auto,
        minimum_duration_s=60.0,
        maximum_duration_s=700.0,
    )

    human_values = binned_values(
        human_lap,
        track_length_m=track_length_m,
        yaw_column="yaw_rate_rad_s",
        lateral_column="lateral_accel_g",
        bin_m=bin_m,
    )
    auto_values = binned_values(
        auto_lap,
        track_length_m=track_length_m,
        yaw_column="yaw_rate_rad_s",
        lateral_column="acceleration_g_x",
        bin_m=bin_m,
    )

    grid = np.arange(0.0, track_length_m, bin_m)
    human_speed = np.interp(
        grid,
        human_values[:, 0],
        human_values[:, 1],
        left=human_values[0, 1],
        right=human_values[-1, 1],
    )
    auto_speed = np.interp(
        grid,
        auto_values[:, 0],
        auto_values[:, 1],
        left=auto_values[0, 1],
        right=auto_values[-1, 1],
    )
    human_yaw = np.interp(
        grid,
        human_values[:, 0],
        human_values[:, 2],
        left=human_values[0, 2],
        right=human_values[-1, 2],
    )
    auto_yaw = np.interp(
        grid,
        auto_values[:, 0],
        auto_values[:, 2],
        left=auto_values[0, 2],
        right=auto_values[-1, 2],
    )
    human_lateral = np.interp(
        grid,
        human_values[:, 0],
        human_values[:, 3],
        left=human_values[0, 3],
        right=human_values[-1, 3],
    )
    auto_lateral = np.interp(
        grid,
        auto_values[:, 0],
        auto_values[:, 3],
        left=auto_values[0, 3],
        right=auto_values[-1, 3],
    )

    fig, axes = plt.subplots(
        3,
        1,
        figsize=(16, 11),
        sharex=True,
        facecolor=COLORS["paper"],
    )
    for ax in axes:
        ax.set_facecolor(COLORS["white"])
        ax.grid(color=COLORS["grid"], alpha=0.65)

    axes[0].plot(
        grid / 1000.0,
        human_speed,
        color=COLORS["human"],
        linewidth=1.7,
        label=f"Human {human_time:.3f} s",
    )
    axes[0].plot(
        grid / 1000.0,
        auto_speed,
        color=COLORS["auto"],
        linewidth=1.7,
        label=f"Auto {auto_time:.3f} s",
    )
    axes[0].set_ylabel("speed [km/h]")
    axes[0].legend(loc="best")

    axes[1].plot(
        grid / 1000.0,
        human_yaw,
        color=COLORS["human"],
        linewidth=1.4,
        label="Human |yaw rate|",
    )
    axes[1].plot(
        grid / 1000.0,
        auto_yaw,
        color=COLORS["auto"],
        linewidth=1.4,
        label="Auto |yaw rate|",
    )
    axes[1].set_ylabel("|yaw rate| [rad/s]")
    axes[1].legend(loc="best")

    axes[2].plot(
        grid / 1000.0,
        human_lateral,
        color=COLORS["human"],
        linewidth=1.4,
        label="Human |Ay|",
    )
    axes[2].plot(
        grid / 1000.0,
        auto_lateral,
        color=COLORS["auto"],
        linewidth=1.4,
        label="Auto |Ay|",
    )
    axes[2].set_ylabel("|lateral accel| [g]")
    axes[2].set_xlabel("track distance [km]")
    axes[2].legend(loc="best")

    fig.suptitle(
        label,
        fontsize=18,
        fontweight="bold",
        color=COLORS["ink"],
    )
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)

    speed_delta = auto_speed - human_speed
    metrics: dict[str, object] = {
        "track": label,
        "human_lap_s": human_time,
        "auto_lap_s": auto_time,
        "gap_s": auto_time - human_time,
        "human_max_speed_kmh": float(human_lap["speed_kmh"].max()),
        "auto_max_speed_kmh": float(auto_lap["speed_kmh"].max()),
        "auto_p95_abs_lateral_error_m": float(
            np.percentile(np.abs(auto_lap["lateral_error_m"]), 95)
        ),
        "auto_max_tyres_out": int(auto_lap["tyres_out"].max()),
        "bin_m": bin_m,
        "auto_speed_delta_mean_kmh": float(np.mean(speed_delta)),
        "slowest_auto_bins": [
            {
                "distance_m": float(grid[index]),
                "delta_kmh": float(speed_delta[index]),
                "human_speed_kmh": float(human_speed[index]),
                "auto_speed_kmh": float(auto_speed[index]),
            }
            for index in np.argsort(speed_delta)[:12]
        ],
        "fastest_auto_bins_vs_human": [
            {
                "distance_m": float(grid[index]),
                "delta_kmh": float(speed_delta[index]),
                "human_speed_kmh": float(human_speed[index]),
                "auto_speed_kmh": float(auto_speed[index]),
            }
            for index in np.argsort(speed_delta)[-8:]
        ],
    }
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    metrics_path.write_text(
        json.dumps(metrics, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )
    return metrics


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bin-m", type=float, default=25.0)
    args = parser.parse_args()

    apply_dark_theme()

    track_specs = (
        {
            "label": "Shanghai: Human vs Auto",
            "human_path": (
                PROJECT_ROOT
                / "logs"
                / "vehicle_dynamics_reference_20min.csv"
            ),
            "auto_path": (
                PROJECT_ROOT
                / "logs"
                / "lmpc_shanghai_v63_h50dt05_265_5lap.csv"
            ),
            "track_length_m": 5426.853590742263,
            "figure": "comparison_shanghai.png",
            "metrics": "comparison_shanghai_metrics.json",
        },
        {
            "label": "Zhejiang: Human vs Auto",
            "human_path": (
                PROJECT_ROOT
                / "logs"
                / "manual_reference_v49_moza.csv"
            ),
            "auto_path": (
                PROJECT_ROOT
                / "logs"
                / "lmpc_v59_rollback_5lap.csv"
            ),
            "track_length_m": 3114.099017702242,
            "figure": "comparison_zhejiang.png",
            "metrics": "comparison_zhejiang_metrics.json",
        },
    )

    for spec in track_specs:
        metrics = build_comparison(
            label=str(spec["label"]),
            human_path=Path(spec["human_path"]),
            auto_path=Path(spec["auto_path"]),
            track_length_m=float(spec["track_length_m"]),
            output_path=FIGURE_DIR / str(spec["figure"]),
            metrics_path=DATA_DIR / str(spec["metrics"]),
            bin_m=args.bin_m,
        )
        print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
