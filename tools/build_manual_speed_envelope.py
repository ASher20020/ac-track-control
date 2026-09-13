from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from actc.track import TrackPath, resolve_fast_lane  # noqa: E402


def _fastest_lap(
    frame: pd.DataFrame,
    *,
    minimum_duration_s: float,
    maximum_duration_s: float,
) -> tuple[float, pd.DataFrame]:
    previous = frame["normalized_position"].shift(1)
    crossings = np.flatnonzero(
        (
            (previous > 0.8)
            & (frame["normalized_position"] < 0.2)
        ).to_numpy()
    )
    bounds = np.concatenate([[0], crossings, [len(frame) - 1]])
    candidates: list[tuple[float, pd.DataFrame]] = []
    for start, end in zip(bounds[:-1], bounds[1:]):
        lap = frame.iloc[start : end + 1].copy()
        duration = float(
            lap["timestamp"].iloc[-1] - lap["timestamp"].iloc[0]
        )
        if minimum_duration_s <= duration <= maximum_duration_s:
            candidates.append((duration, lap))
    if not candidates:
        raise RuntimeError("No complete manual lap found")
    return min(candidates, key=lambda item: item[0])


def _periodic_smooth(values: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return values.copy()
    kernel = np.ones(2 * radius + 1) / (2 * radius + 1)
    return np.convolve(
        np.concatenate([values[-radius:], values, values[:radius]]),
        kernel,
        mode="valid",
    )


def _windowed_boost(
    values: np.ndarray,
    *,
    start_index: int,
    end_index: int,
    factor: float,
    transition_indices: int,
) -> np.ndarray:
    count = len(values)
    start_index = max(0, min(start_index, count - 1))
    end_index = max(start_index + 1, min(end_index, count))
    transition = max(1, transition_indices)
    boosted = values.copy()
    for index in range(count):
        if start_index <= index < end_index:
            weight = 1.0
        elif start_index - transition <= index < start_index:
            phase = (index - (start_index - transition)) / transition
            weight = 0.5 - 0.5 * np.cos(np.pi * phase)
        elif end_index <= index < end_index + transition:
            phase = (index - end_index) / transition
            weight = 0.5 + 0.5 * np.cos(np.pi * phase)
        else:
            weight = 0.0
        boosted[index] *= 1.0 + (factor - 1.0) * weight
    return boosted


def build_envelope(
    *,
    manual_log: Path,
    fast_lane: Path,
    output: Path,
    smoothing_radius: int,
    boost_start_index: int | None,
    boost_end_index: int | None,
    boost_factor: float,
    boost_transition_indices: int,
) -> dict:
    frame = pd.read_csv(
        manual_log,
        usecols=[
            "timestamp",
            "speed_kmh",
            "normalized_position",
            "position_x",
            "position_z",
        ],
    ).sort_values("timestamp").reset_index(drop=True)
    lap_time_s, lap = _fastest_lap(
        frame,
        minimum_duration_s=120.0,
        maximum_duration_s=160.0,
    )
    path = TrackPath.load(
        fast_lane,
        centerline=True,
        preserve_fast_lane=True,
        max_speed_kmh=280.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=6.867,
        accel_mps2=3.924,
        brake_mps2=14.0,
    )
    tree = cKDTree(np.asarray(path.points, dtype=float))
    _, indices = tree.query(
        lap[["position_x", "position_z"]].to_numpy(dtype=float)
    )
    speeds = lap["speed_kmh"].to_numpy(dtype=float)
    envelope = np.zeros(len(path.points), dtype=float)
    for index in range(len(path.points)):
        samples = speeds[indices == index]
        envelope[index] = (
            float(np.median(samples))
            if samples.size
            else np.nan
        )
    missing = ~np.isfinite(envelope)
    if missing.any():
        known = np.flatnonzero(~missing)
        if known.size < 2:
            raise RuntimeError("Manual data do not cover the track")
        envelope[missing] = np.interp(
            np.flatnonzero(missing),
            known,
            envelope[known],
        )
    envelope = _periodic_smooth(envelope, smoothing_radius)
    if (
        boost_start_index is not None
        and boost_end_index is not None
    ):
        envelope = _windowed_boost(
            envelope,
            start_index=boost_start_index,
            end_index=boost_end_index,
            factor=boost_factor,
            transition_indices=boost_transition_indices,
        )
    envelope = np.clip(envelope, 25.0, 280.0)
    payload = {
        "source": str(manual_log),
        "fast_lane": str(fast_lane),
        "lap_time_s": lap_time_s,
        "points": len(path.points),
        "smoothing_radius": smoothing_radius,
        "speed_kmh": [round(float(value), 6) for value in envelope],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--manual-log",
        type=Path,
        default=(
            PROJECT_ROOT
            / "logs"
            / "vehicle_dynamics_reference_20min.csv"
        ),
    )
    parser.add_argument(
        "--fast-lane",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            PROJECT_ROOT
            / "configs"
            / "tr_shanghai_manual_speed_envelope.json"
        ),
    )
    parser.add_argument("--smoothing-radius", type=int, default=6)
    parser.add_argument("--boost-start-index", type=int, default=None)
    parser.add_argument("--boost-end-index", type=int, default=None)
    parser.add_argument("--boost-factor", type=float, default=1.0)
    parser.add_argument(
        "--boost-transition-indices",
        type=int,
        default=60,
    )
    args = parser.parse_args()
    fast_lane = args.fast_lane or resolve_fast_lane(
        "tr_shanghai",
        "advanced",
    )
    payload = build_envelope(
        manual_log=args.manual_log,
        fast_lane=fast_lane,
        output=args.output,
        smoothing_radius=args.smoothing_radius,
        boost_start_index=args.boost_start_index,
        boost_end_index=args.boost_end_index,
        boost_factor=args.boost_factor,
        boost_transition_indices=args.boost_transition_indices,
    )
    print(
        json.dumps(
            {
                "output": str(args.output),
                "lap_time_s": payload["lap_time_s"],
                "points": payload["points"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
