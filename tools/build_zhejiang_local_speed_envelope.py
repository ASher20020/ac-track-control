from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from actc.track import TrackPath, resolve_fast_lane


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _window_weight(
    position: float,
    *,
    start: float,
    plateau_start: float,
    plateau_end: float,
    end: float,
) -> float:
    if position <= start or position >= end:
        return 0.0
    if plateau_start <= position <= plateau_end:
        return 1.0
    if position < plateau_start:
        phase = (position - start) / max(
            plateau_start - start,
            1e-9,
        )
    else:
        phase = (end - position) / max(
            end - plateau_end,
            1e-9,
        )
    return 0.5 - 0.5 * math.cos(math.pi * phase)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            PROJECT_ROOT
            / "configs"
            / "speed_envelopes"
            / "zhejiang_local_055.json"
        ),
    )
    parser.add_argument("--start", type=float, default=0.470)
    parser.add_argument("--plateau-start", type=float, default=0.495)
    parser.add_argument("--plateau-end", type=float, default=0.520)
    parser.add_argument("--end", type=float, default=0.545)
    parser.add_argument("--reduction-kmh", type=float, default=6.0)
    args = parser.parse_args()

    path = TrackPath.load(
        resolve_fast_lane("st_zhejiang", ""),
        centerline=True,
        preserve_fast_lane=True,
        max_speed_kmh=280.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=8.3385,
        accel_mps2=7.0,
        brake_mps2=52.0,
    ).resampled(2.0)
    path = path.with_minimum_edge_clearance(
        0.8,
        max_speed_kmh=280.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=8.3385,
        accel_mps2=7.0,
        brake_mps2=52.0,
    )

    speeds: list[float] = []
    for index, speed in enumerate(path.speed_limit_kmh):
        position = (
            path.cumulative_s[index] / path.total_length_m
        )
        weight = _window_weight(
            position,
            start=args.start,
            plateau_start=args.plateau_start,
            plateau_end=args.plateau_end,
            end=args.end,
        )
        speeds.append(
            max(
                25.0,
                float(speed) - args.reduction_kmh * weight,
            )
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(
            {
                "track": "st_zhejiang",
                "start": args.start,
                "plateau_start": args.plateau_start,
                "plateau_end": args.plateau_end,
                "end": args.end,
                "reduction_kmh": args.reduction_kmh,
                "speed_kmh": speeds,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(
        f"Wrote {args.output} with {len(speeds)} speed points"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
