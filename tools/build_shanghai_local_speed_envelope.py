from __future__ import annotations

import json
import math
from pathlib import Path

from actc.track import TrackPath, resolve_fast_lane


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _weight(
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
    path = TrackPath.load(
        resolve_fast_lane("tr_shanghai", ""),
        centerline=True,
        preserve_fast_lane=True,
        max_speed_kmh=300.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=8.3385,
        accel_mps2=7.0,
        brake_mps2=52.0,
    ).resampled(2.0)
    path = path.with_minimum_edge_clearance(
        0.8,
        max_speed_kmh=300.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=8.3385,
        accel_mps2=7.0,
        brake_mps2=52.0,
    )
    offset_path = (
        PROJECT_ROOT
        / "configs"
        / "line_offsets"
        / "shanghai_local_two_corners.json"
    )
    offsets = json.loads(
        offset_path.read_text(encoding="utf-8")
    )["offset_m"]
    path = path.with_line_offsets(
        [float(value) for value in offsets],
        base_margin_m=0.8,
        max_edge_margin_m=0.8,
        max_speed_kmh=300.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=8.3385,
        accel_mps2=7.0,
        brake_mps2=52.0,
    )

    start = 0.405
    plateau_start = 0.415
    plateau_end = 0.440
    end = 0.455
    reduction_kmh = 4.0
    speeds: list[float] = []
    for index, speed in enumerate(path.speed_limit_kmh):
        position = (
            path.cumulative_s[index] / path.total_length_m
        )
        weight = _weight(
            position,
            start=start,
            plateau_start=plateau_start,
            plateau_end=plateau_end,
            end=end,
        )
        speeds.append(
            max(
                25.0,
                float(speed) - reduction_kmh * weight,
            )
        )

    output = (
        PROJECT_ROOT
        / "configs"
        / "speed_envelopes"
        / "shanghai_local_60s.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {
                "track": "tr_shanghai",
                "start": start,
                "plateau_start": plateau_start,
                "plateau_end": plateau_end,
                "end": end,
                "reduction_kmh": reduction_kmh,
                "speed_kmh": speeds,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"Wrote {output} with {len(speeds)} speed points")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
