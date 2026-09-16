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

    windows = (
        (0.185, 0.194, 0.205, 0.215, -0.8),
        (0.400, 0.410, 0.440, 0.455, -0.9),
    )
    offsets: list[float] = []
    for index in range(len(path.points)):
        position = (
            path.cumulative_s[index] / path.total_length_m
        )
        offset = 0.0
        for start, plateau_start, plateau_end, end, magnitude in windows:
            offset += magnitude * _weight(
                position,
                start=start,
                plateau_start=plateau_start,
                plateau_end=plateau_end,
                end=end,
            )
        offsets.append(offset)

    output = (
        PROJECT_ROOT
        / "configs"
        / "line_offsets"
        / "shanghai_local_two_corners.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {
                "track": "tr_shanghai",
                "windows": [
                    {
                        "start": window[0],
                        "plateau_start": window[1],
                        "plateau_end": window[2],
                        "end": window[3],
                        "offset_m": window[4],
                    }
                    for window in windows
                ],
                "offset_m": offsets,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"Wrote {output} with {len(offsets)} offset points")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
