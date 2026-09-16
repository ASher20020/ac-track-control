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
        resolve_fast_lane("ks_nordschleife", ""),
        centerline=True,
        preserve_fast_lane=True,
        max_speed_kmh=300.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=7.2593,
        accel_mps2=7.0,
        brake_mps2=15.3,
    ).resampled(2.0)
    path = path.with_minimum_edge_clearance(
        0.8,
        max_speed_kmh=300.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=7.2593,
        accel_mps2=7.0,
        brake_mps2=15.3,
    )

    line_window = (0.742, 0.750, 0.758, 0.768, -0.7)
    offsets: list[float] = []
    for index in range(len(path.points)):
        position = (
            path.cumulative_s[index] / path.total_length_m
        )
        offsets.append(
            line_window[4]
            * _weight(
                position,
                start=line_window[0],
                plateau_start=line_window[1],
                plateau_end=line_window[2],
                end=line_window[3],
            )
        )

    line_offset_path = (
        PROJECT_ROOT
        / "configs"
        / "line_offsets"
        / "nordschleife_candidate_v2.json"
    )
    line_offset_path.parent.mkdir(parents=True, exist_ok=True)
    line_offset_path.write_text(
        json.dumps(
            {
                "track": "ks_nordschleife",
                "offset_m": offsets,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    path = path.with_line_offsets(
        offsets,
        base_margin_m=0.8,
        max_edge_margin_m=0.8,
        max_speed_kmh=300.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=7.2593,
        accel_mps2=7.0,
        brake_mps2=15.3,
    )

    speed_windows = (
        (0.742, 0.750, 0.758, 0.768, 8.0),
        (0.904, 0.910, 0.930, 0.940, 10.0),
    )
    speeds: list[float] = []
    for index, speed in enumerate(path.speed_limit_kmh):
        position = (
            path.cumulative_s[index] / path.total_length_m
        )
        reduction = sum(
            magnitude
            * _weight(
                position,
                start=start,
                plateau_start=plateau_start,
                plateau_end=plateau_end,
                end=end,
            )
            for start, plateau_start, plateau_end, end, magnitude
            in speed_windows
        )
        speeds.append(max(25.0, float(speed) - reduction))

    speed_envelope_path = (
        PROJECT_ROOT
        / "configs"
        / "speed_envelopes"
        / "nordschleife_candidate_v2.json"
    )
    speed_envelope_path.parent.mkdir(parents=True, exist_ok=True)
    speed_envelope_path.write_text(
        json.dumps(
            {
                "track": "ks_nordschleife",
                "speed_kmh": speeds,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(
        f"Wrote {line_offset_path} and {speed_envelope_path} "
        f"with {len(offsets)} points"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
