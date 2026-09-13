from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


def as_float(row: dict[str, str], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return 0.0


def summarize_segment(
    lap_number: int,
    segment: int,
    rows: list[dict[str, str]],
) -> dict[str, float | int]:
    values = lambda key: [as_float(row, key) for row in rows]  # noqa: E731
    lateral = values("lateral_error_m")
    speed = values("speed_kmh")
    timestamps = values("timestamp")
    return {
        "lap_number": lap_number,
        "segment": segment,
        "segment_start_s": segment / 20.0,
        "segment_end_s": (segment + 1) / 20.0,
        "samples": len(rows),
        "duration_s": max(timestamps) - min(timestamps),
        "path_index_start": int(as_float(rows[0], "path_index")),
        "path_index_end": int(as_float(rows[-1], "path_index")),
        "speed_min_kmh": min(speed),
        "speed_max_kmh": max(speed),
        "speed_mean_kmh": statistics.mean(speed),
        "target_mean_kmh": statistics.mean(
            values("target_speed_kmh")
        ),
        "max_lateral_error_m": max(abs(value) for value in lateral),
        "rms_lateral_error_m": math.sqrt(
            sum(value * value for value in lateral) / max(1, len(lateral))
        ),
        "max_heading_error_rad": max(
            abs(value) for value in values("heading_error_rad")
        ),
        "max_tyres_out": max(
            int(as_float(row, "tyres_out")) for row in rows
        ),
        "brake_mean": statistics.mean(values("cmd_brake")),
        "brake_max": max(values("cmd_brake")),
        "throttle_mean": statistics.mean(values("cmd_throttle")),
        "steer_abs_mean": statistics.mean(
            abs(value) for value in values("cmd_steer")
        ),
        "steer_abs_max": max(
            abs(value) for value in values("cmd_steer")
        ),
        "lateral_accel_abs_max_g": max(
            abs(value) for value in values("acceleration_g_x")
        ),
        "vertical_accel_abs_max_g": max(
            abs(value) for value in values("acceleration_g_y")
        ),
        "longitudinal_accel_abs_max_g": max(
            abs(value) for value in values("acceleration_g_z")
        ),
        "wheel_slip_abs_max": max(
            max(
                abs(as_float(row, key))
                for key in (
                    "wheel_slip_fl",
                    "wheel_slip_fr",
                    "wheel_slip_rl",
                    "wheel_slip_rr",
                )
            )
            for row in rows
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--bins", type=int, default=20)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    if args.bins <= 0:
        raise ValueError("--bins must be positive")
    with args.log.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"No samples in {args.log}")

    grouped: dict[
        tuple[int, int],
        list[dict[str, str]],
    ] = defaultdict(list)
    for row in rows:
        lap_number = int(as_float(row, "lap_number"))
        normalized = max(
            0.0,
            min(0.999999, as_float(row, "normalized_position")),
        )
        segment = min(args.bins - 1, int(normalized * args.bins))
        grouped[(lap_number, segment)].append(row)

    output = args.output or args.log.with_name(
        f"{args.log.stem}_segments.jsonl"
    )
    with output.open("w", encoding="utf-8") as handle:
        for (lap_number, segment), samples in sorted(grouped.items()):
            summary = summarize_segment(
                lap_number,
                segment,
                samples,
            )
            handle.write(json.dumps(summary, ensure_ascii=True) + "\n")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
