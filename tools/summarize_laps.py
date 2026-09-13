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


def summarize_lap(
    lap_number: int,
    rows: list[dict[str, str]],
) -> dict[str, float | int]:
    steer = [as_float(row, "cmd_steer") for row in rows]
    speed = [as_float(row, "speed_kmh") for row in rows]
    target = [as_float(row, "target_speed_kmh") for row in rows]
    lateral = [abs(as_float(row, "lateral_error_m")) for row in rows]
    heading = [abs(as_float(row, "heading_error_rad")) for row in rows]
    timestamps = [as_float(row, "timestamp") for row in rows]
    steer_delta = [
        abs(steer[index] - steer[index - 1])
        for index in range(1, len(steer))
    ]
    return {
        "lap_number": lap_number,
        "samples": len(rows),
        "duration_s": max(timestamps) - min(timestamps),
        "speed_min_kmh": min(speed),
        "speed_max_kmh": max(speed),
        "speed_mean_kmh": statistics.mean(speed),
        "target_max_kmh": max(target),
        "max_lateral_error_m": max(lateral),
        "rms_lateral_error_m": math.sqrt(
            sum(value * value for value in lateral) / max(1, len(lateral))
        ),
        "max_heading_error_rad": max(heading),
        "max_tyres_out": max(
            int(as_float(row, "tyres_out"))
            for row in rows
        ),
        "steering_activity": (
            statistics.mean(steer_delta) if steer_delta else 0.0
        ),
        "start_s": as_float(rows[0], "normalized_position"),
        "end_s": as_float(rows[-1], "normalized_position"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--include-partial", action="store_true")
    args = parser.parse_args()

    with args.log.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"No samples in {args.log}")

    grouped: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[int(as_float(row, "lap_number"))].append(row)

    lap_numbers = sorted(grouped)
    if not args.include_partial and lap_numbers:
        lap_numbers = lap_numbers[:-1]

    output = args.output or args.log.with_name(
        f"{args.log.stem}_summary.jsonl"
    )
    with output.open("w", encoding="utf-8") as handle:
        for lap_number in lap_numbers:
            summary = summarize_lap(lap_number, grouped[lap_number])
            handle.write(json.dumps(summary, ensure_ascii=True) + "\n")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
