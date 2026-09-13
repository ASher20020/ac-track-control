from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from actc.track import TrackPath, resolve_fast_lane  # noqa: E402


def load_log(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_float(rows: list[dict[str, str]], key: str) -> list[float]:
    return [float(row[key]) for row in rows]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--track", default="tr_shanghai")
    parser.add_argument("--track-config", default="advanced")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    rows = load_log(args.log)
    if not rows:
        raise RuntimeError(f"No samples in {args.log}")

    path = TrackPath.load(
        resolve_fast_lane(args.track, args.track_config),
        centerline=True,
    )
    actual_x = as_float(rows, "position_x")
    actual_z = as_float(rows, "position_z")
    indices = [int(row["path_index"]) for row in rows]
    reference_x = [path.points[index][0] for index in indices]
    reference_z = [path.points[index][1] for index in indices]
    elapsed = [
        float(row["timestamp"]) - float(rows[0]["timestamp"])
        for row in rows
    ]

    output = args.output or args.log.with_suffix(".png")
    fig = plt.figure(figsize=(16, 10))
    grid = fig.add_gridspec(2, 2, height_ratios=(1.45, 1.0))
    track_ax = fig.add_subplot(grid[0, 0])
    zoom_ax = fig.add_subplot(grid[0, 1])
    signal_ax = fig.add_subplot(grid[1, :])

    center_x = [point[0] for point in path.points]
    center_z = [point[1] for point in path.points]
    track_ax.plot(center_x, center_z, color="0.65", linewidth=1.0, label="centerline")
    track_ax.plot(reference_x, reference_z, "--", color="tab:blue", label="reference segment")
    track_ax.plot(actual_x, actual_z, color="tab:red", linewidth=2.0, label="actual")
    track_ax.scatter(actual_x[0], actual_z[0], color="black", s=40, label="start")
    track_ax.scatter(actual_x[-1], actual_z[-1], color="tab:red", s=40, label="end")
    track_ax.set_aspect("equal", adjustable="datalim")
    track_ax.grid(True, alpha=0.25)
    track_ax.set_xlabel("world X [m]")
    track_ax.set_ylabel("world Z [m]")
    track_ax.set_title("Reference vs actual trajectory")
    track_ax.legend()

    zoom_ax.plot(center_x, center_z, color="0.75", linewidth=0.8)
    zoom_ax.plot(reference_x, reference_z, "--", color="tab:blue", label="reference")
    zoom_ax.plot(actual_x, actual_z, color="tab:red", linewidth=3.0, label="actual")
    zoom_ax.scatter(actual_x[0], actual_z[0], color="black", s=40)
    zoom_ax.scatter(actual_x[-1], actual_z[-1], color="tab:red", s=40)
    pad = max(40.0, 0.2 * max(
        max(actual_x) - min(actual_x),
        max(actual_z) - min(actual_z),
    ))
    zoom_ax.set_xlim(min(actual_x) - pad, max(actual_x) + pad)
    zoom_ax.set_ylim(min(actual_z) - pad, max(actual_z) + pad)
    zoom_ax.set_aspect("equal", adjustable="box")
    zoom_ax.grid(True, alpha=0.25)
    zoom_ax.set_xlabel("world X [m]")
    zoom_ax.set_ylabel("world Z [m]")
    zoom_ax.set_title("Zoom around executed segment")
    zoom_ax.legend()

    speed = as_float(rows, "speed_kmh")
    target = as_float(rows, "target_speed_kmh")
    lateral = as_float(rows, "lateral_error_m")
    heading_error = as_float(rows, "heading_error_rad")
    command = as_float(rows, "cmd_steer")

    signal_ax.plot(elapsed, speed, color="tab:red", label="speed")
    signal_ax.plot(elapsed, target, color="tab:blue", label="target speed")
    signal_ax.set_ylabel("speed [km/h]")
    signal_ax.grid(True, alpha=0.25)

    error_ax = signal_ax.twinx()
    error_ax.plot(elapsed, lateral, color="tab:orange", label="lateral error")
    error_ax.plot(elapsed, heading_error, color="tab:green", label="heading error")
    error_ax.plot(elapsed, command, color="tab:purple", label="steer command")
    error_ax.set_ylabel("error / normalized steer")

    lines = signal_ax.get_lines() + error_ax.get_lines()
    signal_ax.legend(lines, [line.get_label() for line in lines], loc="best")
    signal_ax.set_xlabel("elapsed [s]")
    signal_ax.set_title("Longitudinal and lateral diagnostics")

    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
