from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from actc.shared_memory import SharedMemoryReader  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Record steering calibration telemetry without sending any input. "
            "Use this mode when driving with a physical wheel."
        )
    )
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--print-period", type=float, default=2.0)
    parser.add_argument(
        "--log",
        type=Path,
        default=(
            PROJECT_ROOT
            / "logs"
            / "manual_steering_calibration.csv"
        ),
    )
    parser.add_argument("--hz", type=float, default=50.0)
    args = parser.parse_args()

    reader = SharedMemoryReader()
    reader.connect(wait_seconds=10.0)
    state = reader.snapshot()
    print(
        f"Connected: car={state.car_model!r} track={state.track!r}"
    )
    print(
        "Manual input mode: this process does not send steering, "
        "throttle, or brake."
    )

    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open("w", newline="", encoding="utf-8") as handle:
        fields = [
            "timestamp",
            "packet_id",
            "speed_kmh",
            "steer",
            "gas",
            "brake",
            "gear",
            "rpm",
            "auto_shifter",
            "heading_rad",
            "yaw_rate_rad_s",
            "local_angular_velocity_x",
            "local_angular_velocity_y",
            "local_angular_velocity_z",
            "lateral_accel_g",
            "longitudinal_accel_g",
            "local_velocity_x",
            "local_velocity_y",
            "local_velocity_z",
            "world_velocity_x",
            "world_velocity_y",
            "world_velocity_z",
            "position_x",
            "position_y",
            "position_z",
            "tyres_out",
            "wheel_slip_abs_max",
            "wheel_slip_fl",
            "wheel_slip_fr",
            "wheel_slip_rl",
            "wheel_slip_rr",
            "wheel_load_fl",
            "wheel_load_fr",
            "wheel_load_rl",
            "wheel_load_rr",
            "surface_grip",
            "normalized_position",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        started = time.monotonic()
        next_print = started
        period = 1.0 / max(1.0, args.hz)
        try:
            while time.monotonic() - started < args.duration:
                state = reader.snapshot()
                writer.writerow(
                    {
                        "timestamp": f"{state.timestamp:.6f}",
                        "packet_id": state.packet_id,
                        "speed_kmh": f"{state.speed_kmh:.6f}",
                        "steer": f"{state.steer:.9f}",
                        "gas": f"{state.gas:.6f}",
                        "brake": f"{state.brake:.6f}",
                        "gear": state.gear,
                        "rpm": f"{state.rpm:.3f}",
                        "auto_shifter": int(state.auto_shifter_on),
                        "heading_rad": f"{state.heading_rad:.9f}",
                        "yaw_rate_rad_s": (
                            f"{state.yaw_rate_rad_s:.9f}"
                        ),
                        "local_angular_velocity_x": (
                            f"{state.local_angular_velocity_rad_s[0]:.9f}"
                        ),
                        "local_angular_velocity_y": (
                            f"{state.local_angular_velocity_rad_s[1]:.9f}"
                        ),
                        "local_angular_velocity_z": (
                            f"{state.local_angular_velocity_rad_s[2]:.9f}"
                        ),
                        "lateral_accel_g": (
                            f"{state.acceleration_g[0]:.9f}"
                        ),
                        "longitudinal_accel_g": (
                            f"{state.acceleration_g[2]:.9f}"
                        ),
                        "local_velocity_x": (
                            f"{state.local_velocity_m_s[0]:.9f}"
                        ),
                        "local_velocity_y": (
                            f"{state.local_velocity_m_s[1]:.9f}"
                        ),
                        "local_velocity_z": (
                            f"{state.local_velocity_m_s[2]:.9f}"
                        ),
                        "world_velocity_x": (
                            f"{state.velocity_m_s[0]:.9f}"
                        ),
                        "world_velocity_y": (
                            f"{state.velocity_m_s[1]:.9f}"
                        ),
                        "world_velocity_z": (
                            f"{state.velocity_m_s[2]:.9f}"
                        ),
                        "position_x": f"{state.position[0]:.9f}",
                        "position_y": f"{state.position[1]:.9f}",
                        "position_z": f"{state.position[2]:.9f}",
                        "tyres_out": state.tyres_out,
                        "wheel_slip_abs_max": (
                            f"{max(abs(value) for value in state.tyre_slip):.9f}"
                        ),
                        "wheel_slip_fl": f"{state.tyre_slip[0]:.9f}",
                        "wheel_slip_fr": f"{state.tyre_slip[1]:.9f}",
                        "wheel_slip_rl": f"{state.tyre_slip[2]:.9f}",
                        "wheel_slip_rr": f"{state.tyre_slip[3]:.9f}",
                        "wheel_load_fl": f"{state.wheel_load_n[0]:.9f}",
                        "wheel_load_fr": f"{state.wheel_load_n[1]:.9f}",
                        "wheel_load_rl": f"{state.wheel_load_n[2]:.9f}",
                        "wheel_load_rr": f"{state.wheel_load_n[3]:.9f}",
                        "surface_grip": f"{state.surface_grip:.6f}",
                        "normalized_position": (
                            f"{state.normalized_position:.9f}"
                        ),
                    }
                )
                now = time.monotonic()
                if now >= next_print:
                    print(
                        f"t={now - started:6.1f}s "
                        f"speed={state.speed_kmh:6.1f} "
                        f"steer={state.steer:+.3f} "
                        f"yaw={state.yaw_rate_rad_s:+.3f} "
                        f"ay={state.acceleration_g[0]:+.3f}g",
                        flush=True,
                    )
                    next_print = now + args.print_period
                time.sleep(period)
        except KeyboardInterrupt:
            pass

    print(f"Manual calibration log: {args.log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
