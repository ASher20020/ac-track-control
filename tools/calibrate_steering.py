from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from actc.controllers import SpeedController, SpeedControllerConfig  # noqa: E402
from actc.inputs import VGamepadInput, focus_ac_window  # noqa: E402
from actc.shared_memory import SharedMemoryReader  # noqa: E402


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def write_row(
    writer: csv.DictWriter,
    state,
    command_steer: float,
    throttle: float,
    brake: float,
    phase: str,
    target_kmh: float,
) -> None:
    writer.writerow(
        {
            "timestamp": f"{state.timestamp:.6f}",
            "packet_id": state.packet_id,
            "phase": phase,
            "target_kmh": f"{target_kmh:.6f}",
            "speed_kmh": f"{state.speed_kmh:.6f}",
            "steer_axis": f"{command_steer:.9f}",
            "measured_steer": f"{state.steer:.9f}",
            "yaw_rate_rad_s": f"{state.yaw_rate_rad_s:.9f}",
            "lateral_accel_g": f"{state.acceleration_g[0]:.9f}",
            "longitudinal_accel_g": f"{state.acceleration_g[2]:.9f}",
            "local_velocity_x": f"{state.local_velocity_m_s[0]:.9f}",
            "local_velocity_z": f"{state.local_velocity_m_s[2]:.9f}",
            "throttle": f"{throttle:.6f}",
            "brake": f"{brake:.6f}",
            "tyres_out": state.tyres_out,
            "wheel_slip_abs_max": (
                f"{max(abs(value) for value in state.tyre_slip):.9f}"
            ),
        }
    )


def analyze(path: Path) -> dict:
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    samples = []
    for row in rows:
        if row["phase"] != "sweep":
            continue
        speed_kmh = float(row["speed_kmh"])
        steer = float(row["steer_axis"])
        yaw_rate = -float(row["yaw_rate_rad_s"])
        lateral_g = abs(float(row["lateral_accel_g"]))
        slip = float(row["wheel_slip_abs_max"])
        if (
            speed_kmh < 20.0
            or abs(steer) < 0.015
            or abs(yaw_rate) < 0.02
            or lateral_g > 0.70
            or slip > 0.35
        ):
            continue
        speed_mps = max(3.0, speed_kmh / 3.6)
        samples.append((speed_mps, steer, yaw_rate))

    if len(samples) < 100:
        raise RuntimeError("Not enough valid steering samples.")
    data = np.asarray(samples)
    speed = data[:, 0]
    steer = data[:, 1]
    yaw_rate = data[:, 2]
    wheelbase = 2.85
    # r / (k*u) = v / (L + K*v^2)
    # (u*v/r) = L/k + (K/k)*v^2
    x = speed * speed
    y = steer * speed / yaw_rate
    design = np.column_stack((np.ones_like(x), x))
    coefficients, *_ = np.linalg.lstsq(design, y, rcond=None)
    intercept, slope = (float(value) for value in coefficients)
    # Effective axis-to-road-wheel scale under the assumed bicycle model.
    # It is not independently identifiable without a physical steering-angle
    # measurement, so treat this as a consistency statistic rather than an
    # additional plant model.
    k_steer = wheelbase / max(intercept, 1e-6)
    understeer = slope * k_steer

    band_results = []
    for lower, upper in ((20.0, 40.0), (40.0, 65.0), (65.0, 100.0)):
        mask = (speed * 3.6 >= lower) & (speed * 3.6 < upper)
        if not np.any(mask):
            continue
        band_speed = speed[mask]
        band_steer = steer[mask]
        band_yaw = yaw_rate[mask]
        estimates = (
            band_yaw
            * (wheelbase + understeer * band_speed * band_speed)
            / (band_speed * band_steer)
        )
        band_results.append(
            {
                "lower_kmh": lower,
                "upper_kmh": upper,
                "samples": int(np.sum(mask)),
                "k_steer_median": float(np.median(estimates)),
                "k_steer_p10": float(np.quantile(estimates, 0.10)),
                "k_steer_p90": float(np.quantile(estimates, 0.90)),
            }
        )
    return {
        "wheelbase_m": wheelbase,
        "k_steer_rad_per_axis": k_steer,
        "understeer_gradient": understeer,
        "fit_samples": len(samples),
        "bands": band_results,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--speeds",
        nargs="+",
        type=float,
        default=(30.0, 50.0, 80.0),
    )
    parser.add_argument("--duration-per-speed", type=float, default=12.0)
    parser.add_argument("--frequency-hz", type=float, default=0.15)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument(
        "--log",
        type=Path,
        default=PROJECT_ROOT / "logs" / "steering_calibration.csv",
    )
    parser.add_argument("--focus-ac", action="store_true")
    args = parser.parse_args()

    if args.focus_ac:
        focus_ac_window()
    reader = SharedMemoryReader()
    reader.connect(wait_seconds=10.0)
    state = reader.snapshot()
    print(
        f"Connected: car={state.car_model!r} track={state.track!r}",
        flush=True,
    )
    if state.status != 2:
        reader.close()
        raise RuntimeError("AC is not in a live driving state.")

    pad = VGamepadInput()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    handle = args.log.open("w", newline="", encoding="utf-8")
    fields = [
        "timestamp",
        "packet_id",
        "phase",
        "target_kmh",
        "speed_kmh",
        "steer_axis",
        "measured_steer",
        "yaw_rate_rad_s",
        "lateral_accel_g",
        "longitudinal_accel_g",
        "local_velocity_x",
        "local_velocity_z",
        "throttle",
        "brake",
        "tyres_out",
        "wheel_slip_abs_max",
    ]
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()

    period = 1.0 / max(1.0, args.hz)
    try:
        for target_kmh in args.speeds:
            speed_controller = SpeedController(
                SpeedControllerConfig(target_kmh=target_kmh)
            )
            amplitude = clamp(
                4.5 / max(10.0, target_kmh),
                0.025,
                0.15,
            )
            print(
                f"Calibrating {target_kmh:.0f} km/h, "
                f"axis amplitude={amplitude:.3f}",
                flush=True,
            )

            stable_since: float | None = None
            approach_started = time.monotonic()
            while True:
                state = reader.snapshot()
                throttle, brake = speed_controller.step(
                    state.speed_kmh,
                    period,
                    target_kmh,
                )
                pad.apply(0.0, throttle, brake)
                write_row(
                    writer,
                    state,
                    command_steer=0.0,
                    throttle=throttle,
                    brake=brake,
                    phase="approach",
                    target_kmh=target_kmh,
                )
                now = time.monotonic()
                if abs(state.speed_kmh - target_kmh) < 2.0:
                    stable_since = stable_since or now
                else:
                    stable_since = None
                if stable_since is not None and now - stable_since >= 1.0:
                    break
                if now - approach_started > 25.0:
                    raise RuntimeError(
                        f"Could not reach {target_kmh:.0f} km/h."
                    )
                time.sleep(period)

            sweep_started = time.monotonic()
            while time.monotonic() - sweep_started < args.duration_per_speed:
                state = reader.snapshot()
                elapsed = time.monotonic() - sweep_started
                envelope = min(1.0, elapsed / 1.0)
                command_steer = envelope * amplitude * math.sin(
                    2.0 * math.pi * args.frequency_hz * elapsed
                )
                throttle, brake = speed_controller.step(
                    state.speed_kmh,
                    period,
                    target_kmh,
                )
                pad.apply(command_steer, throttle, brake)
                write_row(
                    writer,
                    state,
                    command_steer=command_steer,
                    throttle=throttle,
                    brake=brake,
                    phase="sweep",
                    target_kmh=target_kmh,
                )
                lateral_g = abs(state.acceleration_g[0])
                slip = max(abs(value) for value in state.tyre_slip)
                if (
                    lateral_g > 0.65
                    or slip > 0.40
                    or state.tyres_out >= 2
                    or state.speed_kmh > target_kmh + 12.0
                ):
                    raise RuntimeError(
                        "Calibration abort: "
                        f"ay={lateral_g:.3f}g slip={slip:.3f} "
                        f"tyres_out={state.tyres_out} "
                        f"speed={state.speed_kmh:.1f}"
                    )
                handle.flush()
                time.sleep(period)

            pad.apply(0.0, 0.0, 0.35)
            time.sleep(1.5)
    finally:
        pad.apply(0.0, 0.0, 0.8)
        time.sleep(1.5)
        pad.apply(0.0, 0.0, 0.0)
        handle.close()
        pad.close()
        reader.close()

    result = analyze(args.log)
    output = args.log.with_suffix(".json")
    output.write_text(
        json.dumps(result, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, ensure_ascii=True))
    print(f"Calibration data: {args.log}")
    print(f"Calibration result: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
