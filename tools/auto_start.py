from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from actc.shared_memory import SharedMemoryReader  # noqa: E402
from actc.inputs import VGamepadInput  # noqa: E402


def build_command(
    log_path: Path,
    *,
    max_speed_kmh: float = 280.0,
    lateral_controller: str,
    resume_tuning: Path | None,
    mpc_model: Path | None,
    total_laps: int = 30,
    control_laps: int = 10,
    planning_laps: int = 20,
    racing_line: str = "optimized",
    duration_s: float = 8400.0,
    lateral_accel_mps2: float = 6.867,
    scaled_speed_max_lateral_g: float = 0.0,
    longitudinal_accel_mps2: float = 3.924,
    target_rise_kmh_s: float = 10.0,
    target_fall_kmh_s: float = 60.0,
    braking_accel_mps2: float = 16.0,
    max_braking_accel_mps2: float = 20.0,
    longitudinal_horizon: int = 45,
    longitudinal_dt_s: float = 0.03,
    braking_safety_factor: float = 0.96,
    braking_response_time_s: float = 0.0,
    high_speed_braking_gain_s_per_kmh: float = 0.004,
    braking_lead_distance_m: float = 10.0,
    braking_lead_speed_gain_m_per_kmh: float = 0.08,
    braking_lead_speed_min_kmh: float = 50.0,
    braking_lead_speed_max_kmh: float = 300.0,
    entry_lateral_accel_boost: float = 0.20,
    friction_ellipse_planning: bool = False,
    path_resample_spacing_m: float = 0.0,
    line_target_clearance_m: float = 0.0,
    offtrack_limit_m: float = 12.0,
    longitudinal_controller: str = "pi",
    merge_speed_kmh: float = 80.0,
    startup_merge_distance_m: float = 120.0,
    startup_ramp_distance_m: float = 0.0,
    max_speed_scale: float = 1.45,
    speed_envelope: Path | None = None,
    speed_envelope_blend: float = 1.0,
    speed_envelope_scale: float = 0.95,
    speed_envelope_only_increase: bool = True,
    autotune: bool = True,
) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "actc.cli",
        "--mode",
        "lap",
        "--input",
        "vgamepad",
        "--max-speed-kmh",
        str(max_speed_kmh),
        "--min-speed-kmh",
        "25",
        "--start-speed-kmh",
        "80",
        "--lateral-accel",
        str(lateral_accel_mps2),
        "--scaled-speed-max-lateral-g",
        str(scaled_speed_max_lateral_g),
        "--longitudinal-accel",
        str(longitudinal_accel_mps2),
        "--braking-accel",
        str(braking_accel_mps2),
        "--max-braking-accel",
        str(max_braking_accel_mps2),
        "--braking-safety-factor",
        str(braking_safety_factor),
        "--braking-response-time-s",
        str(braking_response_time_s),
        "--high-speed-braking-gain-s-per-kmh",
        str(high_speed_braking_gain_s_per_kmh),
        "--braking-lead-distance-m",
        str(braking_lead_distance_m),
        "--braking-lead-speed-gain-m-per-kmh",
        str(braking_lead_speed_gain_m_per_kmh),
        "--braking-lead-speed-min-kmh",
        str(braking_lead_speed_min_kmh),
        "--braking-lead-speed-max-kmh",
        str(braking_lead_speed_max_kmh),
        "--entry-lateral-accel-boost",
        str(entry_lateral_accel_boost),
        (
            "--friction-ellipse-planning"
            if friction_ellipse_planning
            else "--no-friction-ellipse-planning"
        ),
        "--path-resample-spacing-m",
        str(path_resample_spacing_m),
        "--line-target-clearance-m",
        str(line_target_clearance_m),
        "--offtrack-limit-m",
        str(offtrack_limit_m),
        "--target-rise-kmh-s",
        str(target_rise_kmh_s),
        "--target-fall-kmh-s",
        str(target_fall_kmh_s),
        "--merge-speed-kmh",
        str(merge_speed_kmh),
        "--startup-merge-distance",
        str(startup_merge_distance_m),
        "--startup-ramp-distance",
        str(startup_ramp_distance_m),
        "--laps",
        str(total_laps),
        "--control-laps",
        str(control_laps),
        "--planning-laps",
        str(planning_laps),
        "--racing-line",
        racing_line,
        "--lateral-controller",
        lateral_controller,
        "--longitudinal-controller",
        longitudinal_controller,
        "--longitudinal-horizon",
        str(longitudinal_horizon),
        "--longitudinal-dt",
        str(longitudinal_dt_s),
        "--mpc-horizon",
        "30",
        "--target-lap-s",
        "180",
        "--speed-scale-step",
        "0.05",
        "--max-speed-scale",
        str(max_speed_scale),
        "--count-from-start",
        "--duration",
        str(duration_s),
        "--arm-delay",
        "2",
        "--wait",
        "30",
        "--hz",
        "50",
        "--focus-ac",
        "--log",
        str(log_path),
    ]
    if autotune:
        command.append("--autotune")
    if speed_envelope is not None:
        command.extend(
            [
                "--speed-envelope",
                str(speed_envelope),
                "--speed-envelope-blend",
                str(speed_envelope_blend),
                "--speed-envelope-scale",
                str(speed_envelope_scale),
                (
                    "--speed-envelope-only-increase"
                    if speed_envelope_only_increase
                    else "--no-speed-envelope-only-increase"
                ),
            ]
        )
    if resume_tuning is not None:
        command.extend(["--resume-tuning", str(resume_tuning)])
    if mpc_model is not None:
        command.extend(["--mpc-model", str(mpc_model)])
    return command


def wait_for_start(
    reader: SharedMemoryReader,
    pad: VGamepadInput,
    timeout_s: float,
) -> tuple[bool, str]:
    deadline = time.monotonic() + max(1.0, timeout_s)
    previous_packet = -1
    last_packet_change = time.monotonic()
    stable_since: float | None = None

    while time.monotonic() < deadline:
        pad.apply(0.0, 0.0, 0.0)
        try:
            state = reader.snapshot()
        except Exception:
            return False, "telemetry-unavailable"

        now = time.monotonic()
        if state.packet_id != previous_packet:
            previous_packet = state.packet_id
            last_packet_change = now
        if state.status != 2 and now - last_packet_change > 5.0:
            return False, "telemetry-stale-reconnect"
        live = now - last_packet_change < 1.0 and state.status == 2
        at_initial_position = (
            (
                0.80 <= state.normalized_position <= 0.99
                or 0.0 <= state.normalized_position <= 0.03
            )
            and not state.is_in_pit
            and not state.is_in_pit_lane
        )
        stationary = state.speed_kmh < 1.0
        released = state.brake < 0.05 and state.gas < 0.05
        on_track = state.tyres_out <= 1
        ready = (
            live
            and at_initial_position
            and stationary
            and released
            and on_track
        )

        if ready:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= 1.0:
                return True, (
                    f"ready speed={state.speed_kmh:.3f} "
                    f"s={state.normalized_position:.6f} "
                    f"packet={state.packet_id}"
                )
        else:
            stable_since = None

        time.sleep(0.10)

    return False, "timeout"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=1800.0)
    parser.add_argument("--laps", type=int, default=30)
    parser.add_argument("--control-laps", type=int, default=10)
    parser.add_argument("--planning-laps", type=int, default=20)
    parser.add_argument(
        "--racing-line",
        choices=("centerline", "optimized"),
        default="optimized",
    )
    parser.add_argument("--duration", type=float, default=8400.0)
    parser.add_argument(
        "--lateral-accel-mps2",
        type=float,
        default=6.867,
    )
    parser.add_argument(
        "--scaled-speed-max-lateral-g",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--longitudinal-accel-mps2",
        type=float,
        default=3.924,
    )
    parser.add_argument("--max-speed-kmh", type=float, default=280.0)
    parser.add_argument("--target-rise-kmh-s", type=float, default=10.0)
    parser.add_argument("--target-fall-kmh-s", type=float, default=60.0)
    parser.add_argument("--braking-accel-mps2", type=float, default=16.0)
    parser.add_argument(
        "--max-braking-accel-mps2",
        type=float,
        default=20.0,
    )
    parser.add_argument("--longitudinal-horizon", type=int, default=45)
    parser.add_argument("--longitudinal-dt", type=float, default=0.03)
    parser.add_argument("--braking-safety-factor", type=float, default=0.96)
    parser.add_argument("--braking-response-time-s", type=float, default=0.0)
    parser.add_argument(
        "--high-speed-braking-gain-s-per-kmh",
        type=float,
        default=0.004,
    )
    parser.add_argument(
        "--braking-lead-distance-m",
        type=float,
        default=10.0,
    )
    parser.add_argument(
        "--braking-lead-speed-gain-m-per-kmh",
        type=float,
        default=0.08,
    )
    parser.add_argument(
        "--braking-lead-speed-min-kmh",
        type=float,
        default=50.0,
    )
    parser.add_argument(
        "--braking-lead-speed-max-kmh",
        type=float,
        default=300.0,
    )
    parser.add_argument(
        "--entry-lateral-accel-boost",
        type=float,
        default=0.20,
    )
    parser.add_argument(
        "--friction-ellipse-planning",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--path-resample-spacing-m",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--line-target-clearance-m",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--offtrack-limit-m",
        type=float,
        default=12.0,
    )
    parser.add_argument(
        "--longitudinal-controller",
        choices=("pi", "mpc"),
        default="pi",
    )
    parser.add_argument("--merge-speed-kmh", type=float, default=80.0)
    parser.add_argument(
        "--startup-merge-distance",
        type=float,
        default=120.0,
    )
    parser.add_argument(
        "--startup-ramp-distance",
        type=float,
        default=0.0,
    )
    parser.add_argument("--max-speed-scale", type=float, default=1.45)
    parser.add_argument("--speed-envelope", type=Path, default=None)
    parser.add_argument(
        "--speed-envelope-blend",
        type=float,
        default=1.0,
    )
    parser.add_argument(
        "--speed-envelope-scale",
        type=float,
        default=0.95,
    )
    parser.add_argument(
        "--speed-envelope-only-increase",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--autotune",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=PROJECT_ROOT / "logs" / "lmpc_round3_30_laps.csv",
    )
    parser.add_argument(
        "--lateral-controller",
        choices=("stanley", "lmpc", "nmpc"),
        default="lmpc",
    )
    parser.add_argument(
        "--resume-tuning",
        type=Path,
        default=(
            PROJECT_ROOT
            / "configs"
            / "lmpc_round2_incremental_rstable.json"
        ),
    )
    parser.add_argument(
        "--mpc-model",
        type=Path,
        default=(
            PROJECT_ROOT
            / "logs"
            / "lmpc_round2_identified_model_v6.json"
        ),
    )
    args = parser.parse_args()

    reader: SharedMemoryReader | None = None
    pad: VGamepadInput | None = None
    try:
        while True:
            if reader is None:
                try:
                    reader = SharedMemoryReader()
                    reader.connect(wait_seconds=2.0)
                except Exception:
                    if reader is not None:
                        reader.close()
                    reader = None
                    time.sleep(0.5)
                    continue

            if pad is None:
                pad = VGamepadInput()
            ready, detail = wait_for_start(reader, pad, args.timeout)
            print(f"auto-start check: {detail}", flush=True)
            if ready:
                break

            reader.close()
            reader = None

        reader.close()
        reader = None
        if pad is not None:
            pad.close()
            pad = None
        args.log.parent.mkdir(parents=True, exist_ok=True)
        command = build_command(
            args.log,
            max_speed_kmh=args.max_speed_kmh,
            lateral_controller=args.lateral_controller,
            resume_tuning=args.resume_tuning,
            mpc_model=args.mpc_model,
            total_laps=args.laps,
            control_laps=args.control_laps,
            planning_laps=args.planning_laps,
            racing_line=args.racing_line,
            duration_s=args.duration,
            lateral_accel_mps2=args.lateral_accel_mps2,
            scaled_speed_max_lateral_g=(
                args.scaled_speed_max_lateral_g
            ),
            longitudinal_accel_mps2=args.longitudinal_accel_mps2,
            target_rise_kmh_s=args.target_rise_kmh_s,
            target_fall_kmh_s=args.target_fall_kmh_s,
            braking_accel_mps2=args.braking_accel_mps2,
            max_braking_accel_mps2=args.max_braking_accel_mps2,
            longitudinal_horizon=args.longitudinal_horizon,
            longitudinal_dt_s=args.longitudinal_dt,
            braking_safety_factor=args.braking_safety_factor,
            braking_response_time_s=args.braking_response_time_s,
            high_speed_braking_gain_s_per_kmh=(
                args.high_speed_braking_gain_s_per_kmh
            ),
            braking_lead_distance_m=args.braking_lead_distance_m,
            braking_lead_speed_gain_m_per_kmh=(
                args.braking_lead_speed_gain_m_per_kmh
            ),
            braking_lead_speed_min_kmh=(
                args.braking_lead_speed_min_kmh
            ),
            braking_lead_speed_max_kmh=(
                args.braking_lead_speed_max_kmh
            ),
            entry_lateral_accel_boost=(
                args.entry_lateral_accel_boost
            ),
            friction_ellipse_planning=args.friction_ellipse_planning,
            path_resample_spacing_m=args.path_resample_spacing_m,
            line_target_clearance_m=args.line_target_clearance_m,
            offtrack_limit_m=args.offtrack_limit_m,
            longitudinal_controller=args.longitudinal_controller,
            merge_speed_kmh=args.merge_speed_kmh,
            startup_merge_distance_m=args.startup_merge_distance,
            startup_ramp_distance_m=args.startup_ramp_distance,
            max_speed_scale=args.max_speed_scale,
            speed_envelope=args.speed_envelope,
            speed_envelope_blend=args.speed_envelope_blend,
            speed_envelope_scale=args.speed_envelope_scale,
            speed_envelope_only_increase=(
                args.speed_envelope_only_increase
            ),
            autotune=args.autotune,
        )
        print("starting:", " ".join(command), flush=True)
        return subprocess.call(command, cwd=PROJECT_ROOT)
    finally:
        if reader is not None:
            reader.close()
        if pad is not None:
            pad.close()


if __name__ == "__main__":
    raise SystemExit(main())
