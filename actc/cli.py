from __future__ import annotations

import argparse
import bisect
import copy
import csv
import hashlib
import json
import math
import sys
import time
from dataclasses import fields, replace
from pathlib import Path

from .controllers import (
    HeadingController,
    HeadingControllerConfig,
    SpeedController,
    SpeedControllerConfig,
)
from .inputs import InputBackend, create_input_backend, focus_ac_window
from .lap import (
    LapControlInfo,
    PurePursuitController,
    StanleyConfig,
    StanleyController,
)
from .longitudinal import (
    LongitudinalMpcConfig,
    LongitudinalMpcController,
    LongitudinalPedalConfig,
    LongitudinalPedalMapper,
)
from .mpc import (
    LinearMpcController,
    MpcConfig,
    MpcVehicleParameters,
    NonlinearMpcController,
)
from .planning import (
    PlanningConfig,
    PlanningLineTuner,
    PlanningSample,
    PlanningUpdate,
)
from .shared_memory import SharedMemoryReader, SharedMemoryUnavailable, VehicleState
from .track import (
    TrackPath,
    clamp,
    physics_heading_to_world,
    resolve_fast_lane,
)


XUSB_GAMEPAD_A = 0x1000
SPEED_BAND_LIMITS_KMH = (
    40.0,
    60.0,
    80.0,
    100.0,
    120.0,
    140.0,
    180.0,
    220.0,
)
SPEED_BAND_COUNT = len(SPEED_BAND_LIMITS_KMH) + 1
LEGACY_4_BAND_LIMITS_KMH = (40.0, 80.0, 140.0)
LEGACY_6_BAND_LIMITS_KMH = (40.0, 80.0, 140.0, 180.0, 220.0)
LEGACY_8_BAND_LIMITS_KMH = (
    40.0,
    60.0,
    80.0,
    100.0,
    140.0,
    180.0,
    220.0,
)
SPEED_SCALE_TRANSITION_KMH = 10.0


def _speed_band_limits(band_count: int) -> tuple[float, ...]:
    if band_count == 4:
        return LEGACY_4_BAND_LIMITS_KMH
    if band_count == 6:
        return LEGACY_6_BAND_LIMITS_KMH
    if band_count == 8:
        return LEGACY_8_BAND_LIMITS_KMH
    return SPEED_BAND_LIMITS_KMH[: max(0, band_count - 1)]


def _speed_band(
    speed_kmh: float,
    band_count: int = SPEED_BAND_COUNT,
) -> int:
    return bisect.bisect_right(
        _speed_band_limits(band_count),
        speed_kmh,
    )


def _speed_scale_kmh(
    speed_kmh: float,
    speed_scale_by_band: list[float] | tuple[float, ...],
) -> float:
    if not speed_scale_by_band:
        return 1.0
    if len(speed_scale_by_band) == 1:
        return float(speed_scale_by_band[0])
    transition = SPEED_SCALE_TRANSITION_KMH
    speed_kmh = max(0.0, speed_kmh)
    boundaries = _speed_band_limits(len(speed_scale_by_band))
    for band, boundary in enumerate(boundaries):
        start = speed_scale_by_band[band]
        end = speed_scale_by_band[band + 1]
        if speed_kmh <= boundary - transition:
            return float(start)
        if speed_kmh < boundary + transition:
            fraction = (speed_kmh - boundary + transition) / (
                2.0 * transition
            )
            return float(start + fraction * (end - start))
    return float(speed_scale_by_band[-1])


class InitialGearEngager:
    def __init__(self) -> None:
        self.attempted = False

    def update(
        self,
        state: VehicleState,
        backend: InputBackend,
        _now: float,
    ) -> None:
        if self.attempted:
            return
        if state.gear != 0 or state.speed_kmh >= 1.0:
            self.attempted = True
            return
        backend.tap_button(XUSB_GAMEPAD_A)
        self.attempted = True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Minimal Assetto Corsa telemetry and control loop."
    )
    parser.add_argument(
        "--mode",
        choices=("monitor", "speed", "heading", "lap"),
        default="monitor",
    )
    parser.add_argument(
        "--input",
        choices=("dry-run", "keyboard", "vjoy", "vgamepad"),
        default="keyboard",
    )
    parser.add_argument("--target-kmh", type=float, default=40.0)
    parser.add_argument("--heading-deg", type=float, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--wait", type=float, default=120.0)
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--arm-delay", type=float, default=3.0)
    parser.add_argument("--focus-ac", action="store_true")
    parser.add_argument("--log", type=Path, default=None)
    parser.add_argument("--steer-sign", type=float, default=1.0)
    parser.add_argument("--heading-kp", type=float, default=1.20)
    parser.add_argument("--heading-kd", type=float, default=0.25)
    parser.add_argument("--fast-lane", type=Path, default=None)
    parser.add_argument("--max-speed-kmh", type=float, default=75.0)
    parser.add_argument("--min-speed-kmh", type=float, default=25.0)
    parser.add_argument("--start-speed-kmh", type=float, default=40.0)
    parser.add_argument("--target-rise-kmh-s", type=float, default=8.0)
    parser.add_argument("--target-fall-kmh-s", type=float, default=None)
    parser.add_argument("--longitudinal-accel", type=float, default=2.5)
    parser.add_argument("--braking-accel", type=float, default=16.0)
    parser.add_argument("--braking-accel-step", type=float, default=0.30)
    parser.add_argument("--max-braking-accel", type=float, default=20.0)
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
    parser.add_argument("--lateral-accel", type=float, default=3.0)
    parser.add_argument(
        "--scaled-speed-max-lateral-g",
        type=float,
        default=0.0,
    )
    parser.add_argument("--lookahead-gain", type=float, default=0.80)
    parser.add_argument("--heading-sign", type=float, default=1.0)
    parser.add_argument("--offtrack-limit-m", type=float, default=9.0)
    parser.add_argument("--lateral-offset", type=float, default=None)
    parser.add_argument("--merge-distance", type=float, default=60.0)
    parser.add_argument("--merge-speed-kmh", type=float, default=10.0)
    parser.add_argument("--use-merge-plan", action="store_true")
    parser.add_argument(
        "--startup-merge-distance",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--startup-ramp-distance",
        type=float,
        default=0.0,
    )
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--control-laps", type=int, default=0)
    parser.add_argument("--planning-laps", type=int, default=0)
    parser.add_argument(
        "--racing-line",
        choices=("centerline", "optimized"),
        default="centerline",
    )
    parser.add_argument("--autotune", action="store_true")
    parser.add_argument("--resume-tuning", type=Path, default=None)
    parser.add_argument("--mpc-model", type=Path, default=None)
    parser.add_argument("--target-lap-s", type=float, default=180.0)
    parser.add_argument("--speed-scale-step", type=float, default=0.05)
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
    parser.add_argument("--count-from-start", action="store_true")
    parser.add_argument(
        "--lateral-controller",
        choices=("stanley", "lmpc", "nmpc"),
        default="stanley",
    )
    parser.add_argument(
        "--longitudinal-controller",
        choices=("pi", "mpc"),
        default="pi",
    )
    parser.add_argument("--mpc-horizon", type=int, default=20)
    parser.add_argument("--longitudinal-horizon", type=int, default=45)
    parser.add_argument("--longitudinal-dt", type=float, default=0.03)
    parser.add_argument("--planning-initial-blend", type=float, default=0.5)
    parser.add_argument("--planning-blend-step", type=float, default=0.04)
    parser.add_argument("--planning-max-blend", type=float, default=0.90)
    parser.add_argument(
        "--planning-target-clearance-m",
        type=float,
        default=1.0,
    )
    parser.add_argument(
        "--planning-hard-clearance-m",
        type=float,
        default=0.80,
    )
    parser.add_argument(
        "--planning-clearance-weight",
        type=float,
        default=4000.0,
    )
    parser.add_argument(
        "--planning-optimization-nodes",
        type=int,
        default=128,
    )
    parser.add_argument(
        "--planning-optimize-initial-line",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


def format_state(state: VehicleState) -> str:
    return (
        f"status={state.status} "
        f"speed={state.speed_kmh:6.2f} km/h "
        f"throttle={state.gas:4.2f} brake={state.brake:4.2f} "
        f"steer={state.steer:+.3f} yaw_rate={state.yaw_rate_rad_s:+.3f} "
        f"gear={state.gear} rpm={state.rpm:7.0f} "
        f"s={state.normalized_position:.4f} tyres_out={state.tyres_out}"
    )


def log_writer(path: Path):
    handle = path.open("w", newline="", encoding="utf-8")
    fields = [
        "timestamp",
        "packet_id",
        "speed_kmh",
        "gas",
        "brake",
        "steer",
        "gear",
        "rpm",
        "heading_rad",
        "yaw_rate_rad_s",
        "position_x",
        "position_y",
        "position_z",
        "normalized_position",
        "tyres_out",
        "cmd_steer",
        "cmd_throttle",
        "cmd_brake",
    ]
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    return handle, writer


def write_log_row(
    writer: csv.DictWriter,
    state: VehicleState,
    cmd_steer: float,
    cmd_throttle: float,
    cmd_brake: float,
) -> None:
    writer.writerow(
        {
            "timestamp": f"{state.timestamp:.6f}",
            "packet_id": state.packet_id,
            "speed_kmh": f"{state.speed_kmh:.6f}",
            "gas": f"{state.gas:.6f}",
            "brake": f"{state.brake:.6f}",
            "steer": f"{state.steer:.6f}",
            "gear": state.gear,
            "rpm": f"{state.rpm:.3f}",
            "heading_rad": f"{state.heading_rad:.9f}",
            "yaw_rate_rad_s": f"{state.yaw_rate_rad_s:.9f}",
            "position_x": f"{state.position[0]:.6f}",
            "position_y": f"{state.position[1]:.6f}",
            "position_z": f"{state.position[2]:.6f}",
            "normalized_position": f"{state.normalized_position:.9f}",
            "tyres_out": state.tyres_out,
            "cmd_steer": f"{cmd_steer:.6f}",
            "cmd_throttle": f"{cmd_throttle:.6f}",
            "cmd_brake": f"{cmd_brake:.6f}",
        }
    )


def _load_tuning_state(path: Path | None) -> dict:
    if path is None:
        return {}
    if not path.exists():
        raise RuntimeError(f"Tuning state does not exist: {path}")
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        raise RuntimeError(f"Tuning state is empty: {path}")
    try:
        last_record = json.loads(text)
    except json.JSONDecodeError:
        last_record = None
        for line in text.splitlines():
            if line.strip():
                last_record = json.loads(line)
    if last_record is None:
        raise RuntimeError(f"Tuning state is empty: {path}")
    parameters = last_record.get("after", last_record)
    return {
        "speed_scale_by_band": parameters.get(
            "speed_scale_by_band",
            (1.0, 1.0, 1.0, 1.0),
        ),
        "braking_accel_by_band": parameters.get(
            "braking_accel_by_band",
        ),
        "controller": parameters.get("controller", {}),
        "speed_controller": parameters.get(
            "speed_controller",
            {},
        ),
        "longitudinal_controller": parameters.get(
            "longitudinal_controller",
            {},
        ),
    }


def _load_mpc_parameters(
    path: Path | None,
) -> MpcVehicleParameters | None:
    if path is None:
        return None
    if not path.exists():
        raise RuntimeError(f"MPC model file does not exist: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    values = payload.get("model", payload)
    allowed = {item.name for item in fields(MpcVehicleParameters)}
    filtered = {}
    for key, value in values.items():
        if key not in allowed:
            continue
        if isinstance(value, (int, float)):
            filtered[key] = float(value)
        elif isinstance(value, (list, tuple)) and all(
            isinstance(item, (int, float)) for item in value
        ):
            filtered[key] = tuple(float(item) for item in value)
    return MpcVehicleParameters(**filtered)


def _apply_saved_lmpc_parameters(
    config: MpcConfig,
    parameters: dict,
) -> None:
    allowed = {
        "q_lateral",
        "q_heading",
        "q_speed",
        "q_lateral_velocity",
        "q_yaw_rate",
        "r_steer",
        "r_accel",
        "rd_steer",
        "rd_accel",
        "terminal_weight_scale",
        "steering_lead_s",
        "steering_filter_tau_s",
        "steering_actuator_tau_s",
        "steering_state_weight",
        "steering_rate_gain",
        "steering_jerk_limit_rad_s3",
        "steering_deadband",
        "steering_hysteresis",
        "solve_time_budget_ms",
        "reference_preview_s",
    }
    map_fields = {
        "weight_map_kmh",
        "q_lateral_weight_map",
        "q_heading_weight_map",
        "q_yaw_rate_weight_map",
        "r_steer_weight_map",
        "rd_steer_weight_map",
    }
    for key in allowed:
        value = parameters.get(key)
        if isinstance(value, (int, float)):
            setattr(config, key, float(value))
    for key in (
        "coast_down_accel_map_kmh",
        "coast_down_accel_map_mps2",
        "lateral_weight_schedule_g",
        "r_accel_lateral_scale",
        "r_jerk_lateral_scale",
        "speed_weight_schedule_kmh",
        "r_accel_speed_scale",
        "r_jerk_speed_scale",
    ):
        value = parameters.get(key)
        if (
            isinstance(value, (list, tuple))
            and len(value) >= 2
        ):
            setattr(
                config,
                key,
                tuple(float(item) for item in value),
            )
    actuator_model = parameters.get("use_steering_actuator_model")
    if isinstance(actuator_model, bool):
        config.use_steering_actuator_model = actuator_model
    for key in map_fields:
        value = parameters.get(key)
        if isinstance(value, (list, tuple)):
            setattr(config, key, tuple(float(item) for item in value))
    vehicle_map_fields = {
        "steer_lateral_gain_map",
        "steer_yaw_gain_map",
        "yaw_inertia_map_kgm2",
        "lateral_velocity_damping_map",
        "lateral_yaw_coupling_map",
        "yaw_velocity_coupling_map",
        "yaw_rate_damping_map",
        "yaw_response_load_schedule_g",
        "yaw_response_load_scale",
    }
    vehicle_updates = {}
    for key in vehicle_map_fields:
        value = parameters.get(key)
        if isinstance(value, (list, tuple)):
            vehicle_updates[key] = tuple(float(item) for item in value)
    if vehicle_updates:
        config.parameters = replace(
            config.parameters,
            **vehicle_updates,
        )


def _apply_saved_stanley_parameters(
    config: StanleyConfig,
    parameters: dict,
) -> None:
    scalar_keys = {
        "cross_track_gain",
        "heading_gain",
        "yaw_rate_gain",
        "steering_filter_tau_s",
    }
    map_keys = {
        "cross_track_gain_map",
        "heading_gain_map",
        "yaw_rate_gain_map",
        "filter_tau_map",
    }
    for key in scalar_keys:
        value = parameters.get(key)
        if isinstance(value, (int, float)):
            setattr(config, key, float(value))
    for key in map_keys:
        value = parameters.get(key)
        if isinstance(value, (list, tuple)) and len(value) == 5:
            setattr(config, key, tuple(float(item) for item in value))


def _apply_saved_speed_controller_parameters(
    config,
    parameters: dict,
) -> None:
    allowed = {
        "kp_accel",
        "kp_brake",
        "ki",
        "integral_limit",
        "deadband_kmh",
        "throttle_rate_per_s",
        "brake_rate_per_s",
        "speed_rate_filter_tau_s",
        "prediction_time_s",
        "max_accel_kmh_s",
    }
    for key in allowed:
        value = parameters.get(key)
        if isinstance(value, (int, float)):
            setattr(config, key, float(value))


def _apply_saved_longitudinal_parameters(
    config: LongitudinalMpcConfig,
    parameters: dict,
) -> None:
    allowed = {
        "response_tau_s",
        "q_speed",
        "q_accel",
        "r_accel",
        "r_jerk",
        "max_accel_mps2",
        "max_brake_mps2",
        "max_jerk_mps3",
        "max_brake_jerk_mps3",
        "reference_filter_up_tau_s",
        "reference_filter_down_tau_s",
        "eps_abs",
        "eps_rel",
        "max_iter",
    }
    for key in allowed:
        value = parameters.get(key)
        if isinstance(value, (int, float)):
            setattr(config, key, float(value))


def run(args: argparse.Namespace) -> int:
    reader = SharedMemoryReader()
    if args.mode == "monitor":
        if args.focus_ac and not focus_ac_window():
            print(
                "Warning: could not focus the Assetto Corsa window.",
                file=sys.stderr,
            )
        print(f"Waiting for Assetto Corsa telemetry (up to {args.wait:.0f}s)...")
        reader.connect(wait_seconds=args.wait)
        state = reader.snapshot()
        print(f"Connected: car={state.car_model!r} track={state.track!r}")
        return run_monitor(reader, args)

    backend = create_input_backend(args.input)
    print(f"Input backend ready: {backend.name}")
    if args.focus_ac and not focus_ac_window():
        print(
            "Warning: could not focus the Assetto Corsa window.",
            file=sys.stderr,
        )
    print(f"Waiting for Assetto Corsa telemetry (up to {args.wait:.0f}s)...")
    try:
        reader.connect(wait_seconds=args.wait)
    except Exception:
        backend.close()
        reader.close()
        raise

    state = reader.snapshot()
    print(f"Connected: car={state.car_model!r} track={state.track!r}")
    if args.mode != "monitor" and state.status != 2:
        reader.close()
        raise RuntimeError(
            f"AC is not in live driving state (graphics.status={state.status}). "
            "Exit Replay and start a live Practice session."
        )
    duration = (
        args.duration
        if args.duration is not None
        else (420.0 if args.mode == "lap" else 20.0)
    )

    if args.mode == "lap":
        resume_tuning = _load_tuning_state(args.resume_tuning)
        mpc_parameters = _load_mpc_parameters(args.mpc_model)
        fast_lane = resolve_fast_lane(
            state.track,
            state.track_configuration,
            args.fast_lane,
        )
        path = TrackPath.load(
            fast_lane,
            centerline=True,
            preserve_fast_lane=args.racing_line == "optimized",
            lateral_offset_m=args.lateral_offset,
            max_speed_kmh=args.max_speed_kmh,
            min_speed_kmh=args.min_speed_kmh,
            lateral_accel_mps2=args.lateral_accel,
            accel_mps2=args.longitudinal_accel,
            brake_mps2=args.braking_accel,
        )
        if args.path_resample_spacing_m > 0.0:
            path = path.resampled(args.path_resample_spacing_m)
            print(
                "Resampled path: "
                f"spacing={args.path_resample_spacing_m:.2f}m "
                f"points={len(path.points)}"
            )
        if args.line_target_clearance_m > 0.0:
            path = path.with_minimum_edge_clearance(
                args.line_target_clearance_m,
                max_speed_kmh=args.max_speed_kmh,
                min_speed_kmh=args.min_speed_kmh,
                lateral_accel_mps2=args.lateral_accel,
                accel_mps2=args.longitudinal_accel,
                brake_mps2=args.braking_accel,
            )
            print(
                "Line clearance projection: "
                f"target={args.line_target_clearance_m:.2f}m"
            )
        if (
            args.braking_lead_distance_m > 0.0
            or args.braking_lead_speed_gain_m_per_kmh > 0.0
        ):
            path = path.with_late_braking(
                max_speed_kmh=args.max_speed_kmh,
                min_speed_kmh=args.min_speed_kmh,
                lateral_accel_mps2=args.lateral_accel,
                accel_mps2=args.longitudinal_accel,
                brake_mps2=args.braking_accel,
                brake_point_shift_base_m=(
                    args.braking_lead_distance_m
                ),
                brake_point_shift_speed_gain_m_per_kmh=(
                    args.braking_lead_speed_gain_m_per_kmh
                ),
                brake_point_shift_speed_min_kmh=(
                    args.braking_lead_speed_min_kmh
                ),
                brake_point_shift_speed_max_kmh=(
                    args.braking_lead_speed_max_kmh
                ),
                entry_lateral_accel_boost=(
                    args.entry_lateral_accel_boost
                ),
                use_friction_ellipse=args.friction_ellipse_planning,
            )
            print(
                "Late braking envelope: "
                f"base={args.braking_lead_distance_m:.1f}m "
                f"gain={args.braking_lead_speed_gain_m_per_kmh:.3f}"
                "m/kmh "
                f"estimated_lap={path.estimated_lap_s:.1f}s"
            )
        if args.speed_envelope is not None:
            envelope_payload = json.loads(
                args.speed_envelope.read_text(encoding="utf-8")
            )
            envelope_speeds = envelope_payload.get("speed_kmh")
            if not isinstance(envelope_speeds, list):
                raise RuntimeError(
                    "Speed envelope must contain a speed_kmh list"
                )
            path = path.with_speed_envelope(
                [
                    float(value) * args.speed_envelope_scale
                    for value in envelope_speeds
                ],
                blend=args.speed_envelope_blend,
                maximum_speed_kmh=args.max_speed_kmh,
                only_increase=args.speed_envelope_only_increase,
            )
            print(
                "Manual speed envelope: "
                f"scale={args.speed_envelope_scale:.2f} "
                f"blend={args.speed_envelope_blend:.2f} "
                f"estimated_lap={path.estimated_lap_s:.1f}s"
            )
        base_path: TrackPath | None = path
        if args.use_merge_plan and args.lateral_offset is None:
            car_heading = physics_heading_to_world(state.heading_rad)
            path = path.plan_merge(
                car_x=state.position[0],
                car_z=state.position[2],
                car_heading_rad=car_heading,
                merge_distance_m=args.merge_distance,
                merge_speed_kmh=args.merge_speed_kmh,
            )
            base_path = None
        elif (
            args.startup_merge_distance > 0.0
            and args.lateral_offset is None
        ):
            path = path.with_start_speed_limit(
                car_x=state.position[0],
                car_z=state.position[2],
                distance_m=args.startup_merge_distance,
                speed_kmh=args.merge_speed_kmh,
                taper_distance_m=args.startup_ramp_distance,
            )
        print(
            f"Loaded racing line: {fast_lane} "
            f"length={path.total_length_m:.1f}m "
            f"estimated_lap={path.estimated_lap_s:.1f}s"
        )
        if args.lateral_controller == "lmpc":
            mpc_config = MpcConfig(
                horizon=max(5, args.mpc_horizon),
                dt=0.02,
            )
            if mpc_parameters is not None:
                mpc_config.parameters = mpc_parameters
            _apply_saved_lmpc_parameters(
                mpc_config,
                resume_tuning.get("controller", {}),
            )
            lap_controller = LinearMpcController(mpc_config)
        elif args.lateral_controller == "nmpc":
            nmpc_config = MpcConfig(
                horizon=max(5, args.mpc_horizon),
                dt=0.02,
            )
            if mpc_parameters is not None:
                nmpc_config.parameters = mpc_parameters
            lap_controller = NonlinearMpcController(nmpc_config)
        else:
            stanley_config = StanleyConfig(
                    merge_distance_m=args.merge_distance,
                    steering_sign=args.steer_sign,
                    heading_sign=args.heading_sign,
            )
            _apply_saved_stanley_parameters(
                stanley_config,
                resume_tuning.get("controller", {}),
            )
            lap_controller = StanleyController(stanley_config)
        lap_controller.calibrate(state, path)
        if args.longitudinal_controller == "mpc":
            longitudinal_config = LongitudinalMpcConfig(
                horizon=max(20, args.longitudinal_horizon),
                dt=max(0.02, args.longitudinal_dt),
            )
            _apply_saved_longitudinal_parameters(
                longitudinal_config,
                resume_tuning.get("longitudinal_controller", {}),
            )
            longitudinal_mpc = LongitudinalMpcController(
                longitudinal_config
            )
        else:
            longitudinal_mpc = None
        longitudinal_pedal_mapper = LongitudinalPedalMapper(
            LongitudinalPedalConfig(
                acceleration_deadband_mps2=0.80,
                throttle_rate_up_per_s=3.0,
                brake_rate_up_per_s=4.0,
            )
        )
        control_laps = max(0, args.control_laps)
        planning_laps = max(0, args.planning_laps)
        lap_limit = (
            max(1, control_laps + planning_laps)
            if planning_laps > 0
            else max(1, args.laps)
        )
        planning_tuner: PlanningLineTuner | None = None
        planning_active = planning_laps > 0
        if args.racing_line == "optimized" and planning_laps <= 0:
            clearance_values = [
                min(path.edge_clearance(index))
                for index in range(len(path.points))
            ]
            print(
                "Using fast_lane.ai racing line: "
                f"min_clearance={min(clearance_values):.2f}m "
                f"soft_1m_points="
                f"{sum(value < 1.0 for value in clearance_values)}"
            )
        if planning_active:
            planning_reference = base_path or path
            optimized_line = args.racing_line == "optimized"
            planning_config = PlanningConfig(
                initial_blend=(
                    1.0
                    if optimized_line
                    else args.planning_initial_blend
                ),
                minimum_blend=(
                    1.0 if optimized_line else 0.15
                ),
                maximum_blend=(
                    1.0
                    if optimized_line
                    else args.planning_max_blend
                ),
                stable_blend_step=(
                    0.0
                    if optimized_line
                    else args.planning_blend_step
                ),
                geometry_target_clearance_m=(
                    args.planning_target_clearance_m
                ),
                geometry_hard_clearance_m=(
                    args.planning_hard_clearance_m
                ),
                clearance_penalty_weight=(
                    args.planning_clearance_weight
                ),
                optimization_control_nodes=(
                    args.planning_optimization_nodes
                ),
                max_speed_kmh=args.max_speed_kmh,
                min_speed_kmh=args.min_speed_kmh,
                lateral_accel_mps2=args.lateral_accel,
                accel_mps2=args.longitudinal_accel,
                brake_mps2=args.braking_accel,
            )
            planning_tuner = PlanningLineTuner(
                planning_reference,
                planning_config,
            )
            path = planning_tuner.current_path
            lap_controller.calibrate(state, path)
            lap_controller.invalidate_plan_cache()
            print(
                f"Racing line={args.racing_line} "
                f"control_laps={control_laps} "
                f"planning_laps={planning_laps} "
                f"blend={planning_config.initial_blend:.2f} "
                f"planned_lap={planning_tuner.current_path.estimated_lap_s:.1f}s"
            )
            if planning_laps <= 0:
                planning_tuner = None
        speed_controller = SpeedController(
            SpeedControllerConfig(
                target_kmh=path.speed_limit_kmh[
                    lap_controller.previous_index
                ]
            )
        )
        initial_speed_scales = list(
            resume_tuning.get(
                "speed_scale_by_band",
                (1.0, 1.0, 1.0, 1.0),
            )
        )
        initial_braking = list(
            resume_tuning.get(
                "braking_accel_by_band",
                (
                    args.braking_accel,
                    args.braking_accel,
                    args.braking_accel,
                    args.braking_accel,
                ),
            )
        )
        if args.log is not None:
            _write_run_metadata(
                args.log,
                args=args,
                fast_lane_path=fast_lane,
                model_path=args.mpc_model,
                tuning_path=args.resume_tuning,
                controller_snapshot=_controller_parameter_snapshot(
                    lap_controller,
                    speed_controller,
                    initial_speed_scales,
                    initial_braking,
                    longitudinal_mpc,
                    longitudinal_pedal_mapper,
                ),
            )

        print(
            f"Arming lap controller in {args.arm_delay:.1f}s. "
            "Press Ctrl+C to stop."
        )
        _delay_with_zero_input(backend, args.arm_delay)
        return run_lap_loop(
            reader=reader,
            backend=backend,
            speed_controller=speed_controller,
            lap_controller=lap_controller,
            path=path,
            duration=duration,
            hz=args.hz,
            log_path=args.log,
            offtrack_limit_m=args.offtrack_limit_m,
            start_speed_kmh=args.start_speed_kmh,
            target_rise_kmh_s=args.target_rise_kmh_s,
            target_fall_kmh_s=(
                args.target_fall_kmh_s
                if args.target_fall_kmh_s is not None
                else args.braking_accel * 3.6
            ),
            lap_limit=lap_limit,
            autotune=args.autotune,
            control_laps=control_laps,
            planning_tuner=planning_tuner,
            target_lap_s=args.target_lap_s,
            speed_scale_step=args.speed_scale_step,
            max_speed_scale=args.max_speed_scale,
            count_from_start=args.count_from_start,
            base_lateral_accel_mps2=args.lateral_accel,
            scaled_speed_max_lateral_accel_mps2=(
                args.scaled_speed_max_lateral_g * 9.80665
                if args.scaled_speed_max_lateral_g > 0.0
                else 0.0
            ),
            base_longitudinal_accel_mps2=args.longitudinal_accel,
            base_max_speed_kmh=args.max_speed_kmh,
            base_braking_accel_mps2=args.braking_accel,
            braking_accel_step_mps2=args.braking_accel_step,
            max_braking_accel_mps2=args.max_braking_accel,
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
            longitudinal_mpc=longitudinal_mpc,
            longitudinal_pedal_mapper=longitudinal_pedal_mapper,
            initial_speed_scale_by_band=initial_speed_scales,
            initial_braking_accel_by_band=initial_braking,
            initial_controller_parameters={
                **resume_tuning.get("controller", {}),
                **resume_tuning.get("speed_controller", {}),
            },
            initial_longitudinal_parameters=resume_tuning.get(
                "longitudinal_controller",
                {},
            ),
            base_path=base_path,
        )

    speed_controller = SpeedController(
        SpeedControllerConfig(target_kmh=args.target_kmh)
    )
    heading_controller = HeadingController(
        HeadingControllerConfig(
            kp=args.heading_kp,
            kd=args.heading_kd,
            steer_sign=args.steer_sign,
        ),
        target_heading_rad=(
            math.radians(args.heading_deg)
            if args.heading_deg is not None
            else None
        ),
    )

    if args.focus_ac:
        if not focus_ac_window():
            print("Warning: could not focus the Assetto Corsa window.", file=sys.stderr)

    print(
        f"Arming {args.mode} controller in {args.arm_delay:.1f}s. "
        "Press Ctrl+C to stop."
    )
    _delay_with_zero_input(backend, args.arm_delay)

    return run_control_loop(
        reader=reader,
        backend=backend,
        speed_controller=speed_controller,
        heading_controller=heading_controller,
        mode=args.mode,
        target_kmh=args.target_kmh,
        duration=duration,
        hz=args.hz,
        log_path=args.log,
    )


def run_monitor(reader: SharedMemoryReader, args: argparse.Namespace) -> int:
    started = time.monotonic()
    next_print = 0.0
    last_packet_id = reader.snapshot().packet_id
    last_packet_change = started
    try:
        while True:
            state = reader.snapshot()
            now = time.monotonic()
            if state.packet_id != last_packet_id:
                last_packet_id = state.packet_id
                last_packet_change = now
            if now >= next_print:
                stale_for = now - last_packet_change
                suffix = " STALE" if stale_for > 1.0 else ""
                print(f"{format_state(state)}{suffix}")
                next_print = now + 0.25
            if args.duration > 0.0 and now - started >= args.duration:
                return 0
            time.sleep(0.01)
    except KeyboardInterrupt:
        return 0
    finally:
        reader.close()


def _delay_with_zero_input(backend: InputBackend, seconds: float) -> None:
    deadline = time.monotonic() + max(0.0, seconds)
    try:
        while time.monotonic() < deadline:
            backend.apply(0.0, 0.0, 0.0)
            time.sleep(0.01)
    except KeyboardInterrupt:
        backend.close()
        raise


def run_control_loop(
    reader: SharedMemoryReader,
    backend: InputBackend,
    speed_controller: SpeedController,
    heading_controller: HeadingController,
    mode: str,
    target_kmh: float,
    duration: float,
    hz: float,
    log_path: Path | None,
) -> int:
    period = 1.0 / max(1.0, hz)
    started = time.monotonic()
    next_tick = started
    next_print = started
    last_packet_id = -1
    last_packet_change = started
    log_handle = None
    summary_handle = None
    writer = None
    gear_engager = InitialGearEngager()

    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_handle, writer = log_writer(log_path)

    try:
        while True:
            now = time.monotonic()
            if duration > 0.0 and now - started >= duration:
                break

            state = reader.snapshot()
            dt = min(max(period, 0.001), 0.05)
            gear_engager.update(state, backend, now)

            if state.packet_id != last_packet_id:
                last_packet_id = state.packet_id
                last_packet_change = now
            stale_for = now - last_packet_change
            if stale_for > 1.0:
                backend.apply(0.0, 0.0, 0.0)
                if now >= next_print:
                    print(f"Telemetry stale for {stale_for:.1f}s; releasing controls.")
                    next_print = now + 0.5
                _sleep_until(next_tick + period)
                next_tick += period
                continue

            throttle, brake = speed_controller.step(state.speed_kmh, dt)
            steer = 0.0
            if mode == "heading":
                steer = heading_controller.step(
                    state.heading_rad,
                    state.yaw_rate_rad_s,
                    dt,
                )

            if state.is_ai_controlled:
                throttle = 0.0
                brake = 0.2
                steer = 0.0

            backend.apply(steer, throttle, brake)
            if writer is not None:
                write_log_row(writer, state, steer, throttle, brake)

            if now >= next_print:
                print(
                    f"{format_state(state)} "
                    f"cmd(throttle={throttle:.2f}, brake={brake:.2f}, steer={steer:+.2f})"
                )
                next_print = now + 0.25

            next_tick += period
            _sleep_until(next_tick)
    except KeyboardInterrupt:
        print("Stopping control loop.")
        return 0
    finally:
        backend.close()
        reader.close()
        if log_handle is not None:
            log_handle.close()

    print(f"Finished {mode} control loop (target {target_kmh:.1f} km/h).")
    return 0


def lap_log_writer(path: Path):
    handle = path.open("w", newline="", encoding="utf-8")
    fields = [
        "timestamp",
        "packet_id",
        "speed_kmh",
        "gas",
        "brake",
        "steer",
        "gear",
        "rpm",
        "heading_rad",
        "yaw_rate_rad_s",
        "position_x",
        "position_y",
        "position_z",
        "normalized_position",
        "tyres_out",
        "cmd_steer",
        "mpc_steer",
        "cmd_throttle",
        "cmd_brake",
        "target_speed_kmh",
        "path_index",
        "lookahead_index",
        "lookahead_m",
        "lateral_error_m",
        "heading_error_rad",
        "lap_number",
        "velocity_x",
        "velocity_y",
        "velocity_z",
        "local_velocity_x",
        "local_velocity_y",
        "local_velocity_z",
        "acceleration_g_x",
        "acceleration_g_y",
        "acceleration_g_z",
        "wheel_speed_fl",
        "wheel_speed_fr",
        "wheel_speed_rl",
        "wheel_speed_rr",
        "wheel_slip_fl",
        "wheel_slip_fr",
        "wheel_slip_rl",
        "wheel_slip_rr",
        "wheel_load_fl",
        "wheel_load_fr",
        "wheel_load_rl",
        "wheel_load_rr",
        "surface_grip",
        "mpc_solve_ms",
        "mpc_assembly_ms",
        "mpc_solver_ms",
        "mpc_iterations",
        "mpc_status",
        "longitudinal_accel_cmd",
        "longitudinal_accel_applied",
        "longitudinal_solve_ms",
        "longitudinal_iterations",
        "longitudinal_status",
    ]
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    return handle, writer


def write_lap_log_row(
    writer: csv.DictWriter,
    state: VehicleState,
    command: tuple[float, float, float],
    info: LapControlInfo,
    target_speed_kmh: float | None = None,
    lap_number: int = 1,
    lateral_mpc_stats=None,
    longitudinal_stats: LongitudinalMpcResult | None = None,
    applied_acceleration_mps2: float | None = None,
) -> None:
    steer, throttle, brake = command
    logged_target_speed = (
        info.target_speed_kmh
        if target_speed_kmh is None
        else target_speed_kmh
    )
    writer.writerow(
        {
            "timestamp": f"{state.timestamp:.6f}",
            "packet_id": state.packet_id,
            "speed_kmh": f"{state.speed_kmh:.6f}",
            "gas": f"{state.gas:.6f}",
            "brake": f"{state.brake:.6f}",
            "steer": f"{state.steer:.6f}",
            "gear": state.gear,
            "rpm": f"{state.rpm:.3f}",
            "heading_rad": f"{state.heading_rad:.9f}",
            "yaw_rate_rad_s": f"{state.yaw_rate_rad_s:.9f}",
            "position_x": f"{state.position[0]:.6f}",
            "position_y": f"{state.position[1]:.6f}",
            "position_z": f"{state.position[2]:.6f}",
            "normalized_position": f"{state.normalized_position:.9f}",
            "tyres_out": state.tyres_out,
            "cmd_steer": f"{steer:.6f}",
            "mpc_steer": (
                f"{info.raw_steer:.6f}"
                if info.raw_steer is not None
                else f"{steer:.6f}"
            ),
            "cmd_throttle": f"{throttle:.6f}",
            "cmd_brake": f"{brake:.6f}",
            "target_speed_kmh": f"{logged_target_speed:.6f}",
            "path_index": info.path_index,
            "lookahead_index": info.lookahead_index,
            "lookahead_m": f"{info.lookahead_m:.6f}",
            "lateral_error_m": f"{info.lateral_error_m:.6f}",
            "heading_error_rad": f"{info.heading_error_rad:.9f}",
            "lap_number": lap_number,
            "velocity_x": f"{state.velocity_m_s[0]:.9f}",
            "velocity_y": f"{state.velocity_m_s[1]:.9f}",
            "velocity_z": f"{state.velocity_m_s[2]:.9f}",
            "local_velocity_x": f"{state.local_velocity_m_s[0]:.9f}",
            "local_velocity_y": f"{state.local_velocity_m_s[1]:.9f}",
            "local_velocity_z": f"{state.local_velocity_m_s[2]:.9f}",
            "acceleration_g_x": f"{state.acceleration_g[0]:.9f}",
            "acceleration_g_y": f"{state.acceleration_g[1]:.9f}",
            "acceleration_g_z": f"{state.acceleration_g[2]:.9f}",
            "wheel_speed_fl": f"{state.wheel_angular_speed_rad_s[0]:.9f}",
            "wheel_speed_fr": f"{state.wheel_angular_speed_rad_s[1]:.9f}",
            "wheel_speed_rl": f"{state.wheel_angular_speed_rad_s[2]:.9f}",
            "wheel_speed_rr": f"{state.wheel_angular_speed_rad_s[3]:.9f}",
            "wheel_slip_fl": f"{state.tyre_slip[0]:.9f}",
            "wheel_slip_fr": f"{state.tyre_slip[1]:.9f}",
            "wheel_slip_rl": f"{state.tyre_slip[2]:.9f}",
            "wheel_slip_rr": f"{state.tyre_slip[3]:.9f}",
            "wheel_load_fl": f"{state.wheel_load_n[0]:.9f}",
            "wheel_load_fr": f"{state.wheel_load_n[1]:.9f}",
            "wheel_load_rl": f"{state.wheel_load_n[2]:.9f}",
            "wheel_load_rr": f"{state.wheel_load_n[3]:.9f}",
            "surface_grip": f"{state.surface_grip:.9f}",
            "mpc_solve_ms": (
                f"{lateral_mpc_stats.solve_ms:.6f}"
                if lateral_mpc_stats is not None
                else ""
            ),
            "mpc_assembly_ms": (
                f"{lateral_mpc_stats.assembly_ms:.6f}"
                if lateral_mpc_stats is not None
                else ""
            ),
            "mpc_solver_ms": (
                f"{lateral_mpc_stats.solver_ms:.6f}"
                if lateral_mpc_stats is not None
                else ""
            ),
            "mpc_iterations": (
                lateral_mpc_stats.iterations
                if lateral_mpc_stats is not None
                else ""
            ),
            "mpc_status": (
                lateral_mpc_stats.status
                if lateral_mpc_stats is not None
                else ""
            ),
            "longitudinal_accel_cmd": (
                f"{longitudinal_stats.acceleration_command_mps2:.6f}"
                if longitudinal_stats is not None
                else ""
            ),
            "longitudinal_accel_applied": (
                f"{applied_acceleration_mps2:.6f}"
                if applied_acceleration_mps2 is not None
                else ""
            ),
            "longitudinal_solve_ms": (
                f"{longitudinal_stats.solve_time_ms:.6f}"
                if longitudinal_stats is not None
                else ""
            ),
            "longitudinal_iterations": (
                longitudinal_stats.iterations
                if longitudinal_stats is not None
                else ""
            ),
            "longitudinal_status": (
                longitudinal_stats.status
                if longitudinal_stats is not None
                else ""
            ),
        }
    )


def run_lap_loop(
    reader: SharedMemoryReader,
    backend: InputBackend,
    speed_controller: SpeedController,
    lap_controller: PurePursuitController,
    path: TrackPath,
    duration: float,
    hz: float,
    log_path: Path | None,
    offtrack_limit_m: float,
    start_speed_kmh: float,
    target_rise_kmh_s: float,
    target_fall_kmh_s: float,
    lap_limit: int,
    autotune: bool,
    target_lap_s: float,
    speed_scale_step: float,
    max_speed_scale: float,
    count_from_start: bool,
    base_lateral_accel_mps2: float,
    scaled_speed_max_lateral_accel_mps2: float,
    base_longitudinal_accel_mps2: float,
    base_max_speed_kmh: float,
    base_braking_accel_mps2: float,
    braking_accel_step_mps2: float,
    max_braking_accel_mps2: float,
    braking_safety_factor: float,
    braking_lead_distance_m: float = 10.0,
    braking_lead_speed_gain_m_per_kmh: float = 0.08,
    braking_lead_speed_min_kmh: float = 50.0,
    braking_lead_speed_max_kmh: float = 300.0,
    control_laps: int = 0,
    planning_tuner: PlanningLineTuner | None = None,
    braking_response_time_s: float = 0.0,
    high_speed_braking_gain_s_per_kmh: float = 0.0,
    longitudinal_mpc: LongitudinalMpcController | None = None,
    longitudinal_pedal_mapper: LongitudinalPedalMapper | None = None,
    initial_speed_scale_by_band: list[float] | None = None,
    initial_braking_accel_by_band: list[float] | None = None,
    initial_controller_parameters: dict | None = None,
    initial_longitudinal_parameters: dict | None = None,
    base_path: TrackPath | None = None,
) -> int:
    period = 1.0 / max(1.0, hz)
    _apply_saved_speed_controller_parameters(
        speed_controller.config,
        initial_controller_parameters or {},
    )
    if longitudinal_mpc is not None:
        _apply_saved_longitudinal_parameters(
            longitudinal_mpc.config,
            initial_longitudinal_parameters or {},
        )
        longitudinal_mpc.rebuild_solver()
    started = time.monotonic()
    next_tick = started
    next_print = started
    last_packet_id = -1
    last_packet_change = started
    previous_normalized = reader.snapshot().normalized_position
    previous_speed_mps = reader.snapshot().speed_ms
    filtered_longitudinal_accel_mps2 = 0.0
    lap_started_at: float | None = started if count_from_start else None
    lap_started = count_from_start
    offtrack_since: float | None = None
    log_handle = None
    writer = None
    lap_completed = False
    gear_engager = InitialGearEngager()
    commanded_target_speed_kmh: float | None = None
    filtered_target_speed_kmh: float | None = None
    tyres_out_since: float | None = None
    lap_number = 1
    speed_scale_by_band = list(
        initial_speed_scale_by_band
        or [1.0] * SPEED_BAND_COUNT
    )
    band_count = len(speed_scale_by_band)
    braking_accel_by_band = list(
        initial_braking_accel_by_band
        or [base_braking_accel_mps2] * band_count
    )
    if not 1 <= band_count <= SPEED_BAND_COUNT:
        raise ValueError(
            "speed_scale_by_band must contain between 1 and "
            f"{SPEED_BAND_COUNT} values"
        )
    if len(braking_accel_by_band) != band_count:
        raise ValueError(
            "braking_accel_by_band must match speed_scale_by_band"
        )
    lap_max_lateral_error = 0.0
    lap_max_heading_error = 0.0
    lap_max_tyres_out = 0
    lap_lateral_errors: list[float] = []
    lap_heading_errors: list[float] = []
    band_lateral_errors: list[list[float]] = [
        [] for _ in range(band_count)
    ]
    band_heading_errors: list[list[float]] = [
        [] for _ in range(band_count)
    ]
    band_steer_delta_sum = [0.0] * band_count
    band_raw_steer_delta_sum = [0.0] * band_count
    band_sample_count = [0] * band_count
    band_max_tyres_out = [0] * band_count
    band_speed_errors: list[list[float]] = [
        [] for _ in range(band_count)
    ]
    band_lateral_g: list[list[float]] = [
        [] for _ in range(band_count)
    ]
    band_longitudinal_g: list[list[float]] = [
        [] for _ in range(band_count)
    ]
    previous_steer_command = 0.0
    previous_raw_steer_command = 0.0
    steer_delta_sum = 0.0
    steer_delta_count = 0
    planning_samples: list[PlanningSample] = []

    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_handle, writer = lap_log_writer(log_path)
        summary_handle = log_path.with_name(
            f"{log_path.stem}_laps.jsonl"
        ).open("w", encoding="utf-8")

    try:
        while True:
            now = time.monotonic()
            if duration > 0.0 and now - started >= duration:
                print("Lap time limit reached.")
                break

            state = reader.snapshot()
            dt = min(max(period, 0.001), 0.05)
            raw_longitudinal_accel_mps2 = (
                state.speed_ms - previous_speed_mps
            ) / dt
            filtered_longitudinal_accel_mps2 += (
                dt
                / (0.10 + dt)
                * (
                    raw_longitudinal_accel_mps2
                    - filtered_longitudinal_accel_mps2
                )
            )
            previous_speed_mps = state.speed_ms
            gear_engager.update(state, backend, now)

            if state.packet_id != last_packet_id:
                last_packet_id = state.packet_id
                last_packet_change = now
            stale_for = now - last_packet_change
            if stale_for > 1.0:
                backend.apply(0.0, 0.0, 0.0)
                previous_speed_mps = state.speed_ms
                _sleep_until(next_tick + period)
                next_tick += period
                continue

            info = lap_controller.step(state, path, dt)
            if (
                base_path is not None
                and path.fixed_speed_limit is not None
                and not path.fixed_speed_limit[info.path_index]
            ):
                path = base_path
                base_path = None
                lap_controller.calibrate(state, path)
                lap_controller.invalidate_plan_cache()
            speed_band = min(
                _speed_band(info.target_speed_kmh, band_count),
                band_count - 1,
            )
            speed_scale = speed_scale_by_band[speed_band]
            active_braking_accel = braking_accel_by_band[speed_band]
            target_speed_kmh = _scaled_path_speed_kmh(
                path,
                info.path_index,
                speed_scale_by_band,
                max_speed_kmh=base_max_speed_kmh,
                max_lateral_accel_mps2=(
                    scaled_speed_max_lateral_accel_mps2
                ),
            )
            braking_limited_speed = _braking_limited_speed_kmh(
                path,
                info.path_index,
                speed_scale_by_band,
                braking_accel_by_band,
                safety_factor=braking_safety_factor,
                response_time_s=braking_response_time_s,
                high_speed_response_gain_s_per_kmh=(
                    high_speed_braking_gain_s_per_kmh
                ),
                lead_distance_m=0.0,
                lead_speed_gain_m_per_kmh=0.0,
                lead_speed_min_kmh=braking_lead_speed_min_kmh,
                lead_speed_max_kmh=braking_lead_speed_max_kmh,
                max_speed_kmh=base_max_speed_kmh,
                max_lateral_accel_mps2=(
                    scaled_speed_max_lateral_accel_mps2
                ),
            )
            target_speed_kmh = min(
                target_speed_kmh,
                braking_limited_speed,
            )
            lap_max_lateral_error = max(
                lap_max_lateral_error,
                abs(info.lateral_error_m),
            )
            lap_lateral_errors.append(abs(info.lateral_error_m))
            lap_max_heading_error = max(
                lap_max_heading_error,
                abs(info.heading_error_rad),
            )
            lap_heading_errors.append(abs(info.heading_error_rad))
            lap_max_tyres_out = max(lap_max_tyres_out, state.tyres_out)
            band_lateral_errors[speed_band].append(
                abs(info.lateral_error_m)
            )
            band_heading_errors[speed_band].append(
                abs(info.heading_error_rad)
            )
            band_sample_count[speed_band] += 1
            band_max_tyres_out[speed_band] = max(
                band_max_tyres_out[speed_band],
                state.tyres_out,
            )
            band_speed_errors[speed_band].append(
                abs(state.speed_kmh - target_speed_kmh)
            )
            band_lateral_g[speed_band].append(
                abs(state.acceleration_g[0])
            )
            band_longitudinal_g[speed_band].append(
                abs(filtered_longitudinal_accel_mps2) / 9.81
            )
            band_steer_delta_sum[speed_band] += abs(
                info.steer - previous_steer_command
            )
            raw_steer = (
                info.steer
                if info.raw_steer is None
                else info.raw_steer
            )
            raw_steer_delta = abs(
                raw_steer - previous_raw_steer_command
            )
            band_raw_steer_delta_sum[speed_band] += raw_steer_delta
            steer_delta_sum += abs(info.steer - previous_steer_command)
            steer_delta_count += 1
            if (
                planning_tuner is not None
                and lap_number > control_laps
            ):
                left_clearance, right_clearance = path.edge_clearance(
                    info.path_index
                )
                planning_samples.append(
                    PlanningSample(
                        path_index=info.path_index,
                        lateral_error_m=info.lateral_error_m,
                        heading_error_rad=info.heading_error_rad,
                        lateral_g=abs(float(state.acceleration_g[0])),
                        longitudinal_g=(
                            abs(filtered_longitudinal_accel_mps2)
                            / 9.81
                        ),
                        tyres_out=state.tyres_out,
                        steer_delta=raw_steer_delta,
                        speed_error_kmh=abs(
                            state.speed_kmh - target_speed_kmh
                        ),
                        edge_clearance_m=min(
                            left_clearance,
                            right_clearance,
                        ),
                    )
                )
            previous_steer_command = info.steer
            previous_raw_steer_command = raw_steer
            if not lap_started:
                target_speed_kmh = min(
                    target_speed_kmh,
                    start_speed_kmh,
                )
            if commanded_target_speed_kmh is None:
                commanded_target_speed_kmh = target_speed_kmh
                filtered_target_speed_kmh = target_speed_kmh
            else:
                (
                    commanded_target_speed_kmh,
                    filtered_target_speed_kmh,
                ) = _next_target_speed(
                    current_kmh=commanded_target_speed_kmh,
                    filtered_kmh=filtered_target_speed_kmh,
                    requested_kmh=target_speed_kmh,
                    dt=dt,
                    rise_rate_kmh_s=target_rise_kmh_s,
                    fall_rate_kmh_s=target_fall_kmh_s,
                    rise_tau_s=(
                        longitudinal_mpc.config.reference_filter_up_tau_s
                        if longitudinal_mpc is not None
                        else 0.0
                    ),
                    fall_tau_s=(
                        longitudinal_mpc.config.reference_filter_down_tau_s
                        if longitudinal_mpc is not None
                        else 0.0
                    ),
                )
            target_speed_kmh = commanded_target_speed_kmh
            longitudinal_result = None
            applied_acceleration_mps2 = None
            if longitudinal_mpc is None:
                throttle, brake = speed_controller.step(
                    state.speed_kmh,
                    dt,
                    target_speed_kmh,
                )
            else:
                active_braking_limit = (
                    max(1.0, active_braking_accel)
                    * braking_safety_factor
                )
                load_adjusted_braking_limit = (
                    active_braking_limit
                    * _combined_braking_scale(
                        steering=info.steer,
                        lateral_g=abs(
                            float(state.acceleration_g[0])
                        ),
                    )
                )
                preview = _build_longitudinal_preview(
                    path,
                    info.path_index,
                    target_speed_kmh=target_speed_kmh,
                    speed_mps=state.speed_ms,
                    horizon=longitudinal_mpc.config.horizon,
                    dt=longitudinal_mpc.config.dt,
                    acceleration_limit_mps2=max(
                        0.5,
                        base_longitudinal_accel_mps2,
                    ),
                    braking_limit_mps2=active_braking_limit,
                    speed_scale_by_band=speed_scale_by_band,
                    steering=info.steer,
                    lateral_g=abs(
                        float(state.acceleration_g[0])
                    ),
                    max_speed_kmh=base_max_speed_kmh,
                    max_lateral_accel_mps2=(
                        scaled_speed_max_lateral_accel_mps2
                    ),
                )
                mpc_result = longitudinal_mpc.step(
                    speed_mps=state.speed_ms,
                    acceleration_mps2=(
                        float(state.acceleration_g[2]) * 9.81
                    ),
                    reference_speed_mps=preview,
                    acceleration_limit_mps2=max(
                        0.5,
                        base_longitudinal_accel_mps2,
                    ),
                    braking_limit_mps2=max(
                        1.0,
                        load_adjusted_braking_limit,
                    ),
                    lateral_accel_g=abs(
                        float(state.acceleration_g[0])
                    ),
                )
                longitudinal_result = mpc_result
                acceleration_command = (
                    mpc_result.acceleration_command_mps2
                )
                overspeed_mps = max(
                    0.0,
                    state.speed_ms - target_speed_kmh / 3.6,
                )
                overspeed_margin_mps = 5.0 / 3.6
                if overspeed_mps > overspeed_margin_mps:
                    guard_acceleration = -min(
                        6.0,
                        2.0
                        * (overspeed_mps - overspeed_margin_mps),
                    )
                    acceleration_command = min(
                        acceleration_command,
                        guard_acceleration,
                    )
                applied_acceleration_mps2 = acceleration_command
                throttle, brake = longitudinal_pedal_mapper.step(
                    acceleration_mps2=acceleration_command,
                    speed_mps=state.speed_ms,
                    steering=info.steer,
                    rear_slip=max(
                        abs(float(state.tyre_slip[2])),
                        abs(float(state.tyre_slip[3])),
                    ),
                    dt=dt,
                    acceleration_command_mps2=acceleration_command,
                    drag_acceleration_mps2=(
                        longitudinal_mpc.coast_down_acceleration(
                            state.speed_ms
                        )
                    ),
                )
            steer = info.steer

            if state.is_ai_controlled:
                print("Aborting: vehicle is AI controlled.")
                throttle, brake, steer = 0.0, 0.4, 0.0
                backend.apply(steer, throttle, brake)
                time.sleep(1.5)
                break

            if abs(info.lateral_error_m) > offtrack_limit_m:
                if offtrack_since is None:
                    offtrack_since = now
                elif now - offtrack_since > 3.0:
                    print(
                        "Aborting: lateral error "
                        f"{info.lateral_error_m:.1f}m exceeded "
                        f"{offtrack_limit_m:.1f}m."
                    )
                    throttle, brake, steer = 0.0, 0.8, 0.0
                    backend.apply(steer, throttle, brake)
                    time.sleep(2.0)
                    break
            else:
                offtrack_since = None

            if state.tyres_out >= 4:
                if tyres_out_since is None:
                    tyres_out_since = now
                elif now - tyres_out_since > 1.0:
                    print(
                        "Aborting: four tyres outside "
                        "the track for more than 1.0s."
                    )
                    throttle, brake, steer = 0.0, 0.8, 0.0
                    backend.apply(steer, throttle, brake)
                    time.sleep(2.0)
                    break
            else:
                tyres_out_since = None

            backend.apply(steer, throttle, brake)
            if writer is not None:
                write_lap_log_row(
                    writer,
                    state,
                    (steer, throttle, brake),
                    info,
                    target_speed_kmh,
                    lap_number,
                    lateral_mpc_stats=getattr(
                        lap_controller,
                        "last_stats",
                        None,
                    ),
                    longitudinal_stats=longitudinal_result,
                    applied_acceleration_mps2=(
                        applied_acceleration_mps2
                    ),
                )

            current_normalized = state.normalized_position
            crossed_line = previous_normalized > 0.90 and current_normalized < 0.10
            previous_normalized = current_normalized
            if crossed_line:
                if not lap_started:
                    lap_started = True
                    lap_started_at = now
                    print(f"Start line crossed. Lap timing started at {lap_started_at:.3f}.")
                elif lap_started_at is not None and now - lap_started_at > 60.0:
                    lap_time = now - lap_started_at
                    planning_update: PlanningUpdate | None = None
                    completed_lap_number = lap_number
                    completed_max_lateral_error = lap_max_lateral_error
                    completed_max_heading_error = lap_max_heading_error
                    completed_max_tyres_out = lap_max_tyres_out
                    p95_lateral_error = _percentile(
                        lap_lateral_errors,
                        0.95,
                    )
                    p95_heading_error = _percentile(
                        lap_heading_errors,
                        0.95,
                    )
                    band_p95_lateral_errors = tuple(
                        _percentile(values, 0.95)
                        for values in band_lateral_errors
                    )
                    band_p95_heading_errors = tuple(
                        _percentile(values, 0.95)
                        for values in band_heading_errors
                    )
                    band_steering_activity = tuple(
                        total / max(1, count)
                        for total, count in zip(
                            band_steer_delta_sum,
                            band_sample_count,
                        )
                    )
                    band_raw_steering_activity = tuple(
                        total / max(1, count)
                        for total, count in zip(
                            band_raw_steer_delta_sum,
                            band_sample_count,
                        )
                    )
                    band_p95_speed_errors = tuple(
                        _percentile(values, 0.95)
                        for values in band_speed_errors
                    )
                    band_p95_lateral_g = tuple(
                        _percentile(values, 0.95)
                        for values in band_lateral_g
                    )
                    band_p95_longitudinal_g = tuple(
                        _percentile(values, 0.95)
                        for values in band_longitudinal_g
                    )
                    steering_activity = steer_delta_sum / max(
                        1,
                        steer_delta_count,
                    )
                    effective_lateral_gs = [
                        (
                            base_lateral_accel_mps2
                            * scale
                            * scale
                            / 9.81
                        )
                        for scale in speed_scale_by_band
                    ]
                    effective_max_speeds = [
                        min(
                            base_max_speed_kmh,
                            base_max_speed_kmh * scale,
                        )
                        for scale in speed_scale_by_band
                    ]
                    effective_braking_gs = [
                        value / 9.81
                        for value in braking_accel_by_band
                    ]
                    print(
                        f"Lap {lap_number} complete: {lap_time:.1f}s "
                        f"max_lat={lap_max_lateral_error:.2f}m "
                        f"p95_lat={p95_lateral_error:.2f}m "
                        f"max_heading={lap_max_heading_error:.3f}rad "
                        f"p95_heading={p95_heading_error:.3f}rad "
                        f"tyres_out={lap_max_tyres_out} "
                        f"steer_activity={steering_activity:.4f} "
                        f"speed_scale={speed_scale_by_band} "
                        f"lat_limit_g={[round(v, 3) for v in effective_lateral_gs]} "
                        f"speed_limit_kmh={[round(v, 1) for v in effective_max_speeds]} "
                        f"brake_limit_g={[round(v, 3) for v in effective_braking_gs]}"
                        f" band_p95_lat={[round(v, 2) for v in band_p95_lateral_errors]}"
                        f" actual_lat_g={[round(v, 3) for v in band_p95_lateral_g]}"
                        f" actual_long_g={[round(v, 3) for v in band_p95_longitudinal_g]}"
                    )
                    before_parameters = _controller_parameter_snapshot(
                        lap_controller,
                        speed_controller,
                        speed_scale_by_band,
                        braking_accel_by_band,
                        longitudinal_mpc,
                        longitudinal_pedal_mapper,
                    )
                    if autotune and lap_number < lap_limit:
                        control_tuning_active = (
                            control_laps <= 0
                            or completed_lap_number <= control_laps
                        )
                        if isinstance(lap_controller, StanleyController):
                            (
                                speed_scale_by_band,
                                lap_controller.config.cross_track_gain,
                                lap_controller.config.heading_gain,
                                lap_controller.config.yaw_rate_gain,
                                lap_controller.config.steering_filter_tau_s,
                                braking_accel_by_band,
                            ) = _next_lap_tuning(
                                lap_time_s=lap_time,
                                target_lap_s=target_lap_s,
                                max_lateral_error_m=lap_max_lateral_error,
                                max_heading_error_rad=lap_max_heading_error,
                                p95_lateral_error_m=p95_lateral_error,
                                p95_heading_error_rad=p95_heading_error,
                                band_p95_lateral_error_m=band_p95_lateral_errors,
                                band_p95_heading_error_rad=band_p95_heading_errors,
                                band_max_tyres_out=tuple(band_max_tyres_out),
                                band_steering_activity=band_steering_activity,
                                band_raw_steering_activity=(
                                    band_raw_steering_activity
                                ),
                                band_sample_count=tuple(band_sample_count),
                                max_tyres_out=lap_max_tyres_out,
                                steering_activity=steering_activity,
                                speed_scale_by_band=speed_scale_by_band,
                                cross_track_gain=lap_controller.config.cross_track_gain,
                                heading_gain=lap_controller.config.heading_gain,
                                yaw_rate_gain=lap_controller.config.yaw_rate_gain,
                                steering_filter_tau_s=(
                                    lap_controller.config.steering_filter_tau_s
                                ),
                                braking_accel_by_band=braking_accel_by_band,
                                speed_scale_step=speed_scale_step,
                                max_speed_scale=max_speed_scale,
                                braking_accel_step_mps2=braking_accel_step_mps2,
                                max_braking_accel_mps2=max_braking_accel_mps2,
                            )
                            _tune_gain_maps(
                                lap_controller.config,
                                band_p95_lateral_errors,
                                band_steering_activity,
                                band_sample_count,
                            )
                        elif isinstance(
                            lap_controller,
                            LinearMpcController,
                        ):
                            tuning_config = (
                                lap_controller.config
                                if control_tuning_active
                                else copy.deepcopy(lap_controller.config)
                            )
                            (
                                speed_scale_by_band,
                                braking_accel_by_band,
                            ) = _next_lmpc_tuning(
                                lap_time_s=lap_time,
                                target_lap_s=target_lap_s,
                                p95_lateral_error_m=p95_lateral_error,
                                p95_heading_error_rad=p95_heading_error,
                                steering_activity=steering_activity,
                                band_raw_steering_activity=(
                                    band_raw_steering_activity
                                ),
                                band_p95_lateral_error_m=band_p95_lateral_errors,
                                band_p95_heading_error_rad=band_p95_heading_errors,
                                band_max_tyres_out=tuple(band_max_tyres_out),
                                band_sample_count=tuple(band_sample_count),
                                band_p95_lateral_g=band_p95_lateral_g,
                                band_p95_longitudinal_g=(
                                    band_p95_longitudinal_g
                                ),
                                speed_scale_by_band=speed_scale_by_band,
                                braking_accel_by_band=braking_accel_by_band,
                                base_lateral_accel_mps2=(
                                    base_lateral_accel_mps2
                                ),
                                max_speed_scale=max_speed_scale,
                                max_braking_accel_mps2=(
                                    max_braking_accel_mps2
                                ),
                                mpc_config=tuning_config,
                            )
                        elif autotune:
                            raise RuntimeError(
                                "Autotune supports Stanley and linear MPC."
                            )
                        if (
                            longitudinal_mpc is not None
                            and control_tuning_active
                        ):
                            _tune_longitudinal_mpc(
                                longitudinal_mpc,
                                band_p95_speed_errors,
                                tuple(band_sample_count),
                                band_p95_longitudinal_g,
                                tuple(band_max_tyres_out),
                                max_brake_limit_mps2=(
                                    max_braking_accel_mps2
                                ),
                            )
                        else:
                            _tune_speed_controller(
                                speed_controller.config,
                                band_p95_speed_errors,
                                band_sample_count,
                                band_p95_longitudinal_g,
                            )
                        if (
                            isinstance(
                                lap_controller,
                                LinearMpcController,
                            )
                            and control_tuning_active
                        ):
                            lap_controller.invalidate_plan_cache()
                        if planning_tuner is not None:
                            if completed_lap_number == control_laps:
                                path = planning_tuner.current_path
                                lap_controller.calibrate(state, path)
                                lap_controller.invalidate_plan_cache()
                                print(
                                    "Planning phase started: "
                                    "installed conservative optimized line "
                                    f"estimated_lap="
                                    f"{path.estimated_lap_s:.1f}s"
                                )
                            elif completed_lap_number > control_laps:
                                planning_update = planning_tuner.update(
                                    planning_samples,
                                    lap_time_s=lap_time,
                                )
                                path = planning_tuner.current_path
                                lap_controller.calibrate(state, path)
                                lap_controller.invalidate_plan_cache()
                                print(
                                    "Planning update "
                                    f"{planning_update.planning_lap_number}: "
                                    f"stable={planning_update.stable_corners} "
                                    f"caution={planning_update.caution_corners} "
                                    f"unsafe={planning_update.unsafe_corners} "
                                    f"accepted={planning_update.accepted} "
                                    f"estimated_lap="
                                    f"{planning_update.estimated_lap_after_s:.1f}s"
                                )
                        print(
                            f"Autotune next lap {lap_number + 1}: "
                            f"speed_scale={speed_scale_by_band} "
                            f"mpc={_summarize_lmpc_parameters(lap_controller)} "
                            f"brake_accel={braking_accel_by_band}"
                        )
                        lap_number += 1
                        lap_started_at = now
                        lap_max_lateral_error = 0.0
                        lap_max_heading_error = 0.0
                        lap_max_tyres_out = 0
                        lap_lateral_errors.clear()
                        lap_heading_errors.clear()
                        for values in band_lateral_errors:
                            values.clear()
                        for values in band_heading_errors:
                            values.clear()
                        for values in band_speed_errors:
                            values.clear()
                        for values in band_lateral_g:
                            values.clear()
                        for values in band_longitudinal_g:
                            values.clear()
                        band_steer_delta_sum = [0.0] * band_count
                        band_raw_steer_delta_sum = [0.0] * band_count
                        band_sample_count = [0] * band_count
                        band_max_tyres_out = [0] * band_count
                        steer_delta_sum = 0.0
                        steer_delta_count = 0
                        previous_raw_steer_command = 0.0
                        planning_samples.clear()
                        after_parameters = _controller_parameter_snapshot(
                            lap_controller,
                            speed_controller,
                            speed_scale_by_band,
                            braking_accel_by_band,
                            longitudinal_mpc,
                            longitudinal_pedal_mapper,
                        )
                        _write_lap_summary(
                            summary_handle,
                            lap_number=completed_lap_number,
                            lap_time_s=lap_time,
                            max_lateral_error_m=completed_max_lateral_error,
                            max_heading_error_rad=completed_max_heading_error,
                            p95_lateral_error_m=p95_lateral_error,
                            p95_heading_error_rad=p95_heading_error,
                            max_tyres_out=completed_max_tyres_out,
                            steering_activity=steering_activity,
                            effective_lateral_g=effective_lateral_gs,
                            effective_max_speed_kmh=effective_max_speeds,
                            braking_accel_mps2=braking_accel_by_band,
                            band_p95_lateral_g=list(
                                band_p95_lateral_g
                            ),
                            band_p95_longitudinal_g=list(
                                band_p95_longitudinal_g
                            ),
                            band_p95_speed_error_kmh=list(
                                band_p95_speed_errors
                            ),
                            before_parameters=before_parameters,
                            after_parameters=after_parameters,
                            planning_update=planning_update,
                        )
                        continue
                    if not autotune and lap_number < lap_limit:
                        _write_lap_summary(
                            summary_handle,
                            lap_number=completed_lap_number,
                            lap_time_s=lap_time,
                            max_lateral_error_m=completed_max_lateral_error,
                            max_heading_error_rad=completed_max_heading_error,
                            p95_lateral_error_m=p95_lateral_error,
                            p95_heading_error_rad=p95_heading_error,
                            max_tyres_out=completed_max_tyres_out,
                            steering_activity=steering_activity,
                            effective_lateral_g=effective_lateral_gs,
                            effective_max_speed_kmh=effective_max_speeds,
                            braking_accel_mps2=braking_accel_by_band,
                            band_p95_lateral_g=list(
                                band_p95_lateral_g
                            ),
                            band_p95_longitudinal_g=list(
                                band_p95_longitudinal_g
                            ),
                            band_p95_speed_error_kmh=list(
                                band_p95_speed_errors
                            ),
                            before_parameters=before_parameters,
                            after_parameters=before_parameters,
                        )
                        print(
                            f"Fixed-config lap {completed_lap_number} "
                            f"complete; continuing to lap "
                            f"{lap_number + 1}."
                        )
                        lap_number += 1
                        lap_started_at = now
                        lap_max_lateral_error = 0.0
                        lap_max_heading_error = 0.0
                        lap_max_tyres_out = 0
                        lap_lateral_errors.clear()
                        lap_heading_errors.clear()
                        for values in band_lateral_errors:
                            values.clear()
                        for values in band_heading_errors:
                            values.clear()
                        for values in band_speed_errors:
                            values.clear()
                        for values in band_lateral_g:
                            values.clear()
                        for values in band_longitudinal_g:
                            values.clear()
                        band_steer_delta_sum = [0.0] * band_count
                        band_raw_steer_delta_sum = [0.0] * band_count
                        band_sample_count = [0] * band_count
                        band_max_tyres_out = [0] * band_count
                        steer_delta_sum = 0.0
                        steer_delta_count = 0
                        previous_raw_steer_command = 0.0
                        planning_samples.clear()
                        continue
                    _write_lap_summary(
                        summary_handle,
                        lap_number=completed_lap_number,
                        lap_time_s=lap_time,
                        max_lateral_error_m=completed_max_lateral_error,
                        max_heading_error_rad=completed_max_heading_error,
                        p95_lateral_error_m=p95_lateral_error,
                        p95_heading_error_rad=p95_heading_error,
                        max_tyres_out=completed_max_tyres_out,
                        steering_activity=steering_activity,
                        effective_lateral_g=effective_lateral_gs,
                        effective_max_speed_kmh=effective_max_speeds,
                        braking_accel_mps2=braking_accel_by_band,
                        band_p95_lateral_g=list(band_p95_lateral_g),
                        band_p95_longitudinal_g=list(
                            band_p95_longitudinal_g
                        ),
                        band_p95_speed_error_kmh=list(
                            band_p95_speed_errors
                        ),
                        before_parameters=before_parameters,
                        after_parameters=before_parameters,
                    )
                    lap_completed = True
                    backend.apply(0.0, 0.0, 0.8)
                    time.sleep(2.5)
                    return 0

            if now >= next_print:
                lap_text = (
                    f"lap_time={now - lap_started_at:6.1f}s"
                    if lap_started_at is not None
                    else "lap_time=waiting"
                )
                print(
                    f"{format_state(state)} "
                    f"target={target_speed_kmh:5.1f} "
                    f"lat_error={info.lateral_error_m:+5.2f}m "
                    f"heading_error={info.heading_error_rad:+.3f} "
                    f"{lap_text} "
                    f"cmd(throttle={throttle:.2f}, brake={brake:.2f}, "
                    f"steer={steer:+.2f})"
                )
                next_print = now + 0.5

            next_tick += period
            _sleep_until(next_tick)
    except KeyboardInterrupt:
        print("Stopping lap controller.")
        return 0
    finally:
        if not lap_completed:
            _bring_vehicle_to_stop(reader, backend)
        backend.close()
        reader.close()
        if log_handle is not None:
            log_handle.close()
        if summary_handle is not None:
            summary_handle.close()

    return 0


def _bring_vehicle_to_stop(
    reader: SharedMemoryReader,
    backend: InputBackend,
    max_seconds: float = 5.0,
) -> None:
    deadline = time.monotonic() + max(0.0, max_seconds)
    while time.monotonic() < deadline:
        try:
            state = reader.snapshot()
        except SharedMemoryUnavailable:
            return
        if state.speed_kmh < 1.0:
            break
        backend.apply(0.0, 0.0, 0.8)
        time.sleep(0.05)
    backend.apply(0.0, 0.0, 0.0)


def _sha256_file(path: Path | None) -> str | None:
    if path is None or not path.exists():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json_safe(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {
            str(key): _json_safe(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _write_run_metadata(
    log_path: Path,
    *,
    args: argparse.Namespace,
    fast_lane_path: Path,
    model_path: Path | None,
    tuning_path: Path | None,
    controller_snapshot: dict,
) -> None:
    root = Path(__file__).resolve().parents[1]
    setup_path = (
        Path.home()
        / "Documents"
        / "Assetto Corsa"
        / "setups"
        / "uk_bmw_m4_g82"
        / "generic"
        / "last.ini"
    )
    metadata = {
        "created_at_unix": time.time(),
        "arguments": _json_safe(vars(args)),
        "hashes": {
            "cli.py": _sha256_file(root / "actc" / "cli.py"),
            "mpc.py": _sha256_file(root / "actc" / "mpc.py"),
            "track.py": _sha256_file(root / "actc" / "track.py"),
            "controllers.py": _sha256_file(
                root / "actc" / "controllers.py"
            ),
            "longitudinal.py": _sha256_file(
                root / "actc" / "longitudinal.py"
            ),
            "fast_lane.ai": _sha256_file(fast_lane_path),
            "mpc_model": _sha256_file(model_path),
            "tuning_config": _sha256_file(tuning_path),
            "vehicle_setup": _sha256_file(setup_path),
        },
        "effective_controller": _json_safe(controller_snapshot),
    }
    metadata_path = log_path.with_name(
        f"{log_path.stem}_run_metadata.json"
    )
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )
def _scaled_path_speed_kmh(
    path: TrackPath,
    index: int,
    speed_scale_by_band: list[float],
    max_speed_kmh: float | None = None,
    max_lateral_accel_mps2: float | None = None,
) -> float:
    base_speed = path.speed_limit_kmh[index]
    if (
        path.fixed_speed_limit is not None
        and path.fixed_speed_limit[index]
    ):
        scaled_speed = base_speed
    else:
        scaled_speed = base_speed * _speed_scale_kmh(
            base_speed,
            speed_scale_by_band,
        )
    if (
        max_lateral_accel_mps2 is not None
        and max_lateral_accel_mps2 > 0.0
    ):
        curvature = abs(path.curvature[index])
        if curvature > 1e-7:
            scaled_speed = min(
                scaled_speed,
                math.sqrt(max_lateral_accel_mps2 / curvature) * 3.6,
            )
    if max_speed_kmh is None:
        return scaled_speed
    return min(scaled_speed, max(0.0, max_speed_kmh))


def _filter_target_speed(
    current_kmh: float,
    desired_kmh: float,
    *,
    dt: float,
    rise_tau_s: float,
    fall_tau_s: float,
) -> float:
    tau_s = (
        rise_tau_s
        if desired_kmh >= current_kmh
        else fall_tau_s
    )
    alpha = dt / (max(0.0, tau_s) + dt)
    return current_kmh + alpha * (desired_kmh - current_kmh)


def _next_target_speed(
    *,
    current_kmh: float | None,
    filtered_kmh: float | None,
    requested_kmh: float,
    dt: float,
    rise_rate_kmh_s: float,
    fall_rate_kmh_s: float,
    rise_tau_s: float,
    fall_tau_s: float,
) -> tuple[float, float]:
    if current_kmh is None:
        return requested_kmh, requested_kmh
    filtered = (
        requested_kmh
        if filtered_kmh is None
        else _filter_target_speed(
            filtered_kmh,
            requested_kmh,
            dt=dt,
            rise_tau_s=rise_tau_s,
            fall_tau_s=fall_tau_s,
        )
    )
    rate_limit = (
        max(0.0, rise_rate_kmh_s)
        if filtered >= current_kmh
        else max(0.0, fall_rate_kmh_s)
    )
    max_delta = rate_limit * dt
    next_speed = current_kmh + clamp(
        filtered - current_kmh,
        -max_delta,
        max_delta,
    )
    return next_speed, filtered


def _build_longitudinal_preview(
    path: TrackPath,
    index: int,
    *,
    target_speed_kmh: float,
    speed_mps: float,
    horizon: int,
    dt: float,
    acceleration_limit_mps2: float,
    braking_limit_mps2: float,
    speed_scale_by_band: list[float],
    steering: float,
    lateral_g: float,
    max_speed_kmh: float | None = None,
    max_lateral_accel_mps2: float | None = None,
) -> list[float]:
    current_reference = max(0.5, target_speed_kmh / 3.6)
    preview = [current_reference]
    acceleration_step = max(0.5, acceleration_limit_mps2) * dt
    effective_braking_limit = (
        max(1.0, braking_limit_mps2)
        * _combined_braking_scale(
            steering=steering,
            lateral_g=lateral_g,
        )
    )
    ahead_indices = [index]
    for stage in range(1, horizon + 1):
        ahead_index = path.lookahead_index(
            index,
            max(0.0, speed_mps) * dt * stage,
        )
        ahead_indices.append(ahead_index)
        raw_reference = (
            _scaled_path_speed_kmh(
                path,
                ahead_index,
                speed_scale_by_band,
                max_speed_kmh=max_speed_kmh,
                max_lateral_accel_mps2=max_lateral_accel_mps2,
            )
            / 3.6
        )
        raw_reference = min(current_reference, raw_reference)
        preview.append(
            min(
                raw_reference,
                preview[-1] + acceleration_step,
            )
        )

    # A downstream speed drop also constrains every earlier point. This keeps
    # a local path-speed bump inside a braking zone from becoming a
    # release-and-reapply pedal sequence.
    for stage in range(horizon - 1, -1, -1):
        previous_index = ahead_indices[stage]
        next_index = ahead_indices[stage + 1]
        distance_m = (
            path.cumulative_s[next_index]
            - path.cumulative_s[previous_index]
        )
        if distance_m < 0.0:
            distance_m += path.total_length_m
        distance_m = max(0.0, distance_m)
        previous_limit = math.sqrt(
            preview[stage + 1] ** 2
            + 2.0 * effective_braking_limit * distance_m
        )
        preview[stage] = min(preview[stage], previous_limit)

    # Make the reference executable from the current speed. A chain of speed
    # limits alone lets the MPC delay braking because it can always satisfy
    # the final preview point with a short, hard stop. A continuous
    # acceleration profile keeps the braking intent connected across stages.
    limit_preview = list(preview)
    minimum_stage = min(
        range(horizon + 1),
        key=lambda stage: limit_preview[stage],
    )
    terminal_reference = limit_preview[minimum_stage]
    braking_time_s = max(dt * minimum_stage, 1e-3)
    required_acceleration = clamp(
        (terminal_reference - max(0.5, speed_mps)) / braking_time_s,
        -effective_braking_limit,
        max(0.5, acceleration_limit_mps2),
    )
    for stage in range(minimum_stage + 1):
        shaped_reference = max(
            0.5,
            speed_mps + required_acceleration * dt * stage,
        )
        preview[stage] = min(limit_preview[stage], shaped_reference)
    for stage in range(minimum_stage + 1, horizon + 1):
        preview[stage] = min(
            limit_preview[stage],
            preview[stage - 1]
            + max(0.5, acceleration_limit_mps2) * dt,
        )
    return preview


def _braking_limited_speed_kmh(
    path: TrackPath,
    index: int,
    speed_scale_by_band: list[float],
    braking_accel_by_band: list[float],
    *,
    safety_factor: float,
    response_time_s: float = 0.0,
    high_speed_response_gain_s_per_kmh: float = 0.0,
    lead_distance_m: float = 10.0,
    lead_speed_gain_m_per_kmh: float = 0.08,
    lead_speed_min_kmh: float = 50.0,
    lead_speed_max_kmh: float = 300.0,
    max_speed_kmh: float | None = None,
    max_lateral_accel_mps2: float | None = None,
) -> float:
    current_limit = _scaled_path_speed_kmh(
        path,
        index,
        speed_scale_by_band,
        max_speed_kmh=max_speed_kmh,
        max_lateral_accel_mps2=max_lateral_accel_mps2,
    )
    current_speed_ms = current_limit / 3.6
    lead_speed_kmh = clamp(
        current_limit,
        min(lead_speed_min_kmh, lead_speed_max_kmh),
        max(lead_speed_min_kmh, lead_speed_max_kmh),
    )
    lead_distance = (
        max(0.0, lead_distance_m)
        + max(0.0, lead_speed_gain_m_per_kmh)
        * lead_speed_kmh
    )
    response_time_s = max(0.0, response_time_s)
    response_time_s += (
        max(0.0, current_limit - 200.0)
        * max(0.0, high_speed_response_gain_s_per_kmh)
    )
    safety_factor = max(0.30, min(1.0, safety_factor))
    conservative_accel = max(
        1.0,
        min(braking_accel_by_band) * safety_factor,
    )
    horizon_m = min(
        600.0,
        max(
            80.0,
            current_speed_ms * current_speed_ms
            / (2.0 * conservative_accel)
            + 40.0,
        ),
    )
    start_s = path.cumulative_s[index]
    allowed_speed_kmh = current_limit
    count = len(path.points)

    for offset in range(1, count):
        ahead_index = (index + offset) % count
        ds = path.cumulative_s[ahead_index] - start_s
        if ds < 0.0:
            ds += path.total_length_m
        if ds > horizon_m:
            break
        base_speed = path.speed_limit_kmh[ahead_index]
        band = min(
            _speed_band(
                base_speed,
                len(braking_accel_by_band),
            ),
            len(braking_accel_by_band) - 1,
        )
        ahead_limit_kmh = _scaled_path_speed_kmh(
            path,
            ahead_index,
            speed_scale_by_band,
            max_speed_kmh=max_speed_kmh,
            max_lateral_accel_mps2=max_lateral_accel_mps2,
        )
        ahead_limit_ms = ahead_limit_kmh / 3.6
        braking_accel = max(
            1.0,
            braking_accel_by_band[band] * safety_factor,
        )
        effective_ds = max(
            0.0,
            ds
            - current_speed_ms * response_time_s
            + lead_distance,
        )
        candidate_kmh = math.sqrt(
            ahead_limit_ms * ahead_limit_ms
            + 2.0 * braking_accel * effective_ds
        ) * 3.6
        allowed_speed_kmh = min(allowed_speed_kmh, candidate_kmh)

    return max(0.0, allowed_speed_kmh)


def _combined_braking_scale(
    *,
    steering: float,
    lateral_g: float,
) -> float:
    steering_usage = min(1.0, abs(steering) / 0.52)
    lateral_usage = min(1.0, abs(lateral_g) / 0.90)
    combined_usage = max(steering_usage, lateral_usage)
    return math.sqrt(
        max(0.25, 1.0 - combined_usage * combined_usage)
    )


def _tune_gain_maps(
    controller_config,
    band_p95_lateral_error_m: tuple[float, ...],
    band_steering_activity: tuple[float, ...],
    band_sample_count: tuple[int, ...],
) -> None:
    cross_map = list(controller_config.cross_track_gain_map)
    heading_map = list(controller_config.heading_gain_map)
    yaw_map = list(controller_config.yaw_rate_gain_map)
    filter_map = list(controller_config.filter_tau_map)
    band_count = min(
        len(cross_map),
        len(heading_map),
        len(yaw_map),
        len(filter_map),
        len(band_p95_lateral_error_m),
        len(band_steering_activity),
        len(band_sample_count),
    )
    for band in range(band_count):
        if band_sample_count[band] < 30:
            continue
        lateral_error = band_p95_lateral_error_m[band]
        steering_activity = band_steering_activity[band]
        if lateral_error > 2.5:
            cross_map[band] = min(2.50, cross_map[band] * 1.12)
            heading_map[band] = min(2.00, heading_map[band] * 1.05)
            yaw_map[band] = min(2.00, yaw_map[band] * 1.10)
        elif lateral_error > 1.5:
            cross_map[band] = min(2.50, cross_map[band] * 1.06)
            heading_map[band] = min(2.00, heading_map[band] * 1.03)
        if steering_activity > 0.0020:
            filter_map[band] = min(2.20, filter_map[band] * 1.18)
            yaw_map[band] = min(2.00, yaw_map[band] * 1.10)
        elif lateral_error < 1.0 and steering_activity < 0.0008:
            filter_map[band] = max(0.70, filter_map[band] * 0.95)

    controller_config.cross_track_gain_map = tuple(cross_map)
    controller_config.heading_gain_map = tuple(heading_map)
    controller_config.yaw_rate_gain_map = tuple(yaw_map)
    controller_config.filter_tau_map = tuple(filter_map)


def _controller_parameter_snapshot(
    controller,
    speed_controller: SpeedController,
    speed_scale_by_band: list[float],
    braking_accel_by_band: list[float],
    longitudinal_mpc: LongitudinalMpcController | None = None,
    longitudinal_pedal_mapper: LongitudinalPedalMapper | None = None,
) -> dict:
    snapshot = {
        "speed_scale_by_band": list(speed_scale_by_band),
        "braking_accel_by_band": list(braking_accel_by_band),
        "speed_controller": {
            "kp_accel": speed_controller.config.kp_accel,
            "kp_brake": speed_controller.config.kp_brake,
            "ki": speed_controller.config.ki,
            "prediction_time_s": (
                speed_controller.config.prediction_time_s
            ),
        },
    }
    if isinstance(controller, StanleyController):
        snapshot["controller_type"] = "stanley"
        snapshot["controller"] = {
            "cross_track_gain": controller.config.cross_track_gain,
            "heading_gain": controller.config.heading_gain,
            "yaw_rate_gain": controller.config.yaw_rate_gain,
            "steering_filter_tau_s": (
                controller.config.steering_filter_tau_s
            ),
            "steering_rate_gain": controller.config.steering_rate_gain,
            "steering_jerk_limit_rad_s3": (
                controller.config.steering_jerk_limit_rad_s3
            ),
            "cross_track_gain_map": list(
                controller.config.cross_track_gain_map
            ),
            "heading_gain_map": list(
                controller.config.heading_gain_map
            ),
            "yaw_rate_gain_map": list(
                controller.config.yaw_rate_gain_map
            ),
            "filter_tau_map": list(controller.config.filter_tau_map),
        }
    elif isinstance(controller, LinearMpcController):
        snapshot["controller_type"] = "lmpc"
        snapshot["controller"] = {
            "q_lateral": controller.config.q_lateral,
            "q_heading": controller.config.q_heading,
            "q_speed": controller.config.q_speed,
            "q_lateral_velocity": (
                controller.config.q_lateral_velocity
            ),
            "q_yaw_rate": controller.config.q_yaw_rate,
            "r_steer": controller.config.r_steer,
            "r_accel": controller.config.r_accel,
            "rd_steer": controller.config.rd_steer,
            "rd_accel": controller.config.rd_accel,
            "terminal_weight_scale": (
                controller.config.terminal_weight_scale
            ),
            "steering_lead_s": controller.config.steering_lead_s,
            "steering_filter_tau_s": (
                controller.config.steering_filter_tau_s
            ),
            "steering_actuator_tau_s": (
                controller.config.steering_actuator_tau_s
            ),
            "steering_state_weight": (
                controller.config.steering_state_weight
            ),
            "use_steering_actuator_model": (
                controller.config.use_steering_actuator_model
            ),
            "steering_deadband": controller.config.steering_deadband,
            "steering_hysteresis": (
                controller.config.steering_hysteresis
            ),
            "solve_time_budget_ms": (
                controller.config.solve_time_budget_ms
            ),
            "reference_preview_s": (
                controller.config.reference_preview_s
            ),
            "weight_map_kmh": list(
                controller.config.weight_map_kmh
            ),
            "q_lateral_weight_map": list(
                controller.config.q_lateral_weight_map
            ),
            "q_heading_weight_map": list(
                controller.config.q_heading_weight_map
            ),
            "q_yaw_rate_weight_map": list(
                controller.config.q_yaw_rate_weight_map
            ),
            "r_steer_weight_map": list(
                controller.config.r_steer_weight_map
            ),
            "rd_steer_weight_map": list(
                controller.config.rd_steer_weight_map
            ),
            "steer_lateral_gain_map": list(
                controller.config.parameters.steer_lateral_gain_map
            ),
            "steer_yaw_gain_map": list(
                controller.config.parameters.steer_yaw_gain_map
            ),
            "yaw_inertia_map_kgm2": list(
                controller.config.parameters.yaw_inertia_map_kgm2
            ),
            "lateral_velocity_damping_map": list(
                controller.config.parameters.lateral_velocity_damping_map
            ),
            "lateral_yaw_coupling_map": list(
                controller.config.parameters.lateral_yaw_coupling_map
            ),
            "yaw_velocity_coupling_map": list(
                controller.config.parameters.yaw_velocity_coupling_map
            ),
            "yaw_rate_damping_map": list(
                controller.config.parameters.yaw_rate_damping_map
            ),
            "yaw_response_load_schedule_g": list(
                controller.config.parameters.yaw_response_load_schedule_g
            ),
            "yaw_response_load_scale": list(
                controller.config.parameters.yaw_response_load_scale
            ),
        }
    else:
        snapshot["controller_type"] = type(controller).__name__
        snapshot["controller"] = {}
    if longitudinal_mpc is not None:
        longitudinal = longitudinal_mpc.config
        snapshot["longitudinal_controller"] = {
            "response_tau_s": longitudinal.response_tau_s,
            "q_speed": longitudinal.q_speed,
            "q_accel": longitudinal.q_accel,
            "r_accel": longitudinal.r_accel,
            "r_jerk": longitudinal.r_jerk,
            "max_accel_mps2": longitudinal.max_accel_mps2,
            "max_brake_mps2": longitudinal.max_brake_mps2,
            "max_jerk_mps3": longitudinal.max_jerk_mps3,
            "max_brake_jerk_mps3": (
                longitudinal.max_brake_jerk_mps3
            ),
            "reference_filter_up_tau_s": (
                longitudinal.reference_filter_up_tau_s
            ),
            "reference_filter_down_tau_s": (
                longitudinal.reference_filter_down_tau_s
            ),
            "coast_down_accel_map_kmh": list(
                longitudinal.coast_down_accel_map_kmh
            ),
            "coast_down_accel_map_mps2": list(
                longitudinal.coast_down_accel_map_mps2
            ),
            "lateral_weight_schedule_g": list(
                longitudinal.lateral_weight_schedule_g
            ),
            "r_accel_lateral_scale": list(
                longitudinal.r_accel_lateral_scale
            ),
            "r_jerk_lateral_scale": list(
                longitudinal.r_jerk_lateral_scale
            ),
            "speed_weight_schedule_kmh": list(
                longitudinal.speed_weight_schedule_kmh
            ),
            "r_accel_speed_scale": list(
                longitudinal.r_accel_speed_scale
            ),
            "r_jerk_speed_scale": list(
                longitudinal.r_jerk_speed_scale
            ),
        }
    if longitudinal_pedal_mapper is not None:
        pedal = longitudinal_pedal_mapper.config
        snapshot["longitudinal_pedal_mapper"] = {
            "max_throttle": pedal.max_throttle,
            "throttle_limit_map_kmh": list(
                pedal.throttle_limit_map_kmh
            ),
            "throttle_limit_map": list(
                pedal.throttle_limit_map
            ),
            "throttle_rate_up_per_s": (
                pedal.throttle_rate_up_per_s
            ),
            "brake_rate_up_per_s": pedal.brake_rate_up_per_s,
            "steering_throttle_reduction": (
                pedal.steering_throttle_reduction
            ),
            "slip_throttle_cut": pedal.slip_throttle_cut,
        }
    return snapshot


def _summarize_lmpc_parameters(controller) -> str:
    if not isinstance(controller, LinearMpcController):
        return "n/a"
    config = controller.config
    return (
        f"q_lat={config.q_lateral:.1f},"
        f"q_head={config.q_heading:.1f},"
        f"q_yaw={config.q_yaw_rate:.2f},"
        f"r_steer={config.r_steer:.2f},"
        f"rd_steer={config.rd_steer:.1f}"
    )


def _next_lmpc_tuning(
    *,
    lap_time_s: float,
    target_lap_s: float,
    p95_lateral_error_m: float,
    p95_heading_error_rad: float,
    steering_activity: float,
    band_raw_steering_activity: tuple[float, ...],
    band_p95_lateral_error_m: tuple[float, ...],
    band_p95_heading_error_rad: tuple[float, ...],
    band_max_tyres_out: tuple[int, ...],
    band_sample_count: tuple[int, ...],
    band_p95_lateral_g: tuple[float, ...],
    band_p95_longitudinal_g: tuple[float, ...],
    speed_scale_by_band: list[float],
    braking_accel_by_band: list[float],
    base_lateral_accel_mps2: float,
    max_speed_scale: float,
    max_braking_accel_mps2: float,
    mpc_config: MpcConfig,
) -> tuple[list[float], list[float]]:
    if p95_lateral_error_m > 1.20 or p95_heading_error_rad > 0.16:
        mpc_config.q_lateral = min(120.0, mpc_config.q_lateral * 1.12)
        mpc_config.q_heading = min(120.0, mpc_config.q_heading * 1.08)
        mpc_config.q_yaw_rate = min(
            30.0,
            mpc_config.q_yaw_rate * 1.08,
        )
    elif p95_lateral_error_m > 0.65 or p95_heading_error_rad > 0.08:
        mpc_config.q_lateral = min(120.0, mpc_config.q_lateral * 1.06)
        mpc_config.q_heading = min(120.0, mpc_config.q_heading * 1.04)
        mpc_config.q_yaw_rate = min(
            30.0,
            mpc_config.q_yaw_rate * 1.04,
        )

    if steering_activity > 0.0012:
        mpc_config.rd_steer = min(520.0, mpc_config.rd_steer * 1.16)
        mpc_config.r_steer = min(30.0, mpc_config.r_steer * 1.07)
    elif (
        p95_lateral_error_m < 0.45
        and steering_activity < 0.0010
    ):
        mpc_config.rd_steer = max(320.0, mpc_config.rd_steer)

    if (
        p95_lateral_error_m < 0.50
        and p95_heading_error_rad < 0.06
    ):
        mpc_config.terminal_weight_scale = min(
            8.0,
            mpc_config.terminal_weight_scale * 1.03,
        )

    parameters = mpc_config.parameters
    lateral_gain_map = list(parameters.steer_lateral_gain_map)
    yaw_gain_map = list(parameters.steer_yaw_gain_map)
    yaw_inertia_map = list(parameters.yaw_inertia_map_kgm2)
    q_lateral_weight_map = list(
        mpc_config.q_lateral_weight_map
    )
    q_heading_weight_map = list(
        mpc_config.q_heading_weight_map
    )
    q_yaw_weight_map = list(
        mpc_config.q_yaw_rate_weight_map
    )
    r_steer_weight_map = list(mpc_config.r_steer_weight_map)
    rd_steer_weight_map = list(mpc_config.rd_steer_weight_map)
    oscillation_limits = (
        0.0040,
        0.0018,
        0.0014,
        0.0012,
        0.0010,
        0.0010,
        0.0010,
        0.0010,
        0.0010,
    )
    for band in range(
        min(
            len(band_raw_steering_activity),
            len(band_p95_lateral_error_m),
            len(band_sample_count),
            len(lateral_gain_map),
            len(yaw_gain_map),
            len(yaw_inertia_map),
            len(q_lateral_weight_map),
            len(q_heading_weight_map),
            len(q_yaw_weight_map),
            len(r_steer_weight_map),
            len(rd_steer_weight_map),
        )
    ):
        if band_sample_count[band] < 30:
            continue
        raw_activity = band_raw_steering_activity[band]
        lateral_error = band_p95_lateral_error_m[band]
        oscillation_limit = oscillation_limits[
            min(band, len(oscillation_limits) - 1)
        ]
        if raw_activity > oscillation_limit:
            q_lateral_weight_map[band] = min(
                1.5,
                q_lateral_weight_map[band] * 1.05,
            )
            q_heading_weight_map[band] = min(
                1.6,
                q_heading_weight_map[band] * 1.04,
            )
            q_yaw_weight_map[band] = min(
                2.0,
                q_yaw_weight_map[band] * 1.08,
            )
            r_steer_weight_map[band] = min(
                3.20,
                r_steer_weight_map[band] * 1.08,
            )
            rd_steer_weight_map[band] = min(
                3.80,
                rd_steer_weight_map[band] * 1.15,
            )
            if lateral_error < 1.0:
                lateral_gain_map[band] = max(
                    0.75 * parameters.steer_lateral_gain_map[band],
                    lateral_gain_map[band] * 0.97,
                )
                yaw_gain_map[band] = max(
                    0.75 * parameters.steer_yaw_gain_map[band],
                    yaw_gain_map[band] * 0.97,
                )
                yaw_inertia_map[band] = min(
                    1.35 * parameters.yaw_inertia_map_kgm2[band],
                    yaw_inertia_map[band] * 1.02,
                )

    mpc_config.q_lateral_weight_map = tuple(
        q_lateral_weight_map
    )
    mpc_config.q_heading_weight_map = tuple(
        q_heading_weight_map
    )
    mpc_config.q_yaw_rate_weight_map = tuple(q_yaw_weight_map)
    mpc_config.r_steer_weight_map = tuple(r_steer_weight_map)
    mpc_config.rd_steer_weight_map = tuple(rd_steer_weight_map)
    mpc_config.parameters = replace(
        parameters,
        steer_lateral_gain_map=tuple(lateral_gain_map),
        steer_yaw_gain_map=tuple(yaw_gain_map),
        yaw_inertia_map_kgm2=tuple(yaw_inertia_map),
    )

    band_count = min(
            len(speed_scale_by_band),
            len(braking_accel_by_band),
            len(band_p95_lateral_error_m),
            len(band_p95_heading_error_rad),
            len(band_max_tyres_out),
            len(band_sample_count),
            len(band_p95_lateral_g),
            len(band_p95_longitudinal_g),
        )
    scale_increments = (
        0.06,
        0.04,
        0.025,
        0.015,
        0.012,
        0.010,
        0.008,
        0.006,
        0.005,
    )[:band_count]
    braking_increments = (
        0.25,
        0.20,
        0.15,
        0.10,
        0.08,
        0.06,
        0.05,
        0.04,
        0.035,
    )[:band_count]
    for band in range(band_count):
        if band_sample_count[band] < 30:
            continue
        lateral_error = band_p95_lateral_error_m[band]
        heading_error = band_p95_heading_error_rad[band]
        tyres_out = band_max_tyres_out[band]
        lateral_g = band_p95_lateral_g[band]
        longitudinal_g = band_p95_longitudinal_g[band]
        critical = (
            tyres_out >= 3
            or lateral_error > 3.0
            or heading_error > 0.30
            or lateral_g > 1.55
        )
        unsafe = (
            lateral_error > 1.5
            or heading_error > 0.15
            or lateral_g > 1.35
        )
        caution = (
            lateral_error > 0.80
            or heading_error > 0.10
            or lateral_g > 1.20
        )
        if critical:
            speed_scale_by_band[band] = max(
                0.55,
                speed_scale_by_band[band] - 0.12,
            )
            continue
        if unsafe:
            speed_scale_by_band[band] = max(
                0.60,
                speed_scale_by_band[band] - 0.06,
            )
            continue
        if caution:
            speed_scale_by_band[band] = max(
                0.70,
                speed_scale_by_band[band] - 0.025,
            )
            continue

        tracking_is_safe = (
            lateral_error < 1.0
            and heading_error < 0.14
            and tyres_out <= 1
            and lateral_g < 1.30
        )
        candidate_scale = min(
            max_speed_scale,
            speed_scale_by_band[band] + scale_increments[band],
        )
        candidate_lateral_g = (
            base_lateral_accel_mps2
            * candidate_scale
            * candidate_scale
            / 9.81
        )
        if (
            lap_time_s > target_lap_s
            and tracking_is_safe
            and candidate_lateral_g <= 1.45
        ):
            speed_scale_by_band[band] = candidate_scale

        if (
            lateral_error < 3.0
            and tyres_out <= 1
            and longitudinal_g < 0.78
        ):
            braking_accel_by_band[band] = min(
                max_braking_accel_mps2,
                braking_accel_by_band[band]
                + braking_increments[band],
            )
    return speed_scale_by_band, braking_accel_by_band


def _tune_speed_controller(
    config: SpeedControllerConfig,
    band_p95_speed_error_kmh: tuple[float, ...],
    band_sample_count: tuple[int, ...],
    band_p95_longitudinal_g: tuple[float, ...],
) -> None:
    active_errors = [
        error
        for error, count in zip(
            band_p95_speed_error_kmh,
            band_sample_count,
        )
        if count >= 30
    ]
    if not active_errors:
        return
    worst_error = max(active_errors)
    if worst_error > 4.0:
        config.kp_accel = min(0.16, config.kp_accel * 1.08)
        config.kp_brake = min(0.14, config.kp_brake * 1.08)
        config.ki = min(0.02, config.ki * 1.05)
        config.prediction_time_s = min(
            0.08,
            config.prediction_time_s + 0.01,
        )
    elif worst_error > 2.0:
        config.kp_accel = min(0.16, config.kp_accel * 1.04)
        config.kp_brake = min(0.14, config.kp_brake * 1.04)

    max_longitudinal_g = max(
        (
            value
            for value, count in zip(
                band_p95_longitudinal_g,
                band_sample_count,
            )
            if count >= 30
        ),
        default=0.0,
    )
    if max_longitudinal_g > 0.85:
        config.kp_brake = max(0.03, config.kp_brake * 0.94)
        config.prediction_time_s = max(
            0.0,
            config.prediction_time_s - 0.01,
        )


def _tune_longitudinal_mpc(
    controller: LongitudinalMpcController,
    band_p95_speed_error_kmh: tuple[float, ...],
    band_sample_count: tuple[int, ...],
    band_p95_longitudinal_g: tuple[float, ...],
    band_max_tyres_out: tuple[int, ...],
    *,
    max_brake_limit_mps2: float,
) -> None:
    config = controller.config
    active_errors = [
        error
        for error, count in zip(
            band_p95_speed_error_kmh,
            band_sample_count,
        )
        if count >= 30
    ]
    if not active_errors:
        return
    active_longitudinal_g = [
        value
        for value, count in zip(
            band_p95_longitudinal_g,
            band_sample_count,
        )
        if count >= 30
    ]
    worst_error = max(active_errors)
    max_longitudinal_g = max(active_longitudinal_g, default=0.0)
    unsafe = (
        max(band_max_tyres_out, default=0) >= 3
        or max_longitudinal_g > 1.45
    )

    if unsafe:
        config.max_accel_mps2 = max(
            1.40,
            config.max_accel_mps2 * 0.96,
        )
        config.max_brake_mps2 = max(
            3.80,
            config.max_brake_mps2 * 0.98,
        )
        config.max_jerk_mps3 = max(
            3.0,
            config.max_jerk_mps3 * 0.95,
        )
        config.max_brake_jerk_mps3 = max(
            12.0,
            config.max_brake_jerk_mps3 * 0.95,
        )
    elif worst_error > 4.0:
        config.q_speed = min(45.0, config.q_speed * 1.10)
        config.q_accel = min(3.00, config.q_accel * 1.05)
        config.r_jerk = max(12.0, config.r_jerk * 0.96)
        config.max_jerk_mps3 = min(
            9.0,
            config.max_jerk_mps3 * 1.04,
        )
    elif worst_error > 2.0:
        config.q_speed = min(45.0, config.q_speed * 1.05)
        config.q_accel = min(3.00, config.q_accel * 1.03)
    elif worst_error < 1.5 and max_longitudinal_g < 0.78:
        config.max_accel_mps2 = min(
            3.60,
            config.max_accel_mps2 + 0.05,
        )
        config.max_brake_mps2 = min(
            max_brake_limit_mps2,
            config.max_brake_mps2 + 0.08,
        )
        config.max_jerk_mps3 = min(
            9.0,
            config.max_jerk_mps3 + 0.15,
        )
        config.max_brake_jerk_mps3 = min(
            26.0,
            config.max_brake_jerk_mps3 + 0.40,
        )

    if worst_error > 4.0:
        config.response_tau_s = max(
            0.06,
            config.response_tau_s * 0.96,
        )
    elif worst_error < 1.5 and max_longitudinal_g < 0.80:
        config.response_tau_s = min(
            0.16,
            config.response_tau_s + 0.002,
        )
    controller.rebuild_solver()


def _next_lap_tuning(
    *,
    lap_time_s: float,
    target_lap_s: float,
    max_lateral_error_m: float,
    max_heading_error_rad: float,
    p95_lateral_error_m: float,
    p95_heading_error_rad: float,
    band_p95_lateral_error_m: tuple[float, ...],
    band_p95_heading_error_rad: tuple[float, ...],
    band_max_tyres_out: tuple[int, ...],
    band_steering_activity: tuple[float, ...],
    band_sample_count: tuple[int, ...],
    max_tyres_out: int,
    steering_activity: float,
    speed_scale_by_band: list[float],
    cross_track_gain: float,
    heading_gain: float,
    yaw_rate_gain: float,
    steering_filter_tau_s: float,
    braking_accel_by_band: list[float],
    speed_scale_step: float,
    max_speed_scale: float,
    braking_accel_step_mps2: float,
    max_braking_accel_mps2: float,
) -> tuple[list[float], float, float, float, float, list[float]]:
    del max_lateral_error_m, max_heading_error_rad, max_tyres_out
    band_count = min(
        len(speed_scale_by_band),
        len(braking_accel_by_band),
        len(band_p95_lateral_error_m),
        len(band_p95_heading_error_rad),
        len(band_max_tyres_out),
        len(band_steering_activity),
        len(band_sample_count),
    )
    scale_increments = (
        speed_scale_step * 2.4,
        speed_scale_step * 1.8,
        speed_scale_step * 1.2,
        speed_scale_step * 0.8,
        speed_scale_step * 0.6,
        speed_scale_step * 0.5,
        speed_scale_step * 0.4,
        speed_scale_step * 0.3,
        speed_scale_step * 0.25,
    )[:band_count]
    braking_increments = (
        braking_accel_step_mps2 * 1.7,
        braking_accel_step_mps2 * 1.3,
        braking_accel_step_mps2 * 1.0,
        braking_accel_step_mps2 * 0.7,
        braking_accel_step_mps2 * 0.55,
        braking_accel_step_mps2 * 0.45,
        braking_accel_step_mps2 * 0.35,
        braking_accel_step_mps2 * 0.30,
        braking_accel_step_mps2 * 0.25,
    )[:band_count]
    for band in range(band_count):
        if band_sample_count[band] < 30:
            continue
        lateral_error = band_p95_lateral_error_m[band]
        heading_error = band_p95_heading_error_rad[band]
        tyres_out = band_max_tyres_out[band]
        unsafe = (
            tyres_out >= 3
            or lateral_error > 4.0
            or heading_error > 0.45
        )
        if unsafe:
            speed_scale_by_band[band] = max(
                0.70,
                speed_scale_by_band[band]
                - 2.0 * scale_increments[band],
            )
            braking_accel_by_band[band] = max(
                3.0,
                braking_accel_by_band[band]
                - 2.0 * braking_increments[band],
            )
            continue
        if (
            lap_time_s > target_lap_s
            and lateral_error < 2.0
            and heading_error < 0.20
            and tyres_out <= 1
        ):
            speed_scale_by_band[band] = min(
                max_speed_scale,
                speed_scale_by_band[band] + scale_increments[band],
            )
        if lateral_error < 3.5 and tyres_out <= 1:
            braking_accel_by_band[band] = min(
                max_braking_accel_mps2,
                braking_accel_by_band[band] + braking_increments[band],
            )

    if p95_lateral_error_m > 1.5 or p95_heading_error_rad > 0.18:
        cross_track_gain = min(2.0, cross_track_gain * 1.05)
        heading_gain = min(2.0, heading_gain * 1.02)
        yaw_rate_gain = min(0.35, yaw_rate_gain + 0.005)

    if steering_activity > 0.0035:
        steering_filter_tau_s = min(
            0.25,
            steering_filter_tau_s * 1.10,
        )
        yaw_rate_gain = min(0.35, yaw_rate_gain * 1.05)
    elif (
        p95_lateral_error_m < 1.2
        and steering_activity < 0.0015
    ):
        steering_filter_tau_s = max(
            0.05,
            steering_filter_tau_s * 0.95,
        )
        yaw_rate_gain = max(0.05, yaw_rate_gain * 0.95)

    return (
        speed_scale_by_band,
        cross_track_gain,
        heading_gain,
        yaw_rate_gain,
        steering_filter_tau_s,
        braking_accel_by_band,
    )


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    fraction = max(0.0, min(1.0, fraction))
    index = int(round((len(ordered) - 1) * fraction))
    return ordered[index]


def _write_lap_summary(
    handle,
    *,
    lap_number: int,
    lap_time_s: float,
    max_lateral_error_m: float,
    max_heading_error_rad: float,
    p95_lateral_error_m: float,
    p95_heading_error_rad: float,
    max_tyres_out: int,
    steering_activity: float,
    effective_lateral_g: float,
    effective_max_speed_kmh: float,
    braking_accel_mps2: float,
    band_p95_lateral_g: list[float],
    band_p95_longitudinal_g: list[float],
    band_p95_speed_error_kmh: list[float],
    before_parameters: dict[str, float],
    after_parameters: dict[str, float],
    planning_update: PlanningUpdate | None = None,
) -> None:
    if handle is None:
        return
    record = {
        "lap_number": lap_number,
        "lap_time_s": lap_time_s,
        "max_lateral_error_m": max_lateral_error_m,
        "max_heading_error_rad": max_heading_error_rad,
        "p95_lateral_error_m": p95_lateral_error_m,
        "p95_heading_error_rad": p95_heading_error_rad,
        "max_tyres_out": max_tyres_out,
        "steering_activity": steering_activity,
        "effective_lateral_g": effective_lateral_g,
        "effective_max_speed_kmh": effective_max_speed_kmh,
        "braking_accel_mps2": braking_accel_mps2,
        "band_p95_lateral_g": band_p95_lateral_g,
        "band_p95_longitudinal_g": band_p95_longitudinal_g,
        "band_p95_speed_error_kmh": band_p95_speed_error_kmh,
        "before": before_parameters,
        "after": after_parameters,
    }
    if planning_update is not None:
        record["planning"] = {
            "lap_number": planning_update.planning_lap_number,
            "estimated_lap_before_s": (
                planning_update.estimated_lap_before_s
            ),
            "estimated_lap_after_s": (
                planning_update.estimated_lap_after_s
            ),
            "accepted": planning_update.accepted,
            "stable_corners": planning_update.stable_corners,
            "caution_corners": planning_update.caution_corners,
            "unsafe_corners": planning_update.unsafe_corners,
            "max_p95_lateral_error_m": (
                planning_update.max_p95_lateral_error_m
            ),
            "max_p95_heading_error_rad": (
                planning_update.max_p95_heading_error_rad
            ),
            "max_tyres_out": planning_update.max_tyres_out,
            "min_edge_clearance_m": (
                planning_update.min_edge_clearance_m
            ),
            "blend_by_corner": list(
                planning_update.blend_by_corner
            ),
        }
    handle.write(json.dumps(record, ensure_ascii=True) + "\n")
    handle.flush()


def _sleep_until(deadline: float) -> None:
    remaining = deadline - time.monotonic()
    if remaining > 0.0:
        time.sleep(remaining)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except SharedMemoryUnavailable as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
