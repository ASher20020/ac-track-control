from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import psutil

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


def read_completed_lap(
    summary_path: Path,
    required_lap: int,
) -> dict | None:
    if not summary_path.exists():
        return None
    records: list[dict] = []
    for line in summary_path.read_text(
        encoding="utf-8",
        errors="ignore",
    ).splitlines():
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    candidates = [
        record
        for record in records
        if int(record.get("lap_number", 0)) >= required_lap
    ]
    if not candidates:
        return None
    return candidates[-1]


def controller_running(log_name: str) -> bool:
    needle = log_name.lower()
    for process in psutil.process_iter(["cmdline"]):
        try:
            command_line = " ".join(process.info["cmdline"] or [])
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
        if "actc.cli" in command_line and needle in command_line.lower():
            return True
    return False


def wait_for_source_completion(
    summary_path: Path,
    log_name: str,
    required_lap: int,
    timeout_s: float,
) -> dict:
    deadline = time.monotonic() + max(1.0, timeout_s)
    completed: dict | None = None
    while time.monotonic() < deadline:
        completed = read_completed_lap(summary_path, required_lap)
        if completed is not None and not controller_running(log_name):
            time.sleep(1.0)
            if not controller_running(log_name):
                return completed
        time.sleep(0.5)
    raise TimeoutError(
        f"Timed out waiting for lap {required_lap} in {summary_path}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-log", type=Path, required=True)
    parser.add_argument("--await-lap", type=int, required=True)
    parser.add_argument("--continuation-laps", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=3600.0)
    parser.add_argument("--config-output", type=Path, required=True)
    parser.add_argument("--log-output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=1800.0)
    parser.add_argument("--target-rise-kmh-s", type=float, default=18.0)
    parser.add_argument("--braking-safety-factor", type=float, default=0.88)
    parser.add_argument("--braking-response-time-s", type=float, default=0.10)
    parser.add_argument("--merge-speed-kmh", type=float, default=60.0)
    parser.add_argument(
        "--startup-merge-distance",
        type=float,
        default=250.0,
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
    parser.add_argument(
        "--longitudinal-controller",
        choices=("pi", "mpc"),
        default="pi",
    )
    parser.add_argument("--r-scale", type=float, default=1.12)
    parser.add_argument("--rd-scale", type=float, default=1.15)
    parser.add_argument("--mid-r-boost", type=float, default=1.15)
    parser.add_argument("--mid-rd-boost", type=float, default=1.15)
    args = parser.parse_args()

    source_log = args.source_log.resolve()
    summary_path = source_log.with_name(
        f"{source_log.stem}_laps.jsonl"
    )
    completed = wait_for_source_completion(
        summary_path,
        source_log.name,
        args.await_lap,
        args.timeout,
    )
    continuation_parameters = completed.get("after")
    if not isinstance(continuation_parameters, dict):
        raise RuntimeError(
            f"Missing 'after' parameters in {summary_path}"
        )
    controller_parameters = continuation_parameters.get("controller")
    if not isinstance(controller_parameters, dict):
        raise RuntimeError(
            f"Missing controller parameters in {summary_path}"
        )
    controller_parameters["r_steer"] = min(
        30.0,
        float(controller_parameters.get("r_steer", 8.0))
        * max(1.0, args.r_scale),
    )
    controller_parameters["rd_steer"] = min(
        700.0,
        float(controller_parameters.get("rd_steer", 320.0))
        * max(1.0, args.rd_scale),
    )
    r_weight_map = list(
        controller_parameters.get(
            "r_steer_weight_map",
            (1.05, 1.15, 1.35, 2.25, 2.60),
        )
    )
    rd_weight_map = list(
        controller_parameters.get(
            "rd_steer_weight_map",
            (1.05, 1.20, 1.50, 2.50, 3.00),
        )
    )
    for index in range(min(3, len(r_weight_map))):
        r_weight_map[index] *= max(1.0, args.mid_r_boost)
        rd_weight_map[index] *= max(1.0, args.mid_rd_boost)
    controller_parameters["r_steer_weight_map"] = r_weight_map
    controller_parameters["rd_steer_weight_map"] = rd_weight_map
    args.config_output.parent.mkdir(parents=True, exist_ok=True)
    args.config_output.write_text(
        json.dumps(continuation_parameters, indent=2),
        encoding="utf-8",
    )
    auto_start = PROJECT_ROOT / "tools" / "auto_start.py"
    command = [
        sys.executable,
        str(auto_start),
        "--timeout",
        str(args.timeout),
        "--laps",
        str(args.continuation_laps),
        "--control-laps",
        str(args.continuation_laps),
        "--planning-laps",
        "0",
        "--duration",
        str(args.duration),
        "--target-rise-kmh-s",
        str(args.target_rise_kmh_s),
        "--braking-safety-factor",
        str(args.braking_safety_factor),
        "--braking-response-time-s",
        str(args.braking_response_time_s),
        "--merge-speed-kmh",
        str(args.merge_speed_kmh),
        "--startup-merge-distance",
        str(args.startup_merge_distance),
        "--longitudinal-controller",
        args.longitudinal_controller,
        "--log",
        str(args.log_output),
        "--resume-tuning",
        str(args.config_output),
        "--mpc-model",
        str(args.mpc_model),
    ]
    stdout_path = args.log_output.with_name(
        f"{args.log_output.stem}_auto_start.out.log"
    )
    stderr_path = args.log_output.with_name(
        f"{args.log_output.stem}_auto_start.err.log"
    )
    with stdout_path.open("w", encoding="utf-8") as stdout_handle:
        with stderr_path.open("w", encoding="utf-8") as stderr_handle:
            return subprocess.call(
                command,
                cwd=PROJECT_ROOT,
                stdout=stdout_handle,
                stderr=stderr_handle,
            )


if __name__ == "__main__":
    raise SystemExit(main())
