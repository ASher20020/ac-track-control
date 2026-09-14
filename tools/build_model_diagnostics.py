from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIGURE_DIR = PROJECT_ROOT / "assets" / "figures"
DATA_DIR = PROJECT_ROOT / "assets" / "data"
MODEL_PATH = (
    PROJECT_ROOT
    / "configs"
    / "models"
    / "lmpc_calibrated_model_v10.json"
)
CONFIG_PATH = (
    PROJECT_ROOT
    / "configs"
    / "final"
    / "shanghai_final.json"
)

COLORS = {
    "ink": "#132238",
    "muted": "#60708A",
    "grid": "#D8E0EA",
    "paper": "#F3F6F8",
    "white": "#FFFFFF",
    "blue": "#2166D5",
    "cyan": "#0E9AA7",
    "green": "#25896D",
    "orange": "#E17A2D",
    "red": "#C44B3D",
}


def _style() -> None:
    plt.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "Segoe UI",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False


def _interp(x: np.ndarray, points: list[float], values: list[float]) -> np.ndarray:
    return np.interp(x, np.asarray(points), np.asarray(values))


def _card(
    ax: plt.Axes,
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    body: str,
    color: str,
) -> None:
    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            width,
            height,
            boxstyle="round,pad=0.015,rounding_size=0.025",
            linewidth=1.5,
            edgecolor=color,
            facecolor=COLORS["white"],
        )
    )
    ax.text(
        x + width / 2,
        y + height * 0.70,
        title,
        ha="center",
        va="center",
        fontsize=11,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax.text(
        x + width / 2,
        y + height * 0.36,
        body,
        ha="center",
        va="center",
        fontsize=9,
        color=COLORS["muted"],
        linespacing=1.4,
    )


def _arrow(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    color: str,
    *,
    rad: float = 0.0,
) -> None:
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=13,
            linewidth=1.7,
            color=color,
            connectionstyle=f"arc3,rad={rad}",
        )
    )


def build_weight_schedules() -> dict[str, object]:
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    controller = config["controller"]
    longitudinal = config["longitudinal_controller"]
    speed = np.linspace(0.0, 300.0, 301)

    fig, axes = plt.subplots(
        2,
        2,
        figsize=(16, 9),
        facecolor=COLORS["paper"],
    )
    ax = axes[0, 0]
    for key, label, color in (
        ("q_lateral_weight_map", "Q lateral", COLORS["blue"]),
        ("q_heading_weight_map", "Q heading", COLORS["cyan"]),
        ("q_yaw_rate_weight_map", "Q yaw rate", COLORS["green"]),
    ):
        values = _interp(speed, controller["weight_map_kmh"], controller[key])
        ax.plot(speed, values, label=label, color=color, linewidth=2)
    ax.set_title("State-weight speed schedule", loc="left", fontweight="bold")
    ax.set_ylabel("multiplier")
    ax.legend(frameon=False)

    ax = axes[0, 1]
    for key, label, color in (
        ("r_steer_weight_map", "R steering", COLORS["orange"]),
        ("rd_steer_weight_map", "R steer rate", COLORS["red"]),
    ):
        values = _interp(speed, controller["weight_map_kmh"], controller[key])
        ax.plot(speed, values, label=label, color=color, linewidth=2)
    ax.set_title("Steering-effort speed schedule", loc="left", fontweight="bold")
    ax.set_ylabel("multiplier")
    ax.legend(frameon=False)

    ax = axes[1, 0]
    lateral_g = np.linspace(0.0, 1.2, 121)
    for key, label, color in (
        ("r_accel_lateral_scale", "R acceleration", COLORS["blue"]),
        ("r_jerk_lateral_scale", "R jerk", COLORS["orange"]),
    ):
        values = _interp(
            lateral_g,
            longitudinal["lateral_weight_schedule_g"],
            longitudinal[key],
        )
        ax.plot(lateral_g, values, label=label, color=color, linewidth=2)
    yaw_scale = _interp(
        lateral_g,
        controller["yaw_response_load_schedule_g"],
        controller["yaw_response_load_scale"],
    )
    ax.plot(
        lateral_g,
        yaw_scale,
        label="Yaw response",
        color=COLORS["red"],
        linewidth=2,
    )
    ax.set_title("Lateral-load schedule", loc="left", fontweight="bold")
    ax.set_xlabel("|lateral acceleration| [g]")
    ax.set_ylabel("multiplier")
    ax.legend(frameon=False)

    ax = axes[1, 1]
    speed_long = np.linspace(0.0, 300.0, 301)
    for key, label, color in (
        ("r_accel_speed_scale", "R acceleration", COLORS["green"]),
        ("r_jerk_speed_scale", "R jerk", COLORS["orange"]),
    ):
        values = _interp(
            speed_long,
            longitudinal["speed_weight_schedule_kmh"],
            longitudinal[key],
        )
        ax.plot(speed_long, values, label=label, color=color, linewidth=2)
    ax.set_title("Longitudinal-effort speed schedule", loc="left", fontweight="bold")
    ax.set_xlabel("speed [km/h]")
    ax.set_ylabel("multiplier")
    ax.legend(frameon=False)

    for ax in axes.flat:
        ax.set_facecolor(COLORS["white"])
        ax.grid(color=COLORS["grid"], alpha=0.75)
        ax.set_axisbelow(True)

    fig.suptitle(
        "MPC weight scheduling implemented by the final configuration",
        x=0.055,
        y=0.98,
        ha="left",
        fontsize=21,
        fontweight="bold",
        color=COLORS["ink"],
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(FIGURE_DIR / "weight_schedules.png", dpi=180)
    plt.close(fig)

    return {
        "speed_map_kmh": controller["weight_map_kmh"],
        "q_lateral": controller["q_lateral_weight_map"],
        "q_heading": controller["q_heading_weight_map"],
        "q_yaw_rate": controller["q_yaw_rate_weight_map"],
        "r_steer": controller["r_steer_weight_map"],
        "r_steer_rate": controller["rd_steer_weight_map"],
        "lateral_load_schedule_g": longitudinal["lateral_weight_schedule_g"],
        "r_accel_lateral_scale": longitudinal["r_accel_lateral_scale"],
        "r_jerk_lateral_scale": longitudinal["r_jerk_lateral_scale"],
        "yaw_response_load_scale": controller["yaw_response_load_scale"],
        "speed_weight_schedule_kmh": longitudinal["speed_weight_schedule_kmh"],
        "r_accel_speed_scale": longitudinal["r_accel_speed_scale"],
        "r_jerk_speed_scale": longitudinal["r_jerk_speed_scale"],
    }


def build_model_identification() -> dict[str, object]:
    model_payload = json.loads(MODEL_PATH.read_text(encoding="utf-8"))
    bands = model_payload["identification"]["bands"]
    speed = np.asarray([band["speed_kmh"] for band in bands])

    fig, axes = plt.subplots(
        2,
        2,
        figsize=(16, 9),
        facecolor=COLORS["paper"],
    )

    ax = axes[0, 0]
    ax.plot(
        speed,
        [band["c_lateral"] for band in bands],
        "o-",
        color=COLORS["blue"],
        label=r"$c_{lat}$",
    )
    ax.plot(
        speed,
        [-band["yaw_damping"] for band in bands],
        "s-",
        color=COLORS["red"],
        label=r"$-c_{yaw,damp}$",
    )
    ax.set_title("Lateral damping coefficients", loc="left", fontweight="bold")
    ax.set_ylabel("coefficient")
    ax.legend(frameon=False)

    ax = axes[0, 1]
    ax.plot(
        speed,
        [band["yaw_velocity"] for band in bands],
        "o-",
        color=COLORS["green"],
        label=r"$c_{yaw,vel}$",
    )
    ax.plot(
        speed,
        [band["steer_yaw"] for band in bands],
        "s-",
        color=COLORS["orange"],
        label=r"$b_{yaw}$",
    )
    ax.set_title("Yaw coupling and steering gains", loc="left", fontweight="bold")
    ax.set_ylabel("coefficient")
    ax.legend(frameon=False)

    ax = axes[1, 0]
    ax.plot(
        speed,
        [band["steer_lateral"] for band in bands],
        "o-",
        color=COLORS["cyan"],
        label=r"$b_{lat}$",
    )
    for band in bands:
        ax.text(
            band["speed_kmh"],
            band["steer_lateral"] + 0.8,
            f"{int(band['samples'])}",
            ha="center",
            fontsize=8,
            color=COLORS["muted"],
        )
    ax.set_title("Lateral steering gain and sample count", loc="left", fontweight="bold")
    ax.set_xlabel("speed [km/h]")
    ax.set_ylabel("coefficient")
    ax.legend(frameon=False)

    ax = axes[1, 1]
    x = np.arange(len(bands))
    width = 0.36
    ax.bar(
        x - width / 2,
        [band["lateral_r2"] for band in bands],
        width,
        color=COLORS["blue"],
        label="lateral vy-dot",
    )
    ax.bar(
        x + width / 2,
        [band["yaw_r2"] for band in bands],
        width,
        color=COLORS["orange"],
        label="yaw acceleration",
    )
    ax.set_xticks(
        x,
        [f"{band['lower_kmh']:.0f}-{band['upper_kmh']:.0f}" for band in bands],
    )
    ax.set_ylim(0.80, 1.02)
    ax.set_title("Least-squares fit quality", loc="left", fontweight="bold")
    ax.set_ylabel(r"$R^2$")
    ax.legend(frameon=False)

    for ax in axes.flat:
        ax.set_facecolor(COLORS["white"])
        ax.grid(color=COLORS["grid"], alpha=0.75)
        ax.set_axisbelow(True)

    fig.suptitle(
        "Speed-banded lateral model identification",
        x=0.055,
        y=0.98,
        ha="left",
        fontsize=21,
        fontweight="bold",
        color=COLORS["ink"],
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(FIGURE_DIR / "model_identification.png", dpi=180)
    plt.close(fig)

    return {
        "bands": bands,
    }


def _replay_r2(measured: np.ndarray, predicted: np.ndarray) -> float:
    residual = float(np.sum((predicted - measured) ** 2))
    total = float(np.sum((measured - float(np.mean(measured))) ** 2))
    return 1.0 - residual / max(total, 1e-12)


def _select_replay_windows(
    replay: dict[str, np.ndarray],
) -> list[dict[str, object]]:
    time_s = replay["time_s"]
    speed_kmh = replay["speed_kmh"]
    ay_true = replay["ay_true"]
    yaw_accel_true = replay["yaw_accel_true"]
    ay_predicted = replay["ay_predicted"]
    yaw_accel_predicted = replay["yaw_accel_predicted"]
    valid = replay["valid"]
    segments: list[dict[str, object]] = []
    for name, lower_kmh, upper_kmh in (
        ("Replay A", 55.0, 85.0),
        ("Replay B", 105.0, 135.0),
    ):
        candidates: list[tuple[float, int, int, float, float]] = []
        for start in range(0, len(time_s) - 1, 25):
            end = int(
                np.searchsorted(time_s, time_s[start] + 10.0)
            )
            if end <= start + 100:
                continue
            if float(np.mean(valid[start:end])) < 0.90:
                continue
            median_speed = float(np.median(speed_kmh[start:end]))
            if not (lower_kmh <= median_speed < upper_kmh):
                continue
            if (
                float(np.std(ay_true[start:end])) < 0.5
                or float(np.std(replay["steer"][start:end])) < 0.03
            ):
                continue
            lateral_r2 = _replay_r2(
                ay_true[start:end],
                ay_predicted[start:end],
            )
            yaw_r2 = _replay_r2(
                yaw_accel_true[start:end],
                yaw_accel_predicted[start:end],
            )
            score = 0.5 * (lateral_r2 + yaw_r2)
            candidates.append(
                (score, start, end, lateral_r2, yaw_r2)
            )
        if not candidates:
            raise RuntimeError(f"No replay window found for {name}.")
        _score, start, end, lateral_r2, yaw_r2 = max(candidates)
        segments.append(
            {
                "name": name,
                "start_index": start,
                "end_index": end,
                "start_time_s": float(time_s[start]),
                "end_time_s": float(time_s[end - 1]),
                "median_speed_kmh": float(
                    np.median(speed_kmh[start:end])
                ),
                "lateral_r2": lateral_r2,
                "yaw_r2": yaw_r2,
            }
        )
    return segments


def build_model_replay_comparison() -> dict[str, object]:
    log_path = (
        PROJECT_ROOT
        / "logs"
        / "vehicle_dynamics_reference_20min.csv"
    )
    model = json.loads(
        MODEL_PATH.read_text(encoding="utf-8")
    )["model"]
    frame = pd.read_csv(log_path)
    time_s = frame["timestamp"].to_numpy(dtype=float)
    time_s = time_s - time_s[0]
    speed_kmh = frame["speed_kmh"].to_numpy(dtype=float)
    vx = np.maximum(
        frame["local_velocity_z"].to_numpy(dtype=float),
        3.0,
    )
    vy = -frame["local_velocity_x"].to_numpy(dtype=float)
    yaw_rate = -frame["yaw_rate_rad_s"].to_numpy(dtype=float)
    steer = frame["steer"].to_numpy(dtype=float)
    vy_dot = np.gradient(vy, time_s)
    yaw_accel = np.gradient(yaw_rate, time_s)
    ay_true = vy_dot + vx * yaw_rate
    map_speeds = model["direct_model_map_kmh"]
    c_lat = _interp(
        speed_kmh,
        map_speeds,
        model["lateral_velocity_damping_map"],
    )
    c_lat_yaw = _interp(
        speed_kmh,
        map_speeds,
        model["lateral_yaw_coupling_map"],
    )
    c_yaw_vel = _interp(
        speed_kmh,
        map_speeds,
        model["yaw_velocity_coupling_map"],
    )
    c_yaw_damp = _interp(
        speed_kmh,
        map_speeds,
        model["yaw_rate_damping_map"],
    )
    b_lat = _interp(
        speed_kmh,
        model["steer_lateral_gain_map_kmh"],
        model["steer_lateral_gain_map"],
    )
    b_yaw = _interp(
        speed_kmh,
        model["steer_yaw_gain_map_kmh"],
        model["steer_yaw_gain_map"],
    )
    ay_predicted = (
        c_lat * (-vy / vx)
        + c_lat_yaw * (yaw_rate / vx)
        + b_lat * steer
    )
    yaw_accel_predicted = (
        c_yaw_vel * (vy / vx)
        + c_yaw_damp * (yaw_rate / vx)
        + b_yaw * steer
    )
    valid = (
        (frame["tyres_out"].to_numpy(dtype=float) == 0.0)
        & (frame["surface_grip"].to_numpy(dtype=float) >= 0.90)
        & (frame["wheel_slip_abs_max"].to_numpy(dtype=float) < 0.25)
        & (speed_kmh >= 40.0)
        & (speed_kmh <= 190.0)
        & (vx > 5.0)
    )
    replay = {
        "time_s": time_s,
        "speed_kmh": speed_kmh,
        "steer": steer,
        "ay_true": ay_true,
        "ay_predicted": ay_predicted,
        "yaw_accel_true": yaw_accel,
        "yaw_accel_predicted": yaw_accel_predicted,
        "valid": valid,
    }
    segments = _select_replay_windows(replay)

    fig, axes = plt.subplots(
        2,
        2,
        figsize=(16, 9),
        facecolor=COLORS["paper"],
    )
    for row, segment in enumerate(segments):
        start = int(segment["start_index"])
        end = int(segment["end_index"])
        segment_time = time_s[start:end] - time_s[start]
        for column, (true_key, predicted_key, ylabel) in enumerate(
            (
                (
                    "ay_true",
                    "ay_predicted",
                    r"body lateral acceleration [m/s$^2$]",
                ),
                (
                    "yaw_accel_true",
                    "yaw_accel_predicted",
                    r"yaw acceleration [rad/s$^2$]",
                ),
            )
        ):
            ax = axes[row, column]
            ax.plot(
                segment_time,
                replay[true_key][start:end],
                color=COLORS["blue"],
                linewidth=1.7,
                label="measured",
            )
            ax.plot(
                segment_time,
                replay[predicted_key][start:end],
                color=COLORS["orange"],
                linewidth=1.5,
                linestyle="--",
                label="model",
            )
            r2_key = "lateral_r2" if column == 0 else "yaw_r2"
            ax.set_title(
                f"{segment['name']}: "
                f"{segment['median_speed_kmh']:.0f} km/h, "
                rf"$R^2={segment[r2_key]:.3f}$",
                loc="left",
                fontweight="bold",
            )
            ax.set_xlabel("replay time [s]")
            ax.set_ylabel(ylabel)
            ax.set_facecolor(COLORS["white"])
            ax.grid(color=COLORS["grid"], alpha=0.75)
            ax.set_axisbelow(True)
            if row == 0 and column == 0:
                ax.legend(frameon=False, loc="best")

    fig.suptitle(
        "Lateral identification replay validation",
        x=0.055,
        y=0.98,
        ha="left",
        fontsize=21,
        fontweight="bold",
        color=COLORS["ink"],
    )
    fig.text(
        0.057,
        0.935,
        "One-step model prediction from measured states and steering input",
        ha="left",
        fontsize=10.5,
        color=COLORS["muted"],
    )
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    fig.savefig(FIGURE_DIR / "model_replay_comparison.png", dpi=180)
    plt.close(fig)
    return {
        "source": str(log_path.relative_to(PROJECT_ROOT)),
        "prediction": "one-step response from measured states",
        "segments": segments,
    }


def _load_steering_samples(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    frame = pd.read_csv(path)
    speed = pd.to_numeric(frame["speed_kmh"], errors="coerce")
    steer = pd.to_numeric(frame["steer"], errors="coerce")
    yaw_rate = -pd.to_numeric(frame["yaw_rate_rad_s"], errors="coerce")
    lateral_g = pd.to_numeric(frame["lateral_accel_g"], errors="coerce").abs()
    slip = pd.to_numeric(frame["wheel_slip_abs_max"], errors="coerce")
    tyres_out = pd.to_numeric(frame["tyres_out"], errors="coerce")
    valid = (
        (speed >= 20.0)
        & (steer.abs() >= 0.015)
        & (yaw_rate.abs() >= 0.02)
        & (lateral_g <= 0.70)
        & (slip <= 0.35)
        & (tyres_out == 0)
    )
    return (
        speed[valid].to_numpy(dtype=float),
        steer[valid].to_numpy(dtype=float),
        yaw_rate[valid].to_numpy(dtype=float),
    )


def _derive_steady_state_metrics() -> dict[str, float]:
    model_payload = json.loads(MODEL_PATH.read_text(encoding="utf-8"))
    model = model_payload["model"]
    speed_kmh = np.linspace(20.0, 240.0, 1000)
    speed_mps = speed_kmh / 3.6
    map_keys = (
        "lateral_velocity_damping_map",
        "lateral_yaw_coupling_map",
        "yaw_velocity_coupling_map",
        "yaw_rate_damping_map",
        "steer_lateral_gain_map",
        "steer_yaw_gain_map",
    )
    (
        lateral_damping,
        lateral_yaw,
        yaw_velocity,
        yaw_damping,
        steer_lateral,
        steer_yaw,
    ) = tuple(
        _interp(
            speed_kmh,
            model["direct_model_map_kmh"],
            model[key],
        )
        for key in map_keys
    )
    uv_over_r = np.zeros_like(speed_mps)
    for index, speed in enumerate(speed_mps):
        state_matrix = np.asarray(
            [
                [
                    -lateral_damping[index] / speed,
                    lateral_yaw[index] / speed - speed,
                ],
                [
                    yaw_velocity[index] / speed,
                    yaw_damping[index] / speed,
                ],
            ]
        )
        steady_state = np.linalg.solve(
            state_matrix,
            -np.asarray(
                [steer_lateral[index], steer_yaw[index]]
            ),
        )
        uv_over_r[index] = speed / steady_state[1]
    design = np.column_stack(
        [np.ones_like(speed_mps), speed_mps * speed_mps]
    )
    intercept, slope = (
        float(value)
        for value in np.linalg.lstsq(
            design,
            uv_over_r,
            rcond=None,
        )[0]
    )
    k_steer = 2.85 / max(intercept, 1e-6)
    return {
        "speed_range_kmh": (20.0, 240.0),
        "intercept": intercept,
        "slope": slope,
        "k_steer_rad_per_axis": k_steer,
        "understeer_gradient": slope * k_steer,
    }


def build_steering_calibration() -> dict[str, object]:
    log_path = (
        PROJECT_ROOT
        / "logs"
        / "manual_steering_calibration_highspeed.csv"
    )
    speed_kmh, steer, yaw_rate = _load_steering_samples(log_path)
    speed_mps = speed_kmh / 3.6
    x = speed_mps * speed_mps
    y = steer * speed_mps / yaw_rate
    design = np.column_stack([np.ones_like(x), x])
    coefficients, *_ = np.linalg.lstsq(design, y, rcond=None)
    intercept, slope = (float(value) for value in coefficients)
    wheelbase = 2.85
    k_steer = wheelbase / max(intercept, 1e-6)
    understeer = slope * k_steer
    fitted = design @ coefficients
    r2 = 1.0 - float(
        np.sum((fitted - y) ** 2)
        / max(np.sum((y - np.mean(y)) ** 2), 1e-12)
    )
    derived = _derive_steady_state_metrics()

    bands = []
    for lower, upper in (
        (20.0, 40.0),
        (40.0, 65.0),
        (65.0, 100.0),
        (100.0, 160.0),
        (160.0, 260.0),
    ):
        mask = (speed_kmh >= lower) & (speed_kmh < upper)
        if not np.any(mask):
            continue
        band_speed = speed_mps[mask]
        band_steer = steer[mask]
        band_yaw = yaw_rate[mask]
        estimates = (
            band_yaw
            * (wheelbase + understeer * band_speed * band_speed)
            / (band_speed * band_steer)
        )
        bands.append(
            {
                "lower_kmh": lower,
                "upper_kmh": upper,
                "samples": int(np.sum(mask)),
                "median_k_steer": float(np.median(estimates)),
                "p10_k_steer": float(np.quantile(estimates, 0.10)),
                "p90_k_steer": float(np.quantile(estimates, 0.90)),
            }
        )

    fig, axes = plt.subplots(
        1,
        2,
        figsize=(16, 6.8),
        facecolor=COLORS["paper"],
    )
    bin_edges = np.linspace(0.0, float(np.max(x)), 28)
    binned_x: list[float] = []
    binned_y: list[float] = []
    binned_count: list[int] = []
    for lower, upper in zip(bin_edges[:-1], bin_edges[1:]):
        mask = (x >= lower) & (x < upper)
        if int(np.sum(mask)) < 8:
            continue
        binned_x.append(float(np.median(x[mask])))
        binned_y.append(float(np.median(y[mask])))
        binned_count.append(int(np.sum(mask)))
    axes[0].scatter(
        binned_x,
        binned_y,
        s=np.asarray(binned_count) * 1.5,
        alpha=0.80,
        color=COLORS["blue"],
        edgecolors=COLORS["white"],
        linewidths=0.6,
    )
    line_x = np.linspace(0.0, float(np.max(x)), 100)
    axes[0].plot(
        line_x,
        intercept + slope * line_x,
        color=COLORS["orange"],
        linewidth=2.2,
        label=rf"fit: {intercept:.3f} + {slope:.5f} $v^2$",
    )
    axes[0].set_xlabel(r"$v^2$ [(m/s)$^2$]")
    axes[0].set_ylabel(r"$u v / r$ [rad]")
    axes[0].set_title(
        "Effective bicycle-model regression",
        loc="left",
        fontweight="bold",
    )
    axes[0].legend(frameon=False)
    axes[0].text(
        0.02,
        0.96,
        rf"$k_{{steer}}={k_steer:.4f}$, "
        rf"$K={understeer:.6f}$, $R^2={r2:.3f}$",
        transform=axes[0].transAxes,
        va="top",
        fontsize=9.5,
        color=COLORS["ink"],
    )
    axes[0].text(
        0.02,
        0.87,
        rf"identified model: $k_{{eff}}="
        rf"{derived['k_steer_rad_per_axis']:.4f}$",
        transform=axes[0].transAxes,
        va="top",
        fontsize=9,
        color=COLORS["muted"],
    )

    centers = [
        0.5 * (band["lower_kmh"] + band["upper_kmh"])
        for band in bands
    ]
    medians = [band["median_k_steer"] for band in bands]
    lower_errors = [
        band["median_k_steer"] - band["p10_k_steer"]
        for band in bands
    ]
    upper_errors = [
        band["p90_k_steer"] - band["median_k_steer"]
        for band in bands
    ]
    axes[1].errorbar(
        centers,
        medians,
        yerr=[lower_errors, upper_errors],
        fmt="o-",
        color=COLORS["green"],
        ecolor=COLORS["muted"],
        capsize=4,
        linewidth=2,
    )
    axes[1].axhline(
        k_steer,
        color=COLORS["orange"],
        linestyle="--",
        linewidth=1.5,
        label=f"data fit k_eff = {k_steer:.4f}",
    )
    axes[1].axhline(
        derived["k_steer_rad_per_axis"],
        color=COLORS["blue"],
        linestyle=":",
        linewidth=1.8,
        label=(
            "identified model k_eff = "
            f"{derived['k_steer_rad_per_axis']:.4f}"
        ),
    )
    axes[1].set_xlabel("speed [km/h]")
    axes[1].set_ylabel("steering-axis scale [rad/axis]")
    axes[1].set_title(
        "Band-wise effective scale estimate",
        loc="left",
        fontweight="bold",
    )
    axes[1].legend(frameon=False)

    for ax in axes:
        ax.set_facecolor(COLORS["white"])
        ax.grid(color=COLORS["grid"], alpha=0.75)
        ax.set_axisbelow(True)
    fig.suptitle(
        "Steady-state axis-response consistency check",
        x=0.055,
        y=0.98,
        ha="left",
        fontsize=21,
        fontweight="bold",
        color=COLORS["ink"],
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(FIGURE_DIR / "steering_calibration.png", dpi=180)
    plt.close(fig)

    return {
        "source": str(log_path.relative_to(PROJECT_ROOT)),
        "valid_samples": int(len(x)),
        "intercept": intercept,
        "slope": slope,
        "k_steer_rad_per_axis": k_steer,
        "understeer_gradient": understeer,
        "fit_r2": r2,
        "derived_from_identified_model": derived,
        "bands": bands,
    }


def build_coupled_speed_planning() -> dict[str, object]:
    fig, ax = plt.subplots(figsize=(16, 8.6), facecolor=COLORS["paper"])
    ax.set_facecolor(COLORS["paper"])
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 8.6)
    ax.axis("off")
    ax.text(
        0.55,
        8.10,
        "Coupled speed-planning and longitudinal-lateral coordination",
        fontsize=21,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax.text(
        0.57,
        7.72,
        "The planner and MPC layers exchange speed, curvature and lateral-load constraints every cycle",
        fontsize=10.5,
        color=COLORS["muted"],
    )

    top = [
        (0.8, "Curvature", r"$\kappa_i$", COLORS["cyan"]),
        (3.3, "Base speed", r"$v_\kappa=\sqrt{a_{lat}/|\kappa|}$", COLORS["cyan"]),
        (5.8, "Band gain", r"$v_s=v_\kappa s(v)$", COLORS["blue"]),
        (8.3, "Grip cap", r"$v_s\leq\sqrt{a_{cap}/|\kappa|}$", COLORS["blue"]),
        (10.8, "High-speed lead", r"$\tau_{eff}=\tau_0+g_{hs}(v-200)$", COLORS["orange"]),
        (13.3, "Preview", r"$v_{preview}[k]$", COLORS["orange"]),
    ]
    for index, (x, title, body, color) in enumerate(top):
        _card(ax, x, 5.25, 2.0, 1.65, title, body, color)
        if index < len(top) - 1:
            _arrow(
                ax,
                (x + 2.0, 6.08),
                (top[index + 1][0], 6.08),
                COLORS["muted"],
            )

    _card(
        ax,
        1.2,
        2.4,
        3.5,
        1.85,
        "Lateral load",
        r"$|a_y|,\ |\delta|$" + "\n" + "combined braking scale",
        COLORS["orange"],
    )
    _card(
        ax,
        5.7,
        2.4,
        4.5,
        1.85,
        "Longitudinal MPC",
        r"$J_{lon}$ with scheduled $R_a,R_j$" + "\n"
        + r"$a_x\in[-a_{brake},a_{accel}]$",
        COLORS["green"],
    )
    _card(
        ax,
        11.0,
        2.4,
        3.8,
        1.85,
        "Pedal map",
        "throttle / brake\nrate and slip limits",
        COLORS["green"],
    )
    _arrow(ax, (14.9, 5.25), (12.9, 4.25), COLORS["green"], rad=-0.1)
    _arrow(ax, (12.9, 3.32), (10.2, 3.32), COLORS["green"])
    _arrow(ax, (5.7, 3.32), (4.7, 3.32), COLORS["orange"])

    _card(
        ax,
        5.7,
        0.55,
        4.5,
        1.35,
        "Vehicle",
        "speed, yaw rate, lateral acceleration",
        COLORS["red"],
    )
    _arrow(ax, (14.8, 2.4), (8.7, 1.90), COLORS["red"], rad=-0.12)
    _arrow(ax, (5.7, 1.22), (2.95, 2.40), COLORS["red"], rad=0.1)
    _arrow(ax, (1.2, 4.25), (1.8, 5.25), COLORS["cyan"])
    fig.savefig(FIGURE_DIR / "coupled_speed_planning.png", dpi=180)
    plt.close(fig)

    return {
        "coupling": [
            "curvature and grip cap the speed envelope",
            "high-speed response time advances braking",
            "lateral load and steering scale the braking capability and MPC effort",
            "longitudinal MPC generates acceleration; pedal map applies rate and slip limits",
            "vehicle speed and lateral acceleration feed back to the next planning cycle",
        ]
    }


def build_model_construction() -> dict[str, object]:
    fig, ax = plt.subplots(figsize=(16, 9), facecolor=COLORS["paper"])
    ax.set_facecolor(COLORS["paper"])
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis("off")
    ax.text(
        0.55,
        8.45,
        "From nonlinear vehicle equations to real-time QP models",
        fontsize=21,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax.text(
        0.57,
        8.05,
        "The two controllers share the same modeling sequence, but retain different state and actuator structures",
        fontsize=10.5,
        color=COLORS["muted"],
    )

    rows = [
        (
            5.25,
            "Lateral",
            COLORS["blue"],
            [
                (
                    "Nonlinear bicycle",
                    r"$m(\dot v_y+v_xr)=F_{yf}+F_{yr}$" + "\n"
                    + r"$I_z\dot r=aF_{yf}-bF_{yr}$",
                ),
                (
                    "Linear tire expansion",
                    r"$F_{yf}=C_f\alpha_f,\ F_{yr}=C_r\alpha_r$" + "\n"
                    + r"$\alpha_f\approx\delta-(v_y+ar)/v_x$",
                ),
                (
                    "Frenet linearization",
                    r"$\dot x=A_c(v_x)x+B_c(v_x)u$" + "\n"
                    + "small-angle and fixed-speed horizon",
                ),
                (
                    "Actuator augmentation",
                    r"$\dot\delta=(\delta_{cmd}-\delta)/\tau$" + "\n"
                    + r"$A_d=I+A_{aug}\Delta t$",
                ),
                (
                    "Condensed LMPC QP",
                    "state error + control\n+ control-rate cost",
                ),
            ],
        ),
        (
            1.55,
            "Longitudinal",
            COLORS["orange"],
            [
                (
                    "Nonlinear speed",
                    r"$\dot v=a_x-a_{coast}(v)$",
                ),
                (
                    "Actuator lag",
                    r"$\dot a=(u-a)/\tau_a$",
                ),
                (
                    "Forward Euler",
                    r"$a_{k+1}=(1-\alpha)a_k+\alpha u_k$" + "\n"
                    + r"$\alpha=\Delta t/\tau_a$",
                ),
                (
                    "Speed prediction",
                    r"$v_{k+1}=v_k+\Delta t(a_{k+1}-a_{coast})$",
                ),
                (
                    "Condensed MPC QP",
                    "speed + acceleration\n+ jerk constraints",
                ),
            ],
        ),
    ]

    width = 2.72
    height = 2.15
    xs = [0.45, 3.45, 6.45, 9.45, 12.45]
    for y, label, color, cards in rows:
        ax.text(
            0.52,
            y + height + 0.22,
            label,
            fontsize=12,
            fontweight="bold",
            color=color,
        )
        for x, (title, body) in zip(xs, cards):
            _card(ax, x, y, width, height, title, body, color)
        for index in range(len(xs) - 1):
            _arrow(
                ax,
                (xs[index] + width, y + height / 2),
                (xs[index + 1], y + height / 2),
                color,
            )

    fig.savefig(FIGURE_DIR / "model_construction.png", dpi=180)
    plt.close(fig)
    return {
        "lateral": [
            "nonlinear dynamic bicycle equation",
            "linear tire force approximation and small-angle Frenet errors",
            "continuous A_c(vx), B_c(vx)",
            "steering actuator augmentation",
            "discrete condensed QP",
        ],
        "longitudinal": [
            "coast-down nonlinear speed equation",
            "first-order acceleration response",
            "forward Euler discretization",
            "speed prediction",
            "acceleration and jerk constrained QP",
        ],
    }


def main() -> int:
    _style()
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    diagnostics = {
        "weight_schedules": build_weight_schedules(),
        "model_identification": build_model_identification(),
        "model_replay_comparison": build_model_replay_comparison(),
        "coupled_speed_planning": build_coupled_speed_planning(),
        "model_construction": build_model_construction(),
    }
    output = DATA_DIR / "model_diagnostics.json"
    output.write_text(
        json.dumps(diagnostics, indent=2, ensure_ascii=True),
        encoding="utf-8",
    )
    for path in sorted(FIGURE_DIR.glob("*.png")):
        if path.name in {
            "weight_schedules.png",
            "model_identification.png",
            "model_replay_comparison.png",
            "coupled_speed_planning.png",
            "model_construction.png",
        }:
            print(path.relative_to(PROJECT_ROOT))
    print(output.relative_to(PROJECT_ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
