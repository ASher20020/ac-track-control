from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch

from actc.track import TrackPath, resolve_fast_lane
from tools.build_comparison_figures import (
    find_auto_fastest_lap,
    find_manual_fastest_lap,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIGURE_DIR = PROJECT_ROOT / "assets" / "figures"

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
    "navy": "#0A1726",
    "navy_2": "#10263A",
    "cream": "#F8F4EA",
}

SPEED_CMAP = LinearSegmentedColormap.from_list(
    "speed",
    [
        "#2857A4",
        "#1688B0",
        "#20A486",
        "#E4B63E",
        "#E66A2C",
    ],
)


def _style_font() -> None:
    plt.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "Bahnschrift",
        "Segoe UI",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False


def _format_lap_time(seconds: float) -> str:
    minutes = int(seconds // 60)
    remainder = seconds - minutes * 60
    return f"{minutes}:{remainder:06.3f}"


def _card(
    ax: plt.Axes,
    x: float,
    y: float,
    width: float,
    height: float,
    *,
    edge: str = COLORS["grid"],
    fill: str = COLORS["white"],
    radius: float = 0.035,
) -> None:
    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            width,
            height,
            boxstyle=f"round,pad=0.015,rounding_size={radius}",
            linewidth=1.4,
            edgecolor=edge,
            facecolor=fill,
        )
    )


def _arrow(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    color: str,
    rad: float = 0.0,
    linewidth: float = 1.8,
) -> None:
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=13,
            linewidth=linewidth,
            color=color,
            connectionstyle=f"arc3,rad={rad}",
        )
    )


def _load_fastest_lap(
    path: Path,
    *,
    automatic: bool,
) -> tuple[float, pd.DataFrame]:
    frame = pd.read_csv(path).sort_values("timestamp")
    if automatic:
        return find_auto_fastest_lap(
            frame,
            minimum_duration_s=60.0,
            maximum_duration_s=700.0,
        )
    return find_manual_fastest_lap(
        frame,
        minimum_duration_s=60.0,
        maximum_duration_s=300.0,
    )


def _map_transform(
    frames: list[pd.DataFrame],
) -> tuple[np.ndarray, np.ndarray]:
    points = np.vstack(
        [
            frame[["position_x", "position_z"]].to_numpy(dtype=float)
            for frame in frames
        ]
    )
    center = np.mean(points, axis=0)
    centered = points - center
    covariance = np.cov(centered, rowvar=False)
    values, vectors = np.linalg.eigh(covariance)
    basis = vectors[:, np.argsort(values)[::-1]]
    if np.linalg.det(basis) < 0:
        basis[:, 1] *= -1
    return center, basis


def _transform_points(
    frame: pd.DataFrame,
    center: np.ndarray,
    basis: np.ndarray,
) -> np.ndarray:
    points = frame[["position_x", "position_z"]].to_numpy(dtype=float)
    return (points - center) @ basis


def _draw_speed_track(
    ax: plt.Axes,
    points: np.ndarray,
    speed_kmh: np.ndarray,
    *,
    norm: Normalize,
    linewidth: float = 2.5,
    alpha: float = 1.0,
    close: bool = False,
) -> LineCollection:
    closed_points = (
        np.vstack([points, points[0]])
        if close
        else points
    )
    closed_speed = (
        np.concatenate([speed_kmh, speed_kmh[:1]])
        if close
        else speed_kmh
    )
    segments = np.stack(
        [closed_points[:-1], closed_points[1:]],
        axis=1,
    )
    segment_speed = 0.5 * (closed_speed[:-1] + closed_speed[1:])
    collection = LineCollection(
        segments,
        cmap=SPEED_CMAP,
        norm=norm,
        linewidth=linewidth,
        alpha=alpha,
        zorder=3,
    )
    collection.set_array(segment_speed)
    ax.add_collection(collection)
    return collection


def _load_track_path(track_name: str, configuration: str = "") -> TrackPath:
    fast_lane = resolve_fast_lane(track_name, configuration)
    return TrackPath.load(
        fast_lane,
        centerline=True,
        preserve_fast_lane=True,
        max_speed_kmh=300.0,
        min_speed_kmh=25.0,
        lateral_accel_mps2=6.867,
        accel_mps2=3.924,
        brake_mps2=14.0,
    ).resampled(2.0)


def _speed_on_path(
    frame: pd.DataFrame,
    path: TrackPath,
) -> np.ndarray:
    count = len(path.points)
    indices = np.rint(
        frame["normalized_position"].to_numpy(dtype=float) * count
    ).astype(int) % count
    values = frame["speed_kmh"].to_numpy(dtype=float)
    grouped = pd.Series(values).groupby(indices).median()
    speed = np.full(count, np.nan, dtype=float)
    speed[grouped.index.to_numpy(dtype=int)] = grouped.to_numpy(dtype=float)
    missing = ~np.isfinite(speed)
    if np.any(missing):
        known = np.flatnonzero(~missing)
        if known.size < 2:
            return np.full(count, float(np.nanmedian(speed)), dtype=float)
        extended_index = np.concatenate(
            [known - count, known, known + count]
        )
        extended_speed = np.concatenate(
            [speed[known], speed[known], speed[known]]
        )
        speed = np.interp(
            np.arange(count, dtype=float),
            extended_index,
            extended_speed,
        )
    return speed


def build_hero() -> None:
    log_path = PROJECT_ROOT / "logs" / "lmpc_nordschleife_v2_safe.csv"
    _, lap = _load_fastest_lap(log_path, automatic=True)
    track_path = _load_track_path("ks_nordschleife", "nordschleife")
    points = np.asarray(track_path.points, dtype=float)
    speed = _speed_on_path(lap, track_path)
    points = points / 1000.0

    fig, ax = plt.subplots(figsize=(16, 7.4), facecolor=COLORS["navy"])
    ax.set_facecolor(COLORS["navy"])
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 7.4)
    ax.axis("off")

    for x in np.linspace(0.4, 15.6, 10):
        ax.plot(
            [x, x],
            [0.3, 7.1],
            color="#17334B",
            linewidth=0.6,
            alpha=0.35,
            zorder=0,
        )
    for y in np.linspace(0.4, 7.0, 7):
        ax.plot(
            [0.4, 15.6],
            [y, y],
            color="#17334B",
            linewidth=0.6,
            alpha=0.35,
            zorder=0,
        )

    ax.text(
        0.8,
        6.35,
        "AC TRACK CONTROL",
        fontsize=29,
        fontweight="bold",
        color=COLORS["white"],
    )
    ax.text(
        0.82,
        5.78,
        "Planning + lateral MPC + longitudinal MPC + AC closed loop",
        fontsize=13,
        color="#A9C5D5",
    )
    ax.text(
        0.82,
        5.03,
        "A reusable simulator testbed for autonomous racing research.",
        fontsize=11.5,
        color="#D0E0E8",
    )

    metric_cards = [
        ("3", "validated tracks"),
        ("50 Hz", "closed-loop control"),
        ("20 x 20 ms", "lateral LMPC"),
        ("50 x 50 ms", "longitudinal MPC"),
    ]
    for index, (value, label) in enumerate(metric_cards):
        x = 0.8 + index * 1.72
        _card(
            ax,
            x,
            3.35,
            1.5,
            1.12,
            edge="#28445D",
            fill=COLORS["navy_2"],
            radius=0.025,
        )
        ax.text(
            x + 0.20,
            4.11,
            value,
            color=COLORS["white"],
            fontsize=15,
            fontweight="bold",
        )
        ax.text(
            x + 0.20,
            3.68,
            label,
            color="#9EB9CA",
            fontsize=9.5,
        )

    ax.text(
        0.82,
        2.58,
        "Fastest verified automatic lap",
        fontsize=11,
        color="#8BA9BB",
    )
    ax.text(
        0.82,
        2.04,
        "8:46.82",
        fontsize=31,
        fontweight="bold",
        color="#F1A24B",
    )
    ax.text(
        0.85,
        1.58,
        "Nordschleife V2 safe  |  0 tyres out  |  20.66 km",
        fontsize=10.5,
        color="#C1D6E1",
    )

    map_ax = fig.add_axes([0.53, 0.10, 0.43, 0.78])
    map_ax.set_facecolor(COLORS["navy"])
    map_points = points.copy()
    map_points[:, 0] -= np.mean(map_points[:, 0])
    map_points[:, 1] -= np.mean(map_points[:, 1])
    map_points /= max(np.ptp(map_points[:, 0]), np.ptp(map_points[:, 1]))

    norm = Normalize(vmin=float(np.percentile(speed, 5)), vmax=float(speed.max()))
    _draw_speed_track(
        map_ax,
        map_points,
        speed,
        norm=norm,
        linewidth=4.0,
        close=True,
    )
    map_ax.scatter(
        [map_points[0, 0]],
        [map_points[0, 1]],
        s=90,
        color=COLORS["white"],
        edgecolor=COLORS["orange"],
        linewidth=2.0,
        zorder=5,
    )
    map_ax.text(
        map_points[0, 0] + 0.03,
        map_points[0, 1] + 0.03,
        "START / FINISH",
        color=COLORS["white"],
        fontsize=8.5,
        fontweight="bold",
        zorder=6,
    )
    map_ax.set_xlim(map_points[:, 0].min() - 0.06, map_points[:, 0].max() + 0.06)
    map_ax.set_ylim(map_points[:, 1].min() - 0.06, map_points[:, 1].max() + 0.06)
    map_ax.set_aspect("equal")
    map_ax.axis("off")

    fig.savefig(
        FIGURE_DIR / "hero_overview.png",
        dpi=200,
        facecolor=COLORS["navy"],
    )
    plt.close(fig)


def build_track_maps() -> None:
    specs = [
        {
            "name": "Shanghai",
            "human_path": PROJECT_ROOT / "logs" / "vehicle_dynamics_reference_20min.csv",
            "auto_path": PROJECT_ROOT / "logs" / "lmpc_shanghai_v63_h50dt05_265_5lap.csv",
            "human_label": f"Human {_format_lap_time(136.247968)}",
            "auto_label": f"Auto {_format_lap_time(143.542842)}",
            "color": COLORS["blue"],
            "track_name": "tr_shanghai",
            "configuration": "advanced",
        },
        {
            "name": "Zhejiang",
            "human_path": PROJECT_ROOT / "logs" / "manual_reference_v49_moza.csv",
            "auto_path": PROJECT_ROOT / "logs" / "lmpc_v59_rollback_5lap.csv",
            "human_label": f"Human {_format_lap_time(94.328507)}",
            "auto_label": f"Auto {_format_lap_time(103.764338)}",
            "color": COLORS["green"],
            "track_name": "st_zhejiang",
            "configuration": "layout_main",
        },
        {
            "name": "Nordschleife",
            "human_path": None,
            "auto_path": PROJECT_ROOT / "logs" / "lmpc_nordschleife_v2_safe.csv",
            "human_label": "",
            "auto_label": f"Auto {_format_lap_time(526.824546)}",
            "color": COLORS["orange"],
            "track_name": "ks_nordschleife",
            "configuration": "nordschleife",
        },
    ]

    fig, axes = plt.subplots(
        1,
        3,
        figsize=(17, 6.8),
        facecolor=COLORS["paper"],
    )
    speed_min = 0.0
    speed_max = 300.0
    norm = Normalize(vmin=speed_min, vmax=speed_max)

    for ax, spec in zip(axes, specs):
        auto_time, auto_lap = _load_fastest_lap(
            Path(spec["auto_path"]),
            automatic=True,
        )
        track_path = _load_track_path(
            str(spec["track_name"]),
            str(spec["configuration"]),
        )
        path_points = np.asarray(track_path.points, dtype=float)
        auto_speed = _speed_on_path(auto_lap, track_path)
        frames = [pd.DataFrame(path_points, columns=["position_x", "position_z"])]
        human_lap = None
        if spec["human_path"] is not None:
            _, human_lap = _load_fastest_lap(
                Path(spec["human_path"]),
                automatic=False,
            )
        if spec["name"] == "Nordschleife":
            center = np.zeros(2, dtype=float)
            basis = np.eye(2, dtype=float)
        else:
            center, basis = _map_transform(frames)
        auto_points = (path_points - center) @ basis

        ax.set_facecolor(COLORS["white"])
        for spine in ax.spines.values():
            spine.set_color(COLORS["grid"])
        ax.set_xticks([])
        ax.set_yticks([])

        all_points = [auto_points]
        if human_lap is not None:
            ax.plot(
                auto_points[:, 0],
                auto_points[:, 1],
                color="#BCC6D4",
                linewidth=1.35,
                alpha=0.85,
                zorder=1,
            )
            ax.text(
                0.0,
                -0.075,
                spec["human_label"],
                transform=ax.transAxes,
                fontsize=9.5,
                color=COLORS["muted"],
                va="top",
            )

        _draw_speed_track(
            ax,
            auto_points,
            auto_speed,
            norm=norm,
            linewidth=2.4,
            close=True,
        )
        ax.scatter(
            [auto_points[0, 0]],
            [auto_points[0, 1]],
            s=46,
            color=COLORS["white"],
            edgecolor=spec["color"],
            linewidth=1.8,
            zorder=5,
        )

        joined = np.vstack(all_points)
        span_x = max(np.ptp(joined[:, 0]), 1.0)
        span_y = max(np.ptp(joined[:, 1]), 1.0)
        ax.set_xlim(
            joined[:, 0].min() - 0.07 * span_x,
            joined[:, 0].max() + 0.07 * span_x,
        )
        ax.set_ylim(
            joined[:, 1].min() - 0.07 * span_y,
            joined[:, 1].max() + 0.07 * span_y,
        )
        ax.set_aspect("equal")
        ax.set_title(
            spec["name"],
            loc="left",
            fontsize=18,
            fontweight="bold",
            color=COLORS["ink"],
            pad=12,
        )
        ax.text(
            0.0,
            -0.125,
            spec["auto_label"],
            transform=ax.transAxes,
            fontsize=9.8,
            color=spec["color"],
            fontweight="bold",
            va="top",
        )

    scalar = plt.cm.ScalarMappable(norm=norm, cmap=SPEED_CMAP)
    scalar.set_array([])
    colorbar = fig.colorbar(
        scalar,
        ax=axes,
        orientation="horizontal",
        fraction=0.047,
        pad=0.045,
        aspect=45,
    )
    colorbar.set_label("speed [km/h]", color=COLORS["muted"])
    colorbar.ax.tick_params(colors=COLORS["muted"])
    colorbar.outline.set_edgecolor(COLORS["grid"])
    fig.suptitle(
        "Measured track trajectories and speed distribution",
        x=0.055,
        y=0.98,
        ha="left",
        fontsize=21,
        fontweight="bold",
        color=COLORS["ink"],
    )
    fig.subplots_adjust(
        left=0.035,
        right=0.98,
        top=0.90,
        bottom=0.22,
        wspace=0.12,
    )
    fig.savefig(FIGURE_DIR / "track_maps.png", dpi=180)
    plt.close(fig)


def build_performance_dashboard() -> None:
    fig = plt.figure(figsize=(16, 8.6), facecolor=COLORS["paper"])
    grid = fig.add_gridspec(
        2,
        2,
        left=0.06,
        right=0.96,
        bottom=0.08,
        top=0.82,
        hspace=0.34,
        wspace=0.24,
    )

    ax_gap = fig.add_subplot(grid[0, 0])
    tracks = ["Shanghai", "Zhejiang"]
    gaps = [7.295, 9.436]
    colors = [COLORS["blue"], COLORS["green"]]
    bars = ax_gap.barh(tracks, gaps, color=colors, height=0.48)
    for bar, gap in zip(bars, gaps):
        ax_gap.text(
            gap + 0.12,
            bar.get_y() + bar.get_height() / 2,
            f"+{gap:.3f} s",
            va="center",
            color=COLORS["ink"],
            fontsize=11,
            fontweight="bold",
        )
    ax_gap.set_xlim(0, 11)
    ax_gap.set_title(
        "Gap to human teaching lap",
        loc="left",
        fontsize=15,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax_gap.set_xlabel("seconds slower")
    ax_gap.grid(axis="x", color=COLORS["grid"], alpha=0.8)
    ax_gap.set_axisbelow(True)

    ax_track = fig.add_subplot(grid[0, 1])
    error_tracks = ["Shanghai", "Zhejiang", "Nordschleife"]
    errors = [0.566, 0.535, 0.461]
    bars = ax_track.barh(
        error_tracks,
        errors,
        color=[COLORS["blue"], COLORS["green"], COLORS["orange"]],
        height=0.48,
    )
    ax_track.axvline(
        0.40,
        color=COLORS["red"],
        linestyle="--",
        linewidth=1.2,
        label="caution threshold",
    )
    for bar, error in zip(bars, errors):
        ax_track.text(
            error + 0.012,
            bar.get_y() + bar.get_height() / 2,
            f"{error:.3f} m",
            va="center",
            color=COLORS["ink"],
            fontsize=10.5,
            fontweight="bold",
        )
    ax_track.set_xlim(0, 0.72)
    ax_track.set_title(
        "p95 absolute lateral tracking error",
        loc="left",
        fontsize=15,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax_track.set_xlabel("metres")
    ax_track.grid(axis="x", color=COLORS["grid"], alpha=0.8)
    ax_track.set_axisbelow(True)
    ax_track.legend(loc="upper right", frameon=False, fontsize=8.5)

    ax_speed = fig.add_subplot(grid[1, 0])
    labels = [
        "Shanghai\nhuman",
        "Shanghai\nauto",
        "Zhejiang\nhuman",
        "Zhejiang\nauto",
        "Nordschleife\nauto",
    ]
    speeds = [273.2, 268.7, 239.4, 229.2, 293.8]
    speed_colors = [
        "#8AA8D8",
        COLORS["blue"],
        "#8FC6B5",
        COLORS["green"],
        COLORS["orange"],
    ]
    bars = ax_speed.bar(labels, speeds, color=speed_colors, width=0.62)
    for bar, value in zip(bars, speeds):
        ax_speed.text(
            bar.get_x() + bar.get_width() / 2,
            value + 4,
            f"{value:.1f}",
            ha="center",
            fontsize=9,
            color=COLORS["ink"],
            fontweight="bold",
        )
    ax_speed.set_ylim(0, 330)
    ax_speed.set_ylabel("km/h")
    ax_speed.set_title(
        "Peak speed by run",
        loc="left",
        fontsize=15,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax_speed.grid(axis="y", color=COLORS["grid"], alpha=0.8)
    ax_speed.set_axisbelow(True)
    ax_speed.tick_params(axis="x", rotation=15)

    ax_reliability = fig.add_subplot(grid[1, 1])
    ax_reliability.set_xlim(0, 1)
    ax_reliability.set_ylim(0, 1)
    ax_reliability.axis("off")
    reliability_cards = [
        (
            0.02,
            0.54,
            0.46,
            0.40,
            "Shanghai",
            "0 tyres out",
            "V63 verified / V64 candidate",
            COLORS["blue"],
        ),
        (
            0.52,
            0.54,
            0.46,
            0.40,
            "Zhejiang",
            "4 complete laps",
            "lap spread about 27 ms",
            COLORS["green"],
        ),
        (
            0.02,
            0.06,
            0.46,
            0.40,
            "Nordschleife",
            "0 tyres out",
            "first full stable lap",
            COLORS["orange"],
        ),
        (
            0.52,
            0.06,
            0.46,
            0.40,
            "Final stack",
            "50 Hz",
            "lateral + longitudinal MPC",
            COLORS["cyan"],
        ),
    ]
    for x, y, width, height, title, value, note, color in reliability_cards:
        _card(
            ax_reliability,
            x,
            y,
            width,
            height,
            edge=color,
            fill=COLORS["white"],
            radius=0.03,
        )
        ax_reliability.text(
            x + 0.035,
            y + height - 0.085,
            title,
            fontsize=10,
            color=COLORS["muted"],
        )
        ax_reliability.text(
            x + 0.035,
            y + height * 0.47,
            value,
            fontsize=14,
            color=color,
            fontweight="bold",
        )
        ax_reliability.text(
            x + 0.035,
            y + 0.07,
            note,
            fontsize=8.6,
            color=COLORS["ink"],
        )
    ax_reliability.set_title(
        "Evidence and repeatability",
        loc="left",
        fontsize=15,
        fontweight="bold",
        color=COLORS["ink"],
    )

    fig.suptitle(
        "Performance and tracking dashboard",
        x=0.055,
        y=0.97,
        ha="left",
        fontsize=23,
        fontweight="bold",
        color=COLORS["ink"],
    )
    fig.text(
        0.056,
        0.922,
        "Human comparison, control quality, peak speed and validation evidence",
        color=COLORS["muted"],
        fontsize=10.5,
    )
    fig.savefig(FIGURE_DIR / "performance_dashboard.png", dpi=180)
    plt.close(fig)


def build_calibration_pipeline() -> None:
    fig, ax = plt.subplots(figsize=(16, 8.6), facecolor=COLORS["paper"])
    ax.set_facecolor(COLORS["paper"])
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 8.6)
    ax.axis("off")

    ax.text(
        0.55,
        8.08,
        "Calibration and model-identification pipeline",
        fontsize=22,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax.text(
        0.57,
        7.68,
        "Measured data -> filtering -> regression -> speed-scheduled maps -> closed-loop validation",
        fontsize=10.7,
        color=COLORS["muted"],
    )

    stages = [
        (
            0.7,
            "01",
            "Data collection",
            "steering sweep\nbraking levels\nmanual reference",
            COLORS["cyan"],
        ),
        (
            4.0,
            "02",
            "Clean samples",
            "in-bounds\ngrip > 0.85\nslip < thresholds",
            COLORS["blue"],
        ),
        (
            7.3,
            "03",
            "Least squares",
            "lateral vy-dot\nsteady-state yaw\nbrake decel",
            COLORS["green"],
        ),
        (
            10.6,
            "04",
            "Scheduled maps",
            "speed bands\nload scaling\npedal grid",
            COLORS["orange"],
        ),
        (
            13.5,
            "05",
            "Closed-loop",
            "lap feedback\nstability gates\ngain rollback",
            COLORS["red"],
        ),
    ]

    width = 2.32
    height = 2.05
    for x, number, title, body, color in stages:
        _card(ax, x, 4.35, width, height, edge=color, radius=0.025)
        ax.text(
            x + 0.18,
            6.04,
            number,
            color=color,
            fontsize=16,
            fontweight="bold",
        )
        ax.text(
            x + 0.18,
            5.64,
            title,
            color=COLORS["ink"],
            fontsize=12,
            fontweight="bold",
        )
        ax.text(
            x + 0.18,
            4.68,
            body,
            color=COLORS["muted"],
            fontsize=9.3,
            linespacing=1.45,
        )

    for index in range(len(stages) - 1):
        x = stages[index][0] + width
        _arrow(
            ax,
            (x + 0.04, 5.38),
            (stages[index + 1][0] - 0.04, 5.38),
            color=COLORS["muted"],
        )

    _card(
        ax,
        0.7,
        0.65,
        7.05,
        2.7,
        edge=COLORS["blue"],
        radius=0.025,
    )
    ax.text(
        0.98,
        2.96,
        "Lateral model",
        color=COLORS["blue"],
        fontsize=13,
        fontweight="bold",
    )
    ax.text(
        0.98,
        2.47,
        r"$\dot v_y + v_x r = c_1(-v_y/v_x) + c_2\delta + c_3$",
        color=COLORS["ink"],
        fontsize=11,
    )
    ax.text(
        0.98,
        2.05,
        r"$\dot r = c_4(v_y/v_x) + c_5(r/v_x) + c_6\delta + c_7$",
        color=COLORS["ink"],
        fontsize=11,
    )
    ax.text(
        0.98,
        1.52,
        "5 bands: 40-70, 70-100, 100-130, 130-160, 160-200 km/h",
        color=COLORS["muted"],
        fontsize=9.5,
    )
    ax.text(
        0.98,
        1.14,
        "Lateral R²: 0.976-0.989",
        color=COLORS["green"],
        fontsize=10,
        fontweight="bold",
    )

    _card(
        ax,
        8.2,
        0.65,
        7.05,
        2.7,
        edge=COLORS["orange"],
        radius=0.025,
    )
    ax.text(
        8.48,
        2.96,
        "Longitudinal model",
        color=COLORS["orange"],
        fontsize=13,
        fontweight="bold",
    )
    ax.text(
        8.48,
        2.50,
        "Speed x pedal measured deceleration grid",
        color=COLORS["ink"],
        fontsize=11,
    )
    ax.text(
        8.48,
        2.08,
        r"$a_{k+1}=(1-\alpha)a_k+\alpha u_k$",
        color=COLORS["ink"],
        fontsize=11,
    )
    ax.text(
        8.48,
        1.66,
        r"$v_{k+1}=v_k+\Delta t(a_{k+1}-a_{coast}(v_k))$",
        color=COLORS["ink"],
        fontsize=11,
    )
    ax.text(
        8.48,
        1.14,
        "Brake range: 0-17.19 m/s²; final horizon: 50 x 0.05 s",
        color=COLORS["muted"],
        fontsize=9.5,
    )

    _arrow(
        ax,
        (4.2, 4.35),
        (4.2, 3.35),
        color=COLORS["blue"],
        rad=0.0,
    )
    _arrow(
        ax,
        (11.7, 4.35),
        (11.7, 3.35),
        color=COLORS["orange"],
        rad=0.0,
    )
    _arrow(
        ax,
        (7.75, 1.95),
        (8.20, 1.95),
        color=COLORS["green"],
        rad=0.0,
    )
    _arrow(
        ax,
        (14.65, 2.0),
        (14.65, 4.35),
        color=COLORS["red"],
        rad=0.0,
    )
    ax.text(
        14.15,
        3.20,
        "track feedback",
        color=COLORS["red"],
        fontsize=9,
        rotation=90,
        va="center",
    )

    fig.savefig(FIGURE_DIR / "calibration_pipeline.png", dpi=180)
    plt.close(fig)


def build_formula_overview() -> None:
    fig, ax = plt.subplots(figsize=(16, 9), facecolor=COLORS["paper"])
    ax.set_facecolor(COLORS["paper"])
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis("off")

    ax.text(
        0.55,
        8.48,
        "Planning, estimation and MPC equations",
        fontsize=22,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax.text(
        0.57,
        8.08,
        "The formulas implemented by the planner and fast control layer",
        fontsize=10.7,
        color=COLORS["muted"],
    )

    cards = [
        (
            0.7,
            4.75,
            7.1,
            2.85,
            "Reference-line optimization",
            COLORS["cyan"],
            [
                r"$J=t_{lap}+w_\kappa\sum\kappa_i^2\Delta s_i$",
                r"$+w_c\sum\max(0,c_{target}-c_i)^2\Delta s_i$",
                r"$+w_s\sum\|\Delta^2o_i\|^2$",
            ],
            "Minimizes lap-time proxy while preserving clearance and smooth offsets.",
        ),
        (
            8.2,
            4.75,
            7.1,
            2.85,
            "Curvature and braking limits",
            COLORS["green"],
            [
                r"$v_\kappa=\sqrt{a_{lat}/|\kappa|}$",
                r"$v_i^2\leq v_{i+1}^2+2a_{brake}\Delta s_i$",
                r"$\tau_{eff}=\tau_0+\max(0,v-200)g_{hs}$",
            ],
            "Combines grip, longitudinal acceleration and high-speed braking lead.",
        ),
        (
            0.7,
            1.15,
            7.1,
            3.05,
            "Lateral LMPC",
            COLORS["blue"],
            [
                r"$x=[e_y,e_\psi,v_x,v_y,r]^T,\quad u=[\delta,a_x]^T$",
                r"$\dot e_y=v_xe_\psi+v_y,\quad\dot e_\psi=r$",
                r"$\dot v_y=a_{33}v_y+a_{34}r+b_{31}\delta$",
                r"$\dot r=a_{43}v_y+a_{44}r+b_{41}\delta$",
                r"$J=\sum(x-x_{ref})^TQ(x-x_{ref})+u^TRu+\Delta u^TR_d\Delta u$",
            ],
            "Speed- and load-scheduled dynamic bicycle model with actuator compensation.",
        ),
        (
            8.2,
            1.15,
            7.1,
            3.05,
            "Longitudinal MPC",
            COLORS["orange"],
            [
                r"$\alpha=\Delta t/\tau_{response}$",
                r"$a_{k+1}=(1-\alpha)a_k+\alpha u_k$",
                r"$v_{k+1}=v_k+\Delta t(a_{k+1}-a_{coast}(v_k))$",
                r"$J=\sum q_v e_v^2+q_a e_a^2+r_a\Delta a^2+r_j\Delta^2a^2$",
            ],
            "Five-second preview with explicit acceleration, braking and jerk limits.",
        ),
    ]

    for x, y, width, height, title, color, formulas, note in cards:
        _card(ax, x, y, width, height, edge=color, radius=0.025)
        ax.text(
            x + 0.22,
            y + height - 0.34,
            title,
            fontsize=13,
            fontweight="bold",
            color=color,
        )
        for index, formula in enumerate(formulas):
            ax.text(
                x + 0.24,
                y + height - 0.86 - index * 0.38,
                formula,
                fontsize=10.5,
                color=COLORS["ink"],
            )
        ax.text(
            x + 0.24,
            y + 0.18,
            note,
            fontsize=8.7,
            color=COLORS["muted"],
        )

    ax.text(
        0.72,
        0.48,
        "Online constraints: steering position / rate / jerk | acceleration / braking | command freshness",
        fontsize=10.2,
        color=COLORS["ink"],
        fontweight="bold",
    )
    fig.savefig(FIGURE_DIR / "mpc_formulas.png", dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--only",
        choices=[
            "hero",
            "maps",
            "dashboard",
            "calibration",
        ],
        default=None,
    )
    args = parser.parse_args()
    _style_font()
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)

    builders = {
        "hero": build_hero,
        "maps": build_track_maps,
        "dashboard": build_performance_dashboard,
        "calibration": build_calibration_pipeline,
    }
    outputs = {
        "hero": "hero_overview.png",
        "maps": "track_maps.png",
        "dashboard": "performance_dashboard.png",
        "calibration": "calibration_pipeline.png",
    }
    selected = [args.only] if args.only else list(builders)
    for name in selected:
        builders[name]()
        print(FIGURE_DIR / outputs[name])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
