from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from tools.figure_theme import THEME, apply_dark_theme


PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIGURE_DIR = PROJECT_ROOT / "assets" / "figures"

COLORS = THEME


def _add_box(
    ax: plt.Axes,
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    body: str,
    *,
    edge: str,
    fill: str = COLORS["white"],
) -> None:
    box = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.018,rounding_size=0.03",
        linewidth=1.5,
        edgecolor=edge,
        facecolor=fill,
    )
    ax.add_patch(box)
    ax.text(
        x + width / 2,
        y + height * 0.68,
        title,
        ha="center",
        va="center",
        color=COLORS["ink"],
        fontsize=12,
        fontweight="bold",
    )
    ax.text(
        x + width / 2,
        y + height * 0.34,
        body,
        ha="center",
        va="center",
        color=COLORS["muted"],
        fontsize=9,
        linespacing=1.35,
    )


def _arrow(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    color: str = COLORS["blue"],
    connectionstyle: str = "arc3",
) -> None:
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=14,
            linewidth=1.8,
            color=color,
            connectionstyle=connectionstyle,
        )
    )


def _draw_controller_icon(
    ax: plt.Axes,
    x: float,
    y: float,
    color: str,
) -> None:
    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            0.55,
            0.30,
            boxstyle="round,pad=0.015,rounding_size=0.05",
            linewidth=1.4,
            edgecolor=color,
            facecolor=COLORS["white"],
        )
    )
    ax.add_patch(plt.Circle((x + 0.14, y + 0.15), 0.045, color=color))
    ax.add_patch(plt.Circle((x + 0.42, y + 0.19), 0.035, color=color))
    ax.add_patch(plt.Circle((x + 0.42, y + 0.10), 0.035, color=color))


def _draw_car_icon(
    ax: plt.Axes,
    x: float,
    y: float,
    color: str,
) -> None:
    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            0.24,
            0.52,
            boxstyle="round,pad=0.008,rounding_size=0.05",
            linewidth=1.4,
            edgecolor=color,
            facecolor="#FFF8E7",
        )
    )
    for wheel_y in (y + 0.05, y + 0.39):
        ax.add_patch(
            FancyBboxPatch(
                (x - 0.07, wheel_y),
                0.07,
                0.12,
                boxstyle="round,pad=0.006,rounding_size=0.02",
                linewidth=0,
                facecolor=color,
            )
        )
        ax.add_patch(
            FancyBboxPatch(
                (x + 0.24, wheel_y),
                0.07,
                0.12,
                boxstyle="round,pad=0.006,rounding_size=0.02",
                linewidth=0,
                facecolor=color,
            )
        )
    _arrow(
        ax,
        (x + 0.12, y + 0.58),
        (x + 0.12, y + 0.78),
        color=color,
    )


def _draw_monitor_icon(
    ax: plt.Axes,
    x: float,
    y: float,
    color: str,
) -> None:
    ax.add_patch(
        FancyBboxPatch(
            (x, y),
            0.65,
            0.40,
            boxstyle="round,pad=0.01,rounding_size=0.025",
            linewidth=1.5,
            edgecolor=color,
            facecolor=COLORS["white"],
        )
    )
    ax.text(
        x + 0.325,
        y + 0.20,
        "AC",
        ha="center",
        va="center",
        fontsize=9,
        fontweight="bold",
        color=color,
    )
    ax.plot(
        [x + 0.24, x + 0.41],
        [y - 0.06, y],
        color=color,
        linewidth=1.4,
    )
    ax.plot(
        [x + 0.18, x + 0.47],
        [y - 0.06, y - 0.06],
        color=color,
        linewidth=1.4,
    )


def _draw_pages_icon(
    ax: plt.Axes,
    x: float,
    y: float,
    color: str,
) -> None:
    labels = ("PH", "GX", "ST")
    for index, label in enumerate(labels):
        offset = index * 0.08
        ax.add_patch(
            FancyBboxPatch(
                (x + offset, y + offset),
                0.36,
                0.46,
                boxstyle="round,pad=0.008,rounding_size=0.02",
                linewidth=1.2,
                edgecolor=color,
                facecolor=COLORS["white"],
                alpha=0.95,
            )
        )
    ax.text(
        x + 0.28,
        y + 0.28,
        "memory",
        ha="center",
        va="center",
        fontsize=7.5,
        color=color,
        fontweight="bold",
    )


def _draw_axes_icon(
    ax: plt.Axes,
    x: float,
    y: float,
    color: str,
) -> None:
    _arrow(ax, (x, y), (x + 0.35, y), color=color)
    _arrow(ax, (x, y), (x, y + 0.35), color=color)
    ax.text(x + 0.36, y - 0.02, "x", fontsize=8, color=color)
    ax.text(x - 0.02, y + 0.36, "y", fontsize=8, color=color)


def build_architecture() -> None:
    fig, ax = plt.subplots(figsize=(15, 8.2), facecolor=COLORS["paper"])
    ax.set_facecolor(COLORS["paper"])
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 8)
    ax.axis("off")

    ax.text(
        0.5,
        7.58,
        "Assetto Corsa autonomous racing closed loop",
        fontsize=20,
        fontweight="bold",
        color=COLORS["ink"],
    )
    ax.text(
        0.5,
        7.18,
        "Planning -> MPC control -> virtual actuator -> simulator -> telemetry feedback",
        fontsize=11,
        color=COLORS["muted"],
    )

    _add_box(
        ax,
        0.7,
        5.1,
        2.55,
        1.38,
        "1. Track & path",
        "fast_lane.ai + track widths\ncenterline / racing-line offsets",
        edge=COLORS["cyan"],
    )
    _add_box(
        ax,
        3.75,
        5.1,
        2.55,
        1.38,
        "2. Planning",
        "corner detection\nline and speed optimization",
        edge=COLORS["cyan"],
    )
    _add_box(
        ax,
        6.8,
        5.1,
        2.55,
        1.38,
        "3. Lateral LMPC",
        "Frenet states\nsteering sequence",
        edge=COLORS["blue"],
    )
    _add_box(
        ax,
        9.85,
        5.1,
        2.35,
        1.38,
        "4. Longitudinal MPC",
        "speed / acceleration\njerk constraints",
        edge=COLORS["blue"],
    )
    _add_box(
        ax,
        12.7,
        5.1,
        1.6,
        1.38,
        "5. Input",
        "X360 / vJoy / keys",
        edge=COLORS["green"],
    )

    _add_box(
        ax,
        5.55,
        1.15,
        3.9,
        1.55,
        "State conversion",
        "world pose, Frenet error\nyaw rate, speed, tyre state",
        edge=COLORS["green"],
    )
    _add_box(
        ax,
        0.7,
        1.15,
        3.35,
        1.55,
        "AC shared memory",
        "physics / graphics / static pages",
        edge=COLORS["green"],
    )
    _add_box(
        ax,
        10.9,
        1.15,
        3.4,
        1.55,
        "Assetto Corsa vehicle",
        "tyre, suspension, aero and track physics",
        edge=COLORS["orange"],
        fill=COLORS["panel"],
    )
    _draw_car_icon(ax, 11.05, 1.35, COLORS["orange"])
    _draw_monitor_icon(ax, 12.95, 1.35, COLORS["orange"])
    _draw_pages_icon(ax, 3.60, 1.42, COLORS["blue"])
    _draw_axes_icon(ax, 0.92, 1.48, COLORS["green"])

    for x in (3.25, 6.3, 9.35, 12.2):
        _arrow(ax, (x, 5.79), (x + 0.5, 5.79), color=COLORS["cyan"])

    _arrow(
        ax,
        (13.5, 5.1),
        (12.6, 2.78),
        color=COLORS["green"],
        connectionstyle="arc3,rad=0.12",
    )
    ax.text(
        13.05,
        3.85,
        "apply controls",
        color=COLORS["green"],
        fontsize=9,
        rotation=90,
        va="center",
    )

    _arrow(
        ax,
        (10.9, 1.93),
        (9.45, 1.93),
        color=COLORS["orange"],
    )
    ax.text(
        10.17,
        2.08,
        "read telemetry",
        ha="center",
        color=COLORS["orange"],
        fontsize=9,
    )

    _arrow(
        ax,
        (5.55, 1.93),
        (4.05, 1.93),
        color=COLORS["green"],
    )
    ax.text(
        4.80,
        2.08,
        "convert state",
        ha="center",
        color=COLORS["green"],
        fontsize=9,
    )

    _arrow(
        ax,
        (2.38, 2.7),
        (1.98, 5.1),
        color=COLORS["green"],
    )
    ax.text(
        2.12,
        3.88,
        "feedback",
        color=COLORS["green"],
        fontsize=9,
        rotation=90,
        va="center",
    )

    ax.text(
        0.72,
        0.42,
        "Hard real-time boundary: telemetry -> state -> optimization -> actuator at 50 Hz",
        fontsize=10,
        color=COLORS["ink"],
        fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "closed_loop_architecture.png", dpi=180)
    plt.close(fig)


def build_control_stack() -> None:
    fig, ax = plt.subplots(figsize=(14, 7.6), facecolor=COLORS["paper"])
    ax.set_facecolor(COLORS["paper"])
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 7.8)
    ax.axis("off")
    ax.text(
        0.5,
        7.4,
        "Control stack and interfaces",
        fontsize=20,
        fontweight="bold",
        color=COLORS["ink"],
    )

    rows = [
        (
            "Global layer",
            5.45,
            [
                ("Reference geometry", "centerline\nwidths\ncurvature"),
                ("Line optimizer", "L-BFGS-B\nclearance\nlap-time proxy"),
                ("Speed planner", "grip ellipse\naccel / brake\nband scaling"),
            ],
            COLORS["cyan"],
        ),
        (
            "Fast layer",
            2.9,
            [
                ("Lateral LMPC", "20 x 20 ms\nsteering\nactuator model"),
                ("Longitudinal MPC", "50 x 50 ms\naccel / brake\njerk"),
                ("Pedal mapper", "rate limit\nslip cut\nsteer reduction"),
            ],
            COLORS["blue"],
        ),
        (
            "Plant interface",
            0.35,
            [
                ("Virtual X360", "steer\nthrottle\nbrake"),
                ("AC vehicle", "tyres\naero\nsuspension"),
                ("Telemetry", "50 Hz\nshared memory\nrun metadata"),
            ],
            COLORS["green"],
        ),
    ]

    width = 3.45
    height = 1.35
    xs = [0.8, 5.25, 9.7]
    for label, y, boxes, color in rows:
        ax.text(0.75, y + height + 0.2, label, fontsize=12, fontweight="bold", color=color)
        for x, (title, body) in zip(xs, boxes):
            _add_box(ax, x, y, width, height, title, body, edge=color)
        if y != 0.35:
            for x in xs:
                next_y = 2.9 + height if y > 3.0 else 0.35 + height
                _arrow(
                    ax,
                    (x + width / 2, y - 0.08),
                    (x + width / 2, next_y + 0.08),
                    color=color,
                )

    ax.plot(
        [13.0, 13.6, 13.6],
        [1.72, 1.72, 6.0],
        color=COLORS["green"],
        linewidth=1.8,
    )
    _arrow(
        ax,
        (13.6, 6.0),
        (13.15, 6.0),
        color=COLORS["green"],
    )
    ax.text(
        13.67,
        3.9,
        "feedback",
        fontsize=10,
        color=COLORS["green"],
        fontweight="bold",
        rotation=90,
        va="center",
    )
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "control_stack.png", dpi=180)
    plt.close(fig)


def build_track_results() -> None:
    tracks = [
        {
            "name": "Shanghai",
            "length": "5.43 km",
            "lap": 143.542842,
            "label": "2:23.543",
            "human": "2:16.248",
            "status": "V63 verified; V64 final candidate",
            "max_speed": 268.726,
            "lat": 0.566,
            "tyres": 0,
            "color": COLORS["blue"],
        },
        {
            "name": "Zhejiang",
            "length": "3.11 km",
            "lap": 103.764338,
            "label": "1:43.764",
            "human": "1:34.329",
            "status": "4 complete laps; 2 tyres out",
            "max_speed": 229.152,
            "lat": 0.530,
            "tyres": 2,
            "color": COLORS["green"],
        },
        {
            "name": "Nordschleife",
            "length": "20.66 km",
            "lap": 526.824546,
            "label": "8:46.825",
            "human": "7:17.632",
            "status": "1 complete lap; zero tyres out",
            "max_speed": 293.8,
            "lat": 0.461,
            "tyres": 0,
            "color": COLORS["orange"],
        },
    ]
    fig, axes = plt.subplots(1, 3, figsize=(16, 6.4), facecolor=COLORS["paper"])
    for ax, item in zip(axes, tracks):
        ax.set_facecolor(COLORS["white"])
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis("off")
        ax.add_patch(
            FancyBboxPatch(
                (0.03, 0.03),
                0.94,
                0.94,
                boxstyle="round,pad=0.015,rounding_size=0.035",
                linewidth=1.6,
                edgecolor=item["color"],
                facecolor=COLORS["white"],
            )
        )
        ax.text(0.1, 0.87, item["name"], fontsize=18, fontweight="bold", color=COLORS["ink"])
        ax.text(0.1, 0.79, item["length"], fontsize=11, color=COLORS["muted"])
        ax.text(0.1, 0.63, item["label"], fontsize=30, fontweight="bold", color=item["color"])
        ax.text(0.1, 0.54, "best recorded lap", fontsize=10, color=COLORS["muted"])
        ax.plot([0.1, 0.9], [0.48, 0.48], color=COLORS["grid"], linewidth=1)
        rows = [
            ("Human reference", item["human"]),
            ("Max speed", f"{item['max_speed']:.1f} km/h"),
            ("p95 lateral", f"{item['lat']:.3f} m"),
            ("Tyres out", str(item["tyres"])),
            ("Evidence", item["status"]),
        ]
        for index, (key, value) in enumerate(rows):
            y = 0.39 - index * 0.09
            ax.text(0.1, y, key, fontsize=10, color=COLORS["muted"])
            ax.text(0.9, y, value, fontsize=10, color=COLORS["ink"], ha="right", fontweight="bold")
    fig.suptitle(
        "Verified simulator results",
        fontsize=22,
        fontweight="bold",
        color=COLORS["ink"],
        x=0.055,
        ha="left",
        y=0.98,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(FIGURE_DIR / "track_results.png", dpi=180)
    plt.close(fig)


def build_nordschleife_profile() -> None:
    path = PROJECT_ROOT / "logs" / "lmpc_nordschleife_v2_safe.csv"
    if not path.exists():
        return
    distance: list[float] = []
    speed: list[float] = []
    target: list[float] = []
    lateral: list[float] = []
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            if int(float(row.get("lap_number", 0))) != 1:
                continue
            distance.append(float(row["normalized_position"]) * 20655.8 / 1000.0)
            speed.append(float(row["speed_kmh"]))
            target.append(float(row["target_speed_kmh"]))
            lateral.append(abs(float(row["lateral_error_m"])))
    if not distance:
        return

    x = np.asarray(distance)
    y_speed = np.asarray(speed)
    y_target = np.asarray(target)
    y_lateral = np.asarray(lateral)
    bins = np.linspace(0.0, 20.6558, 420)
    centers = 0.5 * (bins[:-1] + bins[1:])
    speed_binned = np.full_like(centers, np.nan)
    target_binned = np.full_like(centers, np.nan)
    lateral_binned = np.full_like(centers, np.nan)
    for index in range(len(centers)):
        mask = (x >= bins[index]) & (x < bins[index + 1])
        if np.any(mask):
            speed_binned[index] = np.median(y_speed[mask])
            target_binned[index] = np.median(y_target[mask])
            lateral_binned[index] = np.percentile(y_lateral[mask], 95)

    fig, axes = plt.subplots(
        2,
        1,
        figsize=(16, 8.5),
        sharex=True,
        facecolor=COLORS["paper"],
        gridspec_kw={"height_ratios": [2.1, 1.0]},
    )
    axes[0].set_facecolor(COLORS["white"])
    axes[1].set_facecolor(COLORS["white"])
    axes[0].plot(centers, target_binned, color=COLORS["blue"], linewidth=1.7, label="target")
    axes[0].plot(centers, speed_binned, color=COLORS["red"], linewidth=1.6, label="actual")
    axes[0].fill_between(
        centers,
        speed_binned,
        target_binned,
        where=target_binned >= speed_binned,
        color="#F4B183",
        alpha=0.28,
        label="target deficit",
    )
    axes[0].set_ylabel("speed [km/h]")
    axes[0].legend(loc="upper right", ncol=3)
    axes[0].grid(color=COLORS["grid"], alpha=0.7)
    axes[0].set_title(
        "Nordschleife V2 safe: one complete lap, zero tyres out",
        loc="left",
        fontsize=18,
        fontweight="bold",
        color=COLORS["ink"],
    )
    axes[1].plot(centers, lateral_binned, color=COLORS["green"], linewidth=1.6)
    axes[1].axhline(1.2, color=COLORS["orange"], linestyle="--", linewidth=1.2, label="1.2 m guide")
    axes[1].set_ylabel("p95 |e_y| [m]")
    axes[1].set_xlabel("track distance [km]")
    axes[1].legend(loc="upper right")
    axes[1].grid(color=COLORS["grid"], alpha=0.7)
    fig.tight_layout()
    fig.savefig(FIGURE_DIR / "nordschleife_profile.png", dpi=180)
    plt.close(fig)


def main() -> int:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    apply_dark_theme()
    build_architecture()
    build_control_stack()
    build_track_results()
    build_nordschleife_profile()
    for path in sorted(FIGURE_DIR.glob("*.png")):
        print(path.relative_to(PROJECT_ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
