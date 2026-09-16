from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from tools.figure_theme import THEME, apply_dark_theme


PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIGURE_DIR = PROJECT_ROOT / "assets" / "figures"


def _style() -> None:
    apply_dark_theme()
    plt.rcParams["axes.titleweight"] = "bold"


def build_lateral_calibration() -> None:
    labels = ["Yaw err\nP95", "Heading err\nP95", "Raw steer\nstep mean", "Sideslip\nRMS"]
    v6 = np.array([0.2672, 0.1202, 0.002993, 0.01280])
    v7 = np.array([0.1896, 0.1040, 0.001963, 0.01211])
    v6_norm = v6 / v6
    v7_norm = v7 / v6

    x = np.arange(len(labels))
    width = 0.34
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.patch.set_facecolor(THEME["navy"])
    ax.set_facecolor(THEME["navy"])
    ax.bar(
        x - width / 2,
        v6_norm,
        width,
        label="V6",
        color=THEME["blue"],
    )
    ax.bar(
        x + width / 2,
        v7_norm,
        width,
        label="V7 refined",
        color=THEME["orange"],
    )
    for index, ratio in enumerate(v7_norm):
        ax.text(
            index + width / 2,
            ratio + 0.025,
            f"{ratio * 100:.1f}%",
            ha="center",
            color="#F4F7FB",
            fontsize=10,
        )
    ax.axhline(1.0, color=THEME["grid"], linestyle="--", linewidth=1)
    ax.set_xticks(x, labels)
    ax.set_ylabel("Normalized to V6")
    ax.set_title(
        "Zhejiang lateral calibration: V7 keeps tracking while reducing yaw and steering activity"
    )
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(
        FIGURE_DIR / "lateral_calibration_zhejiang.png",
        dpi=170,
        facecolor=fig.get_facecolor(),
    )
    plt.close(fig)


def build_braking_upgrade() -> None:
    labels = ["Max brake", "P95 brake", "Peak decel"]
    before = np.array([0.58, 0.474, 9.9])
    zhejiang = np.array([0.983, 0.745, 12.64])
    shanghai = np.array([1.0, 0.791, 13.03])
    nordschleife = np.array([0.907, 0.513, 9.72])

    x = np.arange(len(labels))
    width = 0.19
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.patch.set_facecolor(THEME["navy"])
    ax.set_facecolor(THEME["navy"])
    for offset, values, label, color in (
        (-1.5, before, "Before", THEME["muted"]),
        (-0.5, zhejiang, "Zhejiang local055", THEME["orange"]),
        (0.5, shanghai, "Shanghai line60", THEME["blue"]),
        (1.5, nordschleife, "Nordschleife sync", THEME["green"]),
    ):
        plotted = values / before
        ax.bar(
            x + offset * width,
            plotted,
            width,
            label=label,
            color=color,
        )
    ax.axhline(1.0, color=THEME["grid"], linestyle="--", linewidth=1)
    ax.set_xticks(x, labels)
    ax.set_ylabel("Normalized to pre-fix run")
    ax.set_title(
        "Longitudinal braking upgrade: target, preview and pedal are now connected"
    )
    ax.legend(frameon=False, ncol=2)
    fig.tight_layout()
    fig.savefig(
        FIGURE_DIR / "longitudinal_braking_upgrade.png",
        dpi=170,
        facecolor=fig.get_facecolor(),
    )
    plt.close(fig)


def build_latest_track_results() -> None:
    labels = ["Shanghai\nline60", "Zhejiang\nlocal055", "Nordschleife\ncorrected"]
    lap_times = np.array([134.399, 96.245, 472.546])
    tyres = np.array([1, 2, 2])

    fig, ax = plt.subplots(figsize=(11, 5.6))
    fig.patch.set_facecolor(THEME["navy"])
    ax.set_facecolor(THEME["navy"])
    bars = ax.bar(
        labels,
        lap_times,
        color=[THEME["blue"], THEME["orange"], THEME["green"]],
        width=0.58,
    )
    for bar, seconds, out in zip(bars, lap_times, tyres):
        minutes = int(seconds // 60)
        remainder = seconds - 60 * minutes
        label = (
            f"{minutes}:{remainder:06.3f}"
            if minutes
            else f"{remainder:.3f} s"
        )
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            seconds + 8,
            f"{label}\ntyres out {out}",
            ha="center",
            va="bottom",
            color="#F4F7FB",
            fontsize=11,
        )
    ax.set_ylabel("Lap time (s)")
    ax.set_title("Latest validated configurations")
    ax.set_ylim(0, 540)
    fig.tight_layout()
    fig.savefig(
        FIGURE_DIR / "latest_track_results.png",
        dpi=170,
        facecolor=fig.get_facecolor(),
    )
    plt.close(fig)


def build_tuning_pipeline() -> None:
    fig, ax = plt.subplots(figsize=(15, 7))
    fig.patch.set_facecolor(THEME["navy"])
    ax.set_facecolor(THEME["navy"])
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 7)
    ax.axis("off")
    stages = (
        ("20-lap scan", "Fixed space-filling\nlateral-weight scan", THEME["blue"]),
        ("11-lap refine", "A-B-B-A confirmation\n103 selected", THEME["orange"]),
        ("Local correction", "Narrow speed envelope\nand line offset", THEME["green"]),
        ("Brake pipeline", "Reachable planning brake\n+ 52 m/s² command", "#C44B3D"),
    )
    xs = (0.6, 4.2, 7.8, 11.4)
    for index, ((title, body, color), x) in enumerate(zip(stages, xs)):
        ax.add_patch(
            FancyBboxPatch(
                (x, 1.5),
                2.7,
                4.0,
                boxstyle="round,pad=0.12,rounding_size=0.18",
                linewidth=2,
                edgecolor=color,
                facecolor="#0B1A28",
            )
        )
        ax.text(
            x + 1.35,
            4.75,
            title,
            ha="center",
            va="center",
            color=color,
            fontsize=15,
            fontweight="bold",
        )
        ax.text(
            x + 1.35,
            3.15,
            body,
            ha="center",
            va="center",
            color="#F4F7FB",
            fontsize=13,
            linespacing=1.5,
        )
        if index < len(stages) - 1:
            ax.add_patch(
                FancyArrowPatch(
                    (x + 2.75, 3.5),
                    (xs[index + 1] - 0.05, 3.5),
                    arrowstyle="-|>",
                    mutation_scale=20,
                    linewidth=2.5,
                    color=THEME["grid"],
                )
            )
    ax.text(
        7.5,
        6.3,
        "Lateral calibration to full-braking closed loop",
        ha="center",
        color="#F4F7FB",
        fontsize=20,
        fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(
        FIGURE_DIR / "tuning_pipeline_20260916.png",
        dpi=170,
        facecolor=fig.get_facecolor(),
    )
    plt.close(fig)


def main() -> int:
    _style()
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    build_lateral_calibration()
    build_braking_upgrade()
    build_latest_track_results()
    build_tuning_pipeline()
    print("Wrote evening summary figures")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
