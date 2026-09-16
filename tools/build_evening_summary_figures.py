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
    metrics = (
        {
            "label": "Yaw error P95",
            "v6": 0.2672,
            "v7": 0.1896,
            "unit": "rad/s",
            "color": THEME["cyan"],
        },
        {
            "label": "Heading error P95",
            "v6": 0.1202,
            "v7": 0.1040,
            "unit": "rad",
            "color": THEME["blue"],
        },
        {
            "label": "Raw steer step mean",
            "v6": 0.002993,
            "v7": 0.001963,
            "unit": "normalized",
            "color": THEME["orange"],
        },
        {
            "label": "Sideslip RMS",
            "v6": 0.01280,
            "v7": 0.01211,
            "unit": "rad",
            "color": THEME["green"],
        },
    )

    fig, (ax, summary) = plt.subplots(
        1,
        2,
        figsize=(14, 6.6),
        gridspec_kw={"width_ratios": [3.0, 1.18], "wspace": 0.10},
        facecolor=THEME["navy"],
    )
    fig.patch.set_facecolor(THEME["navy"])
    ax.set_facecolor(THEME["navy"])
    summary.set_facecolor(THEME["navy"])
    ax.set_xlim(0, 1)
    ax.set_ylim(0.62, 1.02)
    ax.axis("off")

    fig.text(
        0.045,
        0.94,
        "Lateral calibration | Zhejiang V6 -> V7",
        ha="left",
        va="top",
        color=THEME["ink"],
        fontsize=20,
        fontweight="bold",
    )
    fig.text(
        0.045,
        0.885,
        "Every response is normalized to the V6 run; lower is better.",
        ha="left",
        va="top",
        color=THEME["muted"],
        fontsize=10.5,
    )
    ax.axhline(
        1.0,
        color=THEME["grid"],
        linestyle=(0, (5, 5)),
        linewidth=1.0,
    )
    ax.text(
        0.74,
        1.012,
        "V6 = 100%",
        ha="right",
        va="bottom",
        color=THEME["muted"],
        fontsize=9.5,
    )

    x_v6 = 0.43
    x_v7 = 0.61
    ax.text(
        x_v6,
        0.675,
        "V6 BASELINE",
        ha="center",
        color=THEME["muted"],
        fontsize=9.5,
        fontweight="bold",
    )
    ax.text(
        x_v7,
        0.675,
        "V7 REFINED",
        ha="center",
        color=THEME["orange"],
        fontsize=9.5,
        fontweight="bold",
    )

    for item in metrics:
        ratio = item["v7"] / item["v6"]
        improvement = (1.0 - ratio) * 100.0
        ax.plot(
            [x_v6, x_v7],
            [1.0, ratio],
            color=item["color"],
            linewidth=3.0,
            alpha=0.78,
            solid_capstyle="round",
            zorder=2,
        )
        ax.scatter(
            [x_v7],
            [ratio],
            s=76,
            facecolor=item["color"],
            edgecolor=THEME["navy"],
            linewidth=1.8,
            zorder=4,
        )
        ax.text(
            x_v7 + 0.025,
            ratio,
            f"{improvement:.1f}% lower",
            ha="left",
            va="center",
            color=item["color"],
            fontsize=11,
            fontweight="bold",
        )
        ax.text(
            0.02,
            ratio,
            item["label"],
            ha="left",
            va="center",
            color=THEME["ink"],
            fontsize=11,
            fontweight="bold",
        )
        if item["unit"] == "normalized":
            value_label = f"{item['v7']:.6f}"
        else:
            value_label = f"{item['v7']:.4f} {item['unit']}"
        ax.text(
            0.98,
            ratio,
            value_label,
            ha="right",
            va="center",
            color=THEME["muted"],
            fontsize=9.5,
        )

    summary.set_xlim(0, 1)
    summary.set_ylim(0, 1)
    summary.axis("off")
    summary.add_patch(
        FancyBboxPatch(
            (0.04, 0.09),
            0.92,
            0.80,
            boxstyle="round,pad=0.025,rounding_size=0.035",
            linewidth=1.4,
            edgecolor=THEME["border"],
            facecolor=THEME["white"],
        )
    )
    summary.text(
        0.12,
        0.81,
        "CONTROL RESPONSE",
        color=THEME["cyan"],
        fontsize=10.5,
        fontweight="bold",
    )
    summary.text(
        0.12,
        0.69,
        "Less correction",
        color=THEME["ink"],
        fontsize=18,
        fontweight="bold",
    )
    summary.text(
        0.12,
        0.60,
        "for the same path",
        color=THEME["muted"],
        fontsize=12,
    )
    summary.plot(
        [0.12, 0.88],
        [0.53, 0.53],
        color=THEME["grid"],
        linewidth=1.2,
    )
    highlights = (
        ("-29.0%", "yaw error P95"),
        ("-34.4%", "raw steer activity"),
        ("-5.4%", "sideslip RMS"),
    )
    for index, (value, label) in enumerate(highlights):
        y = 0.43 - index * 0.11
        summary.text(
            0.12,
            y,
            value,
            color=THEME["orange"] if index == 0 else THEME["ink"],
            fontsize=15,
            fontweight="bold",
        )
        summary.text(
            0.44,
            y,
            label,
            color=THEME["muted"],
            fontsize=10.5,
            va="center",
        )

    fig.subplots_adjust(left=0.055, right=0.985, top=0.82, bottom=0.08)
    fig.savefig(
        FIGURE_DIR / "lateral_calibration_zhejiang.png",
        dpi=180,
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
    build_tuning_pipeline()
    print("Wrote evening summary figures")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
