from __future__ import annotations

import matplotlib.pyplot as plt


THEME = {
    "ink": "#F5F7FA",
    "muted": "#94A3B8",
    "grid": "#22364A",
    "paper": "#07111B",
    "white": "#0E1B28",
    "panel": "#132537",
    "border": "#20384D",
    "blue": "#3B82F6",
    "cyan": "#22D3EE",
    "green": "#34D399",
    "orange": "#F59E0B",
    "red": "#FB7185",
    "navy": "#07111B",
    "navy_2": "#0E1B28",
    "cream": "#F5F7FA",
}


def apply_dark_theme() -> None:
    plt.rcParams.update(
        {
            "font.sans-serif": [
                "Microsoft YaHei",
                "Bahnschrift",
                "Segoe UI",
                "DejaVu Sans",
            ],
            "axes.unicode_minus": False,
            "figure.facecolor": THEME["paper"],
            "savefig.facecolor": THEME["paper"],
            "axes.facecolor": THEME["white"],
            "axes.edgecolor": THEME["border"],
            "axes.labelcolor": THEME["ink"],
            "axes.titlecolor": THEME["ink"],
            "text.color": THEME["ink"],
            "xtick.color": THEME["muted"],
            "ytick.color": THEME["muted"],
            "grid.color": THEME["grid"],
            "legend.edgecolor": THEME["border"],
            "legend.facecolor": THEME["white"],
            "legend.labelcolor": THEME["ink"],
            "axes.prop_cycle": plt.cycler(
                color=[
                    THEME["blue"],
                    THEME["orange"],
                    THEME["cyan"],
                    THEME["green"],
                    THEME["red"],
                ]
            ),
        }
    )
