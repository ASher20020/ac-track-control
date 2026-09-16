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

LIGHT_THEME = {
    "ink": "#0F172A",
    "muted": "#64748B",
    "grid": "#D7E0EA",
    "paper": "#F4F7FB",
    "white": "#FFFFFF",
    "panel": "#E8EEF5",
    "border": "#C7D3E0",
    "blue": "#2563EB",
    "cyan": "#0891B2",
    "green": "#059669",
    "orange": "#D97706",
    "red": "#DC5266",
    "navy": "#F4F7FB",
    "navy_2": "#FFFFFF",
    "cream": "#0F172A",
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


def apply_light_theme() -> None:
    plt.rcParams.update(
        {
            "font.sans-serif": [
                "Microsoft YaHei",
                "Bahnschrift",
                "Segoe UI",
                "DejaVu Sans",
            ],
            "axes.unicode_minus": False,
            "figure.facecolor": LIGHT_THEME["paper"],
            "savefig.facecolor": LIGHT_THEME["paper"],
            "axes.facecolor": LIGHT_THEME["white"],
            "axes.edgecolor": LIGHT_THEME["border"],
            "axes.labelcolor": LIGHT_THEME["ink"],
            "axes.titlecolor": LIGHT_THEME["ink"],
            "text.color": LIGHT_THEME["ink"],
            "xtick.color": LIGHT_THEME["muted"],
            "ytick.color": LIGHT_THEME["muted"],
            "grid.color": LIGHT_THEME["grid"],
            "legend.edgecolor": LIGHT_THEME["border"],
            "legend.facecolor": LIGHT_THEME["white"],
            "legend.labelcolor": LIGHT_THEME["ink"],
            "axes.prop_cycle": plt.cycler(
                color=[
                    LIGHT_THEME["blue"],
                    LIGHT_THEME["orange"],
                    LIGHT_THEME["cyan"],
                    LIGHT_THEME["green"],
                    LIGHT_THEME["red"],
                ]
            ),
        }
    )
