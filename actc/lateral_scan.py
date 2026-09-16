from __future__ import annotations

import math
from dataclasses import asdict, dataclass

from .mpc import MpcConfig


SCAN_METRIC_WEIGHTS = {
    "lateral": 0.30,
    "heading": 0.20,
    "shake": 0.20,
    "yaw_rate": 0.20,
    "sideslip": 0.10,
}

# Zhejiang V7 historical log values. They are used only to normalize
# scan metrics so candidates can be ranked without a dedicated baseline lap.
SCAN_BASELINE = {
    "lateral_error_rms_m": 0.5459,
    "heading_error_p95_rad": 0.1040,
    "yaw_rate_error_p95_rad_s": 0.1896,
    "shake_score": 0.0321576,
    "sideslip_score_rad": 0.018726,
}

REFINE_EXPLORATION = (
    (12.4, 110.0, 24.4),
    (15.0, 110.0, 28.0),
    (11.5, 135.0, 28.0),
    (15.0, 135.0, 24.0),
    (13.25, 122.5, 26.0),
    (14.0, 127.5, 30.0),
)

REFINE_FIXED = {
    "q_lateral_velocity": 3.5,
    "r_steer": 15.0,
    "rd_steer": 700.0,
}


@dataclass(frozen=True)
class LateralScanCandidate:
    design_index: int
    q_lateral: float
    q_heading: float
    q_yaw_rate: float
    q_lateral_velocity: float
    r_steer: float
    rd_steer: float

    def apply(self, config: MpcConfig) -> None:
        config.q_lateral = self.q_lateral
        config.q_heading = self.q_heading
        config.q_yaw_rate = self.q_yaw_rate
        config.q_lateral_velocity = self.q_lateral_velocity
        config.r_steer = self.r_steer
        config.rd_steer = self.rd_steer

    def as_dict(self) -> dict[str, float | int]:
        return asdict(self)


@dataclass(frozen=True)
class LapScanMetrics:
    lateral_error_rms_m: float
    heading_error_p95_rad: float
    yaw_rate_error_rms_rad_s: float
    yaw_rate_error_p95_rad_s: float
    sideslip_rms_rad: float
    sideslip_p95_rad: float
    raw_steer_step_p95: float
    raw_steer_jerk_rms: float
    shake_score: float
    sideslip_score_rad: float
    composite_score: float

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


def build_lateral_scan_schedule() -> tuple[LateralScanCandidate, ...]:
    """Return the fixed 20-run Zhejiang lateral-weight scan."""
    raw_candidates = (
        (16.864, 71.002, 27.815, 4.985, 22.064, 640.935),
        (14.595, 122.518, 26.416, 7.157, 16.928, 524.053),
        (18.468, 95.972, 14.530, 3.432, 16.071, 782.727),
        (12.365, 109.605, 24.359, 3.285, 13.798, 622.926),
        (11.982, 77.265, 17.348, 5.416, 17.814, 948.091),
        (15.475, 93.558, 17.018, 5.222, 20.383, 974.545),
        (11.035, 104.578, 16.601, 4.370, 12.082, 933.676),
        (19.542, 113.123, 18.454, 6.764, 24.900, 684.721),
        (10.489, 88.888, 21.035, 6.518, 14.272, 561.021),
        (11.503, 84.071, 22.755, 7.661, 23.810, 585.847),
        (21.943, 134.869, 25.466, 3.063, 13.176, 717.743),
        (13.631, 99.031, 15.080, 5.793, 15.451, 540.001),
        (19.652, 137.737, 18.823, 4.547, 14.957, 800.093),
        (14.174, 92.034, 14.164, 4.786, 12.701, 608.008),
        (16.143, 79.892, 15.647, 6.201, 22.479, 744.290),
        (13.076, 127.849, 24.765, 3.815, 21.370, 767.000),
        (15.401, 114.279, 20.046, 7.355, 25.271, 850.815),
        (10.359, 80.599, 19.615, 3.884, 18.592, 906.420),
        (17.790, 121.749, 22.443, 4.053, 17.396, 845.473),
        (20.415, 72.568, 21.612, 3.503, 19.260, 669.274),
    )
    return tuple(
        LateralScanCandidate(
            design_index=index,
            q_lateral=q_lateral,
            q_heading=q_heading,
            q_yaw_rate=q_yaw_rate,
            q_lateral_velocity=q_lateral_velocity,
            r_steer=r_steer,
            rd_steer=rd_steer,
        )
        for index, (
            q_lateral,
            q_heading,
            q_yaw_rate,
            q_lateral_velocity,
            r_steer,
            rd_steer,
        ) in enumerate(raw_candidates, start=1)
    )


def build_lateral_refine_schedule() -> list[LateralScanCandidate | None]:
    """One warm-up, six exploration laps, and four confirmation laps."""
    candidates: list[LateralScanCandidate | None] = [
        LateralScanCandidate(
            design_index=100,
            q_lateral=15.0,
            q_heading=100.0,
            q_yaw_rate=20.0,
            q_lateral_velocity=5.0,
            r_steer=18.0,
            rd_steer=676.0,
        )
    ]
    candidates.extend(
        [
            LateralScanCandidate(
                design_index=100 + index,
                q_lateral=q_lateral,
                q_heading=q_heading,
                q_yaw_rate=q_yaw_rate,
                **REFINE_FIXED,
            )
            for index, (
                q_lateral,
                q_heading,
                q_yaw_rate,
            ) in enumerate(REFINE_EXPLORATION, start=1)
        ]
    )
    candidates.extend([None] * 4)
    return candidates


def select_refinement_confirmations(
    metrics_by_design: dict[int, LapScanMetrics],
) -> list[LateralScanCandidate]:
    if len(metrics_by_design) < 2:
        raise ValueError("at least two exploration candidates are required")
    ranked = sorted(
        metrics_by_design.items(),
        key=lambda item: item[1].composite_score,
    )
    top = [
        _candidate_from_exploration_index(design_index)
        for design_index, _metrics in ranked[:2]
    ]
    first, second = top
    return [first, second, second, first]


def _candidate_from_exploration_index(
    design_index: int,
) -> LateralScanCandidate:
    index = design_index - 101
    if not 0 <= index < len(REFINE_EXPLORATION):
        raise ValueError(f"unknown exploration candidate {design_index}")
    q_lateral, q_heading, q_yaw_rate = REFINE_EXPLORATION[index]
    return LateralScanCandidate(
        design_index=design_index,
        q_lateral=q_lateral,
        q_heading=q_heading,
        q_yaw_rate=q_yaw_rate,
        **REFINE_FIXED,
    )


def compute_lap_scan_metrics(
    *,
    lateral_errors_m: list[float],
    heading_errors_rad: list[float],
    yaw_rate_errors_rad_s: list[float],
    sideslip_abs_rad: list[float],
    raw_steer_commands: list[float],
    dt: float,
) -> LapScanMetrics:
    safe_dt = max(1e-3, float(dt))
    lateral_rms = _rms(lateral_errors_m)
    heading_p95 = _percentile_abs(heading_errors_rad, 0.95)
    yaw_rms = _rms(yaw_rate_errors_rad_s)
    yaw_p95 = _percentile_abs(yaw_rate_errors_rad_s, 0.95)
    sideslip_rms = _rms(sideslip_abs_rad)
    sideslip_p95 = _percentile(sideslip_abs_rad, 0.95)

    raw_steps = [
        raw_steer_commands[index] - raw_steer_commands[index - 1]
        for index in range(1, len(raw_steer_commands))
    ]
    raw_step_p95 = _percentile_abs(raw_steps, 0.95)
    raw_jerks = [
        (raw_steps[index] - raw_steps[index - 1]) / safe_dt
        for index in range(1, len(raw_steps))
    ]
    raw_jerk_rms = _rms(raw_jerks)
    shake_score = 0.7 * raw_step_p95 + 0.3 * raw_jerk_rms
    sideslip_score = 0.6 * sideslip_rms + 0.4 * sideslip_p95

    composite = (
        SCAN_METRIC_WEIGHTS["lateral"]
        * _normalized(lateral_rms, "lateral_error_rms_m")
        + SCAN_METRIC_WEIGHTS["heading"]
        * _normalized(heading_p95, "heading_error_p95_rad")
        + SCAN_METRIC_WEIGHTS["shake"]
        * _normalized(shake_score, "shake_score")
        + SCAN_METRIC_WEIGHTS["yaw_rate"]
        * _normalized(yaw_p95, "yaw_rate_error_p95_rad_s")
        + SCAN_METRIC_WEIGHTS["sideslip"]
        * _normalized(sideslip_score, "sideslip_score_rad")
    )
    return LapScanMetrics(
        lateral_error_rms_m=lateral_rms,
        heading_error_p95_rad=heading_p95,
        yaw_rate_error_rms_rad_s=yaw_rms,
        yaw_rate_error_p95_rad_s=yaw_p95,
        sideslip_rms_rad=sideslip_rms,
        sideslip_p95_rad=sideslip_p95,
        raw_steer_step_p95=raw_step_p95,
        raw_steer_jerk_rms=raw_jerk_rms,
        shake_score=shake_score,
        sideslip_score_rad=sideslip_score,
        composite_score=composite,
    )


def _normalized(value: float, baseline_key: str) -> float:
    baseline = max(1e-9, float(SCAN_BASELINE[baseline_key]))
    return max(0.0, float(value)) / baseline


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(
        sum(float(value) * float(value) for value in values)
        / len(values)
    )


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    fraction = max(0.0, min(1.0, fraction))
    index = int(round((len(ordered) - 1) * fraction))
    return ordered[index]


def _percentile_abs(values: list[float], fraction: float) -> float:
    return _percentile([abs(float(value)) for value in values], fraction)
