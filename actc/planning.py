from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize

from .track import TrackPath, clamp


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = clamp(fraction, 0.0, 1.0) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _smooth(values: list[float], radius: int) -> list[float]:
    if radius <= 0:
        return list(values)
    count = len(values)
    return [
        sum(
            values[(index + offset) % count]
            for offset in range(-radius, radius + 1)
        )
        / (2 * radius + 1)
        for index in range(count)
    ]


@dataclass(frozen=True)
class PlanningSample:
    path_index: int
    lateral_error_m: float
    heading_error_rad: float
    lateral_g: float
    longitudinal_g: float
    tyres_out: int
    steer_delta: float
    speed_error_kmh: float
    edge_clearance_m: float


@dataclass(frozen=True)
class CornerWindow:
    indices: tuple[int, ...]
    center_s_m: float
    length_m: float


@dataclass(frozen=True)
class PlanningUpdate:
    planning_lap_number: int
    estimated_lap_before_s: float
    estimated_lap_after_s: float
    accepted: bool
    stable_corners: int
    caution_corners: int
    unsafe_corners: int
    max_p95_lateral_error_m: float
    max_p95_heading_error_rad: float
    max_tyres_out: int
    min_edge_clearance_m: float
    blend_by_corner: tuple[float, ...]


@dataclass
class PlanningConfig:
    initial_blend: float = 0.50
    minimum_blend: float = 0.15
    maximum_blend: float = 0.90
    stable_blend_step: float = 0.04
    caution_blend_step: float = 0.02
    unsafe_blend_retreat: float = 0.10
    corner_curvature_threshold: float = 0.006
    corner_merge_gap_m: float = 80.0
    corner_min_samples: int = 20
    stable_p95_lateral_error_m: float = 0.18
    stable_p95_heading_error_rad: float = 0.06
    stable_steering_activity: float = 0.0012
    caution_p95_lateral_error_m: float = 0.40
    caution_p95_heading_error_rad: float = 0.12
    caution_steering_activity: float = 0.0020
    minimum_edge_clearance_m: float = 0.75
    geometry_target_clearance_m: float = 1.00
    geometry_hard_clearance_m: float = 0.80
    clearance_penalty_weight: float = 4000.0
    curvature_penalty_weight: float = 22000.0
    offset_smoothness_weight: float = 0.008
    estimated_lap_reject_margin_s: float = 0.25
    optimize_initial_line: bool = True
    optimization_control_nodes: int = 128
    optimization_max_iterations: int = 60
    optimization_max_function_evaluations: int = 1600
    optimization_step_m: float = 0.12
    max_speed_kmh: float = 260.0
    min_speed_kmh: float = 25.0
    lateral_accel_mps2: float = 3.924
    accel_mps2: float = 3.924
    brake_mps2: float = 8.34

    def line_kwargs(self) -> dict[str, float]:
        return {
            "base_margin_m": self.geometry_hard_clearance_m,
            "speed_margin_m_per_kmh": 0.0,
            "max_edge_margin_m": self.geometry_hard_clearance_m,
            "max_speed_kmh": self.max_speed_kmh,
            "min_speed_kmh": self.min_speed_kmh,
            "lateral_accel_mps2": self.lateral_accel_mps2,
            "accel_mps2": self.accel_mps2,
            "brake_mps2": self.brake_mps2,
        }


class PlanningLineTuner:
    """Slow per-lap racing-line updater with conservative safety gates."""

    def __init__(
        self,
        reference_path: TrackPath,
        config: PlanningConfig | None = None,
    ) -> None:
        if (
            reference_path.reference_points is None
            or reference_path.left_width_m is None
            or reference_path.right_width_m is None
            or reference_path.fast_lane_offset_m is None
        ):
            raise ValueError(
                "Racing-line planning needs centerline and track widths"
            )
        self.reference_path = reference_path
        self.config = config or PlanningConfig()
        self.corners = _identify_corners(
            reference_path,
            curvature_threshold=(
                self.config.corner_curvature_threshold
            ),
            merge_gap_m=self.config.corner_merge_gap_m,
        )
        self.corner_index_by_point = [-1] * len(reference_path.points)
        for corner_index, corner in enumerate(self.corners):
            for path_index in corner.indices:
                self.corner_index_by_point[path_index] = corner_index
        self.blend_by_corner = [
            clamp(
                self.config.initial_blend,
                self.config.minimum_blend,
                self.config.maximum_blend,
            )
            for _ in self.corners
        ]
        self.planning_lap_number = 0
        self.target_offset_m = (
            _optimize_initial_offsets(
                reference_path,
                self.config,
            )
            if self.config.optimize_initial_line
            else [0.0] * len(reference_path.points)
        )
        self.current_path = self._build_path()

    def _build_path(self) -> TrackPath:
        factors = [self.config.initial_blend] * len(
            self.reference_path.points
        )
        for corner_index, corner in enumerate(self.corners):
            factor = self.blend_by_corner[corner_index]
            for path_index in corner.indices:
                factors[path_index] = factor
        smoothed = _smooth(factors, radius=16)
        return self.reference_path.with_line_offsets(
            [
                self.target_offset_m[index] * smoothed[index]
                for index in range(len(self.reference_path.points))
            ],
            **self.config.line_kwargs(),
        )

    def update(
        self,
        samples: list[PlanningSample],
        *,
        lap_time_s: float,
    ) -> PlanningUpdate:
        self.planning_lap_number += 1
        estimated_before = self.current_path.estimated_lap_s
        stable_corners = 0
        caution_corners = 0
        unsafe_corners = 0
        max_p95_lateral = 0.0
        max_p95_heading = 0.0
        max_tyres_out = 0
        min_clearance = float("inf")
        previous_blend = list(self.blend_by_corner)

        samples_by_corner: list[list[PlanningSample]] = [
            [] for _ in self.corners
        ]
        for sample in samples:
            index = sample.path_index % len(self.corner_index_by_point)
            corner_index = self.corner_index_by_point[index]
            if corner_index >= 0:
                samples_by_corner[corner_index].append(sample)
            min_clearance = min(
                min_clearance,
                sample.edge_clearance_m,
            )
            max_tyres_out = max(max_tyres_out, sample.tyres_out)

        for corner_index, corner_samples in enumerate(
            samples_by_corner
        ):
            if len(corner_samples) < self.config.corner_min_samples:
                continue
            lateral_errors = [
                abs(sample.lateral_error_m)
                for sample in corner_samples
            ]
            heading_errors = [
                abs(sample.heading_error_rad)
                for sample in corner_samples
            ]
            steering_activity = sum(
                abs(sample.steer_delta)
                for sample in corner_samples
            ) / len(corner_samples)
            p95_lateral = _percentile(lateral_errors, 0.95)
            p95_heading = _percentile(heading_errors, 0.95)
            corner_tyres_out = max(
                sample.tyres_out
                for sample in corner_samples
            )
            corner_clearance = min(
                sample.edge_clearance_m
                for sample in corner_samples
            )
            max_p95_lateral = max(max_p95_lateral, p95_lateral)
            max_p95_heading = max(max_p95_heading, p95_heading)

            unsafe = (
                corner_tyres_out >= 3
                or p95_lateral > self.config.caution_p95_lateral_error_m
                or p95_heading > self.config.caution_p95_heading_error_rad
                or steering_activity > self.config.caution_steering_activity
                or corner_clearance < self.config.minimum_edge_clearance_m
            )
            stable = (
                not unsafe
                and corner_tyres_out <= 0
                and p95_lateral
                <= self.config.stable_p95_lateral_error_m
                and p95_heading
                <= self.config.stable_p95_heading_error_rad
                and steering_activity
                <= self.config.stable_steering_activity
            )
            if unsafe:
                unsafe_corners += 1
                delta = -self.config.unsafe_blend_retreat
            elif stable:
                stable_corners += 1
                delta = self.config.stable_blend_step
            else:
                caution_corners += 1
                delta = -self.config.caution_blend_step

            self.blend_by_corner[corner_index] = clamp(
                self.blend_by_corner[corner_index] + delta,
                self.config.minimum_blend,
                self.config.maximum_blend,
            )

        candidate = self._build_path()
        estimated_after = candidate.estimated_lap_s
        accepted = (
            estimated_after
            <= estimated_before
            + self.config.estimated_lap_reject_margin_s
        )
        if not accepted:
            # Rejecting the geometry change is safer than rolling it back
            # after it has already been executed. The next lap will try a
            # smaller change from the same blend values.
            self.blend_by_corner = previous_blend
            candidate = self.current_path
            estimated_after = estimated_before
        else:
            self.current_path = candidate

        return PlanningUpdate(
            planning_lap_number=self.planning_lap_number,
            estimated_lap_before_s=estimated_before,
            estimated_lap_after_s=estimated_after,
            accepted=accepted,
            stable_corners=stable_corners,
            caution_corners=caution_corners,
            unsafe_corners=unsafe_corners,
            max_p95_lateral_error_m=max_p95_lateral,
            max_p95_heading_error_rad=max_p95_heading,
            max_tyres_out=max_tyres_out,
            min_edge_clearance_m=(
                min_clearance
                if math.isfinite(min_clearance)
                else 0.0
            ),
            blend_by_corner=tuple(self.blend_by_corner),
        )


def _optimize_initial_offsets(
    path: TrackPath,
    config: PlanningConfig,
) -> list[float]:
    count = len(path.points)
    node_count = min(
        max(8, config.optimization_control_nodes),
        count,
    )
    node_s = np.linspace(
        0.0,
        path.total_length_m,
        node_count,
        endpoint=False,
    )
    bounds = _control_node_bounds(path, node_s, config)
    target_s = np.asarray(path.cumulative_s)

    centerline = path.with_line_offsets(
        [0.0] * count,
        **config.line_kwargs(),
    )
    centerline_time = centerline.estimated_lap_s

    def objective(values: np.ndarray) -> float:
        offsets = _periodic_offsets(
            target_s,
            node_s,
            values,
            total_length_m=path.total_length_m,
        )
        candidate = path.with_line_offsets(
            offsets.tolist(),
            **config.line_kwargs(),
        )
        left_clearance = (
            np.asarray(path.left_width_m, dtype=float) - offsets
        )
        right_clearance = (
            np.asarray(path.right_width_m, dtype=float) + offsets
        )
        left_target = np.minimum(
            config.geometry_target_clearance_m,
            np.asarray(path.left_width_m, dtype=float),
        )
        right_target = np.minimum(
            config.geometry_target_clearance_m,
            np.asarray(path.right_width_m, dtype=float),
        )
        clearance_violation = np.maximum(
            0.0,
            left_target - left_clearance,
        ) + np.maximum(
            0.0,
            right_target - right_clearance,
        )
        curvature = np.asarray(candidate.curvature, dtype=float)
        segment_length = np.diff(
            np.concatenate(
                [
                    np.asarray(candidate.cumulative_s, dtype=float),
                    [candidate.total_length_m],
                ]
            )
        )
        curvature_smoothness = float(
            np.sum(np.diff(values, n=2, append=values[:2]) ** 2)
        )
        return (
            candidate.estimated_lap_s
            + config.curvature_penalty_weight
            * float(np.sum(curvature**2 * segment_length))
            + config.clearance_penalty_weight
            * float(np.sum(clearance_violation**2 * segment_length))
            + config.offset_smoothness_weight * curvature_smoothness
        )

    try:
        result = minimize(
            objective,
            np.zeros(node_count),
            method="L-BFGS-B",
            bounds=bounds,
            options={
                "maxiter": config.optimization_max_iterations,
                "maxfun": (
                    config.optimization_max_function_evaluations
                ),
                "ftol": 1e-3,
                "eps": config.optimization_step_m,
                "disp": False,
            },
        )
    except (FloatingPointError, ValueError):
        return [0.0] * count

    if not np.all(np.isfinite(result.x)):
        return [0.0] * count
    optimized_offsets = _periodic_offsets(
        target_s,
        node_s,
        result.x,
        total_length_m=path.total_length_m,
    )
    optimized = path.with_line_offsets(
        optimized_offsets.tolist(),
        **config.line_kwargs(),
    )
    if optimized.estimated_lap_s >= centerline_time - 0.05:
        return [0.0] * count
    return optimized_offsets.tolist()


def _periodic_offsets(
    target_s: np.ndarray,
    node_s: np.ndarray,
    values: np.ndarray,
    *,
    total_length_m: float,
) -> np.ndarray:
    closed_s = np.concatenate([node_s, [total_length_m]])
    closed_values = np.concatenate([values, values[:1]])
    spline = CubicSpline(
        closed_s,
        closed_values,
        bc_type="periodic",
        extrapolate="periodic",
    )
    return spline(target_s % total_length_m)


def _control_node_bounds(
    path: TrackPath,
    node_s: np.ndarray,
    config: PlanningConfig,
) -> list[tuple[float, float]]:
    if (
        path.left_width_m is None
        or path.right_width_m is None
    ):
        raise ValueError("TrackPath is missing track widths")
    count = len(path.points)
    half_window = 0.5 * path.total_length_m / len(node_s)
    bounds: list[tuple[float, float]] = []
    for distance in node_s:
        lower_limit = -float("inf")
        upper_limit = float("inf")
        for path_index in range(count):
            delta = (
                path.cumulative_s[path_index] - float(distance)
            ) % path.total_length_m
            if delta > half_window and delta < path.total_length_m - half_window:
                continue
            left_width = path.left_width_m[path_index]
            right_width = path.right_width_m[path_index]
            left_hard_clearance = min(
                config.geometry_hard_clearance_m,
                left_width,
            )
            right_hard_clearance = min(
                config.geometry_hard_clearance_m,
                right_width,
            )
            lower_limit = max(
                lower_limit,
                -(right_width - right_hard_clearance),
            )
            upper_limit = min(
                upper_limit,
                left_width - left_hard_clearance,
            )
        if lower_limit > upper_limit:
            midpoint = 0.5 * (lower_limit + upper_limit)
            lower_limit = midpoint
            upper_limit = midpoint
        bounds.append((lower_limit, upper_limit))
    return bounds


def _identify_corners(
    path: TrackPath,
    *,
    curvature_threshold: float,
    merge_gap_m: float,
) -> list[CornerWindow]:
    count = len(path.points)
    mask = [
        abs(value) >= curvature_threshold
        for value in path.curvature
    ]
    used = [False] * count
    raw_groups: list[list[int]] = []
    for start in range(count):
        if not mask[start] or used[start]:
            continue
        indices: list[int] = []
        index = start
        while mask[index] and not used[index]:
            used[index] = True
            indices.append(index)
            index = (index + 1) % count
        raw_groups.append(indices)

    if len(raw_groups) <= 1:
        groups = raw_groups
    else:
        groups: list[list[int]] = []
        pending = list(raw_groups)
        while pending:
            current = pending.pop(0)
            changed = True
            while changed:
                changed = False
                current_start, current_end = _index_span(current, count)
                current_length = _span_length(
                    path,
                    current_start,
                    current_end,
                )
                for candidate in list(pending):
                    candidate_start, candidate_end = _index_span(
                        candidate,
                        count,
                    )
                    gap = _cyclic_gap_m(
                        path,
                        current_start,
                        current_end,
                        candidate_start,
                        candidate_end,
                    )
                    if gap <= merge_gap_m:
                        current.extend(candidate)
                        pending.remove(candidate)
                        changed = True
                        break
            groups.append(current)

    windows: list[CornerWindow] = []
    for indices in groups:
        ordered = tuple(
            sorted(set(indices), key=lambda value: path.cumulative_s[value])
        )
        if not ordered:
            continue
        start_s, end_s = _index_span(list(ordered), count)
        length = _span_length(path, start_s, end_s)
        center_s = (start_s + 0.5 * length) % path.total_length_m
        windows.append(
            CornerWindow(
                indices=ordered,
                center_s_m=center_s,
                length_m=length,
            )
        )
    return windows


def _index_span(
    indices: list[int],
    count: int,
) -> tuple[int, int]:
    ordered = sorted(set(indices))
    if len(ordered) == 1:
        return ordered[0], (ordered[0] + 1) % count
    largest_gap = -1
    split = 0
    for offset, index in enumerate(ordered):
        following = ordered[(offset + 1) % len(ordered)]
        gap = (following - index) % count
        if gap > largest_gap:
            largest_gap = gap
            split = (offset + 1) % len(ordered)
    return ordered[split], ordered[split - 1]


def _span_length(
    path: TrackPath,
    start_index: int,
    end_index: int,
) -> float:
    count = len(path.points)
    if start_index == end_index:
        return path.total_length_m
    return (
        path.cumulative_s[end_index]
        - path.cumulative_s[start_index]
    ) % path.total_length_m


def _cyclic_gap_m(
    path: TrackPath,
    first_start: int,
    first_end: int,
    second_start: int,
    second_end: int,
) -> float:
    count = len(path.points)
    first_length = _span_length(path, first_start, first_end)
    second_length = _span_length(path, second_start, second_end)
    forward = (
        path.cumulative_s[second_start]
        - path.cumulative_s[first_end]
    ) % path.total_length_m
    backward = (
        path.cumulative_s[first_start]
        - path.cumulative_s[second_end]
    ) % path.total_length_m
    if first_length + second_length + min(forward, backward) >= path.total_length_m:
        return 0.0
    return min(forward, backward)
