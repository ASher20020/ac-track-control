from __future__ import annotations

import math
import re
import struct
import bisect
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


AC_PHYSICS_TO_WORLD_HEADING_RAD = math.pi / 2.0


def physics_heading_to_world(heading_rad: float) -> float:
    """Convert AC's physics heading to the X/Z world-frame heading."""
    return wrap_pi(heading_rad + AC_PHYSICS_TO_WORLD_HEADING_RAD)


@dataclass
class TrackPath:
    points: list[tuple[float, float]]
    cumulative_s: list[float]
    heading_rad: list[float]
    curvature: list[float]
    speed_limit_kmh: list[float]
    total_length_m: float
    estimated_lap_s: float
    reference_points: list[tuple[float, float]] | None = None
    left_width_m: list[float] | None = None
    right_width_m: list[float] | None = None
    fast_lane_offset_m: list[float] | None = None
    line_offset_m: list[float] | None = None
    fixed_speed_limit: list[bool] | None = None

    def plan_merge(
        self,
        *,
        car_x: float,
        car_z: float,
        car_heading_rad: float,
        merge_distance_m: float = 150.0,
        merge_speed_kmh: float = 20.0,
        samples: int = 120,
        close_loop: bool = True,
    ) -> "TrackPath":
        """Create a smooth local path from the car pose to the centerline."""
        start_index = self.nearest_index(car_x, car_z)
        start_s = self.cumulative_s[start_index]
        lateral_error = self.signed_lateral_error(start_index, car_x, car_z)
        start_lateral = lateral_error
        heading_delta = wrap_pi(car_heading_rad - self.heading_rad[start_index])
        initial_slope = (
            math.tan(clamp(heading_delta, -0.55, 0.55))
            - 0.5 * start_lateral / merge_distance_m
        )
        initial_slope = clamp(initial_slope, -0.5, 0.5)

        distance = max(1.0, merge_distance_m)
        merge_points: list[tuple[float, float]] = []
        merge_speeds: list[float] = []
        fixed_speed_limit = [False] * len(self.points)
        count = len(self.points)
        for offset in range(count):
            index = (start_index + offset) % count
            ds = self.cumulative_s[index] - start_s
            if ds < 0.0:
                ds += self.total_length_m
            distance_before_start = self.total_length_m - ds
            if distance_before_start >= self.total_length_m:
                distance_before_start = 0.0

            if ds <= distance:
                t = clamp(ds / distance, 0.0, 1.0)
                h00 = 2 * t**3 - 3 * t**2 + 1
                h10 = t**3 - 2 * t**2 + t
                lateral = h00 * start_lateral + h10 * distance * initial_slope
            elif close_loop and distance_before_start <= distance:
                t = clamp(1.0 - distance_before_start / distance, 0.0, 1.0)
                h01 = -2 * t**3 + 3 * t**2
                h11 = t**3 - t**2
                lateral = h01 * start_lateral + h11 * distance * initial_slope
            else:
                lateral = 0.0

            base_x, base_z = self.points[index]
            base_heading = self.heading_rad[index]
            normal_x = -math.sin(base_heading)
            normal_z = math.cos(base_heading)
            merge_points.append(
                (car_x, car_z)
                if offset == 0
                else (
                    base_x + lateral * normal_x,
                    base_z + lateral * normal_z,
                )
            )
            near_merge = ds <= distance or (
                close_loop and distance_before_start <= distance
            )
            fixed_speed_limit[index] = near_merge
            merge_speeds.append(
                min(merge_speed_kmh, self.speed_limit_kmh[index])
                if near_merge
                else self.speed_limit_kmh[index]
            )

        return TrackPath.from_points(
            merge_points,
            merge_speeds,
            fixed_speed_limit=fixed_speed_limit,
        )

    def plan_start_merge(
        self,
        *,
        car_x: float,
        car_z: float,
        car_heading_rad: float,
        merge_distance_m: float = 120.0,
        merge_speed_kmh: float = 80.0,
    ) -> "TrackPath":
        """Blend the car offset to the centerline without reordering the loop."""
        count = len(self.points)
        start_index = self.nearest_index(car_x, car_z)
        start_s = self.cumulative_s[start_index]
        start_lateral = self.signed_lateral_error(
            start_index,
            car_x,
            car_z,
        )
        heading_delta = wrap_pi(
            car_heading_rad - self.heading_rad[start_index]
        )
        distance = max(1.0, merge_distance_m)
        initial_slope = (
            math.tan(clamp(heading_delta, -0.55, 0.55))
            - 0.5 * start_lateral / distance
        )
        initial_slope = clamp(initial_slope, -0.5, 0.5)

        points = list(self.points)
        speeds = list(self.speed_limit_kmh)
        fixed_speed_limit = [False] * count
        for offset in range(1, count):
            index = (start_index - offset) % count
            ds = (
                start_s - self.cumulative_s[index]
            ) % self.total_length_m
            if ds > 20.0:
                break
            speeds[index] = min(
                merge_speed_kmh,
                self.speed_limit_kmh[index],
            )
            fixed_speed_limit[index] = True
        for offset in range(count):
            index = (start_index + offset) % count
            ds = self.cumulative_s[index] - start_s
            if ds < 0.0:
                ds += self.total_length_m
            if ds > distance:
                continue
            t = clamp(ds / distance, 0.0, 1.0)
            h00 = 2.0 * t**3 - 3.0 * t**2 + 1.0
            h10 = t**3 - 2.0 * t**2 + t
            lateral = (
                h00 * start_lateral
                + h10 * distance * initial_slope
            )
            base_x, base_z = self.points[index]
            normal_x = -math.sin(self.heading_rad[index])
            normal_z = math.cos(self.heading_rad[index])
            points[index] = (
                base_x + lateral * normal_x,
                base_z + lateral * normal_z,
            )
            speeds[index] = min(
                merge_speed_kmh,
                self.speed_limit_kmh[index],
            )
            fixed_speed_limit[index] = True
        return TrackPath.from_points(
            points,
            speeds,
            fixed_speed_limit=fixed_speed_limit,
        )

    def with_start_speed_limit(
        self,
        *,
        car_x: float,
        car_z: float,
        distance_m: float,
        speed_kmh: float,
        pre_margin_m: float = 20.0,
        taper_distance_m: float = 0.0,
    ) -> "TrackPath":
        """Limit start speed without changing the reference geometry."""
        count = len(self.points)
        start_index = self.nearest_index(car_x, car_z)
        start_s = self.cumulative_s[start_index]
        distance = max(0.0, distance_m)
        taper_distance = max(0.0, taper_distance_m)
        speeds = list(self.speed_limit_kmh)
        fixed = (
            list(self.fixed_speed_limit)
            if self.fixed_speed_limit is not None
            else [False] * count
        )
        for offset in range(1, count):
            index = (start_index - offset) % count
            ds = (
                start_s - self.cumulative_s[index]
            ) % self.total_length_m
            if ds > pre_margin_m:
                break
            speeds[index] = min(speed_kmh, self.speed_limit_kmh[index])
            fixed[index] = True
        for offset in range(count):
            index = (start_index + offset) % count
            ds = self.cumulative_s[index] - start_s
            if ds < 0.0:
                ds += self.total_length_m
            if ds > distance:
                if taper_distance <= 0.0:
                    break
                if ds > distance + taper_distance:
                    break
                progress = clamp(
                    (ds - distance) / taper_distance,
                    0.0,
                    1.0,
                )
                blend = progress * progress * (3.0 - 2.0 * progress)
                target_speed = (
                    speed_kmh
                    + blend * (self.speed_limit_kmh[index] - speed_kmh)
                )
                speeds[index] = min(
                    self.speed_limit_kmh[index],
                    target_speed,
                )
            else:
                speeds[index] = min(
                    speed_kmh,
                    self.speed_limit_kmh[index],
                )
            fixed[index] = True
        return replace(
            self,
            speed_limit_kmh=speeds,
            fixed_speed_limit=fixed,
        )

    def with_speed_envelope(
        self,
        speed_kmh: list[float],
        *,
        blend: float = 1.0,
        maximum_speed_kmh: float = 280.0,
        only_increase: bool = False,
    ) -> "TrackPath":
        if len(speed_kmh) != len(self.points):
            raise ValueError(
                "speed envelope must match the path point count"
            )
        blend = clamp(blend, 0.0, 1.0)
        limits: list[float] = []
        for index in range(len(self.points)):
            current = self.speed_limit_kmh[index]
            target = current + blend * (
                float(speed_kmh[index]) - current
            )
            if only_increase:
                target = max(current, target)
            limits.append(
                clamp(target, 0.0, maximum_speed_kmh)
            )
        return replace(
            self,
            speed_limit_kmh=limits,
            estimated_lap_s=_estimate_lap_time(
                self.cumulative_s,
                limits,
                self.total_length_m,
            ),
        )

    def with_late_braking(
        self,
        *,
        max_speed_kmh: float,
        min_speed_kmh: float,
        lateral_accel_mps2: float,
        accel_mps2: float,
        brake_mps2: float,
        brake_point_shift_base_m: float,
        brake_point_shift_speed_gain_m_per_kmh: float,
        brake_point_shift_speed_min_kmh: float = 50.0,
        brake_point_shift_speed_max_kmh: float = 300.0,
        brake_point_shift_reference_distance_m: float = 120.0,
        entry_lateral_accel_boost: float = 0.0,
        use_friction_ellipse: bool = False,
    ) -> "TrackPath":
        limits = _speed_limits_to_kmh(
            ds=_segment_lengths(self.points),
            curvature=self.curvature,
            max_speed_kmh=max_speed_kmh,
            min_speed_kmh=min_speed_kmh,
            lateral_accel_mps2=lateral_accel_mps2,
            accel_mps2=accel_mps2,
            brake_mps2=brake_mps2,
            brake_point_shift_base_m=brake_point_shift_base_m,
            brake_point_shift_speed_gain_m_per_kmh=(
                brake_point_shift_speed_gain_m_per_kmh
            ),
            brake_point_shift_speed_min_kmh=(
                brake_point_shift_speed_min_kmh
            ),
            brake_point_shift_speed_max_kmh=(
                brake_point_shift_speed_max_kmh
            ),
            brake_point_shift_reference_distance_m=(
                brake_point_shift_reference_distance_m
            ),
            entry_lateral_accel_boost=entry_lateral_accel_boost,
            use_friction_ellipse=use_friction_ellipse,
        )
        if self.fixed_speed_limit is not None:
            limits = [
                (
                    self.speed_limit_kmh[index]
                    if self.fixed_speed_limit[index]
                    else limits[index]
                )
                for index in range(len(self.points))
            ]
        return replace(
            self,
            speed_limit_kmh=limits,
            estimated_lap_s=_estimate_lap_time(
                self.cumulative_s,
                limits,
                self.total_length_m,
            ),
        )

    def with_minimum_edge_clearance(
        self,
        target_clearance_m: float = 1.0,
        *,
        max_speed_kmh: float,
        min_speed_kmh: float,
        lateral_accel_mps2: float,
        accel_mps2: float,
        brake_mps2: float,
        smoothing_radius: int = 4,
    ) -> "TrackPath":
        if (
            self.left_width_m is None
            or self.right_width_m is None
        ):
            raise ValueError("TrackPath is missing track widths")
        current_offsets = (
            list(self.line_offset_m)
            if self.line_offset_m is not None
            else [0.0] * len(self.points)
        )
        requested: list[float] = []
        bounds: list[tuple[float, float]] = []
        for index, current in enumerate(current_offsets):
            left_width = self.left_width_m[index]
            right_width = self.right_width_m[index]
            lower = target_clearance_m - right_width
            upper = left_width - target_clearance_m
            if lower <= upper:
                requested.append(clamp(current, lower, upper))
                bounds.append((lower, upper))
            else:
                center = 0.5 * (lower + upper)
                requested.append(center)
                bounds.append((center, center))
        if smoothing_radius > 0:
            requested = _smooth(requested, smoothing_radius)
        requested = [
            clamp(offset, lower, upper)
            for offset, (lower, upper) in zip(requested, bounds)
        ]
        return self.with_line_offsets(
            requested,
            base_margin_m=target_clearance_m,
            speed_margin_m_per_kmh=0.0,
            max_edge_margin_m=target_clearance_m,
            max_speed_kmh=max_speed_kmh,
            min_speed_kmh=min_speed_kmh,
            lateral_accel_mps2=lateral_accel_mps2,
            accel_mps2=accel_mps2,
            brake_mps2=brake_mps2,
        )

    def resampled(self, spacing_m: float = 2.0) -> "TrackPath":
        spacing_m = max(0.5, float(spacing_m))
        count = max(3, int(math.ceil(self.total_length_m / spacing_m)))
        sample_s = np.linspace(
            0.0,
            self.total_length_m,
            count,
            endpoint=False,
        )
        node_s = np.asarray(self.cumulative_s, dtype=float)
        closed_s = np.concatenate([node_s, [self.total_length_m]])

        def periodic_spline(values: list[float]) -> np.ndarray:
            closed_values = np.concatenate(
                [
                    np.asarray(values, dtype=float),
                    [float(values[0])],
                ]
            )
            return CubicSpline(
                closed_s,
                closed_values,
                bc_type="periodic",
            )(sample_s)

        points_x = periodic_spline(
            [point[0] for point in self.points]
        )
        points_z = periodic_spline(
            [point[1] for point in self.points]
        )
        points = list(zip(points_x.tolist(), points_z.tolist()))
        reference_points = None
        if self.reference_points is not None:
            reference_x = periodic_spline(
                [point[0] for point in self.reference_points]
            )
            reference_z = periodic_spline(
                [point[1] for point in self.reference_points]
            )
            reference_points = list(
                zip(reference_x.tolist(), reference_z.tolist())
            )

        def optional_periodic(values: list[float] | None) -> list[float] | None:
            if values is None:
                return None
            return periodic_spline(values).tolist()

        def optional_nearest(
            values: list[bool] | None,
        ) -> list[bool] | None:
            if values is None:
                return None
            interpolated = periodic_spline(
                [1.0 if value else 0.0 for value in values]
            )
            return [value >= 0.5 for value in interpolated]

        speed_limits = periodic_spline(self.speed_limit_kmh)
        return TrackPath.from_points(
            points,
            speed_limits.tolist(),
            reference_points=reference_points,
            left_width_m=optional_periodic(self.left_width_m),
            right_width_m=optional_periodic(self.right_width_m),
            fast_lane_offset_m=optional_periodic(
                self.fast_lane_offset_m
            ),
            line_offset_m=optional_periodic(self.line_offset_m),
            fixed_speed_limit=optional_nearest(
                self.fixed_speed_limit
            ),
            curvature_smoothing_radius=4,
        )

    def _sample_at_s(
        self,
        distance_m: float,
    ) -> tuple[float, float, float, float]:
        count = len(self.points)
        wrapped = distance_m % self.total_length_m
        index = bisect.bisect_right(self.cumulative_s, wrapped) - 1
        index = max(0, min(count - 1, index))
        following = (index + 1) % count
        segment_start = self.cumulative_s[index]
        segment_length = self.cumulative_s[following] - segment_start
        if following == 0:
            segment_length += self.total_length_m
        if segment_length <= 1e-9:
            fraction = 0.0
        else:
            fraction = clamp(
                (wrapped - segment_start) / segment_length,
                0.0,
                1.0,
            )
        x0, z0 = self.points[index]
        x1, z1 = self.points[following]
        heading = self.heading_rad[index]
        speed = self.speed_limit_kmh[index]
        return (
            x0 + fraction * (x1 - x0),
            z0 + fraction * (z1 - z0),
            heading,
            speed,
        )

    @classmethod
    def from_points(
        cls,
        points: list[tuple[float, float]],
        speed_limit_kmh: list[float],
        *,
        reference_points: list[tuple[float, float]] | None = None,
        left_width_m: list[float] | None = None,
        right_width_m: list[float] | None = None,
        fast_lane_offset_m: list[float] | None = None,
        line_offset_m: list[float] | None = None,
        fixed_speed_limit: list[bool] | None = None,
        curvature_smoothing_radius: int = 3,
    ) -> "TrackPath":
        if len(points) < 2:
            raise ValueError("A track path needs at least two points")
        count = len(points)
        ds: list[float] = []
        cumulative_s = [0.0]
        for index in range(count):
            x0, z0 = points[index]
            x1, z1 = points[(index + 1) % count] if index + 1 < count else points[0]
            distance = max(math.hypot(x1 - x0, z1 - z0), 1e-6)
            ds.append(distance)
            cumulative_s.append(cumulative_s[-1] + distance)
        total_length_m = cumulative_s[-1]
        cumulative_s = cumulative_s[:-1]

        heading = [
            math.atan2(
                (
                    points[(index + 1) % count][1]
                    - points[index][1]
                ),
                points[(index + 1) % count][0] - points[index][0],
            )
            for index in range(count)
        ]
        signed_curvature: list[float] = []
        for index in range(count):
            previous = (index - 1) % count
            following = (index + 1) % count
            distance = ds[previous] + ds[index]
            signed_curvature.append(
                wrap_pi(heading[following] - heading[previous])
                / max(distance, 1e-6)
            )
        curvature = _smooth(
            signed_curvature,
            curvature_smoothing_radius,
        )
        lap_time = sum(
            ds[index]
            / max(
                0.5
                * (
                    speed_limit_kmh[index]
                    + speed_limit_kmh[(index + 1) % count]
                )
                / 3.6,
                1e-6,
            )
            for index in range(count)
        )
        return cls(
            points=points,
            cumulative_s=cumulative_s,
            heading_rad=heading,
            curvature=curvature,
            speed_limit_kmh=speed_limit_kmh,
            total_length_m=total_length_m,
            estimated_lap_s=lap_time,
            reference_points=reference_points,
            left_width_m=left_width_m,
            right_width_m=right_width_m,
            fast_lane_offset_m=fast_lane_offset_m,
            line_offset_m=line_offset_m,
            fixed_speed_limit=fixed_speed_limit,
        )

    @classmethod
    def load(
        cls,
        path: Path,
        *,
        centerline: bool = False,
        preserve_fast_lane: bool = False,
        lateral_offset_m: float = 0.0,
        max_speed_kmh: float = 75.0,
        min_speed_kmh: float = 25.0,
        lateral_accel_mps2: float = 3.0,
        accel_mps2: float = 2.5,
        brake_mps2: float = 4.0,
        smoothing_radius: int = 4,
    ) -> "TrackPath":
        raw = path.read_bytes()
        header = struct.unpack("4i", raw[:16])
        count = int(header[1])
        if count <= 2:
            raise ValueError(f"Invalid fast lane point count: {count}")
        record_size = 20
        raw_points: list[tuple[float, float]] = []
        for index in range(count):
            offset = 16 + index * record_size
            x, _y, z, _distance, _identifier = struct.unpack(
                "4f i",
                raw[offset : offset + record_size],
            )
            raw_points.append((float(x), float(z)))

        left_width_m: list[float] | None = None
        right_width_m: list[float] | None = None
        if centerline:
            detail_offset = 16 + count * record_size
            details = [
                struct.unpack(
                    "18f",
                    raw[
                        detail_offset + index * 72 :
                        detail_offset + (index + 1) * 72
                    ],
                )
                for index in range(count)
            ]
            left_width_m = [
                max(0.0, float(details[index][7]))
                for index in range(count)
            ]
            right_width_m = [
                max(0.0, float(details[index][6]))
                for index in range(count)
            ]
            if preserve_fast_lane:
                points = list(raw_points)
                reference_points = list(raw_points)
                fast_lane_offset_m = [0.0] * count
                line_offset_m = [0.0] * count
            else:
                points = _centerline_from_widths(
                    raw_points,
                    details,
                    lateral_offset_m=(
                        0.0
                        if lateral_offset_m is None
                        else lateral_offset_m
                    ),
                )
                reference_points = points
                headings = _headings_from_points(points)
                fast_lane_offset_m = [
                    -math.sin(headings[index])
                    * (raw_points[index][0] - points[index][0])
                    + math.cos(headings[index])
                    * (raw_points[index][1] - points[index][1])
                    for index in range(count)
                ]
                line_offset_m = [
                    (
                        -math.sin(headings[index])
                        * (
                            points[index][0]
                            - reference_points[index][0]
                        )
                        + math.cos(headings[index])
                        * (
                            points[index][1]
                            - reference_points[index][1]
                        )
                    )
                    for index in range(count)
                ]
        else:
            points = raw_points
            reference_points = raw_points
            fast_lane_offset_m = [0.0] * count
            line_offset_m = [0.0] * count

        ds: list[float] = []
        cumulative_s = [0.0]
        for index in range(count):
            x0, z0 = points[index]
            x1, z1 = points[(index + 1) % count]
            distance = max(math.hypot(x1 - x0, z1 - z0), 1e-6)
            ds.append(distance)
            cumulative_s.append(cumulative_s[-1] + distance)
        total_length_m = cumulative_s[-1]
        cumulative_s = cumulative_s[:-1]

        heading = _headings_from_points(points)

        signed_curvature: list[float] = []
        for index in range(count):
            previous = (index - 1) % count
            following = (index + 1) % count
            distance = ds[previous] + ds[index]
            signed_curvature.append(
                wrap_pi(heading[following] - heading[previous]) / max(distance, 1e-6)
            )
        curvature = _smooth(signed_curvature, smoothing_radius)

        speed = _speed_limits_from_geometry(
            ds=ds,
            curvature=curvature,
            max_speed_kmh=max_speed_kmh,
            min_speed_kmh=min_speed_kmh,
            lateral_accel_mps2=lateral_accel_mps2,
            accel_mps2=accel_mps2,
            brake_mps2=brake_mps2,
        )

        lap_time = sum(
            ds[index]
            / max(0.5 * (speed[index] + speed[(index + 1) % count]), 1e-6)
            for index in range(count)
        )
        return cls(
            points=points,
            cumulative_s=cumulative_s,
            heading_rad=heading,
            curvature=curvature,
            speed_limit_kmh=[value * 3.6 for value in speed],
            total_length_m=total_length_m,
            estimated_lap_s=lap_time,
            reference_points=reference_points,
            left_width_m=left_width_m,
            right_width_m=right_width_m,
            fast_lane_offset_m=fast_lane_offset_m,
            line_offset_m=line_offset_m,
            fixed_speed_limit=[False] * count,
        )

    def with_offset_factors(
        self,
        factors: list[float],
        *,
        smooth_radius: int = 12,
        base_margin_m: float = 0.35,
        speed_margin_m_per_kmh: float = 0.002,
        max_edge_margin_m: float = 1.0,
        max_speed_kmh: float = 200.0,
        min_speed_kmh: float = 25.0,
        lateral_accel_mps2: float = 3.924,
        accel_mps2: float = 3.924,
        brake_mps2: float = 4.905,
    ) -> "TrackPath":
        if self.fast_lane_offset_m is None:
            raise ValueError("TrackPath is missing planning geometry")
        if len(factors) != len(self.points):
            raise ValueError("offset factors must match the path point count")
        smoothed = _smooth(
            [clamp(value, 0.0, 1.0) for value in factors],
            smooth_radius,
        )
        return self.with_line_offsets(
            [
                self.fast_lane_offset_m[index] * smoothed[index]
                for index in range(len(self.points))
            ],
            base_margin_m=base_margin_m,
            speed_margin_m_per_kmh=speed_margin_m_per_kmh,
            max_edge_margin_m=max_edge_margin_m,
            max_speed_kmh=max_speed_kmh,
            min_speed_kmh=min_speed_kmh,
            lateral_accel_mps2=lateral_accel_mps2,
            accel_mps2=accel_mps2,
            brake_mps2=brake_mps2,
        )

    def with_line_offsets(
        self,
        requested_offsets: list[float],
        *,
        base_margin_m: float = 0.35,
        speed_margin_m_per_kmh: float = 0.002,
        max_edge_margin_m: float = 1.0,
        max_speed_kmh: float = 200.0,
        min_speed_kmh: float = 25.0,
        lateral_accel_mps2: float = 3.924,
        accel_mps2: float = 3.924,
        brake_mps2: float = 4.905,
    ) -> "TrackPath":
        if (
            self.reference_points is None
            or self.left_width_m is None
            or self.right_width_m is None
        ):
            raise ValueError("TrackPath is missing planning geometry")
        count = len(self.points)
        if len(requested_offsets) != count:
            raise ValueError("line offsets must match the path point count")
        headings = _headings_from_points(self.reference_points)
        points: list[tuple[float, float]] = []
        offsets: list[float] = []
        for index in range(count):
            speed_kmh = self.speed_limit_kmh[index]
            edge_margin = clamp(
                base_margin_m + speed_margin_m_per_kmh * speed_kmh,
                base_margin_m,
                max_edge_margin_m,
            )
            left_hard_clearance = min(
                edge_margin,
                self.left_width_m[index],
            )
            right_hard_clearance = min(
                edge_margin,
                self.right_width_m[index],
            )
            left_limit = (
                self.left_width_m[index] - left_hard_clearance
            )
            right_limit = (
                self.right_width_m[index] - right_hard_clearance
            )
            offset = clamp(
                requested_offsets[index],
                -right_limit,
                left_limit,
            )
            normal_x = -math.sin(headings[index])
            normal_z = math.cos(headings[index])
            base_x, base_z = self.reference_points[index]
            points.append(
                (
                    base_x + offset * normal_x,
                    base_z + offset * normal_z,
                )
            )
            offsets.append(offset)

        candidate = TrackPath.from_points(
            points,
            list(self.speed_limit_kmh),
            reference_points=list(self.reference_points),
            left_width_m=list(self.left_width_m),
            right_width_m=list(self.right_width_m),
            fast_lane_offset_m=(
                list(self.fast_lane_offset_m)
                if self.fast_lane_offset_m is not None
                else None
            ),
            line_offset_m=offsets,
            fixed_speed_limit=(
                list(self.fixed_speed_limit)
                if self.fixed_speed_limit is not None
                else None
            ),
            curvature_smoothing_radius=4,
        )
        candidate.speed_limit_kmh = _speed_limits_to_kmh(
            ds=_segment_lengths(points),
            curvature=candidate.curvature,
            max_speed_kmh=max_speed_kmh,
            min_speed_kmh=min_speed_kmh,
            lateral_accel_mps2=lateral_accel_mps2,
            accel_mps2=accel_mps2,
            brake_mps2=brake_mps2,
        )
        candidate.estimated_lap_s = _estimate_lap_time(
            candidate.cumulative_s,
            candidate.speed_limit_kmh,
            candidate.total_length_m,
        )
        return candidate

    def edge_clearance(self, index: int) -> tuple[float, float]:
        if (
            self.left_width_m is None
            or self.right_width_m is None
            or self.line_offset_m is None
        ):
            return (float("inf"), float("inf"))
        offset = self.line_offset_m[index]
        return (
            self.left_width_m[index] - offset,
            self.right_width_m[index] + offset,
        )

    def nearest_index(
        self,
        x: float,
        z: float,
        previous_index: int | None = None,
        search_radius: int = 120,
    ) -> int:
        count = len(self.points)
        if previous_index is None:
            candidates = range(count)
        else:
            radius = min(search_radius, count - 1)
            candidates = (
                (previous_index + offset) % count
                for offset in range(-radius, radius + 1)
            )
        return min(
            candidates,
            key=lambda index: self._distance_sq(index, x, z),
        )

    def project(
        self,
        x: float,
        z: float,
        previous_index: int | None = None,
        search_radius: int = 120,
    ) -> tuple[int, float, float, float, float, float]:
        count = len(self.points)
        if previous_index is None:
            candidates = range(count)
        else:
            radius = min(search_radius, count - 1)
            candidates = (
                (previous_index + offset) % count
                for offset in range(-radius, radius + 1)
            )
        best: tuple[float, int, float, float, float, float, float] | None = None
        for index in candidates:
            following = (index + 1) % count
            x0, z0 = self.points[index]
            x1, z1 = self.points[following]
            dx = x1 - x0
            dz = z1 - z0
            length_sq = max(dx * dx + dz * dz, 1e-12)
            fraction = clamp(
                ((x - x0) * dx + (z - z0) * dz) / length_sq,
                0.0,
                1.0,
            )
            projected_x = x0 + fraction * dx
            projected_z = z0 + fraction * dz
            distance_sq = (
                (x - projected_x) ** 2
                + (z - projected_z) ** 2
            )
            heading_delta = wrap_pi(
                self.heading_rad[following]
                - self.heading_rad[index]
            )
            heading = wrap_pi(
                self.heading_rad[index] + fraction * heading_delta
            )
            curvature = (
                self.curvature[index]
                + fraction
                * (
                    self.curvature[following]
                    - self.curvature[index]
                )
            )
            if best is None or distance_sq < best[0]:
                best = (
                    distance_sq,
                    index,
                    fraction,
                    projected_x,
                    projected_z,
                    heading,
                    curvature,
                )
        assert best is not None
        return (
            best[1],
            best[2],
            best[3],
            best[4],
            best[5],
            best[6],
        )

    def _distance_sq(self, index: int, x: float, z: float) -> float:
        point_x, point_z = self.points[index]
        return (point_x - x) ** 2 + (point_z - z) ** 2

    def lookahead_index(self, start_index: int, distance_m: float) -> int:
        count = len(self.points)
        start_s = self.cumulative_s[start_index]
        target_s = start_s + max(0.0, distance_m)
        offset = 0
        while offset < count:
            index = (start_index + offset) % count
            current_s = self.cumulative_s[index]
            if current_s < start_s:
                current_s += self.total_length_m
            if current_s >= target_s:
                return index
            offset += 1
        return start_index

    def signed_lateral_error(self, index: int, x: float, z: float) -> float:
        point_x, point_z = self.points[index]
        heading = self.heading_rad[index]
        tangent_x = math.cos(heading)
        tangent_z = math.sin(heading)
        return tangent_x * (z - point_z) - tangent_z * (x - point_x)


def _smooth(values: list[float], radius: int) -> list[float]:
    if radius <= 0:
        return list(values)
    count = len(values)
    return [
        sum(values[(index + offset) % count] for offset in range(-radius, radius + 1))
        / (2 * radius + 1)
        for index in range(count)
    ]


def _headings_from_points(
    points: list[tuple[float, float]],
) -> list[float]:
    count = len(points)
    return [
        math.atan2(
            points[(index + 1) % count][1] - points[index][1],
            points[(index + 1) % count][0] - points[index][0],
        )
        for index in range(count)
    ]


def _segment_lengths(
    points: list[tuple[float, float]],
) -> list[float]:
    count = len(points)
    return [
        max(
            math.hypot(
                points[(index + 1) % count][0] - points[index][0],
                points[(index + 1) % count][1] - points[index][1],
            ),
            1e-6,
        )
        for index in range(count)
    ]


def _speed_limits_from_geometry(
    *,
    ds: list[float],
    curvature: list[float],
    max_speed_kmh: float,
    min_speed_kmh: float,
    lateral_accel_mps2: float,
    accel_mps2: float,
    brake_mps2: float,
    speed_smoothing_radius: int = 6,
    brake_point_shift_base_m: float = 0.0,
    brake_point_shift_speed_gain_m_per_kmh: float = 0.0,
    brake_point_shift_speed_min_kmh: float = 50.0,
    brake_point_shift_speed_max_kmh: float = 300.0,
    brake_point_shift_reference_distance_m: float = 120.0,
    entry_lateral_accel_boost: float = 0.0,
    use_friction_ellipse: bool = False,
) -> list[float]:
    count = len(curvature)
    min_speed = min_speed_kmh / 3.6
    max_speed = max(min_speed, max_speed_kmh / 3.6)
    entry_weights = _entry_lateral_accel_weights(curvature)
    speed = []
    for index, kappa in enumerate(curvature):
        entry_scale = 1.0 + max(
            0.0,
            entry_lateral_accel_boost,
        ) * entry_weights[index]
        speed.append(
            clamp(
                math.sqrt(
                    lateral_accel_mps2
                    * entry_scale
                    / max(abs(kappa), 1e-7)
                ),
                min_speed,
                max_speed,
            )
        )
    if speed_smoothing_radius > 0:
        smoothed = _smooth(speed, speed_smoothing_radius)
        speed = [
            min(value, smoothed_value)
            for value, smoothed_value in zip(speed, smoothed)
        ]

    # Repeated forward/backward passes close the acceleration limits around
    # the loop because the start index is arbitrary.
    for _ in range(5):
        for index in range(count):
            previous = (index - 1) % count
            effective_accel = accel_mps2
            if use_friction_ellipse:
                lateral_usage = clamp(
                    speed[previous] ** 2
                    * abs(curvature[previous])
                    / max(lateral_accel_mps2, 1e-6),
                    0.0,
                    1.0,
                )
                effective_accel *= math.sqrt(
                    max(0.0, 1.0 - lateral_usage**2)
                )
            speed[index] = min(
                speed[index],
                math.sqrt(
                    speed[previous] ** 2
                    + 2.0 * effective_accel * ds[previous]
                ),
            )
        for index in range(count - 1, -1, -1):
            following = (index + 1) % count
            speed_kmh = speed[index] * 3.6
            shift_speed_kmh = clamp(
                speed_kmh,
                min(
                    brake_point_shift_speed_min_kmh,
                    brake_point_shift_speed_max_kmh,
                ),
                max(
                    brake_point_shift_speed_min_kmh,
                    brake_point_shift_speed_max_kmh,
                ),
            )
            brake_point_shift_m = (
                max(0.0, brake_point_shift_base_m)
                + max(
                    0.0,
                    brake_point_shift_speed_gain_m_per_kmh,
                )
                * shift_speed_kmh
            )
            reference_distance = max(
                1.0,
                brake_point_shift_reference_distance_m,
            )
            effective_brake = brake_mps2 * clamp(
                1.0 + brake_point_shift_m / reference_distance,
                1.0,
                2.0,
            )
            if use_friction_ellipse:
                lateral_usage = clamp(
                    speed[index] ** 2
                    * abs(curvature[index])
                    / max(lateral_accel_mps2, 1e-6),
                    0.0,
                    1.0,
                )
                effective_brake *= math.sqrt(
                    max(0.0, 1.0 - lateral_usage**2)
                )
            speed[index] = min(
                speed[index],
                math.sqrt(
                    speed[following] ** 2
                    + 2.0 * effective_brake * ds[index]
                ),
            )
    return speed


def _entry_lateral_accel_weights(
    curvature: list[float] | tuple[float, ...],
) -> list[float]:
    count = len(curvature)
    if count == 0:
        return []
    weights: list[float] = []
    for index in range(count):
        following = (index + 1) % count
        current_abs = abs(curvature[index])
        following_abs = abs(curvature[following])
        increase = max(0.0, following_abs - current_abs)
        relative_increase = increase / max(
            current_abs,
            following_abs,
            1e-5,
        )
        weights.append(
            clamp(math.sqrt(relative_increase), 0.0, 1.0)
        )
    return _smooth(weights, 2)


def _speed_limits_to_kmh(**kwargs) -> list[float]:
    return [
        value * 3.6
        for value in _speed_limits_from_geometry(**kwargs)
    ]


def _estimate_lap_time(
    cumulative_s: list[float],
    speed_limit_kmh: list[float],
    total_length_m: float,
) -> float:
    count = len(cumulative_s)
    return sum(
        (
            (
                cumulative_s[(index + 1) % count]
                - cumulative_s[index]
            )
            if index + 1 < count
            else total_length_m - cumulative_s[index]
        )
        / max(
            0.5
            * (
                speed_limit_kmh[index]
                + speed_limit_kmh[(index + 1) % count]
            )
            / 3.6,
            1e-6,
        )
        for index in range(count)
    )


def _centerline_from_widths(
    points: list[tuple[float, float]],
    details: list[tuple[float, ...]],
    *,
    lateral_offset_m: float = 0.0,
) -> list[tuple[float, float]]:
    count = len(points)
    centers: list[tuple[float, float]] = []
    for index in range(count):
        previous = (index - 1) % count
        following = (index + 1) % count
        x0, z0 = points[previous]
        x1, z1 = points[index]
        x2, z2 = points[following]
        heading = math.atan2(z2 - z0, x2 - x0)
        right_width = max(0.0, float(details[index][6]))
        left_width = max(0.0, float(details[index][7]))
        normal_x = -math.sin(heading)
        normal_z = math.cos(heading)
        center_offset = 0.5 * (left_width - right_width)
        center_x = x1 + center_offset * normal_x
        center_z = z1 + center_offset * normal_z
        safe_half_width = max(0.0, 0.5 * (left_width + right_width) - 1.2)
        lateral_offset = math.copysign(
            min(abs(lateral_offset_m), safe_half_width),
            lateral_offset_m,
        )
        centers.append(
            (
                center_x + lateral_offset * normal_x,
                center_z + lateral_offset * normal_z,
            )
        )
    return _smooth_points(centers, radius=2)


def _smooth_points(
    points: list[tuple[float, float]],
    radius: int,
) -> list[tuple[float, float]]:
    if radius <= 0:
        return list(points)
    count = len(points)
    return [
        (
            sum(
                points[(index + offset) % count][0]
                for offset in range(-radius, radius + 1)
            )
            / (2 * radius + 1),
            sum(
                points[(index + offset) % count][1]
                for offset in range(-radius, radius + 1)
            )
            / (2 * radius + 1),
        )
        for index in range(count)
    ]


def resolve_fast_lane(
    track_name: str,
    track_configuration: str = "",
    explicit_path: Path | None = None,
) -> Path:
    if explicit_path is not None:
        if not explicit_path.exists():
            raise FileNotFoundError(f"fast_lane.ai not found: {explicit_path}")
        return explicit_path

    if not track_configuration:
        track_configuration = _configured_track_layout(track_name)

    roots = _find_assetto_roots()
    track_dir_names = [track_configuration, "advanced", "normal", ""]
    candidates: list[Path] = []
    for root in roots:
        base = root / "content" / "tracks" / track_name
        for directory in track_dir_names:
            if directory:
                candidates.append(base / directory / "ai" / "fast_lane.ai")
            else:
                candidates.append(base / "ai" / "fast_lane.ai")
        if base.is_dir():
            layout_dirs = sorted(
                (
                    entry
                    for entry in base.iterdir()
                    if entry.is_dir()
                ),
                key=lambda entry: (
                    entry.name.lower() != "layout_main",
                    entry.name.lower(),
                ),
            )
            for layout in layout_dirs:
                candidates.append(layout / "ai" / "fast_lane.ai")
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        f"Cannot locate fast_lane.ai for track {track_name!r} ({track_configuration!r})"
    )


def _configured_track_layout(track_name: str) -> str:
    race_ini = (
        Path.home()
        / "Documents"
        / "Assetto Corsa"
        / "cfg"
        / "race.ini"
    )
    try:
        lines = race_ini.read_text(
            encoding="utf-8-sig",
            errors="ignore",
        ).splitlines()
    except OSError:
        return ""

    values: dict[str, str] = {}
    for line in lines:
        key, separator, raw_value = line.partition("=")
        if not separator:
            continue
        normalized_key = key.strip().upper()
        if normalized_key in {"TRACK", "CONFIG_TRACK"}:
            values[normalized_key] = raw_value.split(";", 1)[0].strip()

    if values.get("TRACK", "").casefold() != track_name.casefold():
        return ""
    configuration = values.get("CONFIG_TRACK", "")
    if configuration in {"", "-"}:
        return ""
    return configuration


def _find_assetto_roots() -> list[Path]:
    if _is_windows():
        import winreg

        roots: list[Path] = []
        try:
            with winreg.OpenKey(
                winreg.HKEY_CURRENT_USER,
                r"Software\Valve\Steam",
            ) as key:
                steam_path = Path(winreg.QueryValueEx(key, "SteamPath")[0])
            roots.append(steam_path / "steamapps" / "common" / "assettocorsa")
            library_file = steam_path / "steamapps" / "libraryfolders.vdf"
            if library_file.exists():
                for match in re.finditer(
                    r'"path"\s+"([^"]+)"',
                    library_file.read_text(encoding="utf-8", errors="ignore"),
                ):
                    library = Path(match.group(1).replace("\\\\", "\\"))
                    roots.append(
                        library / "steamapps" / "common" / "assettocorsa"
                    )
        except OSError:
            pass
        roots.extend(
            [
                Path(r"E:\SteamLibrary\steamapps\common\assettocorsa"),
                Path(r"C:\Program Files (x86)\Steam\steamapps\common\assettocorsa"),
            ]
        )
        return list(dict.fromkeys(root for root in roots if root.exists()))
    return []


def _is_windows() -> bool:
    import os

    return os.name == "nt"
