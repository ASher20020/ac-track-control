from __future__ import annotations

import math
import unittest

import numpy as np

from actc.planning import (
    PlanningConfig,
    PlanningLineTuner,
    PlanningSample,
    _control_node_bounds,
)
from actc.track import TrackPath


def make_track() -> TrackPath:
    count = 240
    points = [
        (
            50.0 * math.cos(2.0 * math.pi * index / count),
            50.0 * math.sin(2.0 * math.pi * index / count),
        )
        for index in range(count)
    ]
    path = TrackPath.from_points(points, [80.0] * count)
    path.reference_points = list(points)
    path.left_width_m = [6.0] * count
    path.right_width_m = [6.0] * count
    path.fast_lane_offset_m = [2.0] * count
    path.line_offset_m = [0.0] * count
    return path


class PlanningLineTunerTests(unittest.TestCase):
    def test_line_offsets_are_clamped_inside_safe_boundary(self) -> None:
        path = make_track()
        candidate = path.with_line_offsets(
            [20.0] * len(path.points),
            base_margin_m=0.5,
            speed_margin_m_per_kmh=0.0,
            max_edge_margin_m=0.5,
        )
        clearances = [
            min(candidate.edge_clearance(index))
            for index in range(len(candidate.points))
        ]
        self.assertGreaterEqual(min(clearances), 0.49)

    def test_narrow_half_width_uses_soft_clearance(self) -> None:
        path = make_track()
        path.left_width_m[0] = 0.82
        config = PlanningConfig(
            geometry_target_clearance_m=1.0,
            geometry_hard_clearance_m=0.80,
        )
        bounds = _control_node_bounds(
            path,
            np.asarray([path.cumulative_s[0]]),
            config,
        )
        lower, upper = bounds[0]
        self.assertLess(lower, upper)
        self.assertLessEqual(upper, 0.02 + 1e-9)
        candidate = path.with_line_offsets(
            [0.0] * len(path.points),
            base_margin_m=0.80,
            speed_margin_m_per_kmh=0.0,
            max_edge_margin_m=0.80,
        )
        self.assertGreaterEqual(
            candidate.edge_clearance(0)[0],
            0.80 - 1e-9,
        )

    def test_stable_lap_releases_margin_and_unsafe_lap_retreats(self) -> None:
        path = make_track()
        tuner = PlanningLineTuner(
            path,
            PlanningConfig(
                initial_blend=0.5,
                maximum_blend=0.9,
                stable_blend_step=0.1,
                unsafe_blend_retreat=0.2,
                optimize_initial_line=False,
                corner_curvature_threshold=0.001,
            ),
        )
        stable_samples = [
            PlanningSample(
                path_index=index,
                lateral_error_m=0.05,
                heading_error_rad=0.01,
                lateral_g=0.4,
                longitudinal_g=0.2,
                tyres_out=0,
                steer_delta=0.0005,
                speed_error_kmh=2.0,
                edge_clearance_m=1.0,
            )
            for index in range(len(path.points))
        ]
        update = tuner.update(stable_samples, lap_time_s=100.0)
        self.assertEqual(update.stable_corners, len(tuner.corners))
        self.assertTrue(
            all(
                value > 0.5
                for value in update.blend_by_corner
            )
        )

        unsafe_samples = [
            PlanningSample(
                path_index=sample.path_index,
                lateral_error_m=1.0,
                heading_error_rad=0.02,
                lateral_g=0.9,
                longitudinal_g=0.3,
                tyres_out=0,
                steer_delta=0.001,
                speed_error_kmh=2.0,
                edge_clearance_m=0.05,
            )
            for sample in stable_samples
        ]
        update = tuner.update(unsafe_samples, lap_time_s=110.0)
        self.assertEqual(update.unsafe_corners, len(tuner.corners))
        self.assertTrue(
            all(
                value < 0.6
                for value in update.blend_by_corner
            )
        )


if __name__ == "__main__":
    unittest.main()
