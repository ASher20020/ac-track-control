from __future__ import annotations

from dataclasses import asdict, dataclass


BASE_SPEED_SCALE_BY_BAND = (
    1.16,
    1.23,
    1.24,
    1.28,
    1.29,
    1.29,
    1.31,
    1.33,
    1.33,
)


@dataclass(frozen=True)
class SpeedCalibrationCandidate:
    design_index: int
    speed_scale_by_band: tuple[float, ...]

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def build_speed_calibration_schedule(
) -> tuple[SpeedCalibrationCandidate, ...]:
    updates = (
        (0.005, 0.000),
        (0.005, 0.000),
        (0.000, 0.005),
        (0.005, 0.000),
        (0.000, 0.005),
        (0.005, 0.000),
        (0.000, 0.005),
        (0.005, 0.000),
        (0.000, 0.005),
        (0.005, 0.005),
    )
    schedule: list[SpeedCalibrationCandidate] = []
    current = list(BASE_SPEED_SCALE_BY_BAND)
    for design_index, (band_3_delta, band_6_delta) in enumerate(
        updates,
        start=1,
    ):
        current[3] += band_3_delta
        current[6] += band_6_delta
        schedule.append(
            SpeedCalibrationCandidate(
                design_index=design_index,
                speed_scale_by_band=tuple(current),
            )
        )
    return tuple(schedule)
