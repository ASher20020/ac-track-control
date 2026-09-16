from __future__ import annotations


ACCELERATION_CAPABILITY_KMH = (
    0.0,
    80.0,
    100.0,
    120.0,
    140.0,
    160.0,
    180.0,
    220.0,
    300.0,
)
ACCELERATION_CAPABILITY_MPS2 = (
    7.0,
    7.0,
    6.2,
    5.25,
    4.45,
    3.50,
    3.05,
    2.00,
    1.80,
)

BRAKING_CAPABILITY_KMH = (
    0.0,
    60.0,
    90.0,
    120.0,
    160.0,
    200.0,
    260.0,
    300.0,
)
BRAKING_CAPABILITY_MPS2 = (
    16.28,
    15.75,
    15.96,
    16.38,
    17.22,
    18.06,
    18.06,
    17.33,
)
PLANNING_BRAKING_MAX_MPS2 = 17.0
PLANNING_BRAKING_MARGIN = 1.00
LATE_BRAKING_ENGAGEMENT = 0.0
LATE_BRAKING_USAGE = 0.0
LATE_BRAKING_MARGIN = 1.05


def _interpolate(
    value_mps: float,
    breakpoints_kmh: tuple[float, ...],
    values_mps2: tuple[float, ...],
) -> float:
    speed_kmh = max(0.0, float(value_mps) * 3.6)
    if speed_kmh <= breakpoints_kmh[0]:
        return values_mps2[0]
    if speed_kmh >= breakpoints_kmh[-1]:
        return values_mps2[-1]
    for index in range(1, len(breakpoints_kmh)):
        upper = breakpoints_kmh[index]
        if speed_kmh <= upper:
            lower = breakpoints_kmh[index - 1]
            fraction = (speed_kmh - lower) / max(upper - lower, 1e-9)
            return (
                values_mps2[index - 1]
                + fraction
                * (values_mps2[index] - values_mps2[index - 1])
            )
    return values_mps2[-1]


def acceleration_capability_mps2(speed_mps: float) -> float:
    return _interpolate(
        speed_mps,
        ACCELERATION_CAPABILITY_KMH,
        ACCELERATION_CAPABILITY_MPS2,
    )


def braking_capability_mps2(speed_mps: float) -> float:
    return _interpolate(
        speed_mps,
        BRAKING_CAPABILITY_KMH,
        BRAKING_CAPABILITY_MPS2,
    )


def planning_braking_capability_mps2(speed_mps: float) -> float:
    return min(
        PLANNING_BRAKING_MAX_MPS2,
        braking_capability_mps2(speed_mps) * PLANNING_BRAKING_MARGIN,
    )
