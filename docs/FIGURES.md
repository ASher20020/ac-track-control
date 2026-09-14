# Figure gallery

All public figures are generated from project code or measured CSV logs. The
raw logs are excluded from the public repository, but the final metrics and
source mapping remain in `configs/final/manifest.json`.

## Project cover

![Project cover](../assets/figures/hero_overview.png)

Source: fastest complete Nordschleife V2 safe lap, colored by speed.

## Closed-loop architecture

![Closed-loop architecture](../assets/figures/closed_loop_architecture.png)

The upper row is the forward chain. The lower row shows the simulator,
telemetry conversion and state feedback path.

## Control stack

![Control stack](../assets/figures/control_stack.png)

Global planning, fast MPC layers and plant interfaces are separated so each
layer can be replaced independently.

## Track trajectories

![Track trajectories](../assets/figures/track_maps.png)

The trajectory uses the complete `fast_lane.ai` geometry for each track.
Automatic speed is mapped to the path by continuous world-coordinate
projection, not by directly indexing `normalized_position`. The color scale is
fixed at `0-300 km/h`, which avoids distorting the low-speed sections and keeps
all three tracks comparable. Human lap times remain the reference labels; the
detailed human-versus-auto speed traces appear in the comparison figures below.

## Performance dashboard

![Performance dashboard](../assets/figures/performance_dashboard.png)

The dashboard summarizes the lap-time gap, p95 tracking error, peak speed and
the strongest evidence for each closed-loop run.

## Calibration pipeline

![Calibration pipeline](../assets/figures/calibration_pipeline.png)

The pipeline keeps data collection, sample filtering, regression, scheduled
maps and closed-loop validation separate.

## Formula overview

![Formula overview page 1](../assets/figures/mpc_equations_page-1.png)

![Formula overview page 2](../assets/figures/mpc_equations_page-2.png)

![Linear model and actuator augmentation](../assets/figures/mpc_equations_page-3.png)

The equations are typeset with LaTeX, equation numbering and standard academic
math spacing. The source is `docs/assets/mpc_equations.tex` and the compiled PDF
is `docs/assets/build/mpc_equations.pdf`.

## Weight scheduling

![MPC weight scheduling](../assets/figures/weight_schedules.png)

The figure shows the speed and lateral-load multipliers applied to the state,
steering, acceleration and jerk costs.

## Model identification

![Speed-banded model identification](../assets/figures/model_identification.png)

Coefficients are fitted independently in five speed bands. The figure also
shows the sample count and the lateral/yaw fit quality.

## Steering calibration

![Steering calibration](../assets/figures/steering_calibration.png)

The left panel fits the steady-state steering relation. The right panel shows
the steering-axis scale by speed band and the global fitted scale.

## Coupled speed planning

![Coupled speed planning](../assets/figures/coupled_speed_planning.png)

The planner and both MPC layers exchange speed, curvature and lateral-load
constraints. The diagram makes clear that this is coordinated planning rather
than one monolithic joint lateral-longitudinal NLP.

## Human comparison

![Shanghai comparison](../assets/figures/comparison_shanghai.png)

![Zhejiang comparison](../assets/figures/comparison_zhejiang.png)

The three panels compare speed, yaw rate and lateral acceleration over track
distance.

## Nordschleife profile

![Nordschleife profile](../assets/figures/nordschleife_profile.png)

The lower panel shows p95 lateral error; the upper panel exposes the remaining
longitudinal speed deficit.
