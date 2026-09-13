# From simulator to a real car

## Reusable directly

The following parts are largely vehicle-independent:

- `actc/track.py`: track geometry, Frenet projection, path resampling,
  curvature, speed envelope and edge-clearance handling.
- `actc/planning.py`: corner detection, offset optimization and conservative
  per-lap line updates.
- `actc/mpc.py`: Frenet lateral MPC structure, speed-scheduled models,
  actuator compensation and QP assembly.
- `actc/longitudinal.py`: acceleration MPC, jerk constraints, coast-down model
  and pedal mapping structure.
- `actc/lap.py`: controller comparison and offline simulation support.
- Logging, lap segmentation and analysis tools under `tools/`.
- The calibration and model-identification workflow.

## Must be replaced or hardened

### State estimation and localization

Assetto Corsa supplies ground-truth world position, heading, velocity and
acceleration. A real car needs:

- RTK-GNSS, IMU, wheel-speed and steering-angle fusion.
- Map matching and track-boundary estimation.
- Estimators for sideslip, tyre slip, vertical load and road grade.
- A validated coordinate and timing convention for the vehicle body frame.

### Actuator interface

The simulator uses a virtual Xbox controller. A real car needs:

- CAN, EtherCAT or vendor-specific steering and powertrain interfaces.
- Position, rate and torque limits for steering.
- Throttle, brake, gear and energy-recovery constraints.
- Latency, deadband, hysteresis and actuator health monitoring.

### Vehicle dynamics and tyre model

The simulator model is identified from one car and one set of simulator tyres.
For a real car, recalibrate:

- Mass, inertia and centre-of-gravity position.
- Front and rear tyre cornering stiffness.
- Load transfer, camber, temperature and pressure effects.
- Longitudinal acceleration and braking capability.
- Surface friction and changing grip.

### Safety architecture

A track-only prototype still needs:

- Independent emergency stop and safe-state control.
- Steering and braking redundancy or mechanical fallback.
- Watchdogs, timeout handling and command freshness checks.
- Envelope limiting independent of the MPC solver.
- Driver override and clear handover/recovery procedures.
- Fault logging with deterministic replay.

### Compute and real-time execution

The simulator runs on a desktop. A vehicle computer should provide:

- Deterministic scheduling and measured worst-case solve time.
- CPU, memory and thermal margin.
- Redundant power and network paths.
- A fallback controller if the QP/NLP solver times out or fails.
- Automated regression tests against recorded vehicle data.

### Validation

The simulator provides repeatable conditions that a real track does not.
Real-car validation needs:

- Low-speed actuator and emergency-stop tests.
- Skidpad and straight-line braking calibration.
- Progressive track testing with independent limits.
- Repeatable lap logging and incident review.
- Track permissions, insurance and an agreed safety plan.

## Recommended migration order

1. Keep the current planner and MPC as offline replay tools.
2. Replace simulator telemetry with a timestamped vehicle-state interface.
3. Implement a low-level actuator controller with independent limits.
4. Re-identify the vehicle model from controlled real-car tests.
5. Run the MPC in shadow mode before enabling closed-loop steering.
6. Enable longitudinal control first on a closed test area.
7. Enable lateral control at low speed and gradually expand the envelope.
8. Add redundant safety, fault handling and incident logging before any
   performance-oriented testing.
