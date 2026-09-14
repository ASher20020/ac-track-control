# Quick start

## Runtime requirements

- Windows 10 or Windows 11.
- Assetto Corsa with Content Manager.
- Python 3.11 or newer.
- A working virtual controller path:
  - `vgamepad` plus ViGEmBus, or
  - the bundled ViGEm bridge under `tools/`.
- The car and track must already exist in Assetto Corsa.

The project uses the standard AC shared-memory interfaces. It does not require
an AC plugin or a custom physics build.

## Install

```powershell
cd path\to\ac_track_control
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -U pip
python -m pip install -r requirements.txt
```

Run the focused regression suite:

```powershell
python -m unittest discover -s tests -v
```

The expected baseline is `72` passing tests.

## Prepare AC input

Close Content Manager and Assetto Corsa. Prepare the virtual Xbox profile:

```powershell
python -m actc.prepare_input --x360
```

Restore the wheel profile after testing:

```powershell
python -m actc.prepare_input --restore
```

## Start a controller

Use the current AC session's track. The launcher detects the loaded track and
resolves `fast_lane.ai` from `race.ini`; it does not accept a track override.

```powershell
# Shanghai final candidate: V64, 300 km/h, full braking table
.\tools\start_v64_shanghai.ps1

# Zhejiang final candidate: V59 speed strategy, synchronized longitudinal MPC
.\tools\start_zhejiang_final.ps1

# Nordschleife safe candidate: 1.2 m line clearance and 1.02 g scaled-speed cap
.\tools\start_nordschleife_v2_safe.ps1
```

Each launcher starts a hidden Python process and writes three files under
`logs/`:

- `<run>.csv`: synchronized telemetry, commands, MPC statistics and lap data.
- `<run>.out.log`: controller console output.
- `<run>.err.log`: errors and tracebacks.

The controller waits until AC reports a live driving state and the car is
stationary with no pedal input. It then takes over and releases all controls on
shutdown.

## Inspect a run

```powershell
python -m tools.summarize_laps logs\lmpc_shanghai_v63_h50dt05_265_5lap.csv
python -m tools.analyze_segments logs\lmpc_shanghai_v63_h50dt05_265_5lap.csv
python -m tools.plot_lap_debug logs\lmpc_shanghai_v63_h50dt05_265_5lap.csv
```

Regenerate the public figures:

```powershell
python -m tools.build_comparison_figures
python -m tools.build_visual_report
python -m tools.build_model_diagnostics
python -m tools.build_project_assets
```

## Rebuild the identified lateral model

The checked-in model used by the final launchers is:

```text
configs/models/lmpc_calibrated_model_v10.json
```

To repeat identification from a clean manual or controller log:

```powershell
python -m tools.fit_calibrated_dynamics `
  logs\vehicle_dynamics_reference_20min.csv `
  --base-model logs\lmpc_round2_identified_model_v6.json `
  --identification-seconds 600 `
  --max-lateral-g 0.80 `
  --output configs\models\lmpc_calibrated_model_v10.json
```

The source log is not distributed in the public repository. Reuse requires a
new low-slip, in-bounds driving log that covers the relevant speed bands.

## Safety boundaries

This repository controls a simulator through virtual input devices. It is not
a production vehicle controller and must not be connected to a real steering,
throttle or brake actuator without an independent safety architecture,
hardware limits, fault handling and track-specific validation.
