<h1 align="center">AC TRACK CONTROL</h1>

<p align="center">
  Planning + lateral MPC + longitudinal MPC + AC closed loop
</p>

<p align="center">
  A reusable simulator testbed for autonomous racing research.
</p>

<p align="center">
  <img src="assets/figures/hero_overview.png" alt="AC Track Control product overview" width="100%">
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Platform-Windows-0078D4?style=for-the-badge" alt="Windows">
  <img src="https://img.shields.io/badge/Python-3.11%2B-3776AB?style=for-the-badge&amp;logo=python&amp;logoColor=white" alt="Python 3.11+">
  <img src="https://img.shields.io/badge/Control-50%20Hz-25896D?style=for-the-badge" alt="50 Hz control">
  <img src="https://img.shields.io/badge/Lateral-LMPC-2166D5?style=for-the-badge" alt="Lateral LMPC">
  <img src="https://img.shields.io/badge/Longitudinal-MPC-E17A2D?style=for-the-badge" alt="Longitudinal MPC">
</p>

<p align="center">
  <b>Assetto Corsa 赛车自动驾驶规划与控制闭环</b><br>
  共享内存遥测、Frenet 路径跟踪、横向 LMPC、纵向加速度 MPC、赛车线优化与速度包络规划
</p>

项目目标不是做一个“键盘自动驾驶 demo”，而是把赛车自动驾驶中真正需要打通的环节做成可复用工程：实时状态接口、模型辨识、速度与制动标定、路径规划、MPC 控制、日志复盘和赛道结果验证。

## Product specs

<p align="center">
  <img src="https://img.shields.io/badge/Tracks-3-132238?style=for-the-badge" alt="3 validated tracks">
  <img src="https://img.shields.io/badge/Control-50%20Hz-132238?style=for-the-badge" alt="50 Hz closed-loop control">
  <img src="https://img.shields.io/badge/Lateral%20LMPC-20%20x%2020%20ms-132238?style=for-the-badge" alt="Lateral LMPC 20 by 20 ms">
  <img src="https://img.shields.io/badge/Longitudinal%20MPC-50%20x%2050%20ms-132238?style=for-the-badge" alt="Longitudinal MPC 50 by 50 ms">
  <img src="https://img.shields.io/badge/Vehicle-BMW%20M4%20Competition%20G82-132238?style=for-the-badge" alt="BMW M4 Competition G82">
  <img src="https://img.shields.io/badge/Tyres-Michelin%20PS4%20Cup%202-132238?style=for-the-badge" alt="Michelin PS4 Cup 2">
</p>

统一实验平台：

```text
Vehicle: BMW M4 Competition (G82)
Tyres:   Michelin PS4 Cup 2
```

## Verified results

<p align="center">
  <img src="https://img.shields.io/badge/Shanghai-2%3A14.399-2166D5?style=flat-square" alt="Shanghai 2:14.399">
  <img src="https://img.shields.io/badge/Zhejiang-1%3A36.245-25896D?style=flat-square" alt="Zhejiang 1:36.245">
  <img src="https://img.shields.io/badge/Nordschleife-7%3A52.546-C44B3D?style=flat-square" alt="Nordschleife 7:52.546">
  <img src="https://img.shields.io/badge/Shanghai%20tyres%20out-1-6B7280?style=flat-square" alt="Shanghai tyres out 1">
  <img src="https://img.shields.io/badge/Zhejiang%20tyres%20out-2-E17A2D?style=flat-square" alt="Zhejiang tyres out 2">
  <img src="https://img.shields.io/badge/Nordschleife%20tyres%20out-2-E17A2D?style=flat-square" alt="Nordschleife tyres out 2">
</p>

| Track | Human | Auto | Gap | Auto max speed | Tyres out | Status |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Shanghai | 136.248 s | 134.399 s | -1.849 s | 300 km/h cap | 1 | local-line60 candidate |
| Zhejiang | 94.329 s | 96.245 s | +1.917 s | 280 km/h cap | 2 | local055 user-confirmed candidate |
| Nordschleife | 7:17.632 | 7:52.546 | +34.914 s | 300 km/h cap | 2 | corrected crossing-to-crossing result |

![Track results](assets/figures/track_results.png)

![Performance dashboard](assets/figures/performance_dashboard.png)

![Latest validated configurations](assets/figures/latest_track_results.png)

![Zhejiang lateral calibration](assets/figures/lateral_calibration_zhejiang.png)

![Longitudinal braking upgrade](assets/figures/longitudinal_braking_upgrade.png)

The latest candidates combine the 103 lateral weights, measured acceleration
envelope, reachable planning-brake envelope and isolated track-local speed or
line offsets. See `configs/final/manifest.json` for exact run provenance.

## Visual overview

![Closed-loop architecture](assets/figures/closed_loop_architecture.png)

![Track trajectories](assets/figures/track_maps.png)

## What is implemented

- Standard AC shared-memory telemetry for physics, graphics and static pages.
- 50 Hz closed-loop control through a virtual Xbox controller.
- Frenet-frame lateral linear MPC with a speed-scheduled dynamic bicycle model.
- Steering actuator, lead, lag, rate and jerk compensation.
- Longitudinal acceleration MPC with speed, acceleration and jerk costs.
- Measured throttle/brake mapping and a speed-dependent braking response table.
- Fast-lane geometry, track widths, racing-line offsets and edge clearance.
- Curvature-based speed limits, acceleration/braking passes and per-track gain tables.
- High-speed braking lead compensation and a scaled-speed lateral grip cap.
- CSV run logs, lap summaries, segment analysis and figure generation.

![Control stack](assets/figures/control_stack.png)

![Calibration pipeline](assets/figures/calibration_pipeline.png)

## Repository layout

```text
actc/                    Planning, controllers, MPC and simulator interfaces
configs/final/           Frozen track configurations and provenance manifest
configs/models/          Identified vehicle model used by final launchers
tools/                   Launchers, calibration, analysis and figure scripts
tests/                   Focused regression tests
assets/figures/          Public project figures
assets/data/             Reproducible comparison metrics
docs/                    Technical report, quick start and transfer notes
```

## Quick start

Requirements: Windows, Assetto Corsa, Content Manager, Python 3.11+, and
ViGEmBus/vgamepad.

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

Prepare the virtual controller:

```powershell
python -m actc.prepare_input --x360
```

Launch one of the final candidates:

```powershell
.\tools\start_v64_shanghai.ps1
.\tools\start_zhejiang_final.ps1
.\tools\start_nordschleife_v2_safe.ps1
```

Read `docs/QUICKSTART.md` for the complete workflow.

## Documentation

- [Technical report](docs/TECHNICAL_REPORT.md)
- [Figure gallery](docs/FIGURES.md)
- [Paper-style MPC equations](docs/assets/build/mpc_equations.pdf)
- [Quick start](docs/QUICKSTART.md)
- [Verified results](docs/RESULTS.md)
- [Real-car transfer](docs/REAL_CAR_TRANSFER.md)
- [Final configuration manifest](configs/final/manifest.json)

## Model details

- [Lateral model replay validation](assets/figures/model_replay_comparison.png)
- [Longitudinal throttle and brake maps](assets/figures/longitudinal_maps.png)
- [MPC weight scheduling](assets/figures/weight_schedules.png)
- [Coupled speed planning](assets/figures/coupled_speed_planning.png)
- [Lateral model identification](assets/figures/model_identification.png)

## Key limitations

- The public repository excludes raw run logs and the vendored acados tree.
  The checked-in figures and manifest preserve the verified evidence.
- The Nordschleife corrected time uses the crossing-to-crossing interval and
  excludes the pre-line launch segment.
- Track-local speed envelopes and line offsets are kept as separate assets and
  must be supplied by the corresponding launcher.
- Nordschleife candidate v2 remains pending test; the sync result is the
  current verified run.
- The project is simulator-only and is not a production vehicle controller.

## Project lineage

The project evolved from basic speed/heading control into a full racing stack:
shared-memory bring-up, steering and pedal calibration, LQR/Stanley baselines,
lateral LMPC, longitudinal calibration, longitudinal MPC, per-track speed
scaling, racing-line safety constraints and Nordschleife validation.

This project is not affiliated with Kunos Simulazioni or Assetto Corsa.
