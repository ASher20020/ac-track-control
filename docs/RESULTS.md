# Verified results

All lap times below are derived from complete laps in the original CSV logs.
The public repository excludes the raw logs because they are large, but every
number has a corresponding source path in `configs/final/manifest.json`.

## Summary

| Track | Human teaching lap | Best automatic lap | Gap | Automatic max speed | Tyres out | Status |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Shanghai | 136.248 s | 143.543 s | +7.295 s | 268.7 km/h | 0 | V63 verification baseline; V64 final candidate retained |
| Zhejiang | 94.329 s | 103.764 s | +9.436 s | 229.2 km/h | 2 | V59 speed strategy with Shanghai longitudinal MPC parameters |
| Nordschleife | - | 526.825 s | - | 293.8 km/h | 0 | V2 safe candidate, one complete lap |

![Verified results](../assets/figures/track_results.png)

## Shanghai

The fastest verified automatic lap is `143.542842 s`, obtained with:

- `lmpc_shanghai_v63_h50dt05_265_5lap.csv`
- Historical verification config `lmpc_shanghai_v63_h50dt05_265.json`
  (not distributed; the public final config is in `configs/final/`)
- 265 km/h maximum speed
- Longitudinal MPC horizon `50 x 0.05 s`

The retained final candidate is V64:

- `configs/final/shanghai_final.json`
- 300 km/h maximum speed
- Full pre-reduction braking table
- High-speed braking lead `0.004 s/(km/h)`

V64 has one complete clean lap in the log at `159.736690 s`; that lap includes
a low-speed/reset phase and is not a valid lap-time comparison against V63.
The V64 configuration should be treated as the deployment candidate, while the
V63 result is the verified comparison baseline.

The Shanghai human-versus-auto comparison shows:

- The automatic car is consistently close to the human on long straights.
- The largest deficits occur in corner entry and exit speed, around
  `1.45-1.50 km`, `2.95-3.05 km`, and `4.60-4.70 km`.
- The human reaches higher peak lateral acceleration in several corners,
  especially around `0.7-0.9 km` and `4.9-5.1 km`.
- The automatic controller is smoother in yaw rate and avoids abrupt steering
  reversals.

## Zhejiang

The verified automatic lap is `103.764338 s` with V59:

- Historical verification config `lmpc_zhejiang_manual_v59_microspeed.json`
  (not distributed; the public final config is in `configs/final/`)
- `lmpc_v59_rollback_5lap.csv`
- Four complete laps between `103.764 s` and `103.791 s`
- Maximum speed `229.2 km/h`
- p95 lateral error `0.535 m`

The final configuration keeps the Zhejiang-specific speed scale table and
synchronizes the longitudinal MPC section with Shanghai. The synchronized
`50 x 0.05 s` configuration has not yet been re-run end-to-end, so the
`103.764 s` result is the verification baseline rather than a measurement of
the final synchronized file.

The Zhejiang comparison shows:

- The automatic car is about `10.2 km/h` slower on average over the lap.
- Most of the gap comes from lower mid-corner and corner-exit speed.
- The human uses roughly `1.1-1.5 g` in several corners where the automatic
  controller remains closer to `0.8-1.1 g`.
- The automatic controller is repeatable, with four laps separated by only
  about `27 ms`.

## Nordschleife

The V2 safe candidate completed the first full Nordschleife lap without any
tyre leaving the track:

- Lap time: `526.824546 s`, about `8:46.82`
- Maximum speed: `293.8 km/h`
- p95 lateral error: `0.461 m`
- Maximum lateral error: `1.877 m`
- Tyres out: `0`

The remaining bottleneck is longitudinal execution, not lateral stability.
High-speed speed deficits reached `50-87 km/h` at p95 in the main speed bands.
The most visible case is `480-490 s`, where the target was `300 km/h`, the
actual speed was about `254 km/h`, and throttle was already saturated.

![Nordschleife speed and tracking profile](../assets/figures/nordschleife_profile.png)

## Interpretation

The project demonstrated:

- Stable closed-loop control at 50 Hz.
- A complete Nordschleife lap with no tyre exit.
- Repeatable laps on Shanghai and Zhejiang.
- Good path tracking with p95 lateral error between `0.46 m` and `0.57 m`
  on the verified runs.

The main performance gap against the human lap is not basic path following.
It is the use of available lateral grip and the ability to convert that grip
into earlier throttle and higher corner-exit speed.
