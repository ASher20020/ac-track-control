# Final configuration set

The three JSON files in this directory are the frozen configuration entry
points for the project:

| Track | File | Status |
| --- | --- | --- |
| Shanghai | `shanghai_final.json` | V64 final candidate; verified comparison baseline is V63 |
| Zhejiang | `zhejiang_final.json` | V59 speed strategy with Shanghai longitudinal MPC parameters |
| Nordschleife | `nordschleife_candidate_v2_safe.json` | Safe candidate retained for further validation |

Use `manifest.json` for the exact source logs, lap times and runtime overrides.
The shared identified vehicle model is stored at
`configs/models/lmpc_calibrated_model_v10.json`.

The configuration files only contain controller and planning parameters.
Track layout resolution, maximum speed, path clearance and execution limits are
set by the launcher scripts under `tools/`.
