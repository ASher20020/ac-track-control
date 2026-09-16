# Final configuration set

The current configuration entry points are:

| Track | File | Status |
| --- | --- | --- |
| Shanghai | `shanghai_frenet_local_line60_perfect.json` | User-confirmed 134.469 s lap with local line and speed offsets |
| Zhejiang | `zhejiang_frenet_local055_perfect.json` | User-confirmed 96.282 s run with final 103 lateral weights |
| Nordschleife | `nordschleife_frenet_refine_103_sync.json` | Corrected 7:52.546 crossing-to-crossing run |
| Nordschleife | `nordschleife_frenet_refine_103_candidate_v2.json` | Pending two-corner local correction candidate |

Use `manifest.json` for the exact source logs, lap times and runtime overrides.
The shared identified vehicle model is stored at
`configs/models/lmpc_calibrated_model_v10.json`.

The configuration files only contain controller and planning parameters.
Track layout resolution, maximum speed, path clearance and execution limits are
set by the launcher scripts under `tools/`.

Shared validation platform:

```text
Vehicle: BMW M4 Competition (G82)
Tyres:   Michelin PS4 Cup 2
```
