# Reproducibility

This document separates what can be reproduced from the public repository from
what requires the original simulator logs or local Assetto Corsa installation.

## Reproducible from the public repository

The checked-in code, final configurations and identified vehicle model are
sufficient to:

- install the Python dependencies;
- run the complete regression suite;
- load every final launcher configuration;
- run the final controllers inside Assetto Corsa;
- inspect the archived result tables, metrics and figures.

```powershell
python -m pip install -r requirements.txt
python -m unittest discover -s tests -q
```

The expected test baseline is `94` passing tests.

## Runtime dependencies outside the repository

A closed-loop run also requires:

- Windows 10 or Windows 11;
- Assetto Corsa and Content Manager;
- the BMW M4 Competition (G82) and the three validated track layouts;
- a working virtual Xbox path through `vgamepad` and ViGEmBus;
- `fast_lane.ai` and track width data installed by Assetto Corsa.

These are simulator assets, not Python package dependencies, so they are not
vendored in this repository.

## Results that require original logs

The following figures consume measured CSV logs and cannot be regenerated from
a clean clone:

- human-versus-auto comparison plots;
- the Nordschleife speed and tracking profile;
- the three-track speed maps and cover figure;
- replay-validation plots and steering fitting diagnostics.

The corresponding CSV logs are intentionally excluded from the public
repository because of their size. The checked-in final configurations, metric
summaries and figures preserve the verified evidence, but independent
recalculation of every lap metric requires a new simulator run or the original
logs.

## Optional acados check

`tools/check_acados.py` is an optional nonlinear-MPC smoke test. It requires a
local `vendor/acados` checkout and is not needed by the final linear MPC
controllers.

## Regeneration summary

| Artifact | Public repo only | Requires original logs |
| --- | --- | --- |
| Python regression tests | Yes | No |
| Final controller launch | Yes, with Assetto Corsa installed | No |
| Track result cards | Yes | No |
| Architecture and control-stack diagrams | Yes | No |
| Human-versus-auto comparisons | No | Yes |
| Track speed maps and cover | No | Yes |
| Replay validation | No | Yes |
