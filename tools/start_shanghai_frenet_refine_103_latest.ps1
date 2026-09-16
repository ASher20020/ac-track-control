$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$logPath = Join-Path $projectRoot "logs\lmpc_shanghai_frenet_refine_103_latest_local_line60.csv"
$outPath = Join-Path $projectRoot "logs\lmpc_shanghai_frenet_refine_103_latest_local_line60.out.log"
$errPath = Join-Path $projectRoot "logs\lmpc_shanghai_frenet_refine_103_latest_local_line60.err.log"

$arguments = @(
    "-m", "tools.auto_start",
    "--laps", "5",
    "--control-laps", "5",
    "--planning-laps", "0",
    "--racing-line", "optimized",
    "--path-resample-spacing-m", "2.0",
    "--line-target-clearance-m", "0.8",
    "--line-offset-envelope",
    "configs\line_offsets\shanghai_local_two_corners.json",
    "--offtrack-limit-m", "12.0",
    "--longitudinal-horizon", "50",
    "--longitudinal-dt", "0.05",
    "--duration", "1800",
    "--timeout", "900",
    "--max-speed-kmh", "300",
    "--lateral-accel-mps2", "8.3385",
    "--longitudinal-accel-mps2", "7.0",
    "--braking-accel-mps2", "52.0",
    "--max-braking-accel-mps2", "52.0",
    "--target-rise-kmh-s", "40",
    "--target-fall-kmh-s", "188",
    "--braking-safety-factor", "0.99",
    "--braking-response-time-s", "0.03",
    "--braking-lead-distance-m", "0",
    "--braking-lead-speed-gain-m-per-kmh", "0",
    "--entry-lateral-accel-boost", "0",
    "--no-friction-ellipse-planning",
    "--speed-envelope",
    "configs\speed_envelopes\shanghai_local_60s.json",
    "--speed-envelope-blend", "1.0",
    "--speed-envelope-scale", "1.0",
    "--no-speed-envelope-only-increase",
    "--merge-speed-kmh", "100",
    "--startup-merge-distance", "60",
    "--lateral-controller", "lmpc",
    "--longitudinal-controller", "mpc",
    "--no-autotune",
    "--resume-tuning",
    "configs\final\shanghai_frenet_refine_103_latest.json",
    "--mpc-model",
    "configs\models\lmpc_calibrated_model_v10.json",
    "--log",
    $logPath
)

$process = Start-Process `
    -FilePath "python" `
    -ArgumentList $arguments `
    -WorkingDirectory $projectRoot `
    -RedirectStandardOutput $outPath `
    -RedirectStandardError $errPath `
    -WindowStyle Hidden `
    -PassThru

Write-Output "Shanghai refine 103 latest launcher PID: $($process.Id)"
