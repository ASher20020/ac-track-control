$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$logPath = Join-Path $projectRoot "logs\lmpc_nordschleife_v1_accel065g.csv"
$outPath = Join-Path $projectRoot "logs\lmpc_nordschleife_v1_accel065g.out.log"
$errPath = Join-Path $projectRoot "logs\lmpc_nordschleife_v1_accel065g.err.log"

$arguments = @(
    "-m", "tools.auto_start",
    "--laps", "5",
    "--control-laps", "5",
    "--planning-laps", "0",
    "--racing-line", "optimized",
    "--path-resample-spacing-m", "2.0",
    "--line-target-clearance-m", "0.8",
    "--offtrack-limit-m", "12.0",
    "--longitudinal-horizon", "50",
    "--longitudinal-dt", "0.05",
    "--duration", "3600",
    "--max-speed-kmh", "300",
    "--lateral-accel-mps2", "6.3765",
    "--longitudinal-accel-mps2", "5.0",
    "--braking-accel-mps2", "16.0",
    "--max-braking-accel-mps2", "20.0",
    "--target-rise-kmh-s", "20",
    "--braking-safety-factor", "0.96",
    "--braking-response-time-s", "0.1",
    "--high-speed-braking-gain-s-per-kmh", "0.004",
    "--braking-lead-distance-m", "0",
    "--braking-lead-speed-gain-m-per-kmh", "0",
    "--entry-lateral-accel-boost", "0",
    "--no-friction-ellipse-planning",
    "--merge-speed-kmh", "100",
    "--startup-merge-distance", "60",
    "--lateral-controller", "lmpc",
    "--longitudinal-controller", "mpc",
    "--no-autotune",
    "--resume-tuning",
    "configs\lmpc_nordschleife_v1_lat065g.json",
    "--mpc-model",
    "logs\lmpc_calibrated_model_v10.json",
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

Write-Output "Nordschleife launcher PID: $($process.Id)"
