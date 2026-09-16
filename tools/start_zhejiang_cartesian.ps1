$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$logPath = Join-Path $projectRoot "logs\lmpc_zhejiang_cartesian_v5.csv"
$outPath = Join-Path $projectRoot "logs\lmpc_zhejiang_cartesian_v5.out.log"
$errPath = Join-Path $projectRoot "logs\lmpc_zhejiang_cartesian_v5.err.log"

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
    "--duration", "1800",
    "--timeout", "600",
    "--max-speed-kmh", "280",
    "--lateral-accel-mps2", "6.867",
    "--longitudinal-accel-mps2", "6.0",
    "--braking-accel-mps2", "18.0",
    "--max-braking-accel-mps2", "22.0",
    "--target-rise-kmh-s", "30",
    "--braking-safety-factor", "0.98",
    "--braking-response-time-s", "0.05",
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
    "configs\final\zhejiang_cartesian_v5.json",
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

Write-Output "Zhejiang Cartesian launcher PID: $($process.Id)"
