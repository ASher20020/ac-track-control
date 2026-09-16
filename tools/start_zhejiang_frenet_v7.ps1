$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$logPath = Join-Path $projectRoot "logs\lmpc_zhejiang_frenet_v7.csv"
$outPath = Join-Path $projectRoot "logs\lmpc_zhejiang_frenet_v7.out.log"
$errPath = Join-Path $projectRoot "logs\lmpc_zhejiang_frenet_v7.err.log"

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
    "--lateral-accel-mps2", "7.848",
    "--longitudinal-accel-mps2", "7.0",
    "--braking-accel-mps2", "40.0",
    "--max-braking-accel-mps2", "40.0",
    "--target-rise-kmh-s", "40",
    "--target-fall-kmh-s", "100",
    "--braking-safety-factor", "0.99",
    "--braking-response-time-s", "0.03",
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
    "configs\final\zhejiang_frenet_v7.json",
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

Write-Output "Zhejiang Frenet strong launcher PID: $($process.Id)"
