$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$logPath = Join-Path $projectRoot "logs\lmpc_nordschleife_frenet_refine_103_candidate_v2.csv"
$outPath = Join-Path $projectRoot "logs\lmpc_nordschleife_frenet_refine_103_candidate_v2.out.log"
$errPath = Join-Path $projectRoot "logs\lmpc_nordschleife_frenet_refine_103_candidate_v2.err.log"

$arguments = @(
    "-m", "tools.auto_start",
    "--laps", "2",
    "--control-laps", "2",
    "--planning-laps", "0",
    "--racing-line", "optimized",
    "--path-resample-spacing-m", "2.0",
    "--line-target-clearance-m", "0.8",
    "--line-offset-envelope",
    "configs\line_offsets\nordschleife_candidate_v2.json",
    "--offtrack-limit-m", "12.0",
    "--longitudinal-horizon", "50",
    "--longitudinal-dt", "0.05",
    "--duration", "3600",
    "--timeout", "600",
    "--max-speed-kmh", "300",
    "--lateral-accel-mps2", "7.2593",
    "--longitudinal-accel-mps2", "7.0",
    "--braking-accel-mps2", "15.3",
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
    "configs\speed_envelopes\nordschleife_candidate_v2.json",
    "--speed-envelope-blend", "1.0",
    "--speed-envelope-scale", "1.0",
    "--no-speed-envelope-only-increase",
    "--merge-speed-kmh", "100",
    "--startup-merge-distance", "60",
    "--lateral-controller", "lmpc",
    "--longitudinal-controller", "mpc",
    "--no-autotune",
    "--resume-tuning",
    "configs\final\nordschleife_frenet_refine_103_candidate_v2.json",
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

Write-Output "Nordschleife refine 103 candidate v2 launcher PID: $($process.Id)"
