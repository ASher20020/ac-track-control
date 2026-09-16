$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$lapsPath = Join-Path $projectRoot "logs\lmpc_nordschleife_frenet_075_tuned_v2_laps.jsonl"
$watchLogPath = Join-Path $projectRoot "logs\close_ac_after_nordschleife_075_tuned_v2.log"
$deadline = (Get-Date).AddHours(8)
$controllerSeen = $false
$reason = "timeout"

while ((Get-Date) -lt $deadline) {
    $lapCount = 0
    if (Test-Path $lapsPath) {
        $lapCount = @(
            Get-Content -LiteralPath $lapsPath |
            Where-Object { $_.Trim().Length -gt 0 }
        ).Count
    }

    if ($lapCount -ge 2) {
        $reason = "two laps completed"
        break
    }

    $controller = Get-CimInstance Win32_Process |
        Where-Object {
            $_.Name -eq "python.exe" -and
            $_.CommandLine -like "*lmpc_nordschleife_frenet_075_tuned_v2.csv*"
        }
    if ($controller) {
        $controllerSeen = $true
    } elseif ($controllerSeen) {
        $reason = "controller ended before two laps"
        break
    }

    Start-Sleep -Seconds 5
}

$closed = @()
if ($reason -eq "two laps completed") {
    Start-Sleep -Seconds 3
    $closed = @(
        Get-Process -Name acs -ErrorAction SilentlyContinue |
        ForEach-Object {
            Stop-Process -Id $_.Id -Force -ErrorAction SilentlyContinue
            $_.Id
        }
    )
}

$timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
Add-Content -LiteralPath $watchLogPath -Encoding utf8 -Value (
    "$timestamp reason=$reason closed_acs_pids=$($closed -join ',')"
)
