param(
    [string]$DllPath = "$PSScriptRoot\Nefarius.ViGEm.Client.dll"
)

$ErrorActionPreference = "Stop"
Add-Type -Path $DllPath

$client = [Nefarius.ViGEm.Client.ViGEmClient]::new()
$pad = $client.CreateXbox360Controller()
$pad.AutoSubmitReport = $false

try {
    $pad.Connect()
    [Console]::Out.WriteLine("READY")
    [Console]::Out.Flush()

    while ($true) {
        $line = [Console]::In.ReadLine()
        if ($null -eq $line -or $line -eq "QUIT") {
            break
        }

        $parts = $line.Split(",")
        if ($parts.Length -lt 3) {
            continue
        }

        $steer = [int16]$parts[0]
        $throttle = [byte]$parts[1]
        $brake = [byte]$parts[2]
        $buttons = if ($parts.Length -ge 4) { [uint16]$parts[3] } else { [uint16]0 }

        $pad.SetButtonsFull($buttons)
        $pad.SetAxisValue(
            [Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Axis]::LeftThumbX,
            $steer
        )
        $pad.SetSliderValue(
            [Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Slider]::RightTrigger,
            $throttle
        )
        $pad.SetSliderValue(
            [Nefarius.ViGEm.Client.Targets.Xbox360.Xbox360Slider]::LeftTrigger,
            $brake
        )
        $pad.SubmitReport()
    }
}
finally {
    try {
        $pad.ResetReport()
        $pad.SubmitReport()
        Start-Sleep -Milliseconds 250
    }
    catch {
    }
    try {
        $pad.Disconnect()
    }
    catch {
    }
    $client.Dispose()
}
