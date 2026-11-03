Get-Process CarlaUE4, "CarlaUE4-Win64-Shipping" -ErrorAction SilentlyContinue |
    Stop-Process -Force
# ============================================================
# CARLA 0.9.16 Launcher - V2V Cooperative ADAS
# RPC Port: 2500 | Streaming Port: 2501
# ============================================================

$ErrorActionPreference = "Stop"

# Project directory (folder containing this script)
$ProjectRoot = $PSScriptRoot

# Look for CARLA in common folder layouts
$Candidates = @(
    (Join-Path $ProjectRoot "CARLA_0.9.16\CarlaUE4.exe"),
    (Join-Path $ProjectRoot "CARLA_0.9.16\WindowsNoEditor\CarlaUE4.exe"),
    (Join-Path $ProjectRoot "CarlaUE4.exe")
)

$CarlaExe = $Candidates |
    Where-Object { Test-Path $_ } |
    Select-Object -First 1

# Stop if the executable cannot be found
if (-not $CarlaExe) {
    Write-Host "`nERROR: Could not find CarlaUE4.exe." -ForegroundColor Red
    Write-Host "Check the CARLA_0.9.16 folder inside:"
    Write-Host $ProjectRoot
    Write-Host "`nExpected one of these locations:"
    $Candidates | ForEach-Object { Write-Host "  $_" }
    Read-Host "`nPress Enter to exit"
    exit 1
}

$CarlaDirectory = Split-Path -Parent $CarlaExe

Write-Host "`n============================================" -ForegroundColor Cyan
Write-Host "   Starting CARLA 0.9.16 - V2V ADAS" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "Executable : $CarlaExe"
Write-Host "RPC Port   : 2500"
Write-Host "Stream Port: 2501"
Write-Host "Graphics   : Low"
Write-Host "`nLaunching CARLA..." -ForegroundColor Yellow

try {
    Start-Process `
        -FilePath $CarlaExe `
        -WorkingDirectory $CarlaDirectory `
        -ArgumentList @(
            "-quality-level=Low",
            "-carla-rpc-port=2500",
            "-carla-streaming-port=2501"
        )

    Write-Host "`nCARLA launch command sent successfully." -ForegroundColor Green
    Write-Host "Wait for the map to finish loading before starting your V2V app."
}
catch {
    Write-Host "`nFailed to launch CARLA: $_" -ForegroundColor Red
    exit 1
}