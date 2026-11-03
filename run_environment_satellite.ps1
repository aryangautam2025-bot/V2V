# ==========================================
# CARLA V2V ENVIRONMENT LAUNCHER
# Official manual_control.py drives the car.
# Satellite ADAS dashboard is observation-only.
# ==========================================
$ErrorActionPreference = "Stop"
$project = $PSScriptRoot
$python = Join-Path $project ".venv\Scripts\python.exe"
$examples = Join-Path $project "CARLA_0.9.16\PythonAPI\examples"
$trafficScript = Join-Path $examples "generate_traffic.py"
$manualScript = Join-Path $examples "manual_control.py"
$meshScript = Join-Path $project "v2v_cooperative_adas_satellite.py"

$hostAddress = "127.0.0.1"
$port = 2500
$trafficManagerPort = 8000

if (!(Test-Path $python)) { throw "Python venv not found: $python" }
if (!(Test-Path $trafficScript)) { throw "CARLA generate_traffic.py not found: $trafficScript" }
if (!(Test-Path $manualScript)) { throw "CARLA manual_control.py not found: $manualScript" }
if (!(Test-Path $meshScript)) { throw "Satellite ADAS app not found: $meshScript" }

Write-Host "Start CARLA with run_carla.ps1 first. For a clean test, restart CARLA before spawning traffic." -ForegroundColor Yellow
$numberOfCars = Read-Host "Enter number of NPC vehicles (19 = about 20 total including the manual hero; 49 = about 50)"
[int]$npcCount = 0
if (![int]::TryParse($numberOfCars, [ref]$npcCount) -or $npcCount -lt 0) {
    throw "Enter a non-negative whole number of NPC vehicles."
}
if ($npcCount -gt 49) {
    Write-Host "Limiting NPC count to 49 so the target stays at 50 total vehicles." -ForegroundColor Yellow
    $npcCount = 49
}
$totalTarget = [Math]::Min(50, $npcCount + 1)

Write-Host "[1/4] Starting NPC traffic..." -ForegroundColor Green
$trafficCommand = "& '$python' '$trafficScript' --host '$hostAddress' --port $port --tm-port $trafficManagerPort --number-of-vehicles $npcCount --safe"
Start-Process -FilePath "powershell.exe" -WorkingDirectory $project -ArgumentList @("-NoExit", "-Command", $trafficCommand)
$waitSeconds = if ($npcCount -ge 35) { 12 } elseif ($npcCount -ge 15) { 9 } else { 6 }
Write-Host "Waiting $waitSeconds seconds for NPC traffic to initialize..." -ForegroundColor DarkCyan
Start-Sleep -Seconds $waitSeconds

Write-Host "[2/4] Starting the official CARLA manual driving window..." -ForegroundColor Cyan
Write-Host "This window is for driving only. Controls are the standard CARLA manual_control.py controls." -ForegroundColor Cyan
$manualCommand = "& '$python' '$manualScript' --host '$hostAddress' --port $port --res 1280x720"
Start-Process -FilePath "powershell.exe" -WorkingDirectory $project -ArgumentList @("-NoExit", "-Command", $manualCommand)

Write-Host "[3/4] Waiting for manual_control.py to spawn its HERO vehicle..." -ForegroundColor DarkCyan
$heroProbe = @"
import carla, time, sys
client = carla.Client('$hostAddress', $port)
client.set_timeout(3.0)
deadline = time.time() + 45.0
while time.time() < deadline:
    try:
        vehicles = list(client.get_world().get_actors().filter('vehicle.*'))
        hero = next((v for v in vehicles if v.attributes.get('role_name', '') == 'hero'), None)
        if hero is not None:
            print(f'HERO_READY id={hero.id}', flush=True)
            sys.exit(0)
    except Exception:
        pass
    time.sleep(0.5)
print('HERO_TIMEOUT', flush=True)
sys.exit(2)
"@
$probeOutput = & $python -c $heroProbe
$probeOutput | ForEach-Object { Write-Host $_ }
if ($LASTEXITCODE -ne 0) {
    throw "No role_name=hero vehicle appeared. Check that manual_control.py opened successfully and spawned its player. The mesh was not started."
}

Write-Host "[4/4] Starting observation-only satellite ADAS dashboard ($totalTarget target vehicles)..." -ForegroundColor Yellow
Write-Host "Session reports will be saved to: $project\session_reports\session_<timestamp>" -ForegroundColor Green
Write-Host "DRIVING: use the official manual_control.py Pygame window." -ForegroundColor Green
Write-Host "SATELLITE: observation and vehicle selection only; WASD in this window will NOT drive." -ForegroundColor Cyan
Write-Host "Mouse: left-click a car to inspect; right/middle-drag pan; wheel zoom; Q/E rotate; F follow; M recenter; Home select ego; Esc exits dashboard." -ForegroundColor Cyan
& $python $meshScript --host $hostAddress --port $port --tm-port $trafficManagerPort --max-vehicles $totalTarget --camera-mode satellite --satellite-altitude 90 --fps 60 --report-dir session_reports --report-sample-hz 2.0 --no-manual-control
if ($LASTEXITCODE -ne 0) {
    throw "ADAS dashboard exited with an error. Review the output above. The manual-control and traffic windows may still be open."
}
$reportsRoot = Join-Path $project "session_reports"
if (Test-Path $reportsRoot) {
    $latestReport = Get-ChildItem -Path $reportsRoot -Directory -Filter "session_*" | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($latestReport) {
        Write-Host ""
        Write-Host "SESSION SUMMARY READY" -ForegroundColor Green
        Write-Host $latestReport.FullName -ForegroundColor Cyan
        Write-Host "Open session_summary.md for the human-readable report and vehicle_summary.csv for per-car statistics." -ForegroundColor Green
    }
}
