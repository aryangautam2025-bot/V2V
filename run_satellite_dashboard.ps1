$ErrorActionPreference = "Stop"
$project = $PSScriptRoot
$python = Join-Path $project ".venv\Scripts\python.exe"
$app = Join-Path $project "v2v_cooperative_adas_satellite.py"
if (!(Test-Path $python)) { throw "Python venv not found: $python" }
if (!(Test-Path $app)) { throw "Satellite ADAS app not found: $app" }
& $python $app --host 127.0.0.1 --port 2500 --max-vehicles 20 --camera-mode satellite --satellite-altitude 90 --fps 60
if ($LASTEXITCODE -ne 0) { throw "Satellite ADAS dashboard exited with an error." }
