# V2V Cooperative ADAS — CARLA Satellite Mesh

A Windows/Python research prototype for a CARLA 0.9.16 cooperative traffic-safety experiment.

The application combines an ego-centred 90-degree satellite view, a live vehicle mesh, V2X state/object messages, per-vehicle telemetry, and automatic end-of-session reports.

> **Research prototype only.** The LOS/NLOS labels are ego-relative sensor/geometry classifications. They are not definitive per-packet radio NLOS labels or a validated vehicle safety system.

---

## Contents

1. [What the project does](#what-the-project-does)
2. [How the simulation is launched](#how-the-simulation-is-launched)
3. [Requirements](#requirements)
4. [Download and prepare the project](#download-and-prepare-the-project)
5. [Install CARLA 0.9.16](#install-carla-0916)
6. [Install Python 3.12 and create the virtual environment](#install-python-312-and-create-the-virtual-environment)
7. [Make the project work from any folder](#make-the-project-work-from-any-folder)
8. [Install the Python packages and CARLA API](#install-the-python-packages-and-carla-api)
9. [Verify the installation](#verify-the-installation)
10. [Run the complete simulation](#run-the-complete-simulation)
11. [Session reports and metric interpretation](#session-reports-and-metric-interpretation)
12. [NLOS diagnostic](#nlos-diagnostic)
13. [Troubleshooting](#troubleshooting)
14. [Project files and repository hygiene](#project-files-and-repository-hygiene)

---

## What the project does

### Current features

- Connects to an existing CARLA world; the ADAS dashboard does **not** spawn the traffic fleet or call `world.tick()`.
- Uses CARLA's road map and lane geometry for the bird's-eye mesh.
- Configures custom V2X channel attributes and reports whether the expected radio configuration was accepted.
- Sends compact vehicle-state messages and detected-object/hazard reports between connected vehicles.
- Uses semantic LiDAR on the instrumented fleet to report visible vehicle/pedestrian actors, subject to the configured sensor field of view/range and CARLA rendering/simulation conditions.
- Separates ego-relative `LOS`, `NLOS`, `CHECK` and `UNTRACKED` states. Missing detections alone must not be treated as NLOS.
- Supports selectable vehicles and remote telemetry in the dashboard.
- Automatically exports per-vehicle statistics and time-series data when the ADAS dashboard closes normally.

### Important simulation behaviour

The integrated workflow uses three distinct parts:

1. **CARLA server:** the simulator application creates and hosts the world.
2. **Environment setup:** the environment launcher starts traffic generation and the official CARLA manual-control client.
3. **ADAS dashboard:** the Python application attaches to the running world and observes the traffic, V2X messages and sensor data.

Drive the ego vehicle in the official `manual_control.py` Pygame window. The satellite ADAS window is for observation, selection, telemetry and LOS/NLOS display.

---

## How the simulation is launched

The intended run order is:

```text
PowerShell 1
  run_carla.ps1
      |
      v
CARLA 0.9.16 server
      |
      |  Wait until the simulator has finished loading
      v
PowerShell 2
  run_environment_satellite.ps1
      |
      +--> generate_traffic.py
      |
      +--> manual_control.py (official CARLA driving window)
      |
      +--> v2v_cooperative_adas_satellite.py (ADAS dashboard)
                    |
                    v
             session_reports/
```

The ADAS dashboard is an observer/client of the CARLA world. Do not launch an additional old V2X dashboard or `v2v_mesh_test.py` alongside the integrated application; duplicate applications can attach duplicate sensors and add simulation load.

`run_satellite_dashboard.ps1` is also included as a dashboard launcher/utility. For the standard integrated workflow, use the sequence documented in [Run the complete simulation](#run-the-complete-simulation). Do not start a second dashboard if the environment launcher already started one.

---

## Requirements

| Component | Requirement |
|---|---|
| Operating system | Windows 10 or Windows 11, 64-bit |
| Simulator | CARLA 0.9.16 |
| Python | Python 3.12, 64-bit (the development environment used 3.12.10) |
| Python environment | `venv` |
| CARLA API | CARLA 0.9.16 client built for the same Python minor version and Windows architecture |
| GPU | A dedicated GPU is recommended |
| Disk space | Allow at least approximately 20 GB for the CARLA package, plus project files and Python dependencies |
| Network ports | CARLA's configured RPC and streaming ports must be available locally |

CARLA 0.9.16's release notes list Python 3.10, 3.11 and 3.12 support. Use a **64-bit Python 3.12** interpreter for the environment described here. The CARLA client wheel must match both the CARLA server release and the Python interpreter. See the [CARLA 0.9.16 quickstart](https://carla.readthedocs.io/en/0.9.16/start_quickstart/) and [0.9.16 release notes](https://carla.org/2025/09/16/release-0.9.16/).

---

## Download and prepare the project

### Option A — Download ZIP

1. Open this GitHub repository.
2. Select **Code → Download ZIP**.
3. Extract the ZIP to a location with sufficient free disk space.
4. Open PowerShell in the extracted project root.

### Option B — Clone with Git

```powershell
git clone YOUR_REPOSITORY_URL
cd YOUR_REPOSITORY_FOLDER
```

Replace `YOUR_REPOSITORY_URL` with the URL of this repository and `YOUR_REPOSITORY_FOLDER` with the folder created by Git.

### Confirm the project root

The repository root is the directory containing files such as:

```text
check_v2x_blueprint.py
requirements.txt
run_carla.ps1
run_environment_satellite.ps1
run_satellite_dashboard.ps1
v2v_cooperative_adas_satellite.py
view_session_reports.py
```

Run:

```powershell
Get-Location
Get-ChildItem
```

Keep the repository's relative folder structure intact. Do not move individual scripts into `.venv`.

---

## Install CARLA 0.9.16

1. Download the **Windows packaged release** of CARLA 0.9.16 from the [CARLA releases page](https://github.com/carla-simulator/carla/releases).
2. Extract the package.
3. Place the extracted CARLA directory in the project root, or record its actual location for the optional `CARLA_ROOT` configuration described below.
4. For the default layout in this guide, name the directory `CARLA_0.9.16`.

The expected layout is:

```text
YOUR_PROJECT/
├── CARLA_0.9.16/
│   ├── CarlaUE4.exe
│   └── PythonAPI/
│       ├── carla/
│       │   └── dist/
│       └── examples/
│           ├── generate_traffic.py
│           └── manual_control.py
├── check_v2x_blueprint.py
├── requirements.txt
├── run_carla.ps1
├── run_environment_satellite.ps1
├── run_satellite_dashboard.ps1
├── v2v_cooperative_adas_satellite.py
└── view_session_reports.py
```

The CARLA distribution must contain both:

- `PythonAPI\examples\generate_traffic.py`
- `PythonAPI\examples\manual_control.py`

Check that the paths exist:

```powershell
Test-Path ".\CARLA_0.9.16\CarlaUE4.exe"
Test-Path ".\CARLA_0.9.16\PythonAPI\examples\generate_traffic.py"
Test-Path ".\CARLA_0.9.16\PythonAPI\examples\manual_control.py"
```

All three commands should return `True`. If not, correct the CARLA folder location or name before continuing.

---

## Install Python 3.12 and create the virtual environment

### 1. Install Python

Download the 64-bit Windows installer for Python 3.12.10:

https://www.python.org/downloads/release/python-31210/

During installation, enable **Add Python to PATH**. Complete the installation.

Verify:

```powershell
py -3.12 --version
```

Expected:

```text
Python 3.12.10
```

If the Python launcher cannot find Python 3.12, install Python 3.12 x64 or correct the installation before continuing.

### 2. Create `.venv`

Run this from the repository root:

```powershell
py -3.12 -m venv .venv
```

This creates a project-local Python virtual environment. It does not contain the project's source files; it contains its own Python interpreter and installed packages.

### 3. Activate `.venv`

```powershell
.\.venv\Scripts\Activate.ps1
```

If PowerShell blocks activation scripts, allow activation for the current PowerShell process only:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
```

The prompt should start with `(.venv)`.

Verify that the correct Python interpreter is active:

```powershell
python --version
where.exe python
```

The first Python path returned by `where.exe python` should be:

```text
YOUR_PROJECT_PATH\.venv\Scripts\python.exe
```

You can also run the environment's interpreter directly without activation:

```powershell
.\.venv\Scripts\python.exe --version
```

---

## Make the project work from any folder

### Why this matters

Do not assume that the project will be installed under the same Windows user account or directory as the developer's computer. For example, another user might extract it to:

```text
C:\Users\Alex\Desktop\V2V
```

or:

```text
D:\Engineering\Projects\V2V
```

Code that contains a developer-specific path such as:

```text
C:\Users\Developer\OneDrive\Desktop\PROJECTS\V2V
```

will fail on another computer.

The project should resolve its paths from the location of the repository, not from a particular Windows username or Desktop location.

### 1. Audit source files for machine-specific paths

From the repository root, run:

```powershell
$sourceFiles = Get-ChildItem -Path . -Recurse -File -Include *.py,*.ps1,*.json,*.txt |
    Where-Object {
        $_.FullName -notmatch '\\(\.venv|CARLA_0\.9\.16|\.git)\\'
    }

$sourceFiles | Select-String -Pattern '(?i)[A-Z]:\\|OneDrive|Desktop|AppData|C:\\Users\\'
```

Review every match. A match may be a comment, example or documentation string rather than an active path, so do not replace every match blindly.

Also search for hardcoded CARLA folders, Python executables, report directories and ports:

```powershell
$sourceFiles | Select-String -Pattern '(?i)CARLA_0\.9\.16|CarlaUE4\.exe|generate_traffic\.py|manual_control\.py|python\.exe|session_reports|localhost|127\.0\.0\.1|2000|2001|2500'
```

Check that every script is using the intended project paths and connection settings.

### 2. Portable paths in Python

For a Python file located directly in the project root, use a project-root-relative pattern like this:

```python
from pathlib import Path
import os

PROJECT_ROOT = Path(__file__).resolve().parent

CARLA_ROOT = Path(
    os.environ.get(
        "CARLA_ROOT",
        str(PROJECT_ROOT / "CARLA_0.9.16")
    )
).resolve()

REPORTS_DIR = PROJECT_ROOT / "session_reports"
REPORTS_DIR.mkdir(parents=True, exist_ok=True)

CARLA_EXAMPLES = CARLA_ROOT / "PythonAPI" / "examples"
```

For a Python file located one folder below the project root, calculate the root one level higher. For example:

```python
PROJECT_ROOT = Path(__file__).resolve().parents[1]
```

Choose the correct parent depth for the actual location of each script. Do not copy the same root expression into every file without checking its location.

### 3. Portable paths in PowerShell

In a `.ps1` file, use `$PSScriptRoot` to locate the directory containing the launcher:

```powershell
$ProjectRoot = $PSScriptRoot

$CarlaRoot = if ($env:CARLA_ROOT) {
    $env:CARLA_ROOT
} else {
    Join-Path $ProjectRoot "CARLA_0.9.16"
}

$CarlaExe = Join-Path $CarlaRoot "CarlaUE4.exe"
$PythonExe = Join-Path $ProjectRoot ".venv\Scripts\python.exe"
$CarlaExamples = Join-Path $CarlaRoot "PythonAPI\examples"
```

Validate important paths before launching:

```powershell
if (-not (Test-Path $CarlaExe)) {
    throw "CARLA executable not found: $CarlaExe"
}

if (-not (Test-Path $PythonExe)) {
    throw "Project virtual environment not found. Create .venv first."
}

if (-not (Test-Path (Join-Path $CarlaExamples "generate_traffic.py"))) {
    throw "CARLA generate_traffic.py was not found."
}

if (-not (Test-Path (Join-Path $CarlaExamples "manual_control.py"))) {
    throw "CARLA manual_control.py was not found."
}
```

**Important:** These are the path-handling patterns that the launchers and Python scripts should use. Adding these examples to a README does not automatically modify existing scripts. If the audit in Step 1 finds active developer-specific paths in a script, that script must be updated to use the appropriate project-relative pattern before the project will run from a different directory.

### 4. Optional: keep CARLA outside the repository

The default setup expects `CARLA_0.9.16` inside the project root. If CARLA is stored elsewhere, the scripts must support the `CARLA_ROOT` environment variable shown above.

For the current PowerShell session:

```powershell
$env:CARLA_ROOT = "D:\Simulators\CARLA_0.9.16"
```

Use the actual folder containing `CarlaUE4.exe`. This setting only works if the project scripts read `CARLA_ROOT`; otherwise, keep CARLA in the default project-relative location.

---

## Install the Python packages and CARLA API

### 1. Install the project requirements

Make sure `.venv` is active. From the project root:

```powershell
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

The development environment was recorded with these package versions:

| Package | Version |
|---|---:|
| carla | 0.9.16 |
| colorama | 0.4.6 |
| contourpy | 1.4.0 |
| cycler | 0.12.1 |
| fonttools | 4.65.0 |
| kiwisolver | 1.5.1 |
| matplotlib | 3.11.2 |
| numpy | 2.5.3 |
| packaging | 26.3 |
| pandas | 3.0.6 |
| pillow | 12.3.0 |
| pygame | 2.6.1 |
| pyparsing | 3.3.3 |
| pyqtgraph | 0.14.0 |
| PySide6 | 6.11.2 |
| PySide6_Addons | 6.11.2 |
| PySide6_Essentials | 6.11.2 |
| python-dateutil | 2.9.0.post0 |
| scipy | 1.18.1 |
| shiboken6 | 6.11.2 |
| six | 1.17.0 |
| tzdata | 2026.4 |

The repository's `requirements.txt` is the installation source of truth. If package versions are changed, update and test that file before publishing the change.

### 2. Install the CARLA 0.9.16 Python API

The CARLA simulator and its Python client API are separate components. The Python API version must match the CARLA server release.

First inspect the wheel included with the CARLA installation:

```powershell
Get-ChildItem ".\CARLA_0.9.16\PythonAPI\carla\dist\*.whl"
```

For Python 3.12 on 64-bit Windows, the wheel filename should include tags similar to:

```text
carla-0.9.16-cp312-cp312-win_amd64.whl
```

Install the matching wheel into the active `.venv`:

```powershell
$wheel = Get-ChildItem ".\CARLA_0.9.16\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl" |
    Select-Object -First 1

if (-not $wheel) {
    throw "A CARLA 0.9.16 wheel for CPython 3.12 / Windows x64 was not found."
}

python -m pip install $wheel.FullName
```

If the packaged CARLA folder does not contain a matching wheel, check the official CARLA 0.9.16 release/package and its Python API distribution. Do not install a wheel for a different Python minor version or a different platform. CARLA 0.9.16 publishes a Python 3.12 Windows x64 wheel; use the package-matched wheel where possible.

**Do not assume that a CARLA wheel for Python 3.10 or 3.11 will work in Python 3.12.** The `cp312` tag identifies a CPython 3.12 wheel.

Official references:

- [CARLA 0.9.16 quickstart](https://carla.readthedocs.io/en/0.9.16/start_quickstart/)
- [CARLA 0.9.16 release information](https://carla.org/2025/09/16/release-0.9.16/)
- [CARLA 0.9.16 Python package](https://pypi.org/project/carla/0.9.16/)

### 3. Install CARLA example-script dependencies if needed

The integrated environment launcher invokes CARLA's `generate_traffic.py` and `manual_control.py` examples. If those scripts report missing imports, inspect the example requirements file:

```powershell
Test-Path ".\CARLA_0.9.16\PythonAPI\examples\requirements.txt"
```

If it exists, install its dependencies into the same active virtual environment:

```powershell
python -m pip install -r ".\CARLA_0.9.16\PythonAPI\examples\requirements.txt"
```

Do not create a second Python environment for the CARLA examples; they should use the same `.venv` as the V2V application.

---

## Verify the installation

Activate `.venv` and run the following from the repository root:

### Check Python

```powershell
python --version
where.exe python
```

### Check the CARLA API and important libraries

```powershell
python -c "import carla; print('CARLA API import successful')"
python -c "import numpy, pygame; print('NumPy and Pygame import successful')"
```

The dashboard also uses the dependencies listed in `requirements.txt`. Verify all of them with:

```powershell
python -m pip check
python -m pip list
```

`pip check` should report no broken requirements.

### Check the CARLA files

```powershell
Test-Path ".\CARLA_0.9.16\CarlaUE4.exe"
Test-Path ".\CARLA_0.9.16\PythonAPI\examples\generate_traffic.py"
Test-Path ".\CARLA_0.9.16\PythonAPI\examples\manual_control.py"
```

All should return `True`.

---

## Run the complete simulation

### Step 1 — Open PowerShell 1 and start CARLA

Open PowerShell in the project root. Activate the environment if the launcher uses the project's Python interpreter:

```powershell
.\.venv\Scripts\Activate.ps1
```

Start the CARLA server:

```powershell
.\run_carla.ps1
```

Wait for the CARLA simulator window to load completely before continuing.

The project documentation uses the CARLA RPC port configured by `run_carla.ps1`. Confirm that the port used by the launchers and Python client is consistent. The default CARLA RPC port is commonly `2000`, but this project may configure a different port.

### Step 2 — Open PowerShell 2 and start the environment

Open a second PowerShell window in the same project root.

Activate the same environment:

```powershell
.\.venv\Scripts\Activate.ps1
```

Run:

```powershell
.\run_environment_satellite.ps1
```

When prompted, enter the number of NPC vehicles:

| Approximate total vehicles | NPC vehicles to enter |
|---:|---:|
| 20 (one manual hero + NPCs) | 19 |
| 50 (one manual hero + NPCs) | 49 |

The actual number may be lower if CARLA does not have enough valid spawn points.

The environment launcher is intended to start the traffic setup, the official CARLA manual-control window and the integrated satellite ADAS dashboard.

### Step 3 — Drive the ego vehicle

Drive in the **official `manual_control.py` Pygame window**.

Use the satellite ADAS window for:

- Ego-centred satellite view.
- Vehicle mesh and vehicle selection.
- V2X state/object message observation.
- Remote telemetry.
- LOS/NLOS/CHECK/UNTRACKED visibility display.

The dashboard observes the CARLA world; it does not replace the official manual-control window.

### Step 4 — End the session and export reports

Close the ADAS dashboard using its window close button or `Esc` so that it can finalize the session reports.

Closing only the manual-control window does not stop the dashboard recorder.

Do not forcibly terminate the dashboard process if you need the final summary files. Sample CSV files are flushed periodically, but an abrupt process termination may leave incomplete data and no final session summary.

### Clean test recommendation

For a clean test, restart CARLA before spawning traffic. Do not run old V2X dashboards or `v2v_mesh_test.py` alongside the integrated app.

---

## Session reports and metric interpretation

When the dashboard closes normally, it creates a time-stamped directory, for example:

```text
session_reports/
└── session_20261019_170000/
    ├── session_summary.md
    ├── vehicle_summary.csv
    ├── vehicle_samples.csv
    ├── visibility_events.csv
    ├── session_summary.json
    └── session_metadata.json
```

The timestamp above is an example; the actual folder name is generated at runtime.

### Report files

| File | Contents |
|---|---|
| `session_summary.md` | Human-readable session overview and one-row-per-vehicle summary |
| `vehicle_summary.csv` | Per-vehicle average/max speed, estimated distance, start/end XYZ, acceleration, moving/stopped/braking time, brake events, visibility duration and TX/RX/sensor-detection counts |
| `vehicle_samples.csv` | Timestamped positions, velocity, acceleration, heading, controls, visibility state and whether the vehicle was instrumented for V2X |
| `visibility_events.csv` | Timestamped changes between visibility states |
| `session_summary.json` | Machine-readable metadata and summary rows |
| `session_metadata.json` | Run settings, start/end times, totals and metric definitions |

### Interpret the metrics correctly

- **Kinematics:** Position, velocity, acceleration and controls in the report are sampled directly from CARLA simulator actors for ground-truth evaluation. They are not all necessarily sent to ego over V2V. Check the V2X columns to see which messages the ego received.
- **LOS:** A fresh ego semantic-LiDAR detection or a recent unblocked geometry check confirms the target is visible to ego under this simulation setup.
- **NLOS:** A fresh valid target track plus a recent matching blocked scene-ray check. Absence of a LiDAR hit alone is not enough.
- **CHECK:** A fresh track exists but visibility could not be confirmed reliably.
- **UNTRACKED:** The car is outside the current adaptive awareness mesh, or there is no fresh valid state/object track. It is **not** automatically NLOS.
- **LOS/NLOS percentages:** Use only the sampled `LOS + NLOS` duration as their denominator. `CHECK` and `UNTRACKED` are excluded.
- **Distance travelled:** Estimated by summing consecutive sampled positions. Long gaps and obvious teleport-sized jumps are excluded, so it may differ from the exact driven path.
- **TX counts:** Successful Python `sensor.send()` calls, not guarantees of delivery.
- **RX at ego:** Parsed messages observed by the ego receiver, grouped by direct radio sender.
- **RF NLOS:** `RF PATH-LOSS: CONFIGURED` means CARLA accepted configured radio attributes. It does not prove that a particular message followed an RF-NLOS path.
- **Time values:** LOS/NLOS/other durations are sampled approximations; they are not certified continuous-time measurements.

Use `view_session_reports.py` to inspect generated reports, according to its command-line options/help:

```powershell
python .\view_session_reports.py --help
```

---

## NLOS diagnostic

With CARLA running, activate `.venv` and run:

```powershell
.\.venv\Scripts\python.exe .\check_v2x_blueprint.py --host 127.0.0.1 --port 2500
```

The diagnostic reports requested V2X blueprint attributes and whether this CARLA build accepted them.

**Port note:** The example diagnostic command above uses port `2500` because that is the port specified in the project's existing diagnostic instructions. The CARLA server RPC port and V2X sensor/communication port are not necessarily the same. Check the launcher and Python source configuration before changing either value. Do not assume that `2500` is the CARLA RPC port.

The receiver API does not provide a definitive per-packet radio NLOS flag, so visual `EGO-NLOS` and configured RF path-loss are deliberately shown separately.

---

## Troubleshooting

### `ModuleNotFoundError: No module named 'carla'`

1. Confirm `.venv` is active.
2. Run `python --version` and `where.exe python`.
3. Confirm the interpreter is `.venv\Scripts\python.exe`.
4. Install the CARLA 0.9.16 wheel that matches Python 3.12 and Windows x64.
5. Run `python -c "import carla"` again.

### CARLA wheel is not compatible

A wheel tag such as `cp312-cp312-win_amd64` is for CPython 3.12 on 64-bit Windows. A `cp310` wheel will not install into Python 3.12. Install the correct wheel or use a Python version supported by the wheel and the project.

### `ModuleNotFoundError` for NumPy, Pygame or another package

Activate `.venv` and run:

```powershell
python -m pip install -r requirements.txt
```

If a CARLA example script reports a missing dependency, install the CARLA examples' `requirements.txt` into the same `.venv`, if that file is present.

### CARLA connection timeout

- Confirm the CARLA window has loaded completely.
- Confirm the host and port in the Python application match the CARLA server configuration.
- Check the configured port with:

```powershell
Test-NetConnection 127.0.0.1 -Port 2000
```

Replace `2000` with the actual RPC port configured by the project if different. A failed test means the server is not listening at that host/port yet, or the connection is blocked.

### `CarlaUE4.exe` or CARLA example scripts cannot be found

Check the three paths:

```powershell
Test-Path ".\CARLA_0.9.16\CarlaUE4.exe"
Test-Path ".\CARLA_0.9.16\PythonAPI\examples\generate_traffic.py"
Test-Path ".\CARLA_0.9.16\PythonAPI\examples\manual_control.py"
```

If any returns `False`, the CARLA folder may be nested one directory deeper after extraction, or it may have a different name. Correct the folder layout or configure `CARLA_ROOT` if the scripts support it.

### PowerShell blocks script activation

Run this in the current PowerShell window:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

Then activate `.venv` again.

### The simulation works on one computer but not another

Run the path audit in [Make the project work from any folder](#make-the-project-work-from-any-folder). Inspect active path references in all Python and PowerShell files. Replace developer-specific absolute paths with project-relative paths. Also verify the CARLA folder location, Python wheel compatibility, and port configuration.

---

## Project files and repository hygiene

The repository's main files include:

| File | Role |
|---|---|
| `run_carla.ps1` | Launches the CARLA simulator |
| `run_environment_satellite.ps1` | Starts the traffic/environment and integrated observation workflow |
| `run_satellite_dashboard.ps1` | Dashboard launcher/utility |
| `v2v_cooperative_adas_satellite.py` | Integrated V2V Cooperative ADAS satellite application |
| `check_v2x_blueprint.py` | V2X blueprint/radio-attribute diagnostic |
| `view_session_reports.py` | Report inspection utility |
| `requirements.txt` | Python dependency pins |

### What belongs in the virtual environment?

The `.venv` directory contains the Python interpreter and installed third-party packages. It is not a folder into which users should copy the project scripts.

Keep source files in the repository. Run them with the `.venv` interpreter:

```powershell
.\.venv\Scripts\python.exe .\v2v_cooperative_adas_satellite.py
```

The standard launcher workflow is still preferred because it starts the CARLA server and traffic/control components in the correct order.

### What should be committed to GitHub?

Normally commit:

- Python source files.
- PowerShell launchers.
- `requirements.txt`.
- README and project documentation.
- Small diagnostic/configuration files.

Normally exclude:

- `.venv/` — recreated by each user.
- `CARLA_0.9.16/` — large simulator distribution installed separately.
- `session_reports/` — generated output from each run.
- Temporary caches, logs and editor state unless the project specifically needs them.

The repository's `.gitignore` should reflect this policy. The CARLA distribution and virtual environment are not required to be stored inside the Git repository for this documented setup.

---

## First-run checklist

Before launching the simulation, confirm each item:

- [ ] Windows 10/11 64-bit.
- [ ] Python 3.12 x64 installed.
- [ ] Repository downloaded and extracted without changing its internal file structure.
- [ ] CARLA 0.9.16 extracted and its folder location confirmed.
- [ ] `CarlaUE4.exe`, `generate_traffic.py` and `manual_control.py` exist.
- [ ] `.venv` created and activated.
- [ ] `requirements.txt` installed successfully.
- [ ] Matching CARLA 0.9.16 Python API installed.
- [ ] No active developer-specific absolute paths remain in the scripts.
- [ ] CARLA host/port settings are consistent.
- [ ] `run_carla.ps1` starts the simulator.
- [ ] `run_environment_satellite.ps1` starts the environment and integrated ADAS workflow.
- [ ] Dashboard closes normally and produces session reports.

---

**Project:** V2V Cooperative ADAS — CARLA Satellite Mesh  
**Simulator:** CARLA 0.9.16  
**Python:** 3.12.10 (64-bit)  
**Platform:** Windows 10 / 11  
**Purpose:** Research prototype for cooperative traffic-safety experimentation
