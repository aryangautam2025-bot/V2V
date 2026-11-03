# V2V Cooperative ADAS — CARLA Satellite Mesh

A Windows/Python prototype for a CARLA 0.9.16 cooperative traffic-safety experiment. The application combines an ego-centred 90-degree satellite view, a live vehicle mesh, V2X state/object messages, per-vehicle telemetry, and an automatic end-of-session report.

> **Research prototype only.** The LOS/NLOS labels are ego-relative sensor/geometry classifications. They are not definitive per-packet radio NLOS labels or a validated vehicle safety system.

## Current features

- Connects to an existing CARLA world; it does not spawn the traffic fleet or call `world.tick()`.
- Uses CARLA's road map and lane geometry for the bird's-eye mesh.
- Configures custom V2X channel attributes and reports whether the expected radio configuration was accepted.
- Sends compact vehicle-state messages and detected-object/hazard reports between connected vehicles.
- Uses semantic LiDAR on the instrumented fleet to report visible vehicle/pedestrian actors, subject to the configured sensor field of view/range and CARLA rendering/simulation conditions.
- Separates ego-relative `LOS`, `NLOS`, `CHECK` and `UNTRACKED` states; missing detections alone must not be treated as NLOS.
- Supports selectable vehicles and remote telemetry in the dashboard.
- Automatically exports per-vehicle statistics and time-series data when the ADAS dashboard closes normally.

## Requirements

- Windows and CARLA 0.9.16 installed under `CARLA_0.9.16/`.
- Python virtual environment `.venv/` with the CARLA 0.9.16 Python API, `numpy` and `pygame`.
- `CARLA_0.9.16/PythonAPI/examples/generate_traffic.py` and `manual_control.py`.

The CARLA distribution and virtual environment are not included in this repository/package. Use the Python API that matches the installed CARLA server version.

## Run

1. Open PowerShell in the project directory.
2. Start CARLA in PowerShell 1:

   ```powershell
   .\run_carla.ps1
   ```

3. In PowerShell 2, start traffic, the official CARLA manual-control window, and the observation-only ADAS dashboard:

   ```powershell
   .\run_environment_satellite.ps1
   ```

4. Enter the number of NPC vehicles when prompted. For a target of about 20 vehicles total (one manual hero + NPCs), enter `19`. For about 50 total, enter `49`. Spawning may fail if CARLA has too few valid spawn points.
5. Drive in the **official `manual_control.py` Pygame window**. Use the satellite ADAS window for observation, selection, telemetry, and LOS/NLOS display.
6. Close the ADAS dashboard with its window close button or `Esc` to finalize the session reports. Closing only the manual-control window does not stop the dashboard recorder.

For a clean test, restart CARLA before spawning traffic. Do not run old V2X dashboards or `v2v_mesh_test.py` alongside the integrated app; multiple copies attach duplicate sensors and add load.

## Session reports

The dashboard records data while it is running. On normal exit it creates a folder such as:

```text
session_reports/session_20261019_170000/
```

It writes:

- `session_summary.md` — human-readable session overview and one-row-per-vehicle summary.
- `vehicle_summary.csv` — one row per sampled car: average/max speed, estimated distance, start/end XYZ, acceleration, moving/stopped/braking time, brake events, LOS/NLOS/CHECK/UNTRACKED time, and TX/RX/sensor-detection counts.
- `vehicle_samples.csv` — timestamped positions, velocity, acceleration, heading, controls, visibility state, and whether that car was instrumented for V2X.
- `visibility_events.csv` — timestamped changes between visibility states.
- `session_summary.json` — machine-readable metadata and summary rows.
- `session_metadata.json` — run settings, start/end times, totals, and metric definitions.

### Interpret the metrics correctly

- **Kinematics:** position, velocity, acceleration and controls in the report are sampled directly from CARLA simulator actors for ground-truth evaluation. They are not all necessarily sent to ego over V2V. Check the V2X columns to see which messages the ego received.
- **LOS:** a fresh ego semantic-LiDAR detection or a recent unblocked geometry check confirms the target is visible to ego under this simulation setup.
- **NLOS:** a fresh valid target track plus a recent matching blocked scene-ray check. Absence of a LiDAR hit alone is not enough.
- **CHECK:** a fresh track exists but visibility could not be confirmed reliably.
- **UNTRACKED:** the car is outside the current adaptive awareness mesh, or there is no fresh valid state/object track. It is **not** automatically NLOS.
- **LOS/NLOS percentages:** use only the sampled `LOS + NLOS` duration as their denominator; `CHECK` and `UNTRACKED` are excluded.
- **Distance travelled:** estimated by summing consecutive sampled positions. Long gaps and obvious teleport-sized jumps are excluded, so it may differ from the exact driven path.
- **TX counts:** successful Python `sensor.send()` calls, not guarantees of delivery.
- **RX at ego:** parsed messages observed by the ego receiver, grouped by direct radio sender.
- **RF NLOS:** `RF PATH-LOSS: CONFIGURED` means CARLA accepted configured radio attributes. It does not prove that a particular message followed an RF-NLOS path.
- **Time values:** LOS/NLOS/other durations are sampled approximations; they are not a certified continuous-time measurement.

Sample CSVs are flushed periodically. Exit the dashboard normally with `Esc` or the close button; forcibly terminating the Python process may leave only partial files and no final summary.

## NLOS diagnostic

With CARLA running, use:

```powershell
.\.venv\Scripts\python.exe .\check_v2x_blueprint.py --host 127.0.0.1 --port 2500
```

The dashboard logs each requested V2X blueprint attribute and whether this CARLA build accepted it. The receiver API does not provide a definitive per-packet radio NLOS flag, so visual `EGO-NLOS` and configured RF path-loss are deliberately shown separately.

## Repository hygiene

`session_reports/`, `.venv/` and the CARLA distribution are excluded by `.gitignore`. Commit source code, documentation, launch scripts and small diagnostic tools, but not simulator binaries, virtual environments, or generated run output by default.
