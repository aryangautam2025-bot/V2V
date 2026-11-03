V2V ADAS — AUTOMATIC PER-SESSION REPORTING
Target: Windows / CARLA 0.9.16 / Python 3.12

WHEN THE REPORT IS GENERATED
The integrated ADAS dashboard creates its final reports when the ADAS satellite window is closed normally (window close button or Esc). Closing only the separate official manual_control.py window does not stop the ADAS recorder. Do not kill the Python process from Task Manager if you want the final summary.

OUTPUT LOCATION
<Project folder>\session_reports\session_YYYYMMDD_HHMMSS\
The launcher prints the latest session folder when the dashboard exits.

OUTPUT FILES
1. session_summary.md — readable overview, totals, per-car table.
2. vehicle_summary.csv — one row for each vehicle sampled in the session.
3. vehicle_samples.csv — timestamped motion/position/control samples for each live CARLA vehicle.
4. visibility_events.csv — timestamped changes in ego-relative LOS/NLOS/CHECK/UNTRACKED.
5. session_summary.json — metadata + per-vehicle summary rows for programmatic analysis.
6. session_metadata.json — settings and explicit definitions for the measurements.

PER-CAR FIELDS
- first and last XYZ position
- time-weighted average speed and maximum speed
- average and maximum acceleration magnitude
- estimated distance travelled from sampled position deltas
- moving, stationary, and braking durations; brake-event count
- ego-relative LOS, NLOS, CHECK, and UNTRACKED durations
- LOS/NLOS percentage of classified time (LOS + NLOS denominator only)
- per-car TX counts, total RX messages seen by that car, and counts of direct messages from the car parsed by ego
- semantic-LiDAR detection-event count and unique vehicle/pedestrian object IDs detected by that vehicle
- average received signal-power value observed at ego for messages from that sender, where available

IMPORTANT INTERPRETATION
Kinematic samples are read directly from CARLA actors as ground truth for simulation evaluation. They are not necessarily data received through V2V. The separate message count fields show communication observations.

LOS/NLOS are ego-relative geometry/sensor-visibility classifications. NLOS needs a fresh valid live target and a recent matching obstructed scene ray; missing detections alone do not imply NLOS. UNTRACKED means outside the adaptive awareness mesh or lacking a fresh ego track. CHECK means the available evidence is insufficient. Durations are sampled estimates. These are not per-packet RF-NLOS classifications.

Distance travelled is an estimate from consecutive sampled positions; long gaps and obviously implausible jumps are excluded. TX counts are successful sensor.send() calls, not proof of delivery. CSV data are flushed periodically but a forced process kill can leave final metadata/summary missing.

SAMPLE RATE
Default is 2 Hz; command-line range is 1–10 Hz. For 50 vehicles, 2 Hz produces about 100 vehicle rows per second if 50 vehicles are in the world, so for long sessions or constrained hardware you can set --report-sample-hz 1.0.

GITHUB PREP
The included .gitignore excludes .venv, CARLA_0.9.16, generated session_reports, logs and transient selection state. Keep run outputs out of the source repository unless you intentionally choose to commit a small anonymized sample.
