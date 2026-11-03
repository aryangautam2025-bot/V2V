#!/usr/bin/env python3
"""CARLA V2V Cooperative ADAS: 90-degree satellite camera + cooperative mesh.

CARLA 0.9.16 / Python 3.12
- Connects to the existing world; does not spawn vehicles or call world.tick().
- Uses a free, unattached RGB camera at a 90-degree top-down angle, shown in the mesh UI.
- Mirrors that camera's transform to CARLA's native spectator; pan/zoom/rotate are independent.
- Offers follow-ego mode plus free-camera mode with right-drag pan, wheel zoom, Q/E rotate.
- Uses native CARLA custom V2X and fleet-scaled semantic LiDAR.
- Rejects stale or non-live tracks before plotting/classifying them.
- EGO-NLOS requires a fresh valid target, a matching recent ray check, and an obstructed scene ray.
- Keeps RF path-loss configuration distinct from ego-relative geometric LOS/NLOS.
- Caches lane rendering and throttles slow panels to preserve UI responsiveness.

NLOS meanings:
  EGO-LOS / EGO-NLOS describe ego-relative geometry/sensor visibility, not RF packet flags.
  A missing LiDAR hit never independently proves NLOS. When track/ray evidence is stale,
  inconsistent or missing, the UI shows CHECK / NO FRESH TRACK instead.
"""
import argparse
import collections
import csv
import json
import math
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pygame
import carla

# ------------------------------- UI palette ---------------------------------
BG = (7, 11, 18)
PANEL = (13, 20, 30)
PANEL_ALT = (17, 27, 40)
GRID = (38, 52, 68)
WHITE = (235, 242, 248)
MUTED = (129, 148, 166)
CYAN = (0, 224, 245)
GREEN = (66, 224, 156)
AMBER = (255, 190, 70)
RED = (255, 80, 90)
MAGENTA = (232, 100, 255)
BLUE = (95, 160, 255)
ORANGE = (255, 143, 70)

STATE_STALE_S = 1.75
OBJECT_STALE_S = 1.75
LINK_STALE_S = 2.0
LOS_CACHE_S = 0.85
LIDAR_LOS_FRESH_S = 1.0
MAX_MSG_BYTES = 90
LIDAR_DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("cos", "<f4"),
    ("obj", "<u4"), ("tag", "<u4")
])  # CARLA semantic-LiDAR wire layout: XYZ, cosine, actor ID, semantic tag


def parse_args():
    p = argparse.ArgumentParser(description="CARLA cooperative V2V drone mesh dashboard")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2500)
    p.add_argument("--rolename", default="hero")
    p.add_argument("--channel", default="v2v_coop_drone_mesh")
    p.add_argument("--max-vehicles", type=int, default=5, help="Maximum existing vehicles to instrument; does not spawn traffic")
    p.add_argument("--width", type=int, default=1440)
    p.add_argument("--height", type=int, default=900)
    p.add_argument("--fps", type=int, default=60, help="Pygame UI target FPS, not CARLA simulation FPS")
    p.add_argument("--camera-width", type=int, default=None)
    p.add_argument("--camera-height", type=int, default=None)
    p.add_argument("--camera-hz", type=float, default=None, help="Camera capture rate; auto-scales by fleet if omitted")
    p.add_argument("--spectator-hz", type=float, default=None, help="Auto-scales by fleet if omitted")
    p.add_argument("--camera-mode", choices=("satellite", "chase"), default="satellite",
                   help="Satellite is a true 90-degree top-down map camera; chase is a third-person view")
    p.add_argument("--satellite-altitude", type=float, default=90.0,
                   help="Camera height above the camera target in metres; mouse wheel adjusts it live")
    p.add_argument("--satellite-yaw", type=float, default=0.0,
                   help="Initial satellite view rotation in degrees; Q/E rotate live")
    p.add_argument("--free-camera-start", action="store_true",
                   help="Start satellite camera free instead of following ego")
    p.add_argument("--camera-back", type=float, default=7.5)
    p.add_argument("--camera-height-m", type=float, default=3.2)
    p.add_argument("--camera-pitch", type=float, default=-13.0)
    p.add_argument("--camera-fov", type=float, default=90.0, help="Camera horizontal field of view")
    p.add_argument("--tm-port", type=int, default=8000)
    p.add_argument("--no-manual-control", action="store_true", help="Disable WASD/arrow driving inside the mesh window")
    p.add_argument("--min-range", type=float, default=50.0)
    p.add_argument("--max-range", type=float, default=250.0)
    p.add_argument("--range-seconds", type=float, default=2.0, help="Adaptive display radius = min + ego speed * this value")
    p.add_argument("--v2x-filter-distance", type=float, default=250.0)
    p.add_argument("--state-hz", type=float, default=6.0, help="Requested state broadcast rate; automatically reduced for larger fleets")
    p.add_argument("--object-hz", type=float, default=1.5, help="Requested object-report rate per car; automatically reduced for larger fleets")
    p.add_argument("--lidar-range", type=float, default=80.0)
    p.add_argument("--lidar-pps", type=int, default=5000, help="Requested points/sec per car; fleet budget caps the actual rate")
    p.add_argument("--lidar-channels", type=int, default=4)
    p.add_argument("--lidar-hz", type=float, default=None, help="Auto-scales by fleet if omitted")
    p.add_argument("--los-checks-per-second", type=float, default=8.0, help="Background geometric visibility checks; no UI blocking")
    p.add_argument("--selection-file", default="selected_vehicle.txt")
    p.add_argument("--report-dir", default="session_reports", help="Directory for automatic per-session CSV/JSON/Markdown reports")
    p.add_argument("--report-sample-hz", type=float, default=2.0, help="Ground-truth vehicle sampling rate for the session report (1-10 Hz)")
    p.add_argument("--lane-tile-size", type=float, default=80.0, help="Spatial tile size for cached lane drawing")
    p.add_argument("--no-spectator-follow", action="store_true", help="Do not move CARLA's original spectator window")
    p.add_argument("--no-lidar", action="store_true", help="Disable per-car semantic LiDAR (disables sensor-derived LOS/object perception)")
    return p.parse_args()


def compact_num(value, digits=1):
    value = round(float(value), digits)
    if abs(value - round(value)) < 1e-8:
        return int(round(value))
    return value


def safe_alive(actor):
    try:
        return actor is not None and actor.is_alive
    except Exception:
        return False


def safe_stop_destroy(actor):
    if actor is None:
        return
    try:
        actor.stop()
    except Exception:
        pass
    try:
        actor.destroy()
    except Exception:
        pass


def speed3(v):
    return math.sqrt(float(v[0]) ** 2 + float(v[1]) ** 2 + float(v[2] if len(v) > 2 else 0.0) ** 2)


def world_to_ego_xy(point_xy, ego_xy, yaw_rad):
    dx = float(point_xy[0]) - float(ego_xy[0])
    dy = float(point_xy[1]) - float(ego_xy[1])
    forward = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
    lateral = -dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
    return forward, lateral


def camera_project(point, camera_transform, width, height, fov=90.0):
    """Project world XYZ into the camera image. Returns None if behind/outside."""
    try:
        inv = np.asarray(camera_transform.get_inverse_matrix(), dtype=np.float64)
        cp = inv @ np.array([float(point[0]), float(point[1]), float(point[2]), 1.0])
        q = np.array([cp[1], -cp[2], cp[0]], dtype=np.float64)
        if q[2] <= 0.1:
            return None
        focal = width / (2.0 * math.tan(math.radians(fov) / 2.0))
        u = focal * q[0] / q[2] + width / 2.0
        v = focal * q[1] / q[2] + height / 2.0
        if 0 <= u < width and 0 <= v < height:
            return int(u), int(v)
    except Exception:
        return None
    return None


class LOSWorker(threading.Thread):
    """Background line-of-sight geometry checker. Never blocks the render loop."""
    def __init__(self, host, port, rate_hz=25.0):
        super().__init__(daemon=True, name="CARLA-LOS-worker")
        self.host, self.port = host, port
        self.interval = 1.0 / max(1.0, float(rate_hz))
        self.tasks = queue.Queue(maxsize=1000)
        self.results = queue.Queue(maxsize=1000)
        self.pending = set()
        self.pending_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.world = None
        self.error = ""

    def submit(self, actor_id, start_xyz, end_xyz, target_radius=2.5):
        actor_id = int(actor_id)
        with self.pending_lock:
            if actor_id in self.pending or self.stop_event.is_set():
                return
            self.pending.add(actor_id)
        try:
            self.tasks.put_nowait((actor_id, tuple(start_xyz), tuple(end_xyz), float(target_radius)))
        except queue.Full:
            with self.pending_lock:
                self.pending.discard(actor_id)

    def run(self):
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(2.0)
            self.world = client.get_world()
        except Exception as exc:
            self.error = str(exc)
            return
        while not self.stop_event.is_set():
            try:
                task = self.tasks.get(timeout=0.15)
            except queue.Empty:
                continue
            actor_id, start, end, target_radius = task
            began = time.monotonic()
            visible = None
            detail = "raycast unavailable"
            try:
                start_loc = carla.Location(x=start[0], y=start[1], z=start[2])
                end_loc = carla.Location(x=end[0], y=end[1], z=end[2])
                ray_hits = self.world.cast_ray(start_loc, end_loc)
                dist_target = math.dist(start, end)
                hit_distances = []
                for hit in ray_hits:
                    loc = getattr(hit, "location", None)
                    if loc is None:
                        continue
                    d = math.dist(start, (loc.x, loc.y, loc.z))
                    if d > 1.25:
                        hit_distances.append(d)
                first_hit = min(hit_distances) if hit_distances else None
                # The target vehicle surface can be several metres before its centre.
                visible = first_hit is None or first_hit >= dist_target - target_radius - 0.8
                detail = "scene ray clear" if visible else "scene geometry blocks ray"
            except Exception as exc:
                detail = f"raycast failed: {exc}"
            try:
                self.results.put_nowait((actor_id, visible, time.monotonic(), detail, start, end))
            except queue.Full:
                pass
            with self.pending_lock:
                self.pending.discard(actor_id)
            pause = self.interval - (time.monotonic() - began)
            if pause > 0:
                self.stop_event.wait(pause)

    def stop(self):
        self.stop_event.set()


class CooperativeADAS:
    def __init__(self, args):
        self.args = args
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(4.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.spectator = self.world.get_spectator()

        pygame.init()
        pygame.font.init()
        pygame.display.set_caption("COOPERATIVE ADAS  /  V2V MESH + DRONE VIEW")
        self.screen = pygame.display.set_mode((args.width, args.height), pygame.RESIZABLE | pygame.DOUBLEBUF)
        self.clock = pygame.time.Clock()
        # One cached panel composite avoids three full-window alpha blends every frame.
        # Heavy map/info/graph draw runs at 10 Hz; the camera/header/footer update at UI FPS.
        self.panel_cache_surface = None
        self.cache_size = None
        self.fonts = {
            11: pygame.font.SysFont("segoeui", 11),
            13: pygame.font.SysFont("segoeui", 13),
            15: pygame.font.SysFont("segoeui", 15),
            17: pygame.font.SysFont("segoeui", 17),
            19: pygame.font.SysFont("segoeui", 19, bold=True),
            22: pygame.font.SysFont("segoeui", 22, bold=True),
        }

        self.running = True
        # Driving is integrated into this main Pygame window; no separate controller window is required.
        self.manual_control_enabled = not args.no_manual_control
        self.manual_autopilot = False
        self.manual_reverse = False
        self.manual_steer = 0.0
        self.manual_focus_active = False
        self.last_manual_tick = time.monotonic()
        self.last_control_send = 0.0
        self.ego = None
        self.ego_id = None
        self.selected_id = None
        self.vehicle_refs = {}
        self.actor_refs = {}
        self.actor_kinds = {}
        self.actor_type_ids = {}
        self.actor_radii = {}
        self.last_actor_sync = 0.0
        self.max_vehicles = max(1, min(50, args.max_vehicles))
        self.connected_ids = []
        self.v2x_sensors = {}
        self.lidar_sensors = {}
        self.sensor_sync_time = 0.0
        self.last_spawn_id = None
        self.next_state_tx = {}
        self.next_object_tx = {}
        self.next_urgent_object_tx = collections.defaultdict(float)
        self.urgent_pending = set()
        self.seq_by_vehicle = collections.defaultdict(int)
        self.object_seq_by_vehicle = collections.defaultdict(int)
        self.object_cursor = collections.defaultdict(int)
        self.last_transmit_hz = (0.0, 0.0)
        self.tx_counts = collections.Counter()
        self.rx_counts = collections.Counter()
        # Parsed ego receiver counts grouped by radio sender and message type.
        self.rx_by_sender = collections.Counter()
        self.rx_power_sum_by_sender = collections.defaultdict(float)
        self.rx_power_n_by_sender = collections.Counter()
        self.sensor_detection_events = collections.Counter()
        self.sensor_detection_by_kind = collections.Counter()
        self.sensor_detected_ids = collections.defaultdict(set)

        self.state_lock = threading.RLock()
        # receiver -> sender -> latest state; receiver -> observed actor -> report
        self.rx_states = collections.defaultdict(dict)
        self.rx_objects = collections.defaultdict(dict)
        self.radio_links = {}
        # observer vehicle -> observed object -> latest sensor-derived detection
        self.local_detections = collections.defaultdict(dict)
        self.vehicle_history = collections.defaultdict(lambda: collections.deque(maxlen=240))
        self.last_remote_velocity = {}
        self.started = time.monotonic()
        self.latest_camera_lock = threading.Lock()
        self.latest_camera = None
        self.camera_surface = None                  # native-size latest RGB camera frame
        self.camera_display_surface = None          # scaled once per frame/resize, not every UI redraw
        self.camera_display_size = None
        self.camera_display_frame_number = -1
        self.camera_frame_number = -1
        self.camera_surface_size = None
        self.camera_sensor = None
        self.camera_spawned_for = None
        # Satellite camera is an independent world-space sensor, not attached to ego.
        self.camera_mode = args.camera_mode
        self.camera_follow = not bool(args.free_camera_start)
        self.satellite_altitude = float(args.satellite_altitude)
        self.satellite_yaw = float(args.satellite_yaw) % 360.0
        self.satellite_center = None          # free camera target (world X/Y/Z)
        self.satellite_current_center = None  # smoothed current view target
        self.camera_last_pose_update = 0.0
        self.camera_pan_dragging = False
        self.camera_last_mouse = None
        self.last_spectator_update = 0.0
        self.latest_image_transform = None
        self.camera_error = ""
        self.camera_pick_points = []
        self.camera_overlay_items = []
        self.camera_overlay_frame_number = -1
        self.camera_overlay_rect_key = None
        self.last_camera_overlay_update = 0.0
        self.mesh_pick_points = []
        self.ego_speed_kmh = 0.0
        self.last_ego_sample = 0.0
        self.last_mesh_draw = 0.0
        self.last_graph_draw = 0.0
        self.last_los_schedule = 0.0
        self.last_health_log = 0.0
        self.last_ui_status = "Connecting"
        self.active_view_range = float(args.min_range)
        self.mesh_nodes = []
        self.visible_ids = set()
        self.los_cache = {}
        self.los_worker = LOSWorker(args.host, args.port, args.los_checks_per_second)
        self.los_worker.start()
        self.use_cast_ray = callable(getattr(self.world, "cast_ray", None))
        self.lane_groups = []
        self.lane_tile_size = max(40.0, float(args.lane_tile_size))
        self.lane_tiles = collections.defaultdict(list)
        self.lane_background_surface = None
        self.lane_background_cache_key = None
        self.last_lane_cache_build = 0.0
        self.state_tx_cursor = 0
        self._build_lane_cache()
        self.radio_model_active = False
        self._setup_v2x_blueprint()
        self._setup_lidar_blueprint()
        self._print_startup()
        self._sync_actor_cache(force=True)
        self._wait_and_find_ego()
        if self.manual_control_enabled and safe_alive(self.ego):
            try:
                self.ego.set_autopilot(False, int(self.args.tm_port))
                self.say(f"[CONTROL] Manual driving enabled for ego={self.ego_id}. Click THIS ADAS window; WASD drive; SPACE handbrake; R reverse; P autopilot; HOME selects ego/recenters.")
            except Exception as exc:
                self.say(f"[CONTROL WARNING] Could not initialise manual control: {exc}")
        self._create_camera_for_ego()
        self._init_session_recorder()

    # ------------------------------- setup -----------------------------------
    def say(self, msg):
        print(msg, flush=True)

    def _print_startup(self):
        self.say(f"[CARLA] Connected to {self.map.name} at {self.args.host}:{self.args.port}")
        if self.camera_mode == "satellite":
            camera_description = f"SATELLITE 90deg altitude={self.satellite_altitude:.0f}m yaw={self.satellite_yaw:.0f}deg follow={'ON' if self.camera_follow else 'OFF'}"
        else:
            camera_description = f"CHASE back={self.args.camera_back:g}m height={self.args.camera_height_m:g}m pitch={self.args.camera_pitch:g}deg"
        self.say(f"[CONFIG] max vehicles={self.max_vehicles}; UI={self.args.fps} FPS; camera={self.args.camera_width}x{self.args.camera_height}@{self.args.camera_hz:g}Hz; {camera_description}; LiDAR={self.args.lidar_channels}ch/{self.args.lidar_hz:g}Hz")
        self.say("[PERFORMANCE] Latest camera frame wins; map lanes are spatially tiled/cached; V2X JSON is parsed only at ego.")
        self.say("[CAMERA] SATELLITE mode is a true pitch=-90deg top-down view; camera is not attached to a car. Spectator mirrors its pose.")
        self.say("[CAMERA CONTROLS] Right/middle drag pans and enters FREE mode; mouse wheel or [ / ] zooms; Q/E rotates; F toggles follow; M recenters and follows ego.")
        self.say("[RULE] This app will not spawn vehicles, call world.tick(), or alter synchronous mode.")

    def _set_bp(self, bp, attr, value):
        try:
            if bp.has_attribute(attr):
                bp.set_attribute(attr, str(value))
                return True
        except Exception:
            pass
        return False

    def _setup_v2x_blueprint(self):
        self.v2x_blueprint = self.world.get_blueprint_library().find("sensor.other.v2x_custom")
        requested = {
            "channel_id": self.args.channel,
            "scenario": "urban",
            "path_loss_model": "geometric",
            "filter_distance": str(float(max(self.args.v2x_filter_distance, self.args.max_range))),
            "use_etsi_fading": "true",
            "transmit_power": "21.5",
            "receiver_sensitivity": "-99.0",
        }
        configured = {}
        errors = {}
        for key, value in requested.items():
            try:
                if not self.v2x_blueprint.has_attribute(key):
                    configured[key] = False
                    errors[key] = "not exposed by this CARLA blueprint"
                    continue
                self.v2x_blueprint.set_attribute(key, str(value))
                configured[key] = True
            except Exception as exc:
                configured[key] = False
                errors[key] = str(exc)
        self.radio_config = configured
        # The label means the relevant settings were accepted by the local blueprint.
        # It does not claim a packet has been proven to traverse an obstructed RF path.
        required = ("channel_id", "scenario", "path_loss_model")
        self.radio_model_active = all(configured.get(key, False) for key in required)
        attrs_available = []
        try:
            attrs_available = sorted(str(getattr(attr, "id", attr)) for attr in self.v2x_blueprint)
        except Exception:
            pass
        for key in requested:
            state = "SET" if configured.get(key) else "MISSING/REJECTED"
            suffix = f" ({errors[key]})" if key in errors else ""
            self.say(f"[RF CONFIG] {key}={requested[key]} -> {state}{suffix}")
        if self.radio_model_active:
            self.say(f"[RF NLOS MODEL: CONFIGURED] channel={self.args.channel}; scenario=urban; path_loss_model=geometric.")
            self.say("[RF LIMITATION] CARLA models radio loss using visibility/scenario; Python receive callbacks do not expose a definitive per-packet RF-NLOS flag.")
        else:
            missing = ", ".join(key for key in required if not configured.get(key, False))
            self.say(f"[RF NLOS MODEL: NOT CONFIRMED] The running server's custom-V2X blueprint did not accept: {missing}.")
            self.say("[RF DEBUG] Available sensor.other.v2x_custom attributes: " + (", ".join(attrs_available) if attrs_available else "could not enumerate attributes"))

    def _setup_lidar_blueprint(self):
        self.lidar_blueprint = None
        if self.args.no_lidar:
            self.say("[PERCEPTION] LiDAR disabled by command line; local LOS/object detection will be unavailable.")
            return
        try:
            self.lidar_blueprint = self.world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
            attrs = {
                "range": str(float(self.args.lidar_range)),
                "channels": str(int(self.args.lidar_channels)),
                "points_per_second": str(int(self.args.lidar_pps)),
                "rotation_frequency": str(float(self.args.lidar_hz)),
                "upper_fov": "10.0",
                "lower_fov": "-25.0",
                "sensor_tick": str(1.0 / max(1.0, float(self.args.lidar_hz))),
            }
            for k, v in attrs.items():
                self._set_bp(self.lidar_blueprint, k, v)
            self.say(f"[PERCEPTION] semantic LiDAR ready: {self.args.lidar_channels} channels, {self.args.lidar_range:g} m, {self.args.lidar_pps} pts/s/car, 360° horizontal scan.")
        except Exception as exc:
            self.lidar_blueprint = None
            self.say(f"[PERCEPTION ERROR] semantic LiDAR unavailable: {exc}")

    def _build_lane_cache(self):
        """Precompute short world-space lane chunks indexed into spatial tiles.

        The old renderer transformed every waypoint in the entire Town map ten
        times per second. This version looks up only chunks near the ego car and
        caches their rendered lane layer until the ego has moved/turned enough.
        """
        try:
            waypoints = self.map.generate_waypoints(8.0)
            groups = collections.defaultdict(list)
            for wp in waypoints:
                loc = wp.transform.location
                groups[(int(wp.road_id), int(wp.lane_id))].append(
                    (float(wp.s), float(loc.x), float(loc.y), float(wp.lane_width))
                )
            self.lane_groups = []
            self.lane_tiles.clear()
            for vals in groups.values():
                vals.sort(key=lambda x: x[0])
                arr = np.asarray([[v[1], v[2], v[3]] for v in vals], dtype=np.float32)
                if len(arr) < 2:
                    continue
                self.lane_groups.append(arr)
                # 5-point chunks overlap by one point (step 4), keeping line joints continuous.
                for first in range(0, len(arr) - 1, 4):
                    chunk = np.array(arr[first:min(first + 5, len(arr)), :2], dtype=np.float32, copy=True)
                    if len(chunk) < 2:
                        continue
                    mid = np.mean(chunk, axis=0)
                    tile = (math.floor(float(mid[0]) / self.lane_tile_size),
                            math.floor(float(mid[1]) / self.lane_tile_size))
                    self.lane_tiles[tile].append(chunk)
            chunk_count = sum(len(v) for v in self.lane_tiles.values())
            self.say(f"[MAP] Cached {len(self.lane_groups)} lane groups as {chunk_count} spatial chunks in {len(self.lane_tiles)} tiles.")
        except Exception as exc:
            self.say(f"[MAP WARNING] Could not cache map waypoints: {exc}")
            self.lane_groups = []
            self.lane_tiles.clear()

    def _get_lane_background(self, plot_size, ego_pos, yaw, radius):
        """Return a cached local lane drawing. Rebuild only after material pose/range change."""
        w, h = max(1, int(plot_size[0])), max(1, int(plot_size[1]))
        # Quantisation intentionally prevents tiny per-frame vehicle movement from forcing a full redraw.
        key = (round(ego_pos[0] / 3.0), round(ego_pos[1] / 3.0),
               round(math.degrees(yaw) / 4.0), round(radius / 8.0), w, h)
        if self.lane_background_surface is not None and self.lane_background_cache_key == key:
            return self.lane_background_surface

        surf = pygame.Surface((w, h)).convert()
        surf.fill((8, 14, 21))
        center = (w // 2, h // 2)
        for r in (radius, radius * 0.66, radius * 0.33):
            ew = int(w * 0.86 * r / max(radius, 1.0))
            eh = int(h * 0.78 * r / max(radius, 1.0))
            pygame.draw.ellipse(surf, GRID, pygame.Rect(center[0] - ew // 2, center[1] - eh // 2, ew, eh), 1)
        pygame.draw.line(surf, GRID, (center[0], 0), (center[0], h), 1)
        pygame.draw.line(surf, GRID, (0, center[1]), (w, center[1]), 1)

        tile = self.lane_tile_size
        margin = radius + 35.0
        tx0, tx1 = math.floor((ego_pos[0] - margin) / tile), math.floor((ego_pos[0] + margin) / tile)
        ty0, ty1 = math.floor((ego_pos[1] - margin) / tile), math.floor((ego_pos[1] + margin) / tile)
        chunks = []
        for tx in range(tx0, tx1 + 1):
            for ty in range(ty0, ty1 + 1):
                chunks.extend(self.lane_tiles.get((tx, ty), ()))

        c, s = math.cos(yaw), math.sin(yaw)
        lane_col = (58, 73, 86)
        fwd_limit = radius * 1.04
        lat_limit = radius * 1.04
        sx = w * 0.43 / max(radius, 1.0)
        sy = h * 0.39 / max(radius, 1.0)
        ex, ey = float(ego_pos[0]), float(ego_pos[1])
        for chunk in chunks:
            dx = chunk[:, 0].astype(np.float64, copy=False) - ex
            dy = chunk[:, 1].astype(np.float64, copy=False) - ey
            fwd = dx * c + dy * s
            lat = -dx * s + dy * c
            mask = (np.abs(fwd) <= fwd_limit) & (np.abs(lat) <= lat_limit)
            idx = np.flatnonzero(mask)
            if len(idx) < 2:
                continue
            split_points = np.flatnonzero(np.diff(idx) > 1) + 1
            for run in np.split(idx, split_points):
                if len(run) < 2:
                    continue
                points = [(int(center[0] + lat[j] * sx), int(center[1] - fwd[j] * sy)) for j in run]
                if len(points) >= 2:
                    pygame.draw.lines(surf, lane_col, False, points, 1)
        self.lane_background_surface = surf
        self.lane_background_cache_key = key
        return surf

    def _sync_actor_cache(self, force=False):
        """Refresh actor metadata at low rate, but attach pending sensors independently."""
        now = time.monotonic()
        do_refresh = force or now - self.last_actor_sync >= 0.6
        if do_refresh:
            self.last_actor_sync = now
            try:
                actors = self.world.get_actors()
                vehicles = list(actors.filter("vehicle.*"))
                walkers = list(actors.filter("walker.pedestrian.*"))
            except Exception as exc:
                self.say(f"[ACTOR SYNC WARNING] {exc}")
                vehicles, walkers = list(self.vehicle_refs.values()), []
            actor_kinds, actor_type_ids, actor_radii = {}, {}, {}
            for actor in vehicles:
                actor_kinds[int(actor.id)] = "v"
                actor_type_ids[int(actor.id)] = str(getattr(actor, "type_id", "vehicle"))
                try:
                    actor_radii[int(actor.id)] = max(1.5, math.hypot(actor.bounding_box.extent.x, actor.bounding_box.extent.y))
                except Exception:
                    actor_radii[int(actor.id)] = 2.5
            for actor in walkers:
                actor_kinds[int(actor.id)] = "p"
                actor_type_ids[int(actor.id)] = str(getattr(actor, "type_id", "walker.pedestrian"))
                actor_radii[int(actor.id)] = 0.7
            self.vehicle_refs = {int(a.id): a for a in vehicles}
            self.actor_refs = {int(a.id): a for a in (vehicles + walkers)}
            self.actor_kinds = actor_kinds
            self.actor_type_ids = actor_type_ids
            self.actor_radii = actor_radii
            hero = next((v for v in vehicles if v.attributes.get("role_name", "") == self.args.rolename), None)
            reference = hero if hero is not None else (self.ego if safe_alive(self.ego) else (vehicles[0] if vehicles else None))
            if reference is not None:
                ref_loc = reference.get_location()
                # Instrument only the ego-centred working set. A speed-dependent
                # enter radius plus a wider exit radius adds hysteresis, so cars do
                # not constantly attach/destroy sensors when hovering near an edge.
                working_radius = min(float(self.args.max_range), max(float(self.args.min_range), float(self.active_view_range)))
                enter_radius = working_radius + 35.0
                exit_radius = working_radius + 65.0
                nearby = []
                for candidate in vehicles:
                    if int(candidate.id) == int(reference.id):
                        continue
                    try:
                        loc = candidate.get_location()
                        distance = math.hypot(loc.x - ref_loc.x, loc.y - ref_loc.y)
                        nearby.append((distance, candidate))
                    except Exception:
                        continue
                nearby.sort(key=lambda pair: pair[0])
                current_sensor_ids = set(self.v2x_sensors)
                # Instrument the requested nearest N-1 peers independent of the current display radius.
                # The mesh itself still hides nodes outside the adaptive awareness radius. This prevents
                # a 20-car test from silently creating only 3-5 radio nodes when traffic is spread out.
                keepers = [actor for distance, actor in nearby if int(actor.id) in current_sensor_ids]
                ordered = [reference]
                ordered.extend(keepers[:max(0, self.max_vehicles - 1)])
                ordered_ids = {int(actor.id) for actor in ordered}
                for distance, actor in nearby:
                    if len(ordered) >= self.max_vehicles:
                        break
                    if int(actor.id) not in ordered_ids:
                        ordered.append(actor)
                        ordered_ids.add(int(actor.id))
            else:
                ordered = sorted(vehicles, key=lambda a: a.id)[:self.max_vehicles]
            self.connected_ids = [int(v.id) for v in ordered]
            if hero is not None and self.ego_id != hero.id:
                old_camera = self.camera_sensor is not None
                self.ego = hero
                self.ego_id = int(hero.id)
                self.selected_id = self.ego_id
                self.say(f"[EGO] Active hero is vehicle {hero.id}.")
                if old_camera:
                    self._create_camera_for_ego()
            elif self.ego is not None and not safe_alive(self.ego):
                self.ego = None
                self.ego_id = None
            # Remove only this application's sensors if actors leave the selected fleet.
            active = set(self.connected_ids)
            for actor_id in list(self.v2x_sensors):
                if actor_id not in active or actor_id not in self.vehicle_refs:
                    safe_stop_destroy(self.v2x_sensors.pop(actor_id, None))
                    safe_stop_destroy(self.lidar_sensors.pop(actor_id, None))
                    self.next_state_tx.pop(actor_id, None)
                    self.next_object_tx.pop(actor_id, None)
        # Attaching sensor actors is deliberately paced; it avoids spawning 100 sensors in one frame at 50 cars.
        attach_delay = 0.12 if len(self.connected_ids) <= 10 else 0.22 if len(self.connected_ids) <= 25 else 0.28
        if self.connected_ids and now - self.sensor_sync_time >= attach_delay:
            new_id = next((i for i in self.connected_ids if i not in self.v2x_sensors), None)
            if new_id is not None:
                self._attach_vehicle_sensors(new_id)
                self.sensor_sync_time = now

    def _wait_and_find_ego(self):
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            vehicles = list(self.world.get_actors().filter("vehicle.*"))
            hero = next((v for v in vehicles if v.attributes.get("role_name", "") == self.args.rolename), None)
            if hero is not None:
                self.ego = hero
                self.ego_id = int(hero.id)
                self.selected_id = self.ego_id
                return
            time.sleep(0.25)
        raise RuntimeError(f"No vehicle with role_name='{self.args.rolename}' found. Start run_environment.ps1 and manual_control first.")

    def _attach_vehicle_sensors(self, actor_id):
        vehicle = self.vehicle_refs.get(int(actor_id))
        if not safe_alive(vehicle):
            return
        try:
            sensor = self.world.spawn_actor(self.v2x_blueprint, carla.Transform(), attach_to=vehicle)
            self.v2x_sensors[int(actor_id)] = sensor
            sensor.listen(lambda event, receiver_id=int(actor_id): self._on_v2x(event, receiver_id))
            now = time.monotonic()
            index = max(0, self.connected_ids.index(int(actor_id))) if int(actor_id) in self.connected_ids else 0
            n = max(1, len(self.connected_ids))
            state_hz, object_hz = self._effective_rates(n)
            self.next_state_tx[int(actor_id)] = now + index / n / max(state_hz, 0.1)
            self.next_object_tx[int(actor_id)] = now + (index / n) / max(object_hz, 0.1)
            self.say(f"[V2X READY] vehicle={actor_id} channel={self.args.channel}")
        except Exception as exc:
            self.say(f"[V2X SENSOR ERROR] vehicle={actor_id}: {exc}")
            return
        if self.lidar_blueprint is not None:
            try:
                # Automatically reduce LiDAR workload as connected fleet grows.
                n = max(1, len(self.connected_ids))
                pps = min(int(self.args.lidar_pps), max(800, int(50000 / n)))
                lidar_bp = self.lidar_blueprint
                # Blueprints are mutable shared configuration; per-car rate is set before each spawn.
                if lidar_bp.has_attribute("points_per_second"):
                    lidar_bp.set_attribute("points_per_second", str(pps))
                lidar_tf = carla.Transform(carla.Location(x=0.0, z=1.7))
                try:
                    lidar = self.world.spawn_actor(lidar_bp, lidar_tf, attach_to=vehicle)
                except Exception:
                    # Restore the common configured pps if this CARLA build rejects an attribute change.
                    lidar = self.world.spawn_actor(self.lidar_blueprint, lidar_tf, attach_to=vehicle)
                self.lidar_sensors[int(actor_id)] = lidar
                lidar.listen(lambda measurement, observer_id=int(actor_id): self._on_semantic_lidar(measurement, observer_id))
                self.say(f"[PERCEPTION READY] vehicle={actor_id} semantic_lidar={pps} pts/s range={self.args.lidar_range:g}m")
            except Exception as exc:
                self.say(f"[PERCEPTION ERROR] vehicle={actor_id}: {exc}")

    def _camera_transform_for_target(self, center):
        """Create the world-space transform for satellite or chase camera mode."""
        if self.camera_mode == "satellite":
            altitude = max(25.0, min(350.0, float(self.satellite_altitude)))
            location = carla.Location(x=float(center[0]), y=float(center[1]), z=float(center[2]) + altitude)
            rotation = carla.Rotation(pitch=-90.0, yaw=float(self.satellite_yaw), roll=0.0)
            return carla.Transform(location, rotation)
        ego_tf = self.ego.get_transform()
        rel = carla.Location(x=-self.args.camera_back, y=0.0, z=self.args.camera_height_m)
        location = ego_tf.transform(rel)
        rotation = carla.Rotation(pitch=self.args.camera_pitch, yaw=ego_tf.rotation.yaw, roll=0.0)
        return carla.Transform(location, rotation)

    def _create_camera_for_ego(self):
        """Create one unattached RGB camera; it can follow ego or move freely over the map."""
        if not safe_alive(self.ego):
            return
        self._destroy_camera()
        try:
            ego_loc = self.ego.get_location()
            if self.satellite_center is None:
                self.satellite_center = [float(ego_loc.x), float(ego_loc.y), float(ego_loc.z)]
            if self.satellite_current_center is None:
                self.satellite_current_center = list(self.satellite_center)
            bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
            self._set_bp(bp, "image_size_x", self.args.camera_width)
            self._set_bp(bp, "image_size_y", self.args.camera_height)
            self._set_bp(bp, "fov", self.args.camera_fov)
            self._set_bp(bp, "sensor_tick", 1.0 / max(1.0, self.args.camera_hz))
            self._set_bp(bp, "motion_blur_intensity", 0.0)
            self._set_bp(bp, "lens_flare_intensity", 0.0)
            self._set_bp(bp, "enable_postprocess_effects", "false")
            initial_center = self.satellite_current_center if self.camera_mode == "satellite" else [ego_loc.x, ego_loc.y, ego_loc.z]
            initial_transform = self._camera_transform_for_target(initial_center)
            # No attach_to: this is a movable camera rig rather than a vehicle-mounted camera.
            camera = self.world.spawn_actor(bp, initial_transform)
            self.camera_sensor = camera
            self.camera_spawned_for = int(self.ego.id)
            camera.listen(self._on_camera)
            self.camera_last_pose_update = 0.0
            self.say(f"[CAMERA READY] mode={self.camera_mode} 90deg_satellite={'ON' if self.camera_mode == 'satellite' else 'OFF'} follow={'ON' if self.camera_follow else 'OFF'}; drag=pan wheel=zoom Q/E=rotate F=follow M=recenter")
        except Exception as exc:
            self.camera_error = str(exc)
            self.say(f"[CAMERA ERROR] {exc}")
            self.camera_sensor = None

    def _destroy_camera(self):
        if self.camera_sensor is not None:
            safe_stop_destroy(self.camera_sensor)
        self.camera_sensor = None
        self.camera_spawned_for = None
        with self.latest_camera_lock:
            self.latest_camera = None
        self.camera_surface = None
        self.camera_display_surface = None
        self.camera_display_size = None
        self.camera_display_frame_number = -1
        self.camera_frame_number = -1
        self.camera_overlay_items = []
        self.camera_pick_points = []
        self.camera_overlay_frame_number = -1
        self.camera_overlay_rect_key = None
        self.latest_image_transform = None

    def _update_camera_pose(self, now):
        """Move the unattached camera rig, optionally tracking ego with smooth interpolation."""
        if not safe_alive(self.camera_sensor) or not safe_alive(self.ego):
            return
        connected = max(1, len(self.v2x_sensors))
        pose_hz = 20.0 if connected <= 20 else 12.0 if connected <= 35 else 10.0
        if now - self.camera_last_pose_update < 1.0 / pose_hz:
            return
        try:
            ego_loc = self.ego.get_location()
            if self.camera_mode == "satellite":
                if self.satellite_center is None:
                    self.satellite_center = [float(ego_loc.x), float(ego_loc.y), float(ego_loc.z)]
                if self.satellite_current_center is None:
                    self.satellite_current_center = list(self.satellite_center)
                if self.camera_follow:
                    target = [float(ego_loc.x), float(ego_loc.y), float(ego_loc.z)]
                    dt = max(0.0, min(0.25, now - self.camera_last_pose_update))
                    alpha = min(1.0, max(0.08, dt * 7.0))
                    self.satellite_current_center = [
                        self.satellite_current_center[i] + (target[i] - self.satellite_current_center[i]) * alpha
                        for i in range(3)
                    ]
                    self.satellite_center = list(self.satellite_current_center)
                transform = self._camera_transform_for_target(self.satellite_current_center)
            else:
                transform = self._camera_transform_for_target([ego_loc.x, ego_loc.y, ego_loc.z])
            self.camera_sensor.set_transform(transform)
            self.camera_last_pose_update = now
        except Exception as exc:
            self.camera_error = f"camera pose update: {exc}"

    def _on_camera(self, image):
        # Sensor callback does no rendering/scaling; only the latest frame survives.
        try:
            packed = (int(image.frame), bytes(image.raw_data), image.transform, int(image.width), int(image.height), time.monotonic())
            with self.latest_camera_lock:
                self.latest_camera = packed
        except Exception:
            pass

    def _on_v2x(self, event, receiver_id):
        received_at = time.monotonic()
        # Every sensor must listen for the native CARLA custom-V2X sensor to work.
        # The driver-facing fused view is held at ego, so non-ego receivers count
        # incoming samples but do not parse JSON or build N^2 Python state tables.
        # This sharply cuts client CPU/lock contention at 20-50 vehicles.
        if int(receiver_id) != int(self.ego_id if self.ego_id is not None else -1):
            try:
                self.rx_counts[int(receiver_id)] += len(event)
            except Exception:
                self.rx_counts[int(receiver_id)] += 1
            return
        try:
            for item in event:
                envelope = item.get()
                msg_container = envelope.get("Message", {})
                raw = msg_container.get("Message", "") if isinstance(msg_container, dict) else ""
                if isinstance(raw, bytes):
                    raw = raw.decode("utf-8", errors="replace")
                if not raw:
                    continue
                payload = json.loads(raw)
                msg_type = payload.get("t", "s")
                header = msg_container.get("Header", {}) if isinstance(msg_container, dict) else {}
                radio_sender_id = int(header.get("Station ID", payload.get("i", -1)))
                origin_id = int(payload.get("i", radio_sender_id))
                if radio_sender_id < 0 or receiver_id == radio_sender_id:
                    continue
                power = envelope.get("Power")
                with self.state_lock:
                    self.rx_counts[receiver_id] += 1
                    self.rx_by_sender[(int(receiver_id), int(radio_sender_id), str(msg_type))] += 1
                    self.rx_by_sender[(int(receiver_id), int(radio_sender_id), "all")] += 1
                    if power is not None:
                        try:
                            self.rx_power_sum_by_sender[int(radio_sender_id)] += float(power)
                            self.rx_power_n_by_sender[int(radio_sender_id)] += 1
                        except (TypeError, ValueError):
                            pass
                    self.radio_links[(receiver_id, radio_sender_id)] = {"rx_time": received_at, "power": power}
                    if msg_type == "s":
                        previous_sequence_state = self.rx_states[receiver_id].get(origin_id)
                        incoming_sequence = int(payload.get("q", 0)) % 1000
                        if previous_sequence_state is not None:
                            previous_sequence = int(previous_sequence_state.get("sequence", -1))
                            if previous_sequence >= 0:
                                sequence_delta = (incoming_sequence - previous_sequence) % 1000
                                # Ignore duplicate/out-of-order state messages; q wraps at 1000.
                                if sequence_delta == 0 or sequence_delta >= 500:
                                    continue
                        position = list(payload.get("p", [0.0, 0.0, 0.0]))
                        while len(position) < 3: position.append(0.0)
                        velocity = list(payload.get("v", [0.0, 0.0]))
                        while len(velocity) < 3: velocity.append(0.0)
                        controls = list(payload.get("c", [0.0, 0.0, 0.0]))
                        while len(controls) < 3: controls.append(0.0)
                        previous = self.rx_states[receiver_id].get(origin_id)
                        accel = [0.0, 0.0, 0.0]
                        if previous is not None:
                            dt = received_at - previous["rx_time"]
                            if 0.05 <= dt <= 3.0:
                                accel = [(float(velocity[i]) - float(previous["velocity"][i])) / dt for i in range(3)]
                                accel = [max(-25.0, min(25.0, a)) for a in accel]
                        state = {
                            "id": origin_id, "position": [float(x) for x in position[:3]],
                            "velocity": [float(x) for x in velocity[:3]], "heading": float(payload.get("h", 0.0)),
                            "controls": [float(x) for x in controls[:3]], "acceleration": accel,
                            "rx_time": received_at, "power": power, "sequence": int(payload.get("q", 0)),
                            "radio_sender_id": radio_sender_id,
                        }
                        self.rx_states[receiver_id][origin_id] = state
                        if receiver_id == self.ego_id:
                            self._append_state_sample_locked(origin_id, state, received_at)
                    elif msg_type in ("o", "h"):
                        target_id = int(payload.get("d", -1))
                        if target_id < 0:
                            continue
                        p = list(payload.get("p", [0.0, 0.0]))
                        while len(p) < 3: p.append(0.0)
                        v = list(payload.get("v", [0.0, 0.0]))
                        while len(v) < 3: v.append(0.0)
                        report = {
                            "object_id": target_id, "kind": str(payload.get("k", "o")),
                            "position": [float(x) for x in p[:3]], "velocity": [float(x) for x in v[:3]],
                            "reporter_id": origin_id, "radio_sender_id": radio_sender_id,
                            "rx_time": received_at,
                            "observed_at": received_at - min(max(float(payload.get("a", 0.0)) / 1000.0, 0.0), OBJECT_STALE_S + 1.0),
                            "power": power, "sequence": int(payload.get("q", 0)),
                            "hazard": msg_type == "h",
                        }
                        previous_report = self.rx_objects[receiver_id].get(target_id)
                        # Multiple peers can report the same target. Do not let an older
                        # observation overwrite a newer one just because its packet arrived later.
                        if (previous_report is not None and
                                float(previous_report.get("observed_at", previous_report.get("rx_time", 0.0))) > report["observed_at"] and
                                received_at - float(previous_report.get("observed_at", 0.0)) <= OBJECT_STALE_S):
                            continue
                        self.rx_objects[receiver_id][target_id] = report
                        if receiver_id == self.ego_id and target_id not in self.rx_states[receiver_id]:
                            self._append_object_sample_locked(target_id, report, received_at)
        except Exception as exc:
            # Rate-limit noisy malformed messages from repeated callbacks.
            if not hasattr(self, "last_rx_error") or time.monotonic() - self.last_rx_error > 3.0:
                self.say(f"[V2X RX WARNING] receiver={receiver_id}: {exc}")
                self.last_rx_error = time.monotonic()

    def _append_state_sample_locked(self, actor_id, state, now):
        hist = self.vehicle_history[int(actor_id)]
        p = state["position"]
        v = state["velocity"]
        yaw = float(state["heading"])
        ax, ay, az = state.get("acceleration", [0.0, 0.0, 0.0])
        rad = math.radians(yaw)
        long_a = ax * math.cos(rad) + ay * math.sin(rad)
        lat_a = -ax * math.sin(rad) + ay * math.cos(rad)
        c = state.get("controls", [0.0, 0.0, 0.0])
        hist.append({
            "t": now - self.started, "speed": speed3(v) * 3.6,
            "accel": math.sqrt(ax*ax + ay*ay + az*az), "steer": c[0], "throttle": c[1], "brake": c[2],
            "long": long_a, "lat": lat_a, "yaw": yaw, "pitch": 0.0, "roll": 0.0,
            "x": p[0], "y": p[1], "z": p[2], "source": "V2V RX"
        })

    def _append_object_sample_locked(self, actor_id, report, now):
        hist = self.vehicle_history[int(actor_id)]
        p, v = report["position"], report["velocity"]
        hist.append({
            "t": now - self.started, "speed": speed3(v)*3.6, "accel": 0.0,
            "steer": 0.0, "throttle": 0.0, "brake": 0.0, "long": 0.0, "lat": 0.0,
            "yaw": 0.0, "pitch": 0.0, "roll": 0.0, "x": p[0], "y": p[1], "z": p[2],
            "source": f"OBJECT REPORT FROM V{report['reporter_id']}"
        })

    def _on_semantic_lidar(self, measurement, observer_id):
        """Low-cost vectorised semantic-LiDAR detections in world coordinates.

        Groups XYZ samples with np.bincount rather than applying a full-length
        boolean mask once per detected actor. That avoids quadratic work when a
        dense view contains many actor IDs.
        """
        try:
            raw = np.frombuffer(measurement.raw_data, dtype=LIDAR_DTYPE)
            if raw.size == 0:
                return
            with self.state_lock:
                kinds = self.actor_kinds.copy()
            # CARLA semantic labels: Pedestrian=4, Vehicle=10. Filter semantic
            # tags first; actor-kind lookup later rejects stale/static actor IDs.
            mask = np.isin(raw["tag"], np.asarray([4, 10], dtype=np.uint32))
            if not np.any(mask):
                return
            selected = raw[mask]
            target_ids = selected["obj"]
            unique_ids, inverse, counts = np.unique(target_ids, return_inverse=True, return_counts=True)
            if not len(unique_ids):
                return
            local_x = selected["x"].astype(np.float64, copy=False)
            local_y = selected["y"].astype(np.float64, copy=False)
            local_z = selected["z"].astype(np.float64, copy=False)
            mat = np.asarray(measurement.transform.get_matrix(), dtype=np.float64)
            world_x = mat[0, 0] * local_x + mat[0, 1] * local_y + mat[0, 2] * local_z + mat[0, 3]
            world_y = mat[1, 0] * local_x + mat[1, 1] * local_y + mat[1, 2] * local_z + mat[1, 3]
            world_z = mat[2, 0] * local_x + mat[2, 1] * local_y + mat[2, 2] * local_z + mat[2, 3]
            denom = counts.astype(np.float64)
            cx = np.bincount(inverse, weights=world_x, minlength=len(unique_ids)) / denom
            cy = np.bincount(inverse, weights=world_y, minlength=len(unique_ids)) / denom
            cz = np.bincount(inverse, weights=world_z, minlength=len(unique_ids)) / denom
            sensor_loc = measurement.transform.location
            seen = time.monotonic()
            updates = []
            for i, raw_id in enumerate(unique_ids):
                target_id = int(raw_id)
                if target_id == int(observer_id):
                    continue
                kind = kinds.get(target_id)
                if kind not in ("v", "p"):
                    continue
                center = [float(cx[i]), float(cy[i]), float(cz[i])]
                distance = math.sqrt((center[0] - sensor_loc.x) ** 2 + (center[1] - sensor_loc.y) ** 2 + (center[2] - sensor_loc.z) ** 2)
                if distance > self.args.lidar_range:
                    continue
                updates.append((target_id, kind, center, distance, int(counts[i])))
            with self.state_lock:
                observer_cache = self.local_detections[int(observer_id)]
                for target_id, kind, center, distance, npoints in updates:
                    self.sensor_detection_events[int(observer_id)] += 1
                    self.sensor_detection_by_kind[(int(observer_id), str(kind))] += 1
                    self.sensor_detected_ids[int(observer_id)].add((int(target_id), str(kind)))
                    prev = observer_cache.get(target_id)
                    velocity = [0.0, 0.0, 0.0]
                    if prev is not None:
                        dt = seen - prev["seen_at"]
                        if 0.1 <= dt <= 2.5:
                            velocity = [(center[j] - prev["position"][j]) / dt for j in range(3)]
                            velocity = [max(-25.0, min(25.0, val)) for val in velocity]
                    observer_cache[target_id] = {
                        "object_id": target_id, "kind": kind, "position": center,
                        "velocity": velocity, "distance": distance, "points": npoints,
                        "observer_id": int(observer_id), "seen_at": seen,
                    }
                    # If ego has no fresh direct state for a connected vehicle, a
                    # peer that sees it may share an observed-object report too.
                    ego_knows_target = False
                    if target_id != self.ego_id:
                        direct = self.rx_states.get(self.ego_id, {}).get(target_id)
                        ego_knows_target = bool(direct and seen - direct.get("rx_time", 0.0) <= STATE_STALE_S)
                    connected_but_missing_at_ego = (target_id in self.v2x_sensors and target_id != self.ego_id and not ego_knows_target)
                    close_priority = ((kind == "p" and distance <= 25.0) or distance <= 12.0)
                    # A peer observation may bridge an ego-radio gap for a connected
                    # vehicle. This is a new observed-object report, not forwarding
                    # the original packet. Bound to one urgent report/reporter/second.
                    if connected_but_missing_at_ego or (target_id not in self.v2x_sensors and close_priority):
                        self.urgent_pending.add(int(observer_id))
        except Exception:
            # A sensor callback must never stall CARLA's UI loop.
            return

    # ---------------------------- V2X transmit --------------------------------
    def _effective_rates(self, count):
        count = max(1, int(count))
        if count <= 10:
            state_cap, object_cap = 6.0, 1.5
        elif count <= 20:
            state_cap, object_cap = 3.5, 0.75
        elif count <= 35:
            state_cap, object_cap = 2.0, 0.4
        else:
            # CARLA custom V2X is broadcast: receiver callback load grows roughly
            # with sender-count × receiver-count. Keep the default modest at 50 cars.
            state_cap, object_cap = 1.5, 0.25
        return min(max(0.5, self.args.state_hz), state_cap), min(max(0.1, self.args.object_hz), object_cap)

    def _send_json(self, sensor, payload, sender_id):
        message = json.dumps(payload, separators=(",", ":"))
        if len(message.encode("utf-8")) > MAX_MSG_BYTES:
            for key in ("p", "v", "c"):
                if key in payload:
                    payload[key] = [compact_num(x, 0) for x in payload[key]]
            message = json.dumps(payload, separators=(",", ":"))
        if len(message.encode("utf-8")) > MAX_MSG_BYTES:
            self.tx_counts[(sender_id, "oversize")] += 1
            return False
        try:
            # String send matches the working CARLA 0.9.16 custom-V2X prototype.
            sensor.send(message)
            self.tx_counts[(sender_id, payload.get("t", "?"))] += 1
            return True
        except Exception as exc:
            if time.monotonic() - getattr(self, "last_tx_error", 0.0) > 3.0:
                self.say(f"[V2X TX WARNING] vehicle={sender_id}: {exc}")
                self.last_tx_error = time.monotonic()
            return False

    def transmit_states(self, now):
        """Spread message sends across UI frames; avoid 50-sensor bursts after a hiccup."""
        count = max(1, len(self.v2x_sensors))
        state_hz, object_hz = self._effective_rates(count)
        interval = 1.0 / state_hz
        ids = list(self.v2x_sensors)
        if not ids:
            return
        due = [aid for aid in ids if now >= self.next_state_tx.get(aid, 0.0)]
        due.sort(key=lambda aid: self.next_state_tx.get(aid, 0.0))
        budget = 8 if count <= 10 else 5 if count <= 25 else 3
        for vehicle_id in due[:budget]:
            sensor = self.v2x_sensors.get(vehicle_id)
            vehicle = self.vehicle_refs.get(vehicle_id)
            if not safe_alive(vehicle) or not safe_alive(sensor):
                self.next_state_tx[vehicle_id] = now + interval
                continue
            try:
                tf = vehicle.get_transform()
                vel = vehicle.get_velocity()
                ctl = vehicle.get_control()
                self.seq_by_vehicle[vehicle_id] = (self.seq_by_vehicle[vehicle_id] + 1) % 1000
                payload = {
                    "t": "s", "i": int(vehicle_id), "q": int(self.seq_by_vehicle[vehicle_id]),
                    "p": [compact_num(tf.location.x), compact_num(tf.location.y), compact_num(tf.location.z)],
                    "v": [compact_num(vel.x), compact_num(vel.y)],
                    "h": int(round(tf.rotation.yaw)),
                    "c": [compact_num(ctl.steer, 2), compact_num(ctl.throttle, 2), compact_num(ctl.brake, 2)],
                }
                self._send_json(sensor, payload, vehicle_id)
            except Exception as exc:
                if time.monotonic() - getattr(self, "last_state_error", 0.0) > 3.0:
                    self.say(f"[STATE ERROR] vehicle={vehicle_id}: {exc}")
                    self.last_state_error = time.monotonic()
            self.next_state_tx[vehicle_id] = now + interval
        self.last_transmit_hz = (state_hz, object_hz)

    def transmit_object_reports(self, now):
        """Send remote-relevant detections; urgent pedestrians/nearby obstacles bypass the slow baseline interval."""
        count = max(1, len(self.v2x_sensors))
        _, object_hz = self._effective_rates(count)
        interval = 1.0 / object_hz
        # Avoid copying every per-observer detection dictionary on every UI frame.
        due = [sid for sid in self.v2x_sensors
               if now >= self.next_object_tx.get(sid, 0.0)
               or (sid in self.urgent_pending and now >= self.next_urgent_object_tx.get(sid, 0.0))]
        if not due:
            return
        due.sort(key=lambda sid: min(self.next_object_tx.get(sid, now), self.next_urgent_object_tx.get(sid, now)))
        # Object reports are less frequent; cap burst size to keep the UI responsive.
        due = due[:2 if len(self.v2x_sensors) > 10 else 4]
        with self.state_lock:
            snapshot = {observer: dict(self.local_detections.get(observer, {})) for observer in due}
            ego_states_snapshot = dict(self.rx_states.get(self.ego_id, {}))
            ego_local_snapshot = dict(self.local_detections.get(self.ego_id, {}))
        for sender_id in due:
            sensor = self.v2x_sensors.get(sender_id)
            if not safe_alive(sensor):
                continue
            detections = snapshot.get(sender_id, {})
            # Share pedestrians and unconnected objects. Also share a connected vehicle
            # when ego has neither a fresh direct state packet nor a fresh direct LiDAR
            # observation for it. This is essential for cooperative awareness when a
            # connected car is hidden from ego or its direct V2X state is stale/missing.
            candidates = []
            for detection in detections.values():
                target_id = int(detection["object_id"])
                if now - detection["seen_at"] > OBJECT_STALE_S or target_id == sender_id:
                    continue
                local = ego_local_snapshot.get(target_id)
                ego_detects_directly = bool(local and now - local.get("seen_at", 0.0) <= LIDAR_LOS_FRESH_S)
                if ego_detects_directly:
                    continue  # don't spend radio budget duplicating ego's current sensor evidence
                is_connected_vehicle = detection["kind"] == "v" and target_id in self.v2x_sensors
                if is_connected_vehicle:
                    direct = ego_states_snapshot.get(target_id)
                    direct_fresh = bool(direct and now - direct.get("rx_time", 0.0) <= STATE_STALE_S)
                    if direct_fresh:
                        continue  # its own fresh V2V state already gives ego this actor's position
                candidates.append(detection)
            if not candidates:
                with self.state_lock:
                    self.urgent_pending.discard(sender_id)
                if now >= self.next_object_tx.get(sender_id, 0.0):
                    self.next_object_tx[sender_id] = now + interval
                continue

            # Urgent reports: pedestrians within 25 m or any unknown obstacle/vehicle within 12 m.
            # Each reporter is still capped at one urgent broadcast per second to limit N² radio load.
            urgent = [d for d in candidates
                      if (d["kind"] == "p" and float(d["distance"]) <= 25.0)
                      or float(d["distance"]) <= 12.0]
            urgent_slot_open = now >= self.next_urgent_object_tx.get(sender_id, 0.0)
            send_urgent = bool(urgent) and urgent_slot_open
            if send_urgent:
                urgent.sort(key=lambda d: (0 if d["kind"] == "p" else 1, d["distance"], d["object_id"]))
                d = urgent[0]
                urgent_gap = 0.5 if count <= 10 else 0.75 if count <= 20 else 1.0
                self.next_urgent_object_tx[sender_id] = now + urgent_gap
                self.next_object_tx[sender_id] = max(self.next_object_tx.get(sender_id, 0.0), now + urgent_gap)
            else:
                if now < self.next_object_tx.get(sender_id, 0.0):
                    continue
                if urgent:
                    # A normal scheduled transmission can carry a hazard flag even if the urgent slot is cooling down.
                    urgent.sort(key=lambda d: (0 if d["kind"] == "p" else 1, d["distance"], d["object_id"]))
                    d = urgent[0]
                    send_urgent = True
                    urgent_gap = 0.5 if count <= 10 else 0.75 if count <= 20 else 1.0
                    self.next_urgent_object_tx[sender_id] = now + urgent_gap
                else:
                    candidates.sort(key=lambda d: (0 if d["kind"] == "p" else 1, d["distance"], d["object_id"]))
                    cursor = self.object_cursor[sender_id] % len(candidates)
                    d = candidates[cursor]
                    self.object_cursor[sender_id] = (cursor + 1) % max(1, len(candidates))
                self.next_object_tx[sender_id] = now + max(interval, urgent_gap if send_urgent else interval)
            with self.state_lock:
                self.urgent_pending.discard(sender_id)

            self.object_seq_by_vehicle[sender_id] = (self.object_seq_by_vehicle[sender_id] + 1) % 1000
            kind = "p" if d["kind"] == "p" else "v"
            packet_type = "h" if send_urgent else "o"
            payload = {
                "t": packet_type, "i": int(sender_id), "q": int(self.object_seq_by_vehicle[sender_id]),
                "d": int(d["object_id"]), "k": kind,
                "p": [compact_num(d["position"][0]), compact_num(d["position"][1]), compact_num(d["position"][2])],
                "v": [compact_num(d["velocity"][0]), compact_num(d["velocity"][1])],
                # Sensor age travels with the report so an old detection does not look brand new at RX.
                "a": min(9999, max(0, int((now - float(d["seen_at"])) * 1000.0))),
            }
            self._send_json(sensor, payload, sender_id)

    def _tick_network(self, now):
        self._sync_actor_cache()
        self._drain_los_results()
        self._schedule_los_checks(now)
        # Spread RPC sends across frames using per-sender due times; never send a fleet in one burst.
        self.transmit_states(now)
        self.transmit_object_reports(now)

    # ------------------------------ LOS ---------------------------------------
    def _valid_live_actor(self, actor_id, expected_kind=None):
        """A track ID must still correspond to a live CARLA vehicle/walker actor."""
        aid = int(actor_id)
        actor = self.actor_refs.get(aid)
        if not safe_alive(actor):
            return False
        kind = self.actor_kinds.get(aid)
        if expected_kind in ("v", "p") and kind != expected_kind:
            return False
        return kind in ("v", "p")

    def _track_for_los(self, actor_id, now):
        """Return a current live node, or None. NLOS classification is forbidden without this."""
        aid = int(actor_id)
        for node in self.mesh_nodes:
            if int(node.get("id", -1)) != aid:
                continue
            kind = node.get("kind", "v")
            observed_at = float(node.get("track_seen_at", now))
            ttl = STATE_STALE_S if node.get("track_type") == "state" else OBJECT_STALE_S
            if now - observed_at > ttl:
                return None
            if kind in ("v", "p") and not self._valid_live_actor(aid, kind):
                return None
            return node
        return None

    def _drain_los_results(self):
        now = time.monotonic()
        ego_state = self._ego_state()
        ego_pos = ego_state["position"] if ego_state else None
        while True:
            try:
                actor_id, visible, checked_at, detail, start, end = self.los_worker.results.get_nowait()
            except queue.Empty:
                break
            aid = int(actor_id)
            node = self._track_for_los(aid, now)
            if node is None or ego_pos is None or not self._valid_live_actor(aid, node.get("kind")):
                self.los_cache.pop(aid, None)
                continue
            current_target = node.get("position", [0.0, 0.0, 0.0])
            # The ray is usable only if neither endpoint has moved materially since it was queued.
            if math.dist(tuple(float(x) for x in current_target[:3]), tuple(float(x) for x in end[:3])) > 3.0:
                continue
            if math.dist(tuple(float(x) for x in ego_pos[:3]), tuple(float(x) for x in start[:3])) > 4.0:
                continue
            if checked_at < float(node.get("track_seen_at", 0.0)) - 0.05:
                continue
            self.los_cache[aid] = {
                "visible": visible, "checked_at": checked_at, "detail": detail,
                "ego_position": tuple(float(x) for x in start[:3]),
                "target_position": tuple(float(x) for x in end[:3]),
                "track_seen_at": float(node.get("track_seen_at", checked_at)),
            }

    def _schedule_los_checks(self, now):
        """Schedule ray checks only for fresh, actor-validated ego-relative tracks."""
        if now - self.last_los_schedule < 0.10:
            return
        self.last_los_schedule = now
        if not safe_alive(self.ego) or not self.use_cast_ray:
            return
        ego_tf = self.ego.get_transform()
        origin = ego_tf.transform(carla.Location(x=1.0, y=0.0, z=1.6))
        ego_pos = (ego_tf.location.x, ego_tf.location.y, ego_tf.location.z)
        vel = self.ego.get_velocity()
        ego_speed = math.hypot(vel.x, vel.y)
        radius = min(self.args.max_range, max(self.args.min_range, self.args.min_range + ego_speed * self.args.range_seconds))
        candidates = self._build_mesh_nodes(now, radius)
        # Save a fresh snapshot so result validation checks exactly the observed tracks.
        self.mesh_nodes = candidates
        candidates.sort(key=lambda node: math.hypot(float(node["position"][0]) - ego_pos[0], float(node["position"][1]) - ego_pos[1]))
        with self.state_lock:
            local_snapshot = dict(self.local_detections.get(self.ego_id, {}))
        for node in candidates:
            aid = int(node["id"])
            if aid == self.ego_id or node.get("kind") not in ("v", "p"):
                continue
            if not self._valid_live_actor(aid, node.get("kind")):
                self.los_cache.pop(aid, None)
                continue
            track_seen_at = float(node.get("track_seen_at", 0.0))
            ttl = STATE_STALE_S if node.get("track_type") == "state" else OBJECT_STALE_S
            if now - track_seen_at > ttl:
                self.los_cache.pop(aid, None)
                continue
            local = local_snapshot.get(aid)
            if local and now - local.get("seen_at", 0.0) <= LIDAR_LOS_FRESH_S:
                self.los_cache[aid] = {
                    "visible": True, "checked_at": now, "detail": "fresh ego semantic-LiDAR hit",
                    "ego_position": tuple(ego_pos), "target_position": tuple(node["position"][:3]),
                    "track_seen_at": float(local.get("seen_at", now)),
                }
                continue
            old = self.los_cache.get(aid)
            if old:
                valid_cache = (
                    old.get("visible") is not None and
                    now - old.get("checked_at", 0.0) < LOS_CACHE_S and
                    math.dist(tuple(old.get("target_position", (1e9, 1e9, 1e9))), tuple(node["position"][:3])) <= 3.0 and
                    math.dist(tuple(old.get("ego_position", (1e9, 1e9, 1e9))), tuple(ego_pos)) <= 4.0 and
                    now - float(node.get("track_seen_at", 0.0)) <= ttl
                )
                if valid_cache:
                    continue
                self.los_cache.pop(aid, None)
            p = node["position"]
            target_radius = self.actor_radii.get(aid, 0.8 if node.get("kind") == "p" else 2.5)
            end = (float(p[0]), float(p[1]), float(p[2]) + (0.6 if node.get("kind") == "p" else 0.9))
            self.los_worker.submit(aid, (origin.x, origin.y, origin.z), end, target_radius)

    def _ego_lidar_recent(self, actor_id, now):
        if self.args.no_lidar or self.ego_id is None:
            return False
        with self.state_lock:
            obs = self.local_detections.get(self.ego_id, {}).get(int(actor_id))
            return bool(obs and now - obs.get("seen_at", 0.0) <= LIDAR_LOS_FRESH_S and
                        self._valid_live_actor(actor_id, obs.get("kind")))

    def _node_los(self, actor_id, now):
        """Never declare NLOS from absence alone; require fresh live track + matching blocked ray."""
        aid = int(actor_id)
        node = self._track_for_los(aid, now)
        if node is None:
            self.los_cache.pop(aid, None)
            return "CHECK", AMBER, "no fresh valid live target track"
        if self._ego_lidar_recent(aid, now):
            return "LOS", GREEN, "fresh ego semantic-LiDAR hit"
        cached = self.los_cache.get(aid)
        if not cached or cached.get("visible") is None:
            return "CHECK", AMBER, "awaiting scene-geometry ray check"
        ttl = STATE_STALE_S if node.get("track_type") == "state" else OBJECT_STALE_S
        valid = (
            now - cached.get("checked_at", 0.0) <= LOS_CACHE_S and
            now - float(node.get("track_seen_at", 0.0)) <= ttl and
            math.dist(tuple(cached.get("target_position", (1e9,1e9,1e9))), tuple(node["position"][:3])) <= 3.0 and
            safe_alive(self.ego) and
            math.dist(tuple(cached.get("ego_position", (1e9,1e9,1e9))),
                      (self.ego.get_location().x, self.ego.get_location().y, self.ego.get_location().z)) <= 4.0 and
            self._valid_live_actor(aid, node.get("kind"))
        )
        if not valid:
            return "CHECK", AMBER, "track/ray evidence changed; rechecking"
        if cached.get("visible") is True:
            return "LOS", GREEN, cached.get("detail", "scene ray clear")
        if cached.get("visible") is False:
            return "NLOS", RED, cached.get("detail", "fresh scene ray confirms obstruction")
        return "CHECK", AMBER, "visibility unresolved"

    # --------------------------- ego / telemetry ------------------------------
    def _ego_state(self):
        if not safe_alive(self.ego):
            return None
        tf = self.ego.get_transform()
        vel = self.ego.get_velocity()
        acc = self.ego.get_acceleration()
        ctl = self.ego.get_control()
        v = [vel.x, vel.y, vel.z]
        a = [acc.x, acc.y, acc.z]
        forward = tf.get_forward_vector()
        right = tf.get_right_vector()
        long_a = acc.x * forward.x + acc.y * forward.y + acc.z * forward.z
        lat_a = acc.x * right.x + acc.y * right.y + acc.z * right.z
        return {
            "id": self.ego_id, "position": [tf.location.x, tf.location.y, tf.location.z],
            "velocity": v, "acceleration": a, "heading": float(tf.rotation.yaw),
            "controls": [ctl.steer, ctl.throttle, ctl.brake], "long": long_a, "lat": lat_a,
            "rx_time": time.monotonic(), "power": None, "source": "EGO LOCAL SENSOR/STATE",
        }

    def _manual_control_tick(self, now):
        """Read W/S/A/D or arrows while THIS dashboard window has focus and control only ego."""
        if not self.manual_control_enabled or not safe_alive(self.ego) or self.manual_autopilot:
            self.last_manual_tick = now
            return
        dt = max(0.0, min(0.10, now - self.last_manual_tick))
        self.last_manual_tick = now
        try:
            focused = bool(pygame.key.get_focused())
        except Exception:
            focused = True
        if not focused:
            # Release held throttle if the user switches to another window.
            if self.manual_focus_active:
                try:
                    self.ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.35))
                except Exception:
                    pass
            self.manual_focus_active = False
            self.manual_steer += (0.0 - self.manual_steer) * min(1.0, dt * 5.0)
            return

        self.manual_focus_active = True
        keys = pygame.key.get_pressed()
        left = bool(keys[pygame.K_a] or keys[pygame.K_LEFT])
        right = bool(keys[pygame.K_d] or keys[pygame.K_RIGHT])
        forward = bool(keys[pygame.K_w] or keys[pygame.K_UP])
        braking = bool(keys[pygame.K_s] or keys[pygame.K_DOWN])
        speed_ms = 0.0
        try:
            vel = self.ego.get_velocity()
            speed_ms = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
        except Exception:
            pass
        steer_limit = max(0.35, 0.62 - min(speed_ms, 35.0) * 0.006)
        steer_target = -steer_limit if left and not right else steer_limit if right and not left else 0.0
        self.manual_steer += (steer_target - self.manual_steer) * min(1.0, dt * 6.0)
        throttle = 0.0 if braking else (0.62 if forward else 0.0)
        brake = 0.70 if braking else 0.0
        hand_brake = bool(keys[pygame.K_SPACE])
        # 30 Hz vehicle-control RPCs are responsive while avoiding an unnecessary call every UI frame.
        if now - self.last_control_send >= (1.0 / 30.0):
            try:
                self.ego.apply_control(carla.VehicleControl(
                    throttle=throttle,
                    steer=max(-1.0, min(1.0, self.manual_steer)),
                    brake=brake,
                    hand_brake=hand_brake,
                    reverse=self.manual_reverse,
                ))
                self.last_control_send = now
            except Exception as exc:
                if now - getattr(self, "last_control_error", 0.0) > 3.0:
                    self.say(f"[CONTROL ERROR] {exc}")
                    self.last_control_error = now

    def _record_ego_sample(self, now):
        if self.ego_id is None or now - self.last_ego_sample < 0.1:
            return
        self.last_ego_sample = now
        st = self._ego_state()
        if st is None:
            return
        p, v, a, c = st["position"], st["velocity"], st["acceleration"], st["controls"]
        with self.state_lock:
            self.vehicle_history[self.ego_id].append({
                "t": now - self.started, "speed": speed3(v)*3.6,
                "accel": math.sqrt(a[0]**2 + a[1]**2 + a[2]**2), "steer": c[0], "throttle": c[1], "brake": c[2],
                "long": st["long"], "lat": st["lat"], "yaw": st["heading"],
                "pitch": float(self.ego.get_transform().rotation.pitch), "roll": float(self.ego.get_transform().rotation.roll),
                "x": p[0], "y": p[1], "z": p[2], "source": "EGO LOCAL"
            })

    def _state_for_selected(self):
        if self.selected_id == self.ego_id:
            return self._ego_state()
        now = time.monotonic()
        with self.state_lock:
            remote = self.rx_states.get(self.ego_id, {}).get(self.selected_id)
            obj = self.rx_objects.get(self.ego_id, {}).get(self.selected_id)
        if remote and now - remote["rx_time"] <= STATE_STALE_S:
            state = dict(remote)
            state["source"] = "V2V RECEIVED"
            state["age"] = now - remote["rx_time"]
            return state
        if obj and now - obj["rx_time"] <= OBJECT_STALE_S:
            return {
                "id": self.selected_id, "position": obj["position"], "velocity": obj["velocity"],
                "heading": 0.0, "controls": [0.0, 0.0, 0.0], "acceleration": [0.0, 0.0, 0.0],
                "rx_time": obj["rx_time"], "power": obj.get("power"), "source": f"COOPERATIVE OBJECT REPORT FROM V{obj['reporter_id']}",
                "age": now - obj["rx_time"],
            }
        with self.state_lock:
            local = self.local_detections.get(self.ego_id, {}).get(self.selected_id)
        if local and now - local["seen_at"] <= OBJECT_STALE_S:
            return {
                "id": self.selected_id, "position": local["position"], "velocity": local["velocity"],
                "heading": 0.0, "controls": [0.0, 0.0, 0.0], "acceleration": [0.0, 0.0, 0.0],
                "rx_time": local["seen_at"], "power": None, "source": "EGO SEMANTIC-LIDAR TRACK",
                "age": now - local["seen_at"],
            }
        return None

    def _get_ego_rx_snapshot(self):
        now = time.monotonic()
        with self.state_lock:
            states = dict(self.rx_states.get(self.ego_id, {}))
            objects = dict(self.rx_objects.get(self.ego_id, {}))
            local_ego = dict(self.local_detections.get(self.ego_id, {}))
            links = dict(self.radio_links)
        return now, states, objects, local_ego, links

    def _build_mesh_nodes(self, now=None, radius=None):
        """Build ego mesh only from fresh V2V/local observations of currently live CARLA actors."""
        if now is None:
            now = time.monotonic()
        if radius is None:
            radius = self.active_view_range
        if self.ego_id is None or not safe_alive(self.ego):
            return []
        ego_state = self._ego_state()
        if ego_state is None:
            return []
        ego_pos = ego_state["position"]
        nodes = {
            self.ego_id: {
                "id": self.ego_id, "kind": "v", "position": ego_pos,
                "velocity": ego_state["velocity"], "source": "EGO",
                "age": 0.0, "track_seen_at": now, "track_type": "state"
            }
        }
        _, states, objects, local_obs, _ = self._get_ego_rx_snapshot()
        # Remote states are accepted only if their source actor still exists and the packet is fresh.
        for aid, state in states.items():
            aid = int(aid)
            age = now - float(state.get("rx_time", 0.0))
            if age > STATE_STALE_S or not self._valid_live_actor(aid, "v"):
                continue
            p = list(state["position"])
            while len(p) < 3:
                p.append(0.0)
            v = list(state.get("velocity", [0.0, 0.0]))
            while len(v) < 3:
                v.append(0.0)
            # Short bounded prediction reduces apparent lag without moving long-stale tracks.
            predict_dt = min(max(age, 0.0), 0.45)
            predicted = [p[0] + v[0] * predict_dt, p[1] + v[1] * predict_dt, p[2]]
            if math.dist((predicted[0], predicted[1]), (ego_pos[0], ego_pos[1])) <= radius:
                nodes[aid] = {
                    "id": aid, "kind": "v", "position": predicted, "reported_position": p,
                    "velocity": v, "source": "V2V RX", "age": age,
                    "track_seen_at": float(state.get("rx_time", 0.0)), "track_type": "state",
                    "power": state.get("power"), "reporter_id": int(aid)
                }
        # Recent local semantic-LiDAR tracks can add directly sensed vehicles/pedestrians.
        for aid, obs in local_obs.items():
            aid = int(aid)
            age = now - float(obs.get("seen_at", 0.0))
            if age > LIDAR_LOS_FRESH_S or not self._valid_live_actor(aid, obs.get("kind")):
                continue
            p = list(obs["position"])
            if math.dist((p[0], p[1]), (ego_pos[0], ego_pos[1])) <= radius:
                nodes[aid] = {
                    "id": aid, "kind": obs["kind"], "position": p, "velocity": obs["velocity"],
                    "source": "EGO LIDAR", "age": age, "track_seen_at": float(obs["seen_at"]),
                    "track_type": "object", "reporter_id": self.ego_id
                }
        # Cooperative object reports are tracks, not eternal markers: observation age is what expires them.
        for aid, report in objects.items():
            aid = int(aid)
            observed_at = float(report.get("observed_at", report.get("rx_time", 0.0)))
            age = now - observed_at
            kind = report.get("kind", "v")
            reporter_id = int(report.get("reporter_id", -1))
            if (age > OBJECT_STALE_S or not self._valid_live_actor(aid, kind) or
                    not self._valid_live_actor(reporter_id, "v")):
                continue
            p = list(report["position"])
            rv = list(report.get("velocity", [0.0, 0.0]))
            while len(rv) < 2:
                rv.append(0.0)
            # Short bounded extrapolation is used only for display smoothing; an old report still expires.
            lead = min(max(age, 0.0), 0.8)
            p[0] += float(rv[0]) * lead
            p[1] += float(rv[1]) * lead
            if math.dist((p[0], p[1]), (ego_pos[0], ego_pos[1])) > radius:
                continue
            # Never override a fresher local sensor or state track with an older remote observation.
            old = nodes.get(aid)
            if old is not None and float(old.get("track_seen_at", 0.0)) >= observed_at:
                continue
            nodes[aid] = {
                "id": aid, "kind": kind, "position": p, "velocity": report["velocity"],
                "source": f"SHARED BY V{report['reporter_id']}", "age": age,
                "track_seen_at": observed_at, "track_type": "object",
                "reporter_id": int(report["reporter_id"]), "power": report.get("power"),
                "hazard": report.get("hazard", False)
            }
        # Expire cached LOS verdicts for IDs no longer represented by a fresh live track.
        live_ids = set(nodes)
        for aid in list(self.los_cache):
            if aid not in live_ids:
                self.los_cache.pop(aid, None)
        return list(nodes.values())

    # ------------------------------- camera -----------------------------------
    def _update_spectator(self, now):
        if self.args.no_spectator_follow or not safe_alive(self.camera_sensor):
            return
        connected = max(1, len(self.v2x_sensors))
        requested = max(1.0, float(self.args.spectator_hz))
        budget = 20.0 if connected <= 10 else 15.0 if connected <= 20 else 10.0
        effective_hz = min(requested, budget)
        if now - self.last_spectator_update < 1.0 / effective_hz:
            return
        try:
            # CARLA's original 3D viewport follows this same sensor camera pose.
            self.spectator.set_transform(self.camera_sensor.get_transform())
            self.last_spectator_update = now
        except Exception as exc:
            self.camera_error = str(exc)

    def _consume_camera_frame(self, width, height):
        with self.latest_camera_lock:
            packed = self.latest_camera
        if packed is None:
            return
        frame_no, raw, transform, src_w, src_h, captured_at = packed
        if frame_no == self.camera_frame_number and self.camera_surface is not None:
            return
        try:
            # Native BGRA surface path avoids multiple NumPy/channel-order copies.
            try:
                surf = pygame.image.frombuffer(raw, (src_w, src_h), "BGRA").copy()
            except Exception:
                rgba = np.frombuffer(raw, dtype=np.uint8).reshape((src_h, src_w, 4))
                rgb = rgba[:, :, :3][:, :, ::-1].copy()
                surf = pygame.surfarray.make_surface(np.swapaxes(rgb, 0, 1))
            if surf.get_size() != (width, height):
                surf = pygame.transform.scale(surf, (width, height))
            self.camera_surface = surf
            self.camera_display_surface = None
            self.camera_display_size = None
            self.camera_display_frame_number = -1
            self.camera_frame_number = frame_no
            self.latest_image_transform = transform
            self.camera_frame_age = max(0.0, time.monotonic() - captured_at)
        except Exception as exc:
            self.camera_error = f"Frame conversion: {exc}"

    # ------------------------------- drawing ---------------------------------
    def _font(self, size):
        return self.fonts.get(size) or self.fonts[15]

    def _label(self, surface, string, pos, color=WHITE, size=13, center=False):
        img = self._font(size).render(str(string), True, color)
        rect = img.get_rect(center=pos) if center else img.get_rect(topleft=pos)
        surface.blit(img, rect)
        return rect

    def _draw_dashed_circle(self, surface, center, radius, color, width=2, segments=10):
        x, y = center
        bounds = pygame.Rect(x-radius, y-radius, radius*2, radius*2)
        step = (2.0 * math.pi) / segments
        for i in range(segments):
            start = i * step + 0.12
            end = (i + 0.62) * step
            pygame.draw.arc(surface, color, bounds, start, end, width)

    def _panel(self, rect, title):
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=8)
        pygame.draw.rect(self.screen, GRID, rect, width=1, border_radius=8)
        self._label(self.screen, title, (rect.x + 12, rect.y + 8), CYAN, 15)

    def _layout(self):
        w, h = self.screen.get_size()
        header_h = 42
        content_h = int((h - header_h) * 0.60)
        left_w = int(w * 0.625)
        gap = 6
        camera_rect = pygame.Rect(6, header_h, left_w - 9, content_h - 6)
        side_x = left_w + 2
        side_w = w - side_x - 6
        mesh_rect = pygame.Rect(side_x, header_h, side_w, int(content_h * 0.62) - 6)
        info_rect = pygame.Rect(side_x, mesh_rect.bottom + 6, side_w, content_h - (mesh_rect.height + 12))
        graph_rect = pygame.Rect(6, header_h + content_h, w - 12, h - (header_h + content_h) - 6)
        return w, h, camera_rect, mesh_rect, info_rect, graph_rect

    def _draw_camera_panel(self, rect, now):
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=8)
        inner_size = (max(1, rect.width - 4), max(1, rect.height - 4))
        if self.camera_surface is not None:
            # Scale only when a new camera frame arrives or the window is resized.
            if (self.camera_display_surface is None or
                    self.camera_display_size != inner_size or
                    self.camera_display_frame_number != self.camera_frame_number):
                self.camera_display_surface = pygame.transform.scale(self.camera_surface, inner_size)
                self.camera_display_size = inner_size
                self.camera_display_frame_number = self.camera_frame_number
            self.screen.blit(self.camera_display_surface, (rect.x + 2, rect.y + 2))
        else:
            self._label(self.screen, "WAITING FOR LIVE CARLA CAMERA...", rect.center, AMBER, 19, center=True)
        pygame.draw.rect(self.screen, GRID, rect, 1, border_radius=8)
        camera_title = (f"SATELLITE 90°  •  ALT {self.satellite_altitude:.0f}m  •  {'FOLLOW' if self.camera_follow else 'FREE CAMERA'}" if self.camera_mode == "satellite" else "CHASE CAMERA  /  SPECTATOR MIRRORED")
        self._label(self.screen, camera_title, (rect.x + 12, rect.y + 10), WHITE, 15)
        if self.camera_mode == "satellite":
            help_text = "RIGHT-DRAG PAN • WHEEL/[/] ZOOM • Q/E ROTATE • F FOLLOW • M RECENTER"
            self._label(self.screen, help_text, (rect.x + 12, rect.bottom - 23), AMBER, 10)

        # Projection is refreshed at 10 Hz or whenever the image/layout changes. Re-drawing
        # existing labels is cheap, whereas repeated matrix inversion / track fusion was not.
        rect_key = (rect.x, rect.y, rect.width, rect.height)
        update_overlays = (
            now - self.last_camera_overlay_update >= 0.10 or
            self.camera_overlay_frame_number != self.camera_frame_number or
            self.camera_overlay_rect_key != rect_key
        )
        if update_overlays:
            items = []
            picks = []
            if self.camera_surface is not None and self.latest_image_transform is not None:
                for node in self.mesh_nodes:
                    if node["id"] == self.ego_id:
                        continue
                    status, los_color, _ = self._node_los(node["id"], now)
                    if status == "CHECK":
                        continue
                    pix = camera_project(node["position"], self.latest_image_transform,
                                         self.args.camera_width, self.args.camera_height, self.args.camera_fov)
                    if pix is None:
                        continue
                    px = rect.x + 2 + int(pix[0] * inner_size[0] / self.args.camera_width)
                    py = rect.y + 2 + int(pix[1] * inner_size[1] / self.args.camera_height)
                    if rect.collidepoint(px, py):
                        is_ped = node.get("kind") == "p"
                        col = (MAGENTA if status == "NLOS" else ORANGE) if is_ped else los_color
                        label = f"P{node['id']} EGO-{status}" if is_ped else f"V{node['id']} EGO-{status}"
                        label_surface = self._font(11).render(label, True, col)
                        items.append((int(node["id"]), (px, py), col, status, is_ped, label_surface))
                        if not is_ped:
                            picks.append((int(node["id"]), (px, py)))
            self.camera_overlay_items = items
            self.camera_pick_points = picks
            self.camera_overlay_frame_number = self.camera_frame_number
            self.camera_overlay_rect_key = rect_key
            self.last_camera_overlay_update = now

        for aid, point, color, status, is_ped, label_surface in self.camera_overlay_items:
            if is_ped:
                pygame.draw.polygon(self.screen, color,
                                    [(point[0], point[1]-8), (point[0]-6, point[1]+5), (point[0]+6, point[1]+5)])
                if status == "NLOS":
                    self._draw_dashed_circle(self.screen, point, 11, color, 2, segments=8)
            elif status == "NLOS":
                # A dashed ring means cooperatively known but geometrically occluded from ego.
                self._draw_dashed_circle(self.screen, point, 11, RED, 2, segments=10)
            else:
                pygame.draw.circle(self.screen, color, point, 9, 2)
            self.screen.blit(label_surface, (point[0] + 9, point[1] - 12))

        speed_kmh = self.ego_speed_kmh
        badge = pygame.Rect(rect.right - 250, rect.y + 8, 236, 29)
        pygame.draw.rect(self.screen, (6, 12, 19), badge, border_radius=6)
        self._label(self.screen, f"EGO {self.ego_id}   {speed_kmh:4.1f} km/h", (badge.x + 10, badge.y + 6), CYAN, 14)

    def _map_point(self, node_pos, ego_pos, yaw, rect, radius):
        fwd, lateral = world_to_ego_xy(node_pos[:2], ego_pos[:2], yaw)
        scale_x = (rect.width * 0.43) / max(radius, 1.0)
        scale_y = (rect.height * 0.39) / max(radius, 1.0)
        return int(rect.centerx + lateral * scale_x), int(rect.centery - fwd * scale_y)

    def _draw_mesh_panel(self, rect, now):
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=8)
        self._label(self.screen, "COOPERATIVE MESH  /  EGO-RELATIVE", (rect.x + 10, rect.y + 7), CYAN, 15)
        if not safe_alive(self.ego):
            self._label(self.screen, "WAITING FOR EGO...", rect.center, AMBER, 15, center=True)
            return
        tf = self.ego.get_transform()
        vel = self.ego.get_velocity()
        self.ego_speed_kmh = math.hypot(vel.x, vel.y) * 3.6
        ego_pos = [tf.location.x, tf.location.y, tf.location.z]
        yaw = math.radians(tf.rotation.yaw)
        self.active_view_range = min(self.args.max_range, max(self.args.min_range, math.hypot(vel.x, vel.y) * self.args.range_seconds + self.args.min_range))
        # Leave header area for title/status; mesh plot is the rest. The map/lane
        # background is spatially indexed and cached; only dynamic nodes/links are
        # redrawn each UI update.
        plot = rect.inflate(-12, -58)
        plot.y += 35
        plot.height -= 24
        radius = self.active_view_range
        lane_bg = self._get_lane_background(plot.size, ego_pos, yaw, radius)
        self.screen.blit(lane_bg, plot.topleft)
        center = plot.center
        nodes = self._build_mesh_nodes(now, radius)
        self.mesh_nodes = nodes
        node_by_id = {n["id"]: n for n in nodes}
        self.mesh_pick_points = []
        # Only show communication edges observed at ego's V2X receiver, not distance-inferred links.
        with self.state_lock:
            links = dict(self.radio_links)
        ego_point = self._map_point(ego_pos, ego_pos, yaw, plot, radius)
        for node in nodes:
            aid = int(node["id"])
            if aid == self.ego_id:
                continue
            link = links.get((self.ego_id, aid))
            if link and now - link["rx_time"] <= LINK_STALE_S and aid in node_by_id:
                pt = self._map_point(node["position"], ego_pos, yaw, plot, radius)
                power = link.get("power")
                color = GREEN if power is not None and power > -65 else CYAN if power is not None and power > -78 else AMBER if power is not None and power > -88 else RED
                pygame.draw.line(self.screen, color, ego_point, pt, 1)
        # draw direct detections and cooperative tracks
        for node in nodes:
            aid = int(node["id"])
            if aid == self.ego_id:
                continue
            dist = math.dist(node["position"][:2], ego_pos[:2])
            if dist > radius:
                continue
            pt = self._map_point(node["position"], ego_pos, yaw, plot, radius)
            status, col, detail = self._node_los(aid, now)
            is_ped = node.get("kind") == "p"
            color = MAGENTA if is_ped and status == "NLOS" else ORANGE if is_ped else col
            if is_ped:
                pygame.draw.polygon(self.screen, color, [(pt[0],pt[1]-8),(pt[0]-7,pt[1]+6),(pt[0]+7,pt[1]+6)])
            else:
                pygame.draw.circle(self.screen, color, pt, 7)
                pygame.draw.circle(self.screen, WHITE if aid == self.selected_id else color, pt, 10, 1 if aid != self.selected_id else 3)
            label = f"P{aid}" if is_ped else f"V{aid}"
            self._label(self.screen, label, (pt[0]+8, pt[1]-13), color, 12)
            status_text = f"EGO-{status}" if status in ("LOS", "NLOS") else "CHECK"
            self._label(self.screen, status_text, (pt[0]+8, pt[1]+1), color, 10)
            if not is_ped:
                self.mesh_pick_points.append((aid, pt))
        pygame.draw.circle(self.screen, CYAN, ego_point, 9)
        pygame.draw.polygon(self.screen, WHITE, [(ego_point[0],ego_point[1]-7),(ego_point[0]-5,ego_point[1]+5),(ego_point[0]+5,ego_point[1]+5)])
        self._label(self.screen, "EGO", (ego_point[0]+10, ego_point[1]-9), CYAN, 12)
        peers = sum(1 for n in nodes if n["id"] != self.ego_id and n.get("kind") == "v")
        lines = sum(1 for n in nodes if n["id"] != self.ego_id and links.get((self.ego_id, n["id"])) and now-links[(self.ego_id,n["id"])]["rx_time"] <= LINK_STALE_S)
        nlos_count = sum(1 for n in nodes if n["id"] != self.ego_id and self._node_los(n["id"], now)[0] == "NLOS")
        los_count = sum(1 for n in nodes if n["id"] != self.ego_id and self._node_los(n["id"], now)[0] == "LOS")
        self._label(self.screen, f"RANGE {radius:.0f}m • PEERS {peers} • EGO-LOS {los_count} • EGO-NLOS {nlos_count} • RX LINKS {lines}", (rect.x+10, rect.bottom-19), MUTED, 11)

    def _draw_info_panel(self, rect, now):
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=8)
        pygame.draw.rect(self.screen, GRID, rect, 1, border_radius=8)
        sid = self.selected_id if self.selected_id is not None else self.ego_id
        self._label(self.screen, f"SELECTED VEHICLE  /  {sid if sid is not None else '--'}", (rect.x+10,rect.y+7), CYAN, 15)
        if sid is None:
            self._label(self.screen,"No vehicle selected",(rect.x+10,rect.y+31),AMBER,13)
            return
        state = self._state_for_selected()
        status, los_color, los_detail = self._node_los(sid, now) if sid != self.ego_id else ("EGO", CYAN, "ego reference")
        kind = self.actor_kinds.get(int(sid), "v")
        typename = "PEDESTRIAN" if kind == "p" else "VEHICLE"
        if state is None:
            self._label(self.screen, f"{typename}  •  STATE NOT RECENTLY RECEIVED", (rect.x+10,rect.y+31), AMBER, 12)
            self._label(self.screen, f"EGOCENTRIC VISIBILITY: {status}", (rect.x+10,rect.y+50), los_color, 12)
            return
        p = state.get("position", [0,0,0]); v = state.get("velocity", [0,0,0]); ctl = state.get("controls", [0,0,0])
        speed = speed3(v)*3.6
        age = max(0.0, float(state.get("age", now-state.get("rx_time", now))))
        power = state.get("power")
        source = state.get("source", "unknown")
        lines = [
            (f"{typename}  •  SPEED {speed:5.1f} km/h  •  HEADING {state.get('heading',0):+.0f}°", WHITE),
            (f"POSITION  X {p[0]:+.1f}  Y {p[1]:+.1f}  Z {p[2]:+.1f} m", WHITE),
            (f"VELOCITY  VX {v[0]:+.1f}  VY {v[1]:+.1f} m/s", WHITE),
            (f"CONTROL  STEER {ctl[0]:+.2f}  THROTTLE {ctl[1]:.2f}  BRAKE {ctl[2]:.2f}", AMBER if ctl[2] > 0.1 else WHITE),
            (f"EGO VISIBILITY  {('EGO-' + status) if status in ('LOS','NLOS') else status}  •  {age*1000:.0f}ms DATA AGE", los_color),
            (f"SOURCE {source[:40]}", MUTED),
            (f"V2X RX POWER  {'—' if power is None else f'{float(power):.1f} dBm'}  •  {'EGO LOCAL' if sid == self.ego_id else 'RECEIVED/REPORTED'}", GREEN if power is not None else MUTED),
        ]
        y = rect.y + 31
        step = max(17, min(21, (rect.height - 35)//len(lines)))
        for line, color in lines:
            self._label(self.screen, line, (rect.x+10, y), color, 12)
            y += step

    def _draw_graph(self, rect, title, values, color=CYAN, second=None, limit=None):
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=5)
        pygame.draw.rect(self.screen, GRID, rect, 1, border_radius=5)
        self._label(self.screen, title, (rect.x+7, rect.y+5), MUTED, 11)
        plot = pygame.Rect(rect.x+8, rect.y+23, max(5, rect.width-16), max(5, rect.height-29))
        pygame.draw.line(self.screen, GRID, (plot.left,plot.centery),(plot.right,plot.centery),1)
        def draw_values(vals, col):
            vals = [float(v) for v in vals if v is not None and np.isfinite(float(v))]
            if len(vals) < 2:
                return
            vals = vals[-240:]
            if limit is not None:
                lo, hi = limit
            else:
                lo, hi = min(vals), max(vals)
                if abs(hi-lo) < 1e-6:
                    hi, lo = hi+1.0, lo-1.0
                pad = (hi-lo)*0.12
                lo -= pad; hi += pad
            pts=[]
            den=max(hi-lo,1e-6)
            for i,val in enumerate(vals):
                x=plot.left + int(i*(plot.width-1)/max(1,len(vals)-1))
                y=plot.bottom-1 - int((min(hi,max(lo,val))-lo)/den*(plot.height-2))
                pts.append((x,y))
            if len(pts)>1:
                pygame.draw.lines(self.screen,col,False,pts,2)
        draw_values(values,color)
        if second is not None:
            draw_values(second[0],second[1])

    def _draw_graphs(self, rect, now):
        pygame.draw.rect(self.screen, BG, rect, border_radius=7)
        self._label(self.screen, "SELECTED VEHICLE TELEMETRY  /  NINE LIVE PLOTS", (rect.x+8, rect.y+5), CYAN, 13)
        sid = self.selected_id if self.selected_id is not None else self.ego_id
        with self.state_lock:
            samples = list(self.vehicle_history.get(int(sid), [])) if sid is not None else []
        if sid == self.ego_id:
            # Ensure a current ego sample even if no ego V2X packet exists.
            pass
        left = rect.x+6; top = rect.y+25; width = (rect.width-24)//3; height=(rect.height-35)//3
        specs = [
            ("VELOCITY PROFILE / km/h", "speed", CYAN, None, None),
            ("ACCELERATION / m/s²", "accel", ORANGE, None, None),
            ("CONTROL INPUTS", "steer", MAGENTA, ("throttle", GREEN), (-1.1,1.1)),
            ("TRAJECTORY / XY", "x", CYAN, ("y", BLUE), None),
            ("ORIENTATION / YAW", "yaw", BLUE, None, (-180,180)),
            ("LONGITUDINAL / LATERAL", "long", ORANGE, ("lat", MAGENTA), None),
            ("SPEED DISTRIBUTION", "speed_hist", CYAN, None, None),
            ("DYNAMIC STATE", "dynamic", GREEN, None, (0,1)),
            ("POSITION VECTOR", "position", BLUE, None, None),
        ]
        # Use graph panels no faster than 10 Hz and draw from compact histories.
        for idx, (title,key,col,secondary,lim) in enumerate(specs):
            row, c = divmod(idx,3)
            r=pygame.Rect(left+c*width, top+row*height, width-5, height-5)
            if key == "speed_hist":
                self._draw_graph(r,title,[s.get("speed",0) for s in samples],col)
                # Overlay a small distribution histogram above the lower baseline.
                if len(samples)>5:
                    vals=np.asarray([s.get("speed",0) for s in samples[-150:]],dtype=float)
                    hist,_=np.histogram(vals,bins=12)
                    maxh=max(1,int(hist.max()))
                    plot=pygame.Rect(r.x+8,r.y+23,r.width-16,max(5,r.height-29))
                    bw=max(1,plot.width//12)
                    for j,hv in enumerate(hist):
                        bh=int((hv/maxh)*(plot.height-2))
                        pygame.draw.rect(self.screen,(0,120,145),pygame.Rect(plot.x+j*bw,plot.bottom-bh,bw-2,bh))
                continue
            if key == "dynamic":
                self._draw_graph(r,title,[s.get("speed",0)/120.0 for s in samples],col,
                                 ([abs(s.get("steer",0)) for s in samples], MAGENTA),(0,1))
                if samples:
                    s=samples[-1]
                    bx=r.x+9; by=r.y+r.height-17
                    self._label(self.screen,f"V {s.get('speed',0):.0f}  A {s.get('accel',0):.1f}  B {s.get('brake',0):.1f}",(bx,by),MUTED,10)
                continue
            if key == "position":
                self._draw_graph(r,title,[s.get("x",0) for s in samples],CYAN,([s.get("y",0) for s in samples], BLUE))
                if samples:
                    s=samples[-1]
                    self._label(self.screen,f"X {s.get('x',0):.0f} Y {s.get('y',0):.0f} Z {s.get('z',0):.1f}",(r.x+8,r.bottom-15),MUTED,10)
                continue
            vals=[s.get(key,0) for s in samples]
            sec=None
            if secondary:
                sec=([s.get(secondary[0],0) for s in samples],secondary[1])
            self._draw_graph(r,title,vals,col,sec,lim)
        if len(samples)<2:
            self._label(self.screen,"Waiting for samples for selected vehicle...",(rect.centerx,rect.centery),AMBER,15,center=True)

    def _draw_header(self, w):
        pygame.draw.rect(self.screen, (4,8,13), (0,0,w,42))
        self._label(self.screen,"COOPERATIVE ADAS",(12,11),WHITE,19)
        self._label(self.screen,"DRIVE W/S  •  STEER A/D  •  P=AI  •  R=REV  •  SPACE=HB  •  HOME=EGO",(226,13),CYAN,10)
        if self.radio_model_active:
            text_status="RF PATH-LOSS: CONFIGURED"
            color=GREEN
        else:
            text_status="RF NLOS: NOT CONFIRMED"
            color=RED
        self._label(self.screen,text_status,(w-330,13),color,13)

    def _draw_status_footer(self, w, h, now):
        fps = self.clock.get_fps()
        cam_age = getattr(self,"camera_frame_age",0.0)
        state_hz, object_hz = self._effective_rates(max(1,len(self.v2x_sensors)))
        ego_peers = 0
        if self.ego_id is not None:
            with self.state_lock:
                ego_peers = sum(now-s.get("rx_time",now) <= STATE_STALE_S for s in self.rx_states.get(self.ego_id,{}).values())
        mode = "AUTO" if self.manual_autopilot else ("MANUAL" if self.manual_control_enabled else "NO CONTROL")
        rf = "CONFIGURED" if self.radio_model_active else "NOT CONFIRMED"
        camera_mode = (f"SAT90 ALT {self.satellite_altitude:.0f}m {'FOLLOW' if self.camera_follow else 'FREE'}" if self.camera_mode == "satellite" else "CHASE")
        txt=f"UI {fps:.0f} FPS • CAM {cam_age*1000:.0f}ms • PEERS {ego_peers} • TX {state_hz:g}/{object_hz:g}Hz • RF {rf} • {camera_mode} • WASD DRIVE • RIGHT-DRAG PAN • WHEEL ZOOM • F FOLLOW • ESC EXIT"
        pygame.draw.rect(self.screen,(4,8,13),(0,h-21,w,21))
        self._label(self.screen,txt,(10,h-18),MUTED,11)

    def _pan_satellite(self, dx_pixels, dy_pixels, panel_rect):
        """Pan camera target by screen-space drag without moving the ego vehicle."""
        if self.camera_mode != "satellite":
            return
        if self.satellite_center is None:
            return
        if self.camera_follow:
            self.camera_follow = False
            if self.satellite_current_center is not None:
                self.satellite_center = list(self.satellite_current_center)
        # Approximate ground-plane metres per captured image pixel from horizontal FOV.
        image_width = max(1.0, float(panel_rect.width - 4))
        world_width = 2.0 * self.satellite_altitude * math.tan(math.radians(self.args.camera_fov) / 2.0)
        metres_per_pixel = world_width / image_width
        yaw = math.radians(self.satellite_yaw)
        # At yaw=0, screen-right maps to +Y and screen-up maps to +X.
        dx = float(dx_pixels) * metres_per_pixel
        dy = float(dy_pixels) * metres_per_pixel
        self.satellite_center[0] += math.sin(yaw) * dx + math.cos(yaw) * dy
        self.satellite_center[1] += -math.cos(yaw) * dx + math.sin(yaw) * dy
        self.satellite_current_center = list(self.satellite_center)

    def _camera_zoom(self, wheel_steps):
        if self.camera_mode != "satellite":
            return
        self.satellite_altitude = max(25.0, min(350.0, self.satellite_altitude * (0.88 ** int(wheel_steps))))
        self.say(f"[SATELLITE CAMERA] altitude={self.satellite_altitude:.1f}m")

    def _camera_recenter(self, follow=True):
        if not safe_alive(self.ego):
            return
        loc = self.ego.get_location()
        self.satellite_center = [float(loc.x), float(loc.y), float(loc.z)]
        self.satellite_current_center = list(self.satellite_center)
        self.camera_follow = bool(follow)
        self.say(f"[SATELLITE CAMERA] {'FOLLOW EGO' if self.camera_follow else 'RECENTERED / FREE'}")

    # ------------------------------ interaction ------------------------------
    def _handle_events(self, layout):
        _, _, cam_rect, mesh_rect, info_rect, graph_rect = layout
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode((max(1000,event.w),max(700,event.h)),pygame.RESIZABLE|pygame.DOUBLEBUF)
                self.panel_cache_surface = None
                self.cache_size = None
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_HOME and self.ego_id is not None:
                    self.select_vehicle(self.ego_id)
                    self._camera_recenter(follow=True)
                elif event.key == pygame.K_m:
                    self._camera_recenter(follow=True)
                elif event.key == pygame.K_f and self.camera_mode == "satellite":
                    self.camera_follow = not self.camera_follow
                    if not self.camera_follow and self.satellite_current_center is not None:
                        self.satellite_center = list(self.satellite_current_center)
                    self.say(f"[SATELLITE CAMERA] follow={'ON' if self.camera_follow else 'OFF / FREE PAN'}")
                elif event.key == pygame.K_q and self.camera_mode == "satellite":
                    self.satellite_yaw = (self.satellite_yaw - 5.0) % 360.0
                elif event.key == pygame.K_e and self.camera_mode == "satellite":
                    self.satellite_yaw = (self.satellite_yaw + 5.0) % 360.0
                elif event.key == pygame.K_LEFTBRACKET and self.camera_mode == "satellite":
                    self.satellite_altitude = max(25.0, self.satellite_altitude - 10.0)
                elif event.key == pygame.K_RIGHTBRACKET and self.camera_mode == "satellite":
                    self.satellite_altitude = min(350.0, self.satellite_altitude + 10.0)
                elif event.key == pygame.K_EQUALS or event.key == pygame.K_PLUS:
                    self.args.range_seconds = min(5.0, self.args.range_seconds+0.25)
                elif event.key == pygame.K_MINUS:
                    self.args.range_seconds = max(0.5, self.args.range_seconds-0.25)
                elif event.key == pygame.K_r and self.manual_control_enabled:
                    self.manual_reverse = not self.manual_reverse
                    self.say(f"[CONTROL] Reverse {'ON' if self.manual_reverse else 'OFF'}")
                elif event.key == pygame.K_p and self.manual_control_enabled and safe_alive(self.ego):
                    self.manual_autopilot = not self.manual_autopilot
                    try:
                        self.ego.set_autopilot(self.manual_autopilot, int(self.args.tm_port))
                        self.manual_steer = 0.0
                        self.say(f"[CONTROL] {'Autopilot' if self.manual_autopilot else 'Manual'} mode enabled.")
                    except Exception as exc:
                        self.manual_autopilot = False
                        self.say(f"[CONTROL ERROR] Could not toggle autopilot: {exc}")
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button in (2, 3) and cam_rect.collidepoint(event.pos) and self.camera_mode == "satellite":
                    self.camera_pan_dragging = True
                    self.camera_last_mouse = event.pos
                    if self.camera_follow:
                        self.camera_follow = False
                        if self.satellite_current_center is not None:
                            self.satellite_center = list(self.satellite_current_center)
                elif event.button == 1:
                    chosen = None
                    if cam_rect.collidepoint(event.pos):
                        best = 22.0
                        for aid, point in self.camera_pick_points:
                            d = math.dist(event.pos, point)
                            if d < best:
                                best, chosen = d, aid
                    elif mesh_rect.collidepoint(event.pos):
                        best = 20.0
                        for aid, point in self.mesh_pick_points:
                            d = math.dist(event.pos, point)
                            if d < best:
                                best, chosen = d, aid
                    if chosen is not None:
                        self.select_vehicle(chosen)
                elif event.button == 4 and cam_rect.collidepoint(event.pos):
                    self._camera_zoom(1)
                elif event.button == 5 and cam_rect.collidepoint(event.pos):
                    self._camera_zoom(-1)
            elif event.type == pygame.MOUSEBUTTONUP and event.button in (2, 3):
                self.camera_pan_dragging = False
                self.camera_last_mouse = None
            elif event.type == pygame.MOUSEMOTION and self.camera_pan_dragging:
                if self.camera_last_mouse is not None:
                    dx = event.pos[0] - self.camera_last_mouse[0]
                    dy = event.pos[1] - self.camera_last_mouse[1]
                    self._pan_satellite(dx, dy, cam_rect)
                self.camera_last_mouse = event.pos
            elif event.type == pygame.MOUSEWHEEL:
                if cam_rect.collidepoint(pygame.mouse.get_pos()):
                    self._camera_zoom(event.y)

    def select_vehicle(self, actor_id):
        actor_id = int(actor_id)
        self.selected_id = actor_id
        try:
            Path(self.args.selection_file).resolve().write_text(str(actor_id), encoding="utf-8")
        except Exception:
            pass
        self.say(f"[SELECTED] vehicle/object={actor_id}")

    def _ensure_panel_caches(self, w, h):
        if self.cache_size == (w, h):
            return
        self.cache_size = (w, h)
        self.panel_cache_surface = pygame.Surface((w, h), pygame.SRCALPHA, 32)
        self.last_mesh_draw = 0.0
        self.last_graph_draw = 0.0

    def _render_panels_into_cache(self, mesh_rect, info_rect, graph_rect, now):
        """Render all slow-changing panels into one shared cache (one full-screen blit/frame)."""
        self.panel_cache_surface.fill((0, 0, 0, 0))
        previous_screen = self.screen
        self.screen = self.panel_cache_surface
        try:
            self._draw_mesh_panel(mesh_rect, now)
            self._draw_info_panel(info_rect, now)
            self._draw_graphs(graph_rect, now)
        finally:
            self.screen = previous_screen

    def _log_health(self, now):
        """Low-frequency diagnostics; deliberately avoids per-message console spam."""
        if now - self.last_health_log < 5.0:
            return
        self.last_health_log = now
        peer_count = 0
        received = 0
        if self.ego_id is not None:
            with self.state_lock:
                states = dict(self.rx_states.get(self.ego_id, {}))
                received = int(self.rx_counts.get(self.ego_id, 0))
            peer_count = sum(1 for st in states.values() if now - st.get("rx_time", 0.0) <= STATE_STALE_S)
        lidar_targets = 0
        if self.ego_id is not None:
            with self.state_lock:
                lidar_targets = sum(1 for ob in self.local_detections.get(self.ego_id, {}).values()
                                   if now - ob.get("seen_at", 0.0) <= OBJECT_STALE_S)
        fps = self.clock.get_fps()
        camera_age = getattr(self, "camera_frame_age", 0.0) * 1000.0
        self.say(f"[HEALTH] UI_FPS={fps:.1f} camera_age_ms={camera_age:.0f} connected={len(self.v2x_sensors)} ego_rx_peers={peer_count} ego_rx_total={received} ego_lidar_tracks={lidar_targets} rf_path_loss={'CONFIGURED' if self.radio_model_active else 'NOT_CONFIRMED'}")

    # -------------------------------- main loop ------------------------------
    def run(self):
        self.say("[LOS] EGO-LOS/EGO-NLOS labels are geometry/sensor-visibility labels, separate from radio propagation NLOS.")
        self.say(f"[LOS] world.cast_ray={'available' if self.use_cast_ray else 'unavailable'}; NLOS requires fresh/live track + same-position blocked ray; missing detections alone remain CHECK.")
        try:
            while self.running:
                self.clock.tick(max(10,min(60,self.args.fps)))
                now = time.monotonic()
                layout = self._layout()
                self._handle_events(layout)
                self._sync_actor_cache()
                self._manual_control_tick(now)
                self._update_camera_pose(now)
                self._update_spectator(now)
                self._consume_camera_frame(self.args.camera_width, self.args.camera_height)
                self._record_ego_sample(now)
                self._tick_network(now)
                self._record_session_metrics(now)
                self._log_health(now)
                layout = self._layout()
                w,h,cam_rect,mesh_rect,info_rect,graph_rect = layout
                self._ensure_panel_caches(w,h)
                # Heavy map/info/graph drawing is limited to 10 Hz and composited once.
                if now - self.last_mesh_draw >= 0.10:
                    self._render_panels_into_cache(mesh_rect, info_rect, graph_rect, now)
                    self.last_mesh_draw = now
                    self.last_graph_draw = now
                self.screen.fill(BG)
                self._draw_header(w)
                self._draw_camera_panel(cam_rect,now)
                self.screen.blit(self.panel_cache_surface, (0,0))
                self._draw_status_footer(w,h,now)
                pygame.display.flip()
        finally:
            self.shutdown()

    def shutdown(self):
        if getattr(self, "shutdown_done", False):
            return
        self.shutdown_done = True
        # Take a final snapshot before controls/sensors are stopped.
        try:
            self._record_session_metrics(time.monotonic(), force=True)
        except Exception as exc:
            self.say(f"[SESSION REPORT WARNING] Final sample failed: {exc}")
        self.say("[SHUTDOWN] Releasing ego control and cleaning up only sensors owned by this application...")
        if safe_alive(self.ego) and self.manual_control_enabled:
            try:
                self.ego.set_autopilot(False, int(self.args.tm_port))
            except Exception:
                pass
            try:
                # Hold the ego safely stopped instead of leaving a prior throttle command active.
                self.ego.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            except Exception:
                pass
        self._destroy_camera()
        for sensors in (self.lidar_sensors, self.v2x_sensors):
            for actor_id, sensor in list(sensors.items()):
                safe_stop_destroy(sensor)
            sensors.clear()
        self.los_worker.stop()
        try:
            self._finalize_session_report()
        except Exception as exc:
            self.say(f"[SESSION REPORT ERROR] Could not finalize report: {exc}")
        pygame.quit()
        self.say("[SHUTDOWN] Sensors cleaned up. CARLA vehicles/traffic were not destroyed.")

    # --------------------------- session reporting ---------------------------
    def _init_session_recorder(self):
        """Open streamed CSV logs and initialize per-vehicle summary accumulators."""
        self.session_started_monotonic = time.monotonic()
        self.session_started_wall = datetime.now().astimezone()
        self.session_id = self.session_started_wall.strftime("%Y%m%d_%H%M%S")
        self.session_report_enabled = False
        self.session_report_finalized = False
        self.session_stats = {}
        self.last_session_sample = self.session_started_monotonic
        self.session_samples_since_flush = 0
        self.session_last_rx_total = 0
        self.session_last_tx_total = 0
        self.session_visibility_event_count = 0
        self.session_report_error = ""
        try:
            report_base = Path(self.args.report_dir).expanduser()
            if not report_base.is_absolute():
                report_base = Path(__file__).resolve().parent / report_base
            self.session_dir = report_base.resolve() / f"session_{self.session_id}"
            self.session_dir.mkdir(parents=True, exist_ok=True)
            self.samples_path = self.session_dir / "vehicle_samples.csv"
            self.visibility_path = self.session_dir / "visibility_events.csv"
            self.summary_csv_path = self.session_dir / "vehicle_summary.csv"
            self.summary_md_path = self.session_dir / "session_summary.md"
            self.summary_json_path = self.session_dir / "session_summary.json"
            self.metadata_path = self.session_dir / "session_metadata.json"
            self._sample_file = self.samples_path.open("w", newline="", encoding="utf-8")
            self._sample_fields = [
                "wall_time", "elapsed_s", "vehicle_id", "type_id", "v2x_instrumented",
                "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps", "speed_kmh",
                "ax_mps2", "ay_mps2", "az_mps2", "accel_mps2", "yaw_deg", "pitch_deg", "roll_deg",
                "steer", "throttle", "brake", "ego_visibility_status", "visibility_detail",
                "state_source", "ego_distance_m"
            ]
            self._sample_writer = csv.DictWriter(self._sample_file, fieldnames=self._sample_fields)
            self._sample_writer.writeheader()
            self._visibility_file = self.visibility_path.open("w", newline="", encoding="utf-8")
            self._visibility_fields = [
                "wall_time", "elapsed_s", "vehicle_id", "from_status", "to_status", "detail", "x_m", "y_m", "z_m"
            ]
            self._visibility_writer = csv.DictWriter(self._visibility_file, fieldnames=self._visibility_fields)
            self._visibility_writer.writeheader()
            self.session_report_enabled = True
            self.say(f"[SESSION REPORT] Recording enabled: {self.session_dir}")
            self.say(f"[SESSION REPORT] Vehicle samples at {float(self.args.report_sample_hz):g} Hz; final summary is written when the ADAS dashboard exits.")
        except Exception as exc:
            self.session_report_error = str(exc)
            self.session_report_enabled = False
            self.say(f"[SESSION REPORT WARNING] Could not create report files: {exc}")

    def _new_session_vehicle_stats(self, actor_id, actor, elapsed):
        return {
            "vehicle_id": int(actor_id),
            "type_id": str(getattr(actor, "type_id", "unknown")),
            "first_seen_s": float(elapsed), "last_seen_s": float(elapsed),
            "first_position": None, "last_position": None, "prev_position": None,
            "prev_sample_monotonic": None, "samples": 0, "speed_weighted_sum": 0.0, "speed_sum_samples": 0.0,
            "speed_weight_s": 0.0, "max_speed_kmh": 0.0, "accel_sum": 0.0,
            "max_accel_mps2": 0.0, "distance_m": 0.0, "moving_time_s": 0.0,
            "stationary_time_s": 0.0, "braking_time_s": 0.0, "brake_events": 0,
            "brake_active_prev": False, "los_s": 0.0, "nlos_s": 0.0,
            "check_s": 0.0, "untracked_s": 0.0, "last_visibility_status": None,
            "instrumented_v2x_ever": False, "x_sum": 0.0, "y_sum": 0.0, "z_sum": 0.0,
            "tx_state_messages": 0, "tx_object_messages": 0, "tx_hazard_messages": 0,
        }

    @staticmethod
    def _xyz_distance(a, b):
        if not a or not b:
            return 0.0
        return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))

    def _record_visibility_transition(self, stats, actor_id, status, detail, position, elapsed, wall_time):
        previous = stats.get("last_visibility_status")
        if previous == status:
            return
        stats["last_visibility_status"] = status
        self.session_visibility_event_count += 1
        if self.session_report_enabled:
            self._visibility_writer.writerow({
                "wall_time": wall_time, "elapsed_s": round(elapsed, 3),
                "vehicle_id": int(actor_id), "from_status": previous or "START",
                "to_status": status, "detail": str(detail)[:180],
                "x_m": round(float(position[0]), 3), "y_m": round(float(position[1]), 3),
                "z_m": round(float(position[2]), 3),
            })

    def _record_session_metrics(self, now, force=False):
        """Sample per-vehicle simulator ground truth and ego-relative visibility once per interval.

        Kinematics are CARLA ground truth for experiment evaluation. LOS/NLOS duration is
        the ADAS visibility classifier's observed state, not a radio-packet NLOS label.
        """
        if not getattr(self, "session_report_enabled", False) or self.session_report_finalized:
            return
        interval = 1.0 / max(0.5, min(10.0, float(self.args.report_sample_hz)))
        if not force and now - self.last_session_sample < interval:
            return
        previous_sample = self.last_session_sample
        elapsed = max(0.0, now - self.session_started_monotonic)
        raw_dt = max(0.0, now - previous_sample)
        # Avoid attributing a long UI/network stall as continuous visibility evidence.
        vis_dt = min(raw_dt, interval * 1.75)
        self.last_session_sample = now
        wall_time = datetime.now().astimezone().isoformat(timespec="milliseconds")

        try:
            ego_state = self._ego_state()
            ego_pos = ego_state["position"] if ego_state else None
            if ego_state and self.ego_id is not None:
                e = self.ego
                ev = ego_state["velocity"]
                self.ego_speed_kmh = speed3(ev) * 3.6
                active_radius = min(self.args.max_range, max(self.args.min_range,
                    self.args.min_range + speed3(ev) * self.args.range_seconds))
                self.active_view_range = active_radius
            else:
                active_radius = self.active_view_range
            # Build one consistent fresh-track snapshot for all visibility dwell accounting.
            nodes = self._build_mesh_nodes(now, active_radius) if ego_pos is not None else []
            self.mesh_nodes = nodes
            nodes_by_id = {int(node["id"]): node for node in nodes}
            actors = [(int(aid), actor) for aid, actor in self.vehicle_refs.items() if safe_alive(actor)]
            for actor_id, actor in actors:
                try:
                    tf = actor.get_transform()
                    loc = tf.location
                    vel = actor.get_velocity()
                    acc = actor.get_acceleration()
                    ctl = actor.get_control()
                    pos = [float(loc.x), float(loc.y), float(loc.z)]
                    velocity = [float(vel.x), float(vel.y), float(vel.z)]
                    acceleration = [float(acc.x), float(acc.y), float(acc.z)]
                    speed_ms = speed3(velocity)
                    speed_kmh = speed_ms * 3.6
                    accel_mag = speed3(acceleration)
                    elapsed_dt = 0.0
                    if actor_id not in self.session_stats:
                        self.session_stats[actor_id] = self._new_session_vehicle_stats(actor_id, actor, elapsed)
                    st = self.session_stats[actor_id]
                    if st["prev_sample_monotonic"] is not None:
                        elapsed_dt = min(max(0.0, now - st["prev_sample_monotonic"]), interval * 1.75)
                    if st["first_position"] is None:
                        st["first_position"] = pos[:]
                    if st["prev_position"] is not None:
                        jump = self._xyz_distance(pos, st["prev_position"])
                        # Exclude obvious teleport/spawn relocation jumps from distance travelled.
                        max_plausible = max(25.0, speed_ms * max(0.1, raw_dt) * 3.0 + 10.0)
                        if raw_dt <= 3.0 and jump <= max_plausible:
                            st["distance_m"] += jump
                    st["last_position"] = pos[:]
                    st["prev_position"] = pos[:]
                    st["prev_sample_monotonic"] = now
                    st["last_seen_s"] = elapsed
                    st["samples"] += 1
                    st["speed_sum_samples"] += speed_kmh
                    st["speed_weighted_sum"] += speed_kmh * elapsed_dt
                    st["speed_weight_s"] += elapsed_dt
                    st["max_speed_kmh"] = max(st["max_speed_kmh"], speed_kmh)
                    st["accel_sum"] += accel_mag
                    st["max_accel_mps2"] = max(st["max_accel_mps2"], accel_mag)
                    st["x_sum"] += pos[0]; st["y_sum"] += pos[1]; st["z_sum"] += pos[2]
                    moving = speed_kmh >= 1.0
                    if moving:
                        st["moving_time_s"] += elapsed_dt
                    else:
                        st["stationary_time_s"] += elapsed_dt
                    braking = float(ctl.brake) > 0.1
                    if braking:
                        st["braking_time_s"] += elapsed_dt
                    if braking and not st["brake_active_prev"]:
                        st["brake_events"] += 1
                    st["brake_active_prev"] = braking
                    st["instrumented_v2x_ever"] = st["instrumented_v2x_ever"] or actor_id in self.v2x_sensors

                    if actor_id == self.ego_id:
                        visibility_status, visibility_detail = "EGO", "ego reference vehicle"
                        ego_distance = 0.0
                    elif actor_id in nodes_by_id:
                        visibility_status, _, visibility_detail = self._node_los(actor_id, now)
                        ego_distance = self._xyz_distance(pos, ego_pos) if ego_pos else 0.0
                        if visibility_status == "LOS": st["los_s"] += vis_dt
                        elif visibility_status == "NLOS": st["nlos_s"] += vis_dt
                        elif visibility_status == "CHECK": st["check_s"] += vis_dt
                        else: st["untracked_s"] += vis_dt
                        self._record_visibility_transition(st, actor_id, visibility_status, visibility_detail, pos, elapsed, wall_time)
                    else:
                        visibility_status = "UNTRACKED"
                        visibility_detail = "No current in-range ego track; outside awareness radius or no fresh state/object report"
                        ego_distance = self._xyz_distance(pos, ego_pos) if ego_pos else float("nan")
                        st["untracked_s"] += vis_dt
                        self._record_visibility_transition(st, actor_id, visibility_status, visibility_detail, pos, elapsed, wall_time)

                    st["tx_state_messages"] = int(self.tx_counts[(actor_id, "s")])
                    st["tx_object_messages"] = int(self.tx_counts[(actor_id, "o")])
                    st["tx_hazard_messages"] = int(self.tx_counts[(actor_id, "h")])
                    row = {
                        "wall_time": wall_time, "elapsed_s": round(elapsed, 3),
                        "vehicle_id": actor_id, "type_id": st["type_id"],
                        "v2x_instrumented": actor_id in self.v2x_sensors,
                        "x_m": round(pos[0], 3), "y_m": round(pos[1], 3), "z_m": round(pos[2], 3),
                        "vx_mps": round(velocity[0], 3), "vy_mps": round(velocity[1], 3), "vz_mps": round(velocity[2], 3),
                        "speed_kmh": round(speed_kmh, 3),
                        "ax_mps2": round(acceleration[0], 3), "ay_mps2": round(acceleration[1], 3),
                        "az_mps2": round(acceleration[2], 3), "accel_mps2": round(accel_mag, 3),
                        "yaw_deg": round(float(tf.rotation.yaw), 3), "pitch_deg": round(float(tf.rotation.pitch), 3),
                        "roll_deg": round(float(tf.rotation.roll), 3),
                        "steer": round(float(ctl.steer), 3), "throttle": round(float(ctl.throttle), 3),
                        "brake": round(float(ctl.brake), 3), "ego_visibility_status": visibility_status,
                        "visibility_detail": str(visibility_detail), "state_source": "CARLA_SIMULATOR_GROUND_TRUTH_FOR_EVALUATION",
                        "ego_distance_m": round(ego_distance, 3) if math.isfinite(ego_distance) else "",
                    }
                    self._sample_writer.writerow(row)
                    self.session_samples_since_flush += 1
                except Exception as exc:
                    if now - getattr(self, "last_session_sample_error", 0.0) > 5.0:
                        self.say(f"[SESSION REPORT SAMPLE WARNING] vehicle={actor_id}: {exc}")
                        self.last_session_sample_error = now
            if self.session_samples_since_flush >= 100:
                self._sample_file.flush()
                self._visibility_file.flush()
                self.session_samples_since_flush = 0
        except Exception as exc:
            if now - getattr(self, "last_session_recorder_error", 0.0) > 5.0:
                self.say(f"[SESSION REPORT WARNING] {exc}")
                self.last_session_recorder_error = now

    def _session_summary_row(self, actor_id, stats, elapsed_end):
        los = float(stats.get("los_s", 0.0)); nlos = float(stats.get("nlos_s", 0.0))
        check = float(stats.get("check_s", 0.0)); untracked = float(stats.get("untracked_s", 0.0))
        classified = los + nlos
        rx_all = self.rx_by_sender[(self.ego_id, int(actor_id), "all")] if self.ego_id is not None else 0
        rx_state = self.rx_by_sender[(self.ego_id, int(actor_id), "s")] if self.ego_id is not None else 0
        rx_obj = self.rx_by_sender[(self.ego_id, int(actor_id), "o")] if self.ego_id is not None else 0
        rx_hazard = self.rx_by_sender[(self.ego_id, int(actor_id), "h")] if self.ego_id is not None else 0
        power_n = self.rx_power_n_by_sender[int(actor_id)]
        first = stats.get("first_position") or [None, None, None]
        last = stats.get("last_position") or [None, None, None]
        actor = self.actor_refs.get(int(actor_id))
        if actor is None:
            actor = self.vehicle_refs.get(int(actor_id))
        alive = safe_alive(actor)
        return {
            "vehicle_id": int(actor_id), "type_id": stats.get("type_id", "unknown"),
            "is_ego": int(actor_id) == self.ego_id, "v2x_instrumented_ever": bool(stats.get("instrumented_v2x_ever", False)),
            "alive_at_report_end": alive, "first_seen_s": round(stats.get("first_seen_s", 0.0), 3),
            "last_seen_s": round(stats.get("last_seen_s", 0.0), 3), "sample_count": stats.get("samples", 0),
            "average_speed_kmh": round(stats["speed_weighted_sum"] / stats["speed_weight_s"], 3) if stats.get("speed_weight_s", 0.0) > 0 else (round(stats.get("speed_sum_samples", 0.0) / max(1, stats.get("samples", 0)), 3)),
            "max_speed_kmh": round(float(stats.get("max_speed_kmh", 0.0)), 3),
            "average_acceleration_mps2": round(stats.get("accel_sum", 0.0) / max(1, stats.get("samples", 0)), 3),
            "max_acceleration_mps2": round(float(stats.get("max_accel_mps2", 0.0)), 3),
            "distance_travelled_m_est": round(float(stats.get("distance_m", 0.0)), 3),
            "moving_time_s": round(float(stats.get("moving_time_s", 0.0)), 3),
            "stationary_time_s": round(float(stats.get("stationary_time_s", 0.0)), 3),
            "braking_time_s": round(float(stats.get("braking_time_s", 0.0)), 3),
            "brake_events": int(stats.get("brake_events", 0)),
            "start_x_m": first[0], "start_y_m": first[1], "start_z_m": first[2],
            "end_x_m": last[0], "end_y_m": last[1], "end_z_m": last[2],
            "ego_los_time_s": round(los, 3), "ego_nlos_time_s": round(nlos, 3),
            "ego_check_time_s": round(check, 3), "ego_untracked_time_s": round(untracked, 3),
            "ego_visibility_classified_time_s": round(classified, 3),
            "ego_los_pct_of_classified": round(100.0 * los / classified, 2) if classified > 0 else "",
            "ego_nlos_pct_of_classified": round(100.0 * nlos / classified, 2) if classified > 0 else "",
            "tx_state_messages": int(self.tx_counts[(int(actor_id), "s")]),
            "tx_object_messages": int(self.tx_counts[(int(actor_id), "o")]),
            "tx_hazard_messages": int(self.tx_counts[(int(actor_id), "h")]),
            "tx_oversize_messages": int(self.tx_counts[(int(actor_id), "oversize")]),
            "semantic_lidar_detection_events": int(self.sensor_detection_events[int(actor_id)]),
            "unique_detected_vehicle_ids": sum(1 for _obj_id, obj_kind in self.sensor_detected_ids[int(actor_id)] if obj_kind == "v"),
            "unique_detected_pedestrian_ids": sum(1 for _obj_id, obj_kind in self.sensor_detected_ids[int(actor_id)] if obj_kind == "p"),
            "rx_messages_received_by_this_car": int(self.rx_counts[int(actor_id)]),
            "rx_messages_from_this_car_at_ego": int(rx_all),
            "rx_state_messages_from_this_car_at_ego": int(rx_state),
            "rx_object_reports_from_this_car_at_ego": int(rx_obj),
            "rx_hazard_reports_from_this_car_at_ego": int(rx_hazard),
            "average_rx_power_from_this_car_dbm": round(self.rx_power_sum_by_sender[int(actor_id)] / power_n, 3) if power_n else "",
            "end_visibility_status": ("EGO" if int(actor_id) == self.ego_id else (stats.get("last_visibility_status") or "NOT_CLASSIFIED")),
            "kinematic_data_source": "CARLA_SIMULATOR_GROUND_TRUTH_FOR_EVALUATION",
            "visibility_time_definition": "Ego-relative classifier dwell while target is in live adaptive-awareness mesh; UNTRACKED includes outside-range/no-fresh-track time",
        }

    def _finalize_session_report(self):
        """Close streamed logs, write per-vehicle CSV/JSON/Markdown summaries, and print paths."""
        if not getattr(self, "session_report_enabled", False) or self.session_report_finalized:
            return
        self.session_report_finalized = True
        ended = datetime.now().astimezone()
        elapsed_end = max(0.0, time.monotonic() - self.session_started_monotonic)
        try:
            self._sample_file.flush(); self._sample_file.close()
        except Exception:
            pass
        try:
            self._visibility_file.flush(); self._visibility_file.close()
        except Exception:
            pass
        summary_rows = [self._session_summary_row(aid, stats, elapsed_end)
                        for aid, stats in sorted(self.session_stats.items())]
        summary_fields = list(summary_rows[0].keys()) if summary_rows else [
            "vehicle_id", "type_id", "is_ego", "average_speed_kmh", "max_speed_kmh", "distance_travelled_m_est",
            "ego_los_time_s", "ego_nlos_time_s", "ego_check_time_s", "ego_untracked_time_s"
        ]
        with self.summary_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=summary_fields, extrasaction="ignore")
            writer.writeheader(); writer.writerows(summary_rows)
        tx_total = sum(v for (aid, kind), v in self.tx_counts.items() if isinstance(aid, int) and kind in ("s", "o", "h"))
        ego_rx_total = int(self.rx_counts[self.ego_id]) if self.ego_id is not None else 0
        total_distance_km = sum(float(r.get("distance_travelled_m_est", 0.0) or 0.0) for r in summary_rows) / 1000.0
        avg_speed_all = sum(float(r.get("average_speed_kmh", 0.0) or 0.0) for r in summary_rows) / max(1, len(summary_rows))
        meta = {
            "session_id": self.session_id,
            "started_at_local": self.session_started_wall.isoformat(timespec="seconds"),
            "ended_at_local": ended.isoformat(timespec="seconds"),
            "session_duration_s": round(elapsed_end, 3),
            "carla_host": self.args.host, "carla_port": int(self.args.port),
            "map_name": str(self.map.name), "ego_vehicle_id": self.ego_id,
            "configured_max_instrumented_vehicles": int(self.max_vehicles),
            "actual_instrumented_vehicle_ids": sorted(int(aid) for aid, stats in self.session_stats.items() if stats.get("instrumented_v2x_ever", False)),
            "vehicle_actor_ids_sampled": sorted(int(x) for x in self.session_stats),
            "vehicle_count_sampled": len(summary_rows),
            "rf_path_loss_configured": bool(self.radio_model_active),
            "los_nlos_definition": "EGO-LOS/EGO-NLOS are sensor/geometry visibility labels, not per-packet RF-NLOS classifications.",
            "kinematics_definition": "Per-vehicle kinematic samples are read directly from CARLA ground truth for simulation evaluation, not all transmitted to ego over V2V.",
            "visibility_time_definition": "LOS/NLOS/CHECK/UNTRACKED durations are sampled estimates. LOS/NLOS durations are accumulated only while a fresh target exists in ego's adaptive mesh; percentages use LOS+NLOS classified time as denominator.",
            "tx_message_count_successful_send_calls": int(tx_total), "ego_rx_message_count_parsed": ego_rx_total,
            "estimated_distance_travelled_all_vehicles_km": round(total_distance_km, 3),
            "mean_of_per_vehicle_average_speeds_kmh": round(avg_speed_all, 3),
            "visibility_transition_events": int(self.session_visibility_event_count),
            "sample_rate_hz": float(self.args.report_sample_hz),
            "report_files": [self.samples_path.name, self.summary_csv_path.name, self.visibility_path.name, self.summary_json_path.name, self.summary_md_path.name],
        }
        self.metadata_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
        self.summary_json_path.write_text(json.dumps({"metadata": meta, "vehicles": summary_rows}, indent=2), encoding="utf-8")
        md = [
            "# CARLA V2V Cooperative ADAS — Session Summary", "",
            f"- **Session:** `{self.session_id}`",
            f"- **Start:** {meta['started_at_local']}",
            f"- **End:** {meta['ended_at_local']}",
            f"- **Duration:** {elapsed_end:.1f} s ({elapsed_end/60.0:.1f} min)",
            f"- **Map:** `{self.map.name}`",
            f"- **Ego vehicle ID:** {self.ego_id}",
            f"- **Vehicle IDs sampled:** {len(summary_rows)}",
            f"- **Successfully submitted TX messages (send calls):** {tx_total}",
            f"- **Messages parsed at ego receiver:** {ego_rx_total}",
            "", "## Per-vehicle summary", "",
            "| Vehicle | Type | Avg / max km/h | Distance m* | Start XYZ (m) | End XYZ (m) | LOS / NLOS / CHECK / UNTRACKED (s) | TX s/o/h | RX at ego |",
            "|---:|---|---:|---:|---|---|---|---|---:|"
        ]
        for r in summary_rows:
            start_xyz = ", ".join("—" if r.get(k) in (None, "") else f"{float(r[k]):.1f}" for k in ("start_x_m", "start_y_m", "start_z_m"))
            end_xyz = ", ".join("—" if r.get(k) in (None, "") else f"{float(r[k]):.1f}" for k in ("end_x_m", "end_y_m", "end_z_m"))
            avgmax = f"{float(r['average_speed_kmh']):.1f} / {float(r['max_speed_kmh']):.1f}"
            vis = f"{float(r['ego_los_time_s']):.1f} / {float(r['ego_nlos_time_s']):.1f} / {float(r['ego_check_time_s']):.1f} / {float(r['ego_untracked_time_s']):.1f}"
            tx = f"{r['tx_state_messages']}/{r['tx_object_messages']}/{r['tx_hazard_messages']}"
            md.append(f"| {r['vehicle_id']} | {r['type_id']} | {avgmax} | {r['distance_travelled_m_est']:.1f} | {start_xyz} | {end_xyz} | {vis} | {tx} | {r['rx_messages_from_this_car_at_ego']} |")
        md += [
            "", "## How to interpret visibility durations", "",
            "- `LOS` and `NLOS` are **ego-relative sensor/geometry classifications** supported by fresh tracks and recent evidence. They are not radio packet labels.",
            "- `CHECK` means the target track exists but there was not enough current evidence to confirm LOS/NLOS.",
            "- `UNTRACKED` includes time when the vehicle was outside the ego adaptive awareness mesh, or when ego did not have a fresh valid track. This is not automatically NLOS.",
            "- LOS/NLOS percentages in `vehicle_summary.csv` use only `LOS + NLOS` as their denominator, excluding CHECK and UNTRACKED.",
            "- Position, speed, acceleration and distance are sampled from CARLA simulator ground truth for evaluation. They should not be presented as data ego received over V2V unless the separate RX fields show those messages.",
            "- Distance is an estimate from sampled consecutive positions; long gaps and obvious teleports are excluded.",
            "", "## Files", "",
            "- `vehicle_summary.csv`: one row per sampled vehicle, with speed/distance/LOS/NLOS/communication and sensor-detection statistics.",
            "- `vehicle_samples.csv`: timestamped position, velocity, acceleration, controls and visibility status samples for every observed vehicle.",
            "- `visibility_events.csv`: timestamped changes between EGO-LOS, EGO-NLOS, CHECK and UNTRACKED.",
            "- `session_summary.json`: machine-readable metadata and per-vehicle summary.",
            "- `session_metadata.json`: session configuration and definitions.",
            "", "Generated automatically when the ADAS dashboard exits.",
        ]
        self.summary_md_path.write_text("\n".join(md) + "\n", encoding="utf-8")
        self.say("[SESSION REPORT] FINAL SUMMARY WRITTEN")
        self.say(f"[SESSION REPORT] Markdown: {self.summary_md_path}")
        self.say(f"[SESSION REPORT] Per-vehicle CSV: {self.summary_csv_path}")
        self.say(f"[SESSION REPORT] Position/time-series CSV: {self.samples_path}")
        self.say(f"[SESSION REPORT] LOS/NLOS event log: {self.visibility_path}")
        self.say(f"[SESSION REPORT] JSON: {self.summary_json_path}")
        self.say(f"[SESSION REPORT] Folder: {self.session_dir}")


def main():
    args = parse_args()
    args.max_vehicles = max(1, min(50, args.max_vehicles))
    args.report_sample_hz = max(1.0, min(10.0, float(args.report_sample_hz)))
    # Explicit fleet-aware defaults avoid rendering a large RGB sensor and high-rate
    # LiDAR on every one of 20-50 cars unless the user deliberately requests it.
    count = args.max_vehicles
    if args.camera_width is None:
        args.camera_width = 800 if count <= 5 else 640 if count <= 20 else 480 if count <= 35 else 400
    if args.camera_height is None:
        args.camera_height = 450 if count <= 5 else 360 if count <= 20 else 270 if count <= 35 else 225
    if args.camera_hz is None:
        args.camera_hz = 25.0 if count <= 5 else 20.0 if count <= 20 else 15.0 if count <= 35 else 12.0
    if args.spectator_hz is None:
        args.spectator_hz = 25.0 if count <= 5 else 20.0 if count <= 20 else 15.0 if count <= 35 else 12.0
    if args.lidar_hz is None:
        args.lidar_hz = 5.0 if count <= 5 else 3.0 if count <= 20 else 2.0 if count <= 35 else 1.5
    args.width = max(1000, min(1920, int(args.width)))
    args.height = max(700, min(1200, int(args.height)))
    args.camera_width = max(320, min(1280, int(args.camera_width)))
    args.camera_height = max(180, min(720, int(args.camera_height)))
    args.min_range = max(20.0, float(args.min_range))
    args.max_range = max(args.min_range, min(500.0, float(args.max_range)))
    args.camera_fov = max(45.0, min(110.0, float(args.camera_fov)))
    args.camera_back = max(3.0, min(16.0, float(args.camera_back)))
    args.camera_height_m = max(1.5, min(80.0, float(args.camera_height_m)))
    args.camera_pitch = max(-89.0, min(5.0, float(args.camera_pitch)))
    args.satellite_altitude = max(25.0, min(350.0, float(args.satellite_altitude)))
    args.satellite_yaw = float(args.satellite_yaw) % 360.0
    args.camera_hz = max(5.0, min(30.0, float(args.camera_hz)))
    args.spectator_hz = max(4.0, min(30.0, float(args.spectator_hz)))
    args.lidar_channels = max(4, min(32, int(args.lidar_channels)))
    args.lidar_hz = max(1.0, min(20.0, float(args.lidar_hz)))
    args.lidar_pps = max(500, min(100000, int(args.lidar_pps)))
    try:
        app = CooperativeADAS(args)
        app.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user.", flush=True)
    except Exception as exc:
        print(f"[FATAL] {exc}", flush=True)
        raise


if __name__ == "__main__":
    main()
