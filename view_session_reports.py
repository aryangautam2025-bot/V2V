#!/usr/bin/env python3
"""Graphical browser for CARLA V2V ADAS session reports.

Reads the report files written by v2v_cooperative_adas_satellite.py. This file is
standalone: it does not import CARLA, change simulation code, or write to reports.

Run from the project venv:
  .\\.venv\\Scripts\\python.exe .\\view_session_reports.py
  .\\.venv\\Scripts\\python.exe .\\view_session_reports.py --session-folder .\\session_reports\\session_YYYYMMDD_HHMMSS
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QFrame, QGridLayout, QHBoxLayout,
    QLabel, QMainWindow, QMessageBox, QPushButton, QScrollArea, QSizePolicy,
    QTabWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

APP_NAME = "V2V ADAS · SESSION ANALYTICS"
BG = "#0A1019"
PANEL = "#101B29"
PANEL_ALT = "#142235"
BORDER = "#25364B"
TEXT = "#EAF1F8"
MUTED = "#91A3B7"
CYAN = "#42D6E8"
BLUE = "#63A8FF"
GREEN = "#43D6A0"
AMBER = "#F4BB54"
RED = "#FF6978"
PURPLE = "#BE91FF"
WHITE = "#F4F7FB"


def safe_float(value: Any, default: float | None = None) -> float | None:
    if value is None or value == "":
        return default
    try:
        number = float(value)
        return number if math.isfinite(number) else default
    except (TypeError, ValueError):
        return default


def fmt(value: Any, digits: int = 2, suffix: str = "", blank: str = "—") -> str:
    number = safe_float(value)
    if number is None:
        if isinstance(value, bool):
            return "Yes" if value else "No"
        if value in (None, ""):
            return blank
        return str(value)
    return f"{number:,.{digits}f}{suffix}"


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    try:
        with path.open("r", newline="", encoding="utf-8-sig") as handle:
            return [dict(row) for row in csv.DictReader(handle)]
    except (OSError, csv.Error):
        return []


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
        return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def resolve_session_folder(path: Path) -> Path:
    path = path.expanduser().resolve()
    if (path / "vehicle_summary.csv").exists() or (path / "session_summary.json").exists():
        return path
    # If a reports root was provided, pick its most recent session folder.
    candidates = [p for p in path.glob("session_*") if p.is_dir() and (
        (p / "vehicle_summary.csv").exists() or (p / "session_summary.json").exists())]
    if candidates:
        return max(candidates, key=lambda p: p.stat().st_mtime)
    return path


class ReportData:
    def __init__(self, folder: Path):
        self.folder = resolve_session_folder(folder)
        self.summary_rows = read_csv_rows(self.folder / "vehicle_summary.csv")
        self.samples = read_csv_rows(self.folder / "vehicle_samples.csv")
        self.visibility_events = read_csv_rows(self.folder / "visibility_events.csv")
        self.summary_json = read_json(self.folder / "session_summary.json")
        self.metadata = read_json(self.folder / "session_metadata.json")
        if not self.metadata and isinstance(self.summary_json.get("metadata"), dict):
            self.metadata = self.summary_json["metadata"]
        if not self.summary_rows and isinstance(self.summary_json.get("vehicles"), list):
            self.summary_rows = [dict(x) for x in self.summary_json["vehicles"] if isinstance(x, dict)]
        self.by_id: dict[str, dict[str, Any]] = {}
        for row in self.summary_rows:
            if row.get("vehicle_id") not in (None, ""):
                self.by_id[str(row["vehicle_id"])] = row
        self.samples_by_id: dict[str, list[dict[str, str]]] = defaultdict(list)
        for row in self.samples:
            vid = str(row.get("vehicle_id", ""))
            if vid:
                self.samples_by_id[vid].append(row)
        self.events_by_id: dict[str, list[dict[str, str]]] = defaultdict(list)
        for row in self.visibility_events:
            vid = str(row.get("vehicle_id", ""))
            if vid:
                self.events_by_id[vid].append(row)
        # Sort time series numerically; exports normally already are in order.
        for group in self.samples_by_id.values():
            group.sort(key=lambda x: safe_float(x.get("elapsed_s"), 0.0) or 0.0)
        for group in self.events_by_id.values():
            group.sort(key=lambda x: safe_float(x.get("elapsed_s"), 0.0) or 0.0)
        self.vehicle_ids = sorted(set(self.by_id) | set(self.samples_by_id), key=self._id_sort_key)
        self.error = ""
        if not self.summary_rows and not self.samples:
            self.error = "No vehicle_summary.csv, vehicle_samples.csv, or usable session_summary.json found in this folder."

    @staticmethod
    def _id_sort_key(value: str):
        try:
            return (0, int(value))
        except ValueError:
            return (1, value)

    def row(self, vehicle_id: str) -> dict[str, Any]:
        return self.by_id.get(str(vehicle_id), {})

    def samples_for(self, vehicle_id: str) -> list[dict[str, str]]:
        return self.samples_by_id.get(str(vehicle_id), [])

    def overall_stats(self) -> dict[str, Any]:
        meta = self.metadata or {}
        rows = self.summary_rows
        instrumented = sum(str(r.get("v2x_instrumented_ever", "")).lower() in ("1", "true", "yes") for r in rows)
        total_dist = sum(safe_float(r.get("distance_travelled_m_est"), 0.0) or 0.0 for r in rows)
        avgs = [safe_float(r.get("average_speed_kmh")) for r in rows]
        avgs = [v for v in avgs if v is not None]
        return {
            "session_id": meta.get("session_id", self.folder.name),
            "map_name": meta.get("map_name", "—"),
            "ego_id": meta.get("ego_vehicle_id", "—"),
            "duration": safe_float(meta.get("session_duration_s")),
            "sampled": safe_float(meta.get("vehicle_count_sampled"), len(self.vehicle_ids)),
            "instrumented": instrumented,
            "tx": safe_float(meta.get("tx_message_count_successful_send_calls")),
            "rx_ego": safe_float(meta.get("ego_rx_message_count_parsed")),
            "distance_km": total_dist / 1000.0,
            "mean_avg_speed": sum(avgs) / len(avgs) if avgs else None,
        }


def make_label(text: str, css: str = "body") -> QLabel:
    label = QLabel(text)
    label.setWordWrap(True)
    if css == "muted":
        label.setStyleSheet(f"color:{MUTED}; font-size:11px;")
    elif css == "title":
        label.setStyleSheet(f"color:{WHITE}; font-size:20px; font-weight:700;")
    elif css == "subtitle":
        label.setStyleSheet(f"color:{CYAN}; font-size:11px; font-weight:700; letter-spacing:1px;")
    else:
        label.setStyleSheet(f"color:{TEXT}; font-size:12px;")
    return label


class MetricCard(QFrame):
    def __init__(self, title: str, value: str, note: str = "", accent: str = CYAN):
        super().__init__()
        self.setObjectName("metricCard")
        self.setMinimumHeight(86)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(13, 10, 13, 10)
        layout.setSpacing(3)
        t = QLabel(title.upper())
        t.setStyleSheet(f"color:{MUTED}; font-size:10px; font-weight:700; letter-spacing:0.8px;")
        self.value_label = QLabel(value)
        self.value_label.setStyleSheet(f"color:{accent}; font-size:23px; font-weight:700;")
        self.value_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.note_label = QLabel(note)
        self.note_label.setStyleSheet(f"color:{MUTED}; font-size:10px;")
        self.note_label.setWordWrap(True)
        layout.addWidget(t)
        layout.addWidget(self.value_label)
        layout.addWidget(self.note_label)


class PlotCard(QFrame):
    def __init__(self, title: str, height: float = 2.45):
        super().__init__()
        self.setObjectName("plotCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(9, 8, 9, 7)
        layout.setSpacing(4)
        title_label = QLabel(title.upper())
        title_label.setStyleSheet(f"color:{MUTED}; font-size:10px; font-weight:700; letter-spacing:0.7px;")
        layout.addWidget(title_label)
        self.figure = Figure(figsize=(5.4, height), dpi=90, facecolor=PANEL)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setMinimumHeight(180)
        layout.addWidget(self.canvas)
        self.ax = self.figure.add_subplot(111)
        self.style_axes()

    def style_axes(self):
        self.ax.set_facecolor(PANEL)
        self.ax.tick_params(colors=MUTED, labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_color(BORDER)
        self.ax.grid(True, color=BORDER, linewidth=0.55, alpha=0.75)
        self.ax.title.set_color(TEXT)
        self.ax.xaxis.label.set_color(MUTED)
        self.ax.yaxis.label.set_color(MUTED)

    def finish(self):
        self.figure.tight_layout(pad=1.25)
        self.canvas.draw_idle()


class SummaryViewer(QMainWindow):
    def __init__(self, initial_folder: Path | None = None):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1500, 960)
        self.setMinimumSize(1050, 700)
        self.data: ReportData | None = None
        self.active_folder: Path | None = None

        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(14, 12, 14, 12)
        root_layout.setSpacing(10)
        self.setCentralWidget(root)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title_box.addWidget(make_label(APP_NAME, "subtitle"))
        title_box.addWidget(make_label("Per-vehicle experiment review", "title"))
        title_box.addWidget(make_label("Offline analytics · reads existing session reports only · does not connect to CARLA", "muted"))
        header.addLayout(title_box, 1)
        self.folder_label = QLabel("No report folder loaded")
        self.folder_label.setStyleSheet(f"color:{MUTED}; font-size:11px;")
        self.folder_label.setWordWrap(True)
        self.folder_label.setMaximumWidth(440)
        header.addWidget(self.folder_label)
        self.open_button = QPushButton("Open session folder")
        self.open_button.clicked.connect(self.choose_folder)
        header.addWidget(self.open_button)
        self.reload_button = QPushButton("Reload")
        self.reload_button.clicked.connect(self.reload_current)
        header.addWidget(self.reload_button)
        root_layout.addLayout(header)
        self.status = QLabel("Choose a session_reports folder or one specific session folder.")
        self.status.setStyleSheet(f"color:{AMBER}; padding:7px 10px; background:{PANEL_ALT}; border:1px solid {BORDER}; border-radius:6px;")
        root_layout.addWidget(self.status)
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        root_layout.addWidget(self.tabs, 1)

        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background:{BG}; color:{TEXT}; }}
            QFrame#metricCard, QFrame#plotCard {{ background:{PANEL}; border:1px solid {BORDER}; border-radius:8px; }}
            QTabWidget::pane {{ border:1px solid {BORDER}; border-radius:7px; background:{BG}; top:-1px; }}
            QTabBar::tab {{ background:{PANEL_ALT}; color:{MUTED}; padding:10px 16px; margin-right:3px; border-top-left-radius:5px; border-top-right-radius:5px; }}
            QTabBar::tab:selected {{ background:{PANEL}; color:{CYAN}; border-bottom:2px solid {CYAN}; }}
            QPushButton {{ background:{PANEL_ALT}; color:{TEXT}; border:1px solid {BORDER}; padding:8px 12px; border-radius:6px; }}
            QPushButton:hover {{ border-color:{CYAN}; color:{CYAN}; }}
            QTableWidget {{ background:{PANEL}; alternate-background-color:{PANEL_ALT}; color:{TEXT}; gridline-color:{BORDER}; border:1px solid {BORDER}; border-radius:6px; selection-background-color:#214761; }}
            QHeaderView::section {{ background:{PANEL_ALT}; color:{MUTED}; padding:7px; border:0; border-bottom:1px solid {BORDER}; font-weight:700; }}
            QScrollArea {{ border:0; background:transparent; }}
            QComboBox {{ background:{PANEL_ALT}; color:{TEXT}; padding:6px; border:1px solid {BORDER}; border-radius:5px; }}
        """)

        if initial_folder:
            self.load_folder(initial_folder)
        else:
            candidates = [Path(__file__).resolve().parent / "session_reports", Path.cwd() / "session_reports"]
            default = next((p for p in candidates if p.exists()), None)
            if default:
                resolved = resolve_session_folder(default)
                if (resolved / "vehicle_summary.csv").exists() or (resolved / "session_summary.json").exists():
                    self.load_folder(resolved)
                else:
                    self.choose_folder()
            else:
                self.choose_folder()

    def choose_folder(self):
        start = str(self.active_folder.parent if self.active_folder else Path(__file__).resolve().parent)
        folder = QFileDialog.getExistingDirectory(self, "Select a session folder or session_reports folder", start)
        if folder:
            self.load_folder(Path(folder))

    def reload_current(self):
        if self.active_folder:
            self.load_folder(self.active_folder)
        else:
            self.choose_folder()

    def load_folder(self, folder: Path):
        data = ReportData(folder)
        if data.error:
            QMessageBox.warning(self, "No session report found", f"{data.error}\n\nSelected folder:\n{data.folder}")
            return
        self.data = data
        self.active_folder = data.folder
        self.folder_label.setText(str(data.folder))
        self.status.setText(f"Loaded {len(data.vehicle_ids)} vehicle tabs · {len(data.samples)} time-series samples · {len(data.visibility_events)} visibility transitions")
        self.status.setStyleSheet(f"color:{GREEN}; padding:7px 10px; background:{PANEL_ALT}; border:1px solid {BORDER}; border-radius:6px;")
        self.tabs.clear()
        self.tabs.addTab(self.build_overview_tab(data), "FLEET OVERVIEW")
        for vid in data.vehicle_ids:
            row = data.row(vid)
            title = f"EGO · {vid}" if str(row.get("is_ego", "")).lower() in ("1", "true", "yes") else f"CAR · {vid}"
            self.tabs.addTab(self.build_vehicle_tab(data, vid), title)

    def build_scroll_page(self):
        outer = QScrollArea()
        outer.setWidgetResizable(True)
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(12, 12, 12, 16)
        layout.setSpacing(12)
        outer.setWidget(page)
        return outer, page, layout

    def build_overview_tab(self, data: ReportData) -> QWidget:
        scroll, page, layout = self.build_scroll_page()
        meta = data.overall_stats()
        layout.addWidget(make_label(f"SESSION  /  {meta['session_id']}", "subtitle"))
        layout.addWidget(make_label(f"Fleet overview · {meta['map_name']}", "title"))
        cards = QGridLayout(); cards.setSpacing(9)
        duration = "—" if meta["duration"] is None else f"{meta['duration']/60:.1f} min"
        card_data = [
            ("Vehicles sampled", str(len(data.vehicle_ids)), f"{int(meta['instrumented'])} instrumented for V2X", CYAN),
            ("Session duration", duration, "Recorder runtime", BLUE),
            ("Mean per-car avg speed", fmt(meta["mean_avg_speed"], 1, " km/h"), "Unweighted mean of each car's average", GREEN),
            ("Estimated fleet distance", fmt(meta["distance_km"], 2, " km"), "Sum of per-car estimated travel", AMBER),
            ("TX send calls", fmt(meta["tx"], 0), "Successful send() calls; not delivery proof", PURPLE),
            ("Messages parsed at ego", fmt(meta["rx_ego"], 0), f"Ego vehicle ID: {meta['ego_id']}", RED),
        ]
        for idx, (label, value, note, color) in enumerate(card_data):
            cards.addWidget(MetricCard(label, value, note, color), idx // 3, idx % 3)
        layout.addLayout(cards)
        charts = QGridLayout(); charts.setSpacing(9)
        avg_plot = PlotCard("Average and maximum speed by vehicle", 2.35)
        ids = data.vehicle_ids
        labels = [f"V{x}" for x in ids]
        avg = [safe_float(data.row(v).get("average_speed_kmh"), 0.0) or 0.0 for v in ids]
        mx = [safe_float(data.row(v).get("max_speed_kmh"), 0.0) or 0.0 for v in ids]
        xvals = list(range(len(ids)))
        avg_plot.ax.bar([x - 0.18 for x in xvals], avg, width=0.36, color=CYAN, label="Average")
        avg_plot.ax.bar([x + 0.18 for x in xvals], mx, width=0.36, color=BLUE, label="Maximum")
        avg_plot.ax.set_xticks(xvals); avg_plot.ax.set_xticklabels(labels, rotation=45, ha="right")
        avg_plot.ax.set_ylabel("km/h"); avg_plot.ax.legend(frameon=False, fontsize=8, labelcolor=TEXT); avg_plot.finish()
        vis_plot = PlotCard("Ego-relative classified visibility time", 2.35)
        los = [safe_float(data.row(v).get("ego_los_time_s"), 0.0) or 0.0 for v in ids]
        nlos = [safe_float(data.row(v).get("ego_nlos_time_s"), 0.0) or 0.0 for v in ids]
        xvals = list(range(len(ids)))
        vis_plot.ax.bar(xvals, los, color=GREEN, label="LOS")
        vis_plot.ax.bar(xvals, nlos, bottom=los, color=RED, label="NLOS")
        vis_plot.ax.set_xticks(xvals); vis_plot.ax.set_xticklabels(labels, rotation=45, ha="right")
        vis_plot.ax.set_ylabel("seconds"); vis_plot.ax.legend(frameon=False, fontsize=8, labelcolor=TEXT); vis_plot.finish()
        traj = PlotCard("Recorded vehicle paths (world X/Y)", 2.35)
        any_traj = False
        color_cycle = [CYAN, BLUE, GREEN, AMBER, PURPLE, RED, "#F48FB1", "#90CAF9"]
        for i, vid in enumerate(ids):
            samples = data.samples_for(vid)
            xs = [safe_float(s.get("x_m")) for s in samples]
            ys = [safe_float(s.get("y_m")) for s in samples]
            pairs = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
            if len(pairs) >= 2:
                traj.ax.plot([p[0] for p in pairs], [p[1] for p in pairs], linewidth=1.2,
                             color=color_cycle[i % len(color_cycle)], label=f"V{vid}")
                any_traj = True
        traj.ax.set_xlabel("X (m)"); traj.ax.set_ylabel("Y (m)"); traj.ax.set_aspect("equal", adjustable="datalim")
        if any_traj and len(ids) <= 18:
            traj.ax.legend(frameon=False, fontsize=7, labelcolor=TEXT, ncol=2)
        elif not any_traj:
            traj.ax.text(0.5, 0.5, "No trajectory samples found", color=MUTED, ha="center", va="center", transform=traj.ax.transAxes)
        traj.finish()
        comm = PlotCard("V2V message activity by vehicle", 2.35)
        kinds = [
            ("tx_state_messages", "TX state", CYAN),
            ("tx_object_messages", "TX objects", BLUE),
            ("tx_hazard_messages", "TX hazards", AMBER),
            ("rx_messages_from_this_car_at_ego", "RX from car at ego", GREEN),
        ]
        width = 0.8 / max(1, len(kinds))
        for j, (key, name, color) in enumerate(kinds):
            vals = [safe_float(data.row(v).get(key), 0.0) or 0.0 for v in ids]
            comm.ax.bar([i - 0.4 + width/2 + j*width for i in range(len(ids))], vals, width=width, color=color, label=name)
        comm.ax.set_xticks(list(range(len(ids)))); comm.ax.set_xticklabels(labels, rotation=45, ha="right")
        comm.ax.set_ylabel("messages"); comm.ax.legend(frameon=False, fontsize=7, labelcolor=TEXT, ncol=2); comm.finish()
        charts.addWidget(avg_plot, 0, 0); charts.addWidget(vis_plot, 0, 1)
        charts.addWidget(traj, 1, 0); charts.addWidget(comm, 1, 1)
        layout.addLayout(charts)
        layout.addWidget(make_label("ALL VEHICLES · SUMMARY TABLE", "subtitle"))
        table_columns = [
            ("vehicle_id", "ID"), ("type_id", "Vehicle type"), ("is_ego", "Ego"),
            ("v2x_instrumented_ever", "V2X"), ("average_speed_kmh", "Avg km/h"),
            ("max_speed_kmh", "Max km/h"), ("distance_travelled_m_est", "Distance m"),
            ("ego_los_time_s", "LOS s"), ("ego_nlos_time_s", "NLOS s"),
            ("ego_check_time_s", "CHECK s"), ("ego_untracked_time_s", "UNTRACKED s"),
            ("rx_messages_from_this_car_at_ego", "RX at ego"),
        ]
        layout.addWidget(self.make_table(data.summary_rows, table_columns, minimum_height=280))
        note = QLabel("Interpretation: kinematics are CARLA ground truth for evaluation. LOS/NLOS are ego-relative visibility classifications, not per-packet RF-NLOS labels. UNTRACKED does not mean NLOS.")
        note.setWordWrap(True); note.setStyleSheet(f"color:{MUTED}; background:{PANEL_ALT}; padding:10px; border:1px solid {BORDER}; border-radius:6px;")
        layout.addWidget(note)
        layout.addStretch(1)
        return scroll

    def build_vehicle_tab(self, data: ReportData, vid: str) -> QWidget:
        scroll, page, layout = self.build_scroll_page()
        row = data.row(vid)
        samples = data.samples_for(vid)
        is_ego = str(row.get("is_ego", "")).lower() in ("1", "true", "yes")
        label = "EGO VEHICLE" if is_ego else "REMOTE / TRAFFIC VEHICLE"
        layout.addWidget(make_label(f"VEHICLE PROFILE  /  ID {vid}  /  {label}", "subtitle"))
        layout.addWidget(make_label(str(row.get("type_id", "Unknown vehicle type")), "title"))
        cards = QGridLayout(); cards.setSpacing(9)
        vis_classified = safe_float(row.get("ego_visibility_classified_time_s"), 0.0) or 0.0
        los_pct = safe_float(row.get("ego_los_pct_of_classified"))
        nlos_pct = safe_float(row.get("ego_nlos_pct_of_classified"))
        card_data = [
            ("Average speed", fmt(row.get("average_speed_kmh"), 1, " km/h"), "Time-weighted mean from report", CYAN),
            ("Maximum speed", fmt(row.get("max_speed_kmh"), 1, " km/h"), "Peak sampled speed", BLUE),
            ("Distance travelled", fmt(row.get("distance_travelled_m_est"), 1, " m"), "Estimated from sampled positions", GREEN),
            ("Average acceleration", fmt(row.get("average_acceleration_mps2"), 2, " m/s²"), "Magnitude, not signed longitudinal acceleration", AMBER),
            ("V2V RX at ego", fmt(row.get("rx_messages_from_this_car_at_ego"), 0), "Messages from this sender parsed by ego", PURPLE),
            ("LOS / NLOS share", f"{fmt(los_pct, 1, '%')} / {fmt(nlos_pct, 1, '%')}", f"Classified time: {vis_classified:.1f} s (LOS + NLOS only)" if vis_classified else "No classified visibility time", RED),
        ]
        for idx, (title, value, note, color) in enumerate(card_data):
            cards.addWidget(MetricCard(title, value, note, color), idx // 3, idx % 3)
        layout.addLayout(cards)

        grid = QGridLayout(); grid.setSpacing(9)
        speed_plot = PlotCard("Speed history", 2.2)
        t, speed = self.series(samples, "elapsed_s", "speed_kmh")
        if t:
            speed_plot.ax.plot(t, speed, color=CYAN, linewidth=1.65)
            speed_plot.ax.fill_between(t, speed, color=CYAN, alpha=0.10)
        else:
            self.empty_plot(speed_plot.ax, "No speed samples")
        speed_plot.ax.set_xlabel("Elapsed session time (s)"); speed_plot.ax.set_ylabel("km/h"); speed_plot.finish()

        accel_plot = PlotCard("Acceleration magnitude", 2.2)
        t, accel = self.series(samples, "elapsed_s", "accel_mps2")
        if t:
            accel_plot.ax.plot(t, accel, color=AMBER, linewidth=1.5)
        else:
            self.empty_plot(accel_plot.ax, "No acceleration samples")
        accel_plot.ax.set_xlabel("Elapsed session time (s)"); accel_plot.ax.set_ylabel("m/s²"); accel_plot.finish()

        path_plot = PlotCard("Vehicle trajectory · world coordinates", 2.2)
        xs, ys = self.series(samples, "x_m", "y_m")
        if len(xs) > 1:
            path_plot.ax.plot(xs, ys, color=BLUE, linewidth=1.5)
            path_plot.ax.scatter(xs[0], ys[0], color=GREEN, s=24, label="Start", zorder=3)
            path_plot.ax.scatter(xs[-1], ys[-1], color=RED, s=24, label="End", zorder=3)
            path_plot.ax.legend(frameon=False, fontsize=8, labelcolor=TEXT)
            path_plot.ax.set_aspect("equal", adjustable="datalim")
        else:
            self.empty_plot(path_plot.ax, "No trajectory samples")
        path_plot.ax.set_xlabel("X (m)"); path_plot.ax.set_ylabel("Y (m)"); path_plot.finish()

        control_plot = PlotCard("Driver / control inputs", 2.2)
        tc, steer = self.series(samples, "elapsed_s", "steer")
        _, throttle = self.series(samples, "elapsed_s", "throttle")
        _, brake = self.series(samples, "elapsed_s", "brake")
        if tc:
            control_plot.ax.plot(tc, steer, color=PURPLE, linewidth=1.2, label="Steer")
            control_plot.ax.plot(tc, throttle, color=GREEN, linewidth=1.2, label="Throttle")
            control_plot.ax.plot(tc, brake, color=RED, linewidth=1.2, label="Brake")
            control_plot.ax.legend(frameon=False, fontsize=8, labelcolor=TEXT, ncol=3)
        else:
            self.empty_plot(control_plot.ax, "No control samples")
        control_plot.ax.set_xlabel("Elapsed session time (s)"); control_plot.ax.set_ylim(-1.05, 1.05); control_plot.finish()

        vis_plot = PlotCard("Ego-relative visibility history", 2.2)
        status_num = {"UNTRACKED": 0, "CHECK": 1, "NLOS": 2, "EGO-NLOS": 2, "LOS": 3, "EGO-LOS": 3, "EGO": 4}
        vt, vv, labels = [], [], []
        for s in samples:
            elapsed = safe_float(s.get("elapsed_s")); status = s.get("ego_visibility_status", "")
            if elapsed is not None and status in status_num:
                vt.append(elapsed); vv.append(status_num[status]); labels.append(status)
        if vt:
            vis_plot.ax.step(vt, vv, where="post", color=GREEN, linewidth=1.55)
            vis_plot.ax.set_yticks([0, 1, 2, 3, 4]); vis_plot.ax.set_yticklabels(["UNTRACKED", "CHECK", "NLOS", "LOS", "EGO"], fontsize=7)
            vis_plot.ax.set_ylim(-0.5, 4.5)
        else:
            self.empty_plot(vis_plot.ax, "No visibility samples")
        vis_plot.ax.set_xlabel("Elapsed session time (s)"); vis_plot.finish()

        comm_plot = PlotCard("Communication and perception counters", 2.2)
        comm_keys = [
            ("tx_state_messages", "TX state", CYAN), ("tx_object_messages", "TX objects", BLUE),
            ("tx_hazard_messages", "TX hazards", AMBER),
            ("rx_messages_from_this_car_at_ego", "RX at ego", GREEN),
        ]
        vals = [safe_float(row.get(k), 0.0) or 0.0 for k, _, _ in comm_keys]
        comm_plot.ax.bar([name for _, name, _ in comm_keys], vals, color=[c for _, _, c in comm_keys])
        comm_plot.ax.tick_params(axis="x", rotation=20, labelsize=7)
        comm_plot.ax.set_ylabel("messages")
        comm_plot.finish()

        grid.addWidget(speed_plot, 0, 0); grid.addWidget(accel_plot, 0, 1)
        grid.addWidget(path_plot, 1, 0); grid.addWidget(control_plot, 1, 1)
        grid.addWidget(vis_plot, 2, 0); grid.addWidget(comm_plot, 2, 1)
        layout.addLayout(grid)

        layout.addWidget(make_label("MOTION & SESSION SUMMARY", "subtitle"))
        motion_columns = [
            ("first_seen_s", "First seen (s)"), ("last_seen_s", "Last seen (s)"), ("sample_count", "Samples"),
            ("average_speed_kmh", "Avg speed (km/h)"), ("max_speed_kmh", "Max speed (km/h)"),
            ("average_acceleration_mps2", "Avg accel (m/s²)"), ("max_acceleration_mps2", "Max accel (m/s²)"),
            ("distance_travelled_m_est", "Distance (m)"), ("moving_time_s", "Moving (s)"),
            ("stationary_time_s", "Stationary (s)"), ("braking_time_s", "Braking (s)"), ("brake_events", "Brake events"),
            ("start_x_m", "Start X"), ("start_y_m", "Start Y"), ("start_z_m", "Start Z"),
            ("end_x_m", "End X"), ("end_y_m", "End Y"), ("end_z_m", "End Z"),
        ]
        layout.addWidget(self.single_record_table(row, motion_columns))
        layout.addWidget(make_label("LOS / NLOS & VISIBILITY", "subtitle"))
        visibility_columns = [
            ("end_visibility_status", "End status"), ("ego_los_time_s", "LOS (s)"), ("ego_nlos_time_s", "NLOS (s)"),
            ("ego_check_time_s", "CHECK (s)"), ("ego_untracked_time_s", "UNTRACKED (s)"),
            ("ego_visibility_classified_time_s", "Classified (s)"),
            ("ego_los_pct_of_classified", "LOS % of classified"), ("ego_nlos_pct_of_classified", "NLOS % of classified"),
        ]
        layout.addWidget(self.single_record_table(row, visibility_columns))
        layout.addWidget(make_label("V2V & SENSOR STATISTICS", "subtitle"))
        comm_columns = [
            ("v2x_instrumented_ever", "V2X instrumented"), ("tx_state_messages", "TX state"),
            ("tx_object_messages", "TX objects"), ("tx_hazard_messages", "TX hazards"),
            ("tx_oversize_messages", "Oversize skipped"), ("rx_messages_received_by_this_car", "RX by this car"),
            ("rx_messages_from_this_car_at_ego", "RX from this car at ego"),
            ("rx_state_messages_from_this_car_at_ego", "RX state at ego"),
            ("rx_object_reports_from_this_car_at_ego", "RX object at ego"),
            ("rx_hazard_reports_from_this_car_at_ego", "RX hazard at ego"),
            ("semantic_lidar_detection_events", "LiDAR detection events"),
            ("unique_detected_vehicle_ids", "Unique vehicles detected"),
            ("unique_detected_pedestrian_ids", "Unique pedestrians detected"),
            ("average_rx_power_from_this_car_dbm", "Avg RX power (dBm)"),
        ]
        layout.addWidget(self.single_record_table(row, comm_columns))
        layout.addWidget(make_label("RECORDED SAMPLES · LATEST 250 ROWS", "subtitle"))
        sample_columns = [
            ("elapsed_s", "Elapsed (s)"), ("wall_time", "Wall time"), ("x_m", "X (m)"), ("y_m", "Y (m)"),
            ("z_m", "Z (m)"), ("speed_kmh", "Speed (km/h)"), ("accel_mps2", "Accel (m/s²)"),
            ("yaw_deg", "Yaw (°)"), ("steer", "Steer"), ("throttle", "Throttle"), ("brake", "Brake"),
            ("ego_visibility_status", "Visibility"), ("visibility_detail", "Visibility detail"), ("state_source", "State source"),
        ]
        layout.addWidget(self.make_table(samples[-250:], sample_columns, minimum_height=250))
        disclaimer = QLabel("Data interpretation: position and kinematics are sampled CARLA simulator ground truth for experiment evaluation. LOS/NLOS duration is an ego-relative visibility classification, not a per-packet radio NLOS label. UNTRACKED is not NLOS.")
        disclaimer.setWordWrap(True); disclaimer.setStyleSheet(f"color:{MUTED}; padding:10px; background:{PANEL_ALT}; border:1px solid {BORDER}; border-radius:6px;")
        layout.addWidget(disclaimer)
        layout.addStretch(1)
        return scroll

    @staticmethod
    def series(rows: list[dict[str, str]], xkey: str, ykey: str) -> tuple[list[float], list[float]]:
        pairs = []
        for row in rows:
            x = safe_float(row.get(xkey)); y = safe_float(row.get(ykey))
            if x is not None and y is not None:
                pairs.append((x, y))
        return [p[0] for p in pairs], [p[1] for p in pairs]

    @staticmethod
    def empty_plot(ax, text: str):
        ax.text(0.5, 0.5, text, color=MUTED, ha="center", va="center", transform=ax.transAxes, fontsize=9)

    def make_table(self, rows: list[dict[str, Any]], columns: list[tuple[str, str]], minimum_height: int = 140) -> QTableWidget:
        table = QTableWidget(len(rows), len(columns))
        table.setHorizontalHeaderLabels([label for _, label in columns])
        table.setAlternatingRowColors(True)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        table.setSortingEnabled(False)
        table.verticalHeader().setVisible(False)
        table.setMinimumHeight(minimum_height)
        table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        for r_idx, row in enumerate(rows):
            for c_idx, (key, _label) in enumerate(columns):
                value = row.get(key, "")
                text = self.display_table_value(key, value)
                item = QTableWidgetItem(text)
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                table.setItem(r_idx, c_idx, item)
        table.resizeColumnsToContents()
        table.horizontalHeader().setStretchLastSection(True)
        table.setMinimumWidth(min(1300, max(500, table.horizontalHeader().length() + 30)))
        return table

    @staticmethod
    def display_table_value(key: str, value: Any) -> str:
        if key in ("is_ego", "v2x_instrumented_ever", "v2x_instrumented", "alive_at_report_end"):
            return "Yes" if str(value).lower() in ("1", "true", "yes") else "No" if str(value).lower() in ("0", "false", "no") else str(value or "—")
        if value in (None, ""):
            return "—"
        num = safe_float(value)
        if num is not None:
            if key in ("vehicle_id", "sample_count", "brake_events", "tx_state_messages", "tx_object_messages", "tx_hazard_messages",
                       "tx_oversize_messages", "rx_messages_received_by_this_car", "rx_messages_from_this_car_at_ego",
                       "rx_state_messages_from_this_car_at_ego", "rx_object_reports_from_this_car_at_ego", "rx_hazard_reports_from_this_car_at_ego",
                       "semantic_lidar_detection_events", "unique_detected_vehicle_ids", "unique_detected_pedestrian_ids"):
                return f"{int(num):,}"
            return f"{num:,.3f}" if abs(num) < 1000 else f"{num:,.1f}"
        return str(value)

    def single_record_table(self, row: dict[str, Any], columns: list[tuple[str, str]]) -> QTableWidget:
        # Render key/value metric groups as a compact, transposed table.
        table = QTableWidget(len(columns), 2)
        table.setHorizontalHeaderLabels(["Metric", "Value"])
        table.setAlternatingRowColors(True)
        table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        table.verticalHeader().setVisible(False)
        table.setMinimumHeight(min(400, max(100, len(columns) * 27 + 36)))
        table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        for i, (key, label) in enumerate(columns):
            table.setItem(i, 0, QTableWidgetItem(label))
            table.setItem(i, 1, QTableWidgetItem(self.display_table_value(key, row.get(key, ""))))
        table.setColumnWidth(0, 240)
        table.horizontalHeader().setStretchLastSection(True)
        return table


def parse_args():
    parser = argparse.ArgumentParser(description="Browse graphical CARLA V2V ADAS session reports.")
    parser.add_argument("--session-folder", help="Path to one session folder, e.g. session_reports/session_YYYYMMDD_HHMMSS")
    parser.add_argument("--reports-root", default="session_reports", help="Reports root; if --session-folder is omitted, newest report is opened or folder picker appears")
    return parser.parse_args()


def main():
    args = parse_args()
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setFont(QFont("Segoe UI", 9))
    folder = Path(args.session_folder).expanduser() if args.session_folder else Path(args.reports_root).expanduser()
    window = SummaryViewer(folder if folder.exists() else None)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
