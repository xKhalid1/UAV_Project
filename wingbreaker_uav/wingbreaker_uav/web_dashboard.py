"""Local web dashboard for the Wingbreaker UAV interceptor mission.

A self-contained ROS 2 node that serves a single-page dashboard, an SSE live
stream, an MJPEG camera feed and an interception approval endpoint using only
the Python standard library HTTP server (adapted from the pothole inspection
dashboard architecture).
"""

import json
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image

from wingbreaker_interfaces.msg import (
    InterceptReport,
    IntruderDetection,
    MissionState,
    VehicleStatus,
)
from wingbreaker_interfaces.srv import RequestInterception


# MissionState names (IDLE=0 CONNECTING=1 PATROL=2 LOCK=3 DECISION=4 ENGAGE=5 REPORT=6 ERROR=255)
MISSION_NAMES = {
    MissionState.IDLE: "IDLE",
    MissionState.CONNECTING: "CONNECTING",
    MissionState.PATROL: "PATROL",
    MissionState.LOCK: "LOCK",
    MissionState.DECISION: "AWAITING APPROVAL",
    MissionState.ENGAGE: "ENGAGE",
    MissionState.REPORT: "REPORT",
    MissionState.ERROR: "ERROR",
}

APPROVAL_SERVICE = "/request_interception"
APPROVER_HUMAN = "human"

QGC_HOST = "127.0.0.1"
QGC_PORT = 6080
QGC_PROBE_TIMEOUT_S = 0.5
QGC_PROBE_TTL_S = 3.0
SERVICE_CALL_TIMEOUT_S = 10.0


def _format_sse(payload):
    """Return a single-line SSE data frame."""
    raw = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
    return f"data: {raw}\n\n".encode("utf-8")


class _SseClient:
    """One browser tab connected to /events."""

    def __init__(self):
        self.queue = queue.Queue(maxsize=5)
        self.closed = False

    def put(self, event):
        if self.closed:
            return False
        try:
            self.queue.put_nowait(event)
            return True
        except queue.Full:
            self.closed = True
            return False


class _MjpegClient:
    """One browser tab connected to /camera.mjpeg."""

    def __init__(self):
        self.queue = queue.Queue(maxsize=5)
        self.closed = False

    def put(self, frame):
        if self.closed:
            return False
        try:
            self.queue.put_nowait(frame)
            return True
        except queue.Full:
            self.closed = True
            return False


DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Wingbreaker UAV Interceptor Dashboard</title>
<style>
:root {
  --bg: #090b10;
  --bg-2: #0e1118;
  --panel: #121620;
  --panel-2: #0c1016;
  --panel-hover: #171d2a;
  --border: #1d2432;
  --border-2: #2a3344;
  --text: #e8eaf2;
  --text-muted: #8b94a7;
  --text-dim: #5c677d;
  --accent: #2dd4bf;
  --accent-2: #38bdf8;
  --success: #34d399;
  --warn: #f59e0b;
  --danger: #f87171;
  --info: #60a5fa;
  --orange: #fb923c;
  --dark-red: #b91c1c;
  --font-sans: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
  --font-mono: ui-monospace, "SFMono-Regular", "SF Mono", "Cascadia Code", "Fira Code", Consolas, "Liberation Mono", Menlo, monospace;
  --radius: 0.625rem;
  --shadow: 0 1px 0 rgba(255,255,255,0.03) inset, 0 8px 32px rgba(0,0,0,0.28);
  --glow-success: 0 0 0 2px rgba(52,211,153,0.12), 0 0 14px rgba(52,211,153,0.45);
  --glow-warn: 0 0 0 2px rgba(245,158,11,0.12), 0 0 14px rgba(245,158,11,0.45);
  --glow-danger: 0 0 0 2px rgba(248,113,113,0.12), 0 0 14px rgba(248,113,113,0.45);
}
* { box-sizing: border-box; }
html { font-size: 16px; }
body {
  margin: 0;
  font-family: var(--font-sans);
  background: var(--bg);
  color: var(--text);
  line-height: 1.45;
  font-size: 0.875rem;
  min-height: 100vh;
  -webkit-font-smoothing: antialiased;
}
::-webkit-scrollbar { width: 6px; height: 6px; }
::-webkit-scrollbar-track { background: transparent; }
::-webkit-scrollbar-thumb { background: var(--border-2); border-radius: 3px; }
::-webkit-scrollbar-thumb:hover { background: #3d485c; }

header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 1rem;
  padding: 0.875rem 1.5rem;
  background: var(--panel);
  border-bottom: 1px solid var(--border);
  box-shadow: 0 2px 16px rgba(0,0,0,0.25);
  position: sticky;
  top: 0;
  z-index: 100;
}
header h1 { margin: 0; font-size: 1.125rem; font-weight: 800; letter-spacing: -0.02em; }
header h1 span { color: var(--danger); font-weight: 700; }
.header-status { display: flex; align-items: center; gap: 0.75rem; flex-wrap: wrap; }
.status-block, .status-pill {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.3125rem 0.625rem;
  background: var(--panel-2);
  border: 1px solid var(--border);
  border-radius: 999px;
  font-size: 0.6875rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.06em;
  color: var(--text-muted);
}
.dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: #3f485c;
  box-shadow: 0 0 0 2px rgba(63,72,92,0.15);
  transition: background 0.3s, box-shadow 0.3s;
}
.dot.on { background: var(--success); box-shadow: var(--glow-success); }
.dot.warn { background: var(--warn); box-shadow: var(--glow-warn); }
.dot.bad { background: var(--danger); box-shadow: var(--glow-danger); }

.tabs { display: inline-flex; gap: 0.375rem; }
.tab {
  padding: 0.375rem 0.75rem;
  background: var(--panel-2);
  border: 1px solid var(--border);
  border-radius: 0.375rem;
  color: var(--text-muted);
  font-size: 0.6875rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.06em;
  cursor: pointer;
  transition: background 0.2s, color 0.2s, border-color 0.2s;
}
.tab:hover { color: var(--text); border-color: var(--border-2); }
.tab.active { color: var(--accent); border-color: var(--accent); background: rgba(45,212,191,0.08); }

#mission-pill {
  display: inline-flex;
  align-items: center;
  gap: 0.375rem;
  padding: 0.3125rem 0.75rem;
  border-radius: 999px;
  font-size: 0.6875rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.06em;
  background: var(--panel-2);
  color: var(--text-muted);
  border: 1px solid var(--border);
  transition: background 0.2s, color 0.2s, border-color 0.2s, box-shadow 0.2s;
}
#mission-pill::before { content: ""; width: 6px; height: 6px; border-radius: 50%; background: currentColor; }
#mission-pill.state-idle { color: var(--text-muted); }
#mission-pill.state-connecting { background: rgba(56,189,248,0.10); color: var(--accent-2); border-color: rgba(56,189,248,0.28); }
#mission-pill.state-patrol { background: rgba(96,165,250,0.10); color: var(--info); border-color: rgba(96,165,250,0.28); }
#mission-pill.state-lock { background: rgba(251,146,60,0.10); color: var(--orange); border-color: rgba(251,146,60,0.30); box-shadow: 0 0 12px rgba(251,146,60,0.12); }
#mission-pill.state-decision { background: rgba(245,158,11,0.12); color: var(--warn); border-color: rgba(245,158,11,0.32); box-shadow: var(--glow-warn); animation: blink 1.6s ease-in-out infinite; }
#mission-pill.state-engage { background: rgba(248,113,113,0.12); color: var(--danger); border-color: rgba(248,113,113,0.32); box-shadow: var(--glow-danger); animation: blink 0.9s ease-in-out infinite; }
#mission-pill.state-report { background: rgba(52,211,153,0.10); color: var(--success); border-color: rgba(52,211,153,0.28); }
#mission-pill.state-error { background: rgba(185,28,28,0.22); color: #fca5a5; border-color: rgba(185,28,28,0.55); }
@keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.55; } }

#view-qgc { display: none; height: calc(100vh - 3.6rem); }
#view-qgc.active { display: block; }
#view-qgc iframe { width: 100%; height: 100%; border: none; background: var(--bg-2); }
#main.active { display: grid; }
#main { display: none; padding: 1.5rem; grid-template-columns: 21rem 1fr 24rem; grid-template-rows: auto auto 1fr; gap: 1rem;
  grid-template-areas: "telem camera approval" "telem camera detection" "console console report"; }

.panel {
  background: var(--panel);
  border: 1px solid var(--border);
  border-radius: 0.75rem;
  padding: 1rem;
  position: relative;
  overflow: hidden;
  box-shadow: var(--shadow);
  transition: border-color 0.2s;
}
.panel:hover { border-color: var(--border-2); }
.panel > h2 {
  margin: 0 0 0.75rem;
  font-size: 0.6875rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.14em;
  color: var(--text-muted);
  display: flex;
  align-items: center;
  gap: 0.5rem;
}
.panel > h2::before { content: ""; display: inline-block; width: 6px; height: 6px; background: var(--danger); border-radius: 50%; box-shadow: 0 0 8px rgba(248,113,113,0.35); }
#panel-telem { grid-area: telem; }
#panel-camera { grid-area: camera; display: flex; flex-direction: column; }
#panel-approval { grid-area: approval; }
#panel-detection { grid-area: detection; }
#panel-report { grid-area: report; }
#panel-console { grid-area: console; }

.value-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 0.75rem; }
.value { background: var(--panel-2); border: 1px solid var(--border); border-radius: 0.5rem; padding: 0.75rem 0.875rem; transition: border-color 0.2s, transform 0.15s; }
.value:hover { border-color: var(--border-2); transform: translateY(-1px); }
.value.wide { grid-column: 1 / -1; }
.value-label { display: block; font-size: 0.625rem; font-weight: 600; text-transform: uppercase; letter-spacing: 0.08em; color: var(--text-muted); margin-bottom: 0.375rem; }
.value-data { display: block; font-family: var(--font-mono); font-size: 1rem; font-weight: 600; font-variant-numeric: tabular-nums; color: var(--text); letter-spacing: -0.01em; transition: color 0.2s; }
.value-data.missing { color: var(--text-dim); }
.progress-bar { height: 0.375rem; background: var(--panel-2); border-radius: 999px; overflow: hidden; border: 1px solid var(--border); }
.progress-fill { height: 100%; background: linear-gradient(90deg, var(--success), var(--accent)); border-radius: 999px; width: 0%; transition: width 0.35s ease; }
.progress-fill.warn { background: linear-gradient(90deg, var(--warn), #facc15); }
.progress-fill.danger { background: linear-gradient(90deg, #ef4444, var(--warn)); }
.progress-meta { display: flex; justify-content: space-between; font-size: 0.6875rem; color: var(--text-muted); margin-top: 0.375rem; }

#approval-body { display: flex; flex-direction: column; gap: 0.75rem; }
#approval-text { font-size: 0.8125rem; color: var(--text-muted); }
.btn-approve {
  width: 100%;
  padding: 1.125rem 1rem;
  background: linear-gradient(135deg, rgba(52,211,153,0.22), rgba(16,185,129,0.30));
  border: 2px solid rgba(52,211,153,0.75);
  color: #6ee7b7;
  border-radius: 0.625rem;
  font-family: var(--font-sans);
  font-size: 1rem;
  font-weight: 800;
  text-transform: uppercase;
  letter-spacing: 0.12em;
  cursor: pointer;
  transition: background 0.2s, box-shadow 0.2s, transform 0.1s;
  box-shadow: 0 0 0 1px rgba(52,211,153,0.25), 0 0 18px rgba(52,211,153,0.35), inset 0 1px 0 rgba(255,255,255,0.06);
  animation: btn-approve-pulse 1.6s ease-in-out infinite;
}
.btn-approve:hover:not(:disabled) {
  background: linear-gradient(135deg, rgba(52,211,153,0.34), rgba(16,185,129,0.45));
  box-shadow: 0 0 0 1px rgba(52,211,153,0.45), 0 0 28px rgba(52,211,153,0.65), inset 0 1px 0 rgba(255,255,255,0.10);
  color: #a7f3d0;
}
.btn-approve:active:not(:disabled) { transform: translateY(1px); }
.btn-approve:disabled { opacity: 0.5; cursor: wait; animation: none; box-shadow: none; }
@keyframes btn-approve-pulse {
  0%, 100% { box-shadow: 0 0 0 1px rgba(52,211,153,0.25), 0 0 14px rgba(52,211,153,0.30), inset 0 1px 0 rgba(255,255,255,0.06); }
  50%      { box-shadow: 0 0 0 2px rgba(52,211,153,0.45), 0 0 28px rgba(52,211,153,0.70), inset 0 1px 0 rgba(255,255,255,0.10); }
}
.approve-banner {
  margin-bottom: 0.5rem;
  padding: 0.5rem 0.75rem;
  background: linear-gradient(90deg, rgba(52,211,153,0.18), rgba(52,211,153,0.06));
  border: 1px solid rgba(52,211,153,0.5);
  border-left: 3px solid var(--success);
  border-radius: 0.5rem;
  font-size: 0.6875rem;
  font-weight: 800;
  text-transform: uppercase;
  letter-spacing: 0.14em;
  color: var(--success);
  display: flex;
  align-items: center;
  gap: 0.5rem;
}
.approve-banner::before {
  content: "";
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: var(--success);
  box-shadow: var(--glow-success);
  animation: pulse 1.4s infinite;
}

.camera-svg {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}
.detection-box {
  fill: rgba(52,211,153,0.10);
  stroke: var(--success);
  stroke-width: 3;
  stroke-dasharray: 0;
  filter: drop-shadow(0 0 6px rgba(52,211,153,0.55));
  transition: x 0.15s ease-out, y 0.15s ease-out, width 0.15s ease-out, height 0.15s ease-out, opacity 0.25s;
  opacity: 0;
  vector-effect: non-scaling-stroke;
}
.detection-box.visible { opacity: 1; }
.detection-box.high { stroke: var(--danger); fill: rgba(248,113,113,0.10); filter: drop-shadow(0 0 6px rgba(248,113,113,0.55)); }
#approval-status { font-family: var(--font-mono); font-size: 0.75rem; color: var(--text-muted); min-height: 1.1rem; word-break: break-word; }

.card-empty { color: var(--text-dim); font-size: 0.8125rem; text-align: center; padding: 1.5rem 0; }
.threat-chip {
  display: inline-flex;
  align-items: center;
  padding: 0.1875rem 0.5625rem;
  border-radius: 999px;
  font-size: 0.625rem;
  font-weight: 800;
  text-transform: uppercase;
  letter-spacing: 0.08em;
}
.threat-chip.low { background: rgba(52,211,153,0.10); color: var(--success); border: 1px solid rgba(52,211,153,0.28); }
.threat-chip.medium { background: rgba(245,158,11,0.10); color: var(--warn); border: 1px solid rgba(245,158,11,0.28); }
.threat-chip.high { background: rgba(248,113,113,0.12); color: var(--danger); border: 1px solid rgba(248,113,113,0.32); }

.locks-section { margin-top: 0.75rem; border-top: 1px solid var(--border); padding-top: 0.75rem; }
.locks-header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 0.5rem; }
.locks-title { font-size: 0.6875rem; font-weight: 700; text-transform: uppercase; letter-spacing: 0.08em; color: var(--text-muted); }
.locks-count { font-family: var(--font-mono); font-size: 0.6875rem; font-weight: 800; padding: 0.125rem 0.5rem; background: rgba(248,113,113,0.14); color: var(--danger); border: 1px solid rgba(248,113,113,0.4); border-radius: 999px; min-width: 1.5rem; text-align: center; }
.locks-count.zero { background: var(--panel-2); color: var(--text-dim); border-color: var(--border); }
.locks-list { display: flex; flex-direction: column; gap: 0.375rem; max-height: 11rem; overflow-y: auto; }
.locks-list .card-empty.locks-empty { padding: 0.75rem 0; }
.lock-entry {
  background: var(--panel-2);
  border: 1px solid rgba(248,113,113,0.32);
  border-left: 3px solid var(--danger);
  border-radius: 0.5rem;
  padding: 0.5rem 0.625rem;
  font-size: 0.75rem;
  animation: lock-flash 1.2s ease-out;
}
.lock-entry .lock-head { display: flex; align-items: center; justify-content: space-between; gap: 0.5rem; margin-bottom: 0.25rem; }
.lock-entry .lock-conf { font-family: var(--font-mono); font-weight: 800; color: var(--danger); }
.lock-entry .lock-time { font-family: var(--font-mono); color: var(--text-muted); font-size: 0.6875rem; }
.lock-entry .lock-coords { font-family: var(--font-mono); color: var(--text); font-variant-numeric: tabular-nums; }
@keyframes lock-flash {
  0%   { background: rgba(248,113,113,0.35); transform: scale(1.02); }
  60%  { background: rgba(248,113,113,0.18); transform: scale(1); }
  100% { background: var(--panel-2); }
}
.kv { display: flex; justify-content: space-between; gap: 0.75rem; padding: 0.3125rem 0; border-bottom: 1px solid rgba(255,255,255,0.04); font-size: 0.8125rem; }
.kv:last-child { border-bottom: none; }
.kv .k { color: var(--text-muted); }
.kv .v { font-family: var(--font-mono); font-variant-numeric: tabular-nums; color: var(--text); text-align: right; word-break: break-word; }

#flag {
  text-align: center;
  font-size: 1.75rem;
  font-weight: 800;
  letter-spacing: 0.14em;
  text-transform: uppercase;
  padding: 1rem 0.5rem;
  border-radius: 0.5rem;
  margin-bottom: 0.75rem;
  border: 1px solid var(--border);
  background: var(--panel-2);
  color: var(--text-dim);
}
#flag.ok { color: var(--success); border-color: rgba(52,211,153,0.4); background: rgba(52,211,153,0.07); box-shadow: var(--glow-success); }
#flag.fail { color: var(--danger); border-color: rgba(248,113,113,0.4); background: rgba(248,113,113,0.07); box-shadow: var(--glow-danger); }

#camera-wrap { position: relative; flex: 1; min-height: 16rem; background: var(--bg-2); border: 1px solid var(--border); border-radius: var(--radius); overflow: hidden; display: flex; align-items: center; justify-content: center; }
#camera-img { width: 100%; height: 100%; min-height: 16rem; display: block; object-fit: contain; background: var(--bg-2); }
#camera-overlay { position: absolute; inset: 0; display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 0.5rem; color: var(--text-muted); pointer-events: none; background: rgba(9,11,16,0.55); transition: opacity 0.3s; }
#camera-overlay.hidden { opacity: 0; }
.live-badge {
  position: absolute;
  top: 0.5rem;
  left: 0.5rem;
  z-index: 5;
  display: inline-flex;
  align-items: center;
  gap: 0.375rem;
  padding: 0.25rem 0.5rem;
  background: rgba(239,68,68,0.12);
  border: 1px solid rgba(239,68,68,0.35);
  color: var(--danger);
  border-radius: 999px;
  font-size: 0.625rem;
  font-weight: 800;
  text-transform: uppercase;
  letter-spacing: 0.08em;
  opacity: 0;
  transition: opacity 0.3s;
  pointer-events: none;
}
#camera-overlay.hidden ~ .live-badge { opacity: 1; }
.live-badge::before { content: ""; width: 6px; height: 6px; border-radius: 50%; background: var(--danger); box-shadow: var(--glow-danger); animation: pulse 1.4s infinite; }
.pulse { width: 0.625rem; height: 0.625rem; border-radius: 50%; background: var(--accent); box-shadow: 0 0 0 3px rgba(45,212,191,0.12), 0 0 16px rgba(45,212,191,0.45); animation: pulse 1.5s infinite; }
@keyframes pulse { 0% { opacity: 1; transform: scale(1); } 50% { opacity: 0.4; transform: scale(1.4); } 100% { opacity: 1; transform: scale(1); } }

#console-wrap { position: relative; height: 13rem; background: var(--bg-2); border: 1px solid var(--border); border-radius: var(--radius); overflow: hidden; }
#console-log { height: 100%; overflow-y: auto; padding: 0.75rem; font-family: var(--font-mono); font-size: 0.8125rem; line-height: 1.55; color: var(--text); }
#console-log .entry { padding: 0.25rem 0; border-bottom: 1px solid rgba(255,255,255,0.04); white-space: pre-wrap; word-break: break-word; }
#console-log .entry:last-child { border-bottom: none; }
#console-log .entry .lvl { font-weight: 700; margin-right: 0.4rem; font-family: var(--font-sans); font-size: 0.6875rem; }
#console-log .entry .node { color: var(--text-muted); margin-right: 0.4rem; }
#console-log .debug { color: var(--text-dim); }
#console-log .info { color: var(--text); }
#console-log .warn { color: #facc15; }
#console-log .error { color: var(--danger); }
#console-log .fatal { color: #fb7185; }

.overlay {
  position: fixed;
  inset: 0;
  background: rgba(9,11,16,0.88);
  display: none;
  align-items: center;
  justify-content: center;
  z-index: 20;
}
.overlay.active { display: flex; }
.overlay .box { background: var(--panel); border: 1px solid var(--border-2); border-radius: 0.75rem; padding: 1.5rem 2rem; text-align: center; box-shadow: var(--shadow); }
.overlay h3 { margin: 0.75rem 0 0; font-size: 1rem; color: var(--text); }
.overlay p { margin: 0.5rem 0 0; font-size: 0.8125rem; color: var(--text-muted); }

@media (max-width: 1100px) {
  #main { grid-template-columns: 1fr; grid-template-rows: auto; grid-template-areas: "telem" "camera" "approval" "detection" "report" "console"; }
}
</style>
</head>
<body>
<header>
  <h1>Wingbreaker <span>Interceptor</span> Dashboard</h1>
  <div class="header-status">
    <div class="tabs">
      <button class="tab active" id="tab-dash">Dashboard</button>
      <button class="tab" id="tab-qgc" style="display:none;">QGC (noVNC)</button>
    </div>
    <div class="status-block"><span class="dot" id="conn-dot"></span><span id="conn-label">Unknown</span></div>
    <div class="status-block"><span class="dot" id="arm-dot"></span><span id="arm-label">Unknown</span></div>
    <div class="status-block"><span class="dot" id="air-dot"></span><span id="air-label">Unknown</span></div>
    <div id="mission-pill">Mission: --</div>
  </div>
</header>

<div id="main" class="active">
  <section class="panel" id="panel-telem">
    <h2>Telemetry</h2>
    <div class="value-grid">
      <div class="value wide">
        <span class="value-label">Battery</span>
        <div class="progress-bar"><div class="progress-fill" id="battery-fill"></div></div>
        <div class="progress-meta"><span>Battery</span><span id="battery-pct">--</span></div>
      </div>
      <div class="value"><span class="value-label">Relative Altitude</span><span class="value-data missing" id="val-alt">--</span></div>
      <div class="value"><span class="value-label">Ground Speed</span><span class="value-data missing" id="val-speed">--</span></div>
      <div class="value"><span class="value-label">Satellites</span><span class="value-data missing" id="val-sats">--</span></div>
      <div class="value"><span class="value-label">Heading</span><span class="value-data missing" id="val-heading">--</span></div>
      <div class="value"><span class="value-label">Latitude</span><span class="value-data missing" id="val-lat">--</span></div>
      <div class="value"><span class="value-label">Longitude</span><span class="value-data missing" id="val-lon">--</span></div>
    </div>
  </section>

  <section class="panel" id="panel-camera">
    <h2>Onboard Camera</h2>
    <div id="camera-wrap">
      <img id="camera-img" src="/camera.mjpeg" alt="Live camera" onload="document.getElementById('camera-overlay').classList.add('hidden')">
      <svg id="camera-svg" class="camera-svg" viewBox="0 0 1280 720" preserveAspectRatio="xMidYMid meet" aria-hidden="true">
        <rect id="detection-box" class="detection-box" x="0" y="0" width="0" height="0" />
      </svg>
      <div id="camera-overlay">
        <div class="pulse"></div>
        <div>Waiting for camera</div>
      </div>
      <div class="live-badge">Live</div>
    </div>
  </section>

  <section class="panel" id="panel-approval" style="display:none;">
    <h2>Approval Required</h2>
    <div id="approval-body">
      <div class="approve-banner">Decision Required &middot; Authorize Interception</div>
      <div id="approval-text">Intruder locked. Interception is awaiting authorization.</div>
      <button class="btn-approve" id="approve-btn">Approve Interception</button>
      <div id="approval-status"></div>
    </div>
  </section>

  <section class="panel" id="panel-detection">
    <h2>Detections</h2>
    <div id="detection-card"><div class="card-empty">No detections yet</div></div>
    <div class="locks-section">
      <div class="locks-header">
        <span class="locks-title">Locks (drone &ge; 0.85)</span>
        <span class="locks-count" id="locks-count">0</span>
      </div>
      <div id="locks-list" class="locks-list"><div class="card-empty locks-empty">No locks yet</div></div>
    </div>
  </section>

  <section class="panel" id="panel-report">
    <h2>Latest Intercept Report</h2>
    <div id="flag">No report yet</div>
    <div id="report-card"><div class="card-empty">Awaiting engagement outcome</div></div>
  </section>

  <section class="panel" id="panel-console">
    <h2>Mission Console</h2>
    <div id="console-wrap"><div id="console-log"></div></div>
  </section>
</div>

<div id="view-qgc">
  <iframe id="qgc-frame" src="about:blank" title="QGroundControl via noVNC"></iframe>
</div>

<div class="overlay" id="waiting-overlay">
  <div class="box">
    <div class="pulse"></div>
    <h3>Waiting for telemetry</h3>
    <p>Connecting to vehicle stream...</p>
  </div>
</div>

<script>
const QGC_AVAILABLE = __QGC_AVAILABLE__;
(function() {
  const STATE_NAMES = { 0: 'Idle', 1: 'Connecting', 2: 'Patrol', 3: 'Lock', 4: 'Awaiting approval', 5: 'Engage', 6: 'Report', 255: 'Error' };
  const STATE_CLASSES = { 0: 'state-idle', 1: 'state-connecting', 2: 'state-patrol', 3: 'state-lock', 4: 'state-decision', 5: 'state-engage', 6: 'state-report', 255: 'state-error' };

  const S = {
    vehicle: { connected: false, armed: false, airborne: false, lat: null, lon: null, alt: null, speed: null, heading: null, battery_percent: -1, num_satellites: -1 },
    mission: { state: 0, name: 'IDLE', detail: '' },
    detection: null,
    report: null,
    logs: [],
    lock_history: [],
    lock_latest_ts: 0.0,
  };
  let approvePending = false;

  const el = (id) => document.getElementById(id);
  const fmt = (n, d) => n === null || n === undefined || (typeof n === 'number' && isNaN(n)) ? '--' : Number(n).toFixed(d);
  const hasValue = (v) => v !== null && v !== undefined && !(typeof v === 'number' && isNaN(v));
  const fmtTime = (ts) => {
    if (!ts) return '';
    const d = new Date(ts * 1000);
    return d.toLocaleTimeString('en-US', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
  };
  function setVal(id, text, ok) { const e = el(id); e.textContent = text; e.classList.toggle('missing', !ok); }

  function update(data) {
    if (data.vehicle) Object.assign(S.vehicle, data.vehicle);
    if (data.mission) Object.assign(S.mission, data.mission);
    if (data.detection !== undefined) S.detection = data.detection;
    if (data.report !== undefined) S.report = data.report;
    if (Array.isArray(data.lock_history)) S.lock_history = data.lock_history;
    if (typeof data.lock_latest_ts === 'number') S.lock_latest_ts = data.lock_latest_ts;
    if (data.new_log) {
      const last = S.logs[S.logs.length - 1];
      const key = (l) => (l.timestamp || 0) + '|' + l.level + '|' + l.name + '|' + l.message;
      if (!last || key(last) !== key(data.new_log)) S.logs.push(data.new_log);
      if (S.logs.length > 200) S.logs = S.logs.slice(-200);
    }
    render();
  }

  function render() {
    renderHeader();
    renderTelemetry();
    renderApproval();
    renderDetection();
    renderLocks();
    renderReport();
    renderCameraBox();
    renderConsole();
    const seenAny = S.vehicle.lat !== null || S.vehicle.connected;
    el('waiting-overlay').classList.toggle('active', !seenAny && S.mission.state === 0);
  }

  function renderHeader() {
    el('conn-dot').className = 'dot' + (S.vehicle.connected ? ' on' : ' bad');
    el('arm-dot').className = 'dot' + (S.vehicle.armed ? ' warn' : '');
    el('air-dot').className = 'dot' + (S.vehicle.airborne ? ' on' : '');
    el('conn-label').textContent = S.vehicle.connected ? 'Connected' : 'Disconnected';
    el('arm-label').textContent = S.vehicle.armed ? 'Armed' : 'Disarmed';
    el('air-label').textContent = S.vehicle.airborne ? 'Airborne' : 'Ground';
    const pill = el('mission-pill');
    const name = STATE_NAMES[S.mission.state] || S.mission.name || 'Unknown';
    pill.textContent = 'Mission: ' + (S.mission.detail ? name + ' - ' + S.mission.detail : name);
    pill.className = STATE_CLASSES[S.mission.state] || 'state-idle';
  }

  function renderTelemetry() {
    const v = S.vehicle;
    const bp = hasValue(v.battery_percent) && v.battery_percent >= 0 ? v.battery_percent : null;
    const fill = el('battery-fill');
    if (bp !== null) {
      const pct = Math.max(0, Math.min(100, bp));
      fill.style.width = pct + '%';
      fill.className = 'progress-fill' + (pct < 20 ? ' danger' : pct < 40 ? ' warn' : '');
      el('battery-pct').textContent = pct.toFixed(0) + '%';
    } else {
      fill.style.width = '0%';
      fill.className = 'progress-fill';
      el('battery-pct').textContent = '--';
    }
    const altOk = hasValue(v.alt);
    const spdOk = hasValue(v.speed);
    const satsOk = hasValue(v.num_satellites) && v.num_satellites >= 0;
    const hdgOk = hasValue(v.heading);
    const latOk = hasValue(v.lat);
    const lonOk = hasValue(v.lon);
    setVal('val-alt', altOk ? fmt(v.alt, 2) + ' m' : '--', altOk);
    setVal('val-speed', spdOk ? fmt(v.speed, 2) + ' m/s' : '--', spdOk);
    setVal('val-sats', satsOk ? String(v.num_satellites) : '--', satsOk);
    setVal('val-heading', hdgOk ? fmt(v.heading, 0) + ' deg' : '--', hdgOk);
    setVal('val-lat', latOk ? fmt(v.lat, 6) : '--', latOk);
    setVal('val-lon', lonOk ? fmt(v.lon, 6) : '--', lonOk);
  }

  function renderApproval() {
    const show = S.mission.state === 4;
    el('panel-approval').style.display = show ? '' : 'none';
    const btn = el('approve-btn');
    btn.disabled = approvePending;
  }

  function renderDetection() {
    const card = el('detection-card');
    const d = S.detection;
    if (!d) {
      card.innerHTML = '<div class="card-empty">No detections yet</div>';
      return;
    }
    const lvl = String(d.threat_level || 'UNKNOWN').toUpperCase();
    const cls = lvl === 'HIGH' ? 'high' : lvl === 'MEDIUM' ? 'medium' : lvl === 'LOW' ? 'low' : '';
    const conf = hasValue(d.confidence) ? (d.confidence * 100).toFixed(1) + '%' : '--';
    card.innerHTML =
      '<div class="kv"><span class="k">Threat</span><span class="v"><span class="threat-chip ' + cls + '">' + lvl + '</span></span></div>' +
      '<div class="kv"><span class="k">Confidence</span><span class="v">' + conf + '</span></div>' +
      '<div class="kv"><span class="k">Latitude</span><span class="v">' + fmt(d.latitude, 6) + '</span></div>' +
      '<div class="kv"><span class="k">Longitude</span><span class="v">' + fmt(d.longitude, 6) + '</span></div>' +
      '<div class="kv"><span class="k">Altitude</span><span class="v">' + fmt(d.altitude, 1) + ' m</span></div>' +
      '<div class="kv"><span class="k">Time</span><span class="v">' + fmtTime(d.timestamp) + '</span></div>';
  }

  function renderLocks() {
    const list = el('locks-list');
    const count = el('locks-count');
    const locks = S.lock_history || [];
    count.textContent = String(locks.length);
    count.classList.toggle('zero', locks.length === 0);
    if (locks.length === 0) {
      list.innerHTML = '<div class="card-empty locks-empty">No locks yet</div>';
      return;
    }
    // Render newest first
    const rows = [];
    for (let i = locks.length - 1; i >= 0; i--) {
      const d = locks[i];
      const conf = hasValue(d.confidence) ? (d.confidence * 100).toFixed(1) + '%' : '--';
      rows.push(
        '<div class="lock-entry">' +
          '<div class="lock-head">' +
            '<span class="lock-conf">drone ' + conf + '</span>' +
            '<span class="lock-time">' + fmtTime(d.timestamp) + '</span>' +
          '</div>' +
          '<div class="lock-coords">' +
            fmt(d.latitude, 6) + ', ' + fmt(d.longitude, 6) + ' &middot; alt ' + fmt(d.altitude, 1) + ' m' +
          '</div>' +
        '</div>'
      );
    }
    list.innerHTML = rows.join('');
  }

  function renderCameraBox() {
    const box = el('detection-box');
    if (!box) return;
    const d = S.detection;
    if (!d || !d.roi_w || !d.roi_h) {
      box.classList.remove('visible');
      return;
    }
    box.setAttribute('x', String(d.roi_x || 0));
    box.setAttribute('y', String(d.roi_y || 0));
    box.setAttribute('width', String(d.roi_w));
    box.setAttribute('height', String(d.roi_h));
    const isHigh = String(d.threat_level || '').toUpperCase() === 'HIGH';
    box.classList.toggle('high', isHigh);
    box.classList.add('visible');
  }

  function renderReport() {
    const flag = el('flag');
    const card = el('report-card');
    const r = S.report;
    if (!r) {
      flag.textContent = 'No report yet';
      flag.className = '';
      card.innerHTML = '<div class="card-empty">Awaiting engagement outcome</div>';
      return;
    }
    if (r.intercepted) {
      flag.textContent = 'INTERCEPTED';
      flag.className = 'ok';
    } else if (r.detected) {
      flag.textContent = 'NOT INTERCEPTED';
      flag.className = 'fail';
    } else {
      flag.textContent = 'NO TARGET';
      flag.className = 'fail';
    }
    card.innerHTML =
      '<div class="kv"><span class="k">Location</span><span class="v">' + (r.location || '--') + '</span></div>' +
      '<div class="kv"><span class="k">Approved by</span><span class="v">' + (r.approved_by || '--') + '</span></div>' +
      '<div class="kv"><span class="k">Confidence</span><span class="v">' + (hasValue(r.detection_confidence) ? (r.detection_confidence * 100).toFixed(1) + '%' : '--') + '</span></div>' +
      '<div class="kv"><span class="k">Coordinates</span><span class="v">' + fmt(r.latitude_deg, 6) + ', ' + fmt(r.longitude_deg, 6) + '</span></div>' +
      '<div class="kv"><span class="k">Altitude</span><span class="v">' + fmt(r.altitude_m, 1) + ' m</span></div>' +
      '<div class="kv"><span class="k">Message</span><span class="v">' + (r.message || '--') + '</span></div>' +
      '<div class="kv"><span class="k">Time</span><span class="v">' + fmtTime(r.timestamp) + '</span></div>';
  }

  function renderConsole() {
    const wrap = el('console-log');
    const nearBottom = wrap.scrollHeight - wrap.scrollTop - wrap.clientHeight < 20;
    wrap.innerHTML = '';
    const levelMap = { 10: ['DEBUG', 'debug'], 20: ['INFO', 'info'], 30: ['WARN', 'warn'], 40: ['ERROR', 'error'], 50: ['FATAL', 'fatal'] };
    for (const line of S.logs.slice(-200)) {
      const entry = document.createElement('div');
      const info = levelMap[line.level] || ['LOG', 'info'];
      entry.className = 'entry ' + info[1];
      entry.innerHTML = '<span class="lvl">[' + info[0] + ']</span><span class="node">[' + (line.name || 'unknown') + ']</span>' + (line.message || '');
      wrap.appendChild(entry);
    }
    if (nearBottom) wrap.scrollTop = wrap.scrollHeight;
  }

  el('approve-btn').addEventListener('click', async () => {
    const btn = el('approve-btn');
    const status = el('approval-status');
    approvePending = true;
    btn.disabled = true;
    status.textContent = 'Requesting authorization...';
    try {
      const res = await fetch('/approve', { method: 'POST' });
      const out = await res.json();
      status.textContent = out.message || (out.success ? 'Approved.' : 'Request rejected.');
    } catch (err) {
      status.textContent = 'Request failed: ' + err;
    } finally {
      approvePending = false;
      btn.disabled = false;
    }
  });

  // Tabs: dashboard view and optional QGC (noVNC) view.
  let qgcLoaded = false;
  function switchView(view) {
    const dash = view === 'dash';
    el('main').classList.toggle('active', dash);
    el('view-qgc').classList.toggle('active', !dash);
    el('tab-dash').classList.toggle('active', dash);
    el('tab-qgc').classList.toggle('active', !dash);
    if (!dash && !qgcLoaded) {
      el('qgc-frame').src = 'http://' + window.location.hostname + ':6080/vnc.html?autoconnect=1';
      qgcLoaded = true;
    }
  }
  el('tab-dash').addEventListener('click', () => switchView('dash'));
  el('tab-qgc').addEventListener('click', () => switchView('qgc'));
  if (QGC_AVAILABLE) el('tab-qgc').style.display = '';

  const es = new EventSource('/events');
  es.onmessage = (e) => {
    try { update(JSON.parse(e.data)); } catch (err) { console.error('Bad SSE payload', err); }
  };
  es.onerror = () => { console.warn('SSE error - reconnecting'); };
})();
</script>
</body>
</html>
"""


class DashboardHandler(BaseHTTPRequestHandler):
    """HTTP request handler: serves the dashboard page, streams and approval."""

    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):  # noqa: A002 - stdlib signature
        self.server.dashboard.get_logger().info(f"{self.address_string()} - {format % args}")

    def do_GET(self):
        if self.path == "/":
            self._serve_page()
        elif self.path == "/events":
            self._serve_events()
        elif self.path == "/camera.mjpeg":
            self._serve_mjpeg()
        elif self.path == "/api/state":
            self._serve_state()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == "/approve":
            self._serve_approve()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_page(self):
        node = self.server.dashboard
        qgc_flag = "true" if node.qgc_available() else "false"
        body = DASHBOARD_HTML.replace("__QGC_AVAILABLE__", qgc_flag).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_state(self):
        node = self.server.dashboard
        body = json.dumps(node.build_event("api")).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_approve(self):
        node = self.server.dashboard
        length = int(self.headers.get("Content-Length") or 0)
        if length > 0:
            self.rfile.read(length)  # drain request body, if any
        success, message = node.request_interception_approval()
        node.get_logger().info(f"Approval request result: success={success} message='{message}'")
        body = json.dumps({"success": success, "message": message}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_events(self):
        node = self.server.dashboard
        client = _SseClient()
        with node._sse_clients_lock:
            node._sse_clients.append(client)

        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        try:
            self.wfile.write(b"retry: 2000\n\n")
            self.wfile.flush()

            while not node._shutting_down and not client.closed:
                try:
                    event = client.queue.get(timeout=1.0)
                except queue.Empty:
                    if node._shutting_down:
                        break
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    continue

                if event is None:
                    break

                try:
                    self.wfile.write(_format_sse(event))
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break
        finally:
            client.closed = True
            with node._sse_clients_lock:
                try:
                    node._sse_clients.remove(client)
                except ValueError:
                    pass

    def _serve_mjpeg(self):
        node = self.server.dashboard
        client = _MjpegClient()
        with node._mjpeg_clients_lock:
            node._mjpeg_clients.append(client)

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        try:
            self.wfile.write(b"--frame\r\n")
            self.wfile.flush()

            while not node._shutting_down and not client.closed:
                try:
                    frame = client.queue.get(timeout=1.0)
                except queue.Empty:
                    continue

                if frame is None:
                    break

                try:
                    self.wfile.write(
                        b"Content-Type: image/jpeg\r\nContent-Length: "
                        + str(len(frame)).encode("utf-8")
                        + b"\r\n\r\n"
                    )
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n--frame\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break
        finally:
            client.closed = True
            with node._mjpeg_clients_lock:
                try:
                    node._mjpeg_clients.remove(client)
                except ValueError:
                    pass


class DashboardServer(ThreadingHTTPServer):
    """ThreadingHTTPServer carrying a reference to the dashboard node."""

    dashboard: "WebDashboard"


class WebDashboard(Node):
    """ROS 2 node that tracks interceptor state and serves the web dashboard."""

    def __init__(self):
        super().__init__("web_dashboard")
        self.declare_parameter("port", 8080)
        port = int(self.get_parameter("port").value)

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self._camera_topic = str(self.get_parameter("camera_topic").value)

        self._bridge = CvBridge()

        self._state = {
            "vehicle": {
                "connected": False,
                "armed": False,
                "airborne": False,
                "lat": None,
                "lon": None,
                "alt": None,
                "speed": None,
                "heading": None,
                "battery_percent": -1.0,
                "num_satellites": -1,
            },
            "mission": {
                "state": MissionState.IDLE,
                "name": MISSION_NAMES[MissionState.IDLE],
                "detail": "",
            },
            "detection": None,
            "report": None,
            "logs": [],
            "lock_history": [],
            "lock_latest_ts": 0.0,
        }

        # cap for the "Locks (drone >=0.85)" panel history
        self._lock_history_max = 15
        self._lock_conf_threshold = 0.85  # HIGH threat == conf >= 0.85

        self._sse_clients = []
        self._sse_clients_lock = threading.Lock()

        self._latest_image = None
        self._latest_image_lock = threading.Lock()
        self._mjpeg_clients = []
        self._mjpeg_clients_lock = threading.Lock()

        self._qgc_cache = {"available": False, "checked_at": 0.0}
        self._qgc_lock = threading.Lock()

        self._shutting_down = False

        self.create_subscription(VehicleStatus, "vehicle_status", self._on_vehicle, 10)
        self.create_subscription(MissionState, "mission_state", self._on_mission, 10)
        self.create_subscription(IntruderDetection, "detections", self._on_detection, 10)
        self.create_subscription(InterceptReport, "intercept_reports", self._on_report, 10)
        self.create_subscription(Log, "/rosout", self._on_rosout, 10)
        self.create_subscription(Image, self._camera_topic, self._on_image, qos_profile_sensor_data)

        self._intercept_client = self.create_client(RequestInterception, APPROVAL_SERVICE)

        self.create_timer(0.5, self._publish_snapshot)

        self._server = DashboardServer(("127.0.0.1", port), DashboardHandler)
        self._server.dashboard = self
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

        self._camera_thread = threading.Thread(target=self._camera_broadcast_loop, daemon=True)
        self._camera_thread.start()

        self.get_logger().info(f"Dashboard serving at http://127.0.0.1:{port}/")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_vehicle(self, msg):
        v = self._state["vehicle"]
        v["connected"] = bool(msg.connected)
        v["armed"] = bool(msg.armed)
        v["airborne"] = bool(getattr(msg, "airborne", False))
        v["lat"] = float(msg.latitude_deg)
        v["lon"] = float(msg.longitude_deg)
        v["alt"] = float(msg.relative_altitude_m)
        v["speed"] = float(msg.ground_speed_m_s)
        v["heading"] = float(msg.heading_deg)
        v["battery_percent"] = float(msg.battery_percent)
        v["num_satellites"] = int(msg.num_satellites)
        self._push_event(self.build_event("vehicle"))

    def _on_mission(self, msg):
        m = self._state["mission"]
        state = int(msg.state)
        m["state"] = state
        m["name"] = MISSION_NAMES.get(state, f"UNKNOWN({state})")
        m["detail"] = str(msg.detail)
        self.get_logger().info(f"Mission state: {m['name']} {('[%s]' % m['detail']) if m['detail'] else ''}")
        self._push_event(self.build_event("mission"))

    def _on_detection(self, msg):
        ts = self._stamp_to_epoch(msg.header)
        roi = getattr(msg, 'roi', None)
        det = {
            "latitude": float(msg.latitude),
            "longitude": float(msg.longitude),
            "altitude": float(msg.altitude),
            "confidence": float(msg.confidence),
            "threat_level": str(msg.threat_level),
            "timestamp": ts if ts > 0.0 else time.time(),
            # pixel bounding box in the original 1280x720 camera frame
            "roi_x": int(getattr(roi, 'x_offset', 0)) if roi else 0,
            "roi_y": int(getattr(roi, 'y_offset', 0)) if roi else 0,
            "roi_w": int(getattr(roi, 'width', 0)) if roi else 0,
            "roi_h": int(getattr(roi, 'height', 0)) if roi else 0,
        }
        self._state["detection"] = det
        # Record a lock event when the detector classifies the detection at
        # HIGH threat (conf >= 0.85) - i.e. drone >=0.85 in the dashboard.
        if det["confidence"] >= self._lock_conf_threshold:
            lock = dict(det)
            lock["id"] = det["timestamp"]
            self._state["lock_history"].append(lock)
            if len(self._state["lock_history"]) > self._lock_history_max:
                self._state["lock_history"] = (
                    self._state["lock_history"][-self._lock_history_max:])
            self._state["lock_latest_ts"] = det["timestamp"]
        self._push_event(self.build_event("detection"))

    def _on_report(self, msg):
        ts = self._stamp_to_epoch(msg.header)
        rep = {
            "detected": bool(msg.detected),
            "intercepted": bool(msg.intercepted),
            "location": str(msg.location),
            "latitude_deg": float(msg.latitude_deg),
            "longitude_deg": float(msg.longitude_deg),
            "altitude_m": float(msg.altitude_m),
            "detection_confidence": float(msg.detection_confidence),
            "approved_by": str(msg.approved_by),
            "message": str(msg.message),
            "timestamp": ts if ts > 0.0 else time.time(),
        }
        self._state["report"] = rep
        self.get_logger().info(
            f"Intercept report: intercepted={rep['intercepted']} approved_by='{rep['approved_by']}'"
        )
        self._push_event(self.build_event("report"))

    def _on_image(self, msg):
        with self._latest_image_lock:
            self._latest_image = msg

    def _on_rosout(self, msg):
        stamp = getattr(msg, "stamp", None)
        ts = time.time()
        if stamp is not None:
            ts = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        entry = {
            "level": int(msg.level),
            "name": str(msg.name),
            "message": str(msg.msg),
            "timestamp": ts,
        }
        self._state["logs"].append(entry)
        if len(self._state["logs"]) > 200:
            self._state["logs"].pop(0)
        self._push_event(self.build_event("log", new_log=entry))

    # ------------------------------------------------------------------
    # Approval service
    # ------------------------------------------------------------------

    def request_interception_approval(self):
        """Call /request_interception with approve=True approver='human'.

        Safe to call from an HTTP handler thread: call_async hands the request
        to the executor, and we poll the future from here without spinning.
        """
        if self._shutting_down:
            return False, "dashboard is shutting down"
        request = RequestInterception.Request()
        request.approve = True
        request.approver = APPROVER_HUMAN
        future = self._intercept_client.call_async(request)
        deadline = time.monotonic() + SERVICE_CALL_TIMEOUT_S
        while not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                return False, "approval service timed out"
            time.sleep(0.05)
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - surface any service failure to the browser
            return False, f"service call failed: {exc}"
        if response is None:
            return False, "service returned no response"
        return bool(response.success), str(response.message)

    # ------------------------------------------------------------------
    # QGC (noVNC) availability probe
    # ------------------------------------------------------------------

    def qgc_available(self):
        """Probe the noVNC endpoint (TCP 127.0.0.1:6080) with a short TTL cache."""
        now = time.monotonic()
        with self._qgc_lock:
            if now - self._qgc_cache["checked_at"] < QGC_PROBE_TTL_S:
                return self._qgc_cache["available"]
        available = False
        try:
            with socket.create_connection((QGC_HOST, QGC_PORT), timeout=QGC_PROBE_TIMEOUT_S):
                available = True
        except OSError:
            available = False
        with self._qgc_lock:
            self._qgc_cache["available"] = available
            self._qgc_cache["checked_at"] = now
        return available

    # ------------------------------------------------------------------
    # State distribution
    # ------------------------------------------------------------------

    @staticmethod
    def _stamp_to_epoch(header):
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return 0.0
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _publish_snapshot(self):
        self._push_event(self.build_event("telemetry"))

    def build_event(self, trigger, new_log=None):
        """Full state snapshot sent to browsers (SSE payload and /api/state)."""
        return {
            "trigger": trigger,
            "ts": time.time(),
            "vehicle": dict(self._state["vehicle"]),
            "mission": dict(self._state["mission"]),
            "detection": self._state["detection"],
            "report": self._state["report"],
            "logs": list(self._state["logs"]),
            "new_log": new_log,
            "lock_history": list(self._state["lock_history"]),
            "lock_latest_ts": float(self._state["lock_latest_ts"]),
            "qgc_available": self.qgc_available(),
        }

    def _push_event(self, event):
        if self._shutting_down:
            return
        with self._sse_clients_lock:
            clients = list(self._sse_clients)
        for client in clients:
            if not client.put(event):
                self._close_client(client)

    def _close_client(self, client):
        client.closed = True
        with self._sse_clients_lock:
            try:
                self._sse_clients.remove(client)
            except ValueError:
                pass

    # ------------------------------------------------------------------
    # Camera MJPEG encoding
    # ------------------------------------------------------------------

    def _camera_broadcast_loop(self):
        while not self._shutting_down:
            time.sleep(0.2)
            with self._latest_image_lock:
                msg = self._latest_image
            if msg is None:
                continue
            try:
                img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                h, w = img.shape[:2]
                if w > 640:
                    scale = 640 / w
                    img = cv2.resize(img, (640, int(h * scale)), interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if not ok:
                    continue
                frame = buf.tobytes()
            except Exception as e:
                self.get_logger().warning(f"Camera encode error: {e}")
                continue
            self._broadcast_camera_frame(frame)

    def _broadcast_camera_frame(self, frame):
        if self._shutting_down:
            return
        with self._mjpeg_clients_lock:
            clients = list(self._mjpeg_clients)
        for client in clients:
            if not client.put(frame):
                self._close_camera_client(client)

    def _close_camera_client(self, client):
        client.closed = True
        with self._mjpeg_clients_lock:
            try:
                self._mjpeg_clients.remove(client)
            except ValueError:
                pass

    def destroy_node(self):
        self._shutting_down = True
        with self._sse_clients_lock:
            clients = list(self._sse_clients)
            self._sse_clients.clear()
        for client in clients:
            client.closed = True
        with self._mjpeg_clients_lock:
            camera_clients = list(self._mjpeg_clients)
            self._mjpeg_clients.clear()
        for client in camera_clients:
            client.closed = True
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:
            pass
        super().destroy_node()


def main():
    import rclpy
    from rclpy import init, shutdown, spin

    init()
    node = WebDashboard()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
