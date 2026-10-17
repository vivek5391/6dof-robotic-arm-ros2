#!/usr/bin/env python3
import os
import json
import time
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler


JOINT_NAMES  = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
LOWER_LIMITS = np.array([-1.57, -0.52, -1.13, -1.57, -1.05])
UPPER_LIMITS = np.array([ 1.57,  1.31,  1.22,  1.57,  1.05])
GRIPPER_MIN  =  0.00
GRIPPER_MAX  =  0.65

PACKAGE_SHARE = get_package_share_directory('arm_description')
WAYPOINTS_DIR = os.path.join(os.path.expanduser('~'), '.ros', 'arm_description')
WAYPOINTS_FILE = os.path.join(WAYPOINTS_DIR, 'saved_waypoints.json')
MESH_DIR = os.path.join(PACKAGE_SHARE, 'meshes')
STATIC_DIR = os.path.join(PACKAGE_SHARE, 'static')
WEB_PORT = 8080


def trans(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def rpy(r, p, y):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [r, p, y]).as_matrix()
    return T


def rotz(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([
        [ c, -s, 0.0, 0.0],
        [ s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rotx(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0,  c, -s, 0.0],
        [0.0,  s,  c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


T_BASE_J1 = trans(-0.10, 0.0, 0.07)
T_J1_J2   = trans(0.00906, -0.01413, 0.03490) @ rpy(1.5708, 0.0, -1.5708)
T_J2_J3   = trans(0.1115, 0.0, 0.0) @ rpy(3.1416, 0.0, 1.5708)
T_J3_J4   = trans(0.1035, -0.008, 0.01825) @ rpy(3.14159, 0.0, 0.0)
T_J4_J5   = trans(0.021, 0.0, -0.0165) @ rpy(3.14159, -1.5708, 0.0)
T_J5_TCP  = trans(-0.013936, 0.120, 0.0165)


def forward_kinematics_chain(q):
    q1, q2, q3, q4, q5 = q
    T1 = T_BASE_J1 @ rotz(q1)
    T2 = T1 @ T_J1_J2 @ rotz(q2)
    T3 = T2 @ T_J2_J3 @ rotz(q3)
    T4 = T3 @ T_J3_J4 @ rotx(q4)
    T5 = T4 @ T_J4_J5 @ rotx(q5)
    Ttcp = T5 @ T_J5_TCP
    return T1, T2, T3, T4, T5, Ttcp


def forward_kinematics(q):
    return forward_kinematics_chain(q)[-1]


def check_collision(q):
    if np.any(q < LOWER_LIMITS - 1e-4) or np.any(q > UPPER_LIMITS + 1e-4):
        return False, 'Joint Angle Limit Reached'

    _, _, T3, T4, T5, Ttcp = forward_kinematics_chain(q)
    for name, T in [('Elbow', T3), ('Wrist', T4), ('Gripper', T5), ('TCP', Ttcp)]:
        p = T[:3, 3]
        if p[2] < 0.025:
            return False, f'Ground Collision Guard ({name} Z < 25mm)'
        if name in ('Wrist', 'Gripper', 'TCP'):
            if (-0.185 <= p[0] <= 0.02) and (-0.090 <= p[1] <= 0.090) and (p[2] < 0.085):
                return False, f'Base Self-Collision Guard ({name})'

    return True, 'SAFE (Collision-Free)'


SORT_CONFIG = {
    'red': {
        'q_pick':  np.array([ 0.70, 0.50, 0.35, 0.0, -0.30]),
        'q_Lift1': np.array([ 0.70, 0.08, 0.08, 0.0,  0.00]),
        'q_Lift2': np.array([-0.45, 0.08, 0.08, 0.0,  0.00]),
        'q_place': np.array([-0.45, 0.50, 0.35, 0.0, -0.30]),
        'rgb': (1.0, 0.22, 0.22),
    },
    'green': {
        'q_pick':  np.array([ 0.38, 0.50, 0.35, 0.0, -0.30]),
        'q_Lift1': np.array([ 0.38, 0.08, 0.08, 0.0,  0.00]),
        'q_Lift2': np.array([-0.78, 0.08, 0.08, 0.0,  0.00]),
        'q_place': np.array([-0.78, 0.50, 0.35, 0.0, -0.30]),
        'rgb': (0.15, 0.90, 0.45),
    },
    'blue': {
        'q_pick':  np.array([ 0.06, 0.50, 0.35, 0.0, -0.30]),
        'q_Lift1': np.array([ 0.06, 0.08, 0.08, 0.0,  0.00]),
        'q_Lift2': np.array([-1.10, 0.08, 0.08, 0.0,  0.00]),
        'q_place': np.array([-1.10, 0.50, 0.35, 0.0, -0.30]),
        'rgb': (0.20, 0.60, 1.00),
    },
}

for k, v in SORT_CONFIG.items():
    v['pick_pos']  = forward_kinematics(v['q_pick'])[:3, 3].copy()
    v['place_pos'] = forward_kinematics(v['q_place'])[:3, 3].copy()


def solve_ik(q_init, target_pos, target_rot=None, max_iters=25):
    target_pos = np.array(target_pos, dtype=float)
    target_pos[2] = max(0.030, target_pos[2])

    q = np.clip(np.array(q_init, dtype=float), LOWER_LIMITS, UPPER_LIMITS)
    last_safe_q = q.copy()
    eps = 1e-4
    damping = 0.015

    for _ in range(max_iters):
        T_curr = forward_kinematics(q)
        pos_err = target_pos - T_curr[:3, 3]

        if target_rot is not None:
            rot_err_mat = target_rot @ T_curr[:3, :3].T
            rot_err = R.from_matrix(rot_err_mat).as_rotvec() * 0.25
            err = np.concatenate([pos_err, rot_err])
        else:
            err = pos_err

        if np.linalg.norm(pos_err) < 5e-4 and (target_rot is None or np.linalg.norm(rot_err) < 5e-3):
            break

        J = np.zeros((len(err), 5))
        for i in range(5):
            dq = np.zeros(5)
            dq[i] = eps
            T_plus = forward_kinematics(q + dq)
            dp = (T_plus[:3, 3] - T_curr[:3, 3]) / eps
            if target_rot is not None:
                dR = R.from_matrix(T_plus[:3, :3] @ T_curr[:3, :3].T).as_rotvec() / eps * 0.25
                J[:, i] = np.concatenate([dp, dR])
            else:
                J[:, i] = dp

        JJT = J @ J.T + (damping ** 2) * np.eye(len(err))
        delta_q = J.T @ np.linalg.solve(JJT, err)
        cand_q = np.clip(q + delta_q, LOWER_LIMITS, UPPER_LIMITS)

        ok, _ = check_collision(cand_q)
        if ok:
            q = cand_q
            last_safe_q = q.copy()
        else:
            break

    return last_safe_q


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<script src="/static/three.min.js"></script>
<script src="/static/OrbitControls.js"></script>
<script src="/static/STLLoader.js"></script>
<style>
  :root {
    --bg: #0b1120;
    --card: #172033;
    --border: #2e3f5c;
    --accent: #38bdf8;
    --green: #10b981;
    --red: #ef4444;
    --blue: #3b82f6;
    --orange: #f59e0b;
    --purple: #8b5cf6;
    --text: #f8fafc;
    --muted: #94a3b8;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Inter', system-ui, -apple-system, sans-serif; }
  body { background: var(--bg); color: var(--text); padding: 14px; min-height: 100vh; }
  header {
    display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center;
    background: var(--card); border: 1px solid var(--border); border-radius: 12px;
    padding: 12px 18px; margin-bottom: 14px; gap: 12px;
  }
  .title-wrap h1 { font-size: 1.28rem; color: var(--accent); }
  .title-wrap p { font-size: 0.80rem; color: var(--muted); margin-top: 2px; }
  .status-bar { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
  .badge {
    padding: 6px 13px; border-radius: 999px; font-weight: 700; font-size: 0.80rem;
    background: rgba(16, 185, 129, 0.15); color: var(--green); border: 1px solid var(--green);
  }
  .badge.warn { background: rgba(239, 68, 68, 0.18); color: var(--red); border-color: var(--red); }
  .estop-btn {
    background: var(--red); color: #fff; border: none; padding: 9px 18px;
    border-radius: 8px; font-weight: 800; font-size: 0.88rem; cursor: pointer;
    box-shadow: 0 0 14px rgba(239, 68, 68, 0.45);
  }

  .top-split {
    display: grid; grid-template-columns: 1.35fr 1fr; gap: 14px; margin-bottom: 14px;
  }
  @media (max-width: 1024px) { .top-split { grid-template-columns: 1fr; } }

  .viewer-card {
    background: var(--card); border: 1px solid var(--border); border-radius: 12px;
    padding: 12px; display: flex; flex-direction: column; gap: 8px; position: relative;
  }
  #threeContainer {
    width: 100%; height: 440px; background: radial-gradient(circle at center, #1e293b 0%, #090d16 100%);
    border-radius: 8px; border: 1px solid var(--border); overflow: hidden; position: relative;
  }
  .viewer-overlay {
    position: absolute; top: 48px; left: 22px; pointer-events: none;
    background: rgba(11, 17, 32, 0.78); border: 1px solid var(--border);
    padding: 8px 12px; border-radius: 8px; font-size: 0.76rem; color: var(--muted);
  }

  .grid {
    display: grid; grid-template-columns: repeat(auto-fit, minmax(340px, 1fr)); gap: 14px;
  }
  .card {
    background: var(--card); border: 1px solid var(--border); border-radius: 12px;
    padding: 14px; display: flex; flex-direction: column; gap: 10px;
  }
  .card h2 {
    font-size: 0.92rem; text-transform: uppercase; letter-spacing: 0.6px;
    color: var(--accent); border-bottom: 1px solid var(--border); padding-bottom: 7px;
    display: flex; justify-content: space-between; align-items: center;
  }

  .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .btn-grid-3 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
  .btn {
    padding: 10px 10px; border: 1px solid var(--border); border-radius: 8px;
    background: #24324d; color: var(--text); font-weight: 600; font-size: 0.83rem;
    cursor: pointer; transition: all 0.14s; display: flex; align-items: center; justify-content: center; gap: 5px;
  }
  .btn:hover { background: #314468; border-color: var(--accent); }
  .btn.red-btn { background: rgba(239, 68, 68, 0.22); border-color: var(--red); color: #fca5a5; font-weight: 700; }
  .btn.red-btn:hover { background: var(--red); color: #fff; }
  .btn.green-btn { background: rgba(16, 185, 129, 0.22); border-color: var(--green); color: #6ee7b7; font-weight: 700; }
  .btn.green-btn:hover { background: var(--green); color: #052e16; }
  .btn.blue-btn { background: rgba(59, 130, 246, 0.22); border-color: var(--blue); color: #93c5fd; font-weight: 700; }
  .btn.blue-btn:hover { background: var(--blue); color: #fff; }
  .btn.rainbow {
    background: linear-gradient(90deg, #ef4444, #f59e0b, #10b981, #3b82f6);
    color: #fff; border: none; font-weight: 800; text-shadow: 0 1px 3px rgba(0,0,0,0.6);
  }
  .btn.accent { background: var(--accent); color: #082f49; border: none; font-weight: 700; }
  .btn.purple { background: var(--purple); color: #fff; border: none; font-weight: 700; }

  .slider-row { display: flex; flex-direction: column; gap: 3px; }
  .slider-head { display: flex; justify-content: space-between; font-size: 0.79rem; }
  .slider-head span.val { font-family: monospace; color: var(--accent); font-weight: 700; }
  input[type=range] {
    width: 100%; accent-color: var(--accent); cursor: pointer; height: 6px;
    background: #0b1120; border-radius: 6px;
  }

  .telemetry-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
  .stat-box {
    background: #0b1120; border: 1px solid var(--border); border-radius: 8px;
    padding: 7px 8px; text-align: center;
  }
  .stat-box .lbl { font-size: 0.70rem; color: var(--muted); }
  .stat-box .num { font-family: monospace; font-size: 0.98rem; font-weight: 700; color: var(--accent); margin-top: 2px; }

  .wp-list {
    max-height: 95px; overflow-y: auto; background: #0b1120; border-radius: 8px;
    padding: 7px; font-family: monospace; font-size: 0.75rem; color: var(--muted);
    border: 1px solid var(--border);
  }

  .servo-studio-grid {
    display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 10px;
  }
  .servo-unit {
    background: #0b1120; border: 1px solid var(--border); border-radius: 10px;
    padding: 10px; display: flex; flex-direction: column; gap: 6px; position: relative;
  }
  .servo-unit.mg-type { border-left: 4px solid #f59e0b; }
  .servo-unit.sg-type { border-left: 4px solid #3b82f6; }
  .servo-top { display: flex; justify-content: space-between; align-items: center; }
  .servo-title { font-size: 0.80rem; font-weight: 700; color: var(--text); }
  .servo-select {
    background: #1e293b; color: #f8fafc; border: 1px solid #475569; border-radius: 6px;
    font-size: 0.72rem; font-weight: 700; padding: 2px 6px; cursor: pointer;
  }
  .servo-visual-row {
    display: flex; align-items: center; gap: 10px; margin: 2px 0;
  }
  .servo-meta {
    display: flex; flex-direction: column; gap: 2px; font-family: monospace; font-size: 0.74rem; flex: 1;
  }
  .servo-meta .deg-big { font-size: 1.05rem; font-weight: 800; color: var(--accent); }
  .servo-meta .pwm-val { color: #f59e0b; font-weight: 700; }
  .servo-meta .spec-val { color: var(--muted); font-size: 0.68rem; }
  .mini-btns { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 4px; }
  .mini-btn {
    padding: 4px; font-size: 0.72rem; font-weight: 700; border-radius: 5px;
    border: 1px solid var(--border); background: #1e293b; color: var(--text); cursor: pointer;
  }
  .mini-btn:hover { border-color: var(--accent); background: #334155; }
</style>
</head>
<body>

<header>
  <div class="title-wrap">
    <h1>🤖 5-DOF Robotic Arm — Interactive 3D Web Digital Twin & SG90 / MG996R Servo Studio</h1>
    <p>Live 3D STL + SG90/MG996R Servo Viewer (Browser-Only) • Multi-Color Cube Sorting • Real-Time IK & PWM Telemetry</p>
  </div>
  <div class="status-bar">
    <div id="safetyBadge" class="badge">✅ SAFE (Collision-Free)</div>
    <button class="estop-btn" onclick="apiCall('/api/action', {action:'estop'})">🛑 E-STOP</button>
  </div>
</header>

<div class="top-split">
  <!-- LEFT: LIVE 3D BROWSER VIEWER (THREE.JS STL MODEL + 3D SG90 & MG996R SERVOS) -->
  <div class="viewer-card">
    <h2 style="font-size:0.92rem;color:var(--accent);display:flex;justify-content:space-between;align-items:center;">
      <span>🌐 Live 3D Robot + SG90 / MG996R Servo Viewer (Browser Only)</span>
      <div style="display:flex;gap:8px;align-items:center;">
        <button id="toggleServoBtn" class="mini-btn" style="padding:3px 8px;background:#1d4ed8;color:#fff;border-color:#60a5fa;" onclick="toggle3DServos()">
          🔩 3D SG90/MG996R: ON
        </button>
        <span id="meshLoadStatus" style="color:var(--green);font-size:0.78rem;">Loading 3D STL Meshes...</span>
      </div>
    </h2>
    <div id="threeContainer"></div>
    <div class="viewer-overlay">
      <div>🖱️ <b>Left-Click + Drag</b>: Rotate 3D Camera</div>
      <div>🖱️ <b>Right-Click + Drag</b>: Pan • <b>Scroll</b>: Zoom</div>
      <div style="margin-top:3px;font-size:0.74rem;">
        <span style="color:#f59e0b;font-weight:700;">⬛ MG996R (Metal 55g)</span>: J1, J2, J3 &nbsp;|&nbsp;
        <span style="color:#60a5fa;font-weight:700;">🟦 SG90 (Micro 9g)</span>: J4, J5, Gripper
      </div>
      <div id="servoReadout" style="margin-top:4px;color:#38bdf8;font-family:monospace;">
        S1=90° S2=90° S3=90° S4=90° S5=90° G=90°
      </div>
    </div>
  </div>

  <!-- RIGHT: MULTI-COLOR OBJECT SORTING & MISSIONS -->
  <div class="card">
    <h2>
      <span>🎨 Multi-Color Cube Sorting (Red / Green / Blue)</span>
      <span id="activeTaskLbl" style="font-size:0.76rem;color:var(--orange);">Idle (Ready)</span>
    </h2>
    <div class="btn-grid-3">
      <button class="btn red-btn" onclick="apiCall('/api/action', {action:'sort_red'})">🔴 Sort Red Cube</button>
      <button class="btn green-btn" onclick="apiCall('/api/action', {action:'sort_green'})">🟢 Sort Green Cube</button>
      <button class="btn blue-btn" onclick="apiCall('/api/action', {action:'sort_blue'})">🔵 Sort Blue Cube</button>
    </div>
    <div class="btn-grid">
      <button class="btn rainbow" onclick="apiCall('/api/action', {action:'sort_all'})">🌈 Auto-Sort ALL 3 Cubes (R → G → B)</button>
      <button class="btn" onclick="apiCall('/api/action', {action:'reset_cubes'})">🔄 Reset All 3 Cubes to Pickup</button>
    </div>

    <h2 style="margin-top:4px;"><span>🚀 Special Demos & Gripper Control</span></h2>
    <div class="btn-grid-3">
      <button class="btn accent" onclick="apiCall('/api/action', {action:'circle_demo'})">⭕ 3D Circle (IK)</button>
      <button class="btn purple" onclick="apiCall('/api/action', {action:'wave_demo'})">👋 Wave Hello</button>
      <button class="btn" onclick="apiCall('/api/action', {action:'scan_demo'})">🔍 360° Scan</button>
      <button class="btn" onclick="apiCall('/api/action', {action:'open_gripper'})">✋ Open Gripper</button>
      <button class="btn" onclick="apiCall('/api/action', {action:'close_gripper'})">✊ Close Gripper</button>
      <button class="btn" onclick="apiCall('/api/action', {action:'home'})">🏠 Home Pose</button>
    </div>

    <div class="slider-row" style="margin-top:4px;">
      <div class="slider-head">
        <span>⚡ Motion Speed Multiplier</span>
        <span class="val" id="speedVal">1.0x</span>
      </div>
      <input type="range" id="speedSlider" min="0.3" max="2.5" step="0.1" value="1.0"
        oninput="document.getElementById('speedVal').innerText=this.value+'x'; apiCall('/api/speed', {speed:parseFloat(this.value)})">
    </div>

    <h2 style="margin-top:4px;">
      <span>📍 Teach & Replay Studio</span>
      <span id="wpCount" style="font-size:0.75rem;color:var(--muted);">0 Waypoints</span>
    </h2>
    <div class="btn-grid" style="grid-template-columns: repeat(4, 1fr);">
      <button class="btn accent" onclick="apiCall('/api/waypoint', {op:'save'})">+ Save</button>
      <button class="btn green-btn" onclick="apiCall('/api/waypoint', {op:'play', loop:false})">▶ Play</button>
      <button class="btn purple" onclick="apiCall('/api/waypoint', {op:'play', loop:true})">🔁 Loop</button>
      <button class="btn" onclick="apiCall('/api/waypoint', {op:'clear'})">🗑 Clear</button>
    </div>
    <div class="wp-list" id="wpList">No waypoints saved yet.</div>
  </div>
</div>

<!-- 📷 VISION & CAMERA TRACKING STUDIO (DJI OSMO ACTION 5 / WEBCAM) -->
<div class="card" style="margin-bottom:14px; border:1px solid #38bdf8;">
  <h2>
    <span>📷 Vision & Camera Tracking Studio (DJI Osmo Action 5 / USB Webcam)</span>
    <span id="visionStatus" class="badge warn">Camera Off</span>
  </h2>

  <div style="display:flex; flex-wrap:wrap; gap:10px; align-items:center; background:#0b1120; padding:10px 12px; border-radius:8px; border:1px solid var(--border);">
    <label style="font-size:0.80rem; font-weight:700; color:var(--muted);">Camera Device:</label>
    <select id="cameraSelect" class="servo-select" style="padding:6px 10px; font-size:0.80rem; flex:1; min-width:200px;"></select>
    <button class="btn green-btn" style="padding:6px 14px;" onclick="startCamera()">▶ Start Camera</button>
    <button class="btn red-btn" style="padding:6px 14px;" onclick="stopCamera()">⏹ Stop Camera</button>
    <label style="font-size:0.78rem; display:flex; align-items:center; gap:6px; cursor:pointer;">
      <input type="checkbox" id="mirrorVideo" checked style="accent-color:var(--accent);"> Mirror Feed
    </label>
  </div>

  <div style="display:grid; grid-template-columns: 1.15fr 1fr; gap:14px; margin-top:8px;">
    <!-- Left: Video HUD & Canvas -->
    <div style="display:flex; flex-direction:column; gap:8px;">
      <div style="position:relative; width:100%; aspect-ratio:4/3; background:#000; border-radius:8px; overflow:hidden; border:1px solid var(--border);">
        <video id="visionVideo" playsinline autoplay muted style="display:none;"></video>
        <canvas id="visionCanvas" width="320" height="240" style="width:100%; height:100%; object-fit:contain; cursor:crosshair; display:block;" onclick="onCanvasClick(event)"></canvas>
        <div id="visionHudOverlay" style="position:absolute; bottom:8px; left:8px; right:8px; background:rgba(11,17,32,0.85); border:1px solid var(--border); border-radius:6px; padding:6px 10px; font-family:monospace; font-size:0.75rem; color:var(--text); display:flex; justify-content:space-between; align-items:center; pointer-events:none;">
          <span id="hudCoords">Target: No Cube Detected</span>
          <span id="hudFps" style="color:var(--accent);">0 FPS</span>
        </div>
      </div>
      <div style="font-size:0.74rem; color:var(--muted); line-height:1.4;">
        💡 <strong>Color Pipette:</strong> Agar color match nahi ho raha, to video screen par cube par click karein — camera instant real-world color aur lighting lock kar lega!
      </div>
    </div>

    <!-- Right: Color Tracking, Servoing & Grasp Actions -->
    <div style="display:flex; flex-direction:column; gap:10px;">
      <div>
        <label style="font-size:0.78rem; font-weight:700; color:var(--muted); display:block; margin-bottom:5px;">Target Color Preset:</label>
        <div class="btn-grid-3" style="grid-template-columns:repeat(4, 1fr); gap:6px;">
          <button class="btn red-btn" style="padding:7px 4px; font-size:0.78rem;" onclick="setTrackColor('red')">🔴 Red</button>
          <button class="btn green-btn" style="padding:7px 4px; font-size:0.78rem;" onclick="setTrackColor('green')">🟢 Green</button>
          <button class="btn blue-btn" style="padding:7px 4px; font-size:0.78rem;" onclick="setTrackColor('blue')">🔵 Blue</button>
          <button class="btn" style="padding:7px 4px; font-size:0.78rem; color:#fde047; border-color:#eab308; background:rgba(234,179,8,0.18);" onclick="setTrackColor('yellow')">🟡 Yellow</button>
        </div>
      </div>

      <div style="display:flex; align-items:center; justify-content:space-between; background:#0b1120; border:1px solid var(--border); border-radius:8px; padding:7px 10px;">
        <span style="font-size:0.78rem; color:var(--muted);">Selected Target Color:</span>
        <div style="display:flex; align-items:center; gap:8px;">
          <div id="targetColorPreview" style="width:20px; height:20px; border-radius:4px; background:#ef4444; border:1px solid #fff;"></div>
          <span id="targetColorLbl" style="font-size:0.80rem; font-weight:700; font-family:monospace; color:var(--accent);">RED</span>
        </div>
      </div>

      <div style="background:#0b1120; border:1px solid var(--border); border-radius:8px; padding:10px 12px; display:flex; align-items:center; justify-content:space-between;">
        <div>
          <div style="font-size:0.82rem; font-weight:700; color:var(--text);">🦾 Live Hand / Cube Following</div>
          <div style="font-size:0.72rem; color:var(--muted);">Robot tracks cube in 3D as you move it in air</div>
        </div>
        <input type="checkbox" id="followCubeToggle" style="width:22px; height:22px; accent-color:var(--accent); cursor:pointer;" onchange="onFollowToggle(this.checked)">
      </div>

      <div style="display:grid; grid-template-columns: 1.35fr 1fr; gap:8px;">
        <button class="btn" style="background:linear-gradient(90deg, #f59e0b, #ef4444); color:#fff; border:none; font-weight:800; padding:12px; font-size:0.85rem;" onclick="triggerVisionGrasp()">
          🎯 Grab & Hold Cube
        </button>
        <button class="btn" style="padding:12px; font-size:0.85rem;" onclick="triggerVisionRelease()">
          ✋ Release Cube
        </button>
      </div>

      <div style="display:grid; grid-template-columns:1fr 1fr; gap:8px;">
        <div class="stat-box" style="padding:5px 8px;"><div class="lbl">Detected Contour</div><div class="num" id="visBoxSize" style="font-size:0.85rem;">0 px</div></div>
        <div class="stat-box" style="padding:5px 8px;"><div class="lbl">Depth Est. (X)</div><div class="num" id="visEstDepth" style="font-size:0.85rem;">--</div></div>
      </div>
    </div>
  </div>
</div>

<!-- BROWSER-ONLY SG90 & MG996R SERVO HARDWARE STUDIO -->
<div class="card" style="margin-bottom:14px;">
  <h2>
    <span>🔩 SG90 & MG996R Servo Motor Hardware Studio (Browser-Only 3D + Live 50Hz PWM Control)</span>
    <span style="font-size:0.75rem;color:var(--muted);">Switch any joint between MG996R (Metal 55g) & SG90 (Blue Micro 9g)</span>
  </h2>
  <div class="servo-studio-grid" id="servoStudioGrid"></div>
</div>

<div class="grid">
  <!-- JOINT SLIDERS -->
  <div class="card">
    <h2><span>🎛️ 6-DOF Joint & Parallel Gripper Sliders</span></h2>
    <div class="slider-row">
      <div class="slider-head"><span>J1: Base Waist (-90°..+90°)</span><span class="val" id="j0_lbl">0.00 rad (90°)</span></div>
      <input type="range" id="j0" min="-1.57" max="1.57" step="0.01" value="0" oninput="sendJoints()">
    </div>
    <div class="slider-row">
      <div class="slider-head"><span>J2: Shoulder Pitch (-30°..+75°)</span><span class="val" id="j1_lbl">0.00 rad (90°)</span></div>
      <input type="range" id="j1" min="-0.52" max="1.31" step="0.01" value="0" oninput="sendJoints()">
    </div>
    <div class="slider-row">
      <div class="slider-head"><span>J3: Elbow Pitch (-65°..+70°)</span><span class="val" id="j2_lbl">0.00 rad (90°)</span></div>
      <input type="range" id="j2" min="-1.13" upper="1.22" max="1.22" step="0.01" value="0" oninput="sendJoints()">
    </div>
    <div class="slider-row">
      <div class="slider-head"><span>J4: Wrist Roll (-90°..+90°)</span><span class="val" id="j3_lbl">0.00 rad (90°)</span></div>
      <input type="range" id="j3" min="-1.57" max="1.57" step="0.01" value="0" oninput="sendJoints()">
    </div>
    <div class="slider-row">
      <div class="slider-head"><span>J5: Wrist Pitch (-60°..+60°)</span><span class="val" id="j4_lbl">0.00 rad (90°)</span></div>
      <input type="range" id="j4" min="-1.05" max="1.05" step="0.01" value="0" oninput="sendJoints()">
    </div>
    <div class="slider-row">
      <div class="slider-head"><span>G: 4-Bar Parallel Gripper (0°..+37° | 90°..127° Servo)</span><span class="val" id="jg_lbl">0.00 rad (90°)</span></div>
      <input type="range" id="jg" min="0.00" max="0.65" step="0.01" value="0" oninput="sendJoints()">
    </div>
  </div>

  <!-- CARTESIAN IK -->
  <div class="card">
    <h2><span>🎯 3D Cartesian Inverse Kinematics (Gripper TCP)</span></h2>
    <div class="telemetry-grid">
      <div class="stat-box"><div class="lbl">TCP X (mm)</div><div class="num" id="tcp_x">-42.2</div></div>
      <div class="stat-box"><div class="lbl">TCP Y (mm)</div><div class="num" id="tcp_y">-237.6</div></div>
      <div class="stat-box"><div class="lbl">TCP Z (mm)</div><div class="num" id="tcp_z">245.9</div></div>
    </div>

    <div class="slider-row">
      <div class="slider-head"><span>Target X (m)</span><span class="val" id="ik_x_lbl">-0.042 m</span></div>
      <input type="range" id="ik_x" min="-0.28" max="0.12" step="0.005" value="-0.042" oninput="sendIK()">
    </div>
    <div class="slider-row">
      <div class="slider-head"><span>Target Y (m)</span><span class="val" id="ik_y_lbl">-0.238 m</span></div>
      <input type="range" id="ik_y" min="-0.32" max="0.20" step="0.005" value="-0.238" oninput="sendIK()">
    </div>
    <div class="slider-row">
      <div class="slider-head"><span>Target Z (m)</span><span class="val" id="ik_z_lbl">0.246 m</span></div>
      <input type="range" id="ik_z" min="0.030" max="0.35" step="0.005" value="0.246" oninput="sendIK()">
    </div>

    <div class="btn-grid-3" style="margin-top:4px;">
      <button class="btn" onclick="jog(-0.015, 0, 0)">◀ -X (15mm)</button>
      <button class="btn" onclick="jog(0, 0.015, 0)">▲ +Y (15mm)</button>
      <button class="btn" onclick="jog(0.015, 0, 0)">▶ +X (15mm)</button>
      <button class="btn" onclick="jog(0, 0, -0.015)">⏬ -Z (Down)</button>
      <button class="btn" onclick="jog(0, -0.015, 0)">▼ -Y (15mm)</button>
      <button class="btn" onclick="jog(0, 0, 0.015)">⏫ +Z (Up)</button>
    </div>
  </div>
</div>

<script>
let userDragging = false;
let dragTimeout = null;

// Browser-only SG90 & MG996R Servo Configuration per channel (S1..S6)
const SERVO_CHANNELS = [
  { id: 0, code: 'S1', name: 'J1: Base Waist',     type: 'MG996R', minDeg: 0,  maxDeg: 180, isGripper: false },
  { id: 1, code: 'S2', name: 'J2: Shoulder Pitch', type: 'MG996R', minDeg: 60, maxDeg: 165, isGripper: false },
  { id: 2, code: 'S3', name: 'J3: Elbow Pitch',    type: 'MG996R', minDeg: 25, maxDeg: 160, isGripper: false },
  { id: 3, code: 'S4', name: 'J4: Wrist Roll',     type: 'SG90',   minDeg: 0,  maxDeg: 180, isGripper: false },
  { id: 4, code: 'S5', name: 'J5: Wrist Pitch',    type: 'SG90',   minDeg: 30, maxDeg: 150, isGripper: false },
  { id: 5, code: 'S6', name: 'G: Parallel Gripper',type: 'SG90',   minDeg: 61, maxDeg: 127, isGripper: true  },
  { id: 5, code: 'S6', name: 'G: Parallel Gripper',type: 'SG90',   minDeg: 90, maxDeg: 127, isGripper: true  },
];

const SERVO_SPECS = {
  MG996R: {
    label: 'MG996R Metal (11 kg·cm)',
    pwmMin: 500, pwmMax: 2500,
    torque: '11.0 kg·cm @ 6V • Metal Gear 55g',
    scale: 1.0,
    bodyColor: '#1e293b', accentColor: '#f59e0b', hornColor: '#fbbf24'
  },
  SG90: {
    label: 'SG90 Micro (1.8 kg·cm)',
    pwmMin: 500, pwmMax: 2400,
    torque: '1.8 kg·cm @ 4.8V • Nylon Gear 9g',
    scale: 0.58,
    bodyColor: '#1d4ed8', accentColor: '#60a5fa', hornColor: '#f8fafc'
  }
};

let show3DServos = true;
let servo3DMeshes = [];
let matMG996R = null;
let matSG90   = null;

function buildServoStudioUI() {
  const grid = document.getElementById('servoStudioGrid');
  grid.innerHTML = SERVO_CHANNELS.map((ch, idx) => {
    const spec = SERVO_SPECS[ch.type];
    const cls  = ch.type === 'MG996R' ? 'mg-type' : 'sg-type';
    return `
      <div class="servo-unit ${cls}" id="servo_card_${idx}">
        <div class="servo-top">
          <span class="servo-title">${ch.code} • ${ch.name}</span>
          <select class="servo-select" id="servo_type_${idx}" onchange="changeServoType(${idx}, this.value)">
            <option value="MG996R" ${ch.type==='MG996R'?'selected':''}>⬛ MG996R</option>
            <option value="SG90" ${ch.type==='SG90'?'selected':''}>🟦 SG90</option>
          </select>
        </div>
        <div class="servo-visual-row">
          <svg width="68" height="68" viewBox="0 0 68 68">
            <!-- Mounting Ears -->
            <rect x="4" y="24" width="60" height="20" rx="3" fill="#334155" stroke="#475569" stroke-width="1"/>
            <!-- Servo Body -->
            <rect id="svg_body_${idx}" x="11" y="14" width="46" height="40" rx="6" fill="${spec.bodyColor}" stroke="${spec.accentColor}" stroke-width="2"/>
            <!-- Gear Boss -->
            <circle cx="34" cy="34" r="12" fill="#0f172a" stroke="#475569" stroke-width="1.5"/>
            <!-- Rotating Servo Horn -->
            <g id="svg_horn_${idx}" transform="translate(34,34) rotate(0)">
              <rect x="-3.5" y="-24" width="7" height="28" rx="3.5" fill="${spec.hornColor}" stroke="#0f172a" stroke-width="1"/>
              <circle cx="0" cy="-19" r="1.5" fill="#0f172a"/>
              <circle cx="0" cy="-13" r="1.5" fill="#0f172a"/>
              <circle cx="0" cy="0" r="3.5" fill="#ef4444"/>
            </g>
          </svg>
          <div class="servo-meta">
            <div class="deg-big" id="s_deg_lbl_${idx}">90°</div>
            <div class="pwm-val" id="s_pwm_lbl_${idx}">1500 µs (7.5%)</div>
            <div class="spec-val" id="s_spec_lbl_${idx}">${spec.torque}</div>
            <div class="spec-val">Safe Limit: ${ch.minDeg}° .. ${ch.maxDeg}°</div>
          </div>
        </div>
        <input type="range" id="s_slider_${idx}" min="${ch.minDeg}" max="${ch.maxDeg}" step="1" value="90"
          oninput="setServoDeg(${idx}, parseFloat(this.value))">
        <div class="mini-btns">
          <button class="mini-btn" onclick="stepServoDeg(${idx}, -5)">-5°</button>
          <button class="mini-btn" onclick="setServoDeg(${idx}, 90)">90° Center</button>
          <button class="mini-btn" onclick="stepServoDeg(${idx}, 5)">+5°</button>
        </div>
      </div>
    `;
  }).join('');
}

function changeServoType(idx, newType) {
  SERVO_CHANNELS[idx].type = newType;
  const spec = SERVO_SPECS[newType];
  const card = document.getElementById('servo_card_' + idx);
  card.className = 'servo-unit ' + (newType === 'MG996R' ? 'mg-type' : 'sg-type');

  const body = document.getElementById('svg_body_' + idx);
  body.setAttribute('fill', spec.bodyColor);
  body.setAttribute('stroke', spec.accentColor);

  const horn = document.querySelector(`#svg_horn_${idx} rect`);
  if (horn) horn.setAttribute('fill', spec.hornColor);

  document.getElementById('s_spec_lbl_' + idx).innerText = spec.torque;

  // Update 3D servo mesh material & scale in Three.js viewer immediately!
  if (servo3DMeshes[idx]) {
    const s = 0.001 * spec.scale;
    servo3DMeshes[idx].scale.set(s, s, s);
    servo3DMeshes[idx].material = (newType === 'MG996R') ? matMG996R : matSG90;
  }
}

function toggle3DServos() {
  show3DServos = !show3DServos;
  servo3DMeshes.forEach(m => { if (m) m.visible = show3DServos; });
  const btn = document.getElementById('toggleServoBtn');
  btn.innerText = show3DServos ? '🔩 3D SG90/MG996R: ON' : '🔩 3D SG90/MG996R: OFF';
  btn.style.background = show3DServos ? '#1d4ed8' : '#334155';
}

function setServoDeg(idx, deg) {
  markDragging();
  const ch = SERVO_CHANNELS[idx];
  const clamped = Math.max(ch.minDeg, Math.min(ch.maxDeg, deg));
  document.getElementById('s_slider_' + idx).value = clamped;
  const rad = (clamped - 90.0) * Math.PI / 180.0;
  if (ch.isGripper) {
    document.getElementById('jg').value = rad;
  } else {
    document.getElementById('j' + idx).value = rad;
  }
  sendJoints();
}

function stepServoDeg(idx, delta) {
  const cur = parseFloat(document.getElementById('s_slider_' + idx).value || '90');
  setServoDeg(idx, cur + delta);
}

function markDragging() {
  userDragging = true;
  if (dragTimeout) clearTimeout(dragTimeout);
  dragTimeout = setTimeout(() => { userDragging = false; }, 400);
}

async function apiCall(url, payload) {
  try {
    await fetch(url, {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(payload)
    });
  } catch (e) {}
}

function sendJoints() {
  markDragging();
  const q = [
    parseFloat(document.getElementById('j0').value),
    parseFloat(document.getElementById('j1').value),
    parseFloat(document.getElementById('j2').value),
    parseFloat(document.getElementById('j3').value),
    parseFloat(document.getElementById('j4').value)
  ];
  const g = parseFloat(document.getElementById('jg').value);
  apiCall('/api/joints', {q, g});
}

function sendIK() {
  markDragging();
  const x = parseFloat(document.getElementById('ik_x').value);
  const y = parseFloat(document.getElementById('ik_y').value);
  const z = parseFloat(document.getElementById('ik_z').value);
  document.getElementById('ik_x_lbl').innerText = x.toFixed(3) + ' m';
  document.getElementById('ik_y_lbl').innerText = y.toFixed(3) + ' m';
  document.getElementById('ik_z_lbl').innerText = z.toFixed(3) + ' m';
  apiCall('/api/ik', {x, y, z});
}

function jog(dx, dy, dz) {
  apiCall('/api/jog', {dx, dy, dz});
}

// ================= THREE.JS 3D ROBOT & WORKCELL VIEWER =================
let scene, camera, renderer, controls;
let jointNodes = {};
let cubeMeshes = {};
let binMeshes = {};

function setURDFTransform(obj, xyz, rpyArr) {
  obj.position.set(xyz[0], xyz[1], xyz[2]);
  // URDF rpy is intrinsic ZYX (extrinsic XYZ) -> Three.js 'ZYX' order with (r, p, y)
  obj.rotation.order = 'ZYX';
  obj.rotation.set(rpyArr[0], rpyArr[1], rpyArr[2]);
}

function initThreeJS() {
  const container = document.getElementById('threeContainer');
  if (!container) return;
  if (typeof THREE === 'undefined') {
    const s = document.getElementById('meshLoadStatus');
    if (s) s.innerText = 'Three.js library offline';
    return;
  }
  if (typeof THREE.OrbitControls === 'undefined' || typeof THREE.STLLoader === 'undefined') {
    const s = document.getElementById('meshLoadStatus');
    if (s) s.innerText = 'Three.js loaders offline';
    return;
  }

  const w = container.clientWidth || 600;
  const h = container.clientHeight || 440;

  scene = new THREE.Scene();
  THREE.Object3D.DefaultUp.set(0, 0, 1);

  camera = new THREE.PerspectiveCamera(45, w / h, 0.01, 10.0);
  camera.position.set(-0.42, -0.45, 0.34);

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(w, h);
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  container.innerHTML = '';
  container.appendChild(renderer.domElement);

  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(-0.08, -0.06, 0.12);
  controls.update();

  // Floor Grid in XY plane (Z=0)
  const grid = new THREE.GridHelper(0.8, 16, 0x38bdf8, 0x1e293b);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);

  // Lights
  scene.add(new THREE.AmbientLight(0xffffff, 0.65));
  const dir1 = new THREE.DirectionalLight(0xffffff, 0.85);
  dir1.position.set(1, -1, 2);
  scene.add(dir1);
  const dir2 = new THREE.DirectionalLight(0x93c5fd, 0.45);
  dir2.position.set(-1, 1, 1);
  scene.add(dir2);

  // Build exact URDF kinematic hierarchy in Three.js
  const worldNode = new THREE.Group();
  scene.add(worldNode);

  const j1 = new THREE.Group(); setURDFTransform(j1, [-0.10, 0, 0.07], [0, 0, 0]); worldNode.add(j1);
  const j1_rot = new THREE.Group(); j1.add(j1_rot);

  const j2 = new THREE.Group(); setURDFTransform(j2, [0.00906, -0.01413, 0.03490], [1.5708, 0, -1.5708]); j1_rot.add(j2);
  const j2_rot = new THREE.Group(); j2.add(j2_rot);

  const j3 = new THREE.Group(); setURDFTransform(j3, [0.1115, 0, 0], [3.1416, 0, 1.5708]); j2_rot.add(j3);
  const j3_rot = new THREE.Group(); j3.add(j3_rot);

  const j4 = new THREE.Group(); setURDFTransform(j4, [0.1035, -0.008, 0.01825], [3.14159, 0, 0]); j3_rot.add(j4);
  const j4_rot = new THREE.Group(); j4.add(j4_rot);

  const j5 = new THREE.Group(); setURDFTransform(j5, [0.021, 0, -0.0165], [3.14159, -1.5708, 0]); j4_rot.add(j5);
  const j5_rot = new THREE.Group(); j5.add(j5_rot);

  // 4-Bar Parallel Gripper sub-tree
  const gL_gear = new THREE.Group(); setURDFTransform(gL_gear, [-0.0275, 0.057565, 0.015], [0,0,0]); j5_rot.add(gL_gear);
  const gL_gear_rot = new THREE.Group(); gL_gear.add(gL_gear_rot);

  const gL_fin = new THREE.Group(); setURDFTransform(gL_fin, [-0.000004, 0.031, 0], [0,0,0]); gL_gear_rot.add(gL_fin);
  const gL_fin_rot = new THREE.Group(); gL_fin.add(gL_fin_rot);

  const gL_in = new THREE.Group(); setURDFTransform(gL_in, [-0.019815, 0.078, 0.015], [0,0,0]); j5_rot.add(gL_in);
  const gL_in_rot = new THREE.Group(); gL_in.add(gL_in_rot);

  const gR_gear = new THREE.Group(); setURDFTransform(gR_gear, [-0.0005, 0.057565, 0.015], [0,0,0]); j5_rot.add(gR_gear);
  const gR_gear_rot = new THREE.Group(); gR_gear.add(gR_gear_rot);

  const gR_fin = new THREE.Group(); setURDFTransform(gR_fin, [0.000777, 0.03099, 0], [0,0,0]); gR_gear_rot.add(gR_fin);
  const gR_fin_rot = new THREE.Group(); gR_fin.add(gR_fin_rot);

  const gR_in = new THREE.Group(); setURDFTransform(gR_in, [-0.008185, 0.078, 0.015], [0,0,0]); j5_rot.add(gR_in);
  const gR_in_rot = new THREE.Group(); gR_in.add(gR_in_rot);

  jointNodes = {
    j1_rot, j2_rot, j3_rot, j4_rot, j5_rot,
    gL_gear_rot, gL_fin_rot, gL_in_rot,
    gR_gear_rot, gR_fin_rot, gR_in_rot
  };

  const matDark   = new THREE.MeshStandardMaterial({ color: 0x383c42, roughness: 0.45, metalness: 0.2 });
  const matOrange = new THREE.MeshStandardMaterial({ color: 0xff7300, roughness: 0.35, metalness: 0.1 });
  const matWhite  = new THREE.MeshStandardMaterial({ color: 0xeaeaea, roughness: 0.40, metalness: 0.1 });

  // Materials for Browser-Only 3D Servos (MG996R Black Metal & SG90 Translucent Royal Blue)
  matMG996R = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.30, metalness: 0.65 });
  matSG90   = new THREE.MeshStandardMaterial({ color: 0x2563eb, roughness: 0.25, metalness: 0.15, transparent: true, opacity: 0.92 });

  const loader = new THREE.STLLoader();
  const parts = [
    ['Base.stl', worldNode, [0, 0, 0], matDark],
    ['Shoulder.stl', j1_rot, [0, 0, 0], matOrange],
    ['Elbow.stl', j2_rot, [0, 0, 0], matDark],
    ['arm.stl', j3_rot, [0, 0, 0], matWhite],
    ['Wrist_.stl', j4_rot, [0, 0.015, -0.0125], matDark],
    ['gripper_base.stl', j5_rot, [-0.013936, 0.020, 0.015], matDark],
    ['gripper_left_gear.stl', gL_gear_rot, [0.013564, -0.037565, 0], matOrange],
    ['gripper_left_finger.stl', gL_fin_rot, [0.013568, -0.068565, 0], matOrange],
    ['gripper_left_inner.stl', gL_in_rot, [0.005879, -0.058, 0], matWhite],
    ['gripper_right_gear.stl', gR_gear_rot, [-0.013436, -0.037565, 0], matOrange],
    ['gripper_right_finger.stl', gR_fin_rot, [-0.014213, -0.068555, 0], matOrange],
    ['gripper_right_inner.stl', gR_in_rot, [-0.005751, -0.058, 0], matWhite],
  ];

  let loadedCount = 0;
  parts.forEach(([file, parent, offset, mat]) => {
    loader.load(
      '/meshes/' + file,
      (geom) => {
        geom.computeVertexNormals();
        const mesh = new THREE.Mesh(geom, mat);
        mesh.scale.set(0.001, 0.001, 0.001);
        mesh.position.set(offset[0], offset[1], offset[2]);
        parent.add(mesh);
        loadedCount++;
        if (loadedCount === parts.length) {
          const s = document.getElementById('meshLoadStatus');
          if (s) s.innerText = '✅ 12 Links + 6 Servos Live';
        }
      },
      undefined,
      (err) => {
        console.error('Failed to load mesh ' + file, err);
        const s = document.getElementById('meshLoadStatus');
        if (s) s.innerText = '⚠️ Error loading ' + file;
      }
    );
  });

  // Load servo.stl exclusively in Browser 3D Viewer for all 6 joints (MG996R & SG90)
  loader.load('/meshes/servo.stl', (rawGeom) => {
    rawGeom.computeVertexNormals();
    // Center servo.stl in its output shaft frame (origin = shaft tip, +Z = rotation axis)
    // using the exact CAD alignment matrix derived from Shoulder.stl cutout:
    const shaftAlign = new THREE.Matrix4();
    shaftAlign.set(
      -0.880873,  0.473352,  0.000001,  113.44,
      -0.000002, -0.000001, -1.000000,   -0.72,
      -0.473352, -0.880873,  0.000001,   80.24,
       0.000000,  0.000000,  0.000000,    1.00
    );
    rawGeom.applyMatrix4(shaftAlign);

    // Mount points for S1..S6 on the kinematic tree (parent node, local xyz, local rpy)
    const servoMounts = [
      [j1,      [0, 0, -0.004], [0, 0, 0]],                // S1: Base Waist (MG996R inside Base)
      [j2,      [0, 0, 0],      [0, 0, 0]],                // S2: Shoulder Pitch (Exact CAD cutout)
      [j3,      [0, 0, 0],      [0, 0, 3.14159]],          // S3: Elbow Pitch (MG996R at Elbow)
      [j4,      [-0.006, 0, 0], [0, 1.5708, 0]],           // S4: Wrist Roll (SG90 inside Forearm)
      [j5,      [0, 0, 0],      [0, 1.5708, 0]],           // S5: Wrist Pitch (SG90 at Wrist)
      [gL_gear, [0, 0, -0.005], [0, 0, -1.5708]],          // S6: Parallel Gripper (SG90 under Left Gear)
    ];

    servo3DMeshes = servoMounts.map(([parentNode, xyz, rpyArr], idx) => {
      const ch   = SERVO_CHANNELS[idx];
      const spec = SERVO_SPECS[ch.type];
      const holder = new THREE.Group();
      setURDFTransform(holder, xyz, rpyArr);
      parentNode.add(holder);

      const sMesh = new THREE.Mesh(rawGeom, ch.type === 'MG996R' ? matMG996R : matSG90);
      const sc = 0.001 * spec.scale;
      sMesh.scale.set(sc, sc, sc);
      sMesh.visible = show3DServos;
      holder.add(sMesh);
      return sMesh;
    });
  });

  // Create Red, Green, Blue Cubes & Bins in 3D Scene
  const colorHex = { red: 0xef4444, green: 0x10b981, blue: 0x3b82f6 };
  ['red', 'green', 'blue'].forEach((col) => {
    const cGeom = new THREE.BoxGeometry(0.020, 0.020, 0.020);
    const cMat  = new THREE.MeshStandardMaterial({ color: colorHex[col], roughness: 0.3 });
    const cMesh = new THREE.Mesh(cGeom, cMat);
    scene.add(cMesh);
    cubeMeshes[col] = cMesh;

    const bGeom = new THREE.CylinderGeometry(0.026, 0.026, 0.004, 24);
    bGeom.rotateX(Math.PI / 2);
    const bMat  = new THREE.MeshStandardMaterial({ color: colorHex[col], transparent: true, opacity: 0.55 });
    const bMesh = new THREE.Mesh(bGeom, bMat);
    scene.add(bMesh);
    binMeshes[col] = bMesh;

    const pMesh = new THREE.Mesh(bGeom, new THREE.MeshStandardMaterial({ color: 0x64748b, transparent: true, opacity: 0.4 }));
    scene.add(pMesh);
    binMeshes[col + '_pick'] = pMesh;
  });

  // 3D Holographic Vision Target Marker
  const visionGeom = new THREE.BoxGeometry(0.026, 0.026, 0.026);
  const visionEdges = new THREE.EdgesGeometry(visionGeom);
  const visionLine = new THREE.LineSegments(visionEdges, new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 2 }));
  const visionMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.6, emissive: 0x0284c7, emissiveIntensity: 0.6 });
  visionTargetMesh = new THREE.Mesh(visionGeom, visionMat);
  visionTargetMesh.add(visionLine);
  visionTargetMesh.visible = false;
  scene.add(visionTargetMesh);

  window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  });

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();
}

function updateThreeScene(st) {
  if (!jointNodes.j1_rot) return;
  jointNodes.j1_rot.rotation.z = st.q[0];
  jointNodes.j2_rot.rotation.z = st.q[1];
  jointNodes.j3_rot.rotation.z = st.q[2];
  jointNodes.j4_rot.rotation.x = st.q[3];
  jointNodes.j5_rot.rotation.x = st.q[4];

  const g = st.g;
  jointNodes.gL_gear_rot.rotation.z =  g;
  jointNodes.gL_fin_rot.rotation.z  = -g;
  jointNodes.gL_in_rot.rotation.z   =  g;
  jointNodes.gR_gear_rot.rotation.z = -g;
  jointNodes.gR_fin_rot.rotation.z  =  g;
  jointNodes.gR_in_rot.rotation.z   = -g;

  if (st.cubes) {
    ['red', 'green', 'blue'].forEach((col) => {
      const info = st.cubes[col];
      if (info && cubeMeshes[col]) {
        cubeMeshes[col].position.set(info.pos[0], info.pos[1], info.pos[2]);
        binMeshes[col].position.set(info.bin[0], info.bin[1], info.bin[2] - 0.013);
        binMeshes[col + '_pick'].position.set(info.pick[0], info.pick[1], info.pick[2] - 0.013);
      }
    });
  }
}

async function pollState() {
  try {
    const res = await fetch('/api/state');
    const st = await res.json();

    const badge = document.getElementById('safetyBadge');
    if (st.is_safe) {
      badge.className = 'badge';
      badge.innerText = '✅ ' + st.safety_status;
    } else {
      badge.className = 'badge warn';
      badge.innerText = '⚠️ ' + st.safety_status;
    }

    document.getElementById('activeTaskLbl').innerText = st.active_task;
    document.getElementById('tcp_x').innerText = (st.tcp[0] * 1000).toFixed(1);
    document.getElementById('tcp_y').innerText = (st.tcp[1] * 1000).toFixed(1);
    document.getElementById('tcp_z').innerText = (st.tcp[2] * 1000).toFixed(1);

    for (let i = 0; i < 5; i++) {
      const deg = Math.round(st.q[i] * 180 / Math.PI + 90);
      document.getElementById('j' + i + '_lbl').innerText = st.q[i].toFixed(2) + ' rad (' + deg + '°)';
      if (!userDragging) document.getElementById('j' + i).value = st.q[i];
    }
    const gdeg = Math.round(st.g * 180 / Math.PI + 90);
    document.getElementById('jg_lbl').innerText = st.g.toFixed(2) + ' rad (' + gdeg + '°)';
    if (!userDragging) {
      document.getElementById('jg').value = st.g;
      document.getElementById('ik_x').value = st.tcp[0].toFixed(3);
      document.getElementById('ik_y').value = st.tcp[1].toFixed(3);
      document.getElementById('ik_z').value = st.tcp[2].toFixed(3);
      document.getElementById('ik_x_lbl').innerText = st.tcp[0].toFixed(3) + ' m';
      document.getElementById('ik_y_lbl').innerText = st.tcp[1].toFixed(3) + ' m';
      document.getElementById('ik_z_lbl').innerText = st.tcp[2].toFixed(3) + ' m';
    }

    // Update SG90 & MG996R Servo Studio Gauges & PWM Microseconds
    if (st.servos) {
      st.servos.forEach((deg, idx) => {
        const ch = SERVO_CHANNELS[idx];
        const spec = SERVO_SPECS[ch.type];
        const pwm = Math.round(spec.pwmMin + (deg / 180.0) * (spec.pwmMax - spec.pwmMin));
        const duty = ((pwm / 20000.0) * 100.0).toFixed(1);
        const degEl = document.getElementById('s_deg_lbl_' + idx);
        const pwmEl = document.getElementById('s_pwm_lbl_' + idx);
        const hornEl = document.getElementById('svg_horn_' + idx);
        if (degEl) degEl.innerText = `${deg}° (${ch.type})`;
        if (pwmEl) pwmEl.innerText = `${pwm} µs (${duty}% @ 50Hz)`;
        if (hornEl) hornEl.setAttribute('transform', `translate(34,34) rotate(${deg - 90})`);
        if (!userDragging) {
          const sl = document.getElementById('s_slider_' + idx);
          if (sl) sl.value = deg;
        }
      });
    }

    document.getElementById('servoReadout').innerText =
      `Servos: S1=${st.servos[0]}° S2=${st.servos[1]}° S3=${st.servos[2]}° S4=${st.servos[3]}° S5=${st.servos[4]}° G=${st.servos[5]}°`;

    document.getElementById('wpCount').innerText = st.waypoints.length + ' Waypoints';
    const wpDiv = document.getElementById('wpList');
    if (st.waypoints.length === 0) {
      wpDiv.innerText = 'No waypoints saved yet.';
    } else {
      wpDiv.innerHTML = st.waypoints.map((w, i) =>
        `<div>#${i+1}: [${w.q.map(v=>v.toFixed(2)).join(', ')}] G=${w.g.toFixed(2)}</div>`
      ).join('');
    }

    updateThreeScene(st);
  } catch (e) {}
  setTimeout(pollState, 60);
}

// ================= VISION & CAMERA TRACKING (OSMO ACTION 5 / WEBCAM) =================
let visionStream = null;
let isVisionRunning = false;
let followCubeActive = false;
let visionTargetColor = 'red';
let customTargetHSV = { h: 0, s: 0.7, v: 0.7, tol: 28 };
let visionSmoothed = { x: -0.042, y: -0.237, z: 0.245 };
let visionDetected = false;
let lastVisionSendTime = 0;
let lastFpsTime = performance.now();
let frameCount = 0;
let visionFps = 0;
let visionTargetMesh = null;

async function populateCameraDevices() {
  const sel = document.getElementById('cameraSelect');
  if (!sel) return;
  sel.innerHTML = '<option value="">Auto-Detect (DJI Osmo Action 5 / Webcam)...</option>';
  if (!navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) {
    return;
  }
  try {
    const devices = await navigator.mediaDevices.enumerateDevices();
    const vDevs = devices.filter(d => d.kind === 'videoinput');
    vDevs.forEach((dev, idx) => {
      const opt = document.createElement('option');
      opt.value = dev.deviceId;
      opt.text = dev.label || `Camera ${idx + 1} (${dev.deviceId.slice(0, 8)}...)`;
      const lbl = (dev.label || '').toLowerCase();
      if (lbl.includes('action') || lbl.includes('osmo') || lbl.includes('dji')) {
        opt.selected = true;
      }
      sel.appendChild(opt);
    });
  } catch (e) {
    console.error('Camera device enumeration error:', e);
  }
}

async function startCamera() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    alert('Camera API (navigator.mediaDevices) is not available in this window!\n\nReason: Browsers restrict camera access to "http://localhost:8080" or HTTPS.\nMake sure your address bar says http://localhost:8080 (not an IP address or remote name).');
    return;
  }
  const sel = document.getElementById('cameraSelect');
  const devId = (sel && sel.value) ? sel.value : null;
  const constraints = {
    video: devId ? { deviceId: { exact: devId } } : true
  };
  try {
    if (visionStream) {
      visionStream.getTracks().forEach(t => t.stop());
    }
    visionStream = await navigator.mediaDevices.getUserMedia(constraints);
    const video = document.getElementById('visionVideo');
    video.srcObject = visionStream;
    await video.play();
    isVisionRunning = true;
    document.getElementById('visionStatus').className = 'badge';
    document.getElementById('visionStatus').innerText = '🟢 Camera Active';
    await populateCameraDevices();
    requestAnimationFrame(processVisionFrame);
  } catch (err) {
    console.error('Primary camera start failed, trying fallback video:true...', err);
    try {
      visionStream = await navigator.mediaDevices.getUserMedia({ video: true });
      const video = document.getElementById('visionVideo');
      video.srcObject = visionStream;
      await video.play();
      isVisionRunning = true;
      document.getElementById('visionStatus').className = 'badge';
      document.getElementById('visionStatus').innerText = '🟢 Camera Active';
      await populateCameraDevices();
      requestAnimationFrame(processVisionFrame);
      return;
    } catch (err2) {
      alert('Camera access failed:\n' + err2.message + '\n\nTips:\n1. Browser me URL bar ke paas Camera Icon/Permission par "Allow" karein\n2. Make sure DJI Osmo Action 5 is turned on in "Webcam" mode and connected via USB\n3. Ensure no other application (Zoom, OBS) is holding the camera open');
    }
  }
}

function stopCamera() {
  if (visionStream) {
    visionStream.getTracks().forEach(t => t.stop());
    visionStream = null;
  }
  isVisionRunning = false;
  document.getElementById('visionStatus').className = 'badge warn';
  document.getElementById('visionStatus').innerText = 'Camera Off';
  document.getElementById('hudCoords').innerText = 'Target: No Cube Detected';
  document.getElementById('hudFps').innerText = '0 FPS';
  if (visionTargetMesh) visionTargetMesh.visible = false;
  const canvas = document.getElementById('visionCanvas');
  if (canvas) {
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#0b1120';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
  }
}

function setTrackColor(col) {
  visionTargetColor = col;
  const prev = document.getElementById('targetColorPreview');
  const lbl = document.getElementById('targetColorLbl');
  if (col === 'red') {
    prev.style.background = '#ef4444';
    lbl.innerText = 'RED';
    lbl.style.color = '#ef4444';
  } else if (col === 'green') {
    prev.style.background = '#10b981';
    lbl.innerText = 'GREEN';
    lbl.style.color = '#10b981';
  } else if (col === 'blue') {
    prev.style.background = '#3b82f6';
    lbl.innerText = 'BLUE';
    lbl.style.color = '#3b82f6';
  } else if (col === 'yellow') {
    prev.style.background = '#eab308';
    lbl.innerText = 'YELLOW';
    lbl.style.color = '#eab308';
  }
}

function onFollowToggle(checked) {
  followCubeActive = checked;
}

function onCanvasClick(evt) {
  if (!isVisionRunning) return;
  const canvas = document.getElementById('visionCanvas');
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const clickX = Math.floor((evt.clientX - rect.left) * scaleX);
  const clickY = Math.floor((evt.clientY - rect.top) * scaleY);

  const ctx = canvas.getContext('2d');
  const pixel = ctx.getImageData(clickX, clickY, 1, 1).data;
  const hsv = rgbToHsv(pixel[0], pixel[1], pixel[2]);
  customTargetHSV = { h: hsv.h, s: hsv.s, v: hsv.v, tol: 28 };
  visionTargetColor = 'custom';

  const prev = document.getElementById('targetColorPreview');
  const lbl = document.getElementById('targetColorLbl');
  prev.style.background = `rgb(${pixel[0]},${pixel[1]},${pixel[2]})`;
  lbl.innerText = `CUSTOM (H:${Math.round(hsv.h)}°)`;
  lbl.style.color = 'var(--accent)';
}

function rgbToHsv(r, g, b) {
  r /= 255; g /= 255; b /= 255;
  const max = Math.max(r, g, b), min = Math.min(r, g, b);
  let h, s, v = max;
  const d = max - min;
  s = max === 0 ? 0 : d / max;
  if (max === min) {
    h = 0;
  } else {
    switch (max) {
      case r: h = (g - b) / d + (g < b ? 6 : 0); break;
      case g: h = (b - r) / d + 2; break;
      case b: h = (r - g) / d + 4; break;
    }
    h /= 6;
  }
  return { h: h * 360, s, v };
}

function processVisionFrame() {
  if (!isVisionRunning) return;
  const video = document.getElementById('visionVideo');
  const canvas = document.getElementById('visionCanvas');
  if (!video || !canvas || video.readyState !== 4) {
    requestAnimationFrame(processVisionFrame);
    return;
  }
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const mirror = document.getElementById('mirrorVideo') ? document.getElementById('mirrorVideo').checked : true;

  ctx.save();
  if (mirror) {
    ctx.translate(W, 0);
    ctx.scale(-1, 1);
  }
  ctx.drawImage(video, 0, 0, W, H);
  ctx.restore();

  const imgData = ctx.getImageData(0, 0, W, H);
  const data = imgData.data;

  let sumX = 0, sumY = 0, count = 0;
  let minX = W, maxX = 0, minY = H, maxY = 0;
  const step = 3;

  for (let y = 0; y < H; y += step) {
    for (let x = 0; x < W; x += step) {
      const idx = (y * W + x) * 4;
      const r = data[idx], g = data[idx + 1], b = data[idx + 2];
      const hsv = rgbToHsv(r, g, b);

      let match = false;
      if (visionTargetColor === 'red') {
        match = (hsv.h < 18 || hsv.h > 342) && hsv.s >= 0.40 && hsv.v >= 0.25;
      } else if (visionTargetColor === 'green') {
        match = (hsv.h >= 85 && hsv.h <= 165) && hsv.s >= 0.35 && hsv.v >= 0.25;
      } else if (visionTargetColor === 'blue') {
        match = (hsv.h >= 195 && hsv.h <= 265) && hsv.s >= 0.40 && hsv.v >= 0.25;
      } else if (visionTargetColor === 'yellow') {
        match = (hsv.h >= 35 && hsv.h <= 75) && hsv.s >= 0.40 && hsv.v >= 0.30;
      } else if (visionTargetColor === 'custom') {
        let diff = Math.abs(hsv.h - customTargetHSV.h);
        if (diff > 180) diff = 360 - diff;
        match = diff <= customTargetHSV.tol && hsv.s >= 0.25 && hsv.v >= 0.20;
      }

      if (match) {
        sumX += x;
        sumY += y;
        count++;
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
      }
    }
  }

  frameCount++;
  const now = performance.now();
  if (now - lastFpsTime >= 500) {
    visionFps = Math.round((frameCount * 1000) / (now - lastFpsTime));
    frameCount = 0;
    lastFpsTime = now;
    document.getElementById('hudFps').innerText = visionFps + ' FPS';
  }

  if (count > 30) {
    visionDetected = true;
    const cx = sumX / count;
    const cy = sumY / count;
    const boxW = Math.max(12, maxX - minX);
    const boxH = Math.max(12, maxY - minY);
    const boxDim = Math.max(boxW, boxH);

    ctx.lineWidth = 2;
    ctx.strokeStyle = '#38bdf8';
    ctx.strokeRect(minX, minY, boxW, boxH);

    ctx.strokeStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(cx, cy, 8, 0, 2 * Math.PI);
    ctx.moveTo(cx - 14, cy); ctx.lineTo(cx + 14, cy);
    ctx.moveTo(cx, cy - 14); ctx.lineTo(cx, cy + 14);
    ctx.stroke();

    const u = (cx / W) - 0.5;
    const targetY = -u * 0.36;

    const v = 1.0 - (cy / H);
    const targetZ = 0.08 + v * 0.20;

    const frac = Math.min(1.0, Math.max(0.0, (boxDim - 25) / 115));
    const targetX = -0.14 - (1.0 - frac) * 0.12;

    const alpha = 0.22;
    visionSmoothed.x = (1 - alpha) * visionSmoothed.x + alpha * targetX;
    visionSmoothed.y = (1 - alpha) * visionSmoothed.y + alpha * targetY;
    visionSmoothed.z = (1 - alpha) * visionSmoothed.z + alpha * targetZ;

    document.getElementById('hudCoords').innerText =
      `X:${visionSmoothed.x.toFixed(3)} Y:${visionSmoothed.y.toFixed(3)} Z:${visionSmoothed.z.toFixed(3)} m`;
    document.getElementById('visBoxSize').innerText = Math.round(boxDim) + ' px';
    document.getElementById('visEstDepth').innerText = (visionSmoothed.x * 1000).toFixed(0) + ' mm';

    if (visionTargetMesh) {
      visionTargetMesh.visible = true;
      visionTargetMesh.position.set(visionSmoothed.x, visionSmoothed.y, visionSmoothed.z);
    }

    if (followCubeActive && Date.now() - lastVisionSendTime > 40) {
      lastVisionSendTime = Date.now();
      apiCall('/api/vision/track', {
        x: visionSmoothed.x,
        y: visionSmoothed.y,
        z: visionSmoothed.z
      });
    }
  } else {
    visionDetected = false;
    document.getElementById('hudCoords').innerText = 'Target: No Cube Detected';
    document.getElementById('visBoxSize').innerText = '0 px';
    document.getElementById('visEstDepth').innerText = '--';
    if (visionTargetMesh) visionTargetMesh.visible = false;
  }

  requestAnimationFrame(processVisionFrame);
}

function triggerVisionGrasp() {
  if (!visionDetected) {
    alert('Phele camera ke samne cube layein taaki robot use detect kar sake!');
    return;
  }
  document.getElementById('followCubeToggle').checked = false;
  followCubeActive = false;
  apiCall('/api/vision/grasp', {
    x: visionSmoothed.x,
    y: visionSmoothed.y,
    z: visionSmoothed.z
  });
}

function triggerVisionRelease() {
  apiCall('/api/vision/release', {});
}

window.addEventListener('DOMContentLoaded', async () => {
  try { buildServoStudioUI(); } catch (e) { console.error('buildServoStudioUI error:', e); }
  try { initThreeJS(); } catch (e) { console.error('initThreeJS error:', e); }
  try { await populateCameraDevices(); } catch (e) { console.error('populateCameraDevices error:', e); }
  try { pollState(); } catch (e) { console.error('pollState error:', e); }
});
</script>
</body>
</html>
"""


class IKInteractiveNode(Node):
    def __init__(self):
        super().__init__('ik_interactive_node')

        self.q = np.zeros(5, dtype=float)
        self.gripper_q = 0.0
        self.dragging = False
        self.playing_demo = False
        self.abort_flag = False
        self.speed_scale = 1.0
        self.active_task = 'Idle (Ready)'
        self.last_ik_pub_time = 0.0
        self.safety_status = 'SAFE (Collision-Free)'
        self.is_safe = True

        self.cubes = {
            col: {
                'pos': SORT_CONFIG[col]['pick_pos'].copy(),
                'attached': False,
            }
            for col in ('red', 'green', 'blue')
        }

        self.saved_waypoints = []
        self.load_waypoints()

        self.ik_pub = self.create_publisher(JointState, '/ik_joint_states', 10)
        self.scene_pub = self.create_publisher(MarkerArray, '/scene_markers', 10)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

        self.server = InteractiveMarkerServer(self, 'arm_ik_marker')
        self.menu_handler = MenuHandler()

        self.menu_handler.insert('Sort All 3 Cubes (R -> G -> B)', callback=lambda fb: self.trigger_sort_all())
        self.menu_handler.insert('Sort Red Cube', callback=lambda fb: self.trigger_sort_color('red'))
        self.menu_handler.insert('Sort Green Cube', callback=lambda fb: self.trigger_sort_color('green'))
        self.menu_handler.insert('Sort Blue Cube', callback=lambda fb: self.trigger_sort_color('blue'))
        self.menu_handler.insert('Reset Cubes & Home', callback=lambda fb: self.trigger_reset_cubes())

        self.create_tcp_marker()
        self.create_timer(0.05, self.publish_scene_markers)
        self.get_logger().info(f'3D Web Digital Twin & Multi-Color Sorter live at http://localhost:{WEB_PORT}')

    def load_waypoints(self):
        if os.path.exists(WAYPOINTS_FILE):
            try:
                with open(WAYPOINTS_FILE, 'r') as f:
                    raw = json.load(f)
                self.saved_waypoints = [(np.array(item['q']), float(item['g']), float(item.get('d', 1.0))) for item in raw]
            except Exception:
                self.saved_waypoints = []

    def persist_waypoints(self):
        try:
            os.makedirs(WAYPOINTS_DIR, exist_ok=True)
            data = [{'q': [round(float(x), 4) for x in q], 'g': round(float(g), 4), 'd': float(d)} for q, g, d in self.saved_waypoints]
            with open(WAYPOINTS_FILE, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception:
            pass

    def matrix_to_pose(self, T):
        pose = Pose()
        pose.position = Point(x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3]))
        quat = R.from_matrix(T[:3, :3]).as_quat()
        pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        return pose

    def publish_scene_markers(self):
        T_tcp = forward_kinematics(self.q)
        tcp_pos = T_tcp[:3, 3]

        if self.gripper_q > 0.42:
            for col in self.cubes:
                self.cubes[col]['attached'] = False
        elif self.gripper_q <= 0.30:
            if not any(c['attached'] for c in self.cubes.values()):
                for col, c in self.cubes.items():
                    if np.linalg.norm(tcp_pos - c['pos']) < 0.032:
                        c['attached'] = True
                        break

        for col, c in self.cubes.items():
            if c['attached']:
                c['pos'] = tcp_pos.copy()

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 1

        for col in ('red', 'green', 'blue'):
            cfg = SORT_CONFIG[col]
            r, g, b = cfg['rgb']
            c_state = self.cubes[col]

            pick_pad = Marker()
            pick_pad.header.frame_id = 'world'
            pick_pad.header.stamp = now
            pick_pad.ns = 'workcell'
            pick_pad.id = mid; mid += 1
            pick_pad.type = Marker.CYLINDER
            pick_pad.action = Marker.ADD
            pick_pad.pose.position = Point(x=float(cfg['pick_pos'][0]), y=float(cfg['pick_pos'][1]), z=float(cfg['pick_pos'][2] - 0.013))
            pick_pad.pose.orientation.w = 1.0
            pick_pad.scale.x = 0.044; pick_pad.scale.y = 0.044; pick_pad.scale.z = 0.003
            pick_pad.color.r = 0.45; pick_pad.color.g = 0.52; pick_pad.color.b = 0.62; pick_pad.color.a = 0.7
            ma.markers.append(pick_pad)

            bin_pad = Marker()
            bin_pad.header.frame_id = 'world'
            bin_pad.header.stamp = now
            bin_pad.ns = 'workcell'
            bin_pad.id = mid; mid += 1
            bin_pad.type = Marker.CYLINDER
            bin_pad.action = Marker.ADD
            bin_pad.pose.position = Point(x=float(cfg['place_pos'][0]), y=float(cfg['place_pos'][1]), z=float(cfg['place_pos'][2] - 0.013))
            bin_pad.pose.orientation.w = 1.0
            bin_pad.scale.x = 0.050; bin_pad.scale.y = 0.050; bin_pad.scale.z = 0.004
            bin_pad.color.r = float(r); bin_pad.color.g = float(g); bin_pad.color.b = float(b); bin_pad.color.a = 0.75
            ma.markers.append(bin_pad)

            cube = Marker()
            cube.header.frame_id = 'world'
            cube.header.stamp = now
            cube.ns = 'workcell'
            cube.id = mid; mid += 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position = Point(x=float(c_state['pos'][0]), y=float(c_state['pos'][1]), z=float(c_state['pos'][2]))
            if c_state['attached']:
                quat = R.from_matrix(T_tcp[:3, :3]).as_quat()
                cube.pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
            else:
                cube.pose.orientation.w = 1.0
            cube.scale.x = 0.020; cube.scale.y = 0.020; cube.scale.z = 0.020
            cube.color.r = float(r); cube.color.g = float(g); cube.color.b = float(b); cube.color.a = 1.0
            ma.markers.append(cube)

        self.scene_pub.publish(ma)

    def create_tcp_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = 'tcp_goal'
        int_marker.description = 'Drag Gripper TCP (Right-Click for Color Sorting)'
        int_marker.scale = 0.08
        int_marker.pose = self.matrix_to_pose(forward_kinematics(self.q))

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.025; sphere.scale.y = 0.025; sphere.scale.z = 0.025
        sphere.color.r = 0.0; sphere.color.g = 0.85; sphere.color.b = 1.0; sphere.color.a = 0.75

        center_ctrl = InteractiveMarkerControl()
        center_ctrl.always_visible = True
        center_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        center_ctrl.markers.append(sphere)
        int_marker.controls.append(center_ctrl)

        axes = [
            ('move_x', 'rotate_x', Quaternion(w=1.0, x=1.0, y=0.0, z=0.0)),
            ('move_z', 'rotate_z', Quaternion(w=1.0, x=0.0, y=1.0, z=0.0)),
            ('move_y', 'rotate_y', Quaternion(w=1.0, x=0.0, y=0.0, z=1.0)),
        ]
        for move_name, rot_name, orn in axes:
            ctrl_m = InteractiveMarkerControl()
            ctrl_m.name = move_name
            ctrl_m.orientation = orn
            ctrl_m.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(ctrl_m)

            ctrl_r = InteractiveMarkerControl()
            ctrl_r.name = rot_name
            ctrl_r.orientation = orn
            ctrl_r.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(ctrl_r)

        self.server.insert(int_marker, feedback_callback=self.on_marker_feedback)
        self.menu_handler.apply(self.server, int_marker.name)
        self.server.applyChanges()

    def publish_ik_state(self):
        self.last_ik_pub_time = time.time()
        self.q = np.clip(self.q, LOWER_LIMITS, UPPER_LIMITS)
        self.gripper_q = float(np.clip(self.gripper_q, GRIPPER_MIN, GRIPPER_MAX))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES + ['gripper_joint']
        msg.position = [float(x) for x in self.q] + [float(self.gripper_q)]
        self.ik_pub.publish(msg)

    def sync_marker_to_fk(self):
        pose = self.matrix_to_pose(forward_kinematics(self.q))
        self.server.setPose('tcp_goal', pose)
        self.server.applyChanges()

    def on_marker_feedback(self, feedback):
        if self.playing_demo:
            return

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.dragging = True
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.dragging = True
            p = feedback.pose.position
            o = feedback.pose.orientation
            target_pos = np.array([p.x, p.y, p.z])
            target_rot = R.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
            self.q = solve_ik(self.q, target_pos, target_rot, max_iters=15)
            self.is_safe, self.safety_status = check_collision(self.q)
            self.publish_ik_state()
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.dragging = False
            self.sync_marker_to_fk()

    def on_joint_states(self, msg):
        if self.dragging or self.playing_demo or (time.time() - self.last_ik_pub_time < 0.25):
            return
        name_map = dict(zip(msg.name, msg.position))
        if all(j in name_map for j in JOINT_NAMES):
            new_q = np.array([name_map[j] for j in JOINT_NAMES])
            if 'gripper_joint' in name_map:
                self.gripper_q = float(np.clip(name_map['gripper_joint'], GRIPPER_MIN, GRIPPER_MAX))

            ok, status = check_collision(new_q)
            self.is_safe = ok
            self.safety_status = status
            if not ok:
                self.publish_ik_state()
                return

            if np.linalg.norm(new_q - self.q) > 1e-3:
                self.q = np.clip(new_q, LOWER_LIMITS, UPPER_LIMITS)
                self.sync_marker_to_fk()

    def set_direct_joints(self, q_list, g_val):
        if self.playing_demo:
            return
        cand_q = np.clip(np.array(q_list, dtype=float), LOWER_LIMITS, UPPER_LIMITS)
        ok, status = check_collision(cand_q)
        self.is_safe = ok
        self.safety_status = status
        if ok:
            self.q = cand_q
        self.gripper_q = float(np.clip(g_val, GRIPPER_MIN, GRIPPER_MAX))
        self.publish_ik_state()
        self.sync_marker_to_fk()

    def set_direct_ik(self, x, y, z):
        if self.playing_demo:
            return
        target = np.array([float(x), float(y), float(z)])
        self.q = solve_ik(self.q, target, None, max_iters=20)
        self.is_safe, self.safety_status = check_collision(self.q)
        self.publish_ik_state()
        self.sync_marker_to_fk()

    def step_cartesian(self, dx, dy, dz):
        if self.playing_demo:
            return
        T_now = forward_kinematics(self.q)
        target = T_now[:3, 3] + np.array([dx, dy, dz])
        self.q = solve_ik(self.q, target, None, max_iters=20)
        self.is_safe, self.safety_status = check_collision(self.q)
        self.publish_ik_state()
        self.sync_marker_to_fk()

    def animate_to(self, target_q, target_gripper, duration=1.0):
        start_q = self.q.copy()
        start_g = float(self.gripper_q)
        target_q = np.clip(target_q, LOWER_LIMITS, UPPER_LIMITS)
        target_gripper = float(np.clip(target_gripper, GRIPPER_MIN, GRIPPER_MAX))
        eff_dur = max(0.15, duration / max(0.25, self.speed_scale))
        steps = max(8, int(eff_dur * 50))
        for i in range(1, steps + 1):
            if self.abort_flag:
                break
            alpha = 0.5 * (1.0 - np.cos(np.pi * i / steps))
            cand_q = (1.0 - alpha) * start_q + alpha * np.array(target_q)
            ok, status = check_collision(cand_q)
            self.is_safe = ok
            self.safety_status = status
            if ok:
                self.q = cand_q
            self.gripper_q = (1.0 - alpha) * start_g + alpha * target_gripper
            self.publish_ik_state()
            self.sync_marker_to_fk()
            time.sleep(eff_dur / steps)

    def trigger_estop(self):
        self.abort_flag = True
        self.playing_demo = False
        self.active_task = '🛑 E-STOPPED'

    def trigger_open_gripper(self):
        if not self.playing_demo:
            threading.Thread(target=self._run_seq, args=([(self.q.copy(), 0.60, 0.5)], 'Opening Gripper'), daemon=True).start()

    def trigger_close_gripper(self):
        if not self.playing_demo:
            threading.Thread(target=self._run_seq, args=([(self.q.copy(), 0.00, 0.5)], 'Closing Gripper'), daemon=True).start()

    def trigger_home_pose(self):
        self.abort_flag = True
        time.sleep(0.05)
        threading.Thread(target=self._run_seq, args=([(np.zeros(5), 0.0, 0.9)], 'Moving Home'), daemon=True).start()

    def trigger_reset_cubes(self):
        self.abort_flag = True
        time.sleep(0.05)
        for col in ('red', 'green', 'blue'):
            self.cubes[col]['attached'] = False
            self.cubes[col]['pos'] = SORT_CONFIG[col]['pick_pos'].copy()
        threading.Thread(target=self._run_seq, args=([(np.zeros(5), 0.0, 0.9)], 'Reset Cubes & Home'), daemon=True).start()

    def _build_color_waypoints(self, col):
        cfg = SORT_CONFIG[col]
        return [
            (cfg['q_Lift1'], 0.60, 0.7),
            (cfg['q_pick'],  0.60, 0.9),
            (cfg['q_pick'],  0.24, 0.5),
            (cfg['q_Lift1'], 0.24, 0.8),
            (cfg['q_Lift2'], 0.24, 1.1),
            (cfg['q_place'], 0.24, 0.9),
            (cfg['q_place'], 0.60, 0.5),
            (cfg['q_Lift2'], 0.60, 0.7),
        ]

    def trigger_sort_color(self, col):
        if not self.playing_demo:
            self.cubes[col]['attached'] = False
            self.cubes[col]['pos'] = SORT_CONFIG[col]['pick_pos'].copy()
            wps = self._build_color_waypoints(col) + [(np.zeros(5), 0.0, 0.8)]
            threading.Thread(target=self._run_seq, args=(wps, f'Sorting {col.upper()} Cube'), daemon=True).start()

    def trigger_sort_all(self):
        if not self.playing_demo:
            for col in ('red', 'green', 'blue'):
                self.cubes[col]['attached'] = False
                self.cubes[col]['pos'] = SORT_CONFIG[col]['pick_pos'].copy()
            wps = (
                self._build_color_waypoints('red') +
                self._build_color_waypoints('green') +
                self._build_color_waypoints('blue') +
                [(np.zeros(5), 0.0, 0.9)]
            )
            threading.Thread(target=self._run_seq, args=(wps, 'Auto-Sorting R → G → B Cubes'), daemon=True).start()

    def trigger_wave_demo(self):
        if not self.playing_demo:
            waypoints = [
                (np.array([ 0.0, -0.25, -0.35,  0.0,  0.2]),  0.55, 0.8),
                (np.array([ 0.35, -0.25, -0.35,  0.8, -0.3]), -0.30, 0.5),
                (np.array([-0.35, -0.25, -0.35, -0.8,  0.3]),  0.55, 0.5),
                (np.array([ 0.35, -0.25, -0.35,  0.8, -0.3]), -0.30, 0.5),
                (np.array([-0.35, -0.25, -0.35, -0.8,  0.3]),  0.55, 0.5),
                (np.array([ 0.0,  0.0,   0.0,   0.0,  0.0]),  0.00, 0.9),
                (np.array([ 0.0, -0.25, -0.35,  0.0,  0.2]), 0.55, 0.8),
                (np.array([ 0.35, -0.25, -0.35,  0.8, -0.3]), 0.05, 0.5),
                (np.array([-0.35, -0.25, -0.35, -0.8,  0.3]), 0.55, 0.5),
                (np.array([ 0.35, -0.25, -0.35,  0.8, -0.3]), 0.05, 0.5),
                (np.array([-0.35, -0.25, -0.35, -0.8,  0.3]), 0.55, 0.5),
                (np.array([ 0.0,  0.0,   0.0,   0.0,  0.0]), 0.00, 0.9),
            ]
            threading.Thread(target=self._run_seq, args=(waypoints, 'Wave Hello Demo'), daemon=True).start()

    def trigger_scan_demo(self):
        if not self.playing_demo:
            waypoints = [
                (np.array([-1.2, 0.35, 0.20,  0.0, -0.4]), 0.2, 1.0),
                (np.array([-0.4, 0.45, 0.25,  0.4, -0.3]), 0.2, 0.9),
                (np.array([ 0.4, 0.45, 0.25, -0.4, -0.3]), 0.2, 0.9),
                (np.array([ 1.2, 0.35, 0.20,  0.0, -0.4]), 0.2, 0.9),
                (np.array([ 0.0, 0.0,  0.0,   0.0,  0.0]), 0.0, 1.0),
            ]
            threading.Thread(target=self._run_seq, args=(waypoints, '360° Workspace Scan'), daemon=True).start()

    def trigger_circle_demo(self):
        if not self.playing_demo:
            waypoints = []
            q_curr = self.q.copy()
            center = np.array([-0.06, -0.20, 0.21])
            radius = 0.055
            for k in range(25):
                ang = 2.0 * np.pi * k / 24.0
                target = center + np.array([radius * np.cos(ang), 0.0, radius * np.sin(ang)])
                q_curr = solve_ik(q_curr, target, None, max_iters=20)
                waypoints.append((q_curr.copy(), 0.0, 0.12))
            waypoints.append((np.zeros(5), 0.0, 0.9))
            threading.Thread(target=self._run_seq, args=(waypoints, 'Drawing 3D Circle (IK)'), daemon=True).start()

    def vision_track(self, x, y, z):
        if self.playing_demo:
            return
        target = np.array([float(x), float(y), float(z)])
        cand_q = solve_ik(self.q, target, None, max_iters=14)
        ok, status = check_collision(cand_q)
        self.is_safe = ok
        self.safety_status = status
        if ok:
            self.q = cand_q
            self.publish_ik_state()
            self.sync_marker_to_fk()

    def vision_grasp(self, x, y, z):
        if self.playing_demo:
            return
        tx, ty, tz = float(x), float(y), float(z)
        q_pre = solve_ik(self.q, np.array([tx, ty, tz + 0.055]), None, max_iters=25)
        q_pick = solve_ik(q_pre, np.array([tx, ty, tz]), None, max_iters=25)
        q_lift = solve_ik(q_pick, np.array([tx, ty, tz + 0.075]), None, max_iters=25)

        waypoints = [
            (q_pre,  0.60, 0.7),
            (q_pick, 0.60, 0.5),
            (q_pick, 0.24, 0.5),
            (q_lift, 0.24, 0.8),
        ]
        threading.Thread(target=self._run_seq, args=(waypoints, f'Vision: Holding Cube at ({tx:.2f}, {ty:.2f})'), daemon=True).start()

    def vision_release(self):
        if self.playing_demo:
            return
        waypoints = [
            (self.q.copy(), 0.60, 0.5),
            (np.zeros(5),   0.00, 0.8),
        ]
        threading.Thread(target=self._run_seq, args=(waypoints, 'Vision: Release Cube & Home'), daemon=True).start()

    def _run_seq(self, waypoints, label='Running Mission', loop=False):
        self.playing_demo = True
        self.abort_flag = False
        self.active_task = label + (' (Loop)' if loop else '')
        try:
            while True:
                for t_q, t_g, dur in waypoints:
                    if self.abort_flag:
                        break
                    self.animate_to(t_q, t_g, dur)
                if not loop or self.abort_flag:
                    break
        finally:
            self.playing_demo = False
            if not self.abort_flag:
                self.active_task = 'Idle (Ready)'

    def get_state_dict(self):
        T1, T2, T3, T4, T5, Ttcp = forward_kinematics_chain(self.q)
        servos = [int(round(np.degrees(ang) + 90)) for ang in self.q] + [int(round(np.degrees(self.gripper_q) + 90))]
        cubes_out = {
            col: {
                'pos':  [round(float(x), 4) for x in self.cubes[col]['pos']],
                'pick': [round(float(x), 4) for x in SORT_CONFIG[col]['pick_pos']],
                'bin':  [round(float(x), 4) for x in SORT_CONFIG[col]['place_pos']],
            }
            for col in ('red', 'green', 'blue')
        }
        return {
            'q': [round(float(x), 4) for x in self.q],
            'g': round(float(self.gripper_q), 4),
            'tcp': [round(float(x), 4) for x in Ttcp[:3, 3]],
            'servos': servos,
            'is_safe': self.is_safe,
            'safety_status': self.safety_status,
            'active_task': self.active_task,
            'cubes': cubes_out,
            'waypoints': [{'q': [round(float(x), 3) for x in q], 'g': round(float(g), 3)} for q, g, _ in self.saved_waypoints],
        }


def start_web_server(node: IKInteractiveNode):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def _send_json(self, obj):
            data = json.dumps(obj).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/api/state':
                self._send_json(node.get_state_dict())
            elif self.path.startswith('/meshes/'):
                fname = os.path.basename(self.path)
                fpath = os.path.join(MESH_DIR, fname)
                if os.path.exists(fpath):
                    with open(fpath, 'rb') as f:
                        data = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/octet-stream')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                else:
                    self.send_error(404)
            elif self.path.startswith('/static/'):
                fname = os.path.basename(self.path)
                fpath = os.path.join(STATIC_DIR, fname)
                if os.path.exists(fpath):
                    with open(fpath, 'rb') as f:
                        data = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/javascript; charset=utf-8')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                else:
                    self.send_error(404)
            else:
                body = HTML_PAGE.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        def do_POST(self):
            length = int(self.headers.get('Content-Length', 0))
            raw = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
            payload = json.loads(raw)

            if self.path == '/api/joints':
                node.set_direct_joints(payload['q'], payload['g'])
            elif self.path == '/api/ik':
                node.set_direct_ik(payload['x'], payload['y'], payload['z'])
            elif self.path == '/api/jog':
                node.step_cartesian(payload['dx'], payload['dy'], payload['dz'])
            elif self.path == '/api/speed':
                node.speed_scale = float(payload.get('speed', 1.0))
            elif self.path == '/api/waypoint':
                op = payload.get('op')
                if op == 'save':
                    node.saved_waypoints.append((node.q.copy(), float(node.gripper_q), 1.0))
                    node.persist_waypoints()
                elif op == 'clear':
                    node.saved_waypoints.clear()
                    node.persist_waypoints()
                elif op == 'play' and not node.playing_demo and node.saved_waypoints:
                    loop = bool(payload.get('loop', False))
                    threading.Thread(
                        target=node._run_seq,
                        args=(list(node.saved_waypoints), 'Playing Saved Waypoints', loop),
                        daemon=True,
                    ).start()
            elif self.path == '/api/action':
                act = payload.get('action')
                if act == 'estop':
                    node.trigger_estop()
                elif act == 'sort_red':
                    node.trigger_sort_color('red')
                elif act == 'sort_green':
                    node.trigger_sort_color('green')
                elif act == 'sort_blue':
                    node.trigger_sort_color('blue')
                elif act == 'sort_all':
                    node.trigger_sort_all()
                elif act == 'reset_cubes':
                    node.trigger_reset_cubes()
                elif act == 'circle_demo':
                    node.trigger_circle_demo()
                elif act == 'wave_demo':
                    node.trigger_wave_demo()
                elif act == 'scan_demo':
                    node.trigger_scan_demo()
                elif act == 'open_gripper':
                    node.trigger_open_gripper()
                elif act == 'close_gripper':
                    node.trigger_close_gripper()
                elif act == 'home':
                    node.trigger_home_pose()
            elif self.path == '/api/vision/track':
                node.vision_track(payload['x'], payload['y'], payload['z'])
            elif self.path == '/api/vision/grasp':
                node.vision_grasp(payload['x'], payload['y'], payload['z'])
            elif self.path == '/api/vision/release':
                node.vision_release()

            self._send_json({'ok': True})

    ThreadingHTTPServer.allow_reuse_address = True
    httpd = ThreadingHTTPServer(('0.0.0.0', WEB_PORT), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    time.sleep(0.4)
    try:
        webbrowser.open(f'http://localhost:{WEB_PORT}')
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = IKInteractiveNode()
    start_web_server(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
