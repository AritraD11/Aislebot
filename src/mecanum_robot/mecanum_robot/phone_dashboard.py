#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  AisleBot Phone Dashboard v2.0                                   ║
║  ROS2 Node + FastAPI WebSocket on port 8080                      ║
║                                                                  ║
║  Controls:                                                       ║
║    • Drive joystick (vx / vy) + separate yaw slider             ║
║    • Speed: SLOW (25%) / NORMAL (60%) / FAST (100%)             ║
║    • Arm: LIFT / LOWER / OPEN / CLOSE                           ║
║    • Record Run → ~/aislebot_logs/run_YYYYMMDD_HHMMSS.csv       ║
║    • E-STOP (latches) / CLEAR                                   ║
║                                                                  ║
║  ROS2 Topics:                                                    ║
║    Publishes  /cmd_vel          geometry_msgs/Twist  (drive)    ║
║    Publishes  /arm/command      std_msgs/String       (arm)     ║
║    Publishes  /esp32/command    std_msgs/String  (<L1>/<L0>/…)  ║
║    Subscribes /motor_telemetry  std_msgs/Float64MultiArray      ║
║                                                                  ║
║  ⚠  esp32_bridge.py must:                                       ║
║      • Subscribe to /esp32/command → forward raw string serial  ║
║      • Parse incoming telemetry CSV → publish /motor_telemetry  ║
║      Float64MultiArray format (12 values):                      ║
║      [FR_t, FR_a, FR_p, FL_t, FL_a, FL_p,                      ║
║       RR_t, RR_a, RR_p, RL_t, RL_a, RL_p]                      ║
║                                                                  ║
║  Aritra Das (25D0074) — IIT Bombay — Prof. Ambarish Kunwar      ║
╚══════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray

import threading
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

import json
import csv
import os
import time
from datetime import datetime
from typing import Optional, Set

# ═══════════════════════════════════════════════════════════════════
#  ROBOT CONSTANTS  (keep in sync with ESP32 firmware)
# ═══════════════════════════════════════════════════════════════════
MAX_LINEAR_SPEED  = 0.48   # m/s   (MAX_WHEEL_SPEED × WHEEL_RADIUS)
MAX_ANGULAR_SPEED = 1.0    # rad/s

SPEED_MODES = [
    {"name": "SLOW",   "mult": 0.25, "label": "0.12 m/s"},
    {"name": "NORMAL", "mult": 0.60, "label": "0.29 m/s"},
    {"name": "FAST",   "mult": 1.00, "label": "0.48 m/s"},
]

# ═══════════════════════════════════════════════════════════════════
#  HTML DASHBOARD  (served to the phone browser)
# ═══════════════════════════════════════════════════════════════════
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>AisleBot</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;-webkit-user-select:none;user-select:none;touch-action:none}
html,body{width:100vw;height:100vh;overflow:hidden;background:#0a0e17;color:#e0e0e0;font-family:'Courier New',monospace}

/* ── HEADER ── */
.hdr{display:flex;align-items:center;justify-content:space-between;
     height:44px;padding:0 12px;background:#111827;
     border-bottom:1.5px solid #1e3a5f;flex-shrink:0;gap:8px}
.hdr-title{font-size:12px;font-weight:700;color:#38bdf8;letter-spacing:2px;white-space:nowrap}
.spd-group{display:flex;gap:4px;flex-shrink:0}
.spd-btn{padding:3px 8px;border-radius:4px;font-size:10px;font-weight:700;
         border:1.5px solid #1e3a5f;background:transparent;color:#4b5563;
         cursor:pointer;letter-spacing:.5px;transition:all .15s;font-family:inherit}
.spd-btn.active-slow  {background:#052e16;color:#22c55e;border-color:#22c55e}
.spd-btn.active-normal{background:#431407;color:#f59e0b;border-color:#f59e0b}
.spd-btn.active-fast  {background:#450a0a;color:#ef4444;border-color:#ef4444}
.hdr-right{display:flex;align-items:center;gap:8px}
.status-dot{width:9px;height:9px;border-radius:50%;background:#ef4444;flex-shrink:0;
            transition:background .3s}
.status-dot.on{background:#22c55e;box-shadow:0 0 6px #22c55e}
.rec-badge{font-size:9px;font-weight:700;padding:2px 6px;border-radius:3px;
           background:#450a0a;color:#f87171;letter-spacing:1px;display:none}
.rec-badge.show{display:inline-block;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.5}}

/* ── LAYOUT ── */
.body{display:flex;height:calc(100vh - 44px - 72px)}

/* ── JOYSTICK AREA ── */
.joy-area{flex:1;position:relative;display:flex;align-items:center;
          justify-content:center;background:#0a0e17;overflow:hidden}
.joy-ring{width:min(58vw,260px);height:min(58vw,260px);border-radius:50%;
          border:2px solid #1e3a5f;position:relative;flex-shrink:0}
.joy-thumb{width:64px;height:64px;border-radius:50%;position:absolute;
           top:50%;left:50%;transform:translate(-50%,-50%);
           background:radial-gradient(circle at 35% 35%,#38bdf8,#0369a1);
           box-shadow:0 0 16px rgba(56,189,248,.3);transition:box-shadow .1s}
.joy-thumb.active{box-shadow:0 0 28px rgba(56,189,248,.7)}
.jlbl{position:absolute;font-size:8px;color:#1e3a5f;font-weight:700;letter-spacing:1px}
.jlbl.t{top:6%;left:50%;transform:translateX(-50%)}
.jlbl.b{bottom:6%;left:50%;transform:translateX(-50%)}
.jlbl.l{left:5%;top:50%;transform:translateY(-50%)}
.jlbl.r{right:5%;top:50%;transform:translateY(-50%)}

/* ── RIGHT PANEL ── */
.right-panel{width:120px;display:flex;flex-direction:column;
             border-left:1.5px solid #1e3a5f;background:#0c1220;flex-shrink:0}

/* YAW SLIDER */
.yaw-wrap{flex:2;display:flex;flex-direction:column;align-items:center;
          justify-content:center;padding:10px 0;gap:5px;border-bottom:1px solid #1e3a5f}
.yaw-lbl{font-size:8px;color:#334155;letter-spacing:1px;font-weight:700}
.yaw-track{width:24px;flex:1;max-height:120px;background:#111827;border-radius:12px;
           border:1.5px solid #1e3a5f;position:relative;cursor:pointer;touch-action:none}
.yaw-thumb{width:38px;height:38px;border-radius:50%;position:absolute;
           left:50%;top:50%;transform:translate(-50%,-50%);
           background:radial-gradient(circle at 35% 35%,#fbbf24,#78350f);
           box-shadow:0 0 10px rgba(251,191,36,.35)}

/* ARM BUTTONS */
.arm-section{flex:3;display:flex;flex-direction:column;padding:6px;
             gap:5px;border-bottom:1px solid #1e3a5f;justify-content:center}
.arm-btn{flex:1;border:1.5px solid #134e4a;background:#042f2e;color:#5eead4;
         font-size:10px;font-weight:700;border-radius:5px;cursor:pointer;
         font-family:inherit;letter-spacing:.5px;transition:background .1s;
         display:flex;align-items:center;justify-content:center}
.arm-btn:active,.arm-btn.held{background:#0d3d3a;box-shadow:inset 0 0 8px rgba(94,234,212,.2)}

/* LIFT BUTTONS */
.lift-section{flex:2;display:flex;flex-direction:column;padding:6px;
              gap:5px;justify-content:center}
.lift-btn{flex:1;border:1.5px solid #1e3a5f;background:#0f172a;color:#7dd3fc;
          font-size:11px;font-weight:700;border-radius:5px;cursor:pointer;
          font-family:inherit;letter-spacing:.5px;transition:background .1s;
          display:flex;align-items:center;justify-content:center;gap:4px}
.lift-btn:active,.lift-btn.held{background:#1e293b;box-shadow:inset 0 0 8px rgba(125,211,252,.2)}

/* ── BOTTOM BAR ── */
.bottom{display:flex;height:72px;flex-shrink:0;border-top:1.5px solid #1e3a5f;
        background:#111827}
.rec-btn{flex:1;border:none;border-right:1.5px solid #1e3a5f;
         background:transparent;color:#22c55e;font-size:11px;font-weight:700;
         cursor:pointer;font-family:inherit;letter-spacing:.5px;
         display:flex;flex-direction:column;align-items:center;justify-content:center;gap:3px;
         transition:background .15s}
.rec-btn .rec-icon{font-size:18px;line-height:1}
.rec-btn.recording{background:#0a1f0a;color:#4ade80;animation:recpulse 1.5s infinite}
@keyframes recpulse{0%,100%{background:#0a1f0a}50%{background:#052e16}}
.estop-btn{flex:1;border:none;background:transparent;
           display:flex;flex-direction:column;align-items:center;justify-content:center;gap:3px;
           cursor:pointer;transition:background .1s}
.estop-btn .estop-circle{width:48px;height:48px;border-radius:50%;
  background:radial-gradient(circle at 40% 35%,#f87171,#7f1d1d);
  border:2px solid #fca5a5;box-shadow:0 0 16px rgba(239,68,68,.25),inset 0 -3px 6px rgba(0,0,0,.3);
  display:flex;align-items:center;justify-content:center;
  font-size:9px;font-weight:700;color:#fff;letter-spacing:.5px}
.estop-btn:active .estop-circle{transform:scale(.88);box-shadow:0 0 28px rgba(239,68,68,.7)}
.estop-btn.armed .estop-circle{background:radial-gradient(circle at 40% 35%,#4ade80,#14532d);
  border-color:#86efac;box-shadow:0 0 16px rgba(74,222,128,.25)}
.estop-lbl{font-size:8px;color:#4b5563;letter-spacing:.5px;font-weight:700}

/* ── FLASH OVERLAY ── */
.flash{position:fixed;inset:0;background:rgba(239,68,68,.2);pointer-events:none;
       opacity:0;transition:opacity .25s;z-index:999}
.flash.show{opacity:1}

/* ── GRID LINES decorative ── */
.joy-area::before{content:'';position:absolute;inset:0;
  background:radial-gradient(ellipse at center,rgba(56,189,248,.04) 0%,transparent 70%);
  pointer-events:none}
</style>
</head>
<body>

<!-- HEADER -->
<div class="hdr">
  <span class="hdr-title">AISLEBOT</span>
  <div class="spd-group">
    <button class="spd-btn active-slow" id="spd0" onclick="setSpeed(0)">SLOW</button>
    <button class="spd-btn" id="spd1" onclick="setSpeed(1)">MED</button>
    <button class="spd-btn" id="spd2" onclick="setSpeed(2)">FAST</button>
  </div>
  <div class="hdr-right">
    <span class="rec-badge" id="recBadge">REC</span>
    <span class="status-dot" id="dot"></span>
  </div>
</div>

<!-- BODY: joystick (left) + right panel -->
<div class="body">

  <!-- JOYSTICK -->
  <div class="joy-area" id="joyArea">
    <div class="joy-ring" id="joyRing">
      <span class="jlbl t">FWD</span>
      <span class="jlbl b">REV</span>
      <span class="jlbl l">LEFT</span>
      <span class="jlbl r">RIGHT</span>
      <div class="joy-thumb" id="joyThumb"></div>
    </div>
  </div>

  <!-- RIGHT PANEL -->
  <div class="right-panel">

    <!-- YAW SLIDER -->
    <div class="yaw-wrap">
      <span class="yaw-lbl">CCW</span>
      <div class="yaw-track" id="yawTrack">
        <div class="yaw-thumb" id="yawThumb"></div>
      </div>
      <span class="yaw-lbl">CW</span>
    </div>

    <!-- ARM OPEN / CLOSE -->
    <div class="arm-section">
      <button class="arm-btn" id="btnOpen">ARM<br>OPEN</button>
      <button class="arm-btn" id="btnClose">ARM<br>CLOSE</button>
    </div>

    <!-- LIFT / LOWER -->
    <div class="lift-section">
      <button class="lift-btn" id="btnUp">▲ LIFT</button>
      <button class="lift-btn" id="btnDown">▼ LOWER</button>
    </div>

  </div><!-- /right-panel -->

</div><!-- /body -->

<!-- BOTTOM BAR -->
<div class="bottom">
  <button class="rec-btn" id="recBtn" onclick="toggleRecord()">
    <span class="rec-icon" id="recIcon">⏺</span>
    <span id="recLabel">RECORD RUN</span>
  </button>
  <button class="estop-btn" id="estopBtn">
    <div class="estop-circle" id="estopCircle">E-STOP</div>
    <span class="estop-lbl" id="estopLbl">TAP TO STOP</span>
  </button>
</div>

<div class="flash" id="flash"></div>

<script>
// ── CONFIG ────────────────────────────────────────────────────────
const MAX_LINEAR  = 0.48;   // m/s  (matches ESP32 firmware)
const MAX_ANGULAR = 1.0;    // rad/s
const DEADZONE    = 0.07;
const SPEEDS      = [0.25, 0.60, 1.00];
const SPD_ACTIVE  = ['active-slow','active-normal','active-fast'];

// ── STATE ─────────────────────────────────────────────────────────
let ws, wsOk = false;
let speedIdx    = 0;
let joyX = 0, joyY = 0, joyActive = false, joyTouchId = null;
let yawVal = 0, yawActive = false, yawTouchId = null;
let estopped  = false;
let recording = false;
let armInterval = null;

// ── WEBSOCKET ─────────────────────────────────────────────────────
function connect() {
  const url = 'ws://' + location.host + '/ws';
  ws = new WebSocket(url);
  ws.onopen  = () => { wsOk = true;  document.getElementById('dot').classList.add('on'); };
  ws.onclose = () => { wsOk = false; document.getElementById('dot').classList.remove('on'); setTimeout(connect, 1500); };
  ws.onerror = () => ws.close();
}
connect();

function send(obj) { if (wsOk) ws.send(JSON.stringify(obj)); }

// ── SPEED MODE ────────────────────────────────────────────────────
function setSpeed(idx) {
  speedIdx = idx;
  for (let i = 0; i < 3; i++) {
    const btn = document.getElementById('spd'+i);
    SPD_ACTIVE.forEach(c => btn.classList.remove(c));
    if (i === idx) btn.classList.add(SPD_ACTIVE[i]);
  }
}

// ── JOYSTICK ──────────────────────────────────────────────────────
const joyArea  = document.getElementById('joyArea');
const joyRing  = document.getElementById('joyRing');
const joyThumb = document.getElementById('joyThumb');

function getJoyOffset(touch) {
  const r   = joyRing.getBoundingClientRect();
  const cx  = r.left + r.width  / 2;
  const cy  = r.top  + r.height / 2;
  const max = r.width / 2 - 32;
  let dx = touch.clientX - cx;
  let dy = touch.clientY - cy;
  const dist = Math.hypot(dx, dy);
  if (dist > max) { const s = max / dist; dx *= s; dy *= s; }
  return { dx, dy, nx: dx / max, ny: -dy / max };   // ny: screen-Y inverted
}

joyArea.addEventListener('touchstart', e => {
  e.preventDefault();
  if (joyActive) return;
  const t = e.changedTouches[0];
  joyTouchId = t.identifier;
  joyActive  = true;
  joyThumb.classList.add('active');
  const p = getJoyOffset(t);
  joyX = p.nx; joyY = p.ny;
  joyThumb.style.transform = `translate(calc(-50% + ${p.dx}px), calc(-50% + ${p.dy}px))`;
}, { passive: false });

joyArea.addEventListener('touchmove', e => {
  e.preventDefault();
  for (const t of e.changedTouches) {
    if (t.identifier !== joyTouchId) continue;
    const p = getJoyOffset(t);
    joyX = p.nx; joyY = p.ny;
    joyThumb.style.transform = `translate(calc(-50% + ${p.dx}px), calc(-50% + ${p.dy}px))`;
  }
}, { passive: false });

joyArea.addEventListener('touchend', e => {
  for (const t of e.changedTouches) {
    if (t.identifier !== joyTouchId) continue;
    joyActive = false; joyTouchId = null;
    joyX = 0; joyY = 0;
    joyThumb.classList.remove('active');
    joyThumb.style.transform = 'translate(-50%, -50%)';
    sendDrive();   // zero out immediately on release
  }
}, { passive: false });

// ── YAW SLIDER ────────────────────────────────────────────────────
const yawTrack = document.getElementById('yawTrack');
const yawThumb = document.getElementById('yawThumb');

yawTrack.addEventListener('touchstart', e => {
  e.preventDefault();
  if (yawActive) return;
  yawActive   = true;
  yawTouchId  = e.changedTouches[0].identifier;
  updateYaw(e.changedTouches[0]);
}, { passive: false });

yawTrack.addEventListener('touchmove', e => {
  e.preventDefault();
  for (const t of e.changedTouches) {
    if (t.identifier === yawTouchId) updateYaw(t);
  }
}, { passive: false });

yawTrack.addEventListener('touchend', e => {
  for (const t of e.changedTouches) {
    if (t.identifier !== yawTouchId) continue;
    yawActive = false; yawTouchId = null; yawVal = 0;
    yawThumb.style.top = '50%';
    sendDrive();   // zero on release
  }
}, { passive: false });

function updateYaw(touch) {
  const r   = yawTrack.getBoundingClientRect();
  const pct = Math.max(0, Math.min(1, (touch.clientY - r.top) / r.height));
  yawThumb.style.top = (pct * 100) + '%';
  yawVal = -(pct - 0.5) * 2;   // top=CCW(+1), centre=0, bottom=CW(-1)
}

// ── DRIVE SEND LOOP (20 Hz while input active) ───────────────────
function applyDead(v) { return Math.abs(v) < DEADZONE ? 0 : v; }

function sendDrive() {
  const m  = SPEEDS[speedIdx];
  const vx = applyDead(joyY)  * m * MAX_LINEAR;
  const vy = applyDead(-joyX) * m * MAX_LINEAR;   // right strafe = negative screen-X
  const wz = applyDead(yawVal)* m * MAX_ANGULAR;
  send({ type: 'drive', vx: +vx.toFixed(3), vy: +vy.toFixed(3), wz: +wz.toFixed(3) });
}

setInterval(() => {
  if ((joyActive || yawActive) && !estopped) sendDrive();
}, 50);   // 20 Hz

// ── ARM BUTTONS (hold to move, release stops) ─────────────────────
function armHold(cmd) {
  send({ type: 'arm', cmd });
}

function armRelease() {
  send({ type: 'arm', cmd: 'STOP' });
}

function bindHold(id, cmd) {
  const btn = document.getElementById(id);
  btn.addEventListener('touchstart', e => {
    e.preventDefault();
    btn.classList.add('held');
    armHold(cmd);
    armInterval = setInterval(() => armHold(cmd), 200);
  }, { passive: false });

  const stop = () => {
    btn.classList.remove('held');
    if (armInterval) { clearInterval(armInterval); armInterval = null; }
    armRelease();
  };
  btn.addEventListener('touchend',   stop, { passive: false });
  btn.addEventListener('touchcancel',stop, { passive: false });
}

bindHold('btnOpen',  'OPEN');
bindHold('btnClose', 'CLOSE');
bindHold('btnUp',    'LIFT');
bindHold('btnDown',  'LOWER');

// ── RECORD RUN ────────────────────────────────────────────────────
function toggleRecord() {
  if (estopped) return;
  recording = !recording;
  const btn   = document.getElementById('recBtn');
  const icon  = document.getElementById('recIcon');
  const label = document.getElementById('recLabel');
  const badge = document.getElementById('recBadge');

  if (recording) {
    send({ type: 'record_start' });
    btn.classList.add('recording');
    icon.textContent  = '⏹';
    label.textContent = 'STOP REC';
    badge.classList.add('show');
  } else {
    send({ type: 'record_stop' });
    btn.classList.remove('recording');
    icon.textContent  = '⏺';
    label.textContent = 'RECORD RUN';
    badge.classList.remove('show');
  }
}

// ── E-STOP ────────────────────────────────────────────────────────
const estopBtn    = document.getElementById('estopBtn');
const estopCircle = document.getElementById('estopCircle');
const estopLbl    = document.getElementById('estopLbl');
const flash       = document.getElementById('flash');

estopBtn.addEventListener('touchstart', e => {
  e.preventDefault();
  e.stopPropagation();
  if (!estopped) {
    estopped = true;
    send({ type: 'estop' });
    // Force joystick/yaw to zero
    joyX = 0; joyY = 0; yawVal = 0;
    joyActive = false; yawActive = false;
    joyThumb.style.transform = 'translate(-50%,-50%)';
    yawThumb.style.top = '50%';
    // Stop recording if active
    if (recording) { recording = false; send({ type: 'record_stop' }); }
    estopBtn.classList.add('armed');
    estopCircle.textContent = 'CLEAR';
    estopLbl.textContent    = 'TAP TO RESUME';
    flash.classList.add('show');
    setTimeout(() => flash.classList.remove('show'), 400);
  } else {
    estopped = false;
    send({ type: 'estop_clear' });
    estopBtn.classList.remove('armed');
    estopCircle.textContent = 'E-STOP';
    estopLbl.textContent    = 'TAP TO STOP';
  }
}, { passive: false });
</script>
</body>
</html>
"""

# ═══════════════════════════════════════════════════════════════════
#  ROS2 NODE
# ═══════════════════════════════════════════════════════════════════

class PhoneDashboard(Node):

    def __init__(self):
        super().__init__('phone_dashboard')

        # Parameters
        self.declare_parameter('port',    8080)
        self.declare_parameter('log_dir', '~/aislebot_logs')

        self.port    = self.get_parameter('port').value
        self.log_dir = os.path.expanduser(
            self.get_parameter('log_dir').get_parameter_value().string_value
        )
        os.makedirs(self.log_dir, exist_ok=True)

        # ── Publishers ────────────────────────────────────────────
        self.cmd_vel_pub  = self.create_publisher(Twist,             '/cmd_vel',       10)
        self.arm_pub      = self.create_publisher(String,            '/arm/command',   10)
        self.esp32_cmd_pub= self.create_publisher(String,            '/esp32/command', 10)

        # ── Subscribers ───────────────────────────────────────────
        # Receives [FR_t,FR_a,FR_p, FL_t,FL_a,FL_p, RR_t,RR_a,RR_p, RL_t,RL_a,RL_p]
        # Published by esp32_bridge when telemetry is enabled.
        self.create_subscription(
            Float64MultiArray, '/motor_telemetry',
            self._telemetry_callback, 10
        )

        # ── Telemetry / Recording ─────────────────────────────────
        self.recording     = False
        self._csv_file     = None
        self._csv_writer   = None
        self._sample_count = 0
        self._run_path     = ''

        # ── WebSocket client set (touched only from asyncio loop) ─
        self.ws_clients: Set[WebSocket] = set()

        self.get_logger().info(f'Phone Dashboard v2.0 — port {self.port}')
        self.get_logger().info(f'Log directory: {self.log_dir}')

    # ── Drive ─────────────────────────────────────────────────────

    def publish_drive(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    # ── Arm ───────────────────────────────────────────────────────

    def publish_arm(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.arm_pub.publish(msg)

    # ── Raw ESP32 serial command (forwarded by esp32_bridge) ──────

    def send_esp32_raw(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.esp32_cmd_pub.publish(msg)

    # ── Record run: start ─────────────────────────────────────────

    def start_recording(self) -> str:
        if self.recording:
            return self._run_path
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._run_path = os.path.join(self.log_dir, f'run_{ts}.csv')
        self._csv_file   = open(self._run_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'pi_time_s',
            'FR_target_rads', 'FR_actual_rads', 'FR_pwm',
            'FL_target_rads', 'FL_actual_rads', 'FL_pwm',
            'RR_target_rads', 'RR_actual_rads', 'RR_pwm',
            'RL_target_rads', 'RL_actual_rads', 'RL_pwm',
        ])
        self._sample_count = 0
        self.recording = True
        self.send_esp32_raw('<L1>')   # Tell ESP32 to start sending telemetry CSV
        self.get_logger().info(f'Recording started → {self._run_path}')
        return self._run_path

    # ── Record run: stop ──────────────────────────────────────────

    def stop_recording(self):
        if not self.recording:
            return
        self.recording = False
        self.send_esp32_raw('<L0>')   # Tell ESP32 to stop telemetry
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file   = None
            self._csv_writer = None
        self.get_logger().info(
            f'Recording stopped — {self._sample_count} samples → {self._run_path}'
        )

    # ── Telemetry callback ────────────────────────────────────────

    def _telemetry_callback(self, msg: Float64MultiArray):
        """Write one CSV row per incoming telemetry message."""
        if not self.recording or self._csv_writer is None:
            return
        data = list(msg.data)
        if len(data) < 12:
            return
        row = [f'{time.time():.4f}'] + [f'{v:.4f}' for v in data[:12]]
        self._csv_writer.writerow(row)
        self._sample_count += 1
        # Flush to disk every 50 samples (~5 s at 10 Hz)
        if self._sample_count % 50 == 0:
            self._csv_file.flush()


# ═══════════════════════════════════════════════════════════════════
#  FASTAPI APP
# ═══════════════════════════════════════════════════════════════════

app  = FastAPI()
_node: Optional[PhoneDashboard] = None


@app.get('/')
async def index():
    return HTMLResponse(DASHBOARD_HTML)


@app.websocket('/ws')
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    if _node:
        _node.ws_clients.add(websocket)
    try:
        while True:
            raw = await websocket.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            _dispatch(msg)
    except (WebSocketDisconnect, Exception):
        pass
    finally:
        if _node:
            _node.ws_clients.discard(websocket)


def _dispatch(msg: dict):
    """Route incoming WebSocket messages to the ROS2 node."""
    if _node is None:
        return

    t = msg.get('type', '')

    if t == 'drive':
        _node.publish_drive(
            msg.get('vx', 0.0),
            msg.get('vy', 0.0),
            msg.get('wz', 0.0),
        )

    elif t == 'arm':
        cmd = msg.get('cmd', '')
        if cmd:
            _node.publish_arm(cmd)

    elif t == 'record_start':
        path = _node.start_recording()
        _node.get_logger().info(f'Dashboard: record start → {path}')

    elif t == 'record_stop':
        _node.stop_recording()

    elif t == 'estop':
        # Zero drive immediately, latch E-STOP on ESP32
        _node.publish_drive(0.0, 0.0, 0.0)
        _node.send_esp32_raw('<S>')
        _node.publish_arm('ESTOP')
        _node.stop_recording()

    elif t == 'estop_clear':
        _node.send_esp32_raw('<E1>')


# ═══════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════

def main(args=None):
    global _node

    rclpy.init(args=args)
    _node = PhoneDashboard()

    # ROS2 spin in a background daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    ros_thread.start()

    # uvicorn runs in the main thread (blocks until Ctrl-C)
    uvicorn.run(
        app,
        host     = '0.0.0.0',
        port     = _node.port,
        log_level= 'warning',
    )

    _node.stop_recording()   # Ensure CSV is closed cleanly on shutdown
    _node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()