#!/usr/bin/env python3
"""
web_node.py
===========
Hosts a web dashboard on port 80 (configurable) that provides full control
of the robot arm over the local network.

Features:
  - Servo sliders for all 4 channels with live angle readout
  - RGB LED colour picker
  - Node status panel (polls /rosout and node graph)
  - WebSocket for real-time state push from ROS to browser

Backend:  FastAPI + uvicorn (runs in its own thread alongside ROS2 spin)
Frontend: Single-page HTML/CSS/JS served inline — no build step, no files

Install dependencies:
    pip install fastapi uvicorn

ROS params:
  host   (str,  default '0.0.0.0')  bind address
  port   (int,  default 80)         HTTP port (80 needs sudo or authbind)
"""

import asyncio
import json
import threading
from typing import Set

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from robot_arm_bringup.msg import ServoCmd, RgbLed

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
    WEB_AVAILABLE = True
except ImportError:
    WEB_AVAILABLE = False

# ---------------------------------------------------------------------------
# HTML dashboard (self-contained, no CDN required)
# ---------------------------------------------------------------------------

DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Robot Arm</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Barlow:wght@300;500;700&display=swap" rel="stylesheet">
<style>
  :root {
    --bg:       #0a0c0f;
    --surface:  #111418;
    --border:   #1e2530;
    --accent:   #00e5ff;
    --accent2:  #ff6b35;
    --dim:      #3a4555;
    --text:     #c8d8e8;
    --textdim:  #556070;
    --mono:     'Share Tech Mono', monospace;
    --sans:     'Barlow', sans-serif;
    --radius:   6px;
  }
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  html, body { height: 100%; background: var(--bg); color: var(--text); font-family: var(--sans); font-weight: 300; }

  /* scanline overlay */
  body::before {
    content: '';
    position: fixed; inset: 0; pointer-events: none; z-index: 999;
    background: repeating-linear-gradient(0deg, transparent, transparent 2px, rgba(0,0,0,0.06) 2px, rgba(0,0,0,0.06) 4px);
  }

  header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 18px 28px; border-bottom: 1px solid var(--border);
    background: var(--surface);
  }
  header h1 {
    font-family: var(--mono); font-size: 18px; letter-spacing: 3px;
    color: var(--accent); text-transform: uppercase;
  }
  header h1 span { color: var(--textdim); }

  #ws-status {
    font-family: var(--mono); font-size: 11px; letter-spacing: 1px;
    display: flex; align-items: center; gap: 8px; color: var(--textdim);
  }
  #ws-dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: var(--dim); transition: background 0.3s;
  }
  #ws-dot.live { background: var(--accent); box-shadow: 0 0 6px var(--accent); }
  #ws-dot.err  { background: var(--accent2); }

  main {
    display: grid;
    grid-template-columns: 1fr 320px;
    grid-template-rows: auto auto;
    gap: 1px;
    background: var(--border);
    height: calc(100vh - 57px);
  }

  .panel {
    background: var(--surface);
    padding: 24px 28px;
    overflow-y: auto;
  }

  .panel-label {
    font-family: var(--mono); font-size: 10px; letter-spacing: 3px;
    color: var(--textdim); text-transform: uppercase;
    margin-bottom: 20px; padding-bottom: 10px;
    border-bottom: 1px solid var(--border);
  }

  /* ── Servos ── */
  #panel-servos { grid-row: 1 / 3; }

  .servo-row {
    display: grid; grid-template-columns: 90px 1fr 56px;
    align-items: center; gap: 16px;
    padding: 18px 0; border-bottom: 1px solid var(--border);
    opacity: 0; transform: translateX(-12px);
    animation: slidein 0.4s forwards;
  }
  .servo-row:last-child { border-bottom: none; }
  .servo-row:nth-child(2) { animation-delay: 0.05s; }
  .servo-row:nth-child(3) { animation-delay: 0.10s; }
  .servo-row:nth-child(4) { animation-delay: 0.15s; }
  .servo-row:nth-child(5) { animation-delay: 0.20s; }

  @keyframes slidein {
    to { opacity: 1; transform: translateX(0); }
  }

  .servo-name {
    font-family: var(--mono); font-size: 12px; letter-spacing: 1px;
    color: var(--accent); text-transform: uppercase;
  }
  .servo-sub {
    font-size: 11px; color: var(--textdim); margin-top: 3px;
  }

  input[type=range] {
    -webkit-appearance: none; width: 100%; height: 3px;
    background: var(--dim); border-radius: 2px; outline: none;
    cursor: pointer; transition: background 0.2s;
  }
  input[type=range]:hover { background: #2a3545; }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none; width: 16px; height: 16px;
    border-radius: 50%; background: var(--accent);
    box-shadow: 0 0 8px rgba(0,229,255,0.5);
    cursor: pointer; transition: transform 0.15s;
  }
  input[type=range]:active::-webkit-slider-thumb { transform: scale(1.3); }

  .servo-val {
    font-family: var(--mono); font-size: 20px; font-weight: 500;
    color: var(--text); text-align: right;
  }
  .servo-val span { font-size: 11px; color: var(--textdim); }

  #send-all {
    margin-top: 28px; width: 100%;
    background: transparent; border: 1px solid var(--accent);
    color: var(--accent); font-family: var(--mono); font-size: 12px;
    letter-spacing: 3px; text-transform: uppercase;
    padding: 14px; border-radius: var(--radius);
    cursor: pointer; transition: all 0.2s;
  }
  #send-all:hover { background: var(--accent); color: var(--bg); }
  #send-all:active { transform: scale(0.98); }
  #send-all.sent { border-color: #00ff88; color: #00ff88; }

  /* ── LED ── */
  #panel-led { grid-column: 2; }

  .led-preview {
    width: 100%; height: 80px; border-radius: var(--radius);
    background: #000; margin-bottom: 20px;
    border: 1px solid var(--border);
    transition: background 0.1s;
    display: flex; align-items: center; justify-content: center;
    font-family: var(--mono); font-size: 11px; color: rgba(255,255,255,0.3);
  }

  .channel-row {
    display: grid; grid-template-columns: 24px 1fr 40px;
    align-items: center; gap: 12px; margin-bottom: 14px;
  }
  .ch-label {
    font-family: var(--mono); font-size: 13px; font-weight: 500;
  }
  .ch-r { color: #ff5555; }
  .ch-g { color: #50fa7b; }
  .ch-b { color: #6ba3ff; }

  input[type=range].r::-webkit-slider-thumb { background: #ff5555; box-shadow: 0 0 8px rgba(255,85,85,0.5); }
  input[type=range].g::-webkit-slider-thumb { background: #50fa7b; box-shadow: 0 0 8px rgba(80,250,123,0.5); }
  input[type=range].b::-webkit-slider-thumb { background: #6ba3ff; box-shadow: 0 0 8px rgba(107,163,255,0.5); }

  .ch-val {
    font-family: var(--mono); font-size: 13px;
    color: var(--textdim); text-align: right;
  }

  #led-presets {
    display: grid; grid-template-columns: repeat(5, 1fr); gap: 8px;
    margin: 18px 0;
  }
  .preset {
    height: 28px; border-radius: 4px; border: 1px solid transparent;
    cursor: pointer; transition: transform 0.15s, border-color 0.15s;
  }
  .preset:hover { transform: scale(1.1); border-color: rgba(255,255,255,0.3); }

  #send-led {
    width: 100%; background: transparent;
    border: 1px solid var(--accent2); color: var(--accent2);
    font-family: var(--mono); font-size: 11px; letter-spacing: 2px;
    text-transform: uppercase; padding: 12px; border-radius: var(--radius);
    cursor: pointer; transition: all 0.2s;
  }
  #send-led:hover { background: var(--accent2); color: var(--bg); }
  #send-led.sent { border-color: #00ff88; color: #00ff88; }

  /* ── Node status ── */
  #panel-status { grid-column: 2; }

  .node-row {
    display: flex; align-items: center; justify-content: space-between;
    padding: 10px 0; border-bottom: 1px solid var(--border);
    font-family: var(--mono); font-size: 11px;
  }
  .node-row:last-child { border-bottom: none; }
  .node-name { color: var(--textdim); letter-spacing: 1px; }
  .node-badge {
    padding: 3px 8px; border-radius: 3px; font-size: 10px;
    letter-spacing: 1px; text-transform: uppercase;
  }
  .node-badge.up   { background: rgba(0,255,136,0.1); color: #00ff88; border: 1px solid rgba(0,255,136,0.3); }
  .node-badge.down { background: rgba(255,107,53,0.1); color: var(--accent2); border: 1px solid rgba(255,107,53,0.3); }
</style>
</head>
<body>

<header>
  <h1>ROBOT<span>/</span>ARM</h1>
  <div id="ws-status">
    <div id="ws-dot"></div>
    <span id="ws-label">CONNECTING</span>
  </div>
</header>

<main>

  <!-- Servo panel -->
  <div class="panel" id="panel-servos">
    <div class="panel-label">Servo channels</div>

    <div class="servo-row">
      <div><div class="servo-name">Base</div><div class="servo-sub">CH 0 — rotation</div></div>
      <input type="range" min="0" max="180" value="90" id="s0" oninput="updateServo(0,this.value)">
      <div class="servo-val" id="v0">90<span>°</span></div>
    </div>
    <div class="servo-row">
      <div><div class="servo-name">Shoulder</div><div class="servo-sub">CH 1 — lift</div></div>
      <input type="range" min="0" max="180" value="90" id="s1" oninput="updateServo(1,this.value)">
      <div class="servo-val" id="v1">90<span>°</span></div>
    </div>
    <div class="servo-row">
      <div><div class="servo-name">Elbow</div><div class="servo-sub">CH 2 — reach</div></div>
      <input type="range" min="0" max="180" value="90" id="s2" oninput="updateServo(2,this.value)">
      <div class="servo-val" id="v2">90<span>°</span></div>
    </div>
    <div class="servo-row">
      <div><div class="servo-name">Gripper</div><div class="servo-sub">CH 3 — grip</div></div>
      <input type="range" min="0" max="180" value="90" id="s3" oninput="updateServo(3,this.value)">
      <div class="servo-val" id="v3">90<span>°</span></div>
    </div>

    <button id="send-all" onclick="sendServos()">Send all angles</button>
  </div>

  <!-- LED panel -->
  <div class="panel" id="panel-led">
    <div class="panel-label">RGB LED</div>
    <div class="led-preview" id="led-preview">●</div>

    <div class="channel-row">
      <div class="ch-label ch-r">R</div>
      <input type="range" class="r" min="0" max="255" value="0" id="lr" oninput="updateLed()">
      <div class="ch-val" id="vr">0</div>
    </div>
    <div class="channel-row">
      <div class="ch-label ch-g">G</div>
      <input type="range" class="g" min="0" max="255" value="0" id="lg" oninput="updateLed()">
      <div class="ch-val" id="vg">0</div>
    </div>
    <div class="channel-row">
      <div class="ch-label ch-b">B</div>
      <input type="range" class="b" min="0" max="255" value="0" id="lb" oninput="updateLed()">
      <div class="ch-val" id="vb">0</div>
    </div>

    <div id="led-presets">
      <div class="preset" style="background:#ff2020" onclick="setPreset(255,32,32)" title="Red"></div>
      <div class="preset" style="background:#20ff60" onclick="setPreset(32,255,96)" title="Green"></div>
      <div class="preset" style="background:#2060ff" onclick="setPreset(32,96,255)" title="Blue"></div>
      <div class="preset" style="background:#ff8c00" onclick="setPreset(255,140,0)"  title="Amber"></div>
      <div class="preset" style="background:#ffffff" onclick="setPreset(255,255,255)" title="White"></div>
    </div>

    <button id="send-led" onclick="sendLed()">Send colour</button>
  </div>

  <!-- Node status panel -->
  <div class="panel" id="panel-status">
    <div class="panel-label">Node status</div>
    <div id="node-list">
      <div class="node-row"><span class="node-name">/servo_node</span><span class="node-badge down" id="ns-servo">—</span></div>
      <div class="node-row"><span class="node-name">/rgb_led_node</span><span class="node-badge down" id="ns-led">—</span></div>
      <div class="node-row"><span class="node-name">/bt_node</span><span class="node-badge down" id="ns-bt">—</span></div>
      <div class="node-row"><span class="node-name">/sound_node</span><span class="node-badge down" id="ns-sound">—</span></div>
      <div class="node-row"><span class="node-name">/web_node</span><span class="node-badge up" id="ns-web">UP</span></div>
    </div>
  </div>

</main>

<script>
  const angles = [90, 90, 90, 90];

  function updateServo(ch, val) {
    angles[ch] = parseInt(val);
    document.getElementById('v' + ch).innerHTML = val + '<span>°</span>';
  }

  function sendServos() {
    fetch('/api/servos', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({angles: angles, speeds: [1,1,1,1]})
    }).then(r => {
      const btn = document.getElementById('send-all');
      btn.classList.add('sent');
      btn.textContent = 'Sent ✓';
      setTimeout(() => { btn.classList.remove('sent'); btn.textContent = 'Send all angles'; }, 1200);
    });
  }

  function updateLed() {
    const r = document.getElementById('lr').value;
    const g = document.getElementById('lg').value;
    const b = document.getElementById('lb').value;
    document.getElementById('vr').textContent = r;
    document.getElementById('vg').textContent = g;
    document.getElementById('vb').textContent = b;
    document.getElementById('led-preview').style.background =
      `rgb(${r},${g},${b})`;
  }

  function setPreset(r, g, b) {
    document.getElementById('lr').value = r;
    document.getElementById('lg').value = g;
    document.getElementById('lb').value = b;
    updateLed();
  }

  function sendLed() {
    const r = parseInt(document.getElementById('lr').value);
    const g = parseInt(document.getElementById('lg').value);
    const b = parseInt(document.getElementById('lb').value);
    fetch('/api/led', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({r, g, b})
    }).then(() => {
      const btn = document.getElementById('send-led');
      btn.classList.add('sent');
      btn.textContent = 'Sent ✓';
      setTimeout(() => { btn.classList.remove('sent'); btn.textContent = 'Send colour'; }, 1200);
    });
  }

  // WebSocket for live node status
  const nodeMap = {
    '/servo_node':   'ns-servo',
    '/rgb_led_node': 'ns-led',
    '/bt_node':      'ns-bt',
    '/sound_node':   'ns-sound',
  };

  function connectWS() {
    const ws = new WebSocket(`ws://${location.host}/ws`);
    const dot = document.getElementById('ws-dot');
    const lbl = document.getElementById('ws-label');

    ws.onopen = () => {
      dot.className = 'live';
      lbl.textContent = 'LIVE';
    };
    ws.onmessage = (e) => {
      const data = JSON.parse(e.data);
      if (data.nodes) {
        for (const [name, elId] of Object.entries(nodeMap)) {
          const el = document.getElementById(elId);
          const up = data.nodes.includes(name);
          el.className = 'node-badge ' + (up ? 'up' : 'down');
          el.textContent = up ? 'UP' : 'DOWN';
        }
      }
      if (data.servos) {
        data.servos.forEach((angle, i) => {
          document.getElementById('s' + i).value = angle;
          document.getElementById('v' + i).innerHTML = Math.round(angle) + '<span>°</span>';
          angles[i] = angle;
        });
      }
    };
    ws.onclose = () => {
      dot.className = 'err';
      lbl.textContent = 'DISCONNECTED';
      setTimeout(connectWS, 3000);
    };
    ws.onerror = () => ws.close();
  }

  connectWS();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class WebNode(Node):

    def __init__(self):
        super().__init__('web_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 80)

        self._host = self.get_parameter('host').value
        self._port = self.get_parameter('port').value

        # Publishers
        self._servo_pub = self.create_publisher(ServoCmd, '/servo_cmd', 10)
        self._led_pub   = self.create_publisher(RgbLed,   '/rgb_led',   10)

        # Current state (mirrored for WS push)
        self._servo_positions = [90.0, 90.0, 90.0, 90.0]

        # Connected WebSocket clients
        self._ws_clients: Set[WebSocket] = set()
        self._ws_lock = threading.Lock()

        # Build FastAPI app
        if WEB_AVAILABLE:
            self._app = self._build_app()
            self._start_server()
        else:
            self.get_logger().error(
                'fastapi / uvicorn not installed.\n'
                'Run: pip install fastapi uvicorn')

        # Push node status over WS every 2 seconds
        self.create_timer(2.0, self._push_status)

        self.get_logger().info(
            f'web_node ready — dashboard at http://<PI_IP>:{self._port}')

    # ------------------------------------------------------------------
    # FastAPI app
    # ------------------------------------------------------------------

    def _build_app(self) -> 'FastAPI':
        app = FastAPI()

        @app.get('/', response_class=HTMLResponse)
        async def dashboard():
            return DASHBOARD_HTML

        @app.post('/api/servos')
        async def set_servos(body: dict):
            angles = body.get('angles', [-1.0] * 4)
            speeds = body.get('speeds', [1.0] * 4)
            msg = ServoCmd()
            msg.angles = [float(a) for a in angles]
            msg.speeds = [float(s) for s in speeds]
            self._servo_pub.publish(msg)
            for i, a in enumerate(msg.angles):
                if a >= 0:
                    self._servo_positions[i] = a
            return {'ok': True}

        @app.post('/api/led')
        async def set_led(body: dict):
            msg = RgbLed()
            msg.r = int(max(0, min(255, body.get('r', 0))))
            msg.g = int(max(0, min(255, body.get('g', 0))))
            msg.b = int(max(0, min(255, body.get('b', 0))))
            self._led_pub.publish(msg)
            return {'ok': True}

        @app.websocket('/ws')
        async def ws_endpoint(websocket: WebSocket):
            await websocket.accept()
            with self._ws_lock:
                self._ws_clients.add(websocket)
            try:
                while True:
                    await websocket.receive_text()  # keep alive
            except WebSocketDisconnect:
                pass
            finally:
                with self._ws_lock:
                    self._ws_clients.discard(websocket)

        return app

    def _start_server(self):
        def run():
            config = uvicorn.Config(
                self._app,
                host=self._host,
                port=self._port,
                log_level='warning',
            )
            server = uvicorn.Server(config)
            asyncio.run(server.serve())

        t = threading.Thread(target=run, daemon=True, name='web_server')
        t.start()

    # ------------------------------------------------------------------
    # WebSocket status push
    # ------------------------------------------------------------------

    def _push_status(self):
        nodes = self.get_node_names_and_namespaces()
        node_list = [
            (ns.rstrip('/') + '/' + name).replace('//', '/')
            for name, ns in nodes
        ]
        payload = json.dumps({
            'nodes':  node_list,
            'servos': self._servo_positions,
        })
        with self._ws_lock:
            dead = set()
            for ws in self._ws_clients:
                try:
                    asyncio.run_coroutine_threadsafe(
                        ws.send_text(payload),
                        asyncio.get_event_loop(),
                    )
                except Exception:
                    dead.add(ws)
            self._ws_clients -= dead


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
