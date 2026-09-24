from __future__ import annotations

import json
import logging
import math
import secrets
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from typing import Any

import numpy as np
from PIL import Image

from .simulator import MH3Simulation

logger = logging.getLogger(__name__)

_LEFT_WHEEL = "left_wheel_joint"
_RIGHT_WHEEL = "right_wheel_joint"
_BASE_INTERNAL_JOINTS = frozenset(
    {
        _LEFT_WHEEL,
        _RIGHT_WHEEL,
        "left_castor_joint",
        "left_castor_wheel_joint",
        "right_castor_joint",
        "right_castor_wheel_joint",
    }
)


class MH3ControlPanel:
    """Local browser controls whose commands are applied on the simulation thread."""

    def __init__(self, simulation: MH3Simulation) -> None:
        self.simulation = simulation
        self._lock = threading.Lock()
        self._token = secrets.token_urlsafe(24)
        self._pending_positions: dict[str, float] = {}
        self._drive = (0.0, 0.0)
        self._drive_changed = False
        self._drive_active = False
        self._drive_received_at = 0.0
        self._last_snapshot_at = 0.0
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._camera_pairs = {
            source: (f"{source}_left", f"{source}_right")
            for source in ("head", "base")
            if {f"{source}_left", f"{source}_right"}.issubset(simulation.camera_names)
        }
        self._camera_frames: dict[str, bytes] = {}
        self._last_camera_request_at = 0.0
        self._last_camera_render_at = 0.0
        self._camera_error: str | None = None

        self._joint_names = tuple(name for name in simulation.joint_names if name not in _BASE_INTERNAL_JOINTS)
        state = simulation.get_joint_state(self._joint_names)
        lower, upper = simulation.get_joint_limits(state.names)
        limits: dict[str, tuple[float, float]] = {}
        for name, low, high in zip(state.names, lower, upper, strict=True):
            fallback = 0.5 if name == "slider" else math.pi
            low = float(low) if math.isfinite(low) else -fallback
            high = float(high) if math.isfinite(high) else fallback
            limits[name] = (low, high)
        self._limits = limits
        self._home = {
            name: float(simulation.config.initial_positions.get(name, value))
            for name, value in zip(state.names, state.positions, strict=True)
        }
        self._positions = {name: float(value) for name, value in zip(state.names, state.positions, strict=True)}

    @property
    def url(self) -> str:
        if self._server is None:
            raise RuntimeError("Control panel has not been started")
        return f"http://127.0.0.1:{self._server.server_port}/"

    def start(self, *, open_browser: bool = True) -> None:
        if self._server is not None:
            return
        panel = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path == "/":
                    self._send(
                        _html(
                            panel._token,
                            panel.simulation.config.max_linear_velocity_mps,
                            panel.simulation.config.max_angular_velocity_radps,
                        ),
                        "text/html; charset=utf-8",
                    )
                elif self.path == "/api/state":
                    self._send(json.dumps(panel.state()), "application/json")
                elif self.path.startswith("/api/camera/"):
                    name = self.path.removeprefix("/api/camera/")
                    if name not in panel._camera_feed_names:
                        self.send_error(HTTPStatus.NOT_FOUND)
                        return
                    frame = panel.camera_frame(name)
                    if frame is None:
                        self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "Camera frame is not ready")
                        return
                    self._send(frame, "image/png" if name.endswith("-depth") else "image/jpeg")
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self) -> None:
                if self.headers.get("X-Control-Token") != panel._token:
                    self.send_error(HTTPStatus.FORBIDDEN)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(length) or b"{}")
                    if self.path == "/api/joints":
                        panel.queue_positions(body.get("targets", {}))
                    elif self.path == "/api/drive":
                        panel.queue_drive(body.get("linear", 0.0), body.get("angular", 0.0))
                    elif self.path == "/api/home":
                        panel.queue_home()
                    else:
                        self.send_error(HTTPStatus.NOT_FOUND)
                        return
                except (TypeError, ValueError, KeyError) as error:
                    self.send_error(HTTPStatus.BAD_REQUEST, str(error))
                    return
                self._send("{}", "application/json")

            def _send(self, body: str | bytes, content_type: str) -> None:
                encoded = body.encode() if isinstance(body, str) else body
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(encoded)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(encoded)

            def log_message(self, format: str, *args: Any) -> None:
                return

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, name="mh3-controls", daemon=True)
        self._thread.start()
        print(f"MH3 control panel: {self.url}")
        if open_browser:
            webbrowser.open(self.url)

    def close(self) -> None:
        if self._server is None:
            return
        self.queue_drive(0.0, 0.0)
        self._server.shutdown()
        self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._server = None
        self._thread = None

    def state(self) -> dict[str, Any]:
        with self._lock:
            return {
                "joints": [
                    {
                        "name": name,
                        "position": self._positions[name],
                        "home": self._home[name],
                        "minimum": self._limits[name][0],
                        "maximum": self._limits[name][1],
                    }
                    for name in self._joint_names
                ],
                "drive": {"linear": self._drive[0], "angular": self._drive[1]},
                "sim_time": self.simulation.sim_time,
                "camera_sources": list(self._camera_pairs),
                "camera_error": self._camera_error,
            }

    @property
    def _camera_feed_names(self) -> set[str]:
        return {f"{source}-{feed}" for source in self._camera_pairs for feed in ("left-rgb", "right-rgb", "depth")}

    def camera_frame(self, name: str) -> bytes | None:
        if name not in self._camera_feed_names:
            raise KeyError(f"Unknown camera feed: {name}")
        with self._lock:
            self._last_camera_request_at = time.monotonic()
            return self._camera_frames.get(name)

    @staticmethod
    def _encode_rgb(rgb: np.ndarray) -> bytes:
        output = BytesIO()
        Image.fromarray(np.asarray(rgb, dtype=np.uint8)).save(output, format="JPEG", quality=80)
        return output.getvalue()

    @staticmethod
    def _encode_depth(depth: np.ndarray, near_m: float, far_m: float) -> bytes:
        values = np.asarray(depth)
        display_far = min(far_m, 5.0)
        valid = np.isfinite(values) & (values > 0.0) & (values < far_m)
        grayscale = np.zeros(values.shape, dtype=np.uint8)
        grayscale[valid] = np.clip((display_far - values[valid]) / (display_far - near_m) * 255.0, 0.0, 255.0).astype(
            np.uint8
        )
        output = BytesIO()
        Image.fromarray(grayscale).save(output, format="PNG")
        return output.getvalue()

    def _render_camera_frames(self, simulation: MH3Simulation) -> None:
        frames = {}
        for source, (left_name, right_name) in self._camera_pairs.items():
            left = simulation.capture_camera(left_name, rgb=True, depth=True)
            right = simulation.capture_camera(right_name, rgb=True, depth=False)
            if left.rgb is None or left.depth is None or right.rgb is None:
                raise RuntimeError(f"Camera pair {source!r} did not return RGB and depth frames")
            camera_config = next(camera for camera in simulation.config.cameras if camera.name == left_name)
            frames.update(
                {
                    f"{source}-left-rgb": self._encode_rgb(left.rgb),
                    f"{source}-right-rgb": self._encode_rgb(right.rgb),
                    f"{source}-depth": self._encode_depth(left.depth, camera_config.near_m, camera_config.far_m),
                }
            )
        with self._lock:
            self._camera_frames = frames
            self._camera_error = None

    def queue_positions(self, targets: dict[str, Any]) -> None:
        if not isinstance(targets, dict):
            raise TypeError("targets must be an object keyed by joint name")
        clean: dict[str, float] = {}
        for name, value in targets.items():
            if name not in self._limits:
                raise KeyError(f"Unknown joint: {name}")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"Joint target must be finite: {name}")
            low, high = self._limits[name]
            clean[name] = min(max(value, low), high)
        with self._lock:
            self._pending_positions.update(clean)

    def queue_drive(self, linear: Any, angular: Any) -> None:
        linear, angular = float(linear), float(angular)
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise ValueError("Drive commands must be finite")
        with self._lock:
            self._drive = (linear, angular)
            self._drive_changed = True
            self._drive_active = linear != 0.0 or angular != 0.0
            self._drive_received_at = time.monotonic()

    def queue_home(self) -> None:
        with self._lock:
            self._pending_positions.update(self._home)
        self.queue_drive(0.0, 0.0)

    def update(self, simulation: MH3Simulation) -> None:
        now = time.monotonic()
        with self._lock:
            positions = self._pending_positions
            self._pending_positions = {}
            drive = self._drive
            drive_changed = self._drive_changed
            self._drive_changed = False
            if self._drive_active and now - self._drive_received_at > simulation.config.drive_command_timeout_s:
                drive = (0.0, 0.0)
                self._drive = drive
                self._drive_active = False
                drive_changed = True
        if positions:
            simulation.set_joint_positions(positions)
        if drive_changed:
            linear, angular = drive
            simulation.drive_base(linear, angular)
        if now - self._last_snapshot_at >= 0.1:
            state = simulation.get_joint_state(self._joint_names)
            with self._lock:
                self._positions = {name: float(value) for name, value in zip(state.names, state.positions, strict=True)}
            self._last_snapshot_at = now
        with self._lock:
            camera_requested = now - self._last_camera_request_at < 2.0
        if self._camera_pairs and camera_requested and now - self._last_camera_render_at >= 0.4:
            try:
                self._render_camera_frames(simulation)
            except Exception as error:
                with self._lock:
                    changed = str(error) != self._camera_error
                    self._camera_error = str(error)
                if changed:
                    logger.exception("MH3 dashboard camera rendering failed")
            finally:
                self._last_camera_render_at = time.monotonic()


def _html(token: str, max_linear: float, max_angular: float) -> str:
    linear_step = max_linear / 12.0
    angular_step = max_angular / 20.0
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MH3 Controls</title><style>
:root{{--bg:#0b1118;--panel:#151e29;--line:#29384a;--text:#e9f0f7;--muted:#8fa2b7;--accent:#48b6ff;--danger:#ff5c6c}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--text);font:14px system-ui,sans-serif}}
header{{position:sticky;top:0;z-index:2;display:flex;align-items:center;gap:16px;padding:14px 20px;background:#0f1721;border-bottom:1px solid var(--line)}}
h1{{font-size:18px;margin:0}}#status{{color:var(--muted)}}main{{display:grid;grid-template-columns:300px 1fr;gap:16px;padding:16px}}
.card{{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:16px}}h2{{font-size:15px;margin:0 0 14px}}
.workspace{{display:grid;gap:16px;min-width:0}}.camera-row{{margin-top:16px}}.camera-row h3{{font-size:13px;margin:0 0 10px}}
.camera-grid{{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}}.camera-grid figure{{margin:0;min-width:0}}
.camera-grid img{{display:block;width:100%;aspect-ratio:16/9;object-fit:contain;background:#0e1620;border-radius:6px}}
.camera-grid figcaption{{margin-top:6px;color:var(--muted);font-size:12px}}
.drive-grid{{display:grid;grid-template-columns:repeat(3,64px);gap:8px;justify-content:center;margin:14px 0}}
button{{border:1px solid #3b526a;background:#223044;color:var(--text);border-radius:8px;padding:11px;cursor:pointer}}
button:hover{{background:#2b4057}}button.stop{{background:#4a2028;border-color:#883945}}button.home{{width:100%;margin-top:8px}}
.speed label{{display:grid;grid-template-columns:90px 1fr 48px;align-items:center;gap:8px;margin:8px 0;color:var(--muted)}}
#joint-groups{{display:grid;grid-template-columns:repeat(auto-fit,minmax(390px,1fr));gap:16px}}.joint-row{{display:grid;grid-template-columns:170px 1fr 72px;gap:10px;align-items:center;margin:8px 0}}
.joint-row span{{overflow:hidden;text-overflow:ellipsis;white-space:nowrap;color:#c8d5e3}}input[type=range]{{width:100%;accent-color:var(--accent)}}
input[type=number]{{width:72px;background:#0e1620;color:var(--text);border:1px solid var(--line);border-radius:6px;padding:5px}}
.hint{{color:var(--muted);font-size:12px;line-height:1.5}}@media(max-width:1000px){{.camera-grid{{grid-template-columns:1fr}}}}@media(max-width:800px){{main{{grid-template-columns:1fr}}#joint-groups{{grid-template-columns:1fr}}}}
</style></head><body><header><h1>MH3 Control Panel</h1><span id="status">Connecting…</span></header><main>
<section class="card"><h2>Base drive</h2><div class="speed">
<label>Linear<input id="linear" type="range" min="{linear_step}" max="{max_linear}" step="{linear_step}" value="{max_linear}"><output>{max_linear:.2f}</output></label>
<label>Angular<input id="angular" type="range" min="{angular_step}" max="{max_angular}" step="{angular_step}" value="{max_angular}"><output>{max_angular:.2f}</output></label></div>
<div class="drive-grid"><i></i><button data-drive="forward">▲</button><i></i><button data-drive="left">◀</button><button class="stop" data-drive="stop">■</button><button data-drive="right">▶</button><i></i><button data-drive="back">▼</button></div>
<button class="home" id="home">Return all joints home</button><p class="hint">Hold the buttons or W/A/S/D to drive. Releasing a control stops the base automatically.</p></section>
<div class="workspace"><section class="card" id="camera-panel"><h2>Camera feeds</h2>
<div class="camera-row" id="head-cameras"><h3>Head cameras</h3><div class="camera-grid"><figure><img id="head-left-rgb" alt="Head left RGB camera feed"><figcaption>Left RGB</figcaption></figure><figure><img id="head-right-rgb" alt="Head right RGB camera feed"><figcaption>Right RGB</figcaption></figure><figure><img id="head-depth" alt="Head depth camera feed"><figcaption>Left camera depth</figcaption></figure></div></div>
<div class="camera-row" id="base-cameras"><h3>Base cameras</h3><div class="camera-grid"><figure><img id="base-left-rgb" alt="Base left RGB camera feed"><figcaption>Left RGB</figcaption></figure><figure><img id="base-right-rgb" alt="Base right RGB camera feed"><figcaption>Right RGB</figcaption></figure><figure><img id="base-depth" alt="Base depth camera feed"><figcaption>Left camera depth</figcaption></figure></div></div><p class="hint" id="camera-status">Waiting for camera frames… Near depth is bright; beyond 5 m is dark.</p></section>
<section id="joint-groups"></section></div></main><script>
const TOKEN={json.dumps(token)};let built=false;let drive='stop';const controls=new Map();
const post=(path,body)=>fetch(path,{{method:'POST',headers:{{'Content-Type':'application/json','X-Control-Token':TOKEN}},body:JSON.stringify(body)}});
function group(name){{if(/^left_[1-7]$/.test(name))return'Left arm';if(/^right_[1-7]$/.test(name))return'Right arm';if(name.startsWith('head_'))return'Head';if(name.startsWith('left_'))return'Left hand';if(name.startsWith('right_'))return'Right hand';return'Other joints';}}
function build(joints){{const root=document.querySelector('#joint-groups');const groups={{}};for(const j of joints){{const g=group(j.name);if(!groups[g]){{const card=document.createElement('div');card.className='card';card.innerHTML=`<h2>${{g}}</h2>`;groups[g]=card;root.append(card)}}const row=document.createElement('label');row.className='joint-row';row.innerHTML=`<span title="${{j.name}}">${{j.name}}</span><input type="range" min="${{j.minimum}}" max="${{j.maximum}}" step="0.01" value="${{j.position}}"><input type="number" min="${{j.minimum}}" max="${{j.maximum}}" step="0.01" value="${{j.position.toFixed(2)}}">`;const slider=row.children[1],number=row.children[2],control={{slider,number,home:j.home,editing:false}};controls.set(j.name,control);let timer;const send=v=>{{v=Number(v);slider.value=v;number.value=v.toFixed(2);clearTimeout(timer);timer=setTimeout(()=>post('/api/joints',{{targets:{{[j.name]:v}}}}),35)}};slider.onpointerdown=()=>control.editing=true;slider.onpointerup=slider.onpointercancel=()=>control.editing=false;slider.oninput=()=>send(slider.value);number.onfocus=()=>control.editing=true;number.onblur=()=>control.editing=false;number.onchange=()=>send(number.value);groups[g].append(row)}}built=true}}
function sync(joints){{for(const j of joints){{const c=controls.get(j.name);if(c&&!c.editing){{c.slider.value=j.position;c.number.value=j.position.toFixed(2)}}}}}}
const cameraStatus=document.querySelector('#camera-status');let cameraSources=[],feedsBusy=false;
async function poll(){{try{{const s=await fetch('/api/state',{{cache:'no-store'}}).then(r=>r.json());if(!built)build(s.joints);else sync(s.joints);document.querySelector('#status').textContent=`Connected · ${{s.sim_time.toFixed(2)}} s`;cameraSources=s.camera_sources;for(const source of ['head','base'])document.getElementById(source+'-cameras').hidden=!cameraSources.includes(source);cameraStatus.textContent=s.camera_error||(!cameraSources.length?'No stereo camera pairs configured':'Live camera feeds · near depth bright, beyond 5 m dark');}}catch(e){{document.querySelector('#status').textContent='Disconnected';cameraStatus.textContent='Camera feeds disconnected'}}}}setInterval(poll,250);poll();
async function refreshCameraFeeds(){{if(document.hidden||!cameraSources.length||feedsBusy)return;feedsBusy=true;try{{const names=cameraSources.flatMap(source=>['left-rgb','right-rgb','depth'].map(feed=>source+'-'+feed));await Promise.all(names.map(async name=>{{const response=await fetch('/api/camera/'+name,{{cache:'no-store'}});if(!response.ok)return;const blob=await response.blob();const img=document.getElementById(name),previous=img.dataset.objectUrl;const url=URL.createObjectURL(blob);img.src=url;img.dataset.objectUrl=url;if(previous)URL.revokeObjectURL(previous)}}))}}catch(e){{cameraStatus.textContent='Waiting for camera frames…'}}finally{{feedsBusy=false}}}}setInterval(refreshCameraFeeds,500);
for(const id of ['linear','angular']){{const el=document.querySelector('#'+id);el.oninput=()=>el.nextElementSibling.value=Number(el.value).toFixed(2)}}
function command(kind){{drive=kind;const v=Number(document.querySelector('#linear').value),w=Number(document.querySelector('#angular').value);const map={{forward:[v,0],back:[-v,0],left:[0,w],right:[0,-w],stop:[0,0]}};post('/api/drive',{{linear:map[kind][0],angular:map[kind][1]}})}}
for(const b of document.querySelectorAll('[data-drive]')){{b.onpointerdown=e=>{{e.preventDefault();command(b.dataset.drive)}};b.onpointerup=b.onpointercancel=b.onpointerleave=()=>{{if(drive===b.dataset.drive)command('stop')}}}}
setInterval(()=>{{if(drive!=='stop')command(drive)}},100);
const keys={{w:'forward',s:'back',a:'left',d:'right'}};window.onkeydown=e=>{{if(e.repeat||!keys[e.key.toLowerCase()]||/INPUT/.test(e.target.tagName))return;command(keys[e.key.toLowerCase()])}};window.onkeyup=e=>{{if(keys[e.key.toLowerCase()]===drive)command('stop')}};window.onblur=()=>command('stop');document.querySelector('#home').onclick=()=>{{for(const c of controls.values()){{c.editing=false;c.slider.value=c.home;c.number.value=c.home.toFixed(2)}}post('/api/home',{{}})}};
</script></body></html>"""
