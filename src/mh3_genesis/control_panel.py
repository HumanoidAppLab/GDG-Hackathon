"""Serve the optional local dashboard for robot joints, drive, and cameras."""
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

from .dashboard_script import SCRIPT
from .dashboard_style import STYLE
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
    """Queue local browser commands for application on the simulation thread.
    :param simulation: Built MH3 simulation to control and observe.
    :type simulation: MH3Simulation
    :return: Browser dashboard controller.
    :rtype: MH3ControlPanel
    """

    def __init__(self, simulation: MH3Simulation) -> None:
        """Initialize dashboard state from a built simulation.
        :param simulation: Built MH3 simulation to control and observe.
        :type simulation: MH3Simulation
        :return: None after initializing dashboard state.
        :rtype: None
        """
        # Store the active simulation instance to observe and command.
        self.simulation = simulation
        # Protect shared mutable dashboard state across threads with a mutex.
        self._lock = threading.Lock()
        # Generate an ephemeral secret token to authenticate local HTTP POST requests.
        self._token = secrets.token_urlsafe(24)
        # Initialize internal buffers for queued joint targets and mobile base drive commands.
        self._pending_positions: dict[str, float] = {}
        self._drive = (0.0, 0.0)
        self._drive_changed = False
        self._drive_active = False
        self._drive_received_at = 0.0
        self._last_snapshot_at = 0.0
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        # Discover stereo camera pairs available on the robot articulation.
        self._camera_pairs = {
            source: (f"{source}_left", f"{source}_right")
            for source in ("head", "base")
            if {f"{source}_left", f"{source}_right"}.issubset(simulation.camera_names)
        }
        self._camera_frames: dict[str, bytes] = {}
        self._last_camera_request_at = 0.0
        self._last_camera_render_at = 0.0
        self._camera_error: str | None = None
        # Exclude internal wheels and passive castors from public joint controls.
        self._joint_names = tuple(name for name in simulation.joint_names if name not in _BASE_INTERNAL_JOINTS)
        # Query current joint states and limit ranges from the built simulation.
        state = simulation.get_joint_state(self._joint_names)
        lower, upper = simulation.get_joint_limits(state.names)
        limits: dict[str, tuple[float, float]] = {}
        # Replace infinite limits with safe fallback ranges.
        for name, low, high in zip(state.names, lower, upper, strict=True):
            fallback = 0.5 if name == "slider" else math.pi
            low = float(low) if math.isfinite(low) else -fallback
            high = float(high) if math.isfinite(high) else fallback
            limits[name] = (low, high)
        self._limits = limits
        # Cache initial home positions and seed current positions from measured state.
        self._home = {
            name: float(simulation.config.initial_positions.get(name, value))
            for name, value in zip(state.names, state.positions, strict=True)
        }
        self._positions = {name: float(value) for name, value in zip(state.names, state.positions, strict=True)}

    @property
    def url(self) -> str:
        """Return the local dashboard URL after server startup.
        :return: Loopback HTTP URL for this dashboard.
        :rtype: str
        """
        if self._server is None:
            raise RuntimeError("Control panel has not been started")
        return f"http://127.0.0.1:{self._server.server_port}/"

    def start(self, *, open_browser: bool = True) -> None:
        """Start the loopback dashboard server and optionally open a browser.
        :param open_browser: Open the dashboard in the default browser when True.
        :type open_browser: bool
        :return: None after starting or confirming the existing server.
        :rtype: None
        """
        if self._server is not None:
            return
        panel = self
        class Handler(BaseHTTPRequestHandler):
            """Handle local dashboard HTML, state, camera, and command requests.
            :return: Request handler bound to the control panel.
            :rtype: Handler
            """
            def do_GET(self) -> None:
                """Serve the dashboard, simulation state, or a camera feed.
                :return: None after sending an HTTP response.
                :rtype: None
                """
                # Serve the main HTML dashboard interface at the root path.
                if self.path == "/":
                    self._send(
                        _html(
                            panel._token,
                            panel.simulation.config.max_linear_velocity_mps,
                            panel.simulation.config.max_angular_velocity_radps,
                        ),
                        "text/html; charset=utf-8",
                    )
                # Return current robot joint states and simulation timing as JSON.
                elif self.path == "/api/state":
                    self._send(json.dumps(panel.state()), "application/json")
                # Return binary JPEG or PNG frames for requested camera streams.
                elif self.path.startswith("/api/camera/"):
                    name = self.path.removeprefix("/api/camera/")
                    # Reject unknown camera feed identifiers with an HTTP 404 response.
                    if name not in panel._camera_feed_names:
                        self.send_error(HTTPStatus.NOT_FOUND)
                        return
                    frame = panel.camera_frame(name)
                    # Report temporary unavailability if the initial frame has not been rendered yet.
                    if frame is None:
                        self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "Camera frame is not ready")
                        return
                    self._send(frame, "image/png" if name.endswith("-depth") else "image/jpeg")
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self) -> None:
                """Validate a dashboard command and enqueue it for simulation.
                :return: None after sending an HTTP response.
                :rtype: None
                """
                # Ensure the request provides the required matching control authentication token.
                if self.headers.get("X-Control-Token") != panel._token:
                    self.send_error(HTTPStatus.FORBIDDEN)
                    return
                try:
                    # Read and deserialize JSON payload from the request body.
                    length = int(self.headers.get("Content-Length", "0"))
                    body = json.loads(self.rfile.read(length) or b"{}")
                    # Dispatch to corresponding target queue handler based on endpoint path.
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
                    # Return client error status if command format or values are invalid.
                    self.send_error(HTTPStatus.BAD_REQUEST, str(error))
                    return
                # Acknowledge successful receipt with an empty JSON object.
                self._send("{}", "application/json")

            def _send(self, body: str | bytes, content_type: str) -> None:
                """Send a no-store success response with the supplied body.
                :param body: Text or binary HTTP response body.
                :type body: str | bytes
                :param content_type: MIME type of the response body.
                :type content_type: str
                :return: None after writing the response.
                :rtype: None
                """
                # Ensure the payload is encoded into raw bytes.
                encoded = body.encode() if isinstance(body, str) else body
                # Write standard HTTP response headers with cache prevention.
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(encoded)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                # Write the response payload to the network stream.
                self.wfile.write(encoded)

            def log_message(self, format: str, *args: Any) -> None:
                """Suppress routine request logging from the local HTTP server.
                :param format: HTTP server log message format.
                :type format: str
                :param args: Values supplied for the log message.
                :type args: Any
                :return: None without emitting a request log.
                :rtype: None
                """
                return
        # Bind the HTTP server to loopback on an ephemeral port.
        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        # Run server request handling on a dedicated background thread.
        self._thread = threading.Thread(target=self._server.serve_forever, name="mh3-controls", daemon=True)
        self._thread.start()
        print(f"MH3 control panel: {self.url}")
        # Launch the local dashboard in the system default browser if enabled.
        if open_browser:
            webbrowser.open(self.url)

    def close(self) -> None:
        """Stop base motion and shut down the local dashboard server.
        :return: None after server shutdown.
        :rtype: None
        """
        # Return cleanly if the server is not currently running.
        if self._server is None:
            return
        # Zero out mobile base motion before shutting down.
        self.queue_drive(0.0, 0.0)
        # Terminate the request processing loop and close the listening socket.
        self._server.shutdown()
        self._server.server_close()
        # Wait for the background worker thread to exit cleanly.
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._server = None
        self._thread = None

    def state(self) -> dict[str, Any]:
        """Return a thread-safe snapshot for the dashboard.
        :return: Joint positions, base command, camera status, and sim time.
        :rtype: dict[str, Any]
        """
        # Assemble thread-safe snapshot dictionary under mutex protection.
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
        """List available dashboard camera feed identifiers.
        :return: Source-qualified RGB and depth feed names.
        :rtype: set[str]
        """
        return {f"{source}-{feed}" for source in self._camera_pairs for feed in ("left-rgb", "right-rgb", "depth")}

    def camera_frame(self, name: str) -> bytes | None:
        """Return a cached encoded camera frame and mark it requested.
        :param name: Source-qualified dashboard feed name.
        :type name: str
        :return: Encoded image bytes, or None before the first render.
        :rtype: bytes | None
        """
        # Validate that the requested camera stream name is supported.
        if name not in self._camera_feed_names:
            raise KeyError(f"Unknown camera feed: {name}")
        # Record access timestamp and retrieve latest encoded frame bytes.
        with self._lock:
            self._last_camera_request_at = time.monotonic()
            return self._camera_frames.get(name)

    @staticmethod
    def _encode_rgb(rgb: np.ndarray) -> bytes:
        """Encode an RGB image as a JPEG for the dashboard.
        :param rgb: RGB image array.
        :type rgb: np.ndarray
        :return: JPEG image bytes.
        :rtype: bytes
        """
        # Convert RGB array to an 8-bit PIL Image and compress as JPEG.
        output = BytesIO()
        Image.fromarray(np.asarray(rgb, dtype=np.uint8)).save(output, format="JPEG", quality=80)
        return output.getvalue()

    @staticmethod
    def _encode_depth(depth: np.ndarray, near_m: float, far_m: float) -> bytes:
        """Encode metric depth as a grayscale dashboard preview.
        :param depth: Metric depth image array.
        :type depth: np.ndarray
        :param near_m: Camera near clipping distance in metres.
        :type near_m: float
        :param far_m: Camera far clipping distance in metres.
        :type far_m: float
        :return: Grayscale PNG image bytes.
        :rtype: bytes
        """
        values = np.asarray(depth)
        display_far = min(far_m, 5.0)
        # Mask out non-finite, negative, and out-of-range depth values.
        valid = np.isfinite(values) & (values > 0.0) & (values < far_m)
        grayscale = np.zeros(values.shape, dtype=np.uint8)
        # Linearly map valid distance values to brighter near and darker far pixels.
        grayscale[valid] = np.clip((display_far - values[valid]) / (display_far - near_m) * 255.0, 0.0, 255.0).astype(
            np.uint8
        )
        # Encode grayscale array as a compressed PNG image.
        output = BytesIO()
        Image.fromarray(grayscale).save(output, format="PNG")
        return output.getvalue()

    def _render_camera_frames(self, simulation: MH3Simulation) -> None:
        """Refresh encoded head and base feeds on the simulation thread.
        :param simulation: Simulation that owns the Genesis cameras.
        :type simulation: MH3Simulation
        :return: None after caching the available camera feeds.
        :rtype: None
        """
        frames = {}
        # Iterate over stereo camera pairs and capture synchronized frames.
        for source, (left_name, right_name) in self._camera_pairs.items():
            left = simulation.capture_camera(left_name, rgb=True, depth=True)
            right = simulation.capture_camera(right_name, rgb=True, depth=False)
            # Ensure captured frames are valid.
            if left.rgb is None or left.depth is None or right.rgb is None:
                raise RuntimeError(f"Camera pair {source!r} did not return RGB and depth frames")
            camera_config = next(camera for camera in simulation.config.cameras if camera.name == left_name)
            # Compress captured RGB and metric depth buffers into web-deliverable image formats.
            frames.update(
                {
                    f"{source}-left-rgb": self._encode_rgb(left.rgb),
                    f"{source}-right-rgb": self._encode_rgb(right.rgb),
                    f"{source}-depth": self._encode_depth(left.depth, camera_config.near_m, camera_config.far_m),
                }
            )
        # Store newly encoded frames and clear previous camera errors.
        with self._lock:
            self._camera_frames = frames
            self._camera_error = None

    def queue_positions(self, targets: dict[str, Any]) -> None:
        """Validate and queue public joint position targets.
        :param targets: Targets keyed by participant-facing joint name.
        :type targets: dict[str, Any]
        :return: None after queuing bounded targets.
        :rtype: None
        """
        # Ensure targets argument is a valid mapping object.
        if not isinstance(targets, dict):
            raise TypeError("targets must be an object keyed by joint name")
        clean: dict[str, float] = {}
        # Verify each target refers to a known joint and contains a finite numeric value.
        for name, value in targets.items():
            if name not in self._limits:
                raise KeyError(f"Unknown joint: {name}")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"Joint target must be finite: {name}")
            # Clamp the commanded target to the configured minimum and maximum limits.
            low, high = self._limits[name]
            clean[name] = min(max(value, low), high)
        # Enqueue validated targets into the pending positions buffer under lock.
        with self._lock:
            self._pending_positions.update(clean)

    def queue_drive(self, linear: Any, angular: Any) -> None:
        """Queue a finite base drive command and refresh its deadman timer.
        :param linear: Forward speed in metres per second.
        :type linear: Any
        :param angular: Yaw speed in radians per second.
        :type angular: Any
        :return: None after queuing the base command.
        :rtype: None
        """
        # Convert commanded velocities to floats and confirm they are finite.
        linear, angular = float(linear), float(angular)
        if not math.isfinite(linear) or not math.isfinite(angular):
            raise ValueError("Drive commands must be finite")
        # Update pending drive command velocities and reset the deadman timer.
        with self._lock:
            self._drive = (linear, angular)
            self._drive_changed = True
            self._drive_active = linear != 0.0 or angular != 0.0
            self._drive_received_at = time.monotonic()

    def queue_home(self) -> None:
        """Queue home positions for every joint and stop base motion.
        :return: None after queuing the home and stop commands.
        :rtype: None
        """
        # Reset all controllable joints to their configured home startup positions.
        with self._lock:
            self._pending_positions.update(self._home)
        # Immediately command base velocities to zero.
        self.queue_drive(0.0, 0.0)

    def update(self, simulation: MH3Simulation) -> None:
        """Apply queued commands and refresh dashboard state after a step.
        :param simulation: Simulation invoking this registered step hook.
        :type simulation: MH3Simulation
        :return: None after applying commands and updating previews.
        :rtype: None
        """
        now = time.monotonic()
        # Retrieve and clear pending position and drive commands under lock.
        with self._lock:
            positions = self._pending_positions
            self._pending_positions = {}
            drive = self._drive
            drive_changed = self._drive_changed
            self._drive_changed = False
            # Check if active drive command has exceeded the deadman switch timeout.
            if self._drive_active and now - self._drive_received_at > simulation.config.drive_command_timeout_s:
                drive = (0.0, 0.0)
                self._drive = drive
                self._drive_active = False
                drive_changed = True
        # Forward queued joint targets to the simulation articulation.
        if positions:
            simulation.set_joint_positions(positions)
        # Forward changed base velocity commands to the simulation differential drive.
        if drive_changed:
            linear, angular = drive
            simulation.drive_base(linear, angular)
        # Periodically sample measured joint state for dashboard display at roughly 10 Hz.
        if now - self._last_snapshot_at >= 0.1:
            state = simulation.get_joint_state(self._joint_names)
            with self._lock:
                self._positions = {name: float(value) for name, value in zip(state.names, state.positions, strict=True)}
            self._last_snapshot_at = now
        # Check whether an active client has requested camera frames in the last two seconds.
        with self._lock:
            camera_requested = now - self._last_camera_request_at < 2.0
        # Render camera frames at a controlled frame rate if cameras are active and requested.
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
    """Render dashboard HTML with drive limits and a private command token.
    :param token: Per-server token required for command POST requests.
    :type token: str
    :param max_linear: Maximum forward speed in metres per second.
    :type max_linear: float
    :param max_angular: Maximum yaw speed in radians per second.
    :type max_angular: float
    :return: Complete dashboard HTML document.
    :rtype: str
    """
    # Calculate step sizes for UI range sliders based on maximum configured speeds.
    linear_step = max_linear / 12.0
    angular_step = max_angular / 20.0
    # Return the assembled HTML template incorporating styles, controls, and script.
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MH3 Controls</title><style>
{STYLE}
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
const TOKEN={json.dumps(token)};
{SCRIPT}
</script></body></html>"""
