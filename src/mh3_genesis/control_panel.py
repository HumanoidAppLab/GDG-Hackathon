"""Serve the optional local dashboard for robot joints, drive, and cameras."""

from __future__ import annotations

import base64
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
from urllib.parse import parse_qs, urlparse

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from .config import CameraConfig
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
_RECOMMENDED_END_EFFECTORS = (
    "left_gripper_base",
    "right_gripper_base",
    "left_index_tip",
    "right_index_tip",
    "left_thumb_tip",
    "right_thumb_tip",
)
_RECOMMENDED_FRAMES = (
    "mobile_base",
    "torso_base",
    "linear_actuator_base",
    "left_arm_base",
    "right_arm_base",
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
        self._pending_ik: dict[str, Any] | None = None
        self._last_ik_status: str | None = None
        self._drive = (0.0, 0.0)
        self._drive_changed = False
        self._drive_active = False
        self._drive_received_at = 0.0
        self._last_snapshot_at = 0.0
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        # Index enabled and available camera configurations from simulation.
        self._active_cameras: dict[str, CameraConfig] = {
            camera.name: camera
            for camera in simulation.config.cameras
            if camera.enabled and camera.name in simulation.camera_names
        }
        # Discover stereo camera sources with at least one active camera.
        self._camera_sources: list[str] = [
            source for source in ("head", "base") if any(cam.startswith(f"{source}_") for cam in self._active_cameras)
        ]
        # Map source-qualified dashboard feed names to their originating cameras and channels.
        feed_map: dict[str, tuple[str, str]] = {}
        for source in ("head", "base"):
            left_cam = f"{source}_left"
            right_cam = f"{source}_right"
            if left_cam in self._active_cameras:
                feed_map[f"{source}-left-rgb"] = ("rgb", left_cam)
                if self._active_cameras[left_cam].include_depth:
                    feed_map[f"{source}-depth"] = ("depth", left_cam)
            if right_cam in self._active_cameras:
                feed_map[f"{source}-right-rgb"] = ("rgb", right_cam)
                if f"{source}-depth" not in feed_map and self._active_cameras[right_cam].include_depth:
                    feed_map[f"{source}-depth"] = ("depth", right_cam)
        # Register any custom non-stereo cameras directly.
        for cam_name, cam_cfg in self._active_cameras.items():
            if not cam_name.startswith(("head_", "base_")):
                feed_map[f"{cam_name}-rgb"] = ("rgb", cam_name)
                if cam_cfg.include_depth:
                    feed_map[f"{cam_name}-depth"] = ("depth", cam_name)
        self._camera_feed_map = feed_map
        self._camera_frames: dict[str, bytes] = {}
        self._last_camera_request_at = 0.0
        self._last_camera_render_times: dict[str, float] = {}
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
        # Cache true initial home positions from simulation startup and seed current positions from measured state.
        self._home = {
            name: float(simulation.home_positions.get(name, simulation.config.initial_positions.get(name, value)))
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
                # Return 3D position and orientation for a requested link and reference frame.
                elif self.path.startswith("/api/link-pose"):
                    parsed = urlparse(self.path)
                    params = parse_qs(parsed.query)
                    link = params.get("link", [None])[0]
                    frame = params.get("frame", [None])[0]
                    if not link or link not in panel.simulation.link_names:
                        self.send_error(HTTPStatus.BAD_REQUEST, "Missing or unknown link")
                        return
                    clean_frame = None if not frame or frame == "world" else frame
                    if clean_frame is not None and clean_frame not in panel.simulation.link_names:
                        self.send_error(HTTPStatus.BAD_REQUEST, "Unknown reference frame link")
                        return
                    try:
                        pos, quat = panel.simulation.get_link_pose(link, frame=clean_frame)
                        # Compute Euler angles (roll, pitch, yaw in degrees) following extrinsic XYZ convention.
                        rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
                        euler_deg = rot.as_euler("xyz", degrees=True)
                        payload = {
                            "link": link,
                            "frame": clean_frame,
                            "position": [float(p) for p in pos],
                            "quaternion": [float(q) for q in quat],
                            "euler_deg": [float(e) for e in euler_deg],
                        }
                        self._send(json.dumps(payload), "application/json")
                    except (KeyError, ValueError, RuntimeError) as error:
                        self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, str(error))
                        return
                # Return all cached camera frames as a consolidated base64 payload.
                elif self.path == "/api/camera-frames":
                    frames = panel.all_camera_frames()
                    self._send(json.dumps(frames), "application/json")
                # Return individual binary frames for backward compatibility.
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
                    content_type = "image/png" if frame.startswith(b"\x89PNG") else "image/jpeg"
                    self._send(frame, content_type)
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
                    elif self.path == "/api/ik":
                        panel.queue_ik(
                            end_effector=body.get("end_effector"),
                            position=body.get("position"),
                            orientation=body.get("orientation"),
                            quaternion=body.get("quaternion"),
                            frame=body.get("frame"),
                            joints=body.get("joints"),
                            euler_degrees=bool(body.get("euler_degrees", False)),
                        )
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
                "camera_sources": list(self._camera_sources),
                "camera_feeds": list(self._camera_feed_map.keys()),
                "camera_error": self._camera_error,
                "ik_status": self._last_ik_status,
                "end_effectors": [name for name in _RECOMMENDED_END_EFFECTORS if name in self.simulation.link_names],
                "frames": [name for name in _RECOMMENDED_FRAMES if name in self.simulation.link_names],
                "all_links": list(self.simulation.link_names),
            }

    @property
    def _camera_feed_names(self) -> set[str]:
        """List available dashboard camera feed identifiers.
        :return: Active RGB and depth feed names.
        :rtype: set[str]
        """
        return set(self._camera_feed_map.keys())

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

    def all_camera_frames(self) -> dict[str, str]:
        """Return all cached camera frames as base64 data URLs.
        :return: Mapping of feed names to base64 data URL strings.
        :rtype: dict[str, str]
        """
        # Record access timestamp and snapshot latest encoded frame bytes under lock.
        with self._lock:
            self._last_camera_request_at = time.monotonic()
            frames = dict(self._camera_frames)
        # Encode raw frame bytes as base64 data URLs with matching MIME types.
        payload = {}
        for name, data in frames.items():
            mime = "image/png" if data.startswith(b"\x89PNG") else "image/jpeg"
            payload[name] = f"data:{mime};base64,{base64.b64encode(data).decode('ascii')}"
        return payload

    @staticmethod
    def _encode_rgb(rgb: np.ndarray, format_name: str = "jpeg", quality: int = 80) -> bytes:
        """Encode an RGB image in the configured image format.
        :param rgb: RGB image array.
        :type rgb: np.ndarray
        :param format_name: Target image format ('jpeg' or 'png').
        :type format_name: str
        :param quality: Compression quality for JPEG encoding (1-100).
        :type quality: int
        :return: Encoded image bytes.
        :rtype: bytes
        """
        output = BytesIO()
        image = Image.fromarray(np.asarray(rgb, dtype=np.uint8))
        # Choose compression format based on camera configuration.
        if format_name.lower() in ("jpeg", "jpg"):
            image.save(output, format="JPEG", quality=int(np.clip(quality, 1, 100)))
        else:
            image.save(output, format="PNG")
        return output.getvalue()

    @staticmethod
    def _encode_depth(
        depth: np.ndarray,
        near_m: float,
        far_m: float,
        format_name: str = "jpeg",
        quality: int = 85,
    ) -> bytes:
        """Encode metric depth as a grayscale preview in the configured image format.
        :param depth: Metric depth image array.
        :type depth: np.ndarray
        :param near_m: Camera near clipping distance in metres.
        :type near_m: float
        :param far_m: Camera far clipping distance in metres.
        :type far_m: float
        :param format_name: Target image format ('jpeg' or 'png').
        :type format_name: str
        :param quality: Compression quality for JPEG encoding (1-100).
        :type quality: int
        :return: Encoded grayscale image bytes.
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
        output = BytesIO()
        image = Image.fromarray(grayscale)
        # Compress grayscale buffer into requested JPEG or PNG image format.
        if format_name.lower() in ("jpeg", "jpg"):
            image.save(output, format="JPEG", quality=int(np.clip(quality, 1, 100)))
        else:
            image.save(output, format="PNG")
        return output.getvalue()

    def _render_cameras(self, simulation: MH3Simulation, camera_names: list[str]) -> None:
        """Refresh encoded feeds for specified cameras on the simulation thread.
        :param simulation: Simulation that owns the Genesis cameras.
        :type simulation: MH3Simulation
        :param camera_names: Names of active cameras due for rendering.
        :type camera_names: list[str]
        :return: None after updating the camera frames cache.
        :rtype: None
        """
        new_frames: dict[str, bytes] = {}
        # Iterate over due cameras and capture frames matching their configuration.
        for cam_name in camera_names:
            cam_cfg = self._active_cameras[cam_name]
            need_depth = cam_cfg.include_depth
            capture = simulation.capture_camera(cam_name, rgb=True, depth=need_depth)
            if capture.rgb is None or (need_depth and capture.depth is None):
                raise RuntimeError(f"Camera {cam_name!r} capture returned incomplete buffers")
            # Compress RGB buffer using camera-specific format and quality.
            encoded_rgb = self._encode_rgb(capture.rgb, format_name=cam_cfg.rgb_format, quality=cam_cfg.rgb_quality)
            # Compress depth buffer using camera-specific format and quality if enabled.
            encoded_depth = (
                self._encode_depth(
                    capture.depth,
                    cam_cfg.near_m,
                    cam_cfg.far_m,
                    format_name=cam_cfg.depth_format,
                    quality=cam_cfg.depth_quality,
                )
                if need_depth and capture.depth is not None
                else None
            )
            # Map encoded image buffers to their configured dashboard feed names.
            for feed_name, (channel, feed_cam) in self._camera_feed_map.items():
                if feed_cam == cam_name:
                    if channel == "rgb":
                        new_frames[feed_name] = encoded_rgb
                    elif channel == "depth" and encoded_depth is not None:
                        new_frames[feed_name] = encoded_depth
        # Store newly encoded frames and clear previous camera errors under lock.
        with self._lock:
            self._camera_frames.update(new_frames)
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

    def queue_ik(
        self,
        end_effector: str,
        position: Any = None,
        orientation: Any = None,
        quaternion: Any = None,
        frame: Any = None,
        joints: Any = None,
        euler_degrees: bool = True,
    ) -> None:
        """Validate and queue an inverse kinematics target for execution.
        :param end_effector: Public link name of the target end effector.
        :type end_effector: str
        :param position: Target XYZ coordinates in the chosen frame, or None to keep current position.
        :type position: Any
        :param orientation: Optional orientation target as Euler angles or quaternion.
        :type orientation: Any
        :param quaternion: Optional backward-compatible quaternion alias.
        :type quaternion: Any
        :param frame: Reference link name for the target pose, or None for world space.
        :type frame: Any
        :param joints: Controllable joint names to vary, or None to auto-select arm joints.
        :type joints: Any
        :param euler_degrees: Interpret 3-element Euler angles as degrees when True.
        :type euler_degrees: bool
        :return: None after queuing the request.
        :rtype: None
        """
        # Validate that the requested end effector is a known robot link.
        if end_effector not in self.simulation.link_names:
            raise KeyError(f"Unknown end effector link: {end_effector}")
        # Parse target position if supplied.
        pos_tuple: tuple[float, float, float] | None = None
        if position is not None:
            pos_coords = tuple(float(coord) for coord in position)
            if len(pos_coords) != 3 or not all(math.isfinite(coord) for coord in pos_coords):
                raise ValueError("IK target position must contain exactly three finite numbers (X, Y, Z)")
            pos_tuple = pos_coords
        # Parse target orientation if supplied as either Euler angles or a quaternion.
        orient_tuple: tuple[float, ...] | None = None
        raw_orient = quaternion if orientation is None else orientation
        if raw_orient is not None:
            orient_coords = tuple(float(coord) for coord in raw_orient)
            if len(orient_coords) not in (3, 4) or not all(math.isfinite(coord) for coord in orient_coords):
                raise ValueError(
                    "IK target orientation must contain 3 (Euler angles) or 4 (quaternion WXYZ) finite numbers"
                )
            orient_tuple = orient_coords
        # Verify that at least one target criterion was provided.
        if pos_tuple is None and orient_tuple is None:
            raise ValueError("At least one of target position or target orientation must be provided")
        # Normalize empty string or 'world' frame to None.
        clean_frame: str | None = None if not frame or frame == "world" else str(frame)
        if clean_frame is not None and clean_frame not in self.simulation.link_names:
            raise KeyError(f"Unknown reference frame link: {clean_frame}")
        # Resolve joint constraints based on user choice or auto-detection.
        joint_subset: list[str] | None = None
        if isinstance(joints, list) and joints:
            joint_subset = [str(j) for j in joints]
        elif joints == "left":
            joint_subset = [f"left_{i}" for i in range(1, 8)]
        elif joints == "right":
            joint_subset = [f"right_{i}" for i in range(1, 8)]
        elif joints == "all":
            joint_subset = None
        elif "left" in end_effector:
            joint_subset = [f"left_{i}" for i in range(1, 8)]
        elif "right" in end_effector:
            joint_subset = [f"right_{i}" for i in range(1, 8)]
        # Store pending request and initial status under lock.
        with self._lock:
            self._pending_ik = {
                "end_effector": end_effector,
                "position": pos_tuple,
                "orientation": orient_tuple,
                "frame": clean_frame,
                "joints": joint_subset,
                "euler_degrees": euler_degrees,
            }
            self._last_ik_status = f"Solving IK for {end_effector}…"

    def update(self, simulation: MH3Simulation) -> None:
        """Apply queued commands and refresh dashboard state after a step.
        :param simulation: Simulation invoking this registered step hook.
        :type simulation: MH3Simulation
        :return: None after applying commands and updating previews.
        :rtype: None
        """
        now = time.monotonic()
        # Retrieve and clear any pending inverse kinematics request under lock.
        with self._lock:
            pending_ik = self._pending_ik
            self._pending_ik = None
        if pending_ik is not None:
            try:
                # Solve inverse kinematics and apply target joint positions on the simulation thread.
                simulation.solve_ik(
                    end_effector=pending_ik["end_effector"],
                    position=pending_ik["position"],
                    orientation=pending_ik.get("orientation"),
                    frame=pending_ik["frame"],
                    joints=pending_ik["joints"],
                    euler_degrees=pending_ik.get("euler_degrees", False),
                    apply=True,
                )
                with self._lock:
                    frame_label = pending_ik["frame"] or "world"
                    desc_parts = []
                    if pending_ik["position"] is not None:
                        p = pending_ik["position"]
                        desc_parts.append(f"pos=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
                    else:
                        desc_parts.append("held position")
                    if pending_ik.get("orientation") is not None:
                        o = pending_ik["orientation"]
                        if len(o) == 3:
                            desc_parts.append(f"rot=({o[0]:.1f}°, {o[1]:.1f}°, {o[2]:.1f}°)")
                        else:
                            desc_parts.append(f"quat=({o[0]:.2f}, {o[1]:.2f}, {o[2]:.2f}, {o[3]:.2f})")
                    self._last_ik_status = (
                        f"Applied {pending_ik['end_effector']} {', '.join(desc_parts)} [{frame_label}]"
                    )
            except Exception as error:
                logger.exception("Inverse kinematics solver failed")
                with self._lock:
                    self._last_ik_status = f"IK failed: {error}"
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
        # Render camera frames when requested and active cameras are configured.
        if camera_requested and self._active_cameras:
            # Determine which active cameras are due for a frame update based on individual framerates.
            due_cameras = [
                cam_name
                for cam_name, cam_cfg in self._active_cameras.items()
                if now - self._last_camera_render_times.get(cam_name, 0.0) >= (1.0 / max(cam_cfg.fps, 1.0))
            ]
            if due_cameras:
                try:
                    self._render_cameras(simulation, due_cameras)
                except Exception as error:
                    with self._lock:
                        changed = str(error) != self._camera_error
                        self._camera_error = str(error)
                    if changed:
                        logger.exception("MH3 dashboard camera rendering failed")
                finally:
                    render_now = time.monotonic()
                    for cam_name in due_cameras:
                        self._last_camera_render_times[cam_name] = render_now


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
<div class="sidebar"><section class="card"><h2>Base drive</h2><div class="speed">
<label>Linear<input id="linear" type="range" min="{linear_step}" max="{max_linear}" step="{linear_step}" value="{max_linear}"><output>{max_linear:.2f}</output></label>
<label>Angular<input id="angular" type="range" min="{angular_step}" max="{max_angular}" step="{angular_step}" value="{max_angular}"><output>{max_angular:.2f}</output></label></div>
<div class="drive-grid"><i></i><button data-drive="forward">▲</button><i></i><button data-drive="left">◀</button><button class="stop" data-drive="stop">■</button><button data-drive="right">▶</button><i></i><button data-drive="back">▼</button></div>
<button class="home" id="home">Return all joints home</button><p class="hint">Hold the buttons or W/A/S/D to drive. Releasing a control stops the base automatically.</p></section>
<section class="card" id="ik-panel"><h2>Inverse kinematics (IK) tester</h2>
<label class="field-label">End effector<select id="ik-ee"></select></label>
<label class="field-label">Reference frame<select id="ik-frame"><option value="">World space (None)</option></select></label>
<div class="ik-target-header">
<label class="orient-check"><input type="checkbox" id="ik-solve-pos" checked> <span>Solve Position</span></label>
</div>
<div class="coord-row" id="ik-pos-row">
<label><span>X</span><input id="ik-x" type="number" step="0.001" value="0.45"></label>
<label><span>Y</span><input id="ik-y" type="number" step="0.001" value="0.25"></label>
<label><span>Z</span><input id="ik-z" type="number" step="0.001" value="0.85"></label>
</div>
<div class="ik-target-header">
<label class="orient-check"><input type="checkbox" id="ik-solve-orient"> <span>Solve Orientation</span></label>
<select id="ik-orient-mode" style="display:none;width:auto;padding:2px 6px;font-size:11px;margin:0">
<option value="euler">Euler (°)</option>
<option value="quat">Quat (WXYZ)</option>
</select>
</div>
<div id="ik-orient-container" style="display:none">
<div class="coord-row" id="ik-euler-row">
<label><span>R</span><input id="ik-roll" type="number" step="1.0" value="0.0"></label>
<label><span>P</span><input id="ik-pitch" type="number" step="1.0" value="0.0"></label>
<label><span>Y</span><input id="ik-yaw" type="number" step="1.0" value="0.0"></label>
</div>
<div class="coord-row quat-row" id="ik-quat-row" style="display:none">
<label><span>W</span><input id="ik-qw" type="number" step="0.01" value="1.0"></label>
<label><span>X</span><input id="ik-qx" type="number" step="0.01" value="0.0"></label>
<label><span>Y</span><input id="ik-qy" type="number" step="0.01" value="0.0"></label>
<label><span>Z</span><input id="ik-qz" type="number" step="0.01" value="0.0"></label>
</div>
</div>
<label class="field-label">Constrain joints<select id="ik-joints">
<option value="auto">Auto (arm joints)</option>
<option value="left">Left arm (left_1–7)</option>
<option value="right">Right arm (right_1–7)</option>
<option value="all">All controllable joints</option>
</select></label>
<div class="ik-buttons"><button type="button" id="ik-read">Read pose</button><button type="button" id="ik-solve" class="btn-primary">Apply IK</button></div>
<p class="hint" id="ik-status">Ready to solve IK.</p></section></div>
<div class="workspace"><section class="card" id="camera-panel"><h2>Camera feeds</h2>
<div class="camera-row" id="head-cameras"><h3>Head cameras</h3><div class="camera-grid"><figure><img id="head-left-rgb" alt="Head left RGB camera feed"><figcaption>Left RGB</figcaption></figure><figure><img id="head-right-rgb" alt="Head right RGB camera feed"><figcaption>Right RGB</figcaption></figure><figure><img id="head-depth" alt="Head depth camera feed"><figcaption>Left camera depth</figcaption></figure></div></div>
<div class="camera-row" id="base-cameras"><h3>Base cameras</h3><div class="camera-grid"><figure><img id="base-left-rgb" alt="Base left RGB camera feed"><figcaption>Left RGB</figcaption></figure><figure><img id="base-right-rgb" alt="Base right RGB camera feed"><figcaption>Right RGB</figcaption></figure><figure><img id="base-depth" alt="Base depth camera feed"><figcaption>Left camera depth</figcaption></figure></div></div><p class="hint" id="camera-status">Waiting for camera frames… Near depth is bright; beyond 5 m is dark.</p></section>
<section id="joint-groups"></section></div></main><script>
const TOKEN={json.dumps(token)};
{SCRIPT}
</script></body></html>"""
