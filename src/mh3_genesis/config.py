"""Load and validate the MH3 simulation and camera configuration."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]


ZED_X_VERTICAL_FOV_DEG: float = 77.95
ZED_X_HORIZONTAL_FOV_DEG: float = 104.63
ZED_X_DEFAULT_WIDTH: int = 960
ZED_X_DEFAULT_HEIGHT: int = 600
ZED_X_ASPECT_RATIO: float = ZED_X_DEFAULT_WIDTH / ZED_X_DEFAULT_HEIGHT
ZED_X_DEFAULT_FPS: float = 15.0

DEFAULT_CAMERA_MOUNTS: dict[str, tuple[str, tuple[tuple[float, float, float, float], ...]]] = {
    "base_left": (
        "mobile_base",
        (
            (0.0, 0.0, -1.0, 0.302159),
            (-1.0, 0.0, 0.0, 0.060000),
            (0.0, 1.0, 0.0, 0.106314),
            (0.0, 0.0, 0.0, 1.0),
        ),
    ),
    "base_right": (
        "mobile_base",
        (
            (0.0, 0.0, -1.0, 0.302159),
            (-1.0, 0.0, 0.0, -0.060000),
            (0.0, 1.0, 0.0, 0.106314),
            (0.0, 0.0, 0.0, 1.0),
        ),
    ),
    "head_left": (
        "head_camera_mount",
        (
            (0.0, 0.0, -1.0, 0.119917),
            (-0.00009623, 1.0, 0.0, 0.042416),
            (1.0, 0.00009623, 0.0, -0.024999),
            (0.0, 0.0, 0.0, 1.0),
        ),
    ),
    "head_right": (
        "head_camera_mount",
        (
            (0.0, 0.0, -1.0, 0.119917),
            (-0.00009623, 1.0, 0.0, 0.042412),
            (1.0, 0.00009623, 0.0, 0.025001),
            (0.0, 0.0, 0.0, 1.0),
        ),
    ),
}

STANDARD_CAMERA_FEEDS: tuple[str, ...] = (
    "head_left",
    "head_right",
    "head_depth",
    "base_left",
    "base_right",
    "base_depth",
)


@dataclass(frozen=True)
class CameraConfig:
    """Describe a camera or vision feed on the MH3 robot.
    :param name: Public camera or feed identifier.
    :type name: str
    :param enabled: Whether this camera feed is active in simulation and streaming.
    :type enabled: bool
    :param format: Image compression format ('jpeg' or 'png').
    :type format: str
    :param quality: Compression quality for JPEG encoding (1-100).
    :type quality: int
    :param width: Image width in pixels (defaults to ZED X SVGA 960).
    :type width: int
    :param height: Image height in pixels (defaults to ZED X SVGA 600).
    :type height: int
    :param near_m: Near clipping distance in metres.
    :type near_m: float
    :param far_m: Far clipping distance in metres.
    :type far_m: float
    :param fps: Locked camera capture and streaming framerate in frames per second (15.0 FPS for real ZED X).
    :type fps: float
    :param vertical_fov_deg: Locked vertical field of view in degrees (77.95° for ZED X 2.2mm lens).
    :type vertical_fov_deg: float
    :param horizontal_fov_deg: Locked horizontal field of view in degrees (104.63° for ZED X 2.2mm lens).
    :type horizontal_fov_deg: float
    :param parent_link: Mounting link name (automatically defaulted from robot definition).
    :type parent_link: str
    :param transform: Homogeneous camera pose relative to the mounting link (automatically defaulted).
    :type transform: tuple[tuple[float, float, float, float], ...]
    :return: Camera configuration.
    :rtype: CameraConfig
    """

    name: str
    enabled: bool = True
    format: str = "jpeg"
    quality: int = 80
    width: int = ZED_X_DEFAULT_WIDTH
    height: int = ZED_X_DEFAULT_HEIGHT
    near_m: float = 0.1
    far_m: float = 20.0
    fps: float = ZED_X_DEFAULT_FPS
    vertical_fov_deg: float = ZED_X_VERTICAL_FOV_DEG
    horizontal_fov_deg: float = ZED_X_HORIZONTAL_FOV_DEG
    parent_link: str = ""
    transform: tuple[tuple[float, float, float, float], ...] = ()

    def __post_init__(self) -> None:
        """Enforce physical ZED X specifications and populate hardware mount defaults.
        :return: None when parameters pass validation and defaults are applied.
        :rtype: None
        """
        # Validate that resolution dimensions are strictly positive integers.
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"camera {self.name!r} resolution must be positive")
        # Populate default mounting link and relative transform from known robot specs if not provided.
        if not self.parent_link or not self.transform:
            mount_key = (
                "head_left" if self.name == "head_depth" else ("base_left" if self.name == "base_depth" else self.name)
            )
            if mount_key in DEFAULT_CAMERA_MOUNTS:
                default_link, default_transform = DEFAULT_CAMERA_MOUNTS[mount_key]
                if not self.parent_link:
                    object.__setattr__(self, "parent_link", default_link)
                if not self.transform:
                    object.__setattr__(self, "transform", default_transform)
        # Maintain native ZED X optical aspect ratio if width or height is modified.
        if self.width != ZED_X_DEFAULT_WIDTH and self.height == ZED_X_DEFAULT_HEIGHT:
            object.__setattr__(self, "height", max(1, round(self.width / ZED_X_ASPECT_RATIO)))
        elif self.height != ZED_X_DEFAULT_HEIGHT and self.width == ZED_X_DEFAULT_WIDTH:
            object.__setattr__(self, "width", max(1, round(self.height * ZED_X_ASPECT_RATIO)))
        elif abs(self.width / self.height - ZED_X_ASPECT_RATIO) > 1e-2:
            object.__setattr__(self, "height", max(1, round(self.width / ZED_X_ASPECT_RATIO)))
        # Reject attempts to modify the locked ZED X vertical field of view.
        if abs(self.vertical_fov_deg - ZED_X_VERTICAL_FOV_DEG) > 1e-3:
            raise ValueError(
                f"camera {self.name!r} vertical_fov_deg is locked to {ZED_X_VERTICAL_FOV_DEG}° "
                "(physical ZED X 2.2mm lens optics) and cannot be modified"
            )
        # Reject attempts to modify the locked ZED X horizontal field of view.
        if abs(self.horizontal_fov_deg - ZED_X_HORIZONTAL_FOV_DEG) > 1e-3:
            raise ValueError(
                f"camera {self.name!r} horizontal_fov_deg is locked to {ZED_X_HORIZONTAL_FOV_DEG}° "
                "(physical ZED X 2.2mm lens optics) and cannot be modified"
            )
        # Reject attempts to modify the locked physical camera framerate.
        if abs(self.fps - ZED_X_DEFAULT_FPS) > 1e-3:
            raise ValueError(
                f"camera {self.name!r} fps is locked to {ZED_X_DEFAULT_FPS} FPS "
                "(physical ZED X camera hardware framerate) and cannot be modified"
            )

    @property
    def resolution(self) -> tuple[int, int]:
        """Return the image width and height in pixels.
        :return: Width and height in pixels.
        :rtype: tuple[int, int]
        """
        return self.width, self.height

    @property
    def rgb_format(self) -> str:
        """Return the encoding format for image frames.
        :return: Encoding format string.
        :rtype: str
        """
        return self.format

    @property
    def rgb_quality(self) -> int:
        """Return the compression quality for image frames.
        :return: Quality integer from 1 to 100.
        :rtype: int
        """
        return self.quality

    @property
    def depth_format(self) -> str:
        """Return the encoding format for depth frames.
        :return: Encoding format string.
        :rtype: str
        """
        return self.format

    @property
    def depth_quality(self) -> int:
        """Return the compression quality for depth frames.
        :return: Quality integer from 1 to 100.
        :rtype: int
        """
        return self.quality

    @property
    def include_depth(self) -> bool:
        """Indicate whether this camera captures depth.
        :return: True if this camera is a depth feed or left stereo camera with depth.
        :rtype: bool
        """
        return self.name.endswith("_depth") or self.name in {"head_left", "base_left"}


@dataclass(frozen=True)
class SimulatorConfig:
    """Hold the ROS-free MH3 scene, actuator, and sensor settings.
    :param robot_usd: Path to the bundled MH3 USD articulation.
    :type robot_usd: Path
    :param backend: Genesis compute backend selection.
    :type backend: str
    :param dt: Physics time step in seconds.
    :type dt: float
    :param realtime: Pace physics to wall time when True.
    :type realtime: bool
    :param show_viewer: Show the Genesis scene viewer when True.
    :type show_viewer: bool
    :param gravity: World gravity vector in metres per second squared.
    :type gravity: tuple[float, float, float]
    :param spawn_position: Robot spawn translation in metres.
    :type spawn_position: tuple[float, float, float]
    :param spawn_euler_deg: Robot spawn orientation in Euler degrees.
    :type spawn_euler_deg: tuple[float, float, float]
    :param position_kp: Default joint position proportional gain.
    :type position_kp: float
    :param position_kv: Default joint position damping gain.
    :type position_kv: float
    :param slider_position_kp: Torso lift position proportional gain.
    :type slider_position_kp: float
    :param slider_position_kv: Torso lift position damping gain.
    :type slider_position_kv: float
    :param slider_max_velocity_mps: Maximum torso lift speed in metres per second.
    :type slider_max_velocity_mps: float
    :param slider_max_reference_lead_m: Maximum lift reference lead in metres.
    :type slider_max_reference_lead_m: float
    :param slider_max_force_n: Maximum torso lift force in newtons.
    :type slider_max_force_n: float
    :param wheel_radius_m: Driven-wheel radius in metres.
    :type wheel_radius_m: float
    :param wheel_separation_m: Separation between driven wheels in metres.
    :type wheel_separation_m: float
    :param max_linear_velocity_mps: Maximum base forward speed in metres per second.
    :type max_linear_velocity_mps: float
    :param max_angular_velocity_radps: Maximum base yaw speed in radians per second.
    :type max_angular_velocity_radps: float
    :param dashboard_camera_fps: Maximum dashboard camera stream framerate in frames per second.
    :type dashboard_camera_fps: float
    :param drive_command_timeout_s: Dashboard base-drive deadman timeout in seconds.
    :type drive_command_timeout_s: float
    :param enable_gravity_compensation: Enable analytic feedforward gravity compensation.
    :type enable_gravity_compensation: bool
    :param joint_kp: Joint-specific position proportional gains keyed by public joint name.
    :type joint_kp: dict[str, float]
    :param joint_kv: Joint-specific position damping gains keyed by public joint name.
    :type joint_kv: dict[str, float]
    :param initial_positions: Startup positions keyed by public joint name.
    :type initial_positions: dict[str, float]
    :param cameras: Mounted camera configurations.
    :type cameras: tuple[CameraConfig, ...]
    :return: Simulator configuration with participant-facing joint and link names.
    :rtype: SimulatorConfig
    """

    robot_usd: Path = PROJECT_ROOT / "assets/MH3.usd"
    backend: str = "auto"
    dt: float = 0.01
    realtime: bool = True
    show_viewer: bool = True
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.81)
    spawn_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    spawn_euler_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    position_kp: float = 350.0
    position_kv: float = 35.0
    enable_gravity_compensation: bool = True
    joint_kp: dict[str, float] = field(default_factory=dict)
    joint_kv: dict[str, float] = field(default_factory=dict)
    slider_position_kp: float = 350000.0
    slider_position_kv: float = 4000.0
    slider_max_velocity_mps: float = 0.008
    slider_max_reference_lead_m: float = 0.015
    slider_max_force_n: float = 400.0
    wheel_radius_m: float = 0.1
    wheel_separation_m: float = 0.45
    max_linear_velocity_mps: float = 0.6
    max_angular_velocity_radps: float = 2.0
    dashboard_camera_fps: float = ZED_X_DEFAULT_FPS
    drive_command_timeout_s: float = 0.3
    initial_positions: dict[str, float] = field(default_factory=dict)
    cameras: tuple[CameraConfig, ...] = ()

    @classmethod
    def from_yaml(cls, path: str | Path) -> SimulatorConfig:
        """Load a simulator configuration from YAML.
        :param path: Configuration file to load.
        :type path: str | Path
        :return: Validated configuration with a resolved robot USD path.
        :rtype: SimulatorConfig
        """
        # Resolve the configuration file path to an absolute path.
        config_path = Path(path).expanduser().resolve()
        # Parse the configuration YAML file into a raw dictionary.
        raw: dict[str, Any] = yaml.safe_load(config_path.read_text()) or {}
        # Extract the robot USD path, defaulting to the class default if unspecified.
        robot_usd = Path(raw.pop("robot_usd", cls.robot_usd)).expanduser()
        # Resolve relative USD paths against the directory containing the configuration file.
        if not robot_usd.is_absolute():
            robot_usd = (config_path.parent / robot_usd).resolve()
        # Parse individual camera entries and cast transform matrices to nested tuples of floats.
        # Parse camera entries supporting both dict mapping and list specifications.
        cameras_raw = raw.pop("cameras", None)
        cameras: list[CameraConfig] = []
        raw_items: list[dict[str, Any]] = []
        if isinstance(cameras_raw, dict):
            for cam_name, item in cameras_raw.items():
                item_dict = dict(item) if isinstance(item, dict) else {}
                item_dict["name"] = str(cam_name)
                raw_items.append(item_dict)
        elif isinstance(cameras_raw, (list, tuple)):
            for item in cameras_raw:
                raw_items.append(dict(item))
        elif cameras_raw is None:
            for feed_name in STANDARD_CAMERA_FEEDS:
                raw_items.append({"name": feed_name})
        for item in raw_items:
            if item.get("transform"):
                item["transform"] = tuple(tuple(float(v) for v in row) for row in item["transform"])
            # Map legacy format and quality keys if present.
            if "format" not in item:
                if "rgb_format" in item:
                    item["format"] = str(item["rgb_format"])
                elif "depth_format" in item:
                    item["format"] = str(item["depth_format"])
            if "quality" not in item:
                if "rgb_quality" in item:
                    item["quality"] = int(item["rgb_quality"])
                elif "depth_quality" in item:
                    item["quality"] = int(item["depth_quality"])
            # Remove legacy prefixed keys.
            for key in ("rgb_format", "rgb_quality", "depth_format", "depth_quality", "include_depth"):
                item.pop(key, None)
            if "fps" in item:
                fps_val = float(item["fps"])
                if abs(fps_val - ZED_X_DEFAULT_FPS) > 1e-3:
                    raise ValueError(
                        f"camera {item.get('name')!r} fps is locked to {ZED_X_DEFAULT_FPS} FPS "
                        "(physical ZED X camera hardware framerate) and cannot be modified"
                    )
                item["fps"] = fps_val
            if "vertical_fov_deg" in item:
                fov_val = float(item["vertical_fov_deg"])
                if abs(fov_val - ZED_X_VERTICAL_FOV_DEG) > 1e-3:
                    raise ValueError(
                        f"camera {item.get('name')!r} vertical_fov_deg is locked to {ZED_X_VERTICAL_FOV_DEG}° "
                        "(physical ZED X 2.2mm lens optics) and cannot be modified"
                    )
                item["vertical_fov_deg"] = fov_val
            if "horizontal_fov_deg" in item:
                hfov_val = float(item["horizontal_fov_deg"])
                if abs(hfov_val - ZED_X_HORIZONTAL_FOV_DEG) > 1e-3:
                    raise ValueError(
                        f"camera {item.get('name')!r} horizontal_fov_deg is locked to {ZED_X_HORIZONTAL_FOV_DEG}° "
                        "(physical ZED X 2.2mm lens optics) and cannot be modified"
                    )
                item["horizontal_fov_deg"] = hfov_val
            # Maintain native optical aspect ratio when modifying width or height in YAML.
            if "width" in item and "height" not in item:
                item["height"] = max(1, round(int(item["width"]) / ZED_X_ASPECT_RATIO))
            elif "height" in item and "width" not in item:
                item["width"] = max(1, round(int(item["height"]) * ZED_X_ASPECT_RATIO))
            elif "width" in item and "height" in item:
                item_w = int(item["width"])
                item_h = int(item["height"])
                if abs(item_w / item_h - ZED_X_ASPECT_RATIO) > 1e-2:
                    if item_w != ZED_X_DEFAULT_WIDTH and item_h == ZED_X_DEFAULT_HEIGHT:
                        item["height"] = max(1, round(item_w / ZED_X_ASPECT_RATIO))
                    elif item_h != ZED_X_DEFAULT_HEIGHT and item_w == ZED_X_DEFAULT_WIDTH:
                        item["width"] = max(1, round(item_h * ZED_X_ASPECT_RATIO))
                    else:
                        item["height"] = max(1, round(item_w / ZED_X_ASPECT_RATIO))
            cameras.append(CameraConfig(**item))
        # Ensure 3D spatial vectors are converted to float tuples.
        for key in ("gravity", "spawn_position", "spawn_euler_deg"):
            if key in raw:
                raw[key] = tuple(float(value) for value in raw[key])
        # Construct the SimulatorConfig instance from parsed and resolved parameters.
        config = cls(robot_usd=robot_usd, cameras=tuple(cameras), **raw)
        # Perform comprehensive consistency and bounds checks on all configured values.
        config.validate()
        return config

    def get_camera(self, name: str) -> CameraConfig | None:
        """Find a camera configuration entry by name.
        :param name: Camera or feed identifier.
        :type name: str
        :return: CameraConfig instance if found, or None.
        :rtype: CameraConfig | None
        """
        # Search configured cameras for a matching identifier.
        for camera in self.cameras:
            if camera.name == name:
                return camera
        return None

    def validate(self) -> None:
        """Reject invalid scene, motion, and camera settings.
        :return: None after all settings pass validation.
        :rtype: None
        """
        # Confirm that the specified robot USD articulation asset exists on disk.
        if not self.robot_usd.is_file():
            raise FileNotFoundError(f"MH3 USD does not exist: {self.robot_usd}")
        # Verify that the asset has a recognised Universal Scene Description file extension.
        if self.robot_usd.suffix.lower() not in {".usd", ".usda", ".usdc", ".usdz"}:
            raise ValueError("robot_usd must be a USD file")
        # Validate that the selected compute backend is supported by Genesis.
        if self.backend not in {"auto", "cpu", "gpu", "cuda", "amdgpu", "metal"}:
            raise ValueError("backend must be one of: auto, cpu, gpu, cuda, amdgpu, metal")
        # Ensure the simulation physics time step is strictly positive.
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        # Verify that standard joint position gains are non-negative and proportional gain is positive.
        if self.position_kp <= 0.0 or self.position_kv < 0.0:
            raise ValueError("joint position gains must be non-negative with positive kp")
        # Ensure configured joint-specific proportional and damping gains are non-negative.
        for name, kp in self.joint_kp.items():
            if kp <= 0.0:
                raise ValueError(f"joint_kp for {name!r} must be strictly positive")
        for name, kv in self.joint_kv.items():
            if kv < 0.0:
                raise ValueError(f"joint_kv for {name!r} must be non-negative")
        # Verify that torso slider position gains are non-negative and proportional gain is positive.
        if self.slider_position_kp <= 0.0 or self.slider_position_kv < 0.0:
            raise ValueError("slider position gains must be non-negative with positive kp")
        # Ensure slider velocity and reference lead bounds are positive.
        if self.slider_max_velocity_mps <= 0.0 or self.slider_max_reference_lead_m <= 0.0:
            raise ValueError("slider motion limits must be positive")
        # Ensure the maximum force exerted by the torso lift is positive.
        if self.slider_max_force_n <= 0.0:
            raise ValueError("slider force limit must be positive")
        # Ensure wheel radius and track separation are positive non-zero measurements.
        if self.wheel_radius_m <= 0.0 or self.wheel_separation_m <= 0.0:
            raise ValueError("wheel dimensions must be positive")
        # Check that maximum linear and angular mobile base velocities are positive.
        if self.max_linear_velocity_mps <= 0.0 or self.max_angular_velocity_radps <= 0.0:
            raise ValueError("base velocity limits must be positive")
        # Ensure the maximum dashboard camera streaming framerate is positive.
        if self.dashboard_camera_fps <= 0.0:
            raise ValueError("dashboard_camera_fps must be positive")
        # Check that the deadman switch timeout for drive commands is positive.
        if self.drive_command_timeout_s <= 0.0:
            raise ValueError("drive_command_timeout_s must be positive")
        # Ensure the gravity vector has exactly three coordinate components.
        if len(self.gravity) != 3:
            raise ValueError("gravity must contain three values")
        # Ensure all configured camera identifiers are distinct.
        camera_names = [camera.name for camera in self.cameras]
        if len(camera_names) != len(set(camera_names)):
            raise ValueError("camera names must be unique")
        # Validate matrix dimensions, image dimensions, framerate, and compression for every camera.
        valid_formats = {"jpeg", "jpg", "png"}
        for camera in self.cameras:
            if camera.transform and (len(camera.transform) != 4 or any(len(row) != 4 for row in camera.transform)):
                raise ValueError(f"camera {camera.name!r} transform must be 4x4")
            if camera.width <= 0 or camera.height <= 0:
                raise ValueError(f"camera {camera.name!r} resolution must be positive")
            # Ensure the capture framerate remains locked to the physical camera hardware.
            if abs(camera.fps - ZED_X_DEFAULT_FPS) > 1e-3:
                raise ValueError(
                    f"camera {camera.name!r} fps is locked to {ZED_X_DEFAULT_FPS} FPS "
                    "(physical ZED X camera hardware framerate) and cannot be modified"
                )
            if camera.format.lower() not in valid_formats:
                raise ValueError(f"camera {camera.name!r} format must be 'jpeg' or 'png', got {camera.format!r}")
            if not (1 <= camera.quality <= 100):
                raise ValueError(f"camera {camera.name!r} quality must be between 1 and 100")
            # Ensure the vertical field of view remains locked to the physical camera optics.
            if abs(camera.vertical_fov_deg - ZED_X_VERTICAL_FOV_DEG) > 1e-3:
                raise ValueError(
                    f"camera {camera.name!r} vertical_fov_deg is locked to {ZED_X_VERTICAL_FOV_DEG}° "
                    "(physical ZED X 2.2mm lens optics) and cannot be modified"
                )
            # Ensure the horizontal field of view remains locked to the physical camera optics.
            if abs(camera.horizontal_fov_deg - ZED_X_HORIZONTAL_FOV_DEG) > 1e-3:
                raise ValueError(
                    f"camera {camera.name!r} horizontal_fov_deg is locked to {ZED_X_HORIZONTAL_FOV_DEG}° "
                    "(physical ZED X 2.2mm lens optics) and cannot be modified"
                )
            # Ensure the camera resolution maintains the native optical aspect ratio.
            if abs(camera.width / camera.height - ZED_X_ASPECT_RATIO) > 1e-2:
                raise ValueError(
                    f"camera {camera.name!r} resolution ({camera.width}x{camera.height}) must maintain "
                    f"the native ZED X aspect ratio of {ZED_X_ASPECT_RATIO:.2f} (16:10)"
                )
