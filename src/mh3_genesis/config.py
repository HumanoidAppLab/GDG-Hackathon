"""Load and validate the MH3 simulation and camera configuration."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]

@dataclass(frozen=True)
class CameraConfig:
    """Describe a Genesis camera mounted to a rigid MH3 link.
    :param name: Public camera identifier.
    :type name: str
    :param parent_link: Public name of the mounting link.
    :type parent_link: str
    :param transform: Homogeneous camera pose relative to the mounting link.
    :type transform: tuple[tuple[float, float, float, float], ...]
    :param width: Image width in pixels.
    :type width: int
    :param height: Image height in pixels.
    :type height: int
    :param vertical_fov_deg: Vertical field of view in degrees.
    :type vertical_fov_deg: float
    :param near_m: Near clipping distance in metres.
    :type near_m: float
    :param far_m: Far clipping distance in metres.
    :type far_m: float
    :return: Camera configuration.
    :rtype: CameraConfig
    """

    name: str
    parent_link: str
    transform: tuple[tuple[float, float, float, float], ...]
    width: int = 640
    height: int = 360
    vertical_fov_deg: float = 54.0
    near_m: float = 0.1
    far_m: float = 20.0

    @property
    def resolution(self) -> tuple[int, int]:
        """Return the image width and height in pixels.
        :return: Width and height in pixels.
        :rtype: tuple[int, int]
        """
        return self.width, self.height

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
    :param drive_command_timeout_s: Dashboard base-drive deadman timeout in seconds.
    :type drive_command_timeout_s: float
    :param initial_positions: Startup positions keyed by public joint name.
    :type initial_positions: dict[str, float]
    :param cameras: Mounted camera configurations.
    :type cameras: tuple[CameraConfig, ...]
    :return: Simulator configuration with participant-facing joint and link names.
    :rtype: SimulatorConfig
    """

    robot_usd: Path = PROJECT_ROOT / "assets/MH3.genesis.usda"
    backend: str = "auto"
    dt: float = 0.01
    realtime: bool = True
    show_viewer: bool = True
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.81)
    spawn_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    spawn_euler_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    position_kp: float = 120.0
    position_kv: float = 12.0
    slider_position_kp: float = 350000.0
    slider_position_kv: float = 4000.0
    slider_max_velocity_mps: float = 0.008
    slider_max_reference_lead_m: float = 0.015
    slider_max_force_n: float = 400.0
    wheel_radius_m: float = 0.1
    wheel_separation_m: float = 0.45
    max_linear_velocity_mps: float = 0.6
    max_angular_velocity_radps: float = 2.0
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
        config_path = Path(path).expanduser().resolve()
        raw: dict[str, Any] = yaml.safe_load(config_path.read_text()) or {}
        robot_usd = Path(raw.pop("robot_usd", cls.robot_usd)).expanduser()
        if not robot_usd.is_absolute():
            robot_usd = (config_path.parent / robot_usd).resolve()
        cameras: list[CameraConfig] = []
        for item in raw.pop("cameras", ()):
            item = dict(item)
            item["transform"] = tuple(tuple(float(value) for value in row) for row in item["transform"])
            cameras.append(CameraConfig(**item))
        for key in ("gravity", "spawn_position", "spawn_euler_deg"):
            if key in raw:
                raw[key] = tuple(float(value) for value in raw[key])
        config = cls(robot_usd=robot_usd, cameras=tuple(cameras), **raw)
        config.validate()
        return config

    def validate(self) -> None:
        """Reject invalid scene, motion, and camera settings.
        :return: None after all settings pass validation.
        :rtype: None
        """
        if not self.robot_usd.is_file():
            raise FileNotFoundError(f"MH3 USD does not exist: {self.robot_usd}")
        if self.robot_usd.suffix.lower() not in {".usd", ".usda", ".usdc", ".usdz"}:
            raise ValueError("robot_usd must be a USD file")
        if self.backend not in {"auto", "cpu", "gpu", "cuda", "amdgpu", "metal"}:
            raise ValueError("backend must be one of: auto, cpu, gpu, cuda, amdgpu, metal")
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.position_kp <= 0.0 or self.position_kv < 0.0:
            raise ValueError("joint position gains must be non-negative with positive kp")
        if self.slider_position_kp <= 0.0 or self.slider_position_kv < 0.0:
            raise ValueError("slider position gains must be non-negative with positive kp")
        if self.slider_max_velocity_mps <= 0.0 or self.slider_max_reference_lead_m <= 0.0:
            raise ValueError("slider motion limits must be positive")
        if self.slider_max_force_n <= 0.0:
            raise ValueError("slider force limit must be positive")
        if self.wheel_radius_m <= 0.0 or self.wheel_separation_m <= 0.0:
            raise ValueError("wheel dimensions must be positive")
        if self.max_linear_velocity_mps <= 0.0 or self.max_angular_velocity_radps <= 0.0:
            raise ValueError("base velocity limits must be positive")
        if self.drive_command_timeout_s <= 0.0:
            raise ValueError("drive_command_timeout_s must be positive")
        if len(self.gravity) != 3:
            raise ValueError("gravity must contain three values")
        camera_names = [camera.name for camera in self.cameras]
        if len(camera_names) != len(set(camera_names)):
            raise ValueError("camera names must be unique")
        for camera in self.cameras:
            if len(camera.transform) != 4 or any(len(row) != 4 for row in camera.transform):
                raise ValueError(f"camera {camera.name!r} transform must be 4x4")
            if camera.width <= 0 or camera.height <= 0:
                raise ValueError(f"camera {camera.name!r} resolution must be positive")
