"""Test simulator and camera configuration loading and validation."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from mh3_genesis.config import PROJECT_ROOT, CameraConfig, SimulatorConfig


def test_load_default_config() -> None:
    """Verify that default YAML configuration loads and passes all validation checks.
    :return: None when default configuration is valid.
    :rtype: None
    """
    # Load configuration from the default repository configuration file.
    config = SimulatorConfig.from_yaml("config/mh3.yaml")
    # Verify core configuration parameters are loaded with positive physical values.
    assert config.robot_usd.is_file()
    assert config.dt == 0.01
    assert config.enable_gravity_compensation is True
    assert config.dashboard_camera_fps == 15.0
    assert len(config.joint_kp) > 0
    assert len(config.joint_kv) > 0
    # Verify default camera configuration defines all 6 standard vision feeds.
    assert len(config.cameras) == 6
    feed_names = {camera.name for camera in config.cameras}
    assert feed_names == {"head_left", "head_right", "head_depth", "base_left", "base_right", "base_depth"}
    # Verify default camera settings match physical ZED X SVGA optics and ROS framerates.
    head_left = config.get_camera("head_left")
    assert head_left is not None
    assert head_left.width == 960
    assert head_left.height == 600
    assert head_left.vertical_fov_deg == 77.95
    assert head_left.horizontal_fov_deg == 104.63
    assert head_left.fps == 15.0
    assert head_left.parent_link == "head_camera_mount"
    assert config.get_camera("base_left").parent_link == "mobile_base"


def test_camera_config_resolution() -> None:
    """Verify camera configuration resolution tuple property and default dimensions.
    :return: None when camera resolution tuple matches width and height.
    :rtype: None
    """
    # Instantiate default camera configuration and verify default resolution and locked FOV.
    cam = CameraConfig(name="test_cam")
    # Confirm default resolution matches ZED X SVGA and locked vertical/horizontal FOVs and framerate.
    assert cam.resolution == (960, 600)
    assert cam.vertical_fov_deg == 77.95
    assert cam.horizontal_fov_deg == 104.63
    assert cam.fps == 15.0
    assert cam.format == "jpeg"
    assert cam.quality == 80


def test_config_validation_failures(tmp_path: Path) -> None:
    """Verify that SimulatorConfig validation rejects physically invalid parameters.
    :param tmp_path: Temporary directory fixture provided by pytest.
    :type tmp_path: Path
    :return: None when validation catches each invalid configuration.
    :rtype: None
    """
    # Load valid baseline configuration.
    base = SimulatorConfig.from_yaml("config/mh3.yaml")

    # Reject non-existent robot USD file.
    with pytest.raises(FileNotFoundError):
        replace(base, robot_usd=tmp_path / "missing.usd").validate()

    # Reject non-USD file extension.
    with pytest.raises(ValueError, match="robot_usd must be a USD file"):
        dummy = tmp_path / "robot.txt"
        dummy.touch()
        replace(base, robot_usd=dummy).validate()

    # Reject unsupported compute backend.
    with pytest.raises(ValueError, match="backend must be one of"):
        replace(base, backend="invalid_backend").validate()

    # Reject non-positive time step.
    with pytest.raises(ValueError, match="dt must be positive"):
        replace(base, dt=0.0).validate()

    # Reject negative position proportional gain.
    with pytest.raises(ValueError, match="joint position gains must be non-negative"):
        replace(base, position_kp=-10.0).validate()

    # Reject invalid joint-specific proportional gain.
    with pytest.raises(ValueError, match="joint_kp for 'left_1' must be strictly positive"):
        replace(base, joint_kp={"left_1": 0.0}).validate()

    # Reject non-positive dashboard camera streaming framerate.
    with pytest.raises(ValueError, match="dashboard_camera_fps must be positive"):
        replace(base, dashboard_camera_fps=0.0).validate()

    # Reject unsupported camera image format.
    bad_cam_format = replace(base.cameras[0], format="bmp")
    with pytest.raises(ValueError, match="format must be 'jpeg' or 'png'"):
        replace(base, cameras=(bad_cam_format,)).validate()

    # Reject out-of-bounds camera JPEG compression quality.
    bad_cam_quality = replace(base.cameras[0], quality=105)
    with pytest.raises(ValueError, match="quality must be between 1 and 100"):
        replace(base, cameras=(bad_cam_quality,)).validate()

    # Reject modifying locked vertical field of view.
    with pytest.raises(ValueError, match="vertical_fov_deg is locked to 77.95°"):
        CameraConfig(name="bad_vfov", vertical_fov_deg=60.0)

    # Reject modifying locked horizontal field of view.
    with pytest.raises(ValueError, match="horizontal_fov_deg is locked to 104.63°"):
        CameraConfig(name="bad_hfov", horizontal_fov_deg=90.0)

    # Reject modifying locked camera framerate.
    with pytest.raises(ValueError, match="fps is locked to 15.0 FPS"):
        CameraConfig(name="bad_fps", fps=30.0)
    # Reject non-positive camera resolution.
    with pytest.raises(ValueError, match="resolution must be positive"):
        CameraConfig(name="bad_res", width=0)
    # Reject camera resolution with invalid aspect ratio during validation.
    bad_cam_res = object.__new__(CameraConfig)
    object.__setattr__(bad_cam_res, "name", "bad_aspect")
    object.__setattr__(bad_cam_res, "enabled", True)
    object.__setattr__(bad_cam_res, "format", "jpeg")
    object.__setattr__(bad_cam_res, "quality", 80)
    object.__setattr__(bad_cam_res, "width", 1280)
    object.__setattr__(bad_cam_res, "height", 720)
    object.__setattr__(bad_cam_res, "near_m", 0.1)
    object.__setattr__(bad_cam_res, "far_m", 20.0)
    object.__setattr__(bad_cam_res, "fps", 15.0)
    object.__setattr__(bad_cam_res, "vertical_fov_deg", 77.95)
    object.__setattr__(bad_cam_res, "horizontal_fov_deg", 104.63)
    object.__setattr__(bad_cam_res, "parent_link", "head_camera_mount")
    object.__setattr__(bad_cam_res, "transform", ())
    with pytest.raises(ValueError, match="must maintain the native ZED X aspect ratio"):
        replace(base, cameras=(bad_cam_res,)).validate()


def test_camera_config_customization_and_disable() -> None:
    """Verify camera configuration options for disabling, format, and compression.
    :return: None when customized camera parameters match configuration.
    :rtype: None
    """
    cam = CameraConfig(
        name="custom_cam",
        width=1280,
        height=720,
        enabled=False,
        format="png",
        quality=95,
    )
    assert cam.enabled is False
    assert cam.fps == 15.0
    assert cam.format == "png"
    assert cam.quality == 95
    assert cam.rgb_format == "png"
    assert cam.depth_format == "png"
    assert cam.rgb_quality == 95
    # Confirm height was updated from 720 to 800 to maintain the proper 16:10 aspect ratio.
    assert cam.width == 1280
    assert cam.height == 800
    assert cam.resolution == (1280, 800)


def test_camera_aspect_ratio_maintained_on_width_change() -> None:
    """Verify that updating a camera's width updates height to maintain aspect ratio.
    :return: None when height matches the computed aspect ratio.
    :rtype: None
    """
    # Changing width to 1920 updates height to 1200.
    cam1 = CameraConfig(name="cam1", width=1920)
    assert cam1.width == 1920
    assert cam1.height == 1200
    assert cam1.resolution == (1920, 1200)
    # Changing width to 480 updates height to 300.
    cam2 = CameraConfig(name="cam2", width=480)
    assert cam2.width == 480
    assert cam2.height == 300
    assert cam2.resolution == (480, 300)


def test_camera_aspect_ratio_maintained_on_height_change() -> None:
    """Verify that updating a camera's height updates width to maintain aspect ratio.
    :return: None when width matches the computed aspect ratio.
    :rtype: None
    """
    # Changing height to 1200 updates width to 1920.
    cam1 = CameraConfig(name="cam1", height=1200)
    assert cam1.width == 1920
    assert cam1.height == 1200
    assert cam1.resolution == (1920, 1200)
    # Changing height to 300 updates width to 480.
    cam2 = CameraConfig(name="cam2", height=300)
    assert cam2.width == 480
    assert cam2.height == 300
    assert cam2.resolution == (480, 300)


def test_camera_aspect_ratio_yaml_loading(tmp_path: Path) -> None:
    """Verify that YAML configs specifying only width or only height maintain aspect ratio.
    :param tmp_path: Temporary directory fixture provided by pytest.
    :type tmp_path: Path
    :return: None when loaded YAML cameras have properly paired resolutions.
    :rtype: None
    """
    usd_path = (PROJECT_ROOT / "assets/MH3.usd").as_posix()
    yaml_content = f"""
robot_usd: "{usd_path}"
cameras:
  head_left:
    width: 1920
  base_left:
    height: 400
  head_right:
    width: 1280
    height: 720
"""
    yaml_file = tmp_path / "mh3_test.yaml"
    yaml_file.write_text(yaml_content)
    config = SimulatorConfig.from_yaml(yaml_file)
    # Verify width-only specification updated height to 1200.
    head_left = config.get_camera("head_left")
    assert head_left is not None
    assert head_left.resolution == (1920, 1200)
    # Verify height-only specification updated width to 640.
    base_left = config.get_camera("base_left")
    assert base_left is not None
    assert base_left.resolution == (640, 400)
    # Verify mismatched dimensions in YAML were reconciled to maintain aspect ratio.
    head_right = config.get_camera("head_right")
    assert head_right is not None
    assert head_right.resolution == (1280, 800)
