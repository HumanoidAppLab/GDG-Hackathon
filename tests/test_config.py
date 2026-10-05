"""Test simulator and camera configuration loading and validation."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from mh3_genesis.config import CameraConfig, SimulatorConfig


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
    assert len(config.joint_kp) > 0
    assert len(config.joint_kv) > 0
    assert len(config.cameras) == 4


def test_camera_config_resolution() -> None:
    """Verify camera configuration resolution tuple property.
    :return: None when camera resolution tuple matches width and height.
    :rtype: None
    """
    # Instantiate a test camera configuration with known resolution.
    cam = CameraConfig(
        name="test_cam",
        parent_link="head_base",
        transform=((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
        width=1280,
        height=720,
    )
    # Confirm resolution property returns width and height as a tuple.
    assert cam.resolution == (1280, 720)


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
