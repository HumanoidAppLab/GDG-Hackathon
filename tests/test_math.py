"""Test quaternion transformations and orientation parsing routines."""

from __future__ import annotations

import numpy as np
import pytest

from mh3_genesis.simulator import _parse_orientation, _quat_multiply, _quat_rotate


def test_parse_orientation_none() -> None:
    """Verify that None orientation returns None without error.
    :return: None when None orientation is parsed as None.
    :rtype: None
    """
    # Confirm None orientation yields None for position-only IK.
    assert _parse_orientation(None) is None


def test_parse_orientation_quaternion() -> None:
    """Verify quaternion parsing and automatic normalization.
    :return: None when quaternion target is normalized properly.
    :rtype: None
    """
    # Unnormalized quaternion (length 4).
    raw_quat = [2.0, 0.0, 0.0, 0.0]
    parsed = _parse_orientation(raw_quat)
    assert parsed is not None
    assert np.allclose(parsed, [1.0, 0.0, 0.0, 0.0])

    # Reject zero-norm quaternion.
    with pytest.raises(ValueError, match="zero norm"):
        _parse_orientation([0.0, 0.0, 0.0, 0.0])


def test_parse_orientation_euler() -> None:
    """Verify Euler angle conversion to normalized WXYZ quaternion.
    :return: None when Euler angle conversions match expected rotations.
    :rtype: None
    """
    # Zero rotation in degrees should yield identity quaternion [1, 0, 0, 0].
    parsed_zero = _parse_orientation([0.0, 0.0, 0.0], euler_degrees=True)
    assert parsed_zero is not None
    assert np.allclose(parsed_zero, [1.0, 0.0, 0.0, 0.0])

    # 90 degrees rotation around X axis (roll).
    parsed_x90 = _parse_orientation([90.0, 0.0, 0.0], euler_degrees=True)
    expected_x90 = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]
    assert parsed_x90 is not None
    assert np.allclose(parsed_x90, expected_x90)

    # Reject orientation with invalid dimensions.
    with pytest.raises(ValueError, match="Orientation target must contain 3 elements"):
        _parse_orientation([0.0, 0.0])


def test_quat_rotate_and_multiply() -> None:
    """Verify vector rotation and Hamilton quaternion multiplication.
    :return: None when quaternion rotation and product match analytical values.
    :rtype: None
    """
    # Rotate vector along Y by 90 degrees around Z axis: (0, 1, 0) rotated by 90 deg around Z -> (-1, 0, 0).
    q_z90 = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
    vector_y = np.array([0.0, 1.0, 0.0])
    rotated = _quat_rotate(q_z90, vector_y)
    assert np.allclose(rotated, [-1.0, 0.0, 0.0], atol=1e-6)

    # Multiplying two 90 degree Z rotations should equal a 180 degree Z rotation.
    q_composed = _quat_multiply(q_z90, q_z90)
    q_z180 = np.array([0.0, 0.0, 0.0, 1.0])
    assert np.allclose(q_composed, q_z180, atol=1e-6)
