"""Validate imported USD limits through the standalone public joint names."""
from __future__ import annotations

import math
from pathlib import Path

import pytest
from pxr import Usd, UsdPhysics

from mh3_genesis.simulator import _public_joint_name


def test_public_head_and_wrist_bounds_use_radians_after_usd_conversion() -> None:
    """Check that simplified names expose the current hardware head and model wrist bounds.
    :return: None after validating the authored USD limits in SI units.
    :rtype: None
    """
    stage = Usd.Stage.Open(str(Path(__file__).parents[1] / "assets/MH3.usd"))
    expected = {
        "head_yaw": (-1.4, 3.1), "head_pitch": (-0.4, 1.0),
        "left_6": (-0.3, 0.3), "left_7": (-0.3, 0.3),
        "right_6": (-0.3, 0.3), "right_7": (-0.3, 0.3),
    }
    found = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        name = _public_joint_name(prim.GetName())
        if name in expected:
            joint = UsdPhysics.RevoluteJoint(prim)
            found[name] = tuple(math.radians(value) for value in (
                joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()))
    assert found.keys() == expected.keys()
    for name, bounds in expected.items():
        assert found[name] == pytest.approx(bounds, abs=2e-6)
