"""Check the bundled hand geometry and its physical articulation contract."""

from pathlib import Path

import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics


@pytest.fixture(scope="module")
def stage():
    return Usd.Stage.Open(str(Path(__file__).resolve().parents[1] / "assets" / "MH3.usd"))


def test_hand_zero_pose_aligns_joint_anchors(stage):
    """Genesis must import a fully open hand when all finger DOFs are zero."""
    cache = UsdGeom.XformCache()
    count = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        targets = joint.GetBody1Rel().GetTargets()
        if not targets or not targets[0].name.startswith(
            tuple(
                f"{side}_{finger}_"
                for side in ("L", "R")
                for finger in ("thumb", "index", "middle", "miidle", "ring", "little", "liitle")
            )
        ):
            continue
        anchors = []
        for side in (0, 1):
            body = (joint.GetBody0Rel() if side == 0 else joint.GetBody1Rel()).GetTargets()[0]
            rotation = (joint.GetLocalRot0Attr() if side == 0 else joint.GetLocalRot1Attr()).Get()
            position = (joint.GetLocalPos0Attr() if side == 0 else joint.GetLocalPos1Attr()).Get()
            anchor = Gf.Matrix4d(1.0)
            anchor.SetRotate(rotation)
            anchor.SetTranslateOnly(Gf.Vec3d(position))
            anchors.append(np.asarray(anchor * cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body))))
        np.testing.assert_allclose(anchors[0], anchors[1], atol=1e-6, rtol=0, err_msg=str(prim.GetPath()))
        state = prim.GetAttribute("state:angular:physics:position")
        if state and state.HasAuthoredValueOpinion():
            assert state.Get() == 0.0
        count += 1
    assert count == 40  # Thirty moving segments plus ten fixed fingertip sensors.


def test_hand_limits_and_mimics_preserve_full_travel(stage):
    """Both hands retain their six actuators and signed follower ratios."""
    joints = {prim.GetName(): prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.RevoluteJoint)}
    for side in ("L", "R"):
        for finger in ("thumb", "index", "middle", "ring", "little"):
            name = f"{side}_{finger}_1_joint"
            joint = UsdPhysics.RevoluteJoint(joints[name])
            assert joint.GetLowerLimitAttr().Get() == 0.0
            upper = 1.6 if finger == "thumb" else 1.57
            assert np.deg2rad(joint.GetUpperLimitAttr().Get()) == pytest.approx(upper)
        thumb = UsdPhysics.RevoluteJoint(joints[f"{side}_thumb_2_joint"])
        assert np.deg2rad(thumb.GetUpperLimitAttr().Get()) == pytest.approx(0.45)
    followers = [prim for prim in joints.values() if prim.GetRelationship("physxMimicJoint:rotZ:referenceJoint")]
    assert len(followers) == 18
    for follower in followers:
        reference = follower.GetRelationship("physxMimicJoint:rotZ:referenceJoint").GetTargets()[0].name
        assert reference in joints
        gearing = follower.GetAttribute("physxMimicJoint:rotZ:gearing").Get()
        expected = -0.8 if "_ee" in follower.GetName() and "thumb" not in follower.GetName() else 0.8
        assert gearing == pytest.approx(expected)
        assert follower.GetAttribute("physxMimicJoint:rotZ:offset").Get() == 0.0
