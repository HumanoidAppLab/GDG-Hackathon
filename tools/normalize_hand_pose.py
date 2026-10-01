"""Bake the bundled hand links at joint zero for Genesis USD import.

Genesis uses authored link transforms as its zero pose. Align both joint
anchors, including fixed fingertip sensors, so a zero command opens the hands.
Run from the repository with ``uv run python tools/normalize_hand_pose.py``.
"""

from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROBOT_USD = Path(__file__).resolve().parents[1] / "assets" / "MH3.usd"


def anchor_transform(joint: UsdPhysics.Joint, side: int) -> Gf.Matrix4d:
    """Return a joint anchor's transform in its body's frame."""
    position = joint.GetLocalPos0Attr() if side == 0 else joint.GetLocalPos1Attr()
    rotation = joint.GetLocalRot0Attr() if side == 0 else joint.GetLocalRot1Attr()
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(rotation.Get())
    matrix.SetTranslateOnly(Gf.Vec3d(position.Get()))
    return matrix


def normalize_hand_pose(stage: Usd.Stage) -> int:
    """Align hand joint anchors at zero without changing articulation settings."""
    cache = UsdGeom.XformCache()
    roots = {
        f"{side}_{finger}_{segment}_joint"
        for side in ("L", "R")
        for finger, segment in (("thumb", 1), ("index", 1), ("middle", 1), ("ring", 1), ("little", 1))
    }
    joints = [UsdPhysics.Joint(prim) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Joint)]
    pending = joints.copy()
    updated = set()
    while pending:
        progress = False
        for joint in pending.copy():
            parents = joint.GetBody0Rel().GetTargets()
            children = joint.GetBody1Rel().GetTargets()
            if not parents or not children:
                continue
            if joint.GetPrim().GetName() not in roots and parents[0] not in updated:
                continue
            parent = stage.GetPrimAtPath(parents[0])
            child = stage.GetPrimAtPath(children[0])
            # USD matrices use row vectors: child_anchor * child_world =
            # parent_anchor * parent_world at joint zero.
            world = anchor_transform(joint, 1).GetInverse() * anchor_transform(joint, 0)
            world *= cache.GetLocalToWorldTransform(parent)
            local = world * cache.GetLocalToWorldTransform(child.GetParent()).GetInverse()
            xform = UsdGeom.Xformable(child)
            operations = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
            operations["xformOp:translate"].Set(local.ExtractTranslation())
            operations["xformOp:orient"].Set(local.ExtractRotationQuat())
            state = joint.GetPrim().GetAttribute("state:angular:physics:position")
            if state and state.HasAuthoredValueOpinion():
                state.Set(0.0)
            updated.add(children[0])
            pending.remove(joint)
            cache.Clear()
            progress = True
        if not progress:
            break
    return len(updated)


if __name__ == "__main__":
    stage = Usd.Stage.Open(str(ROBOT_USD))
    count = normalize_hand_pose(stage)
    if count != 40:
        raise RuntimeError(f"Expected 40 hand links; found {count}. The USD was not saved.")
    stage.GetRootLayer().Save()
    print(f"Normalized {count} hand links in {ROBOT_USD.name}")
