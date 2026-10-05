"""Test public joint and link name translation logic."""

from __future__ import annotations

from mh3_genesis.simulator import _public_joint_name, _public_link_name


def test_public_joint_name_translations() -> None:
    """Verify USD joint name translation to participant-facing vocabulary.
    :return: None when all joint name mappings match expected public identifiers.
    :rtype: None
    """
    # Test arm joint name simplification.
    assert _public_joint_name("L_link_1_joint") == "left_1"
    assert _public_joint_name("R_link_7_joint") == "right_7"

    # Test finger primary control suffix stripping.
    assert _public_joint_name("L_index_1_joint") == "left_index"
    assert _public_joint_name("R_thumb_1_joint") == "right_thumb_1"

    # Test head and base joints.
    assert _public_joint_name("head_yaw_joint") == "head_yaw"
    assert _public_joint_name("slider_joint") == "slider"


def test_public_link_name_translations() -> None:
    """Verify USD link name translation to clean participant-facing identifiers.
    :return: None when all link name mappings match expected public aliases.
    :rtype: None
    """
    # Test special base and camera link aliases.
    assert _public_link_name("base_link_head") == "head_base"
    assert _public_link_name("base_link_torso") == "torso_base"
    assert _public_link_name("head_camera_base_frame") == "head_camera_mount"

    # Test arm link prefix substitutions.
    assert _public_link_name("L_link_1_link") == "left_arm_1"
    assert _public_link_name("R_link_7_link") == "right_arm_7"

    # Test end effector tip suffixing.
    assert _public_link_name("L_gripper_base_link") == "left_gripper_base"
    assert _public_link_name("L_index_ee") == "left_index_tip"
    assert _public_link_name("R_thumb_ee") == "right_thumb_tip"
