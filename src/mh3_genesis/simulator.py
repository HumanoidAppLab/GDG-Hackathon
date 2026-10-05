"""Expose a ROS-free MH3 simulation with joints, links, cameras, and IK."""

from __future__ import annotations

import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from .config import SimulatorConfig

StepHook = Callable[["MH3Simulation"], None]
SceneHook = Callable[["MH3Simulation"], None]

_LEFT_WHEEL = "left_wheel_joint"
_RIGHT_WHEEL = "right_wheel_joint"
_INTERNAL_BASE_JOINTS = frozenset(
    {
        _LEFT_WHEEL,
        _RIGHT_WHEEL,
        "left_castor_joint",
        "left_castor_wheel_joint",
        "right_castor_joint",
        "right_castor_wheel_joint",
    }
)


def _public_joint_name(name: str) -> str:
    """Translate a USD joint name into the participant-facing vocabulary.
    :param name: Original joint name from the USD articulation.
    :type name: str
    :return: Simplified public joint name.
    :rtype: str
    """
    # Strip the trailing joint suffix from the USD name.
    name = name.removesuffix("_joint")
    # Translate internal arm and hand naming prefixes into standard left and right identifiers.
    for prefix, replacement in (("L_link_", "left_"), ("R_link_", "right_"), ("L_", "left_"), ("R_", "right_")):
        if name.startswith(prefix):
            part = name[len(prefix) :]
            # Drop the suffix from primary finger controls to provide intuitive public joint names.
            if prefix in ("L_", "R_") and part.startswith(("index_", "middle_", "ring_", "little_")):
                part = part.removesuffix("_1")
            return replacement + part
    return name


def _public_link_name(name: str) -> str:
    """Give imported links predictable names without changing the USD.
    :param name: Original link name from the USD articulation.
    :type name: str
    :return: Simplified public link name.
    :rtype: str
    """
    # Map special USD link identifiers to clean public link aliases.
    special = {
        "base_link_head": "head_base",
        "base_link_torso": "torso_base",
        "head_camera_base_frame": "head_camera_mount",
    }
    if name in special:
        return special[name]
    # Remove redundant link suffix.
    name = name.removesuffix("_link")
    # Substitute USD prefixes with readable public names.
    for prefix, replacement in (("L_link_", "left_arm_"), ("R_link_", "right_arm_"), ("L_", "left_"), ("R_", "right_")):
        if name.startswith(prefix):
            name = replacement + name[len(prefix) :]
            break
    # Correct spelling anomalies present in the underlying USD assets.
    name = name.replace("liitle", "little").replace("miidle", "middle")
    name = name.replace("_F_sensor", "_force_sensor")
    # Replace end-effector acronyms with descriptive tip designations.
    return name.removesuffix("_ee") + "_tip" if name.endswith("_ee") else name


def _select_genesis_backend(gs: Any, requested: str) -> Any:
    """Map a backend name to the matching Genesis backend.
    :param gs: Imported Genesis module.
    :type gs: Any
    :param requested: Requested compute backend name.
    :type requested: str
    :return: Genesis backend selection, including its automatic GPU selector.
    :rtype: Any
    """
    # Return the Genesis backend enum matching the requested identifier string.
    return {
        "auto": gs.gpu,
        "gpu": gs.gpu,
        "cpu": gs.cpu,
        "cuda": gs.cuda,
        "amdgpu": gs.amdgpu,
        "metal": gs.metal,
    }[requested]


def _quat_rotate(q_wxyz: np.ndarray, v_xyz: np.ndarray) -> np.ndarray:
    """Rotate a 3D vector by a quaternion in WXYZ format.
    :param q_wxyz: Orientation quaternion formatted as (W, X, Y, Z).
    :type q_wxyz: np.ndarray
    :param v_xyz: 3D translation vector to rotate.
    :type v_xyz: np.ndarray
    :return: Rotated 3D vector.
    :rtype: np.ndarray
    """
    # Extract imaginary vector components from the orientation quaternion.
    r = q_wxyz[1:4]
    # Compute double cross product: t = 2.0 * (r x v).
    t = 2.0 * np.cross(r, v_xyz)
    # Complete Rodrigues rotation formula: v' = v + w*t + r x t.
    return v_xyz + q_wxyz[0] * t + np.cross(r, t)


def _quat_multiply(q1_wxyz: np.ndarray, q2_wxyz: np.ndarray) -> np.ndarray:
    """Multiply two quaternions formatted in WXYZ order (q1 * q2).
    :param q1_wxyz: Left orientation quaternion (W, X, Y, Z).
    :type q1_wxyz: np.ndarray
    :param q2_wxyz: Right orientation quaternion (W, X, Y, Z).
    :type q2_wxyz: np.ndarray
    :return: Composed orientation quaternion in WXYZ order.
    :rtype: np.ndarray
    """
    # Unpack scalar and vector parts of both quaternions.
    w1, x1, y1, z1 = q1_wxyz
    w2, x2, y2, z2 = q2_wxyz
    # Compute Hamiltonian quaternion product.
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


@dataclass(frozen=True)
class JointState:
    """Copy the current simulated joint state.
    :param names: Public joint names in array order.
    :type names: tuple[str, ...]
    :param positions: Joint positions in radians or metres.
    :type positions: np.ndarray
    :param velocities: Joint velocities in radians or metres per second.
    :type velocities: np.ndarray
    :param efforts: Joint efforts in newtons or newton-metres.
    :type efforts: np.ndarray
    :return: Joint state snapshot.
    :rtype: JointState
    """

    names: tuple[str, ...]
    positions: np.ndarray
    velocities: np.ndarray
    efforts: np.ndarray


@dataclass(frozen=True)
class CameraFrame:
    """Hold images rendered directly by a Genesis camera.
    :param rgb: RGB image when requested.
    :type rgb: np.ndarray | None
    :param depth: Depth image when requested.
    :type depth: np.ndarray | None
    :param segmentation: Segmentation image when requested.
    :type segmentation: np.ndarray | None
    :param normal: Surface-normal image when requested.
    :type normal: np.ndarray | None
    :return: Camera frame containing the requested channels.
    :rtype: CameraFrame
    """

    rgb: np.ndarray | None = None
    depth: np.ndarray | None = None
    segmentation: np.ndarray | None = None
    normal: np.ndarray | None = None


class MH3Simulation:
    """Expose a ROS-free API around the MH3 Genesis articulation.
    Call :meth:`build` once before using joints, cameras, or IK. Native Genesis
    objects remain available through ``scene`` and ``robot``.
    :param config: Scene, actuator, and camera configuration.
    :type config: SimulatorConfig
    :return: MH3 simulation controller.
    :rtype: MH3Simulation
    """

    def __init__(self, config: SimulatorConfig) -> None:
        """Store configuration and initialize the unbuilt simulator state.
        :param config: Scene, actuator, and camera configuration.
        :type config: SimulatorConfig
        :return: None after initializing the controller.
        :rtype: None
        """
        self.config = config
        self.gs: Any = None
        self.scene: Any = None
        self.floor: Any = None
        self.robot: Any = None
        self._joint_indices: dict[str, int] = {}
        self._q_indices: dict[str, int] = {}
        self._finger_mimics: dict[str, tuple[str, float, float]] = {}
        self._links: dict[str, Any] = {}
        self._cameras: dict[str, Any] = {}
        self._scene_hooks: list[SceneHook] = []
        self._hooks: list[StepHook] = []
        self._sim_time = 0.0
        self._slider_target: float | None = None
        self._slider_reference: float | None = None
        self._slider_limits: tuple[float, float] | None = None
        self._home_positions: dict[str, float] = {}
        self.compute_backend: str | None = None

    @staticmethod
    def _numpy(value: Any, dtype: Any | None = np.float64) -> np.ndarray:
        """Convert a Genesis tensor or array into a NumPy array.
        :param value: Tensor or array to convert.
        :type value: Any
        :param dtype: Requested NumPy dtype, or None to retain the input dtype.
        :type dtype: Any | None
        :return: Converted NumPy array.
        :rtype: np.ndarray
        """
        # Detach PyTorch tensors from computation graphs if gradient tracking is attached.
        if hasattr(value, "detach"):
            value = value.detach()
        # Transfer GPU-resident tensor buffers back to host CPU memory.
        if hasattr(value, "cpu"):
            value = value.cpu()
        # Cast the underlying data to a standard NumPy ndarray.
        return np.asarray(value, dtype=dtype)

    def build(self) -> MH3Simulation:
        """Initialize Genesis and build the floor, robot, and cameras.
        :return: This built simulation for chained calls.
        :rtype: MH3Simulation
        """
        import genesis as gs

        self.gs = gs
        # Select the desired compute backend and initialize the Genesis simulation framework.
        backend = _select_genesis_backend(gs, self.config.backend)
        gs.init(backend=backend, logging_level="warning")
        self.compute_backend = gs.backend.name
        # Construct the Genesis scene containing simulation physics, lighting, and camera options.
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.config.dt, gravity=self.config.gravity),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(3.5, -3.5, 2.8),
                camera_lookat=(0.0, 0.0, 0.9),
                camera_fov=45,
                refresh_rate=60,
            ),
            vis_options=gs.options.VisOptions(show_world_frame=True),
            show_viewer=self.config.show_viewer,
        )
        # Add a static ground collision plane to the scene.
        self.floor = self.scene.add_entity(gs.morphs.Plane())
        # Load the MH3 USD articulation stage into the scene at configured spawn coordinates.
        entities = self.scene.add_stage(
            gs.morphs.USD(
                file=str(self.config.robot_usd),
                pos=self.config.spawn_position,
                euler=self.config.spawn_euler_deg,
            )
        )
        # Identify the primary articulated robot model from the entities imported from the USD stage.
        articulated = [entity for entity in entities if any(joint.n_dofs for joint in entity.joints)]
        if not articulated:
            raise RuntimeError(f"No articulation was found in {self.config.robot_usd}")
        self.robot = max(articulated, key=lambda entity: sum(joint.n_dofs for joint in entity.joints))
        # Add unattached camera sensors to the scene before scene compilation.
        for camera_config in self.config.cameras:
            self._cameras[camera_config.name] = self.scene.add_camera(
                res=camera_config.resolution,
                fov=camera_config.vertical_fov_deg,
                near=camera_config.near_m,
                far=camera_config.far_m,
                GUI=False,
            )
        # Execute registered pre-build scene hooks to add custom user entities.
        for hook in tuple(self._scene_hooks):
            hook(self)
        # Compile Genesis kernels and build the simulation scene.
        self.scene.build()
        # Discover articulation joints, links, and finger mimic chains from the loaded asset.
        self._discover_joints()
        self._discover_finger_mimics()
        # Initialize actuator gains, position holds, and lift limits.
        self._configure_actuators()
        # Attach offscreen cameras to their specified mounting links on the robot.
        self._attach_cameras()
        return self

    def _require_built(self) -> None:
        """Reject robot operations before the scene is built.
        :return: None when a robot articulation is available.
        :rtype: None
        """
        # Ensure that the build() method was executed before performing robot queries.
        if self.robot is None:
            raise RuntimeError("Call build() before using the simulation")

    def _discover_joints(self) -> None:
        """Index imported articulation links and one-DOF joints by USD name.
        :return: None after updating the internal name indexes.
        :rtype: None
        """
        # Index all robot links by their short USD prim name.
        self._links = {link.name.rstrip("/").rsplit("/", 1)[-1]: link for link in self.robot.links}
        # Index one-degree-of-freedom joints to their local articulation DOF and coordinate indices.
        for joint in self.robot.joints:
            if joint.n_dofs == 1:
                name = joint.name.strip().rstrip("/").rsplit("/", 1)[-1]
                self._joint_indices[name] = int(joint.dofs_idx_local[0])
                self._q_indices[name] = int(joint.qs_idx_local[0])

    def _discover_finger_mimics(self) -> None:
        """Read finger follower chains authored in the MH3 USD.
        :return: None after indexing follower relationships.
        :rtype: None
        """
        from pxr import Usd

        # Open the USD stage directly to inspect PhysX mimic joint authoring metadata.
        stage = Usd.Stage.Open(str(self.config.robot_usd))
        if stage is None:
            raise RuntimeError(f"Could not open MH3 USD: {self.config.robot_usd}")
        self._finger_mimics = {}
        # Traverse USD prims to find mimic joint relationships on finger joints.
        for prim in stage.Traverse():
            relation = prim.GetRelationship("physxMimicJoint:rotZ:referenceJoint")
            if not relation or not relation.GetTargets():
                continue
            follower = prim.GetName()
            reference = relation.GetTargets()[0].name
            # Ensure both the reference joint and follower joint exist in the articulation.
            if follower not in self._joint_indices or reference not in self._joint_indices:
                raise RuntimeError(f"MH3 finger mimic joint is missing: {reference} -> {follower}")
            gearing = prim.GetAttribute("physxMimicJoint:rotZ:gearing").Get()
            offset = prim.GetAttribute("physxMimicJoint:rotZ:offset").Get()
            # PhysX gearing uses the opposite sign to the URDF mimic multiplier.
            self._finger_mimics[follower] = (reference, -float(gearing), float(offset or 0.0))

    def _expanded_finger_targets(self, targets: Mapping[str, float]) -> dict[str, float]:
        """Expand commanded finger roots into their follower joint targets.
        :param targets: Joint targets keyed by internal USD name.
        :type targets: Mapping[str, float]
        :return: Targets including resolved finger followers.
        :rtype: dict[str, float]
        """
        expanded = dict(targets)
        pending = dict(self._finger_mimics)
        # Iteratively resolve mimic dependencies until all reachable followers are computed.
        while pending:
            resolved = False
            for follower, (reference, multiplier, offset) in tuple(pending.items()):
                if reference in expanded:
                    expanded[follower] = multiplier * expanded[reference] + offset
                    del pending[follower]
                    resolved = True
            if not resolved:
                break
        return expanded

    def _sync_finger_mimics(self, *, immediate: bool = False) -> None:
        """Command each follower to match its finger root.
        :param immediate: Also teleport followers to their target poses.
        :type immediate: bool
        :return: None after applying follower targets.
        :rtype: None
        """
        if not self._finger_mimics:
            return
        # Find all independent finger root joints that drive dependent follower segments.
        roots = tuple({reference for reference, _, _ in self._finger_mimics.values()} - self._finger_mimics.keys())
        indices = [self._joint_indices[name] for name in roots]
        # Read current physical positions of all finger roots.
        positions = self._numpy(self.robot.get_dofs_position(indices))
        # Compute derived target angles for all corresponding follower joints.
        expanded = self._expanded_finger_targets(dict(zip(roots, positions, strict=True)))
        followers = tuple(name for name in self._finger_mimics if name in expanded)
        values = [expanded[name] for name in followers]
        follower_indices = [self._joint_indices[name] for name in followers]
        # Teleport followers immediately if requested.
        if immediate:
            self.robot.set_dofs_position(values, follower_indices)
        # Command position controller setpoints for the follower DOFs.
        self.robot.control_dofs_position(values, follower_indices)

    def _configure_actuators(self) -> None:
        """Configure position holds and the slower torso lift actuator.
        :return: None after applying gains, force limits, and startup targets.
        :rtype: None
        """
        indices = tuple(self._joint_indices.values())
        # Assign uniform proportional and derivative gains to all standard articulation DOFs.
        if indices:
            self.robot.set_dofs_kp(np.full(len(indices), self.config.position_kp), indices)
            self.robot.set_dofs_kv(np.full(len(indices), self.config.position_kv), indices)
        # Configure specialized high-gain, force-limited controller parameters for the torso slider.
        slider_index = self._joint_indices.get("slider_joint")
        if slider_index is not None:
            self.robot.set_dofs_kp([self.config.slider_position_kp], [slider_index])
            self.robot.set_dofs_kv([self.config.slider_position_kv], [slider_index])
            force = self.config.slider_max_force_n
            self.robot.set_dofs_force_range([-force], [force], [slider_index])
            lower, upper = self.robot.get_dofs_limit([slider_index])
            self._slider_limits = (float(self._numpy(lower)[0]), float(self._numpy(upper)[0]))
        # Apply initial positions specified in the configuration file.
        if self.config.initial_positions:
            self.set_joint_positions(self.config.initial_positions, immediate=True)
        # USD pose alone does not enable a position hold for every imported DOF.
        # Keep the unconfigured fingers and other public joints at their startup positions.
        joint_state = self.get_joint_state()
        if joint_state.names:
            public_indices = [self._public_joint_indices[name] for name in joint_state.names]
            self.robot.control_dofs_position(joint_state.positions, public_indices)
        # Snapshot the true initial startup home pose across all public controllable joints.
        self._home_positions = {
            name: float(self.config.initial_positions.get(name, pos))
            for name, pos in zip(joint_state.names, joint_state.positions, strict=True)
        }
        # Synchronize finger follower joints with their corresponding root joint poses.
        self._sync_finger_mimics()

    def _attach_cameras(self) -> None:
        """Attach configured cameras to their named robot links.
        :return: None after attaching all cameras.
        :rtype: None
        """
        # Attach each configured camera sensor to its parent mounting link using its relative pose.
        for camera_config in self.config.cameras:
            try:
                parent = self.get_link(camera_config.parent_link)
            except KeyError as error:
                raise RuntimeError(
                    f"Camera {camera_config.name!r} parent link {camera_config.parent_link!r} was not found"
                ) from error
            self._cameras[camera_config.name].attach(parent, np.asarray(camera_config.transform))

    def _selection(self, values: Mapping[str, float]) -> tuple[list[float], list[int]]:
        """Resolve public joint targets to articulation DOF indices.
        :param values: Targets keyed by participant-facing joint name.
        :type values: Mapping[str, float]
        :return: Target values and matching local DOF indices.
        :rtype: tuple[list[float], list[int]]
        """
        public_indices = self._public_joint_indices
        # Check for any unrecognized public joint names in the command mapping.
        unknown = sorted(set(values) - public_indices.keys())
        if unknown:
            raise KeyError(f"Unknown MH3 joints: {unknown}")
        names = list(values)
        # Return parallel lists of target values and local DOF indices.
        return [float(values[name]) for name in names], [public_indices[name] for name in names]

    @property
    def _public_joint_usd_names(self) -> dict[str, str]:
        """Map public joint names to their internal USD names.
        :return: Collision-checked public-to-USD joint mapping.
        :rtype: dict[str, str]
        """
        result = {
            _public_joint_name(name): name
            for name in self._joint_indices
            if name not in _INTERNAL_BASE_JOINTS and name not in self._finger_mimics
        }
        expected = (
            len(self._joint_indices)
            - len(_INTERNAL_BASE_JOINTS.intersection(self._joint_indices))
            - len(self._finger_mimics)
        )
        if len(result) != expected:
            raise RuntimeError("MH3 joint names collide after simplification")
        return result

    @property
    def _public_joint_indices(self) -> dict[str, int]:
        """Map public joint names to local articulation DOF indices.
        :return: Public-to-local-DOF mapping.
        :rtype: dict[str, int]
        """
        return {name: self._joint_indices[usd_name] for name, usd_name in self._public_joint_usd_names.items()}

    @property
    def _public_q_indices(self) -> dict[str, int]:
        """Map public joint names to local articulation qpos coordinate indices.
        :return: Public-to-local-qpos mapping.
        :rtype: dict[str, int]
        """
        return {name: self._q_indices[usd_name] for name, usd_name in self._public_joint_usd_names.items()}

    @property
    def _public_links(self) -> dict[str, Any]:
        """Map readable link names to native Genesis links.
        :return: Collision-checked public link mapping.
        :rtype: dict[str, Any]
        """
        links = {_public_link_name(name): link for name, link in self._links.items()}
        if len(links) != len(self._links):
            raise RuntimeError("MH3 link names collide after simplification")
        return links

    @property
    def sim_time(self) -> float:
        """Return elapsed simulated time in seconds.
        :return: Simulated time in seconds.
        :rtype: float
        """
        return self._sim_time

    @property
    def joint_names(self) -> tuple[str, ...]:
        """List joints available through the participant-facing API.
        :return: Public names of controllable joints.
        :rtype: tuple[str, ...]
        """
        return tuple(self._public_joint_indices)

    @property
    def home_positions(self) -> dict[str, float]:
        """Return the startup home positions for all controllable joints.
        :return: Copy of configured or default startup joint positions.
        :rtype: dict[str, float]
        """
        return dict(self._home_positions)

    @property
    def link_names(self) -> tuple[str, ...]:
        """List links available to link lookup and inverse kinematics.
        :return: Public names of robot links.
        :rtype: tuple[str, ...]
        """
        return tuple(self._public_links)

    @property
    def camera_names(self) -> tuple[str, ...]:
        """List names of configured on-robot cameras.
        :return: Public camera names.
        :rtype: tuple[str, ...]
        """
        return tuple(self._cameras)

    def get_joint_state(self, names: Iterable[str] | None = None) -> JointState:
        """Read positions, velocities, and efforts for selected joints.
        :param names: Public joint names, or None for every controllable joint.
        :type names: Iterable[str] | None
        :return: Copied joint state in requested order.
        :rtype: JointState
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        # Default to querying all participant-controllable joints if no subset is specified.
        selected = tuple(names) if names is not None else self.joint_names
        # Reject any requested joint names that are not part of the public vocabulary.
        unknown = sorted(set(selected) - set(self.joint_names))
        if unknown:
            raise KeyError(f"Unknown MH3 joints: {unknown}")
        # Resolve public joint names to local articulation DOF indices.
        indices = [self._public_joint_indices[name] for name in selected]
        # Query current kinematic and dynamic state arrays and package them into a JointState.
        return JointState(
            names=selected,
            positions=self._numpy(self.robot.get_dofs_position(indices)),
            velocities=self._numpy(self.robot.get_dofs_velocity(indices)),
            efforts=self._numpy(self.robot.get_dofs_force(indices)),
        )

    def get_joint_limits(self, names: Iterable[str] | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Read lower and upper position limits for selected joints.
        :param names: Public joint names, or None for every controllable joint.
        :type names: Iterable[str] | None
        :return: Lower and upper position-limit arrays in requested order.
        :rtype: tuple[np.ndarray, np.ndarray]
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        # Default to querying all participant-controllable joints if no subset is specified.
        selected = tuple(names) if names is not None else self.joint_names
        # Check for unknown joint names in the query selection.
        unknown = sorted(set(selected) - set(self.joint_names))
        if unknown:
            raise KeyError(f"Unknown MH3 joints: {unknown}")
        indices = [self._public_joint_indices[name] for name in selected]
        # Query lower and upper physical motion bounds from the Genesis articulation.
        lower, upper = self.robot.get_dofs_limit(indices)
        return self._numpy(lower), self._numpy(upper)

    def get_link(self, name: str) -> Any:
        """Look up a native Genesis link by its public name.
        :param name: Participant-facing link name.
        :type name: str
        :return: Native Genesis link object.
        :rtype: Any
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        links = self._public_links
        # Validate that the requested link name exists in the public link mapping.
        if name not in links:
            raise KeyError(f"Unknown MH3 link {name!r}; choose from sim.link_names")
        return links[name]

    def get_link_pose(
        self,
        name: str,
        frame: str | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Read the position and orientation of a link in world space or a reference frame.
        :param name: Public link name to inspect.
        :type name: str
        :param frame: Reference link name, or None for world space.
        :type frame: str | None
        :return: Tuple containing position (x, y, z) and quaternion (w, x, y, z).
        :rtype: tuple[np.ndarray, np.ndarray]
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        link = self.get_link(name)
        world_pos = self._numpy(link.get_pos())
        world_quat = self._numpy(link.get_quat())
        # Return world-space pose if no relative reference frame is requested.
        if frame is None or frame == "world":
            return world_pos, world_quat
        ref_link = self.get_link(frame)
        ref_pos = self._numpy(ref_link.get_pos())
        ref_quat = self._numpy(ref_link.get_quat())
        # Invert reference quaternion by conjugating its vector components.
        ref_quat_inv = np.array([ref_quat[0], -ref_quat[1], -ref_quat[2], -ref_quat[3]], dtype=float)
        norm = float(np.linalg.norm(ref_quat_inv))
        if norm > 0.0:
            ref_quat_inv /= norm
        # Rotate world translation into the reference coordinate frame.
        rel_pos = _quat_rotate(ref_quat_inv, world_pos - ref_pos)
        # Multiply inverse reference quaternion by world quaternion.
        rel_quat = _quat_multiply(ref_quat_inv, world_quat)
        return rel_pos, rel_quat

    def get_camera(self, name: str) -> Any:
        """Look up a native Genesis camera for advanced sensor operations.
        :param name: Configured public camera name.
        :type name: str
        :return: Native Genesis camera object.
        :rtype: Any
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        # Validate that the camera name is registered in the scene.
        if name not in self._cameras:
            raise KeyError(f"Unknown camera {name!r}; choose from {self.camera_names}")
        return self._cameras[name]

    def set_joint_positions(self, targets: Mapping[str, float], *, immediate: bool = False) -> None:
        """Command joint positions or teleport them immediately.
        :param targets: Position targets keyed by public joint name.
        :type targets: Mapping[str, float]
        :param immediate: Teleport joints when True instead of moving through control.
        :type immediate: bool
        :return: None after submitting position targets.
        :rtype: None
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        values, indices = self._selection(targets)
        if not indices:
            return
        # Translate public joint targets into internal USD articulation joint names.
        usd_names = self._public_joint_usd_names
        internal_targets = {usd_names[name]: float(value) for name, value in targets.items()}
        # Expand finger root targets to automatically include dependent mimic followers.
        expanded = self._expanded_finger_targets(internal_targets)
        values = [float(value) for value in expanded.values()]
        indices = [self._joint_indices[name] for name in expanded]
        # Inspect and validate optional torso slider target position.
        slider_index = self._joint_indices.get("slider_joint")
        slider_target = targets.get("slider")
        if slider_target is not None:
            slider_target = float(slider_target)
            if not np.isfinite(slider_target):
                raise ValueError("Slider target must be finite")
            if self._slider_limits is not None:
                lower, upper = self._slider_limits
                if not lower <= slider_target <= upper:
                    raise ValueError(f"Slider target must be between {lower} and {upper} m")
        # Teleport DOFs immediately to their targets without physical interpolation when requested.
        if immediate:
            self.robot.set_dofs_position(values, indices)
            self.robot.control_dofs_position(values, indices)
            if slider_target is not None:
                self._slider_target = slider_target
                self._slider_reference = slider_target
            return
        # Intercept slider target to handle rate-limited position ramping during step updates.
        if slider_target is not None:
            measured = float(self._numpy(self.robot.get_dofs_position([slider_index]))[0])
            self._slider_target = slider_target
            self._slider_reference = measured
            commands = [(value, index) for value, index in zip(values, indices, strict=True) if index != slider_index]
            values = [value for value, _ in commands]
            indices = [index for _, index in commands]
        # Command standard joint position controller targets for remaining DOFs.
        if indices:
            self.robot.control_dofs_position(values, indices)

    def _advance_slider_reference(self) -> None:
        """Advance the bounded torso lift reference by one physics step.
        :return: None after commanding the next lift reference.
        :rtype: None
        """
        # Return if no active slider target has been set.
        if self._slider_target is None:
            return
        slider_index = self._joint_indices["slider_joint"]
        # Read the current physical position of the torso slider.
        measured = float(self._numpy(self.robot.get_dofs_position([slider_index]))[0])
        reference = measured if self._slider_reference is None else self._slider_reference
        # Step the reference position towards the goal within maximum speed constraints.
        max_step = self.config.slider_max_velocity_mps * self.config.dt
        reference += min(max(self._slider_target - reference, -max_step), max_step)
        # Prevent the position setpoint from leading measured position beyond the configured lead limit.
        lead = self.config.slider_max_reference_lead_m
        reference = min(max(reference, measured - lead), measured + lead)
        # Clamp reference within physical slider travel limits.
        if self._slider_limits is not None:
            lower, upper = self._slider_limits
            reference = min(max(reference, lower), upper)
        self._slider_reference = reference
        # Command updated slider reference position to the actuator.
        self.robot.control_dofs_position([reference], [slider_index])

    def set_joint_velocities(self, targets: Mapping[str, float]) -> None:
        """Command velocities for selected controllable joints.
        :param targets: Velocity targets keyed by public joint name.
        :type targets: Mapping[str, float]
        :return: None after submitting velocity targets.
        :rtype: None
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        # Resolve target velocities to DOF indices and apply them to actuators.
        values, indices = self._selection(targets)
        if indices:
            self.robot.control_dofs_velocity(values, indices)

    def drive_base(self, linear_mps: float, angular_radps: float) -> None:
        """Drive the mobile base with forward and yaw velocities.
        :param linear_mps: Forward speed in metres per second.
        :type linear_mps: float
        :param angular_radps: Yaw speed in radians per second.
        :type angular_radps: float
        :return: None after commanding the internal drive wheels.
        :rtype: None
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        # Clamp linear and angular commands to configured base speed boundaries.
        linear_mps = min(
            max(float(linear_mps), -self.config.max_linear_velocity_mps), self.config.max_linear_velocity_mps
        )
        angular_radps = min(
            max(float(angular_radps), -self.config.max_angular_velocity_radps),
            self.config.max_angular_velocity_radps,
        )
        # Compute differential drive wheel speeds based on wheel radius and axle width.
        half_track = self.config.wheel_separation_m * 0.5
        radius = self.config.wheel_radius_m
        targets = {
            _LEFT_WHEEL: (linear_mps - angular_radps * half_track) / radius,
            _RIGHT_WHEEL: (linear_mps + angular_radps * half_track) / radius,
        }
        # Verify wheel joints exist in the loaded robot model.
        missing = sorted(set(targets) - set(self._joint_indices))
        if missing:
            raise RuntimeError(f"MH3 base joints were not found: {missing}")
        names = list(targets)
        # Command rotational velocities to left and right drive wheels.
        self.robot.control_dofs_velocity(
            [targets[name] for name in names],
            [self._joint_indices[name] for name in names],
        )

    def stop_base(self) -> None:
        """Stop the driven wheels through the base drive API.
        :return: None after commanding zero base velocity.
        :rtype: None
        """
        # Set base linear and angular velocities to zero.
        self.drive_base(0.0, 0.0)

    def set_joint_efforts(self, targets: Mapping[str, float]) -> None:
        """Command efforts for selected controllable joints.
        :param targets: Effort targets keyed by public joint name.
        :type targets: Mapping[str, float]
        :return: None after submitting effort targets.
        :rtype: None
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        # Resolve target efforts to DOF indices and apply direct forces/torques.
        values, indices = self._selection(targets)
        if indices:
            self.robot.control_dofs_force(values, indices)

    def solve_ik(
        self,
        end_effector: str,
        position: Iterable[float],
        quaternion_wxyz: Iterable[float] | None = None,
        *,
        frame: str | None = None,
        joints: Iterable[str] | None = None,
        apply: bool = False,
        **solver_options: Any,
    ) -> np.ndarray:
        """Solve IK for an end-effector pose expressed in world space or a link frame.
        :param end_effector: Public link name of the target end effector.
        :type end_effector: str
        :param position: Target XYZ position in the specified reference frame.
        :type position: Iterable[float]
        :param quaternion_wxyz: Optional reference-frame WXYZ orientation target.
        :type quaternion_wxyz: Iterable[float] | None
        :param frame: Reference link name for the target pose, or None for world space.
        :type frame: str | None
        :param joints: Optional public joint names to vary during solving.
        :type joints: Iterable[str] | None
        :param apply: Command joint controllers to the solved targets when True.
        :type apply: bool
        :param solver_options: Additional Genesis inverse-kinematics options.
        :type solver_options: Any
        :return: Full articulation position solution as a NumPy array.
        :rtype: np.ndarray
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        link = self.get_link(end_effector)
        pos_target = np.asarray(tuple(position), dtype=float)
        quat_target = None if quaternion_wxyz is None else np.asarray(tuple(quaternion_wxyz), dtype=float)
        # Transform target pose from local reference link frame to world coordinates if specified.
        if frame is not None and frame != "world":
            ref_link = self.get_link(frame)
            ref_pos = self._numpy(ref_link.get_pos())
            ref_quat = self._numpy(ref_link.get_quat())
            # Rotate position offset into world orientation and translate by reference link position.
            pos_target = ref_pos + _quat_rotate(ref_quat, pos_target)
            # Compose relative orientation with reference link world orientation if orientation is supplied.
            if quat_target is not None:
                quat_target = _quat_multiply(ref_quat, quat_target)
        dofs = None
        # Convert user-selected joint names to local articulation DOF indices if constrained.
        if joints is not None:
            joint_names = tuple(joints)
            unknown = sorted(set(joint_names) - set(self.joint_names))
            if unknown:
                raise KeyError(f"Unknown MH3 joints: {unknown}")
            dofs = [self._public_joint_indices[name] for name in joint_names]
        # Seed solver with current articulation configuration unless custom init_qpos is specified.
        options = dict(solver_options)
        if "init_qpos" not in options:
            options["init_qpos"] = self.robot.get_qpos()
        # Enforce high-precision convergence criteria for consistent kinematic solutions.
        if "pos_tol" not in options:
            options["pos_tol"] = 1e-4
        if "rot_tol" not in options:
            options["rot_tol"] = 1e-4
        if "max_solver_iters" not in options:
            options["max_solver_iters"] = 40
        # Invoke Genesis numerical inverse kinematics solver.
        solution = self.robot.inverse_kinematics(
            link=link,
            pos=pos_target,
            quat=quat_target,
            dofs_idx_local=dofs,
            **options,
        )
        # Apply the solved configuration through joint position controllers so the robot moves physically.
        if apply:
            solution_arr = self._numpy(solution)
            target_joints = tuple(joints) if joints is not None else self.joint_names
            targets = {name: float(solution_arr[self._public_q_indices[name]]) for name in target_joints}
            self.set_joint_positions(targets)
        return self._numpy(solution)

    def capture_camera(
        self,
        name: str,
        *,
        rgb: bool = True,
        depth: bool = True,
        segmentation: bool = False,
        normal: bool = False,
    ) -> CameraFrame:
        """Render selected channels from a named on-robot camera.
        :param name: Public camera name to render.
        :type name: str
        :param rgb: Include the RGB image when True.
        :type rgb: bool
        :param depth: Include the depth image when True.
        :type depth: bool
        :param segmentation: Include segmentation when True.
        :type segmentation: bool
        :param normal: Include surface normals when True.
        :type normal: bool
        :return: Camera frame containing the requested NumPy arrays.
        :rtype: CameraFrame
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        camera = self.get_camera(name)
        # Update camera position to follow its attached mounting link pose.
        camera.move_to_attach()
        # Render the requested visual channels.
        rendered = camera.render(rgb=rgb, depth=depth, segmentation=segmentation, normal=normal)
        # Convert rendered Genesis tensors into standard NumPy arrays.
        values = [self._numpy(value, None) if value is not None else None for value in rendered]
        return CameraFrame(*values)

    def capture_pointcloud(self, name: str, *, world_frame: bool = True) -> tuple[np.ndarray, np.ndarray]:
        """Render a camera point cloud and its valid-depth mask.
        :param name: Public camera name to render.
        :type name: str
        :param world_frame: Express points in world coordinates when True.
        :type world_frame: bool
        :return: Point coordinates and valid-depth mask as NumPy arrays.
        :rtype: tuple[np.ndarray, np.ndarray]
        """
        camera = self.get_camera(name)
        # Update camera position to follow its attached mounting link pose.
        camera.move_to_attach()
        # Render 3D point cloud coordinates and validity boolean mask.
        points, mask = camera.render_pointcloud(world_frame=world_frame)
        return self._numpy(points, None), self._numpy(mask, None)

    def add_step_hook(self, hook: StepHook) -> None:
        """Register a callback to run after every physics step.
        :param hook: Callback receiving this simulation after each step.
        :type hook: StepHook
        :return: None after registering the callback.
        :rtype: None
        """
        # Append step hook callback to the list of per-step observers.
        self._hooks.append(hook)

    def add_scene_hook(self, hook: SceneHook) -> None:
        """Register a callback to run before the Genesis scene is built.
        Use scene hooks to add geometry or sensors through native Genesis APIs.
        :param hook: Callback receiving this simulation before scene build.
        :type hook: SceneHook
        :return: None after registering the callback.
        :rtype: None
        """
        # Scene hooks must be registered prior to scene compilation.
        if self.scene is not None:
            raise RuntimeError("Scene hooks must be registered before build()")
        self._scene_hooks.append(hook)

    def step(self) -> None:
        """Advance joint controls and physics by one simulation time step.
        :return: None after updating physics and calling step hooks.
        :rtype: None
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        # Progress the rate-limited torso lift reference position.
        self._advance_slider_reference()
        # Synchronize dependent finger segments with their root joints.
        self._sync_finger_mimics()
        # Advance the physical simulation by one time step.
        self.scene.step()
        # Increment internal simulation clock by the time step duration.
        self._sim_time += self.config.dt
        # Execute registered step callbacks with this simulation instance.
        for hook in tuple(self._hooks):
            hook(self)

    def run(self, *, steps: int | None = None) -> None:
        """Run until interrupted or until a fixed number of steps completes.
        :param steps: Number of steps to execute, or None to run indefinitely.
        :type steps: int | None
        :return: None when the requested run completes.
        :rtype: None
        """
        # Ensure the simulation scene is constructed.
        self._require_built()
        next_step = time.monotonic()
        completed = 0
        # Loop until either the step limit is satisfied or the simulation is stopped.
        while steps is None or completed < steps:
            self.step()
            completed += 1
            # Maintain wall-clock synchronization if realtime simulation pacing is enabled.
            if self.config.realtime:
                next_step += self.config.dt
                delay = next_step - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
