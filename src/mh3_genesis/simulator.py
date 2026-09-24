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
    """Translate a USD joint name into the small participant-facing vocabulary."""
    name = name.removesuffix("_joint")
    for prefix, replacement in (("L_link_", "left_"), ("R_link_", "right_"), ("L_", "left_"), ("R_", "right_")):
        if name.startswith(prefix):
            part = name[len(prefix) :]
            if prefix in ("L_", "R_") and part.startswith(("index_", "middle_", "ring_", "little_")):
                part = part.removesuffix("_1")
            return replacement + part
    return name


def _public_link_name(name: str) -> str:
    """Give imported robot links predictable names without changing the USD."""
    special = {
        "base_link_head": "head_base",
        "base_link_torso": "torso_base",
        "head_camera_base_frame": "head_camera_mount",
    }
    if name in special:
        return special[name]
    name = name.removesuffix("_link")
    for prefix, replacement in (("L_link_", "left_arm_"), ("R_link_", "right_arm_"), ("L_", "left_"), ("R_", "right_")):
        if name.startswith(prefix):
            name = replacement + name[len(prefix) :]
            break
    name = name.replace("liitle", "little").replace("miidle", "middle")
    name = name.replace("_F_sensor", "_force_sensor")
    return name.removesuffix("_ee") + "_tip" if name.endswith("_ee") else name


def _select_genesis_backend(gs: Any, requested: str) -> Any:
    """Map configuration names to Genesis backends without pre-empting its GPU detection."""
    return {
        "auto": gs.gpu,
        "gpu": gs.gpu,
        "cpu": gs.cpu,
        "cuda": gs.cuda,
        "amdgpu": gs.amdgpu,
        "metal": gs.metal,
    }[requested]


@dataclass(frozen=True)
class JointState:
    """A copy of the current simulated joint state."""

    names: tuple[str, ...]
    positions: np.ndarray
    velocities: np.ndarray
    efforts: np.ndarray


@dataclass(frozen=True)
class CameraFrame:
    """Images returned directly by a Genesis camera."""

    rgb: np.ndarray | None = None
    depth: np.ndarray | None = None
    segmentation: np.ndarray | None = None
    normal: np.ndarray | None = None


class MH3Simulation:
    """Small, ROS-free API around the MH3 Genesis articulation.

    Call :meth:`build` once, then use the joint, camera, and IK methods from a
    participant's control loop. Genesis objects remain available as ``scene``
    and ``robot`` for projects that need lower-level functionality.
    """

    def __init__(self, config: SimulatorConfig) -> None:
        self.config = config
        self.gs: Any = None
        self.scene: Any = None
        self.floor: Any = None
        self.robot: Any = None
        self._joint_indices: dict[str, int] = {}
        self._finger_mimics: dict[str, tuple[str, float, float]] = {}
        self._links: dict[str, Any] = {}
        self._cameras: dict[str, Any] = {}
        self._scene_hooks: list[SceneHook] = []
        self._hooks: list[StepHook] = []
        self._sim_time = 0.0
        self._slider_target: float | None = None
        self._slider_reference: float | None = None
        self._slider_limits: tuple[float, float] | None = None
        self.compute_backend: str | None = None

    @staticmethod
    def _numpy(value: Any, dtype: Any | None = np.float64) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        return np.asarray(value, dtype=dtype)

    def build(self) -> MH3Simulation:
        """Initialize Genesis and load a floor plane and the articulated MH3 USD."""
        import genesis as gs

        self.gs = gs
        backend = _select_genesis_backend(gs, self.config.backend)
        gs.init(backend=backend, logging_level="warning")
        self.compute_backend = gs.backend.name
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
        self.floor = self.scene.add_entity(gs.morphs.Plane())

        entities = self.scene.add_stage(
            gs.morphs.USD(
                file=str(self.config.robot_usd),
                pos=self.config.spawn_position,
                euler=self.config.spawn_euler_deg,
            )
        )
        articulated = [entity for entity in entities if any(joint.n_dofs for joint in entity.joints)]
        if not articulated:
            raise RuntimeError(f"No articulation was found in {self.config.robot_usd}")
        self.robot = max(articulated, key=lambda entity: sum(joint.n_dofs for joint in entity.joints))

        for camera_config in self.config.cameras:
            self._cameras[camera_config.name] = self.scene.add_camera(
                res=camera_config.resolution,
                fov=camera_config.vertical_fov_deg,
                near=camera_config.near_m,
                far=camera_config.far_m,
                GUI=False,
            )

        for hook in tuple(self._scene_hooks):
            hook(self)
        self.scene.build()
        self._discover_joints()
        self._discover_finger_mimics()
        self._configure_actuators()
        self._attach_cameras()
        return self

    def _require_built(self) -> None:
        if self.robot is None:
            raise RuntimeError("Call build() before using the simulation")

    def _discover_joints(self) -> None:
        self._links = {link.name.rstrip("/").rsplit("/", 1)[-1]: link for link in self.robot.links}
        for joint in self.robot.joints:
            if joint.n_dofs == 1:
                name = joint.name.strip().rstrip("/").rsplit("/", 1)[-1]
                self._joint_indices[name] = int(joint.dofs_idx_local[0])

    def _discover_finger_mimics(self) -> None:
        """Read the follower chains authored in the MH3 USD."""
        from pxr import Usd

        stage = Usd.Stage.Open(str(self.config.robot_usd))
        if stage is None:
            raise RuntimeError(f"Could not open MH3 USD: {self.config.robot_usd}")
        self._finger_mimics = {}
        for prim in stage.Traverse():
            relation = prim.GetRelationship("physxMimicJoint:rotZ:referenceJoint")
            if not relation or not relation.GetTargets():
                continue
            follower = prim.GetName()
            reference = relation.GetTargets()[0].name
            if follower not in self._joint_indices or reference not in self._joint_indices:
                raise RuntimeError(f"MH3 finger mimic joint is missing: {reference} -> {follower}")
            gearing = prim.GetAttribute("physxMimicJoint:rotZ:gearing").Get()
            offset = prim.GetAttribute("physxMimicJoint:rotZ:offset").Get()
            # PhysX gearing uses the opposite sign to the URDF mimic multiplier.
            self._finger_mimics[follower] = (reference, -float(gearing), float(offset or 0.0))

    def _expanded_finger_targets(self, targets: Mapping[str, float]) -> dict[str, float]:
        expanded = dict(targets)
        pending = dict(self._finger_mimics)
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
        if not self._finger_mimics:
            return
        roots = tuple({reference for reference, _, _ in self._finger_mimics.values()} - self._finger_mimics.keys())
        indices = [self._joint_indices[name] for name in roots]
        positions = self._numpy(self.robot.get_dofs_position(indices))
        expanded = self._expanded_finger_targets(dict(zip(roots, positions, strict=True)))
        followers = tuple(name for name in self._finger_mimics if name in expanded)
        values = [expanded[name] for name in followers]
        follower_indices = [self._joint_indices[name] for name in followers]
        if immediate:
            self.robot.set_dofs_position(values, follower_indices)
        self.robot.control_dofs_position(values, follower_indices)

    def _configure_actuators(self) -> None:
        indices = tuple(self._joint_indices.values())
        if indices:
            self.robot.set_dofs_kp(np.full(len(indices), self.config.position_kp), indices)
            self.robot.set_dofs_kv(np.full(len(indices), self.config.position_kv), indices)
        slider_index = self._joint_indices.get("slider_joint")
        if slider_index is not None:
            self.robot.set_dofs_kp([self.config.slider_position_kp], [slider_index])
            self.robot.set_dofs_kv([self.config.slider_position_kv], [slider_index])
            force = self.config.slider_max_force_n
            self.robot.set_dofs_force_range([-force], [force], [slider_index])
            lower, upper = self.robot.get_dofs_limit([slider_index])
            self._slider_limits = (float(self._numpy(lower)[0]), float(self._numpy(upper)[0]))
        if self.config.initial_positions:
            self.set_joint_positions(self.config.initial_positions, immediate=True)
        # USD pose alone does not enable a position hold for every imported DOF.
        # Keep the unconfigured fingers and other public joints at their startup positions.
        joint_state = self.get_joint_state()
        if joint_state.names:
            public_indices = [self._public_joint_indices[name] for name in joint_state.names]
            self.robot.control_dofs_position(joint_state.positions, public_indices)
        self._sync_finger_mimics()

    def _attach_cameras(self) -> None:
        for camera_config in self.config.cameras:
            try:
                parent = self.get_link(camera_config.parent_link)
            except KeyError as error:
                raise RuntimeError(
                    f"Camera {camera_config.name!r} parent link {camera_config.parent_link!r} was not found"
                ) from error
            self._cameras[camera_config.name].attach(parent, np.asarray(camera_config.transform))

    def _selection(self, values: Mapping[str, float]) -> tuple[list[float], list[int]]:
        public_indices = self._public_joint_indices
        unknown = sorted(set(values) - public_indices.keys())
        if unknown:
            raise KeyError(f"Unknown MH3 joints: {unknown}")
        names = list(values)
        return [float(values[name]) for name in names], [public_indices[name] for name in names]

    @property
    def _public_joint_usd_names(self) -> dict[str, str]:
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
        return {name: self._joint_indices[usd_name] for name, usd_name in self._public_joint_usd_names.items()}

    @property
    def _public_links(self) -> dict[str, Any]:
        links = {_public_link_name(name): link for name, link in self._links.items()}
        if len(links) != len(self._links):
            raise RuntimeError("MH3 link names collide after simplification")
        return links

    @property
    def sim_time(self) -> float:
        return self._sim_time

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Names available through the participant-facing joint API."""
        return tuple(self._public_joint_indices)

    @property
    def link_names(self) -> tuple[str, ...]:
        """Names available through :meth:`get_link` and :meth:`solve_ik`."""
        return tuple(self._public_links)

    @property
    def camera_names(self) -> tuple[str, ...]:
        return tuple(self._cameras)

    def get_joint_state(self, names: Iterable[str] | None = None) -> JointState:
        self._require_built()
        selected = tuple(names) if names is not None else self.joint_names
        unknown = sorted(set(selected) - set(self.joint_names))
        if unknown:
            raise KeyError(f"Unknown MH3 joints: {unknown}")
        indices = [self._public_joint_indices[name] for name in selected]
        return JointState(
            names=selected,
            positions=self._numpy(self.robot.get_dofs_position(indices)),
            velocities=self._numpy(self.robot.get_dofs_velocity(indices)),
            efforts=self._numpy(self.robot.get_dofs_force(indices)),
        )

    def get_joint_limits(self, names: Iterable[str] | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Return lower and upper position limits for the selected joints."""
        self._require_built()
        selected = tuple(names) if names is not None else self.joint_names
        unknown = sorted(set(selected) - set(self.joint_names))
        if unknown:
            raise KeyError(f"Unknown MH3 joints: {unknown}")
        indices = [self._public_joint_indices[name] for name in selected]
        lower, upper = self.robot.get_dofs_limit(indices)
        return self._numpy(lower), self._numpy(upper)

    def get_link(self, name: str) -> Any:
        """Return a native Genesis link by its participant-facing name."""
        self._require_built()
        links = self._public_links
        if name not in links:
            raise KeyError(f"Unknown MH3 link {name!r}; choose from sim.link_names")
        return links[name]

    def get_camera(self, name: str) -> Any:
        """Return a native Genesis camera for advanced sensor operations."""
        self._require_built()
        if name not in self._cameras:
            raise KeyError(f"Unknown camera {name!r}; choose from {self.camera_names}")
        return self._cameras[name]

    def set_joint_positions(self, targets: Mapping[str, float], *, immediate: bool = False) -> None:
        """Command position targets, or teleport joints when ``immediate=True``."""
        self._require_built()
        values, indices = self._selection(targets)
        if not indices:
            return
        usd_names = self._public_joint_usd_names
        internal_targets = {usd_names[name]: float(value) for name, value in targets.items()}
        expanded = self._expanded_finger_targets(internal_targets)
        values = [float(value) for value in expanded.values()]
        indices = [self._joint_indices[name] for name in expanded]
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
        if immediate:
            self.robot.set_dofs_position(values, indices)
            self.robot.control_dofs_position(values, indices)
            if slider_target is not None:
                self._slider_target = slider_target
                self._slider_reference = slider_target
            return
        if slider_target is not None:
            measured = float(self._numpy(self.robot.get_dofs_position([slider_index]))[0])
            self._slider_target = slider_target
            self._slider_reference = measured
            commands = [(value, index) for value, index in zip(values, indices, strict=True) if index != slider_index]
            values = [value for value, _ in commands]
            indices = [index for _, index in commands]
        if indices:
            self.robot.control_dofs_position(values, indices)

    def _advance_slider_reference(self) -> None:
        if self._slider_target is None:
            return
        slider_index = self._joint_indices["slider_joint"]
        measured = float(self._numpy(self.robot.get_dofs_position([slider_index]))[0])
        reference = measured if self._slider_reference is None else self._slider_reference
        max_step = self.config.slider_max_velocity_mps * self.config.dt
        reference += min(max(self._slider_target - reference, -max_step), max_step)
        lead = self.config.slider_max_reference_lead_m
        reference = min(max(reference, measured - lead), measured + lead)
        if self._slider_limits is not None:
            lower, upper = self._slider_limits
            reference = min(max(reference, lower), upper)
        self._slider_reference = reference
        self.robot.control_dofs_position([reference], [slider_index])

    def set_joint_velocities(self, targets: Mapping[str, float]) -> None:
        self._require_built()
        values, indices = self._selection(targets)
        if indices:
            self.robot.control_dofs_velocity(values, indices)

    def drive_base(self, linear_mps: float, angular_radps: float) -> None:
        """Drive the mobile base using forward and yaw velocities."""
        self._require_built()
        linear_mps = min(
            max(float(linear_mps), -self.config.max_linear_velocity_mps), self.config.max_linear_velocity_mps
        )
        angular_radps = min(
            max(float(angular_radps), -self.config.max_angular_velocity_radps),
            self.config.max_angular_velocity_radps,
        )
        half_track = self.config.wheel_separation_m * 0.5
        radius = self.config.wheel_radius_m
        targets = {
            _LEFT_WHEEL: (linear_mps - angular_radps * half_track) / radius,
            _RIGHT_WHEEL: (linear_mps + angular_radps * half_track) / radius,
        }
        missing = sorted(set(targets) - set(self._joint_indices))
        if missing:
            raise RuntimeError(f"MH3 base joints were not found: {missing}")
        names = list(targets)
        self.robot.control_dofs_velocity(
            [targets[name] for name in names],
            [self._joint_indices[name] for name in names],
        )

    def stop_base(self) -> None:
        """Stop both driven wheels."""
        self.drive_base(0.0, 0.0)

    def set_joint_efforts(self, targets: Mapping[str, float]) -> None:
        self._require_built()
        values, indices = self._selection(targets)
        if indices:
            self.robot.control_dofs_force(values, indices)

    def solve_ik(
        self,
        end_effector: str,
        position: Iterable[float],
        quaternion_wxyz: Iterable[float] | None = None,
        *,
        joints: Iterable[str] | None = None,
        apply: bool = False,
        **solver_options: Any,
    ) -> np.ndarray:
        """Solve world-frame IK with Genesis and optionally apply the full q-position solution."""
        self._require_built()
        link = self.get_link(end_effector)
        dofs = None
        if joints is not None:
            joint_names = tuple(joints)
            unknown = sorted(set(joint_names) - set(self.joint_names))
            if unknown:
                raise KeyError(f"Unknown MH3 joints: {unknown}")
            dofs = [self._public_joint_indices[name] for name in joint_names]
        solution = self.robot.inverse_kinematics(
            link=link,
            pos=np.asarray(tuple(position), dtype=float),
            quat=None if quaternion_wxyz is None else np.asarray(tuple(quaternion_wxyz), dtype=float),
            dofs_idx_local=dofs,
            **solver_options,
        )
        if apply:
            self.robot.set_qpos(solution)
            self._sync_finger_mimics(immediate=True)
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
        """Render a named on-robot camera and return NumPy arrays directly."""
        self._require_built()
        camera = self.get_camera(name)
        camera.move_to_attach()
        rendered = camera.render(rgb=rgb, depth=depth, segmentation=segmentation, normal=normal)
        values = [self._numpy(value, None) if value is not None else None for value in rendered]
        return CameraFrame(*values)

    def capture_pointcloud(self, name: str, *, world_frame: bool = True) -> tuple[np.ndarray, np.ndarray]:
        """Render a point cloud and return ``(points, valid_depth_mask)``."""
        camera = self.get_camera(name)
        camera.move_to_attach()
        points, mask = camera.render_pointcloud(world_frame=world_frame)
        return self._numpy(points, None), self._numpy(mask, None)

    def add_step_hook(self, hook: StepHook) -> None:
        """Run ``hook(simulation)`` after every physics step."""
        self._hooks.append(hook)

    def add_scene_hook(self, hook: SceneHook) -> None:
        """Run ``hook(simulation)`` just before the Genesis scene is built.

        Register scene hooks before :meth:`build` to add floors, props, or
        additional sensors with the native ``sim.gs`` and ``sim.scene`` APIs.
        """
        if self.scene is not None:
            raise RuntimeError("Scene hooks must be registered before build()")
        self._scene_hooks.append(hook)

    def step(self) -> None:
        self._require_built()
        self._advance_slider_reference()
        self._sync_finger_mimics()
        self.scene.step()
        self._sim_time += self.config.dt
        for hook in tuple(self._hooks):
            hook(self)

    def run(self, *, steps: int | None = None) -> None:
        """Run until interrupted, or for exactly ``steps`` physics steps."""
        self._require_built()
        next_step = time.monotonic()
        completed = 0
        while steps is None or completed < steps:
            self.step()
            completed += 1
            if self.config.realtime:
                next_step += self.config.dt
                delay = next_step - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
