# MH3 Genesis Hackathon Starter

This repository is a self-contained, ROS-free starting point for building projects with the [Mirsee Robotics](https://mirsee.com "Mirsee Robotics Inc.") MH3 in [Genesis](https://genesis-embodied-ai.github.io/). It includes the simulated MH3, a default world with a floor, a browser dashboard, and a Python API for controlling and observing the robot.

## Setup and run

Run the setup script for your platform. It installs uv locally if needed, downloads the pinned Python automatically, installs the frozen release, selects compatible accelerator dependencies, and verifies the bundled robot, physics, and a camera's RGB/depth rendering. No system Python, ROS installation, administrator access, or terminal restart is needed for the Python environment.

```bash
# Linux x64 or Apple Silicon macOS
bash setup.sh
./run.sh
```

```powershell
# Windows x64; the process-only policy handles downloaded/unsigned scripts.
powershell -NoProfile -ExecutionPolicy Bypass -File .\setup.ps1
.\run.ps1
```

This release targets Windows 10+ x64, Linux x64 with glibc 2.28+, and Apple Silicon with macOS 14+. Intel Macs, native Windows ARM, Linux ARM, 32-bit systems, and Alpine/musl lack wheels required by the pinned dependency stack. Setup checks these limits before installing simulation packages. These are dependency-supported targets; verify each intended machine before the event. Windows on ARM running an x64 environment has not been validated.

Internet access and several GB of free disk space are needed for the initial download. Linux needs `curl` or `wget` for uv bootstrapping. A working graphics driver and OpenGL/EGL runtime are needed for the viewer and cameras, including when physics runs on the CPU. GPU drivers and OS graphics libraries are system prerequisites: setup reports recovery instructions rather than installing system software. On Ubuntu/Debian the graphics runtime packages include `libgl1`, `libegl1`, and `libglib2.0-0`; on WSL use WSLg and updated Windows GPU drivers. See [Genesis installation requirements](https://genesis-world.readthedocs.io/en/latest/user_guide/overview/installation.html).

Setup uses `.python-version` and leaves `pyproject.toml` and `uv.lock` unchanged. On fresh installations, PyTorch-only accelerator libraries are deferred until hardware selection, avoiding unnecessary CUDA 13 downloads on CPU-only machines. Genesis's own native dependencies are still installed. A working, compatible PyTorch accelerator wheel is kept. If another accelerator is needed, its complete dependency graph is resolved while the release's non-accelerator versions remain pinned. If acceleration fails, setup tries CPU physics; graphics failures still need the system graphics runtime repaired. Use `bash setup.sh --cpu` or `powershell -NoProfile -ExecutionPolicy Bypass -File .\setup.ps1 --cpu` to request CPU PyTorch and physics explicitly.

The verified backend, installed package versions, dependency constraints, and camera preview are recorded locally in `.setup/`, which is excluded from Git. Run scripts reuse a verified CPU fallback unless you supply `--backend` explicitly. Repeated setup reuses the verified environment when the release and installed package versions match, and repeats the simulator check. Use `--reset` to restore the frozen release and redetect hardware after a driver or hardware change. Local uv lives in `.uv/`, also excluded from Git; setup, run, and upgrade find it directly without changing your shell profile. Its cache and managed Python downloads use uv's normal user directories.

The first simulator verification may take several minutes to compile physics and rendering kernels. A successful check saves `.setup/preview.png` and reports the actual Genesis backend. The check verifies offscreen rendering; use the run script to confirm the interactive viewer on a local desktop. The run scripts forward arguments such as `--headless`, `--backend cpu`, or `--no-controls`.

## Upgrade dependencies (maintainers)

Participants should run **setup**, which installs the distributed, frozen release. Maintainers should run `./upgrade.sh` or `.\upgrade.ps1` before the event, then manually validate the updated release on the intended machines before distributing it.

The wrappers bootstrap uv if needed and run `upgrade.py` from the repository root. It discovers the latest stable, non-yanked Genesis release and newest stable CPython supported by that release. uv resolves the combined runtime, development, optional, and build dependencies, trying an older supported Python minor if the newest cannot resolve the complete set. It removes stale direct package bounds, preserves extras such as USD support, and records compatible direct versions in `pyproject.toml`, transitive versions in `uv.lock`, and the Python patch in `.python-version`. The Python requirement and Ruff target are updated together. Genesis's requirements may keep some packages below their latest standalone releases.

Use `--dry-run` to preview without changing project files or `.venv`, or `--lock-only` to update the three release files without installing. Python and bootstrap dependencies may be downloaded into uv's cache during a preview. Resolution takes place in a temporary directory; failure leaves the existing release files intact. Once files are updated, the default upgrade invokes the same setup and simulator verification used by participants. If installation or verification fails, the updated files remain available for retrying setup. A universal lock resolution does not prove wheel availability, graphics support, or hardware compatibility on every platform; repeat the manual platform checks after upgrading.

## Browser dashboard

The dashboard starts automatically in your default browser when `run.sh` or `run.ps1` starts the simulator. Hold the arrow buttons or W/A/S/D to drive the base, which defaults to the configured maximum linear and angular speeds; lower either speed slider for finer control. Use the joint sliders or numeric inputs to pose the arms, head, and hands, and select **Return all joints home** to restore the configured startup pose.

The camera panel shows the head's left RGB, right RGB, and depth feeds above the matching base camera feeds. In each depth view, nearby surfaces are bright and distances beyond 5 m are dark; all six feeds update while the dashboard is open.

The torso slider travels from 0 to 0.3 m at up to 0.008 m/s when commanded from the dashboard or with `sim.set_joint_positions()`. Its position reference stays close to measured motion, and the simulator limits the lift force; these values can be adjusted in `config/mh3.yaml`. All other controllable joints hold their startup poses until commanded.

Each hand exposes six controls: two thumb joints and one joint for each other finger. The remaining finger segments follow their corresponding control automatically, so they do not appear as separate dashboard sliders or public joint names.

All hand controls use radians, with zero representing the fully open pose. The four finger controls close through 1.57 rad; the thumb controls travel through 1.6 and 0.45 rad. The bundled USD's hand transforms are aligned at joint zero for Genesis, including the fingertip sensors.

Public joint names are short and consistent: `left_1` through `left_7` and `right_1` through `right_7` for the arms, `left_index` and similar names for the fingers, `left_thumb_1`/`left_thumb_2` and their right-hand equivalents for the thumbs, plus `slider`, `head_yaw`, and `head_pitch`. Use `sim.joint_names` to see the full list.

Links use the same readable style: `left_arm_1` through `left_arm_7` for arm segments, `left_gripper_base` for the left gripper's base, and `left_index_tip` for the fingertip; right-side names follow the same pattern. Use `sim.link_names` to discover all links before passing one to `sim.get_link()` or `sim.solve_ik()`.

Pass `--no-controls` to the run script to start without the dashboard. Closing the browser tab only closes the page; stop the simulator with Ctrl+C, or use `panel.close()` when embedding the dashboard in a custom script.

To add the dashboard to a custom control script, create it after `build()`, register its `update` method as a step hook, and close it when the simulation exits:

```python
from mh3_genesis import MH3ControlPanel, MH3Simulation, SimulatorConfig

sim = MH3Simulation(SimulatorConfig.from_yaml("config/mh3.yaml")).build()
panel = MH3ControlPanel(sim)
panel.start()  # Use open_browser=False to print the local URL without opening it.
sim.add_step_hook(panel.update)

try:
    sim.run()
finally:
    panel.close()
```

The dashboard listens only on `127.0.0.1` and chooses an available port automatically. Its commands are queued by the web server and applied safely by `panel.update()` on the simulation thread.

## Using the API

Import `MH3Simulation` in your own script and use the simulator directly:

```python
from mh3_genesis import MH3Simulation, SimulatorConfig

config = SimulatorConfig.from_yaml("config/mh3.yaml")
sim = MH3Simulation(config).build()

# Discover the participant-controllable joints and available cameras.
print(sim.compute_backend)
print(sim.joint_names)
print(sim.link_names)
print(sim.camera_names)

# Joint position, velocity, and effort commands are dictionaries keyed by joint name.
sim.set_joint_positions({"head_yaw": 0.4, "head_pitch": -0.2})
sim.set_joint_efforts({"left_1": 1.0})

# Drive the base in metres per second and radians per second.
sim.drive_base(linear_mps=0.25, angular_radps=0.0)
sim.stop_base()

# State and sensor data are returned as NumPy arrays, with no middleware.
state = sim.get_joint_state(["head_yaw", "head_pitch"])
frame = sim.capture_camera("head_left", rgb=True, depth=True)
print(state.positions, frame.rgb.shape, frame.depth.shape)
points, valid_depth = sim.capture_pointcloud("head_left")

# Genesis' native IK solver can be restricted to one arm.
left_arm = [f"left_{index}" for index in range(1, 8)]
qpos = sim.solve_ik(
    "left_gripper_base",
    position=(0.55, 0.30, 0.90),
    joints=left_arm,
    apply=True,
)

sim.run()
```

`drive_base()` and `stop_base()` allow you to drive the robot. `solve_ik` accepts an optional world-frame `quaternion_wxyz`, plus any Genesis IK keyword such as `pos_tol`, `rot_tol`, or `max_solver_iters`. By default it only returns a solution; `apply=True` teleports the articulation to it.

For event-style projects, register any number of per-step callbacks:

```python
def controller(sim: MH3Simulation) -> None:
    """Return the head to center after one simulated second.
    :param sim: Simulation invoking this step hook.
    :type sim: MH3Simulation
    :return: None after submitting a head target when applicable.
    :rtype: None
    """
    if sim.sim_time > 1.0:
        sim.set_joint_positions({"head_yaw": 0.0})

sim.add_step_hook(controller)
sim.run()
```

Scene hooks add world geometry before Genesis compiles the scene:

```python
sim = MH3Simulation(config)

def add_world(sim: MH3Simulation) -> None:
    """Add a box to the scene before Genesis builds it.
    :param sim: Simulation providing the unbuilt Genesis scene.
    :type sim: MH3Simulation
    :return: None after adding the box.
    :rtype: None
    """
    sim.scene.add_entity(sim.gs.morphs.Box(pos=(1.0, 0.0, 0.25), size=(0.5, 0.5, 0.5)))

sim.add_scene_hook(add_world)
sim.build().run()
```

Advanced projects can also use `sim.scene`, `sim.robot`, `sim.floor`, and `sim.gs` for any other native Genesis feature.
