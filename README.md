# MH3 Genesis Hackathon Starter

This repository is a self-contained, Python-based starting point for building projects with the [Mirsee Robotics](https://mirsee.com "Mirsee Robotics Inc.") MH3 in [Genesis](https://genesis-embodied-ai.github.io/). It includes the simulated MH3, a default world with a floor, a browser dashboard, and a Python API for controlling and observing the robot.

## Setup and run

### Supported platforms

- **Linux x64** (glibc 2.28+)
- **Apple Silicon macOS** (macOS 14+)
- **Windows 10+ x64**

*(Intel Macs, ARM Linux/Windows, and 32-bit platforms are not supported.)*

### Prerequisites

- **Linux:** `curl` or `wget`, plus graphics runtime libraries (`libgl1`, `libegl1`, `libglib2.0-0` on Ubuntu/Debian). On WSL2, ensure WSLg and updated GPU drivers are installed.
- Working GPU drivers and OpenGL/EGL runtime (required for cameras and the viewer, even if physics runs on CPU). See [Genesis installation requirements](https://genesis-world.readthedocs.io/en/latest/user_guide/overview/installation.html).

### Installation & running

**Linux / Apple Silicon macOS:**
```bash
./setup.sh
./run.sh
```

**Windows x64 (PowerShell):**
```powershell
.\setup.ps1
.\run.ps1
```

> **Note:** Initial setup may take a few minutes while kernels compile and the environment is verified.  
> *If Windows blocks script execution, run: `powershell -ExecutionPolicy Bypass -File .\setup.ps1`*

**Common options:**
- Force CPU physics: `./setup.sh --cpu` (or `.\setup.ps1 --cpu`)
- Run options: `./run.sh [--headless] [--no-controls] [--backend cpu]`
- Reset environment: `./setup.sh --reset` (or `.\setup.ps1 --reset`)

## Upgrading

To update dependencies to the latest Genesis release:

**Linux / Apple Silicon macOS:**
```bash
./upgrade.sh
```

**Windows x64 (PowerShell):**
```powershell
.\upgrade.ps1
```

Use `--dry-run` to preview changes or `--lock-only` to update lockfiles without installing.

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
