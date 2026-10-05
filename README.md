# MH3 Genesis Hackathon Starter

This repository is a self-contained, Python-based starting point for building projects with the [Mirsee Robotics](https://mirsee.com "Mirsee Robotics Inc.") MH3 in [Genesis World](https://genesis-embodied-ai.github.io "Genesis World"). It includes the simulated MH3, a default world with a floor, a browser dashboard, and a Python API for controlling and observing the robot.

## Setup and run

### Supported platforms

- **Linux x64** (glibc 2.28+)
- **Apple Silicon macOS** (macOS 14+)
- **Windows 10+ x64**

*Intel Macs, ARM Linux/Windows, and 32-bit platforms are not supported.*

### Prerequisites

- **Linux:** `curl` or `wget`, plus graphics runtime libraries (`libgl1`, `libegl1`, `libglib2.0-0` on Ubuntu/Debian). On WSL2, ensure WSLg and updated GPU drivers are installed.
- Working GPU drivers and OpenGL/EGL runtime (required for cameras and the viewer, even if physics runs on CPU). See [Genesis installation requirements](https://genesis-world.readthedocs.io/en/latest/user_guide/overview/installation.html "Genesis World - Installation").

### Installation & Running

> [!NOTE]
> Initial setup may take a few minutes while kernels compile and the environment is verified.  

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

> [!IMPORTANT]
> If Windows blocks script execution, run: `powershell -ExecutionPolicy Bypass -File .\setup.ps1`

**Common options:**
- Force CPU physics: `./setup.sh --cpu` (or `.\setup.ps1 --cpu`)
- Run options: `./run.sh [--headless] [--no-controls] [--backend cpu]`
- Reset environment: `./setup.sh --reset` (or `.\setup.ps1 --reset`)

## Upgrading

To update dependencies to the latest [Genesis World](https://genesis-embodied-ai.github.io "Genesis World") release:

> [!NOTE]
> In most cases you should only try upgrading if a newer release of [Genesis World](https://genesis-embodied-ai.github.io "Genesis World") has a feature or bug fix you need. Before upgrading, back up your current project.

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

The dashboard starts automatically in your default browser when you launch the simulator with `./run.sh` or `.\run.ps1`.

### Controls & feeds
- **Base drive:** Hold on-screen buttons or use **W / A / S / D** (or arrow keys) to drive the base. Speed sliders adjust maximum linear and angular rates. Releasing a control stops base motion automatically.
- **Joint controls:** Adjust sliders or enter numeric values to pose joints. Click **Return all joints home** to restore the default startup configuration.
- **Camera feeds:** Displays live views for head and base cameras (left RGB, right RGB, and depth). In depth views, close surfaces are bright and distances beyond 5 m are dark.

### Enabling, disabling & embedding
- **Disable dashboard:** Pass `--no-controls` to your run script:
  ```bash
  ./run.sh --no-controls        # Linux / macOS
  .\run.ps1 --no-controls       # Windows
  ```
- **Stopping:** Closing the browser tab only closes the page; press `Ctrl+C` in your terminal to stop the simulator.
- **Embedding in custom scripts:**
  ```python
  from mh3_genesis import MH3ControlPanel, MH3Simulation, SimulatorConfig

  sim = MH3Simulation(SimulatorConfig.from_yaml("config/mh3.yaml")).build()
  panel = MH3ControlPanel(sim)
  panel.start()  # Use open_browser=False to print the local URL without opening a browser tab
  sim.add_step_hook(panel.update)

  try:
      sim.run()
  finally:
      panel.close()
  ```
  The dashboard listens locally on `127.0.0.1` on an automatically assigned port. Commands are queued and applied safely on the simulation thread during `panel.update()`.

## Robot joints & links

### Units
- **Revolute joints (arms, head, hands):** Radians (`rad`).
- **Prismatic joints (`slider`):** Metres (`m`).
- **Mobile base velocities:** Metres per second (`m/s`) for linear speed, radians per second (`rad/s`) for angular speed.

### Joint names & limits
Inspect available joints and limits via `sim.joint_names` and `sim.get_joint_limits()`:

- **Torso lift:** `slider` (travels from 0.0 to 0.3 m).
- **Head:** `head_yaw`, `head_pitch`.
- **Arms (7 DOF each):** `left_1` through `left_7` and `right_1` through `right_7`.
- **Hands (6 controls per hand):**
  - **Fingers:** `left_index`, `left_middle`, `left_ring`, `left_little` (and matching `right_*` joints). Zero radians represents fully open; joints close through 1.57 rad. Dependent finger joints follow these root controls automatically via mimic constraints.
  - **Thumbs:** `left_thumb_1` (up to 1.6 rad) and `left_thumb_2` (up to 0.45 rad), along with their `right_*` counterparts.

### Link names
Discover all link identifiers using `sim.link_names` before passing one to `sim.get_link()` or `sim.solve_ik()`:
- **Arm segments:** `left_arm_1` through `left_arm_7` (and `right_arm_1` through `right_arm_7`).
- **Gripper bases:** `left_gripper_base`, `right_gripper_base`.
- **Fingertips:** `left_index_tip`, `left_middle_tip`, `left_thumb_tip`, etc.
- **Torso & head:** `torso_base`, `head_base`, `head_camera_mount`.

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
