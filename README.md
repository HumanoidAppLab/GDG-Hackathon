# MH3 Genesis Hackathon Starter

This repository is a self-contained, Python-based starting point for building projects with the [Mirsee Robotics](https://mirsee.com "Mirsee Robotics Inc.") MH3 in [Genesis World](https://github.com/Genesis-Embodied-AI/genesis-world "Genesis World"). It includes the simulated MH3, a default world with a floor, a browser dashboard, and a Python API for controlling and observing the robot.

## Setup and Run

### Supported Platforms

- **Linux x64** (glibc 2.28+)
- **Apple Silicon macOS** (macOS 14+)
- **Windows 10+ x64**

*Intel Macs, ARM Linux/Windows, and 32-bit platforms are not supported.*

### Prerequisites

- **Linux:** `curl` or `wget`, plus graphics runtime libraries (`libgl1`, `libegl1`, `libglib2.0-0` on Ubuntu/Debian). On WSL2, ensure WSLg and updated GPU drivers are installed.
- Working GPU drivers and OpenGL/EGL runtime (required for cameras and the viewer, even if physics runs on CPU). See [Genesis installation requirements](https://genesis-world.readthedocs.io/en/latest/user_guide/overview/installation.html "Genesis World - Installation").

### Installation

> [!NOTE]
> Initial setup may take a few minutes while kernels compile and the environment is verified.  

**Linux / Apple Silicon macOS:**
```bash
./setup.sh
```

**Windows x64 (PowerShell):**
```powershell
.\setup.ps1
```

> [!IMPORTANT]
> If Windows blocks script execution, run: `powershell -ExecutionPolicy Bypass -File .\setup.ps1`

**Common options:**
- Force CPU physics: `./setup.sh --cpu` (or `.\setup.ps1 --cpu`)
- Reset environment: `./setup.sh --reset` (or `.\setup.ps1 --reset`)

### Running Scripts

After setup, run simulation scripts (such as [`hello_mh3.py`](hello_mh3.py "hello_mh3.py")) with `uv run`:
```bash
uv run hello_mh3.py
```

## Upgrading

To update dependencies to the latest [Genesis World](https://github.com/Genesis-Embodied-AI/genesis-world "Genesis World") release:

> [!NOTE]
> In most cases you should only try upgrading if a newer release of [Genesis World](https://github.com/Genesis-Embodied-AI/genesis-world "Genesis World") has a feature or bug fix you need. Before upgrading, back up your current project.

**Linux / Apple Silicon macOS:**
```bash
./upgrade.sh
```

**Windows x64 (PowerShell):**
```powershell
.\upgrade.ps1
```

Use `--dry-run` to preview changes or `--lock-only` to update lockfiles without installing.

## Browser Dashboard

The dashboard provides an interactive web interface for driving the base, posing joints, testing inverse kinematics (IK), and monitoring live camera feeds.

### Controls & Feeds

- **Base drive:** Hold on-screen buttons or use **W / A / S / D** (or arrow keys) to drive the base. Speed sliders adjust maximum linear and angular rates. Releasing a control stops base motion automatically.
- **Joint controls:** Adjust sliders or enter numeric values to pose joints. Click **Return all joints home** to restore the default startup configuration.
- **IK tester:** Interactively solve inverse kinematics for chosen end effectors and reference frames with position and orientation targets.
- **Camera feeds:** Displays live views for head and base cameras (left RGB, right RGB, and depth). In depth views, close surfaces are white and far distances are black.

### Using the Dashboard

You can easily embed the dashboard in your custom simulation scripts:

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
The dashboard listens locally on `127.0.0.1` on an automatically assigned port. Commands are queued and applied safely on the simulation thread during `panel.update()`. Press `Ctrl+C` in your terminal to stop the simulation.

## Robot Joints & Links

### Units

- **Revolute joints (arms, head, hands):** Radians (`rad`).
- **Prismatic joints (`slider`):** Metres (`m`).
- **Mobile base velocities:** Metres per second (`m/s`) for linear speed, radians per second (`rad/s`) for angular speed.

### Joint Names & Limits

Inspect available joints and limits via `sim.joint_names` and `sim.get_joint_limits()`:

- **Torso lift:** `slider` (travels from 0.0 to 0.3 m).
- **Head:** `head_yaw`, `head_pitch`.
- **Arms (7 DOF each):** `left_1` through `left_7` and `right_1` through `right_7`.
- **Hands (6 controls per hand):**
  - **Fingers:** `left_index`, `left_middle`, `left_ring`, `left_little` (and matching `right_*` joints). Zero radians represents fully open; joints close through 1.57 rad. Dependent finger joints follow these root controls automatically via mimic constraints.
  - **Thumbs:** `left_thumb_1` (up to 1.6 rad) and `left_thumb_2` (up to 0.45 rad), along with their `right_*` counterparts.

### Link Names

Discover all link identifiers using `sim.link_names` before passing one to `sim.get_link()` or `sim.solve_ik()`:
- **Arm segments:** `left_arm_1` through `left_arm_7` (and `right_arm_1` through `right_arm_7`).
- **Gripper bases:** `left_gripper_base`, `right_gripper_base`.
- **Fingertips:** `left_index_tip`, `left_middle_tip`, `left_thumb_tip`, etc.
- **Torso & head:** `torso_base`, `head_base`, `head_camera_mount`.

## Using the API

### Running Custom Scripts

After running setup, execute your custom scripts with `uv run`:

```bash
uv run my_script.py
```

### API Usage Example - `hello_mh3.py`

A complete, self-contained starter script demonstrating scene hooks, joint commands, base driving, state queries, inverse kinematics, camera capture, and step hooks is provided in [`hello_mh3.py`](hello_mh3.py "hello_mh3.py"). Run it directly with:

```bash
uv run hello_mh3.py
```

Or use it as a reference for your own scripts:

```python
import numpy as np
from mh3_genesis import MH3ControlPanel, MH3Simulation, SimulatorConfig

# 1. Load configuration and initialize simulation.
config = SimulatorConfig.from_yaml("config/mh3.yaml")
sim = MH3Simulation(config)


# 2. Scene hook: add custom entities before Genesis compiles the scene.
def add_custom_world(simulation: MH3Simulation) -> None:
    simulation.scene.add_entity(simulation.gs.morphs.Box(pos=(1.0, 0.0, 0.25), size=(0.3, 0.3, 0.5)))


sim.add_scene_hook(add_custom_world)

# 3. Build simulation, compile kernels, and spawn articulation.
sim.build()

# Inspect available joints, links, and cameras.
print(sim.compute_backend)
print(sim.joint_names)
print(sim.link_names)
print(sim.camera_names)

# 4. Control robot joints and drive mobile base.
sim.set_joint_positions({"head_yaw": 0.2, "head_pitch": -0.2, "slider": 0.1})
sim.set_joint_positions({"left_index": 0.6, "left_thumb_1": 0.5})
sim.drive_base(linear_mps=0.1, angular_radps=0.0)

# Advance simulation steps.
for _ in range(30):
    sim.step()
sim.stop_base()

# 5. Read joint states (positions, velocities, efforts).
joint_state = sim.get_joint_state(["head_yaw", "head_pitch", "slider"])

# 6. Inverse Kinematics (IK) for arm manipulation (world space by default, or frame="link_name").
left_arm_joints = [f"left_{i}" for i in range(1, 8)]
solution = sim.solve_ik(
    end_effector="left_gripper_base",
    position=(0.45, 0.25, 0.85),
    joints=left_arm_joints,
    apply=True,
)

# 7. Sensor capture (RGB, depth, point cloud).
frame = sim.capture_camera("head_left", rgb=True, depth=True)
points, valid_depth = sim.capture_pointcloud("head_left")


# 8. Step hook: register callbacks to run after each physics step.
def behavior_controller(simulation: MH3Simulation) -> None:
    if simulation.sim_time > 1.0:
        simulation.set_joint_positions({"head_pitch": 0.0})


sim.add_step_hook(behavior_controller)

# 9. Start browser dashboard and run.
panel = MH3ControlPanel(sim)
panel.start()
sim.add_step_hook(panel.update)

try:
    sim.run()
finally:
    panel.close()
```

Advanced projects can also directly access `sim.scene`, `sim.robot`, `sim.floor`, and `sim.gs` for any native [Genesis World](https://github.com/Genesis-Embodied-AI/genesis-world "Genesis World") feature.

## Resources

These are resources which you may consider using with this project.

- [Genesis World Documentation](https://genesis-world.readthedocs.io/en/latest/user_guide/index.html "Genesis World - Documentation") - Core physics and rendering engine of this project.
- [Google Gemma](https://ai.google.dev/gemma/docs "Google Gemma") - Google's lightweight, on-device multi-modal on-device AI model.
- [Gemini Robotics ER-2](https://ai.google.dev/gemini-api/docs/generate-content/robotics-overview "Gemini Robotics ER-2") - Google's flagship Embodied Reasoning (ER) Vision-Language Model (VLM).
- [LeRobot](https://huggingface.co/docs/lerobot/index "LeRobot") - Machine learning framework for robotics by [HuggingFace](https://huggingface.co "HuggingFace").