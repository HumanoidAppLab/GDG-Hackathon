"""Provide the command-line entry point for the MH3 starter scene."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from dataclasses import replace
from pathlib import Path

from .config import PROJECT_ROOT, SimulatorConfig
from .control_panel import MH3ControlPanel
from .simulator import MH3Simulation


def verified_setup_backend() -> str | None:
    """Reuse a machine's verified CPU fallback while its release is unchanged.
    :return: The CPU backend string if verification matched, or None.
    :rtype: str | None
    """
    try:
        # Load the recorded verification state generated during local setup.
        state = json.loads((PROJECT_ROOT / ".setup/state.json").read_text())
        # Check whether the current lockfile hash, platform, and python match the verified environment.
        if (
            state["lock_hash"] == hashlib.sha256((PROJECT_ROOT / "uv.lock").read_bytes()).hexdigest()
            and state["platform"] == platform.system()
            and state["machine"] == platform.machine()
            and state["python"] == sys.version.split()[0]
            and state["backend"] == "cpu"
        ):
            return "cpu"
    except (OSError, ValueError, KeyError):
        # Fall back to normal backend resolution if the verification state is missing or invalid.
        pass
    return None


def build_parser() -> argparse.ArgumentParser:
    """Create the simulator command-line argument parser.
    :return: Parser with scene, backend, and dashboard options.
    :rtype: argparse.ArgumentParser
    """
    # Initialize the top-level argument parser with description text.
    parser = argparse.ArgumentParser(description="ROS-free MH3 Genesis hackathon starter")
    # Define flags for scene configuration, headless mode, and compute backend.
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "config/mh3.yaml")
    parser.add_argument("--headless", action="store_true", help="Disable the interactive viewer")
    parser.add_argument("--backend", choices=("auto", "cpu", "gpu", "cuda", "amdgpu", "metal"))
    # Define execution pacing, step limit, and web dashboard options.
    parser.add_argument("--no-realtime", action="store_true", help="Run as fast as possible")
    parser.add_argument("--steps", type=int, help="Stop after this many physics steps")
    parser.add_argument("--no-controls", action="store_true", help="Do not launch the browser control panel")
    return parser


def run(args: argparse.Namespace) -> int:
    """Build and run the configured simulator until completion or interruption.
    :param args: Parsed command-line arguments.
    :type args: argparse.Namespace
    :return: Zero after the simulator exits cleanly.
    :rtype: int
    """
    # Load the base simulator configuration from the specified YAML file.
    config = SimulatorConfig.from_yaml(args.config)
    # Apply CLI argument overrides for viewer visibility, realtime pacing, and backend selection.
    config = replace(
        config,
        show_viewer=config.show_viewer and not args.headless,
        realtime=config.realtime and not args.no_realtime,
        backend=args.backend or (verified_setup_backend() if config.backend == "auto" else None) or config.backend,
    )
    # Instantiate and construct the Genesis scene and robot entities.
    simulation = MH3Simulation(config).build()
    control_panel = None
    # Launch the interactive browser dashboard unless disabled or stepping a fixed count.
    if not args.no_controls and args.steps is None:
        control_panel = MH3ControlPanel(simulation)
        control_panel.start()
        # Register dashboard update logic as a callback executed at every simulation step.
        simulation.add_step_hook(control_panel.update)
    try:
        # Run the simulation loop until the step limit is reached or the user stops execution.
        simulation.run(steps=args.steps)
    except KeyboardInterrupt:
        # Handle manual process interruption cleanly without dumping a stack trace.
        pass
    finally:
        # Ensure the background HTTP server is closed when the simulation exits.
        if control_panel is not None:
            control_panel.close()
    return 0


def main() -> None:
    """Exit the process with the simulator command result.
    :return: None because the process exits.
    :rtype: None
    """
    # Parse incoming CLI arguments and execute the simulation runner.
    raise SystemExit(run(build_parser().parse_args()))
