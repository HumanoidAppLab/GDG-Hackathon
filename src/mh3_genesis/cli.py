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
    """Reuse a machine's verified CPU fallback while its release is unchanged."""
    try:
        state = json.loads((PROJECT_ROOT / ".setup/state.json").read_text())
        if (
            state["lock_hash"] == hashlib.sha256((PROJECT_ROOT / "uv.lock").read_bytes()).hexdigest()
            and state["platform"] == platform.system()
            and state["machine"] == platform.machine()
            and state["python"] == sys.version.split()[0]
            and state["backend"] == "cpu"
        ):
            return "cpu"
    except (OSError, ValueError, KeyError):
        pass
    return None


def build_parser() -> argparse.ArgumentParser:
    """Create the simulator command-line argument parser.
    :return: Parser with scene, backend, and dashboard options.
    :rtype: argparse.ArgumentParser
    """
    parser = argparse.ArgumentParser(description="ROS-free MH3 Genesis hackathon starter")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "config/mh3.yaml")
    parser.add_argument("--headless", action="store_true", help="Disable the interactive viewer")
    parser.add_argument("--backend", choices=("auto", "cpu", "gpu", "cuda", "amdgpu", "metal"))
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
    config = SimulatorConfig.from_yaml(args.config)
    config = replace(
        config,
        show_viewer=config.show_viewer and not args.headless,
        realtime=config.realtime and not args.no_realtime,
        backend=args.backend or (verified_setup_backend() if config.backend == "auto" else None) or config.backend,
    )
    simulation = MH3Simulation(config).build()
    control_panel = None
    if not args.no_controls and args.steps is None:
        control_panel = MH3ControlPanel(simulation)
        control_panel.start()
        simulation.add_step_hook(control_panel.update)
    try:
        simulation.run(steps=args.steps)
    except KeyboardInterrupt:
        pass
    finally:
        if control_panel is not None:
            control_panel.close()
    return 0


def main() -> None:
    """Exit the process with the simulator command result.
    :return: None because the process exits.
    :rtype: None
    """
    raise SystemExit(run(build_parser().parse_args()))
