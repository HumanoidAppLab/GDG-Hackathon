"""Install a frozen MH3 release and verify physics and camera rendering locally."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import struct
import subprocess
import sys
import tomllib
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent


def check_platform(
    system: str | None = None,
    machine: str | None = None,
    release: str | None = None,
    libc: tuple[str, str] | None = None,
    bits: int | None = None,
) -> None:
    """Reject targets lacking wheels for this release before downloading packages.
    :param system: Operating system name, defaulting to platform.system().
    :type system: str | None
    :param machine: Machine architecture name, defaulting to platform.machine().
    :type machine: str | None
    :param release: Operating system release string, defaulting to platform.release().
    :type release: str | None
    :param libc: Libc implementation name and version tuple, defaulting to platform.libc_ver().
    :type libc: tuple[str, str] | None
    :param bits: Pointer bit width, defaulting to the running Python architecture.
    :type bits: int | None
    :return: None when the platform is supported.
    :rtype: None
    """
    # Detect host system parameters if not explicitly provided for testing.
    system = system or platform.system()
    machine = (machine or platform.machine()).lower()
    bits = bits or struct.calcsize("P") * 8
    # Enforce 64-bit operating system and Python architecture.
    if bits != 64:
        raise RuntimeError("A 64-bit Python and operating system are required.")
    # Verify Windows version is 10 or newer on x64 hardware.
    if system == "Windows" and machine in ("amd64", "x86_64"):
        if int((release or platform.release()).split(".")[0]) < 10:
            raise RuntimeError("Windows 10 or newer is required.")
        return
    # Verify macOS is 14 or newer on Apple Silicon hardware.
    if system == "Darwin" and machine in ("arm64", "aarch64"):
        if int((release or platform.mac_ver()[0]).split(".")[0]) < 14:
            raise RuntimeError("The pinned PyTorch wheel requires macOS 14+ on Apple Silicon. Update macOS first.")
        return
    # Verify Linux is running on x86_64 hardware with GNU libc 2.28 or later.
    if system == "Linux" and machine == "x86_64":
        name, version = libc or platform.libc_ver()
        if name != "glibc" or tuple(map(int, version.split(".")[:2])) < (2, 28):
            raise RuntimeError(
                "Linux requires glibc 2.28+. Alpine/musl and older distributions lack compatible wheels."
            )
        return
    # Reject unsupported operating systems, architectures, or non-glibc distributions.
    raise RuntimeError(
        "Supported targets are Windows x64, Linux x64/glibc 2.28+, and Apple Silicon/macOS 14+. "
        "The pinned PyTorch/Quadrants wheels exclude Intel Macs; USD excludes native Windows ARM and Linux ARM."
    )


def accelerator_package(name: str) -> bool:
    """Check whether a package belongs to PyTorch or GPU accelerator toolchains.
    :param name: Package name to inspect.
    :type name: str
    :return: True if the package is a Torch or accelerator runtime component.
    :rtype: bool
    """
    # Normalize the package name by lowercasing and standardizing delimiters.
    name = name.lower().replace("_", "-")
    # Identify PyTorch or vendor-specific GPU driver and compilation wheels.
    return name == "torch" or name.startswith(("nvidia-", "cuda-", "triton", "pytorch-triton", "intel-"))


def core_constraints(requirements: str) -> str:
    """Keep release versions fixed, allowing only accelerator dependencies to vary.
    :param requirements: Locked requirement strings exported from uv.
    :type requirements: str
    :return: Filtered requirement constraints excluding PyTorch accelerator packages.
    :rtype: str
    """
    lines = []
    # Retain exact pins for general packages while omitting deferred accelerator libraries.
    for line in requirements.splitlines():
        match = re.match(r"([A-Za-z0-9_.-]+)(?:\[|[<=>!~;\s])", line)
        if match and not accelerator_package(match[1]):
            lines.append(line)
    return "\n".join(lines) + "\n"


def accelerator_exclusions(lock: dict) -> list[str]:
    """Defer Torch-only accelerator packages on fresh installations.

    Keep dependencies required by Genesis itself, such as Madrona's CUDA 12
    libraries. The selected Torch wheel will install its own complete graph.
    :param lock: Parsed uv.lock data structure.
    :type lock: dict
    :return: Sorted list of accelerator packages to defer during initial installation.
    :rtype: list[str]
    """
    # Map package metadata records by their package name.
    packages = {package["name"]: package for package in lock["package"]}
    protected = set()
    # Find accelerator dependencies required by non-accelerator packages such as Genesis.
    for package in packages.values():
        if accelerator_package(package["name"]) or "editable" in package.get("source", {}):
            continue
        for dependency in package.get("dependencies", []):
            if accelerator_package(dependency["name"]):
                protected.add(dependency["name"])
    # Propagate dependency protection recursively across transitive requirements.
    pending = list(protected)
    while pending:
        package = packages.get(pending.pop(), {})
        dependencies = list(package.get("dependencies", []))
        for group in package.get("optional-dependencies", {}).values():
            dependencies.extend(group)
        for dependency in dependencies:
            if dependency["name"] not in protected:
                protected.add(dependency["name"])
                pending.append(dependency["name"])
    # Exclude only accelerator packages that are not required by non-accelerator dependencies.
    return sorted(name for name in packages if accelerator_package(name) and name not in protected)


def recovery_message(phase: str, error: Exception | str, system: str | None = None) -> str:
    """Give focused recovery instructions instead of hiding the underlying error.
    :param phase: Current setup verification phase where failure occurred.
    :type phase: str
    :param error: Encountered exception or error description.
    :type error: Exception | str
    :param system: Operating system name, defaulting to platform.system().
    :type system: str | None
    :return: User-friendly diagnostic and recovery instructions.
    :rtype: str
    """
    system = system or platform.system()
    text = str(error).lower()
    # Diagnose OpenGL, EGL, or window display failures and provide platform-specific advice.
    if phase == "render" or any(term in text for term in ("opengl", "egl", "glx", "libgl", "osmesa", "display")):
        advice = {
            "Linux": "Install your distribution's OpenGL/EGL runtime libraries (on Ubuntu/Debian: libgl1, libegl1, libglib2.0-0), and use a local desktop session. On WSL use WSLg and current Windows GPU drivers.",
            "Windows": "Update the GPU manufacturer's graphics driver, reboot, and retry from a local desktop session rather than Remote Desktop.",
            "Darwin": "Update macOS and retry from a local desktop session; camera rendering requires working OpenGL even with CPU physics.",
        }.get(system, "Check your graphics driver and OpenGL runtime.")
        return "Camera/graphics verification failed. " + advice
    # Advise on CUDA, ROCm, or Metal driver mismatch or compilation failures.
    if any(term in text for term in ("cuda", "hip", "rocm", "metal", "driver")):
        return "GPU initialization or compilation failed. Update the manufacturer's driver and reboot, or rerun setup with --cpu. CPU physics still needs graphics libraries for the cameras."
    # Advise on Windows missing Visual C++ runtime libraries.
    if system == "Windows" and any(term in text for term in ("dll load failed", "winerror 126", "winerror 127")):
        return "A Windows native library could not load. Install Microsoft's Visual C++ 2015-2022 x64 Redistributable, reboot, and retry. Also check the GPU driver and the DLL named in the error."
    # Advise on Linux missing OpenMP or C++ runtime libraries.
    if system == "Linux" and any(term in text for term in ("libgomp", "libstdc++", "shared object file")):
        return "A Linux native library is missing. Install the runtime named in the loader error using your OS package manager (commonly libgomp1 or libstdc++6 on Ubuntu/Debian), then retry setup."
    # Provide network and disk troubleshooting advice for package installation errors.
    if phase == "install":
        return "Package installation failed. Check internet access, free disk space, proxy/TLS settings, and the package/platform error above. Rerun setup; participants should not run upgrade to repair an installation."
    # Fall back to general diagnostic advice if the cause is unclassified.
    return f"MH3 verification failed during {phase}. Check the error above and that assets/MH3.usd and config/mh3.yaml are included in the checkout."


def smoke_check(root: Path, backend: str, report: Path) -> int:
    """Exercise Genesis, the bundled articulation, physics, RGB, and depth.
    :param root: Root directory of the MH3 starter project checkout.
    :type root: Path
    :param backend: Genesis compute backend name to verify.
    :type backend: str
    :param report: Path where JSON smoke check report should be written.
    :type report: Path
    :return: Process exit code, zero on success and non-zero on failure.
    :rtype: int
    """
    phase = "imports"

    def write(**values: Any) -> None:
        """Write current verification phase and status data to the report file.
        :param values: Key-value attributes to record in the report JSON.
        :type values: Any
        :return: None after writing the report JSON to disk.
        :rtype: None
        """
        report.write_text(json.dumps({"phase": phase, **values}, indent=2), encoding="utf-8")

    try:
        # Record initial status prior to importing third-party libraries.
        write(ok=False)
        from dataclasses import replace

        import genesis as gs
        import numpy as np
        from PIL import Image
        from pxr import Usd

        from mh3_genesis import MH3Simulation, SimulatorConfig

        # Check accessibility and validity of the bundled robot USD stage.
        phase = "usd"
        write(ok=False)
        config = SimulatorConfig.from_yaml(root / "config/mh3.yaml")
        if Usd.Stage.Open(str(config.robot_usd)) is None:
            raise RuntimeError("Could not open the bundled MH3 USD.")
        if not config.cameras:
            raise RuntimeError("The bundled scene needs a camera for setup verification.")
        # Configure a lightweight camera resolution for fast smoke test verification.
        camera = replace(config.cameras[0], width=128, height=72)
        config = replace(config, backend=backend, show_viewer=False, realtime=False, cameras=(camera,))
        # Construct and compile the Genesis simulation scene.
        phase = "build"
        write(ok=False)
        simulation = MH3Simulation(config).build()
        # Verify physics stepping and numeric stability.
        phase = "physics"
        write(ok=False)
        simulation.step()
        if not np.isfinite(simulation.get_joint_state().positions).all():
            raise RuntimeError("Physics produced non-finite joint positions.")
        # Verify RGB and depth offscreen camera rendering.
        phase = "render"
        write(ok=False)
        frame = simulation.capture_camera(camera.name, rgb=True, depth=True)
        if frame.rgb is None or frame.rgb.shape != (72, 128, 3) or not np.isfinite(frame.rgb).all():
            raise RuntimeError("Camera did not produce a valid RGB frame.")
        if frame.depth is None or frame.depth.shape != (72, 128) or not np.isfinite(frame.depth).any():
            raise RuntimeError("Camera did not produce a valid depth frame.")
        # Save verification preview image for user inspection.
        Image.fromarray(frame.rgb.astype(np.uint8)).save(report.parent / "preview.png")
        # Record successful smoke test completion with verified backend details.
        write(
            ok=True,
            backend=simulation.compute_backend,
            device=str(gs.device),
            camera=camera.name,
            python=sys.version.split()[0],
        )
        print(
            f"Verified MH3: Genesis backend={simulation.compute_backend}; physics, RGB, and depth passed.", flush=True
        )
        return 0
    # Classify errors from third-party native libraries at the process boundary.
    except Exception as error:  # noqa: BLE001
        write(ok=False, error=str(error), guidance=recovery_message(phase, error))
        print(recovery_message(phase, error), file=sys.stderr)
        print(f"Underlying error: {error}", file=sys.stderr)
        return 1


class Setup:
    """Manage local virtual environment bootstrapping, accelerator selection, and verification.
    :param root: Root path of the project checkout.
    :type root: Path
    :return: Setup orchestrator instance.
    :rtype: Setup
    """

    def __init__(self, root: Path = ROOT) -> None:
        """Initialize setup environment paths and configuration.
        :param root: Root path of the project repository.
        :type root: Path
        :return: None after initializing file paths and process environment.
        :rtype: None
        """
        # Save project root directory reference.
        self.root = root
        # Set local cache directory for storing verification state and previews.
        self.local = root / ".setup"
        # Determine the virtual environment Python executable path according to OS convention.
        self.python = root / ".venv" / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
        # Locate the uv binary from environment override or default system PATH.
        self.uv = os.environ.get("MH3_UV", "uv")
        # Prepare an isolated environment clean of conflicting Python path variables.
        self.env = os.environ.copy()
        self.env.pop("VIRTUAL_ENV", None)
        self.env.pop("PYTHONPATH", None)
        self.env.pop("PYTHONHOME", None)
        # Direct uv to operate on the project's dedicated virtual environment.
        self.env["UV_PROJECT_ENVIRONMENT"] = str(root / ".venv")
        self.phase = "install"

    def uv_run(self, *args: Any, capture: bool = False, check: bool = True) -> subprocess.CompletedProcess:
        """Execute a uv command within the project repository environment.
        :param args: Command line arguments passed to uv.
        :type args: Any
        :param capture: Capture standard output and error when True.
        :type capture: bool
        :param check: Raise CalledProcessError if the command exits with non-zero status.
        :type check: bool
        :return: Completed process execution result.
        :rtype: subprocess.CompletedProcess
        """
        # Launch the uv CLI subprocess with the prepared environment variables.
        return subprocess.run(
            [self.uv, *map(str, args)], cwd=self.root, env=self.env, text=True, capture_output=capture, check=check
        )

    def fingerprint(self) -> str:
        """Compute a SHA-256 hash of lockfiles, python version, and system platform.
        :return: Hexadecimal hash fingerprint representing the current environment specification.
        :rtype: str
        """
        # Concatenate release file contents and platform string into a single hash.
        return hashlib.sha256(
            b"".join((self.root / name).read_bytes() for name in ("uv.lock", "pyproject.toml", ".python-version"))
            + platform.platform().encode()
        ).hexdigest()

    def inventory(self) -> list[tuple[str, str]]:
        """List currently installed packages and their version strings.
        :return: Sorted list of installed package name and version pairs.
        :rtype: list[tuple[str, str]]
        """
        # Query the virtual environment package manifest using pip list in JSON format.
        result = self.uv_run("pip", "list", "--python", self.python, "--format", "json", capture=True)
        # Extract and sort package name and version tuples.
        return sorted((package["name"], package["version"]) for package in json.loads(result.stdout))

    def reusable(self) -> bool:
        """Check whether the existing virtual environment matches the verified release.
        :return: True if the environment matches the previous verification state and passes checks.
        :rtype: bool
        """
        try:
            # Load previously saved setup state from disk.
            state = json.loads((self.local / "state.json").read_text())
            # Confirm fingerprint, installed packages, and pip dependency consistency.
            return (
                state["fingerprint"] == self.fingerprint()
                and state["packages"] == [list(item) for item in self.inventory()]
                and self.uv_run("pip", "check", "--python", self.python, check=False).returncode == 0
            )
        except (OSError, ValueError, KeyError, subprocess.CalledProcessError):
            return False

    def probe(self) -> dict[str, Any] | None:
        """Probe PyTorch acceleration device availability and runtime version metadata.
        :return: Dictionary containing device, torch, and CUDA/HIP versions, or None if failed.
        :rtype: dict[str, Any] | None
        """
        code = """import json, torch
device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
torch.ones(1, device=device).sum().item()
print(json.dumps({'device': device, 'torch': torch.__version__, 'hip': torch.version.hip, 'cuda': torch.version.cuda}))
"""
        # Execute diagnostic probe code in a subprocess using the target virtual environment.
        result = subprocess.run(
            [str(self.python), "-c", code], cwd=self.root, env=self.env, capture_output=True, text=True, check=False
        )
        return json.loads(result.stdout) if result.returncode == 0 else None

    def install_torch(self, backend: str, constraints: Path, version: str) -> None:
        """Install or reinstall PyTorch with the specified compute backend.
        :param backend: PyTorch accelerator backend ('auto', 'cpu', etc.).
        :type backend: str
        :param constraints: Path to core package constraint file.
        :type constraints: Path
        :param version: Pinned PyTorch release version.
        :type version: str
        :return: None after installing and checking the PyTorch wheel.
        :rtype: None
        """
        # Note that uv resolves the complete dependency graph before installing anything.
        self.uv_run(
            "pip",
            "install",
            "--python",
            self.python,
            "--reinstall-package",
            "torch",
            f"torch=={version}",
            "--torch-backend",
            backend,
            "--constraint",
            constraints,
            "--prerelease",
            "disallow",
        )
        # Verify that all installed package dependencies remain satisfied.
        self.uv_run("pip", "check", "--python", self.python)

    def smoke(self, backend: str) -> dict[str, Any]:
        """Run an isolated verification subprocess and read its report.
        :param backend: Compute backend name to verify.
        :type backend: str
        :return: Dictionary with verification results, phase, and error information.
        :rtype: dict[str, Any]
        """
        report = self.local / "smoke.json"
        report.unlink(missing_ok=True)
        # Execute the smoke check in a dedicated child process to isolate native crashes.
        result = subprocess.run(
            [str(self.python), str(self.root / "setup.py"), "--smoke-backend", backend, "--report", str(report)],
            cwd=self.root,
            env=self.env,
            check=False,
        )
        # Read the generated smoke check report or construct a fallback failure record.
        if report.exists():
            data = json.loads(report.read_text())
        else:
            data = {
                "ok": False,
                "phase": "build",
                "error": f"Verification process exited with code {result.returncode}.",
            }
        data["ok"] = bool(data.get("ok")) and result.returncode == 0
        return data

    def install(self, cpu: bool = False, reset: bool = False) -> None:
        """Install dependencies and verify physics and camera rendering.
        :param cpu: Force CPU computation for PyTorch and Genesis physics.
        :type cpu: bool
        :param reset: Force clean reinstall instead of reusing previously verified environment.
        :type reset: bool
        :return: None after successfully verifying the installation.
        :rtype: None
        """
        # Verify operating system and architecture prerequisites before proceeding.
        check_platform()
        # Verify required repository files and bundled model assets exist.
        for name in ("uv.lock", "pyproject.toml", ".python-version", "assets/MH3.usd", "config/mh3.yaml"):
            if not (self.root / name).is_file():
                raise RuntimeError(f"Missing {name}; download the complete repository, including its assets.")
        # Create local setup data directory.
        self.local.mkdir(exist_ok=True)
        lock = tomllib.loads((self.root / "uv.lock").read_text())
        reused = not reset and self.reusable()
        # Synchronize dependencies with uv if not reusing a previously verified environment.
        if not reused:
            sync_args = ["sync", "--locked"]
            if not self.python.exists():
                for name in accelerator_exclusions(lock):
                    sync_args.extend(["--no-install-package", name])
                print(
                    "Fresh environment: deferring PyTorch's accelerator libraries until hardware selection.", flush=True
                )
            self.uv_run(*sync_args)
        else:
            print("Keeping the previously verified environment; release files and package versions match.", flush=True)
        # Extract the pinned PyTorch version requirement from the lockfile.
        torch_version = next(package["version"] for package in lock["package"] if package["name"] == "torch")
        # Export exact requirement constraints for non-accelerator packages.
        exported = self.uv_run(
            "export",
            "--locked",
            "--all-groups",
            "--no-emit-project",
            "--no-hashes",
            "--no-header",
            "--no-annotate",
            capture=True,
        ).stdout
        constraints = self.local / "core-constraints.txt"
        constraints.write_text(core_constraints(exported), encoding="utf-8")
        # Test current accelerator hardware capabilities.
        probe = self.probe()
        # Select appropriate PyTorch accelerator wheel based on probe results or CPU flag.
        if cpu:
            if not probe or probe["device"] != "cpu" or probe.get("hip") or probe.get("cuda"):
                self.install_torch("cpu", constraints, torch_version)
            backend = "cpu"
        elif probe and (probe["device"] != "cpu" or platform.system() == "Darwin" or reused):
            print(f"Keeping compatible PyTorch {probe['torch']}: {probe['device']} computation succeeded.", flush=True)
            previous = json.loads((self.local / "state.json").read_text()) if reused else {}
            backend = "cpu" if previous.get("backend") == "cpu" else "auto"
        else:
            try:
                self.install_torch("auto", constraints, torch_version)
                probe = self.probe()
                if probe is None:
                    raise RuntimeError("The selected PyTorch build could not execute a tensor calculation.")
            except (subprocess.CalledProcessError, RuntimeError) as error:
                print(
                    f"Accelerator selection failed ({error}); installing the CPU build with compatible dependencies.",
                    flush=True,
                )
                self.uv_run("sync", "--locked")
                self.install_torch("cpu", constraints, torch_version)
            backend = "auto"
        # Validate that all dependencies remain consistent after PyTorch installation.
        self.uv_run("pip", "check", "--python", self.python)
        self.phase = "build"
        print(
            "Verifying the MH3, physics, and camera rendering. The first kernel compilation can take several minutes.",
            flush=True,
        )
        # Run the simulator smoke check to verify physics and offscreen rendering.
        result = self.smoke(backend)
        if not result["ok"] and backend != "cpu" and result.get("phase") in ("build", "physics"):
            print("Genesis GPU verification failed; retrying physics on the CPU.", flush=True)
            result = self.smoke("cpu")
        if not result["ok"]:
            raise RuntimeError(
                result.get("guidance") or recovery_message(result.get("phase", "build"), result.get("error", ""))
            )
        # Record verified environment state for instant setup reuse on subsequent runs.
        state = {
            "fingerprint": self.fingerprint(),
            "lock_hash": hashlib.sha256((self.root / "uv.lock").read_bytes()).hexdigest(),
            "backend": result["backend"],
            "packages": self.inventory(),
            "platform": platform.system(),
            "machine": platform.machine(),
            "python": result.get("python", sys.version.split()[0]),
        }
        (self.local / "state.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
        # Freeze installed packages into requirements.txt for debugging and inspection.
        (self.local / "requirements.txt").write_text(
            self.uv_run("pip", "freeze", "--python", self.python, capture=True).stdout, encoding="utf-8"
        )
        print("Setup passed. Camera preview: .setup/preview.png. Start the simulator with your run script.", flush=True)


def main() -> None:
    """Parse command-line arguments and run setup or smoke verification.
    :return: None because the process exits.
    :rtype: None
    """
    # Parse command line flags for CPU execution, environment reset, and internal smoke verification.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cpu", action="store_true", help="Use CPU physics and a CPU PyTorch wheel")
    parser.add_argument(
        "--reset", action="store_true", help="Restore the release environment and redetect accelerators"
    )
    parser.add_argument("--smoke-backend", choices=("auto", "cpu"), help=argparse.SUPPRESS)
    parser.add_argument("--report", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    # Route internal smoke verification subprocesses directly to the check function.
    if args.smoke_backend:
        raise SystemExit(smoke_check(ROOT, args.smoke_backend, args.report))
    setup = Setup()
    try:
        # Run the setup workflow with requested options.
        setup.install(cpu=args.cpu, reset=args.reset)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Setup failed: {error}", file=sys.stderr)
        print(recovery_message(setup.phase, error), file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
