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

ROOT = Path(__file__).resolve().parent


def check_platform(system=None, machine=None, release=None, libc=None, bits=None):
    """Reject targets lacking wheels for this release before downloading packages."""
    system = system or platform.system()
    machine = (machine or platform.machine()).lower()
    bits = bits or struct.calcsize("P") * 8
    if bits != 64:
        raise RuntimeError("A 64-bit Python and operating system are required.")
    if system == "Windows" and machine in ("amd64", "x86_64"):
        if int((release or platform.release()).split(".")[0]) < 10:
            raise RuntimeError("Windows 10 or newer is required.")
        return
    if system == "Darwin" and machine in ("arm64", "aarch64"):
        if int((release or platform.mac_ver()[0]).split(".")[0]) < 14:
            raise RuntimeError("The pinned PyTorch wheel requires macOS 14+ on Apple Silicon. Update macOS first.")
        return
    if system == "Linux" and machine == "x86_64":
        name, version = libc or platform.libc_ver()
        if name != "glibc" or tuple(map(int, version.split(".")[:2])) < (2, 28):
            raise RuntimeError(
                "Linux requires glibc 2.28+. Alpine/musl and older distributions lack compatible wheels."
            )
        return
    raise RuntimeError(
        "Supported targets are Windows x64, Linux x64/glibc 2.28+, and Apple Silicon/macOS 14+. "
        "The pinned PyTorch/Quadrants wheels exclude Intel Macs; USD excludes native Windows ARM and Linux ARM."
    )


def accelerator_package(name):
    name = name.lower().replace("_", "-")
    return name == "torch" or name.startswith(("nvidia-", "cuda-", "triton", "pytorch-triton", "intel-"))


def core_constraints(requirements):
    """Keep release versions fixed, allowing only accelerator dependencies to vary."""
    lines = []
    for line in requirements.splitlines():
        match = re.match(r"([A-Za-z0-9_.-]+)(?:\[|[<=>!~;\s])", line)
        if match and not accelerator_package(match[1]):
            lines.append(line)
    return "\n".join(lines) + "\n"


def accelerator_exclusions(lock):
    """Defer Torch-only accelerator packages on fresh installations.

    Keep dependencies required by Genesis itself, such as Madrona's CUDA 12
    libraries. The selected Torch wheel will install its own complete graph.
    """
    packages = {package["name"]: package for package in lock["package"]}
    protected = set()
    for package in packages.values():
        if accelerator_package(package["name"]) or "editable" in package.get("source", {}):
            continue
        for dependency in package.get("dependencies", []):
            if accelerator_package(dependency["name"]):
                protected.add(dependency["name"])
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
    return sorted(name for name in packages if accelerator_package(name) and name not in protected)


def recovery_message(phase, error, system=None):
    """Give focused recovery instructions instead of hiding the underlying error."""
    system = system or platform.system()
    text = str(error).lower()
    if phase == "render" or any(term in text for term in ("opengl", "egl", "glx", "libgl", "osmesa", "display")):
        advice = {
            "Linux": "Install your distribution's OpenGL/EGL runtime libraries (on Ubuntu/Debian: libgl1, libegl1, libglib2.0-0), and use a local desktop session. On WSL use WSLg and current Windows GPU drivers.",
            "Windows": "Update the GPU manufacturer's graphics driver, reboot, and retry from a local desktop session rather than Remote Desktop.",
            "Darwin": "Update macOS and retry from a local desktop session; camera rendering requires working OpenGL even with CPU physics.",
        }.get(system, "Check your graphics driver and OpenGL runtime.")
        return "Camera/graphics verification failed. " + advice
    if any(term in text for term in ("cuda", "hip", "rocm", "metal", "driver")):
        return "GPU initialization or compilation failed. Update the manufacturer's driver and reboot, or rerun setup with --cpu. CPU physics still needs graphics libraries for the cameras."
    if system == "Windows" and any(term in text for term in ("dll load failed", "winerror 126", "winerror 127")):
        return "A Windows native library could not load. Install Microsoft's Visual C++ 2015-2022 x64 Redistributable, reboot, and retry. Also check the GPU driver and the DLL named in the error."
    if system == "Linux" and any(term in text for term in ("libgomp", "libstdc++", "shared object file")):
        return "A Linux native library is missing. Install the runtime named in the loader error using your OS package manager (commonly libgomp1 or libstdc++6 on Ubuntu/Debian), then retry setup."
    if phase == "install":
        return "Package installation failed. Check internet access, free disk space, proxy/TLS settings, and the package/platform error above. Rerun setup; participants should not run upgrade to repair an installation."
    return f"MH3 verification failed during {phase}. Check the error above and that assets/MH3.usd and config/mh3.yaml are included in the checkout."


def smoke_check(root, backend, report):
    """Exercise Genesis, the bundled articulation, physics, RGB, and depth."""
    phase = "imports"

    def write(**values):
        report.write_text(json.dumps({"phase": phase, **values}, indent=2), encoding="utf-8")

    try:
        write(ok=False)
        from dataclasses import replace

        import genesis as gs
        import numpy as np
        from PIL import Image
        from pxr import Usd

        from mh3_genesis import MH3Simulation, SimulatorConfig

        phase = "usd"
        write(ok=False)
        config = SimulatorConfig.from_yaml(root / "config/mh3.yaml")
        if Usd.Stage.Open(str(config.robot_usd)) is None:
            raise RuntimeError("Could not open the bundled MH3 USD.")
        if not config.cameras:
            raise RuntimeError("The bundled scene needs a camera for setup verification.")
        camera = replace(config.cameras[0], width=128, height=72)
        config = replace(config, backend=backend, show_viewer=False, realtime=False, cameras=(camera,))
        phase = "build"
        write(ok=False)
        simulation = MH3Simulation(config).build()
        phase = "physics"
        write(ok=False)
        simulation.step()
        if not np.isfinite(simulation.get_joint_state().positions).all():
            raise RuntimeError("Physics produced non-finite joint positions.")
        phase = "render"
        write(ok=False)
        frame = simulation.capture_camera(camera.name, rgb=True, depth=True)
        if frame.rgb is None or frame.rgb.shape != (72, 128, 3) or not np.isfinite(frame.rgb).all():
            raise RuntimeError("Camera did not produce a valid RGB frame.")
        if frame.depth is None or frame.depth.shape != (72, 128) or not np.isfinite(frame.depth).any():
            raise RuntimeError("Camera did not produce a valid depth frame.")
        Image.fromarray(frame.rgb.astype(np.uint8)).save(report.parent / "preview.png")
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
    except Exception as error:  # noqa: BLE001 - classify errors from third-party native libraries at the process boundary.
        write(ok=False, error=str(error), guidance=recovery_message(phase, error))
        print(recovery_message(phase, error), file=sys.stderr)
        print(f"Underlying error: {error}", file=sys.stderr)
        return 1


class Setup:
    def __init__(self, root=ROOT):
        self.root = root
        self.local = root / ".setup"
        self.python = root / ".venv" / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")
        self.uv = os.environ.get("MH3_UV", "uv")
        self.env = os.environ.copy()
        self.env.pop("VIRTUAL_ENV", None)
        self.env.pop("PYTHONPATH", None)
        self.env.pop("PYTHONHOME", None)
        self.env["UV_PROJECT_ENVIRONMENT"] = str(root / ".venv")
        self.phase = "install"

    def uv_run(self, *args, capture=False, check=True):
        return subprocess.run(
            [self.uv, *map(str, args)], cwd=self.root, env=self.env, text=True, capture_output=capture, check=check
        )

    def fingerprint(self):
        return hashlib.sha256(
            b"".join((self.root / name).read_bytes() for name in ("uv.lock", "pyproject.toml", ".python-version"))
            + platform.platform().encode()
        ).hexdigest()

    def inventory(self):
        result = self.uv_run("pip", "list", "--python", self.python, "--format", "json", capture=True)
        return sorted((package["name"], package["version"]) for package in json.loads(result.stdout))

    def reusable(self):
        try:
            state = json.loads((self.local / "state.json").read_text())
            return (
                state["fingerprint"] == self.fingerprint()
                and state["packages"] == [list(item) for item in self.inventory()]
                and self.uv_run("pip", "check", "--python", self.python, check=False).returncode == 0
            )
        except (OSError, ValueError, KeyError, subprocess.CalledProcessError):
            return False

    def probe(self):
        code = """import json, torch
device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
torch.ones(1, device=device).sum().item()
print(json.dumps({'device': device, 'torch': torch.__version__, 'hip': torch.version.hip, 'cuda': torch.version.cuda}))
"""
        result = subprocess.run(
            [str(self.python), "-c", code], cwd=self.root, env=self.env, capture_output=True, text=True, check=False
        )
        return json.loads(result.stdout) if result.returncode == 0 else None

    def install_torch(self, backend, constraints, version):
        # uv resolves the complete dependency graph before installing anything.
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
        self.uv_run("pip", "check", "--python", self.python)

    def smoke(self, backend):
        report = self.local / "smoke.json"
        report.unlink(missing_ok=True)
        result = subprocess.run(
            [str(self.python), str(self.root / "setup.py"), "--smoke-backend", backend, "--report", str(report)],
            cwd=self.root,
            env=self.env,
            check=False,
        )
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

    def install(self, cpu=False, reset=False):
        check_platform()
        for name in ("uv.lock", "pyproject.toml", ".python-version", "assets/MH3.usd", "config/mh3.yaml"):
            if not (self.root / name).is_file():
                raise RuntimeError(f"Missing {name}; download the complete repository, including its assets.")
        self.local.mkdir(exist_ok=True)
        lock = tomllib.loads((self.root / "uv.lock").read_text())
        reused = not reset and self.reusable()
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
        torch_version = next(package["version"] for package in lock["package"] if package["name"] == "torch")
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
        probe = self.probe()
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
        self.uv_run("pip", "check", "--python", self.python)
        self.phase = "build"
        print(
            "Verifying the MH3, physics, and camera rendering. The first kernel compilation can take several minutes.",
            flush=True,
        )
        result = self.smoke(backend)
        if not result["ok"] and backend != "cpu" and result.get("phase") in ("build", "physics"):
            print("Genesis GPU verification failed; retrying physics on the CPU.", flush=True)
            result = self.smoke("cpu")
        if not result["ok"]:
            raise RuntimeError(
                result.get("guidance") or recovery_message(result.get("phase", "build"), result.get("error", ""))
            )
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
        (self.local / "requirements.txt").write_text(
            self.uv_run("pip", "freeze", "--python", self.python, capture=True).stdout, encoding="utf-8"
        )
        print("Setup passed. Camera preview: .setup/preview.png. Start the simulator with your run script.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cpu", action="store_true", help="Use CPU physics and a CPU PyTorch wheel")
    parser.add_argument(
        "--reset", action="store_true", help="Restore the release environment and redetect accelerators"
    )
    parser.add_argument("--smoke-backend", choices=("auto", "cpu"), help=argparse.SUPPRESS)
    parser.add_argument("--report", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.smoke_backend:
        raise SystemExit(smoke_check(ROOT, args.smoke_backend, args.report))
    setup = Setup()
    try:
        setup.install(cpu=args.cpu, reset=args.reset)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Setup failed: {error}", file=sys.stderr)
        print(recovery_message(setup.phase, error), file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
