# /// script
# requires-python = ">=3.11"
# dependencies = ["packaging>=25", "tomlkit>=0.13"]
# ///
"""Resolve upgrades against the latest stable Genesis release before saving."""

from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import subprocess
import sys
import tempfile
import tomllib
import urllib.request
from pathlib import Path

import tomlkit
from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet
from packaging.utils import canonicalize_name
from packaging.version import Version

PROJECT_ROOT = Path(__file__).resolve().parent
BUILD_GROUP = "upgrade-build"


def run_uv(*args: str, cwd: Path, capture: bool = False) -> subprocess.CompletedProcess:
    """Run uv outside the bootstrap script's isolated virtual environment."""
    env = os.environ.copy()
    env.pop("VIRTUAL_ENV", None)
    return subprocess.run(
        [os.environ.get("MH3_UV", "uv"), *args], cwd=cwd, env=env, text=True, capture_output=capture, check=True
    )


def latest_genesis() -> tuple[str, str]:
    """Read stable, non-yanked release metadata from the official PyPI API."""

    def fetch(url: str) -> dict:
        request = urllib.request.Request(url, headers={"User-Agent": "mh3-hackathon-upgrader"})
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.load(response)

    metadata = fetch("https://pypi.org/pypi/genesis-world/json")
    versions = [
        Version(version)
        for version, files in metadata["releases"].items()
        if not Version(version).is_prerelease
        and not Version(version).is_devrelease
        and any(not file.get("yanked", False) for file in files)
    ]
    version = str(max(versions))
    release = fetch(f"https://pypi.org/pypi/genesis-world/{version}/json")
    requires_python = release["info"]["requires_python"]
    if not requires_python:
        raise RuntimeError("Genesis did not publish a Python compatibility range.")
    return version, requires_python


def python_candidates(downloads: list[dict], genesis_range: str, project_range: str) -> list[Version]:
    """Choose the latest stable CPython patch per minor, newest minor first."""
    # Keep the code's minimum Python version, but allow its old ceiling to rise.
    minimum = SpecifierSet(",".join(str(spec) for spec in SpecifierSet(project_range) if spec.operator in (">", ">=")))
    supported = SpecifierSet(genesis_range)
    candidates: dict[tuple[int, int], Version] = {}
    for download in downloads:
        if download["implementation"] != "cpython" or download.get("variant", "default") != "default":
            continue
        version = Version(download["version"])
        if version.is_prerelease or version.is_devrelease or version not in supported or version not in minimum:
            continue
        minor = version.release[:2]
        candidates[minor] = max(candidates.get(minor, version), version)
    return sorted(candidates.values(), reverse=True)


def requirement_text(text: str, version: str | None = None, operator: str = "==") -> str:
    """Replace stale version bounds while preserving extras and markers."""
    requirement = Requirement(text)
    if requirement.url:
        raise RuntimeError(f"Cannot choose a registry upgrade for a direct URL dependency: {requirement.name}")
    extras = f"[{','.join(sorted(requirement.extras))}]" if requirement.extras else ""
    marker = f"; {requirement.marker}" if requirement.marker else ""
    constraint = f"{operator}{version}" if version else ""
    return f"{requirement.name}{extras}{constraint}{marker}"


def dependency_arrays(document):
    """Include runtime dependencies, extras, and every development group."""
    yield document["project"]["dependencies"]
    yield from document["project"].get("optional-dependencies", {}).values()
    yield from document.get("dependency-groups", {}).values()


def prepare(document, python: Version, genesis: str):
    """Relax direct bounds and include build requirements in the resolution."""
    document = copy.deepcopy(document)
    major, minor = python.release[:2]
    document["project"]["requires-python"] = f">={python},<{major}.{minor + 1}"
    document.setdefault("tool", {}).setdefault("ruff", {})["target-version"] = f"py{major}{minor}"
    document["tool"].setdefault("uv", {})["prerelease"] = "disallow"
    found_genesis = False
    for dependencies in dependency_arrays(document):
        for index, dependency in enumerate(dependencies):
            if not isinstance(dependency, str):  # Preserve include-group entries.
                continue
            is_genesis = canonicalize_name(Requirement(dependency).name) == "genesis-world"
            dependencies[index] = requirement_text(dependency, genesis if is_genesis else None)
            found_genesis |= is_genesis
    if not found_genesis:
        raise RuntimeError("The project does not declare Genesis World.")
    groups = document.setdefault("dependency-groups", {})
    if BUILD_GROUP in groups:
        raise RuntimeError(f"The reserved dependency group '{BUILD_GROUP}' already exists.")
    groups[BUILD_GROUP] = [requirement_text(text) for text in document["build-system"]["requires"]]
    return document


def pin_resolved(document, lock: dict):
    """Record compatible direct versions; keep transitive versions in uv.lock."""
    versions: dict[str, set[Version]] = {}
    for package in lock["package"]:
        versions.setdefault(canonicalize_name(package["name"]), set()).add(Version(package["version"]))
    for dependencies in dependency_arrays(document):
        for index, dependency in enumerate(dependencies):
            if not isinstance(dependency, str):
                continue
            name = canonicalize_name(Requirement(dependency).name)
            if name not in versions:
                # A marker can exclude a package on every supported target.
                continue
            if len(versions[name]) != 1:
                # Allow uv's platform-specific versions rather than forcing one.
                dependencies[index] = requirement_text(dependency, str(min(versions[name])), ">=")
            else:
                dependencies[index] = requirement_text(dependency, str(next(iter(versions[name]))))
    build = document["dependency-groups"].pop(BUILD_GROUP)
    document["build-system"]["requires"] = list(build)
    return document


def save_files(root: Path, files: dict[str, bytes]) -> None:
    """Restore the original metadata if writing any upgraded file fails."""
    original = {name: (root / name).read_bytes() if (root / name).exists() else None for name in files}
    try:
        for name, content in files.items():
            (root / name).write_bytes(content)
    except BaseException:
        for name, content in original.items():
            if content is None:
                (root / name).unlink(missing_ok=True)
            else:
                (root / name).write_bytes(content)
        raise


def upgrade(root: Path, *, dry_run: bool = False, lock_only: bool = False) -> None:
    original = tomlkit.parse((root / "pyproject.toml").read_text(encoding="utf-8"))
    genesis, supported = latest_genesis()
    downloads = json.loads(
        run_uv("python", "list", "--only-downloads", "--output-format", "json", cwd=root, capture=True).stdout
    )
    candidates = python_candidates(downloads, supported, original["project"]["requires-python"])
    if not candidates:
        raise RuntimeError(
            f"No stable CPython download satisfies Genesis {genesis} ({supported}) and the code minimum."
        )
    print(f"Latest stable Genesis: {genesis}; supported Python: {supported}", flush=True)
    with tempfile.TemporaryDirectory(prefix="mh3-upgrade-") as temporary:
        staged = Path(temporary)
        shutil.copy2(root / "README.md", staged / "README.md")
        shutil.copytree(root / "src", staged / "src")
        last_error = ""
        for python in candidates:
            print(f"Resolving all dependencies with Python {python}…", flush=True)
            document = prepare(original, python, genesis)
            (staged / "pyproject.toml").write_text(tomlkit.dumps(document), encoding="utf-8")
            (staged / ".python-version").write_text(f"{python}\n", encoding="utf-8")
            (staged / "uv.lock").unlink(missing_ok=True)
            try:
                run_uv(
                    "lock",
                    "--upgrade",
                    "--resolution",
                    "highest",
                    "--prerelease",
                    "disallow",
                    "--python",
                    str(python),
                    cwd=staged,
                    capture=True,
                )
            except subprocess.CalledProcessError as error:
                last_error = error.stderr or str(error)
                if "No solution found" not in last_error:
                    raise RuntimeError(f"uv could not complete dependency resolution:\n{last_error}") from error
                print(f"Python {python} could not resolve the combined requirements:\n{last_error}", file=sys.stderr)
                continue
            lock = tomllib.loads((staged / "uv.lock").read_text(encoding="utf-8"))
            document = pin_resolved(document, lock)
            (staged / "pyproject.toml").write_text(tomlkit.dumps(document), encoding="utf-8")
            # Remove the temporary build group and check the final declarations.
            run_uv("lock", "--python", str(python), "--prerelease", "disallow", cwd=staged)
            break
        else:
            raise RuntimeError(f"No compatible Python/dependency combination was found.\n{last_error}")
        print(f"Selected Python {python}. Direct dependencies:", flush=True)
        for dependencies in dependency_arrays(document):
            for dependency in dependencies:
                if isinstance(dependency, str):
                    print(f"  {dependency}")
        print(f"Build requirements: {list(document['build-system']['requires'])}", flush=True)
        if dry_run:
            print("Dry run complete; project files and .venv were not changed.")
            return
        save_files(
            root, {name: (staged / name).read_bytes() for name in ("pyproject.toml", "uv.lock", ".python-version")}
        )
    print("Updated pyproject.toml, uv.lock, and .python-version.", flush=True)
    if lock_only:
        print("Run the setup script to install the upgraded environment.")
        return
    if sys.platform == "win32":
        powershell = shutil.which("pwsh") or shutil.which("powershell")
        if not powershell:
            raise RuntimeError("PowerShell was not found; run setup.ps1 to install the updated lockfile.")
        command = [powershell, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(root / "setup.ps1")]
    else:
        command = ["bash", str(root / "setup.sh")]
    env = os.environ.copy()
    env.pop("VIRTUAL_ENV", None)
    subprocess.run(command, cwd=root, env=env, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    options = parser.add_mutually_exclusive_group()
    options.add_argument(
        "--dry-run", action="store_true", help="Resolve and report without changing project files or .venv"
    )
    options.add_argument(
        "--lock-only", action="store_true", help="Update project files without installing the environment"
    )
    args = parser.parse_args()
    try:
        upgrade(PROJECT_ROOT, dry_run=args.dry_run, lock_only=args.lock_only)
    except (RuntimeError, OSError, subprocess.CalledProcessError, ValueError) as error:
        print(f"Upgrade failed: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
