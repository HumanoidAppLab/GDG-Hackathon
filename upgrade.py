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
from typing import Any

import tomlkit
from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet
from packaging.utils import canonicalize_name
from packaging.version import Version

PROJECT_ROOT = Path(__file__).resolve().parent
BUILD_GROUP = "upgrade-build"


def run_uv(*args: str, cwd: Path, capture: bool = False) -> subprocess.CompletedProcess:
    """Run uv outside the bootstrap script's isolated virtual environment.
    :param args: Command line arguments passed to uv.
    :type args: str
    :param cwd: Working directory for running uv.
    :type cwd: Path
    :param capture: Capture standard output and error streams when True.
    :type capture: bool
    :return: Completed subprocess execution result.
    :rtype: subprocess.CompletedProcess
    """
    # Prepare environment with isolated virtual environment references removed.
    env = os.environ.copy()
    env.pop("VIRTUAL_ENV", None)
    # Execute the requested uv sub-command.
    return subprocess.run(
        [os.environ.get("MH3_UV", "uv"), *args], cwd=cwd, env=env, text=True, capture_output=capture, check=True
    )


def latest_genesis() -> tuple[str, str]:
    """Read stable, non-yanked release metadata from the official PyPI API.
    :return: Tuple containing the latest version string and its required Python version range.
    :rtype: tuple[str, str]
    """

    def fetch(url: str) -> dict[str, Any]:
        """Fetch and deserialize JSON metadata from a PyPI endpoint.
        :param url: Target HTTP URL for the PyPI JSON API.
        :type url: str
        :return: Deserialized JSON response payload.
        :rtype: dict[str, Any]
        """
        request = urllib.request.Request(url, headers={"User-Agent": "mh3-hackathon-upgrader"})
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.load(response)

    # Fetch release manifest for the Genesis World project from PyPI.
    metadata = fetch("https://pypi.org/pypi/genesis-world/json")
    # Identify non-yanked, stable production releases.
    versions = [
        Version(version)
        for version, files in metadata["releases"].items()
        if not Version(version).is_prerelease
        and not Version(version).is_devrelease
        and any(not file.get("yanked", False) for file in files)
    ]
    version = str(max(versions))
    # Query specific release details to extract required Python compatibility specifiers.
    release = fetch(f"https://pypi.org/pypi/genesis-world/{version}/json")
    requires_python = release["info"]["requires_python"]
    if not requires_python:
        raise RuntimeError("Genesis did not publish a Python compatibility range.")
    return version, requires_python


def python_candidates(downloads: list[dict[str, Any]], genesis_range: str, project_range: str) -> list[Version]:
    """Choose the latest stable CPython patch per minor, newest minor first.
    :param downloads: Available managed Python release records from uv.
    :type downloads: list[dict[str, Any]]
    :param genesis_range: Python version compatibility range declared by Genesis.
    :type genesis_range: str
    :param project_range: Current project Python requirement specifier string.
    :type project_range: str
    :return: List of compatible candidate CPython versions sorted in descending order.
    :rtype: list[Version]
    """
    # Keep the code's minimum Python version, but allow its old ceiling to rise.
    minimum = SpecifierSet(",".join(str(spec) for spec in SpecifierSet(project_range) if spec.operator in (">", ">=")))
    supported = SpecifierSet(genesis_range)
    candidates: dict[tuple[int, int], Version] = {}
    # Iterate through available downloads and filter for stable CPython builds meeting requirements.
    for download in downloads:
        if download["implementation"] != "cpython" or download.get("variant", "default") != "default":
            continue
        version = Version(download["version"])
        if version.is_prerelease or version.is_devrelease or version not in supported or version not in minimum:
            continue
        minor = version.release[:2]
        # Keep the newest patch release for each compatible minor version series.
        candidates[minor] = max(candidates.get(minor, version), version)
    return sorted(candidates.values(), reverse=True)


def requirement_text(text: str, version: str | None = None, operator: str = "==") -> str:
    """Replace stale version bounds while preserving extras and markers.
    :param text: Original dependency requirement specification text.
    :type text: str
    :param version: New target version string to pin, or None to remove version constraints.
    :type version: str | None
    :param operator: Comparison operator prefix to apply when version is provided.
    :type operator: str
    :return: Formatted requirement string preserving extras and environment markers.
    :rtype: str
    """
    requirement = Requirement(text)
    # Reject direct URL dependencies as they cannot be upgraded through the registry.
    if requirement.url:
        raise RuntimeError(f"Cannot choose a registry upgrade for a direct URL dependency: {requirement.name}")
    # Reassemble extras, environment markers, and updated version specifiers.
    extras = f"[{','.join(sorted(requirement.extras))}]" if requirement.extras else ""
    marker = f"; {requirement.marker}" if requirement.marker else ""
    constraint = f"{operator}{version}" if version else ""
    return f"{requirement.name}{extras}{constraint}{marker}"


def dependency_arrays(document: Any) -> Any:
    """Include runtime dependencies, extras, and every development group.
    :param document: Parsed pyproject.toml document model.
    :type document: Any
    :return: Generator yielding mutable dependency lists.
    :rtype: Any
    """
    # Yield core runtime dependencies.
    yield document["project"]["dependencies"]
    # Yield optional dependency extras groups.
    yield from document["project"].get("optional-dependencies", {}).values()
    # Yield development dependency groups.
    yield from document.get("dependency-groups", {}).values()


def prepare(document: Any, python: Version, genesis: str) -> Any:
    """Relax direct bounds and include build requirements in the resolution.
    :param document: Original parsed pyproject.toml document model.
    :type document: Any
    :param python: Candidate CPython version being tested.
    :type python: Version
    :param genesis: Target Genesis release version string.
    :type genesis: str
    :return: Modified pyproject.toml document configured for resolution.
    :rtype: Any
    """
    # Work on a deep copy of the configuration document.
    document = copy.deepcopy(document)
    major, minor = python.release[:2]
    # Update requires-python range and linter target to align with the candidate Python minor.
    document["project"]["requires-python"] = f">={python},<{major}.{minor + 1}"
    document.setdefault("tool", {}).setdefault("ruff", {})["target-version"] = f"py{major}{minor}"
    document["tool"].setdefault("uv", {})["prerelease"] = "disallow"
    found_genesis = False
    # Relax pinned versions on dependencies while fixing Genesis to the latest stable release.
    for dependencies in dependency_arrays(document):
        for index, dependency in enumerate(dependencies):
            # Preserve include-group entries.
            if not isinstance(dependency, str):
                continue
            is_genesis = canonicalize_name(Requirement(dependency).name) == "genesis-world"
            dependencies[index] = requirement_text(dependency, genesis if is_genesis else None)
            found_genesis |= is_genesis
    if not found_genesis:
        raise RuntimeError("The project does not declare Genesis World.")
    # Inject build system requirements into a temporary dependency group for resolution.
    groups = document.setdefault("dependency-groups", {})
    if BUILD_GROUP in groups:
        raise RuntimeError(f"The reserved dependency group '{BUILD_GROUP}' already exists.")
    groups[BUILD_GROUP] = [requirement_text(text) for text in document["build-system"]["requires"]]
    return document


def pin_resolved(document: Any, lock: dict[str, Any]) -> Any:
    """Record compatible direct versions; keep transitive versions in uv.lock.
    :param document: Staged pyproject.toml document model.
    :type document: Any
    :param lock: Parsed uv.lock data structure.
    :type lock: dict[str, Any]
    :return: Final pyproject.toml document model with updated direct dependency pins.
    :rtype: Any
    """
    # Index resolved versions for each package across the locked environment graph.
    versions: dict[str, set[Version]] = {}
    for package in lock["package"]:
        versions.setdefault(canonicalize_name(package["name"]), set()).add(Version(package["version"]))
    # Pin direct dependencies in the document using versions verified in the lockfile.
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
    # Remove the temporary build dependency group and update build system requirements.
    build = document["dependency-groups"].pop(BUILD_GROUP)
    document["build-system"]["requires"] = list(build)
    return document


def save_files(root: Path, files: dict[str, bytes]) -> None:
    """Restore the original metadata if writing any upgraded file fails.
    :param root: Root directory of the repository.
    :type root: Path
    :param files: Mapping of relative paths to new file contents in bytes.
    :type files: dict[str, bytes]
    :return: None after writing files to disk.
    :rtype: None
    """
    # Snapshot existing file contents to allow atomic rollback if an error occurs.
    original = {name: (root / name).read_bytes() if (root / name).exists() else None for name in files}
    try:
        # Write updated contents for each upgraded file.
        for name, content in files.items():
            (root / name).write_bytes(content)
    except BaseException:
        # Restore previous file contents on disk if any write fails.
        for name, content in original.items():
            if content is None:
                (root / name).unlink(missing_ok=True)
            else:
                (root / name).write_bytes(content)
        raise


def upgrade(root: Path, *, dry_run: bool = False, lock_only: bool = False) -> None:
    """Resolve upgrades against the latest stable Genesis release and test them.
    :param root: Repository root directory.
    :type root: Path
    :param dry_run: Preview dependency resolution without modifying repository files.
    :type dry_run: bool
    :param lock_only: Update project release files without invoking local setup.
    :type lock_only: bool
    :return: None after completing upgrade workflow.
    :rtype: None
    """
    # Read the current pyproject.toml configuration.
    original = tomlkit.parse((root / "pyproject.toml").read_text(encoding="utf-8"))
    # Query PyPI for the newest stable Genesis World release and Python compatibility range.
    genesis, supported = latest_genesis()
    downloads = json.loads(
        run_uv("python", "list", "--only-downloads", "--output-format", "json", cwd=root, capture=True).stdout
    )
    # Determine candidate CPython versions meeting compatibility constraints.
    candidates = python_candidates(downloads, supported, original["project"]["requires-python"])
    if not candidates:
        raise RuntimeError(
            f"No stable CPython download satisfies Genesis {genesis} ({supported}) and the code minimum."
        )
    print(f"Latest stable Genesis: {genesis}; supported Python: {supported}", flush=True)
    # Create an isolated staging area to attempt dependency resolution.
    with tempfile.TemporaryDirectory(prefix="mh3-upgrade-") as temporary:
        staged = Path(temporary)
        shutil.copy2(root / "README.md", staged / "README.md")
        shutil.copytree(root / "src", staged / "src")
        last_error = ""
        # Try candidate Python versions from newest to oldest until a valid solution resolves.
        for python in candidates:
            print(f"Resolving all dependencies with Python {python}…", flush=True)
            document = prepare(original, python, genesis)
            (staged / "pyproject.toml").write_text(tomlkit.dumps(document), encoding="utf-8")
            (staged / ".python-version").write_text(f"{python}\n", encoding="utf-8")
            (staged / "uv.lock").unlink(missing_ok=True)
            try:
                # Solve full dependency graph using uv lock with highest resolution.
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
            # Read generated lockfile and pin resolved versions back into pyproject.toml.
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
        # Terminate without altering project files if running in preview mode.
        if dry_run:
            print("Dry run complete; project files and .venv were not changed.")
            return
        # Write staged release files back to the project root.
        save_files(
            root, {name: (staged / name).read_bytes() for name in ("pyproject.toml", "uv.lock", ".python-version")}
        )
    print("Updated pyproject.toml, uv.lock, and .python-version.", flush=True)
    # Stop before running local setup if lock-only execution was requested.
    if lock_only:
        print("Run the setup script to install the upgraded environment.")
        return
    # Dispatch platform setup script to install and verify the newly locked release.
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
    """Parse command-line arguments and run the upgrade workflow.
    :return: None because the process exits.
    :rtype: None
    """
    # Parse CLI arguments for previewing or restricting the upgrade workflow.
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
        # Run the full dependency upgrade pipeline.
        upgrade(PROJECT_ROOT, dry_run=args.dry_run, lock_only=args.lock_only)
    except (RuntimeError, OSError, subprocess.CalledProcessError, ValueError) as error:
        print(f"Upgrade failed: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
