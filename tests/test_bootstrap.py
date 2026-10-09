"""Exercise setup bootstrapping without downloading Python or project dependencies."""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(shutil.which("bash") is None, reason="Bash is required")
@pytest.mark.parametrize("platform", ["Linux", "Darwin"])
@pytest.mark.parametrize("existing_version", [None, "0.12.21", "0.12.23", "0.13.0"])
@pytest.mark.parametrize("install_exit", [0, 7])
def test_setup_bootstrap(tmp_path, platform, existing_version, install_exit):
    """Setup must upgrade stale uv, install Python first, and stop if installation fails."""
    project = tmp_path / "project with spaces"
    project.mkdir()
    for name in ("setup.sh", "uv.sh", ".python-version"):
        shutil.copy2(ROOT / name, project / name)
    commands = tmp_path / "bin"
    commands.mkdir()

    def executable(path, contents):
        path.write_text("#!/bin/sh\n" + contents)
        path.chmod(0o755)

    fake_uv = commands / "fake-uv"
    executable(
        fake_uv,
        'if [ "$1" = --version ]; then echo "uv ${FAKE_UV_VERSION:-0.12.23}"; exit; fi\n'
        'printf "%s\\n" "$*" >> "$CALL_LOG"\n'
        'if [ "$1" = python ]; then exit "$INSTALL_EXIT"; fi\n'
        "exit 0\n",
    )
    executable(
        commands / "uname",
        f'if [ "$1" = -s ]; then echo {platform}; else echo '
        + ("arm64" if platform == "Darwin" else "x86_64")
        + "; fi\n",
    )
    executable(commands / "sw_vers", "echo 14.0\n")
    executable(commands / "getconf", "echo 'glibc 2.28'\n")
    executable(
        commands / "curl",
        'printf "%s\\n" "$*" >> "$DOWNLOAD_LOG"\n'
        'while [ "$1" != -o ]; do shift; done\n'
        "cat > \"$2\" <<'INSTALLER'\n"
        'mkdir -p "$UV_UNMANAGED_INSTALL"\n'
        'cp "$FAKE_UV" "$UV_UNMANAGED_INSTALL/uv"\n'
        "INSTALLER\n",
    )
    if existing_version:
        (project / ".uv").mkdir()
        executable(project / ".uv/uv", f'export FAKE_UV_VERSION={existing_version}\nexec "$FAKE_UV" "$@"\n')

    env = os.environ.copy()
    env.update(
        PATH=f"{commands}:/usr/bin:/bin",
        FAKE_UV=str(fake_uv),
        INSTALL_EXIT=str(install_exit),
        CALL_LOG=str(tmp_path / "calls"),
        DOWNLOAD_LOG=str(tmp_path / "downloads"),
    )
    result = subprocess.run(
        ["bash", str(project / "setup.sh"), "--cpu"], env=env, capture_output=True, text=True, check=False
    )
    assert result.returncode == install_exit, result.stderr
    python_version = (project / ".python-version").read_text().strip()
    expected = [f"python install --no-bin --no-registry {python_version}"]
    if install_exit == 0:
        expected.append(f"run --no-project --python {python_version} python setup.py --cpu")
    assert (tmp_path / "calls").read_text().splitlines() == expected
    if existing_version in (None, "0.12.21"):
        assert "https://astral.sh/uv/0.12.23/install.sh" in (tmp_path / "downloads").read_text()
    else:
        assert not (tmp_path / "downloads").exists()
