"""Verify platform dependencies and accelerator failure handling during setup."""

import importlib.util
import json
import shutil
import subprocess
import tomllib
from pathlib import Path

import pytest
from packaging.markers import Marker

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("mh3_setup", ROOT / "setup.py")
setup = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(setup)


def test_windows_runtime_is_locked():
    """The Windows install must include PyMeshLab's runtime without affecting other platforms."""
    lock = tomllib.loads((ROOT / "uv.lock").read_text())
    packages = {package["name"]: package for package in lock["package"]}
    dependency = next(dep for dep in packages["mh3-genesis"]["dependencies"] if dep["name"] == "msvc-runtime")
    marker = Marker(dependency["marker"])
    assert marker.evaluate({"sys_platform": "win32"})
    assert not marker.evaluate({"sys_platform": "linux"})
    assert not marker.evaluate({"sys_platform": "darwin"})
    assert any("cp313-cp313-win_amd64.whl" in wheel["url"] for wheel in packages["msvc-runtime"]["wheels"])
    assert "msvc-runtime" not in setup.accelerator_exclusions(lock)


@pytest.fixture
def installer(tmp_path, monkeypatch):
    """Isolate setup orchestration from downloads, native libraries, and GPU hardware."""
    for name in ("uv.lock", "pyproject.toml", ".python-version"):
        shutil.copy2(ROOT / name, tmp_path / name)
    for name in ("assets/MH3.usd", "config/mh3.yaml"):
        path = tmp_path / name
        path.parent.mkdir(exist_ok=True)
        path.touch()
    monkeypatch.setattr(setup, "check_platform", lambda: None)
    instance = setup.Setup(tmp_path)
    monkeypatch.setattr(instance, "reusable", lambda: False)
    monkeypatch.setattr(instance, "inventory", list)
    calls = []

    def uv_run(*args, **kwargs):
        calls.append(args)
        return subprocess.CompletedProcess(args, 0, stdout="numpy==2.5.3\n")

    monkeypatch.setattr(instance, "uv_run", uv_run)
    monkeypatch.setattr(instance, "smoke", lambda backend: {"ok": True, "backend": backend})
    return instance, calls, uv_run


def test_dependency_error_does_not_replace_gpu_build(installer, monkeypatch):
    """An unrelated package check failure must propagate without reinstalling Torch on CPU."""
    instance, calls, uv_run = installer
    probes = iter([None, {"device": "cuda", "torch": "2.14.1+cu132", "cuda": "13.2", "hip": None}])
    monkeypatch.setattr(instance, "probe", lambda: next(probes))

    def missing_runtime(*args, **kwargs):
        if args[:2] == ("pip", "check"):
            calls.append(args)
            raise subprocess.CalledProcessError(1, args, stderr="pymeshlab requires msvc-runtime")
        return uv_run(*args, **kwargs)

    monkeypatch.setattr(instance, "uv_run", missing_runtime)
    with pytest.raises(subprocess.CalledProcessError, match="pip.*check"):
        instance.install()
    installations = [call for call in calls if call[:2] == ("pip", "install")]
    assert len(installations) == 1
    assert installations[0][installations[0].index("--torch-backend") + 1] == "auto"
    assert sum(call[0] == "sync" for call in calls) == 1
    assert not (instance.local / "state.json").exists()


@pytest.mark.parametrize("failure", [None, "download", "tensor"])
def test_accelerator_selection(installer, monkeypatch, failure):
    """Keep a working GPU build and retain CPU fallback for actual accelerator failures."""
    instance, calls, uv_run = installer
    probes = iter([None, None if failure == "tensor" else {"device": "cuda"}])
    monkeypatch.setattr(instance, "probe", lambda: next(probes))

    def install_failure(*args, **kwargs):
        result = uv_run(*args, **kwargs)
        if failure == "download" and args[:2] == ("pip", "install") and "auto" in args:
            raise subprocess.CalledProcessError(1, args)
        return result

    monkeypatch.setattr(instance, "uv_run", install_failure)
    instance.install()
    backends = [call[call.index("--torch-backend") + 1] for call in calls if call[:2] == ("pip", "install")]
    assert backends == (["auto"] if failure is None else ["auto", "cpu"])
    assert sum(call[:2] == ("pip", "check") for call in calls) == 1
    assert json.loads((instance.local / "state.json").read_text())["backend"] == "auto"
