"""Keep Python compatibility separate from the bootstrap patch release."""

import importlib.util
import tomllib
from pathlib import Path

import pytest
import tomlkit
from packaging.specifiers import SpecifierSet
from packaging.version import Version

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("mh3_upgrade", ROOT / "upgrade.py")
upgrade = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(upgrade)


def test_release_accepts_python_3139():
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())
    lock = tomllib.loads((ROOT / "uv.lock").read_text())
    for requirement in (project["project"]["requires-python"], lock["requires-python"]):
        supported = SpecifierSet(requirement)
        assert Version("3.13.9") in supported
        assert Version((ROOT / ".python-version").read_text().strip()) in supported
        assert Version("3.12.9") not in supported
        assert Version("3.14.0") not in supported


@pytest.mark.parametrize("python", ["3.13.16", "3.14.7"])
def test_upgrade_does_not_require_selected_patch(python):
    original = tomlkit.parse((ROOT / "pyproject.toml").read_text())
    selected = Version(python)
    document = upgrade.prepare(original, selected, "1.4.3")
    supported = SpecifierSet(document["project"]["requires-python"])
    major, minor = selected.release[:2]
    assert Version(f"{major}.{minor}.9") in supported
    assert selected in supported
    assert Version(f"{major}.{minor - 1}.9") not in supported
    assert Version(f"{major}.{minor + 1}.0") not in supported
    assert document["tool"]["ruff"]["target-version"] == f"py{major}{minor}"
    assert document is not original
