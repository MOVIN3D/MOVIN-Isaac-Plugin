"""The plugin's packaging ships every file the SDK package ships.

The plugin installs the ``movin_sdk_python`` submodule through its own
setup.py / pyproject.toml, so their package-data has to cover every pattern in
the SDK's own pyproject.toml.  A missing pattern does not show with an editable
install (files are read from the source tree) but silently drops those SDK
files from a regular install or a wheel.  The wheel test builds the plugin
wheel from a temporary copy of the sources and checks that every SDK package
file matching the SDK's patterns is inside.

Run with:  python -m unittest discover -s tests
"""

import ast
import glob
import importlib.util
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile

try:
    import tomllib
except ModuleNotFoundError:  # Python < 3.11
    tomllib = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDK_ROOT = os.path.join(ROOT, "movin_sdk_python")
SDK_PACKAGE = "movin_sdk_python"
CAN_BUILD = all(importlib.util.find_spec(m) is not None for m in ("pip", "setuptools", "wheel"))


def pyproject_package_data(path):
    with open(path, "rb") as f:
        return tomllib.load(f)["tool"]["setuptools"]["package-data"]


def setup_py_package_data(path):
    """``package_data`` of the ``setup()`` call, read without running setup.py."""
    with open(path) as f:
        tree = ast.parse(f.read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "setup":
            for keyword in node.keywords:
                if keyword.arg == "package_data":
                    return ast.literal_eval(keyword.value)
    raise AssertionError(f"no package_data in the setup() call of {path}")


@unittest.skipUnless(tomllib, "tomllib needs Python 3.11+")
class PackageDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdk = pyproject_package_data(os.path.join(SDK_ROOT, "pyproject.toml"))

    def assert_covers_the_sdk(self, plugin, source):
        self.assertIn(SDK_PACKAGE, self.sdk)
        for package, patterns in self.sdk.items():
            missing = sorted(set(patterns) - set(plugin.get(package, [])))
            self.assertFalse(
                missing,
                f"{source} package-data for {package!r} lacks the SDK's {missing}",
            )

    def test_pyproject_covers_the_sdk_patterns(self):
        self.assert_covers_the_sdk(
            pyproject_package_data(os.path.join(ROOT, "pyproject.toml")), "pyproject.toml"
        )

    def test_setup_py_covers_the_sdk_patterns(self):
        self.assert_covers_the_sdk(setup_py_package_data(os.path.join(ROOT, "setup.py")), "setup.py")


@unittest.skipUnless(tomllib and CAN_BUILD, "needs Python 3.11+ with pip, setuptools and wheel")
class WheelTests(unittest.TestCase):
    def test_wheel_contains_every_sdk_package_file(self):
        package_dir = os.path.join(SDK_ROOT, SDK_PACKAGE)
        patterns = pyproject_package_data(os.path.join(SDK_ROOT, "pyproject.toml"))[SDK_PACKAGE]
        expected = set()
        for pattern in patterns:
            for path in glob.glob(os.path.join(package_dir, pattern), recursive=True):
                if "__pycache__" not in path and "usd_output" not in path:
                    expected.add(SDK_PACKAGE + "/" + os.path.relpath(path, package_dir).replace(os.sep, "/"))
        self.assertTrue(any(name.endswith(".npz") for name in expected))
        self.assertTrue(any(name.endswith(".xml") for name in expected))

        with tempfile.TemporaryDirectory() as tmp:
            # Build from a copy so no build/ or egg-info lands in the sources
            # (or in the submodule).
            src = os.path.join(tmp, "src")
            os.makedirs(os.path.join(src, "movin_sdk_python"))
            for name in ("setup.py", "pyproject.toml", "README.md"):
                shutil.copy(os.path.join(ROOT, name), src)
            shutil.copytree(
                package_dir, os.path.join(src, "movin_sdk_python", SDK_PACKAGE),
                ignore=shutil.ignore_patterns("__pycache__", "usd_output", "*.egg-info"),
            )
            out = os.path.join(tmp, "dist")
            result = subprocess.run(
                [sys.executable, "-m", "pip", "wheel", ".", "--no-deps", "--no-build-isolation",
                 "-q", "-w", out],
                cwd=src, capture_output=True, text=True, timeout=600,
            )
            self.assertEqual(result.returncode, 0, result.stdout[-2000:] + result.stderr[-2000:])
            wheels = glob.glob(os.path.join(out, "*.whl"))
            self.assertEqual(len(wheels), 1, wheels)
            with zipfile.ZipFile(wheels[0]) as wheel:
                names = set(wheel.namelist())

        missing = sorted(expected - names)
        self.assertFalse(missing, f"{len(missing)} SDK package files missing from the wheel: {missing[:10]}")


if __name__ == "__main__":
    unittest.main()
