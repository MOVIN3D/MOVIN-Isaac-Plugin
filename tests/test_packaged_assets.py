"""The runtime skeleton/mesh assets come from the SDK package and match this repo's sources.

``examples/mocap_to_isaaclab.py`` loads the MJCF and mesh that ship inside
MOVIN-SDK-Python (``SkeletonPreset.mjcf_path`` / ``mesh_npz_path``), while
this repo keeps their generation sources and generators.  These tests keep
the two from drifting apart:

* the packaged V3 MJCF is exactly what ``scripts/generate_skeleton_mjcf.py``
  produces from the identity-local T-pose;
* every packaged asset is byte-identical to its ``data/`` copy, for as long as
  ``data/`` still holds one;
* the presets resolve into the ``movin_sdk_python`` submodule (not another
  installed copy of the SDK).

Run with:  python -m unittest discover -s tests
"""

import hashlib
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUBMODULE = os.path.join(ROOT, "movin_sdk_python")
for path in (ROOT, SUBMODULE, os.path.join(ROOT, "scripts")):
    if path not in sys.path:
        sys.path.insert(0, path)

from movin_sdk_python.utils.skeleton_presets import PRESETS  # noqa: E402

DATA = os.path.join(ROOT, "data")
V3_TPOSE = os.path.join(DATA, "MOVINman_V3_local_rotation_identity_tpose.bvh")


def packaged_assets():
    """(preset, packaged path) for every MJCF and mesh asset of every preset."""
    for preset in PRESETS.values():
        yield preset, preset.mjcf_path
        if preset.mesh_npz_path is not None:
            yield preset, preset.mesh_npz_path


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


class PackagedAssetTests(unittest.TestCase):
    def test_presets_resolve_into_the_submodule(self):
        for preset, path in packaged_assets():
            self.assertTrue(
                os.path.realpath(path).startswith(os.path.realpath(SUBMODULE) + os.sep),
                f"{preset.name}: {path} is not inside the movin_sdk_python submodule",
            )
            self.assertTrue(os.path.exists(path), path)

    def test_packaged_v3_mjcf_is_the_generator_output(self):
        import generate_skeleton_mjcf as gen

        presets = {p.name: p for p in PRESETS.values()}
        v3, legacy = presets["movinman_v3"], presets["movinman"]
        expected = gen.generate_mjcf(V3_TPOSE, "movinman_v3", legacy.mjcf_path)
        with open(v3.mjcf_path) as f:
            self.assertEqual(
                f.read(), expected,
                "the SDK's packaged movinman_v3_skeleton.xml differs from "
                "scripts/generate_skeleton_mjcf.py's output for the T-pose",
            )

    def test_data_copies_match_the_packaged_assets(self):
        compared = 0
        for _, path in packaged_assets():
            copy = os.path.join(DATA, os.path.basename(path))
            if not os.path.exists(copy):
                continue
            compared += 1
            self.assertEqual(sha256(copy), sha256(path), f"data/{os.path.basename(path)} differs")
        if compared == 0:
            self.skipTest("data/ holds no copies of the packaged assets")


if __name__ == "__main__":
    unittest.main()
