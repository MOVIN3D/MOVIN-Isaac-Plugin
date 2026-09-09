"""Guards the generated MOVINManV3 skeleton against its sources.

``data/movinman_v3_skeleton.xml`` must be exactly what
``scripts/generate_skeleton_mjcf.py`` produces from
``data/MOVINman_V3_local_rotation_identity_tpose.bvh`` (the T-pose whose local
rotations are all identity, i.e. parent-local offsets as streamed by MOVIN
Studio), and the SDK's ``movinman_v3`` preset must describe that MJCF.

Run with:  python -m unittest discover -s tests
"""

import importlib.util
import os
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (ROOT, os.path.join(ROOT, "movin_sdk_python"), os.path.join(ROOT, "scripts")):
    if path not in sys.path:
        sys.path.insert(0, path)

DATA = os.path.join(ROOT, "data")
V3_TPOSE = os.path.join(DATA, "MOVINman_V3_local_rotation_identity_tpose.bvh")
V3_MJCF = os.path.join(DATA, "movinman_v3_skeleton.xml")
V3_ACTOR_BVH = os.path.join(DATA, "test_V3.bvh")
LEGACY_MJCF = os.path.join(DATA, "movinman_skeleton.xml")


def mjcf_bodies(path):
    """(name, pos) of every <body> in document (depth-first) order."""
    return [
        (body.get("name"), body.get("pos"))
        for body in ET.parse(path).getroot().iter("body")
    ]


class V3SkeletonAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import generate_skeleton_mjcf as gen

        cls.gen = gen
        cls.tpose_root = gen.parse_bvh_hierarchy(V3_TPOSE)
        cls.tpose_names = [n.name for n in gen.dfs_bodies(cls.tpose_root)]
        cls.tpose_offsets = {n.name: n.offset for n in gen.dfs_bodies(cls.tpose_root)}

    def test_mjcf_is_the_generator_output_for_the_tpose(self):
        expected = self.gen.generate_mjcf(V3_TPOSE, "movinman_v3", LEGACY_MJCF)
        with open(V3_MJCF) as f:
            actual = f.read()
        self.assertEqual(
            actual, expected,
            "data/movinman_v3_skeleton.xml is stale: regenerate it with "
            "scripts/generate_skeleton_mjcf.py from the local-rotation-identity T-pose",
        )

    def test_body_order_matches_the_preset(self):
        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        names = tuple(name for name, _ in mjcf_bodies(V3_MJCF))
        self.assertEqual(names, MOVINMAN_V3_PRESET.body_names)
        self.assertEqual(names, tuple(self.tpose_names))

    def test_body_positions_are_the_tpose_offsets(self):
        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        for name, pos in mjcf_bodies(V3_MJCF):
            actual = tuple(float(v) for v in pos.split())
            if name == "Hips":
                expected = (0.0, 0.0, MOVINMAN_V3_PRESET.default_hips_height)
            else:
                expected = self.gen.yup_to_zup(self.tpose_offsets[name])
            for a, e in zip(actual, expected):
                self.assertAlmostEqual(a, e, places=6, msg=name)

    def test_hips_height_matches_the_preset(self):
        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        self.assertAlmostEqual(
            self.gen.compute_hips_height(self.tpose_root),
            MOVINMAN_V3_PRESET.default_hips_height,
            places=4,
        )

    def test_tpose_hierarchy_matches_studio_actor_exports(self):
        # The identity-local-rotation T-pose and Studio's actor BVH exports share
        # joint names, order, and the parent-local finger offsets (body segments
        # differ per actor calibration, fingers do not).
        actor_root = self.gen.parse_bvh_hierarchy(V3_ACTOR_BVH)
        actor = {n.name: n.offset for n in self.gen.dfs_bodies(actor_root)}
        self.assertEqual([n.name for n in self.gen.dfs_bodies(actor_root)], self.tpose_names)
        for name, offset in self.tpose_offsets.items():
            if "Hand" in name and not name.endswith("Hand"):
                for a, e in zip(actor[name], offset):
                    self.assertAlmostEqual(a, e, delta=1e-4, msg=name)

    @unittest.skipUnless(importlib.util.find_spec("mujoco") is not None, "mujoco not installed")
    def test_mjcf_loads_in_mujoco_with_preset_joints(self):
        import mujoco

        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        model = mujoco.MjModel.from_xml_path(V3_MJCF)
        self.assertEqual(model.nbody, 1 + len(MOVINMAN_V3_PRESET.body_names))  # + world
        self.assertEqual(model.njnt, 1 + 3 * len(MOVINMAN_V3_PRESET.joint_bones))  # + freejoint
        for bone in MOVINMAN_V3_PRESET.joint_bones:
            for axis in "xyz":
                self.assertGreaterEqual(
                    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{bone}_{axis}"), 0
                )


if __name__ == "__main__":
    unittest.main()
