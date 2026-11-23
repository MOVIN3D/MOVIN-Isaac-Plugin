"""Body-proportion fitting as examples/mocap_to_isaaclab.py drives it (no Isaac needed).

Covers ``bvh_utils.bvh_rest_bones`` (BVH OFFSETs -> stream-format bones), the
skeleton USD cache layout of ``skeleton_assets``, and the SDK fitting and
Retargeter sizing calls the example makes with those bones.

Run with:  python -m unittest discover -s tests
"""

import importlib.util
import os
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (ROOT, os.path.join(ROOT, "movin_sdk_python"), os.path.join(ROOT, "examples")):
    if path not in sys.path:
        sys.path.insert(0, path)

from bvh_utils import bvh_rest_bones, read_bvh  # noqa: E402
from skeleton_assets import prepare_skeleton_asset, proportions_key  # noqa: E402
from movin_sdk_python.utils.skeleton_fitting import fit_skeleton_mjcf, stream_offsets  # noqa: E402
from movin_sdk_python.utils.skeleton_presets import (  # noqa: E402
    ASSET_DIR,
    MOVINMAN_PRESET,
    MOVINMAN_V3_PRESET,
)

DATA = os.path.join(ROOT, "data")
V3_ACTOR_BVH = os.path.join(DATA, "test_V3.bvh")
V3_TPOSE_BVH = os.path.join(DATA, "MOVINman_V3_local_rotation_identity_tpose.bvh")
LEGACY_BVH = os.path.join(DATA, "Locomotion.bvh")

HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None
HAS_RETARGETER = HAS_MUJOCO and importlib.util.find_spec("mink") is not None


def mjcf_bodies(xml):
    """{name: (parent_name, pos_zup)} of an MJCF given as text or path."""
    root = ET.fromstring(xml) if xml.lstrip().startswith("<") else ET.parse(xml).getroot()
    bodies = {}

    def walk(element, parent):
        for body in element.findall("body"):
            pos = tuple(float(c) for c in body.get("pos", "0 0 0").split())
            bodies[body.get("name")] = (parent, pos)
            walk(body, body.get("name"))

    walk(root.find("worldbody"), None)
    return bodies


def yup_to_zup(v):
    return (v[0], -v[2], v[1])


def zero_pose_heights(bodies):
    """World Z of every body with all joints at zero (offsets simply add up)."""
    z = {}
    for name, (parent, pos) in bodies.items():  # document order: parents first
        z[name] = pos[2] + (z[parent] if parent is not None else 0.0)
    return z


def scaled_bones(bones, factor):
    """Same skeleton, every bone offset scaled (a differently sized actor)."""
    out = []
    for bone in bones:
        bone = dict(bone)
        bone["p"] = tuple(c * factor for c in bone["p"])
        out.append(bone)
    return out


class BvhRestBonesTests(unittest.TestCase):
    def test_stream_offsets_give_back_the_bvh_offsets(self):
        data = read_bvh(V3_ACTOR_BVH)
        offsets = stream_offsets(bvh_rest_bones(data))
        for i, name in enumerate(data.bones):
            parent, offset = offsets[name]
            expected_parent = data.bones[data.parents[i]] if data.parents[i] >= 0 else None
            self.assertEqual(parent, expected_parent, name)
            for a, e in zip(offset, data.offsets[i]):
                self.assertAlmostEqual(a, e, places=9, msg=name)

    def test_scale_converts_units(self):
        data = read_bvh(V3_ACTOR_BVH)
        unit = bvh_rest_bones(data)
        cm = bvh_rest_bones(data, scale=100.0)
        for a, b in zip(unit, cm):
            for pa, pb in zip(a["p"], b["p"]):
                self.assertAlmostEqual(pb, pa * 100.0, places=6)


class SkeletonFitTests(unittest.TestCase):
    def test_fitted_v3_bodies_sit_at_the_bvh_offsets(self):
        data = read_bvh(V3_ACTOR_BVH)
        bodies = mjcf_bodies(fit_skeleton_mjcf(bvh_rest_bones(data), MOVINMAN_V3_PRESET))
        self.assertEqual(tuple(bodies), MOVINMAN_V3_PRESET.body_names)
        for i, name in enumerate(data.bones[1:], start=1):
            for a, e in zip(bodies[name][1], yup_to_zup(data.offsets[i])):
                self.assertAlmostEqual(a, e, places=5, msg=name)

    def test_tpose_fit_reproduces_the_reference(self):
        # The packaged V3 MJCF is generated from this T-pose, so fitting to it
        # must change nothing.
        data = read_bvh(V3_TPOSE_BVH)
        fitted = mjcf_bodies(fit_skeleton_mjcf(bvh_rest_bones(data), MOVINMAN_V3_PRESET))
        reference = mjcf_bodies(MOVINMAN_V3_PRESET.mjcf_path)
        self.assertEqual(fitted.keys(), reference.keys())
        for name, (parent, pos) in reference.items():
            self.assertEqual(fitted[name][0], parent)
            for a, e in zip(fitted[name][1], pos):
                self.assertAlmostEqual(a, e, places=5, msg=name)

    def test_legacy_bvh_fits_its_bones_and_keeps_the_rest(self):
        data = read_bvh(LEGACY_BVH)
        fitted = mjcf_bodies(fit_skeleton_mjcf(bvh_rest_bones(data), MOVINMAN_PRESET))
        reference = mjcf_bodies(MOVINMAN_PRESET.mjcf_path)
        in_bvh = set(data.bones[1:])
        for name, (_, pos) in reference.items():
            if name == "Hips":
                continue
            if name in in_bvh:
                offset = data.offsets[data.bones.index(name)]
                for a, e in zip(fitted[name][1], yup_to_zup(offset)):
                    self.assertAlmostEqual(a, e, places=5, msg=name)
            else:
                self.assertEqual(fitted[name][1], pos, name)

    def test_fitted_skeleton_keeps_the_reference_ground_clearance(self):
        # The root is raised/lowered by the leg-length change, so the actor's
        # T-pose stands on the ground like the reference does.
        data = read_bvh(V3_ACTOR_BVH)
        fitted = zero_pose_heights(mjcf_bodies(fit_skeleton_mjcf(bvh_rest_bones(data), MOVINMAN_V3_PRESET)))
        reference = zero_pose_heights(mjcf_bodies(MOVINMAN_V3_PRESET.mjcf_path))
        self.assertNotAlmostEqual(fitted["Hips"], reference["Hips"], places=3)
        self.assertAlmostEqual(min(fitted.values()), min(reference.values()), places=3)

    def test_v3_bones_on_the_legacy_preset_raise(self):
        bones = bvh_rest_bones(read_bvh(V3_ACTOR_BVH))
        with tempfile.TemporaryDirectory() as cache:
            with self.assertRaises(ValueError):
                prepare_skeleton_asset(MOVINMAN_PRESET, bones, cache_root=cache)

    @unittest.skipUnless(HAS_MUJOCO, "mujoco not installed")
    def test_fitted_mjcf_loads_in_mujoco(self):
        import mujoco

        bones = bvh_rest_bones(read_bvh(V3_ACTOR_BVH))
        model = mujoco.MjModel.from_xml_string(fit_skeleton_mjcf(bones, MOVINMAN_V3_PRESET))
        self.assertEqual(model.nbody, 1 + len(MOVINMAN_V3_PRESET.body_names))
        self.assertEqual(model.njnt, 1 + 3 * len(MOVINMAN_V3_PRESET.joint_bones))


class SkeletonCacheTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.cache = self._tmp.name
        self.bones = bvh_rest_bones(read_bvh(V3_ACTOR_BVH))

    def tearDown(self):
        self._tmp.cleanup()

    def test_reference_uses_the_packaged_mjcf_and_writes_nothing(self):
        asset = prepare_skeleton_asset(MOVINMAN_V3_PRESET, None, cache_root=self.cache)
        self.assertFalse(asset.fitted)
        self.assertEqual(asset.mjcf_path, MOVINMAN_V3_PRESET.mjcf_path)
        self.assertEqual(os.path.dirname(asset.usd_dir), self.cache)
        self.assertTrue(os.path.basename(asset.usd_dir).startswith("movinman_v3_skeleton_reference_"))
        self.assertEqual(asset.usd_file_name, "movinman_v3_skeleton.usd")
        self.assertEqual(os.listdir(self.cache), [])

    def test_fitted_mjcf_is_written_to_its_own_cache_dir(self):
        before = sorted(os.listdir(ASSET_DIR))
        asset = prepare_skeleton_asset(MOVINMAN_V3_PRESET, self.bones, cache_root=self.cache)
        self.assertTrue(asset.fitted)
        self.assertEqual(os.path.dirname(asset.usd_dir), self.cache)
        self.assertTrue(os.path.basename(asset.usd_dir).startswith("movinman_v3_skeleton_fitted_"))
        self.assertEqual(os.path.dirname(asset.mjcf_path), asset.usd_dir)
        with open(asset.mjcf_path) as f:
            self.assertEqual(f.read(), fit_skeleton_mjcf(self.bones, MOVINMAN_V3_PRESET))
        self.assertEqual(sorted(os.listdir(ASSET_DIR)), before)  # SDK package untouched

    def test_same_actor_reuses_the_cache_without_rewriting(self):
        first = prepare_skeleton_asset(MOVINMAN_V3_PRESET, self.bones, cache_root=self.cache)
        mtime = os.path.getmtime(first.mjcf_path)
        time.sleep(0.02)
        again = prepare_skeleton_asset(MOVINMAN_V3_PRESET, self.bones, cache_root=self.cache)
        self.assertEqual(again, first)
        # An unchanged MJCF keeps its mtime, so a cached USD stays valid.
        self.assertEqual(os.path.getmtime(again.mjcf_path), mtime)

    def test_other_actor_and_reference_get_their_own_dirs(self):
        actor_a = prepare_skeleton_asset(MOVINMAN_V3_PRESET, self.bones, cache_root=self.cache)
        actor_b = prepare_skeleton_asset(
            MOVINMAN_V3_PRESET, scaled_bones(self.bones, 1.05), cache_root=self.cache
        )
        reference = prepare_skeleton_asset(MOVINMAN_V3_PRESET, None, cache_root=self.cache)
        self.assertEqual(len({actor_a.usd_dir, actor_b.usd_dir, reference.usd_dir}), 3)
        legacy = prepare_skeleton_asset(MOVINMAN_PRESET, None, cache_root=self.cache)
        self.assertNotEqual(legacy.usd_dir, reference.usd_dir)


class MeshFitTests(unittest.TestCase):
    def test_v3_mesh_fits_to_bvh_bones(self):
        import numpy as np

        from movin_sdk_python.utils.movinman_mesh_utils import MOVINMeshModel

        data = read_bvh(V3_ACTOR_BVH)
        model = MOVINMeshModel.from_preset(MOVINMAN_V3_PRESET)
        identity = np.tile([1.0, 0.0, 0.0, 0.0], (len(model.bone_names), 1))

        def tpose_lowest_vertex():
            rest_hips = np.asarray(model._bone_offsets[0])  # rest hips, Y-up m
            return model.pose_vertices(identity, rest_hips)[:, 2].min()

        reference_low = tpose_lowest_vertex()
        model.fit_to_bones(bvh_rest_bones(data))
        self.assertTrue(model.is_fitted)
        self.assertEqual(set(model.fitted_bones), set(MOVINMAN_V3_PRESET.body_names[1:]))
        # The actor's T-pose stands on the ground like the reference one.
        self.assertLess(abs(reference_low), 0.05)
        self.assertAlmostEqual(tpose_lowest_vertex(), reference_low, delta=0.01)

    def test_legacy_mesh_fits_to_legacy_bvh_bones(self):
        from movin_sdk_python.utils.movinman_mesh_utils import MOVINMeshModel

        data = read_bvh(LEGACY_BVH)
        model = MOVINMeshModel.from_preset(MOVINMAN_PRESET)
        model.fit_to_bones(bvh_rest_bones(data))
        self.assertEqual(set(model.fitted_bones), set(data.bones[1:]))


class ProportionsKeyTests(unittest.TestCase):
    def test_changes_with_bone_lengths_not_with_root_motion(self):
        bones = bvh_rest_bones(read_bvh(V3_ACTOR_BVH))
        names = MOVINMAN_V3_PRESET.body_names
        key = proportions_key(bones, names)

        moved = [dict(b) for b in bones]
        moved[0]["p"] = (1.0, 0.5, -2.0)  # Hips placement changes every frame
        self.assertEqual(proportions_key(moved, names), key)

        longer = [dict(b) for b in bones]
        arm = next(b for b in longer if b["bone_name"] == "LeftForeArm")
        arm["p"] = tuple(c * 1.1 for c in arm["p"])
        self.assertNotEqual(proportions_key(longer, names), key)


@unittest.skipUnless(HAS_RETARGETER, "retargeter dependencies (mujoco, mink) not installed")
class RetargeterBvhSizingTests(unittest.TestCase):
    """--human_height auto in bvh mode: the example measures the actor from the
    BVH's rest bones because the precomputed frames bypass process_mocap_frame."""

    def scale_for(self, bvh_path, preset):
        from movin_sdk_python.retargeter.retargeter import Retargeter

        retargeter = Retargeter(robot_type="unitree_g1", source_preset=preset.name)
        self.assertEqual(retargeter.body_scale, 1.0)  # reference body until measured
        retargeter.process_mocap_frame(bvh_rest_bones(read_bvh(bvh_path)))
        return retargeter.body_scale

    def test_tpose_bvh_is_the_reference_body(self):
        self.assertAlmostEqual(self.scale_for(V3_TPOSE_BVH, MOVINMAN_V3_PRESET), 1.0, places=6)

    def test_actor_bvh_is_measured(self):
        from movin_sdk_python.utils.skeleton_fitting import body_scale_to_reference

        bones = bvh_rest_bones(read_bvh(V3_ACTOR_BVH))
        expected = body_scale_to_reference(bones, MOVINMAN_V3_PRESET)
        scale = self.scale_for(V3_ACTOR_BVH, MOVINMAN_V3_PRESET)
        self.assertAlmostEqual(scale, expected, places=9)
        self.assertNotAlmostEqual(scale, 1.0, places=3)

    def test_legacy_bvh_is_measured(self):
        self.assertNotAlmostEqual(self.scale_for(LEGACY_BVH, MOVINMAN_PRESET), 1.0, places=3)


if __name__ == "__main__":
    unittest.main()
