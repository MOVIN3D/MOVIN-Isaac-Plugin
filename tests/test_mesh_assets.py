"""Guards the V3 mesh NPZ against the SDK's LBS path and the T-pose.

The NPZ is extracted from ``data/MOVINman_V3_local_rotation_identity.fbx`` with
``scripts/extract_movinman_mesh.py``.  The copy checked is the one the example
loads at runtime, packaged with MOVIN-SDK-Python
(``MOVINMAN_V3_PRESET.mesh_npz_path``).  The extraction runs inside Blender, so
re-running it and comparing with the packaged copy is optional: it happens when
a ``blender`` executable is found (``$BLENDER`` or on ``PATH``) and is skipped
otherwise.

Run with:  python -m unittest discover -s tests
"""

import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (ROOT, os.path.join(ROOT, "movin_sdk_python"), os.path.join(ROOT, "scripts")):
    if path not in sys.path:
        sys.path.insert(0, path)

DATA = os.path.join(ROOT, "data")
V3_TPOSE = os.path.join(DATA, "MOVINman_V3_local_rotation_identity_tpose.bvh")
V3_FBX = os.path.join(DATA, "MOVINman_V3_local_rotation_identity.fbx")
EXTRACTOR = os.path.join(ROOT, "scripts", "extract_movinman_mesh.py")
BLENDER = os.environ.get("BLENDER") or shutil.which("blender")


class V3MeshAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import numpy as np

        from movin_sdk_python.utils.movinman_mesh_utils import MOVINMeshModel
        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        cls.preset = MOVINMAN_V3_PRESET
        cls.npz_path = cls.preset.mesh_npz_path
        cls.npz = np.load(cls.npz_path)
        cls.model = MOVINMeshModel.from_preset(cls.preset)

    def test_preset_points_at_an_existing_asset(self):
        self.assertEqual(self.preset.mesh_npz_filename, "movinman_v3_mesh.npz")
        self.assertEqual(os.path.basename(self.npz_path), "movinman_v3_mesh.npz")
        self.assertTrue(os.path.exists(self.npz_path))

    def test_mesh_is_triangulated_and_indexed_in_range(self):
        faces = self.npz["faces"]
        self.assertEqual(faces.shape[1], 3)
        self.assertEqual(faces.min(), 0)
        self.assertEqual(faces.max(), self.model.num_vertices - 1)
        self.assertGreater(self.model.num_vertices, 0)

    def test_weights_are_normalized_top4(self):
        import numpy as np

        w = self.npz["weight_values"]
        idx = self.npz["weight_bone_indices"]
        self.assertEqual(w.shape[1], 4)
        np.testing.assert_allclose(w.sum(axis=1), 1.0, atol=1e-5)
        self.assertGreaterEqual(idx.min(), 0)
        self.assertLess(idx.max(), len(self.preset.body_names))

    def test_bone_offsets_and_parents_match_the_tpose(self):
        import numpy as np

        import generate_skeleton_mjcf as gen

        root = gen.parse_bvh_hierarchy(V3_TPOSE)
        offsets = {n.name: n.offset for n in gen.dfs_bodies(root)}
        parents = {}

        def walk(node):
            for child in node.children:
                if not child.is_end_site:
                    parents[child.name] = node.name
                    walk(child)

        walk(root)
        names = list(self.npz["bone_names"])
        self.assertEqual(tuple(names), self.preset.body_names)
        for i, name in enumerate(names):
            if i == 0:
                self.assertEqual(self.npz["bone_parents"][0], -1)
                continue
            self.assertEqual(names[self.npz["bone_parents"][i]], parents[name])
            np.testing.assert_allclose(self.npz["bone_offsets"][i], offsets[name], atol=1e-5)

    def test_bind_pose_round_trips_through_lbs(self):
        import numpy as np

        names = self.npz["bone_names"]
        identity = np.tile([1.0, 0.0, 0.0, 0.0], (len(names), 1))
        rest_hips_yup = self.npz["bone_offsets"][0]
        posed = self.model.pose_vertices(identity, rest_hips_yup)
        bind_zup_m = self.npz["vertices"].astype(np.float64) / 100.0
        np.testing.assert_allclose(posed, bind_zup_m, atol=1e-4)
        # Character stands on the ground, ~1.8 m tall, arms spanning X (T-pose).
        self.assertLess(abs(bind_zup_m[:, 2].min()), 0.05)
        self.assertGreater(bind_zup_m[:, 2].max(), 1.6)
        self.assertGreater(bind_zup_m[:, 0].max() - bind_zup_m[:, 0].min(), 1.5)



@unittest.skipUnless(BLENDER, "blender not found (set $BLENDER or put it on PATH)")
class V3MeshExtractionTests(unittest.TestCase):
    def test_packaged_mesh_is_the_extractor_output(self):
        import numpy as np

        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "movinman_v3_mesh.npz")
            result = subprocess.run(
                [BLENDER, "-b", "--python", EXTRACTOR, "--",
                 "--fbx", V3_FBX, "--out", out, "--preset", "movinman_v3"],
                capture_output=True, text=True, timeout=600,
            )
            self.assertEqual(result.returncode, 0, result.stdout[-2000:] + result.stderr[-2000:])
            extracted = np.load(out)
            packaged = np.load(MOVINMAN_V3_PRESET.mesh_npz_path)
            self.assertEqual(sorted(extracted.files), sorted(packaged.files))
            for key in packaged.files:
                self.assertEqual(extracted[key].dtype, packaged[key].dtype, key)
                np.testing.assert_array_equal(
                    extracted[key], packaged[key],
                    err_msg=f"{key}: the SDK's movinman_v3_mesh.npz is not the extractor output",
                )


if __name__ == "__main__":
    unittest.main()
