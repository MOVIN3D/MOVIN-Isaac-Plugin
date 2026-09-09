"""Guards ``data/movinman_v3_mesh.npz`` against the SDK's LBS path and the T-pose.

The NPZ is extracted from ``data/MOVINman_V3_local_rotation_identity.fbx`` with
``scripts/extract_movinman_mesh.py`` (runs in Blender, so the extraction itself is
not repeated here).

Run with:  python -m unittest discover -s tests
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (ROOT, os.path.join(ROOT, "movin_sdk_python"), os.path.join(ROOT, "scripts")):
    if path not in sys.path:
        sys.path.insert(0, path)

DATA = os.path.join(ROOT, "data")
V3_TPOSE = os.path.join(DATA, "MOVINman_V3_local_rotation_identity_tpose.bvh")


class V3MeshAssetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import numpy as np

        from movin_sdk_python.utils.movinman_mesh_utils import MOVINMeshModel
        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        cls.preset = MOVINMAN_V3_PRESET
        cls.npz_path = os.path.join(DATA, cls.preset.mesh_npz_filename or "")
        cls.npz = np.load(cls.npz_path)
        cls.model = MOVINMeshModel(cls.npz_path, expected_bone_names=cls.preset.body_names)

    def test_preset_points_at_an_existing_asset(self):
        self.assertEqual(self.preset.mesh_npz_filename, "movinman_v3_mesh.npz")
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


if __name__ == "__main__":
    unittest.main()
