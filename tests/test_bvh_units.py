"""A centimeter BVH plays exactly like the same clip in meters (no Isaac needed).

The samples in data/ are in meters, so the tests write a centimeter copy of
one (every OFFSET and position channel x100) to a temporary directory and run
both through the same steps as the example's bvh mode: unit detection, the
character root and joints, the fitted skeleton, and the robot retargeting
(both ``--human_height auto`` and a number).

Run with:  python -m unittest discover -s tests
"""

import importlib.util
import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (ROOT, os.path.join(ROOT, "movin_sdk_python"), os.path.join(ROOT, "examples")):
    if path not in sys.path:
        sys.path.insert(0, path)

from bvh_utils import (  # noqa: E402
    bvh_rest_bones,
    detect_bvh_scale,
    load_bvh_file,
    process_bvh_frame_for_isaaclab,
    read_bvh,
)

DATA = os.path.join(ROOT, "data")
V3_BVH = os.path.join(DATA, "test_V3.bvh")
FRAMES = (0, 300, 600)
HAS_RETARGETER = (
    importlib.util.find_spec("mujoco") is not None and importlib.util.find_spec("mink") is not None
)


def scale_bvh_text(text, factor):
    """The BVH with every OFFSET and position channel multiplied by ``factor``."""
    out, channels, in_motion, header_left = [], [], False, 0
    for line in text.splitlines(keepends=True):
        stripped = line.strip()
        if not in_motion:
            if stripped.startswith("OFFSET"):
                values = " ".join(repr(float(v) * factor) for v in stripped.split()[1:4])
                line = line[: len(line) - len(line.lstrip())] + f"OFFSET {values}\n"
            elif stripped.startswith("CHANNELS"):
                channels.extend(stripped.split()[2:])
            elif stripped.startswith("MOTION"):
                in_motion, header_left = True, 2  # "Frames:" and "Frame Time:"
            out.append(line)
            continue
        if not stripped:
            out.append(line)
            continue
        if header_left:
            header_left -= 1
            out.append(line)
            continue
        values = stripped.split()
        assert len(values) == len(channels)
        values = [repr(float(v) * factor) if ch.endswith("position") else v
                  for v, ch in zip(values, channels)]
        out.append(" ".join(values) + "\n")
    return "".join(out)


class CentimeterBvhTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.cm_path = os.path.join(cls._tmp.name, "test_V3_cm.bvh")
        with open(V3_BVH) as src, open(cls.cm_path, "w") as dst:
            dst.write(scale_bvh_text(src.read(), 100.0))
        cls.meters, cls.centimeters = read_bvh(V3_BVH), read_bvh(cls.cm_path)
        cls.scale = detect_bvh_scale(cls.centimeters)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def pair(self):
        return self.meters, self.centimeters, self.scale

    def test_units_are_detected(self):
        meters, centimeters, scale = self.pair()
        self.assertEqual(detect_bvh_scale(meters), 1.0)
        self.assertEqual(scale, 0.01)
        np.testing.assert_allclose(centimeters.offsets * scale, meters.offsets, atol=1e-9)
        np.testing.assert_allclose(centimeters.pos[:, 0] * scale, meters.pos[:, 0], atol=1e-9)

    def test_character_root_and_joints_match(self):
        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        joints = MOVINMAN_V3_PRESET.joint_bones
        meters, centimeters, scale = self.pair()
        for frame in FRAMES:
            want = process_bvh_frame_for_isaaclab(
                meters.quats[frame], meters.pos[frame], meters.bones, skeleton_bone_names=joints
            )
            got = process_bvh_frame_for_isaaclab(
                centimeters.quats[frame], centimeters.pos[frame], centimeters.bones,
                skeleton_bone_names=joints,
            )
            np.testing.assert_allclose(got[0] * scale, want[0], atol=1e-9)  # root position
            np.testing.assert_allclose(got[1], want[1], atol=1e-9)          # root rotation
            np.testing.assert_allclose(got[2], want[2], atol=1e-9)          # joints

    def test_rest_bones_and_fitted_skeleton_match(self):
        from movin_sdk_python.utils.skeleton_fitting import fit_skeleton_mjcf
        from movin_sdk_python.utils.skeleton_presets import MOVINMAN_V3_PRESET

        meters, centimeters, scale = self.pair()
        want, got = bvh_rest_bones(meters), bvh_rest_bones(centimeters, scale)
        for a, b in zip(got, want):
            np.testing.assert_allclose(a["p"], b["p"], atol=1e-9, err_msg=b["bone_name"])
        self.assertEqual(
            fit_skeleton_mjcf(got, MOVINMAN_V3_PRESET), fit_skeleton_mjcf(want, MOVINMAN_V3_PRESET)
        )

    def test_retarget_frames_are_in_meters(self):
        want = load_bvh_file(V3_BVH)[0]
        got = load_bvh_file(self.cm_path, scale=self.scale)[0]
        for frame in FRAMES:
            for bone, (pos, rot) in want[frame].items():
                np.testing.assert_allclose(got[frame][bone][0], pos, atol=1e-9, err_msg=bone)
                np.testing.assert_allclose(got[frame][bone][1], rot, atol=1e-9, err_msg=bone)

    @unittest.skipUnless(HAS_RETARGETER, "retargeter dependencies (mujoco, mink) not installed")
    def test_robot_retargets_the_same(self):
        from movin_sdk_python.retargeter.retargeter import Retargeter

        def retarget(path, human_height):
            # The example's bvh-mode robot setup.
            data = read_bvh(path)
            scale = detect_bvh_scale(data)
            frames = load_bvh_file(path, human_height=human_height, scale=scale)[0]
            retargeter = Retargeter(
                robot_type="unitree_g1", human_height=human_height, source_preset="movinman_v3"
            )
            if human_height == "auto":
                retargeter.process_mocap_frame(bvh_rest_bones(data, scale))
            required = retargeter.get_required_bones()
            out = []
            for frame in FRAMES:
                targets = {k: v for k, v in frames[frame].items() if k in required}
                out.append(np.array(retargeter.retarget(targets)))
            return np.array(out)

        for human_height in ("auto", 1.7):
            want = retarget(V3_BVH, human_height)
            got = retarget(self.cm_path, human_height)
            np.testing.assert_allclose(got, want, atol=1e-6, err_msg=f"human_height={human_height}")


if __name__ == "__main__":
    unittest.main()
