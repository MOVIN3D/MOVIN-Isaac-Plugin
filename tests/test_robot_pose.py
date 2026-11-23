"""The retargeted robot gets the character's forward-mode transform (no Isaac needed).

``--forward_mode isaac_world`` turns the MOVIN character by +90 degrees about
the world Z axis.  The robot root (``robot_pose.robot_root_pose``) must be
turned the same way, so the robot stays where the character is (overlay) or
beside it (side_by_side) in both forward modes.

Run with:  python -m unittest discover -s tests
"""

import importlib.util
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (ROOT, os.path.join(ROOT, "movin_sdk_python"), os.path.join(ROOT, "examples")):
    if path not in sys.path:
        sys.path.insert(0, path)

from bvh_utils import bvh_rest_bones, load_bvh_file, process_bvh_frame_for_isaaclab, read_bvh  # noqa: E402
from robot_pose import robot_root_pose  # noqa: E402

V3_BVH = os.path.join(ROOT, "data", "test_V3.bvh")
MODES = ("coord_equivalent", "isaac_world")
HAS_RETARGETER = (
    importlib.util.find_spec("mujoco") is not None and importlib.util.find_spec("mink") is not None
)


def yaw_deg(q):
    w, x, y, z = q
    return np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def character_root(data, frame, forward_mode):
    """Root pose of the MOVIN character for a BVH frame, as the example computes it."""
    pos, quat, _ = process_bvh_frame_for_isaaclab(
        data.quats[frame], data.pos[frame], data.bones,
        skeleton_bone_names=(), forward_mode=forward_mode,
    )
    return np.asarray(pos, dtype=np.float64), np.asarray(quat, dtype=np.float64)


class RobotRootPoseTests(unittest.TestCase):
    def setUp(self):
        self.data = read_bvh(V3_BVH)

    def test_coord_equivalent_keeps_the_retargeter_pose(self):
        qpos = np.array([0.3, -0.2, 0.8, 0.9, 0.1, -0.2, 0.37, 0.5, -0.5])
        qpos[3:7] /= np.linalg.norm(qpos[3:7])
        pos, quat = robot_root_pose(qpos, "coord_equivalent")
        np.testing.assert_allclose(pos, qpos[:3])
        np.testing.assert_allclose(quat, qpos[3:7])

    def test_robot_gets_the_characters_transform(self):
        # A robot standing exactly where the coord_equivalent character stands
        # must land exactly where the character lands in either mode.
        for frame in (0, 300, 600):
            char_pos, char_quat = character_root(self.data, frame, "coord_equivalent")
            qpos = np.concatenate([char_pos, char_quat])
            for mode in MODES:
                pos, quat = robot_root_pose(qpos, mode)
                want_pos, want_quat = character_root(self.data, frame, mode)
                np.testing.assert_allclose(pos, want_pos, atol=1e-9, err_msg=mode)
                np.testing.assert_allclose(quat, want_quat, atol=1e-9, err_msg=mode)

    def test_isaac_world_turns_by_90_degrees_about_z(self):
        qpos = np.array([1.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0])
        pos, quat = robot_root_pose(qpos, "isaac_world")
        np.testing.assert_allclose(pos, [0.0, 1.0, 0.8], atol=1e-9)
        self.assertAlmostEqual(yaw_deg(quat), 90.0, places=6)

    def test_side_by_side_offset_is_along_world_x(self):
        qpos = np.array([1.0, 2.0, 0.8, 1.0, 0.0, 0.0, 0.0])
        for mode in MODES:
            base, _ = robot_root_pose(qpos, mode)
            moved, _ = robot_root_pose(qpos, mode, x_offset=1.5)
            np.testing.assert_allclose(moved - base, [1.5, 0.0, 0.0], atol=1e-12, err_msg=mode)


@unittest.skipUnless(HAS_RETARGETER, "retargeter dependencies (mujoco, mink) not installed")
class RetargetedRobotFollowsTheCharacterTests(unittest.TestCase):
    """End to end through the SDK Retargeter, as the example's bvh mode drives it."""

    def test_robot_stays_with_the_character_in_both_modes(self):
        from movin_sdk_python.retargeter.retargeter import Retargeter

        data = read_bvh(V3_BVH)
        frames, _, _, _ = load_bvh_file(V3_BVH)
        retargeter = Retargeter(robot_type="unitree_g1", source_preset="movinman_v3")
        retargeter.process_mocap_frame(bvh_rest_bones(data))  # human_height "auto"
        required = retargeter.get_required_bones()
        for frame in (0, 300, 600):
            targets = {k: v for k, v in frames[frame].items() if k in required}
            for _ in range(5):  # let the IK settle on this frame
                qpos = retargeter.retarget(targets)
            yaw_gap = {}
            for mode in MODES:
                robot_pos, robot_quat = robot_root_pose(qpos, mode)
                char_pos, char_quat = character_root(data, frame, mode)
                self.assertLess(
                    np.linalg.norm(robot_pos[:2] - char_pos[:2]), 0.15,
                    f"frame {frame}, {mode}: robot root strays from the character root",
                )
                yaw_gap[mode] = wrap_deg(yaw_deg(robot_quat) - yaw_deg(char_quat))
            self.assertAlmostEqual(
                wrap_deg(yaw_gap["isaac_world"] - yaw_gap["coord_equivalent"]), 0.0, places=6,
                msg=f"frame {frame}: robot heading relative to the character depends on the mode",
            )


if __name__ == "__main__":
    unittest.main()
