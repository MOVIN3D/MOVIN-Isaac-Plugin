"""Guards ``scripts/fake_movin_studio.py`` against the plugin's BVH path.

The fake stream stands in for MOVIN Studio in live/record/replay testing, so it is
only useful if what arrives through the SDK's ``MocapReceiver`` is the motion in the
BVH. Each test streams a clip over UDP on localhost and checks that every received
frame gives the same Isaac root pose and joint DOFs (``process_movin_bones_for_isaaclab``)
as the BVH conversion of that frame (``process_bvh_frame_for_isaaclab``), and the
same retargeter input positions as ``load_bvh_file``.

Run with:  python -m unittest discover -s tests
"""

import os
import sys
import threading
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for path in (ROOT, os.path.join(ROOT, "movin_sdk_python"), os.path.join(ROOT, "scripts"),
             os.path.join(ROOT, "examples")):
    if path not in sys.path:
        sys.path.insert(0, path)

DATA = os.path.join(ROOT, "data")
TOL = 1e-4  # the stream carries float32


def receive_streamed_frames(bvh_path, seconds=1.5):
    """Stream ``bvh_path`` to a local MocapReceiver; return (stream, frames)."""
    from fake_movin_studio import BvhStream, stream_bvh
    from movin_sdk_python.mocap_receiver.mocap_receiver import MocapReceiver

    receiver = MocapReceiver(host="127.0.0.1", port=0)
    receiver.start()
    port = receiver.sock.getsockname()[1]
    stop = threading.Event()
    sender = threading.Thread(
        target=stream_bvh, args=(bvh_path,),
        kwargs=dict(port=port, stop_event=stop, verbose=False), daemon=True)
    frames = []
    try:
        sender.start()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            frame = receiver.get_latest_frame()
            if frame is not None:
                frames.append(frame)
            time.sleep(0.002)
    finally:
        stop.set()
        sender.join(timeout=2.0)
        receiver.stop()
    return BvhStream(bvh_path), frames


class FakeStudioStreamTests(unittest.TestCase):
    def check_clip(self, bvh_name, expected_preset, root_bone):
        import numpy as np

        from bvh_utils import process_bvh_frame_for_isaaclab
        from movin_sdk_python.utils.isaac_lab_utils import process_movin_bones_for_isaaclab
        from movin_sdk_python.utils.skeleton_presets import detect_preset_from_bone_names

        stream, frames = receive_streamed_frames(os.path.join(DATA, bvh_name))
        self.assertGreater(len(frames), 30, "too few frames arrived over UDP")

        names = [b["bone_name"] for b in frames[0]["bones"]]
        self.assertEqual(names[0], root_bone)
        self.assertEqual(len(names), len(stream.bvh.bones) + 1)
        preset = detect_preset_from_bone_names(names)
        self.assertEqual(preset.name, expected_preset)

        bvh = stream.bvh
        for frame in frames:
            f = frame["frame_idx"] % stream.num_frames
            root_pos, root_quat, dofs = process_movin_bones_for_isaaclab(
                frame["bones"], preset.joint_bones)
            ref_pos, ref_quat, ref_dofs = process_bvh_frame_for_isaaclab(
                bvh.quats[f], bvh.pos[f], bvh.bones, preset.joint_bones)
            np.testing.assert_allclose(root_pos, ref_pos * stream.scale, atol=TOL)
            # q and -q are the same rotation
            self.assertLess(min(np.abs(root_quat - ref_quat).max(),
                                np.abs(root_quat + ref_quat).max()), TOL)
            np.testing.assert_allclose(dofs, ref_dofs, atol=TOL)
        return stream, frames, preset

    def check_retarget_input(self, bvh_name, stream, frames, preset):
        import numpy as np

        from bvh_utils import load_bvh_file
        from movin_sdk_python.retargeter.retargeter import Retargeter

        retargeter = Retargeter(robot_type="unitree_g1", source_preset=preset.name)
        reference = load_bvh_file(os.path.join(DATA, bvh_name))[0]
        for frame in frames:
            live = retargeter.process_mocap_frame(frame["bones"])
            ref = reference[frame["frame_idx"] % stream.num_frames]
            for bone in retargeter.get_required_bones():
                if bone in live and bone in ref:
                    np.testing.assert_allclose(
                        np.asarray(live[bone][0]), np.asarray(ref[bone][0]) * stream.scale,
                        atol=TOL, err_msg=bone)

    def test_movinman_v3_clip_round_trips(self):
        stream, frames, preset = self.check_clip("test_V3.bvh", "movinman_v3", "RootBone")
        self.check_retarget_input("test_V3.bvh", stream, frames, preset)

    def test_legacy_movinman_clip_round_trips(self):
        self.check_clip("Locomotion.bvh", "movinman", "Root")


if __name__ == "__main__":
    unittest.main()
