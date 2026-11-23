"""Where mocap_to_isaaclab.py draws the retargeted robot (no Isaac imports).

The Retargeter solves the robot in the same Z-up frame as the character's
``coord_equivalent`` root pose (MOVIN's Y-up stream / BVH converted with
``(x, y, z) -> (x, -z, y)``).  ``--forward_mode isaac_world`` turns the
character by +90 degrees about the world Z axis, so the robot has to be turned
the same way or it ends up rotated and displaced relative to the character
(misplacing both the overlay and the side-by-side views).
"""

import numpy as np

from movin_sdk_python.utils.isaac_lab_utils import _apply_forward_mode_to_root


def robot_root_pose(qpos, forward_mode="coord_equivalent", x_offset=0.0):
    """Isaac world root pose of a retargeted robot.

    Args:
        qpos: Retargeter output; ``qpos[:3]`` root position, ``qpos[3:7]``
            root quaternion (w, x, y, z), both in the Retargeter's Z-up frame.
        forward_mode: The character's forward mode; the robot root gets the
            same transform.
        x_offset: Added to the world X of the result (side-by-side view).

    Returns:
        ``(pos, quat)``: float64 arrays (3,) and (4,), quat (w, x, y, z).
    """
    pos = np.array(qpos[:3], dtype=np.float64)
    quat = np.array(qpos[3:7], dtype=np.float64)
    pos, quat = _apply_forward_mode_to_root(pos, quat, forward_mode)
    pos = np.array(pos, dtype=np.float64)
    pos[0] += x_offset
    return pos, np.array(quat, dtype=np.float64)
