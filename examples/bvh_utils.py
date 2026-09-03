"""BVH utilities owned by MOVIN-Isaac-Plugin.

MOVIN-SDK-Python a8c012a ("refactor: center SDK on mocap receive and
recording") removed every BVH code path from the SDK (``utils/bvh_loader.py``,
``process_bvh_frame_for_isaaclab``, ``extract_bvh_local_quats_yup``,
``Retargeter.load_bvh`` and the euler->quat helpers in ``quat_utils``).
The plugin's ``--mode bvh`` still needs them, so they live here now.  The code
is the SDK's former implementation, unchanged apart from imports.
"""

import re

import numpy as np
from scipy.spatial.transform import Rotation as R

from movin_sdk_python.utils.quat_utils import (
    quat_fk,
    quat_mul_batch,
    quat_mul_vec_batch,
)
from movin_sdk_python.utils.isaac_lab_utils import (
    SKELETON_BODY_NAMES,
    SKELETON_JOINT_BONES,
    _apply_forward_mode_to_root,
    quat_to_exp_map,
    yup_to_zup_quat,
    yup_to_zup_vec,
)


# ---------------------------------------------------------------------------
# Euler / quaternion helpers (formerly movin_sdk_python.utils.quat_utils)
# ---------------------------------------------------------------------------

def angle_axis_to_quat(angle, axis):
    """Angle-axis -> quaternion (w, x, y, z)."""
    c = np.cos(angle / 2.0)[..., np.newaxis]
    s = np.sin(angle / 2.0)[..., np.newaxis]
    return np.concatenate([c, s * axis], axis=-1)


def euler_to_quat(e, order='zyx'):
    """Euler angles (radians) -> quaternion (w, x, y, z) for the given order."""
    axis = {
        'x': np.asarray([1, 0, 0], dtype=np.float32),
        'y': np.asarray([0, 1, 0], dtype=np.float32),
        'z': np.asarray([0, 0, 1], dtype=np.float32)}
    q0 = angle_axis_to_quat(e[..., 0], axis[order[0]])
    q1 = angle_axis_to_quat(e[..., 1], axis[order[1]])
    q2 = angle_axis_to_quat(e[..., 2], axis[order[2]])
    return quat_mul_batch(q0, quat_mul_batch(q1, q2))


def remove_quat_discontinuities(rotations):
    """Remove sign flips along the time axis of a (T, J, 4) quaternion array."""
    rots_inv = -rotations
    for i in range(1, rotations.shape[0]):
        replace_mask = np.sum(rotations[i - 1: i] * rotations[i: i + 1], axis=-1) < np.sum(
            rotations[i - 1: i] * rots_inv[i: i + 1], axis=-1)
        replace_mask = replace_mask[..., np.newaxis]
        rotations[i] = replace_mask * rots_inv[i] + (1.0 - replace_mask) * rotations[i]
    return rotations


# ---------------------------------------------------------------------------
# BVH reader (formerly movin_sdk_python.utils.bvh_loader)
# ---------------------------------------------------------------------------

class BVHAnimation:
    """Local quats (T, J, 4) wxyz, local pos (T, J, 3), offsets, parents, bones."""

    def __init__(self, quats, pos, offsets, parents, bones):
        self.quats = quats
        self.pos = pos
        self.offsets = offsets
        self.parents = parents
        self.bones = bones


def read_bvh(filename, start=None, end=None, order=None):
    """Read a BVH file.  The euler order is auto-detected per file from the
    first CHANNELS line unless ``order`` is given (MOVIN Studio exports differ:
    T-pose reference is ZYX, actor exports are YXZ)."""
    channelmap = {'Xrotation': 'x', 'Yrotation': 'y', 'Zrotation': 'z'}

    f = open(filename, "r")
    i = 0
    active = -1
    end_site = False
    names = []
    orients = np.array([]).reshape((0, 4))
    offsets = np.array([]).reshape((0, 3))
    parents = np.array([], dtype=int)

    for line in f:
        if "HIERARCHY" in line:
            continue
        if "MOTION" in line:
            continue
        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue
        if "{" in line:
            continue
        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue
        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if not end_site:
                offsets[active] = np.array([list(map(float, offmatch.groups()))])
            continue
        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                channelis = 0 if channels == 3 else 3
                channelie = 3 if channels == 3 else 6
                parts = line.split()[2 + channelis:2 + channelie]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue
        jmatch = re.match(r"\s*JOINT\s+(\w+)", line)
        if jmatch:
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue
        if "End Site" in line:
            end_site = True
            continue
        fmatch = re.match(r"\s*Frames:\s+(\d+)", line)
        if fmatch:
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
            positions = offsets[np.newaxis].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue
        fmatch = re.match(r"\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue
        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue
        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            if channels == 3:
                positions[fi, 0:1] = data_block[0:3]
                rotations[fi, :] = data_block[3:].reshape(N, 3)
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                positions[fi, :] = data_block[:, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            elif channels == 9:
                positions[fi, 0] = data_block[0:3]
                data_block = data_block[3:].reshape(N - 1, 9)
                rotations[fi, 1:] = data_block[:, 3:6]
                positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            else:
                raise Exception("Too many channels! %i" % channels)
            i += 1
    f.close()

    rotations = euler_to_quat(np.radians(rotations), order=order)
    rotations = remove_quat_discontinuities(rotations)
    return BVHAnimation(rotations, positions, offsets, parents, names)


def load_bvh_file(bvh_file, human_height=1.75):
    """Load a BVH into per-frame {bone: [global_pos_zup, global_quat_zup]}
    dicts as consumed by ``Retargeter.retarget`` (formerly
    ``Retargeter.load_bvh`` / ``utils.bvh_loader.load_bvh_file``).

    Returns (frames, human_height, parents, bones)."""
    data = read_bvh(bvh_file)
    global_data = quat_fk(data.quats, data.pos, data.parents)

    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)

    frames = []
    for frame in range(data.pos.shape[0]):
        result = {}
        for i, bone in enumerate(data.bones):
            orientation = quat_mul_batch(rotation_quat, global_data[0][frame, i])
            position = global_data[1][frame, i] @ rotation_matrix.T
            result[bone] = [position, orientation]
        if "LeftFoot" in result and "LeftToeBase" in result:
            result["LeftFootMod"] = [result["LeftFoot"][0].copy(), result["LeftToeBase"][1].copy()]
        if "RightFoot" in result and "RightToeBase" in result:
            result["RightFootMod"] = [result["RightFoot"][0].copy(), result["RightToeBase"][1].copy()]
        frames.append(result)
    return frames, human_height, data.parents, data.bones


# ---------------------------------------------------------------------------
# BVH frame -> Isaac Lab (formerly utils.isaac_lab_utils / movinman_mesh_utils)
# ---------------------------------------------------------------------------

def process_bvh_frame_for_isaaclab(
    quats,
    positions,
    bone_names,
    skeleton_bone_names=None,
    forward_mode="coord_equivalent",
):
    """Convert one BVH frame (Y-up local quats) to (root_pos_zup, root_quat_zup, dof_array)."""
    if skeleton_bone_names is None:
        skeleton_bone_names = SKELETON_JOINT_BONES

    bvh_name_to_idx = {name: i for i, name in enumerate(bone_names)}

    root_pos_zup = yup_to_zup_vec(positions[0])
    root_quat_zup = yup_to_zup_quat(quats[0])
    root_pos_zup, root_quat_zup = _apply_forward_mode_to_root(
        root_pos_zup, root_quat_zup, forward_mode
    )

    num_joints = len(skeleton_bone_names)
    dof_array = np.zeros(num_joints * 3)
    for i, bone_name in enumerate(skeleton_bone_names):
        bvh_idx = bvh_name_to_idx.get(bone_name)
        if bvh_idx is None:
            continue
        q_zup = yup_to_zup_quat(quats[bvh_idx])
        dof_array[i * 3: i * 3 + 3] = quat_to_exp_map(q_zup)

    return root_pos_zup, root_quat_zup, dof_array


def extract_bvh_local_quats_yup(quats, positions, bone_names, skeleton_body_names=None):
    """(B, 4) local wxyz quats + (3,) root pos (Y-up, unscaled) for the mesh LBS path."""
    if skeleton_body_names is None:
        skeleton_body_names = SKELETON_BODY_NAMES

    bvh_name_to_idx = {name: i for i, name in enumerate(bone_names)}
    local_quats = np.zeros((len(skeleton_body_names), 4), dtype=np.float64)
    local_quats[:, 0] = 1.0
    root_pos_yup = np.array(positions[0], dtype=np.float64)
    local_quats[0] = quats[0]
    for i, name in enumerate(skeleton_body_names[1:], start=1):
        bvh_idx = bvh_name_to_idx.get(name)
        if bvh_idx is None:
            continue
        local_quats[i] = quats[bvh_idx]
    return local_quats, root_pos_yup
