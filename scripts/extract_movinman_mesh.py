"""
Extract a MOVIN character mesh + skin weights from an FBX into the NPZ consumed by
``movin_sdk_python.utils.movinman_mesh_utils.MOVINMeshModel``.

Runs inside Blender's Python (Blender 4.x / 5.x, bundled FBX importer):

    blender -b --python scripts/extract_movinman_mesh.py -- \
        --fbx data/MOVINman_V3_local_rotation_identity.fbx \
        --out data/movinman_v3_mesh.npz --preset movinman_v3

NPZ layout (identical to ``data/movinman_mesh.npz``):
    vertices               (V, 3) float32  bind-pose vertices, geometry space (cm, Z-up)
    faces                  (F, 3) int32    triangles
    bone_names             (B,)   str      skeleton preset order (root "Hips" first)
    inverse_bind_matrices  (B, 4, 4) f32   geometry space -> bone-local (m, Y-up)
    bone_parents           (B,)   int32    parent index per bone (-1 for the root)
    bone_offsets           (B, 3) float32  bone offset in the parent's frame (m, Y-up);
                                           the root entry holds the rest hips position
    weight_bone_indices    (V, 4) int32    up to 4 influences per vertex (preset index)
    weight_values          (V, 4) float32  normalized to sum 1

Conventions
    The FBX frame is right-handed Y-up meters; that is the SDK's skeleton space
    (the frame streamed ``p``/``q`` and the T-pose BVH OFFSETs are expressed in).
    Geometry space is that frame converted to Z-up and scaled to cm:
    ``(x, y, z)_yup -> (x, -z, y) * 100``.

    The rig's bind pose must have identity local rotations on every joint (true
    for MOVINman_V3_local_rotation_identity.fbx; the script verifies it), so the
    bind transform of every bone is a pure translation to its rest position and
    ``ibm_j = T(-rest_j) @ S(0.01) @ R(Z-up -> Y-up)``.  At runtime the SDK runs
    FK with the streamed local rotations on ``bone_offsets`` and skins with
    ``world_j @ ibm_j`` (see MOVINMeshModel.pose_vertices).
"""

import argparse
import math
import os
import sys

import numpy as np

import bpy
from mathutils import Euler, Matrix

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "movin_sdk_python"))
from movin_sdk_python.utils.skeleton_presets import get_preset  # noqa: E402  (dependency-free)

MAX_INFLUENCES = 4
# Geometry space (cm, Z-up) -> skeleton space (m, Y-up) as a 4x4: x, y_yup = z, z_yup = -y.
GEOM_TO_SKEL = np.array(
    [[0.01, 0.0, 0.0, 0.0],
     [0.0, 0.0, 0.01, 0.0],
     [0.0, -0.01, 0.0, 0.0],
     [0.0, 0.0, 0.0, 1.0]], dtype=np.float64,
)


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--fbx", required=True, help="Skinned character FBX")
    p.add_argument("--out", required=True, help="Output .npz path")
    p.add_argument("--preset", default="movinman_v3",
                   help="Skeleton preset whose body order the NPZ follows (default: movinman_v3)")
    p.add_argument("--rest-rotation-tolerance-deg", type=float, default=0.5,
                   help="Max allowed local rest rotation of any bone (default: 0.5)")
    return p.parse_args(argv)


def yup_to_zup_cm(v):
    """(x, y, z) Y-up meters -> (x, -z, y) Z-up centimeters."""
    return np.array([v[0], -v[2], v[1]], dtype=np.float64) * 100.0


FBX_ROTATION_ORDERS = ("XYZ", "XZY", "YZX", "YXZ", "ZXY", "ZYX")


def fbx_rest_rotation_angles_deg(fbx_path):
    """Local rest rotation angle (deg) of every LimbNode, read from the raw FBX.

    Blender's bone matrices carry Blender's own Y-along-bone orientation, so the
    FBX node rotations (PreRotation * Lcl Rotation * PostRotation^-1) are read
    with the importer's parser instead.  All of them must be ~0 for the NPZ's
    translation-only bind frames to be valid.
    """
    from io_scene_fbx.parse_fbx import parse

    root, _ = parse(fbx_path, use_namedtuple=True)
    objects = next(e for e in root.elems if e.id == b"Objects")
    angles = {}
    for e in objects.elems:
        if e.id != b"Model" or e.props[2] != b"LimbNode":
            continue
        name = e.props[1].split(b"\x00")[0].decode()
        props = {}
        for p70 in e.elems:
            if p70.id != b"Properties70":
                continue
            for p in p70.elems:
                if p.id == b"P":
                    props[p.props[0].decode()] = list(p.props[4:])
        order = FBX_ROTATION_ORDERS[int(props.get("RotationOrder", [0])[0])]

        def rot(key, euler_order="XYZ"):
            v = props.get(key)
            if v is None:
                return Matrix.Identity(3)
            return Euler([math.radians(float(a)) for a in v[:3]], euler_order).to_matrix()

        rest = rot("PreRotation") @ rot("Lcl Rotation", order) @ rot("PostRotation").inverted()
        angles[name] = math.degrees(rest.to_quaternion().angle)
    return angles


def main():
    args = parse_args()
    preset = get_preset(args.preset)
    names = list(preset.body_names)
    index_of = {n: i for i, n in enumerate(names)}

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=os.path.abspath(args.fbx),
                             automatic_bone_orientation=False,
                             ignore_leaf_bones=False,
                             use_custom_normals=False)

    armatures = [o for o in bpy.data.objects if o.type == "ARMATURE"]
    meshes = [o for o in bpy.data.objects if o.type == "MESH" and o.vertex_groups]
    if len(armatures) != 1 or not meshes:
        raise SystemExit(f"expected 1 armature and >=1 skinned mesh, got {len(armatures)} / {len(meshes)}")
    arm = armatures[0]
    mesh_obj = max(meshes, key=lambda o: len(o.data.vertices))
    mesh = mesh_obj.data
    print(f"[info] armature '{arm.name}' ({len(arm.data.bones)} bones), mesh '{mesh_obj.name}' "
          f"({len(mesh.vertices)} verts, {len(mesh.polygons)} polys, {len(mesh_obj.vertex_groups)} vertex groups)")

    # ---- bones: names, parents, rest positions (armature space == FBX frame, Y-up meters)
    bones = {b.name: b for b in arm.data.bones}
    missing = [n for n in names if n not in bones]
    extra = [n for n in bones if n not in index_of]
    if missing or extra:
        raise SystemExit(f"bone set does not match preset '{preset.name}': missing={missing} extra={extra}")
    rest_pos = np.array([bones[n].head_local[:] for n in names], dtype=np.float64)
    parents = np.array([index_of[bones[n].parent.name] if bones[n].parent else -1 for n in names], dtype=np.int32)
    offsets = rest_pos.copy()
    for i, n in enumerate(names):
        if parents[i] >= 0:
            offsets[i] = rest_pos[i] - rest_pos[parents[i]]

    # The NPZ's bind frames are pure translations, which is only right when every
    # joint's rest local rotation is identity.  Check that on the raw FBX nodes.
    angles = fbx_rest_rotation_angles_deg(os.path.abspath(args.fbx))
    offenders = {n: round(a, 2) for n, a in angles.items() if a > args.rest_rotation_tolerance_deg}
    if offenders:
        raise SystemExit(
            f"{len(offenders)} joint(s) have a non-identity rest rotation in the FBX "
            f"(deg): {offenders}. This exporter needs a local-rotation-identity rig."
        )
    print(f"[info] rest local rotations: {len(angles)} FBX joints checked, "
          f"worst {max(angles.values()) if angles else 0.0:.3f} deg (tolerance {args.rest_rotation_tolerance_deg})")
    print(f"[info] armature object matrix (FBX->Blender axis conversion): "
          f"rot={[round(a, 3) for a in arm.matrix_world.to_euler()]} scale={[round(s, 4) for s in arm.matrix_world.to_scale()]}")

    # ---- vertices: object space -> world -> armature space (FBX frame) -> geometry space
    to_arm = arm.matrix_world.inverted() @ mesh_obj.matrix_world
    verts_yup = np.array([(to_arm @ v.co)[:] for v in mesh.vertices], dtype=np.float64)
    verts_geom = np.stack([yup_to_zup_cm(v) for v in verts_yup]).astype(np.float32)
    lo, hi = verts_yup.min(axis=0), verts_yup.max(axis=0)
    print(f"[info] mesh bounds in skeleton space (m, Y-up): min={np.round(lo, 3)} max={np.round(hi, 3)}")

    # ---- faces (triangulated)
    mesh.calc_loop_triangles()
    faces = np.array([[t.vertices[0], t.vertices[1], t.vertices[2]] for t in mesh.loop_triangles], dtype=np.int32)

    # ---- weights: top-4 influences per vertex, preset bone index, normalized
    group_to_bone = {}
    for g in mesh_obj.vertex_groups:
        if g.name in index_of:
            group_to_bone[g.index] = index_of[g.name]
        else:
            print(f"[warn] vertex group '{g.name}' is not a preset bone; ignored")
    w_idx = np.zeros((len(mesh.vertices), MAX_INFLUENCES), dtype=np.int32)
    w_val = np.zeros((len(mesh.vertices), MAX_INFLUENCES), dtype=np.float32)
    truncated = 0
    unweighted = 0
    for v in mesh.vertices:
        infl = [(group_to_bone[g.group], g.weight) for g in v.groups if g.group in group_to_bone and g.weight > 0.0]
        if not infl:
            unweighted += 1
            w_idx[v.index, 0] = 0
            w_val[v.index, 0] = 1.0
            continue
        infl.sort(key=lambda t: -t[1])
        if len(infl) > MAX_INFLUENCES:
            truncated += 1
            infl = infl[:MAX_INFLUENCES]
        total = sum(w for _, w in infl)
        for k, (b, w) in enumerate(infl):
            w_idx[v.index, k] = b
            w_val[v.index, k] = w / total
    print(f"[info] weights: {truncated} vertices truncated to {MAX_INFLUENCES} influences, {unweighted} unweighted (bound to root)")

    # ---- inverse bind matrices: geometry -> bone-local, translation-only bind frames
    ibm = np.zeros((len(names), 4, 4), dtype=np.float64)
    for i in range(len(names)):
        t = np.eye(4)
        t[:3, 3] = -rest_pos[i]
        ibm[i] = t @ GEOM_TO_SKEL

    out = os.path.abspath(args.out)
    np.savez_compressed(
        out,
        vertices=verts_geom,
        faces=faces,
        bone_names=np.array(names),
        inverse_bind_matrices=ibm.astype(np.float32),
        bone_parents=parents,
        bone_offsets=offsets.astype(np.float32),
        weight_bone_indices=w_idx,
        weight_values=w_val,
        source_fbx=np.array(os.path.basename(args.fbx)),
        preset=np.array(preset.name),
    )
    print(f"[info] wrote {out}: {len(verts_geom)} verts, {len(faces)} tris, {len(names)} bones "
          f"({os.path.getsize(out) / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
