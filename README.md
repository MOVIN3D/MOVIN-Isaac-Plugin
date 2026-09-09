# MOVIN Isaac Plugin

MOVIN motion capture integration for **Isaac Lab** (NVIDIA Isaac Sim).
Receives real-time mocap data from **MOVIN Studio** and drives a humanoid skeleton or MOVINMan character in **Isaac Lab**, with optional robot retargeting to drive a Unitree G1 alongside the human character.

## Setup

```bash
git clone --recurse-submodules <repo-url>
cd MOVIN-Isaac-Plugin
pip install -e .
```

If you already cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

## Project Structure

```
MOVIN-Isaac-Plugin/
  movin_sdk_python/              # git submodule (MOVIN-SDK-Python)
  examples/
    mocap_to_isaaclab.py         # MOVIN -> Isaac Lab visualization
    bvh_utils.py                 # BVH loader + BVH->Isaac/retarget conversion (plugin-owned)
  scripts/
    generate_skeleton_mjcf.py    # Generate a skeleton MJCF from a T-pose BVH
    extract_movinman_mesh.py     # Extract mesh + skin weights from a MOVIN FBX into an NPZ (runs in Blender)
  tests/
    test_skeleton_assets.py      # Checks the V3 MJCF against its T-pose and the SDK preset
    test_mesh_assets.py          # Checks the V3 mesh NPZ against the SDK LBS path and the T-pose
  data/
    movinman_skeleton.xml        # MOVINMan MJCF skeleton (51 bones, legacy preset)
    movinman_v3_skeleton.xml     # MOVINManV3 MJCF skeleton (54 joints, generated)
    movinman_mesh.npz            # Pre-extracted mesh for LBS overlay (legacy preset)
    movinman_v3_mesh.npz         # Pre-extracted mesh for LBS overlay (movinman_v3 preset)
    MOVINMan_dump.xml            # MOVINMan FBX dump
    MOVINman_V3_local_rotation_identity.fbx
                                 # MOVINManV3 rig with identity local rotations in the bind pose (skinned mesh)
    MOVINman_V3_local_rotation_identity_tpose.bvh
                                 # Its rest pose as a BVH (source of the V3 MJCF)
    Locomotion.bvh               # Sample BVH for testing
    test_V3.bvh                  # Sample MOVINManV3 BVH for testing
```

## Live Mocap from MOVIN Studio

Enable OSC output in MOVIN Studio, set the target IP and port, then run:

```bash
# Skeleton only
python examples/mocap_to_isaaclab.py --mode live --port 11235

# Skeleton + mesh overlay
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --view_mode mesh_skeleton

# Isaac world-forward facing (+X forward)
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --forward_mode isaac_world --view_mode mesh_skeleton

# Headless (no GUI)
python examples/mocap_to_isaaclab.py --mode live --port 11235 --headless
```

## Recording & Replay

Record a live session to file, then replay it offline without MOVIN Studio. Replay loops continuously by default.

### Record

Add `--record <path>` to any live mode command:

```bash
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --record data/recordings/session.pkl

# Record with mesh overlay
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --view_mode mesh_skeleton --record data/recordings/session.pkl
```

### Replay

Replay requires a `.pkl` file previously created with `--record` (see above).

```bash
python examples/mocap_to_isaaclab.py --mode replay \
    --recording data/recordings/session.pkl

# Replay with mesh overlay
python examples/mocap_to_isaaclab.py --mode replay \
    --recording data/recordings/session.pkl --view_mode mesh_skeleton
```

## BVH Playback

Play back offline BVH motion capture files:

```bash
# Skeleton only
python examples/mocap_to_isaaclab.py --mode bvh \
    --bvh_file data/Locomotion.bvh

# Skeleton + mesh overlay
python examples/mocap_to_isaaclab.py --mode bvh \
    --bvh_file data/Locomotion.bvh --view_mode mesh_skeleton

# Mesh only (no skeleton)
python examples/mocap_to_isaaclab.py --mode bvh \
    --bvh_file data/Locomotion.bvh --view_mode mesh

# MOVINManV3 skeleton (preset auto-detected from the file)
python examples/mocap_to_isaaclab.py --mode bvh \
    --bvh_file data/test_V3.bvh
```

## Robot Retargeting

Add `--robot` to any mode to retarget MOVIN mocap data to a Unitree G1 robot in real time.

```bash
# Live mocap: MOVIN character + robot side by side
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --robot unitree_g1 --robot_view side_by_side

# Robot only (no MOVIN character)
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --robot unitree_g1 --robot_view robot_only

# BVH playback with robot
python examples/mocap_to_isaaclab.py --mode bvh \
    --bvh_file data/Locomotion.bvh --robot unitree_g1

# Replay with robot (requires a .pkl recorded with --record)
python examples/mocap_to_isaaclab.py --mode replay \
    --recording data/recordings/session.pkl --robot unitree_g1

# Robot + mesh overlay
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --robot unitree_g1 --view_mode mesh_skeleton

# Debug mode (print IK error)
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --robot unitree_g1 --debug
```

### Supported Robots

| Robot | Description | DoFs |
|-------|-------------|------|
| `unitree_g1` | Unitree G1 (standard) | 29 |
| `unitree_g1_with_hands` | Unitree G1 with hands | 43 |

## Skeleton Presets

Two MOVINMan skeleton layouts are supported:

- `movinman` -- legacy skeleton (51 bodies)
- `movinman_v3` -- MOVINManV3 skeleton (54 joints), adding `Spine2`, `Spine3`, and `Neck1` to the spine/neck chain, with a full finger set

By default (`--preset auto`) the preset is detected automatically, per mode:

- **bvh**: detected from the joint names in the BVH file
- **replay**: detected by peeking at the recording's first frame
- **live**: waits for the first mocap frame before starting (pass `--preset movinman` or `--preset movinman_v3` explicitly to skip the wait)

Pass `--preset movinman` or `--preset movinman_v3` to force a specific preset instead of auto-detecting.

Mesh overlay (`--view_mode mesh` / `mesh_skeleton`) is available for both presets: `movinman` uses `data/movinman_mesh.npz` and `movinman_v3` uses `data/movinman_v3_mesh.npz`. Pass `--mesh_npz` to use another asset. A preset without a mesh asset prints a warning and falls back to skeleton-only rendering.

`data/movinman_v3_mesh.npz` is extracted from the skinned mesh of `data/MOVINman_V3_local_rotation_identity.fbx` (44,402 vertices, 4 influences per vertex) with Blender's bundled FBX importer; the script checks that every joint's rest rotation is identity, which is what makes the NPZ's translation-only bind frames valid:

```bash
blender -b --python scripts/extract_movinman_mesh.py -- \
    --fbx data/MOVINman_V3_local_rotation_identity.fbx \
    --out data/movinman_v3_mesh.npz --preset movinman_v3
```

Robot retargeting works with both presets; with `movinman_v3` the G1 torso is automatically mapped to `Spine3` via a V3-specific IK config.

`data/movinman_v3_skeleton.xml` is generated from `data/MOVINman_V3_local_rotation_identity_tpose.bvh`, the rest pose of `data/MOVINman_V3_local_rotation_identity.fbx` (a MOVINManV3 rig whose bind pose has identity local rotations on every joint). Each `OFFSET` equals the FBX `Lcl Translation`, i.e. the joint's translation in its parent's frame -- the same `p` MOVIN Studio streams and the same `OFFSET` found in its BVH exports. Because the MJCF bodies sit in those frames, the streamed and BVH local rotations drive the hinges directly and no rest-pose removal is applied: streams from the earlier MOVINManV3 rig carry non-identity rest quaternions on the finger bones (`Thumb1` about 60 degrees) and removing them would straighten the fingers, while the identity-local rig streams identity rest quaternions, where removal is a no-op. The body order follows the BVH and the live stream (fingers Thumb, Index, Middle, Ring, Pinky within each hand); the FBX itself lists fingers alphabetically, which nothing at runtime depends on. Regenerate the MJCF with:

```bash
python scripts/generate_skeleton_mjcf.py \
    data/MOVINman_V3_local_rotation_identity_tpose.bvh \
    data/movinman_v3_skeleton.xml --model-name movinman_v3
```

`python -m unittest discover -s tests` checks that the committed MJCF is exactly this command's output and that its body order and offsets match the SDK preset and the T-pose.

## Options

| Option | Values | Description |
|--------|--------|-------------|
| `--mode` | `live`, `bvh`, `replay` | Input source |
| `--preset` | `auto`, `movinman`, `movinman_v3` | Skeleton preset (default: `auto`, detected from the data source) |
| `--port` | int (default: 11235) | UDP port for live mocap |
| `--bvh_file` | `<path>` | BVH file for playback mode |
| `--bvh_scale` | float | Position scale factor (auto-detected if not set) |
| `--view_mode` | `skeleton`, `mesh`, `mesh_skeleton` | Visualization mode |
| `--forward_mode` | `coord_equivalent`, `isaac_world` | Facing direction (`isaac_world` = +90 deg yaw for Isaac +X) |
| `--record` | `<path>` | Record live session to `.pkl` file |
| `--recording` | `<path>` | Replay from recorded `.pkl` file |
| `--robot` | `unitree_g1`, `unitree_g1_with_hands` | Enable robot retargeting |
| `--human_height` | float (default: 1.75) | Human height for retargeting scaling |
| `--robot_view` | `side_by_side`, `robot_only`, `overlay` | Robot display mode |
| `--robot_offset` | float (default: 2.0) | X offset for side-by-side view |
| `--mesh_npz` | `<path>` | Mesh NPZ for the overlay (default: the preset's asset in `data/`) |
| `--headless` | | No GUI window |
| `--debug` | | Print FPS and debug info |
| `--max_frames` | int | Exit the main loop after N frames (mainly for headless testing) |

## Viewer Controls

- `Alt + left drag` -- orbit camera
- Mouse wheel -- zoom
- `Enter` -- pause / resume
- `Space` -- single-step (while paused)
- `M` -- toggle mesh visibility (`mesh_skeleton` mode)
- `K` -- toggle skeleton visibility (`mesh_skeleton` mode)

## Dependencies

- [MOVIN-SDK-Python](https://github.com/MOVIN3D/MOVIN-SDK-Python) (included as submodule)
- [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) / Isaac Sim
- NumPy, SciPy, MuJoCo

## License

MIT
