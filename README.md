# MOVIN Isaac Plugin

Receives real-time mocap data from **MOVIN Studio** and drives a humanoid skeleton or MOVINMan character in **Isaac Lab**, with optional high-fidelity surface mesh overlay via Linear Blend Skinning.

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
    Locomotion.bvh               # Sample BVH for testing
  data/
    movinman_skeleton.xml        # MOVINMan MJCF skeleton (51 bones)
    movinman_mesh.npz            # Pre-extracted mesh for LBS overlay
    MOVINMan_dump.xml            # MOVINMan FBX dump
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
    --bvh_file examples/Locomotion.bvh

# Skeleton + mesh overlay
python examples/mocap_to_isaaclab.py --mode bvh \
    --bvh_file examples/Locomotion.bvh --view_mode mesh_skeleton

# Mesh only (no skeleton)
python examples/mocap_to_isaaclab.py --mode bvh \
    --bvh_file examples/Locomotion.bvh --view_mode mesh
```

## Options

| Option | Values | Description |
|--------|--------|-------------|
| `--mode` | `live`, `bvh`, `replay` | Input source |
| `--port` | int (default: 11235) | UDP port for live mocap |
| `--bvh_file` | `<path>` | BVH file for playback mode |
| `--bvh_scale` | float | Position scale factor (auto-detected if not set) |
| `--view_mode` | `skeleton`, `mesh`, `mesh_skeleton` | Visualization mode |
| `--forward_mode` | `coord_equivalent`, `isaac_world` | Facing direction (`isaac_world` = +90 deg yaw for Isaac +X) |
| `--record` | `<path>` | Record live session to `.pkl` file |
| `--recording` | `<path>` | Replay from recorded `.pkl` file |
| `--mesh_npz` | `<path>` | Path to `movinman_mesh.npz` (auto-detected if not set) |
| `--headless` | | No GUI window |
| `--debug` | | Print FPS and debug info |

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
