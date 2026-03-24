# MOVIN Isaac Plugin

MOVIN motion capture integration for **Isaac Lab** (NVIDIA Isaac Sim).

Receives real-time mocap data from MOVIN Studio (or plays back BVH files) and drives a humanoid skeleton in Isaac Lab, with optional high-fidelity surface mesh overlay via Linear Blend Skinning from the MOVINMan model.

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
    test.bvh
  data/
    MOVINMan_dump.xml            # MOVINMan mesh data
    movinman_skeleton.xml        # MOVINMan skeleton definition (51 bones)
```

The `movin_sdk_python` submodule provides the base SDK:
`MocapReceiver`, `Retargeter`, `OscRecorder`, `ReplayMocapReceiver`,
coordinate conversion utilities, and MOVINMan mesh/LBS support.

## Quick Start

### BVH Playback

```bash
python examples/mocap_to_isaaclab.py --mode bvh --bvh_file examples/Locomotion.bvh
```

### BVH with Mesh Overlay

```bash
python examples/mocap_to_isaaclab.py --mode bvh --bvh_file examples/Locomotion.bvh \
    --view_mode mesh_skeleton
```

### Live Mocap from MOVIN Studio

```bash
python examples/mocap_to_isaaclab.py --mode live --port 11235
```

### Live with Isaac World-Forward Facing

```bash
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --forward_mode isaac_world --view_mode mesh_skeleton
```

### Headless Mode

```bash
python examples/mocap_to_isaaclab.py --mode bvh --bvh_file examples/Locomotion.bvh --headless
```

## Recording & Replay

Record a live session and replay it offline without MOVIN Studio.

### Record While Visualizing

```bash
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --record data/recordings/movin_session.pkl
```

### Replay Offline

```bash
python examples/mocap_to_isaaclab.py --mode replay \
    --recording data/recordings/movin_session.pkl

# Replay with mesh overlay
python examples/mocap_to_isaaclab.py --mode replay \
    --recording data/recordings/movin_session.pkl --view_mode mesh_skeleton
```

## Options

| Option | Values | Description |
|--------|--------|-------------|
| `--mode` | `bvh`, `live`, `replay` | Input source |
| `--forward_mode` | `coord_equivalent`, `isaac_world` | Facing direction (+90 deg yaw for Isaac +X) |
| `--view_mode` | `skeleton`, `mesh`, `mesh_skeleton` | Visualization mode |
| `--headless` | | No GUI window |
| `--record` | `<path>` | Record live session to file |
| `--recording` | `<path>` | Replay from recorded file |

## Isaac Lab Viewer Controls

- `Alt + left drag`: orbit camera
- Mouse wheel: zoom
- `Enter`: pause / resume
- `Space`: single-step
- `M`: toggle mesh visibility (`mesh_skeleton` mode)
- `K`: toggle skeleton visibility (`mesh_skeleton` mode)

## Dependencies

- [MOVIN-SDK-Python](https://github.com/MOVIN3D/MOVIN-SDK-Python) (included as submodule)
- [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) / Isaac Sim
- NumPy, SciPy, MuJoCo

## License

MIT
