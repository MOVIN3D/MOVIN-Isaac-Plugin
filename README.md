# MOVIN Isaac Plugin

MOVIN motion capture integration for **Isaac Lab** (NVIDIA Isaac Sim).

Receives real-time mocap data from MOVIN Studio and drives a humanoid
skeleton in Isaac Lab, with optional high-fidelity mesh overlay via
Linear Blend Skinning.

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
  movin_sdk_python/          # git submodule (MOVIN-SDK-Python)
  examples/
    mocap_to_isaaclab.py     # MOVIN -> Isaac Lab visualization
    Locomotion.bvh           # Sample BVH for testing
  data/
    MOVINMan_dump.xml        # MOVINMan mesh data
    movinman_skeleton.xml    # MOVINMan skeleton definition
```

## Quick Start

### BVH Playback

```bash
python examples/mocap_to_isaaclab.py --mode bvh --bvh_file examples/Locomotion.bvh
```

### Live Mocap from MOVIN Studio

```bash
python examples/mocap_to_isaaclab.py --mode live --port 11235
```

### Mesh Overlay

```bash
python examples/mocap_to_isaaclab.py --mode bvh --bvh_file examples/Locomotion.bvh --view_mode mesh_skeleton
```

## Dependencies

- [MOVIN-SDK-Python](https://github.com/MOVIN3D/MOVIN-SDK-Python) (included as submodule)
- Isaac Lab / Isaac Sim
- NumPy, SciPy, MuJoCo

## License

MIT
