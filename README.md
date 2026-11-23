# MOVIN Isaac Plugin

MOVIN motion capture integration for **Isaac Lab** (NVIDIA Isaac Sim).
Receives real-time mocap data from **MOVIN Studio** and drives a humanoid skeleton or MOVINMan character in **Isaac Lab**, with optional robot retargeting to drive a Unitree G1 alongside the human character.

## Setup

### Requirements

Tested with:

| Component | Version |
|-----------|---------|
| OS | Ubuntu 24.04 (x86_64) |
| Python | 3.11 (required by Isaac Sim 5.x) |
| Isaac Sim | 5.1.0 |
| Isaac Lab | 2.3.2.post1 |
| PyTorch | 2.7.0 (CUDA 12.8) |
| NVIDIA driver | 580.x (Production Branch) |

Use a Production Branch NVIDIA driver (580.x on Linux). The 595.x New Feature Branch
drivers crash Isaac Sim 5.1's RTX renderer at startup (segfault in
`librtx.scenedb.plugin.so`); this takes down the GUI and `--enable_cameras`, while plain
`--headless` still runs. On Ubuntu:

```bash
sudo apt install nvidia-driver-580-open
sudo reboot
nvidia-smi   # should report Driver Version: 580.x
```

### Install

```bash
git clone --recurse-submodules <repo-url>
cd MOVIN-Isaac-Plugin

# Python 3.11 virtual environment (uv shown; conda or python3.11 -m venv work too)
uv venv --python 3.11 --seed .venv
source .venv/bin/activate

# PyTorch, then Isaac Lab + Isaac Sim, then the plugin
uv pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
uv pip install "isaaclab[isaacsim]==2.3.2.post1" -e . \
    --extra-index-url https://pypi.nvidia.com \
    --extra-index-url https://download.pytorch.org/whl/cu128 \
    --index-strategy unsafe-best-match
```

With plain pip, drop `uv` from the `pip install` lines and `--index-strategy unsafe-best-match`.

MOVIN-SDK-Python comes with the plugin as the `movin_sdk_python` submodule, and `pip install -e .`
installs it from there. Do not also install a separate MOVIN-SDK-Python checkout into the same
environment: two installations of the `movin_sdk_python` package shadow each other, and the plugin
can end up importing the other one.

If you already cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

Isaac Sim asks you to accept the NVIDIA Omniverse EULA on first launch. To accept it
non-interactively (required for headless and scripted runs):

```bash
export OMNI_KIT_ACCEPT_EULA=YES
```

The first launch takes several minutes while Isaac Sim builds its caches; later launches
take about 10 seconds.

## Project Structure

```
MOVIN-Isaac-Plugin/
  movin_sdk_python/              # git submodule (MOVIN-SDK-Python)
  examples/
    mocap_to_isaaclab.py         # MOVIN -> Isaac Lab visualization
    bvh_utils.py                 # BVH loader + BVH->Isaac/retarget conversion (plugin-owned)
    skeleton_assets.py           # Skeleton MJCF selection (reference / actor-fitted) + USD cache layout
    robot_pose.py                # Retargeted robot root pose in Isaac (forward mode, side-by-side offset)
  scripts/
    generate_skeleton_mjcf.py    # Generate a skeleton MJCF from a T-pose BVH
    extract_movinman_mesh.py     # Extract mesh + skin weights from a MOVIN FBX into an NPZ (runs in Blender)
    fake_movin_studio.py         # Stream a BVH as MOVIN Studio OSC output (testing without a mocap rig)
    run_e2e.sh                   # End-to-end test of every mode in Isaac Lab (headless or --gui)
    capture_viewport.py          # Viewport screenshots for run_e2e.sh --gui
  tests/
    test_skeleton_assets.py      # Checks the SDK's V3 MJCF against the generator, its T-pose and the preset
    test_mesh_assets.py          # Checks the SDK's V3 mesh NPZ against the LBS path, the T-pose and
                                 # (when Blender is available) the extractor
    test_fake_studio_stream.py   # Checks the fake MOVIN Studio stream against the BVH path
    test_fitting.py              # Checks body-proportion fitting, BVH sizing and the USD cache layout
    test_robot_pose.py           # Checks that the robot follows the character's forward mode
    test_bvh_units.py            # Checks that a centimeter BVH plays like the same clip in meters
    test_packaging.py            # Checks that the plugin's packaging ships every SDK package file
  data/
    MOVINMan_dump.xml            # MOVINMan FBX dump
    MOVINman_V3_local_rotation_identity.fbx
                                 # MOVINManV3 rig with identity local rotations in the bind pose (skinned mesh)
    MOVINman_V3_local_rotation_identity_tpose.bvh
                                 # Its rest pose as a BVH (source of the V3 MJCF)
    Locomotion.bvh               # Sample BVH for testing
    test_V3.bvh                  # Sample MOVINManV3 BVH for testing
    ground/                      # Vendored Isaac grid ground plane (avoids the cloud asset fetch)
    usd_output/                  # MJCF -> USD conversion cache (generated, git-ignored)
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

No MOVIN Studio at hand? Stream a BVH file in its place with
`scripts/fake_movin_studio.py` (see [Testing without MOVIN Studio](#testing-without-movin-studio)).

The receiver answers MOVIN Studio's OSC status probe, so Studio's Streaming panel shows **OSC connected**. Its *Viewer* fps is the rate at which the example consumes frames, i.e. its render loop rate.

## Body Proportion Fitting

The skeleton MJCF and the mesh ship with MOVIN-SDK-Python in the reference character's proportions. MOVIN Studio calibrates every actor and streams that actor's bone offsets, so by default the example fits both assets to the actor before building the scene:

- **Skeleton**: the SDK's MJCF is fitted with `fit_skeleton_mjcf()` (body offsets from the actor, root height adjusted so the T-pose stands on the ground, capsules resized) and converted to USD.
- **Mesh**: the SDK's mesh is fitted with `MOVINMeshModel.fit_to_bones()`; in live and replay mode it is posed with `pose_from_bones()`, which refits whenever the streamed offsets change.

The proportions come from:

| Mode | Source |
|------|--------|
| `live` | the first streamed frame (the example waits for it before building the scene) |
| `replay` | the recording's first frame |
| `bvh` | the BVH file's `OFFSET`s (MOVIN Studio's BVH exports carry the actor's calibration too) |

Fitting is **on by default**. Pass `--no_fit` to use the SDK's reference assets (`preset.mjcf_path`, `preset.mesh_npz_path`) as they are:

```bash
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --view_mode mesh_skeleton --no_fit
```

Notes:

- Live mode always waits for the first frame while fitting is on; only `--preset <name> --no_fit` starts without one.
- The USD conversions are cached in `data/usd_output/skeletons/`, one directory per MJCF content: `<preset>_skeleton_fitted_<hash>/` for each fitted actor (holding the fitted MJCF as well) and `<preset>_skeleton_reference_<hash>/` for `--no_fit`. A new actor gets its own conversion, a returning actor reuses it, and nothing is written into the SDK package. The directory can be deleted at any time; it is rebuilt on demand.
- If the actor is recalibrated or switched while the example runs, the mesh refits automatically, but the skeleton articulation keeps the proportions it was built with; the example prints a warning, and a restart refits it.
- If the stream's hierarchy does not match the preset (for example `--preset movinman` on a MOVINManV3 stream), the example prints a warning and uses the reference proportions.

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

Stop recording by closing the window or pressing `Ctrl+C` in the terminal: either way the example finishes the current frame, writes the recording and releases the UDP port. A second `Ctrl+C` force-quits, except while the recording is being written.

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

# Size the robot targets for a 1.70 m actor instead of measuring the stream
python examples/mocap_to_isaaclab.py --mode live --port 11235 \
    --robot unitree_g1 --human_height 1.70
```

`--human_height auto|<actor height (m)>` sets how the actor is normalized to the reference body the retargeting tables were tuned for:

- `auto` (default): measured from the actor's bones by the SDK Retargeter (in live and replay mode from the stream, recomputed if the actor is recalibrated; in BVH mode once from the file's `OFFSET`s).
- a number: the actor's standing height in meters.

Earlier versions of the plugin passed a number with a different meaning (default `1.75`); to reproduce a result obtained with an old value `h`, pass `3.24 / h` (the old default corresponds to about `1.85`).

### Supported Robots

| Robot | Description | DoFs |
|-------|-------------|------|
| `unitree_g1` | Unitree G1 (standard) | 29 |
| `unitree_g1_with_hands` | Unitree G1 with hands | 43 |

The robot MJCFs ship with MOVIN-SDK-Python. The example converts each to USD on first use and caches it in `data/usd_output/robots/<model>/` (rebuilt when the MJCF is newer), so nothing is written into the SDK package.

## Skeleton Presets

Two MOVINMan skeleton layouts are supported:

- `movinman` -- legacy skeleton (51 bodies)
- `movinman_v3` -- MOVINManV3 skeleton (54 joints), adding `Spine2`, `Spine3`, and `Neck1` to the spine/neck chain, with a full finger set

By default (`--preset auto`) the preset is detected automatically, per mode:

- **bvh**: detected from the joint names in the BVH file
- **replay**: detected by peeking at the recording's first frame
- **live**: waits for the first mocap frame before starting (it waits for it anyway while [fitting](#body-proportion-fitting) is on; pass `--preset movinman` or `--preset movinman_v3` together with `--no_fit` to skip the wait)

Pass `--preset movinman` or `--preset movinman_v3` to force a specific preset instead of auto-detecting.

The skeleton MJCF and mesh of each preset are the assets packaged with MOVIN-SDK-Python (`movin_sdk_python/movin_sdk_python/assets/`, resolved through `SkeletonPreset.mjcf_path` and `mesh_npz_path`). This repo keeps their sources (`data/`) and generators (`scripts/`): after regenerating an asset, replace the file in MOVIN-SDK-Python's `movin_sdk_python/assets/` and update the submodule. The tests check that the packaged V3 MJCF is the generator's output and, when Blender is available, that the packaged V3 mesh is the extractor's output.

Mesh overlay (`--view_mode mesh` / `mesh_skeleton`) is available for both presets: `movinman` uses the SDK's `movinman_mesh.npz` and `movinman_v3` its `movinman_v3_mesh.npz`. Pass `--mesh_npz` to use another asset with the preset's bone layout (it is fitted like the packaged one). A preset without a mesh asset prints a warning and falls back to skeleton-only rendering.

The SDK's `movinman_v3_mesh.npz` is extracted from the skinned mesh of `data/MOVINman_V3_local_rotation_identity.fbx` (44,402 vertices, 4 influences per vertex) with Blender's bundled FBX importer; the script checks that every joint's rest rotation is identity, which is what makes the NPZ's translation-only bind frames valid:

```bash
blender -b --python scripts/extract_movinman_mesh.py -- \
    --fbx data/MOVINman_V3_local_rotation_identity.fbx \
    --out movinman_v3_mesh.npz --preset movinman_v3
```

`tests/test_mesh_assets.py` runs this extraction when a `blender` executable is found (`$BLENDER` or on `PATH`) and checks that its output equals the packaged NPZ; without Blender that test is skipped.

Robot retargeting works with both presets; with `movinman_v3` the G1 torso is automatically mapped to `Spine3` via a V3-specific IK config.

The SDK's `movinman_v3_skeleton.xml` is generated from `data/MOVINman_V3_local_rotation_identity_tpose.bvh`, the rest pose of `data/MOVINman_V3_local_rotation_identity.fbx` (a MOVINManV3 rig whose bind pose has identity local rotations on every joint). Each `OFFSET` equals the FBX `Lcl Translation`, i.e. the joint's translation in its parent's frame -- the same `p` MOVIN Studio streams and the same `OFFSET` found in its BVH exports. Because the MJCF bodies sit in those frames, the streamed and BVH local rotations drive the hinges directly and no rest-pose removal is applied: streams from the earlier MOVINManV3 rig carry non-identity rest quaternions on the finger bones (`Thumb1` about 60 degrees) and removing them would straighten the fingers, while the identity-local rig streams identity rest quaternions, where removal is a no-op. The body order follows the BVH and the live stream (fingers Thumb, Index, Middle, Ring, Pinky within each hand); the FBX itself lists fingers alphabetically, which nothing at runtime depends on. Regenerate the MJCF with:

```bash
python scripts/generate_skeleton_mjcf.py \
    data/MOVINman_V3_local_rotation_identity_tpose.bvh \
    movinman_v3_skeleton.xml --model-name movinman_v3
```

The generator mirrors geom and actuator styling from the SDK's packaged legacy `movinman_skeleton.xml` (override with `--reference-mjcf`). `python -m unittest discover -s tests` checks that the SDK's packaged V3 MJCF is exactly this command's output and that its body order and offsets match the SDK preset and the T-pose.

## Options

| Option | Values | Description |
|--------|--------|-------------|
| `--mode` | `live`, `bvh`, `replay` | Input source |
| `--preset` | `auto`, `movinman`, `movinman_v3` | Skeleton preset (default: `auto`, detected from the data source) |
| `--port` | int (default: 11235) | UDP port for live mocap |
| `--bvh_file` | `<path>` | BVH file for playback mode |
| `--bvh_scale` | float | Factor converting BVH positions to meters, for the character, mesh and robot alike (auto-detected: `0.01` for centimeter files, else `1`) |
| `--view_mode` | `skeleton`, `mesh`, `mesh_skeleton` | Visualization mode |
| `--forward_mode` | `coord_equivalent`, `isaac_world` | Facing direction (`isaac_world` = +90 deg yaw for Isaac +X; the robot is turned with the character) |
| `--record` | `<path>` | Record live session to `.pkl` file |
| `--recording` | `<path>` | Replay from recorded `.pkl` file |
| `--robot` | `unitree_g1`, `unitree_g1_with_hands` | Enable robot retargeting |
| `--human_height` | `auto` or meters (default: `auto`) | Actor size for retargeting: `auto` measures it from the bones, a number is the actor's height |
| `--robot_view` | `side_by_side`, `robot_only`, `overlay` | Robot display mode |
| `--robot_offset` | float (default: 1.0) | X offset for side-by-side view |
| `--mesh_npz` | `<path>` | Mesh NPZ for the overlay (default: the preset's mesh packaged with the SDK) |
| `--no_fit` | | Use the SDK's reference-proportion skeleton and mesh as-is (default: fit them to the actor) |
| `--headless` | | No GUI window |
| `--debug` | | Print FPS, pose drift and debug info |
| `--max_frames` | int | Exit the main loop after N frames (mainly for headless testing) |
| `--print_joints` | | Print the Isaac Lab and skeleton joint order, then exit |

## Viewer Controls

- `Alt + left drag` -- orbit camera
- Mouse wheel -- zoom
- `Enter` -- pause / resume
- `Space` -- single-step (while paused)
- `M` -- toggle mesh visibility (`mesh_skeleton` mode)
- `K` -- toggle skeleton visibility (`mesh_skeleton` mode)

## Testing

Unit tests (no Isaac Sim needed; a few seconds, plus one run of the Blender mesh extractor when Blender is installed):

```bash
python -m unittest discover -s tests
```

### Testing without MOVIN Studio

`scripts/fake_movin_studio.py` streams a BVH file as MOVIN Studio's OSC output
(`/MOVIN/Frame` packets over UDP), so live mode and `--record` work without a mocap rig.
It loops the clip at its own frame rate until stopped:

```bash
# Terminal 1: the fake stream (MOVINManV3 clip; use data/Locomotion.bvh for MOVINMan)
python scripts/fake_movin_studio.py data/test_V3.bvh --port 11235

# Terminal 2: the plugin in live mode
python examples/mocap_to_isaaclab.py --mode live --port 11235 --view_mode mesh_skeleton
```

`tests/test_fake_studio_stream.py` checks that frames received through the SDK's
`MocapReceiver` match the BVH conversion of the same frames (root pose, joint DOFs and
retargeter input).

### End-to-end

`scripts/run_e2e.sh` runs the full pipeline in Isaac Lab: BVH playback with both presets,
every view mode and both robots, live mode fed by the fake stream with `--record`, and
replay of those recordings. Each scenario must reach `--max_frames`, exit cleanly, log no
traceback, crash or retarget error, and keep its articulations where they are written: the
`--debug` *pose drift* (how far the physics step moved the character and robot away from the
pose written to them) must stay within 1 mm / 0.5 deg.

```bash
scripts/run_e2e.sh            # headless (about 2 minutes)
scripts/run_e2e.sh --gui      # windowed, saves a viewport screenshot per scenario
scripts/run_e2e.sh live       # only scenarios whose name matches the regex
```

Logs, recordings and screenshots go to `build/e2e/<headless|gui>/`; `summary.txt` there lists
the result of each scenario.

## Dependencies

- [MOVIN-SDK-Python](https://github.com/MOVIN3D/MOVIN-SDK-Python) (included as submodule)
- [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) / Isaac Sim
- NumPy, SciPy, MuJoCo

## License

MIT
