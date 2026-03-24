"""
MOVIN Mocap to Isaac Lab Visualization.

Receives real-time mocap data from MOVIN Studio (or plays back a BVH file)
and drives a humanoid skeleton in Isaac Lab.  Optionally renders a high-fidelity
surface mesh overlay via Linear Blend Skinning from the MOVINMan.fbx model.

Usage:
    # BVH playback (body-only)
    python examples/mocap_to_isaaclab.py --mode bvh --bvh_file examples/Locomotion.bvh

    # BVH playback (full skeleton with hands)
    python examples/mocap_to_isaaclab.py --mode bvh --bvh_file data/kthjazz_*.bvh

    # BVH playback with mesh overlay
    python examples/mocap_to_isaaclab.py --mode bvh --bvh_file examples/Locomotion.bvh --view_mode mesh_skeleton

    # BVH playback with mesh only (no skeleton)
    python examples/mocap_to_isaaclab.py --mode bvh --bvh_file examples/Locomotion.bvh --view_mode mesh

    # Live mocap from MOVIN Studio
    python examples/mocap_to_isaaclab.py --mode live --port 11235

    # Live mocap with mesh overlay and Isaac world-forward (+X) facing
    python examples/mocap_to_isaaclab.py --mode live --port 11235 --view_mode mesh_skeleton --forward_mode isaac_world

    # Headless mode
    python examples/mocap_to_isaaclab.py --mode bvh --bvh_file examples/Locomotion.bvh --headless

Visualizer UI:
    Camera control: Hold Alt + left-drag to orbit, scroll to zoom
    Pause/Resume:   Enter key
    Step one frame: Space key
    Toggle mesh:    M key  (mesh_skeleton mode only)
    Toggle skeleton: K key (mesh_skeleton mode only)

Requires Isaac Lab (isaaclab) to be installed and available in the Python environment.
"""

import argparse
import enum
import os
import sys
import time

# ---- Isaac Lab AppLauncher MUST come before any other omniverse imports ----
parser = argparse.ArgumentParser(description="MOVIN Mocap to Isaac Lab Visualization")
parser.add_argument("--mode", type=str, default="bvh", choices=["live", "bvh", "replay"],
                    help="Data source: 'live' for real-time mocap, 'bvh' for file playback, 'replay' for recorded data")
parser.add_argument("--recording", type=str, default=None,
                    help="Path to recorded .pkl file for replay mode")
parser.add_argument("--record", type=str, default=None,
                    help="Record live session to .pkl file")
parser.add_argument("--port", type=int, default=11235,
                    help="UDP port for live mocap (default: 11235)")
parser.add_argument("--bvh_file", type=str, default=None,
                    help="Path to BVH file for playback mode")
parser.add_argument("--bvh_scale", type=float, default=None,
                    help="Scale factor for BVH positions (e.g. 0.01 for cm->m). Auto-detected if not set.")
parser.add_argument("--view_mode", type=str, default="skeleton",
                    choices=["skeleton", "mesh", "mesh_skeleton"],
                    help="Visualization mode: skeleton only, mesh only, or both")
parser.add_argument("--mesh_npz", type=str, default=None,
                    help="Path to movinman_mesh.npz (auto-detected if not set)")
parser.add_argument("--debug", action="store_true",
                    help="Print FPS and debug info")
parser.add_argument("--print_joints", action="store_true",
                    help="Print joint names and ordering at startup, then exit")
parser.add_argument(
    "--forward_mode",
    type=str,
    default="coord_equivalent",
    choices=["coord_equivalent", "isaac_world"],
    help=(
        "Root-facing policy: 'coord_equivalent' keeps Unity +Z -> Isaac -Y, "
        "'isaac_world' adds a +90 deg Isaac-Z yaw so Unity +Z faces Isaac +X."
    ),
)

from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
cli_args = parser.parse_args()

app_launcher = AppLauncher(cli_args)
simulation_app = app_launcher.app

# ---- Now safe to import omniverse/isaaclab modules ----
import carb
import carb.input
import numpy as np
import torch

HEADLESS = getattr(cli_args, "headless", False)

if not HEADLESS:
    try:
        import omni.appwindow
    except ModuleNotFoundError:
        omni = None
else:
    omni = None

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.sim.converters import MjcfConverterCfg, MjcfConverter
from isaacsim.core.utils.extensions import enable_extension

# Add project root and SDK submodule to path for movin_sdk_python imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
SDK_ROOT = os.path.join(PROJECT_ROOT, "movin_sdk_python")
for p in (PROJECT_ROOT, SDK_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

from movin_sdk_python.utils.isaac_lab_utils import (
    process_movin_bones_for_isaaclab,
    process_bvh_frame_for_isaaclab,
    build_dof_reorder_map,
    FORWARD_MODE_CHOICES,
    SKELETON_BODY_NAMES,
    SKELETON_JOINT_BONES,
    DEFAULT_HIPS_HEIGHT,
)
from movin_sdk_python.utils.bvh_loader import read_bvh
from movin_sdk_python.utils.movinman_mesh_utils import (
    MOVINMeshModel,
    extract_movin_local_quats_yup,
    extract_bvh_local_quats_yup,
)

CHARACTER_ROOT_PATH = "/World/envs/env_0/character"
MESH_PRIM_PATH = "/World/MOVINManMesh"


# ---------------------------------------------------------------------------
# Mesh overlay (matches NovaMeshOverlay in nova_to_isaaclab.py)
# ---------------------------------------------------------------------------

class MeshOverlay:
    """Single world-space USD mesh updated from LBS-posed vertices."""

    def __init__(self, mesh_model, prim_path=MESH_PRIM_PATH):
        import omni.usd
        from pxr import Gf, UsdGeom, Vt

        self._Vt = Vt
        self._Gf = Gf
        self._stage = omni.usd.get_context().get_stage()
        self._mesh_prim = UsdGeom.Mesh.Define(self._stage, prim_path)
        face_counts = [3] * int(mesh_model.faces.shape[0])
        face_indices = mesh_model.faces.reshape(-1).astype(np.int32).tolist()

        self._mesh_prim.CreateFaceVertexCountsAttr(face_counts)
        self._mesh_prim.CreateFaceVertexIndicesAttr(face_indices)
        self._mesh_prim.CreateSubdivisionSchemeAttr().Set("none")
        self._mesh_prim.CreateDoubleSidedAttr().Set(True)
        self._mesh_prim.CreateDisplayColorAttr().Set([Gf.Vec3f(0.78, 0.79, 0.82)])
        self._mesh_prim.CreateDisplayOpacityAttr().Set([0.96])
        self._points_attr = self._mesh_prim.CreatePointsAttr()
        self._visible = None
        self.update_vertices(np.zeros((mesh_model.num_vertices, 3), dtype=np.float32))
        self.set_visible(False)

    def update_vertices(self, vertices):
        vertices = np.asarray(vertices, dtype=np.float32)
        if hasattr(self._Vt.Vec3fArray, "FromNumpy"):
            vt_points = self._Vt.Vec3fArray.FromNumpy(vertices)
        else:
            vt_points = self._Vt.Vec3fArray(
                [self._Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in vertices]
            )
        self._points_attr.Set(vt_points)

    def set_visible(self, visible):
        from pxr import UsdGeom

        if self._visible == visible:
            return
        imageable = UsdGeom.Imageable(self._mesh_prim.GetPrim())
        if visible:
            imageable.MakeVisible()
        else:
            imageable.MakeInvisible()
        self._visible = visible


def set_prim_visibility(prim_path, visible):
    import omni.usd
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return False
    imageable = UsdGeom.Imageable(prim)
    if visible:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()
    return True


# ---------------------------------------------------------------------------
# Play mode (matches MimicKit sim_env.py pattern)
# ---------------------------------------------------------------------------

class PlayMode(enum.Enum):
    PLAY = 0
    ONE_STEP = 1
    PAUSE = 2


# ---------------------------------------------------------------------------
# Camera tracker (matches MimicKit camera.py pattern)
# ---------------------------------------------------------------------------

class CameraTracker:
    """Track a character while respecting user camera adjustments.

    Uses ViewportCameraState to read the actual viewport camera each frame
    (including user Alt+drag/scroll changes), then re-centers on the character
    while preserving the user's camera offset. Matches MimicKit camera.py.
    """

    def __init__(self, sim, pos, target):
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        self._sim = sim
        self._camera_state = ViewportCameraState("/OmniverseKit_Persp")
        self._pos = np.array(pos, dtype=np.float64)
        self._target = np.array(target, dtype=np.float64)
        self._sim.set_camera_view(eye=self._pos, target=self._target)

    def update(self, char_root_pos):
        """Re-center on character root, preserving user camera adjustments."""
        # Read current viewport camera (includes user mouse changes)
        cam_pos = np.array(self._camera_state.position_world, dtype=np.float64)
        cam_target = np.array(self._camera_state.target_world, dtype=np.float64)
        cam_delta = cam_target - cam_pos
        cam_dist = np.linalg.norm(cam_delta)
        if cam_dist < 1e-6:
            cam_dist = 1.0
        cam_dir = cam_delta / cam_dist

        prev_delta = self._target - self._pos
        prev_dist = np.linalg.norm(prev_delta)
        if prev_dist < 1e-6:
            prev_dist = 1.0
        prev_dir = prev_delta / prev_dist

        # Detect user camera changes: position moved or direction changed
        if not np.allclose(self._pos, cam_pos, atol=1e-5):
            # User dragged the camera — preserve new offset from target
            cam_offset = cam_pos - self._target
        elif not np.allclose(prev_dir, cam_dir, atol=1e-5):
            # User rotated the camera — preserve distance, use new direction
            cam_offset = -cam_dist * cam_dir
        else:
            cam_offset = cam_pos - self._target

        # New target follows character XY, preserves previous Z
        new_target = np.array([char_root_pos[0], char_root_pos[1], self._target[2]])
        new_pos = new_target + cam_offset

        self._sim.set_camera_view(eye=new_pos, target=new_target)
        self._pos[:] = new_pos
        self._target[:] = new_target


# ---------------------------------------------------------------------------
# Scene setup helpers
# ---------------------------------------------------------------------------

def build_ground():
    """Create a ground plane."""
    from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

    physics_material = sim_utils.RigidBodyMaterialCfg(
        static_friction=1.0, dynamic_friction=1.0, restitution=0.0
    )
    ground_cfg = GroundPlaneCfg(
        physics_material=physics_material,
        color=(0.017, 0.0153, 0.01275),
        size=(100.0, 100.0),
    )
    spawn_ground_plane(prim_path="/World/ground", cfg=ground_cfg)


def build_lights():
    """Create scene lighting."""
    import isaacsim.core.utils.prims as prim_utils

    prim_utils.create_prim("/World/Light", "Xform")

    distant_light_cfg = sim_utils.DistantLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
    distant_light_cfg.func("/World/Light/distant_light", distant_light_cfg)

    dome_light_cfg = sim_utils.DomeLightCfg(intensity=800.0, color=(0.7, 0.7, 0.7))
    dome_light_cfg.func("/World/Light/dome_light", dome_light_cfg)


def convert_mjcf_to_usd(mjcf_path):
    """Convert MJCF XML to USD using Isaac Lab's converter."""
    mjcf_path = os.path.abspath(mjcf_path)
    usd_dir = os.path.join(os.path.dirname(mjcf_path), "usd_output")
    usd_filename = os.path.splitext(os.path.basename(mjcf_path))[0] + ".usd"
    usd_path = os.path.join(usd_dir, usd_filename)
    needs_rebuild = (
        not os.path.exists(usd_path)
        or os.path.getmtime(usd_path) < os.path.getmtime(mjcf_path)
    )

    if not needs_rebuild:
        return usd_path

    if os.path.exists(usd_path):
        print(f"[INFO] Rebuilding stale USD cache: {usd_path}")
    else:
        print(f"[INFO] Creating USD cache: {usd_path}")

    enable_extension("isaacsim.asset.importer.mjcf")
    for _ in range(2):
        simulation_app.update()

    converter_cfg = MjcfConverterCfg(
        asset_path=mjcf_path,
        usd_dir=usd_dir,
        usd_file_name=usd_filename,
        force_usd_conversion=needs_rebuild,
        fix_base=False,
        make_instanceable=False,
    )
    converter = MjcfConverter(converter_cfg)
    return converter.usd_path


def create_articulation(usd_path, start_pos=(0.0, 0.0, DEFAULT_HIPS_HEIGHT)):
    """Create an Isaac Lab Articulation from a USD file.

    Spawns at container path, points ArticulationCfg at Hips subtree
    (avoids dual articulation root from MJCF converter).
    Gravity disabled for kinematic driving.
    """
    spawn_path = "/World/envs/env_0/character"

    articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False,
        fix_root_link=False,
    )
    rigid_props = sim_utils.RigidBodyPropertiesCfg(
        max_depenetration_velocity=10.0,
        angular_damping=0.0,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        disable_gravity=True,
    )

    usd_cfg = sim_utils.UsdFileCfg(
        usd_path=usd_path,
        articulation_props=articulation_props,
        rigid_props=rigid_props,
    )
    usd_cfg.func(spawn_path, usd_cfg)

    actuator_cfg = IdealPDActuatorCfg(
        joint_names_expr=[".*"],
        stiffness=0,
        damping=0,
        effort_limit=0,
    )

    init_state = ArticulationCfg.InitialStateCfg(
        pos=start_pos,
        rot=(1.0, 0.0, 0.0, 0.0),
    )

    art_cfg = ArticulationCfg(
        prim_path=f"{spawn_path}/Hips",
        spawn=None,
        init_state=init_state,
        actuators={"actuators": actuator_cfg},
    )

    articulation = Articulation(art_cfg)
    return articulation


def detect_bvh_scale(bvh_data):
    """Auto-detect whether BVH positions are in centimeters or meters."""
    root_y_values = bvh_data.pos[:, 0, 1]  # root Y position (height in Y-up)
    avg_height = np.mean(np.abs(root_y_values))
    if avg_height > 10.0:
        return 0.01
    else:
        return 1.0


def get_bvh_fps(bvh_file):
    """Read frame time from BVH file header."""
    with open(bvh_file, 'r') as f:
        for line in f:
            if 'Frame Time' in line:
                ft = float(line.strip().split()[-1])
                if ft > 0:
                    return 1.0 / ft
    return 60.0


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = cli_args

    # Validate args
    if args.mode == "bvh" and args.bvh_file is None:
        print("ERROR: --bvh_file is required in bvh mode")
        sys.exit(1)

    # ---- View mode flags ----
    mesh_enabled = args.view_mode in ("mesh", "mesh_skeleton")
    skeleton_visible = args.view_mode in ("skeleton", "mesh_skeleton")
    mesh_only = args.view_mode == "mesh"

    # ---- Determine control FPS ----
    if args.mode == "bvh":
        control_fps = get_bvh_fps(args.bvh_file)
    else:
        control_fps = 60.0

    # ---- Simulation context ----
    # Match MimicKit ViewMotionEnv: sim_freq = control_freq, render_interval = 1
    # One physics step per control step, render every step.
    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / control_fps, render_interval=1)
    sim = sim_utils.SimulationContext(sim_cfg)

    # Disable Isaac Sim rate limiter (MimicKit pattern) -- we throttle ourselves
    carb_settings = carb.settings.get_settings()
    carb_settings.set_bool("/app/runLoops/main/rateLimitEnabled", False)

    # ---- Scene ----
    build_ground()
    build_lights()

    # ---- Character (skeleton articulation) ----
    articulation = None
    if not mesh_only:
        mjcf_path = os.path.join(PROJECT_ROOT, "data", "movinman_skeleton.xml")
        print(f"[INFO] Converting MJCF: {mjcf_path}")
        usd_path = convert_mjcf_to_usd(mjcf_path)
        print(f"[INFO] USD path: {usd_path}")
        articulation = create_articulation(usd_path)

    # ---- Initialize simulation ----
    sim.reset()
    print("[INFO] Simulation initialized")

    # ---- Mesh overlay setup ----
    mesh_model = None
    mesh_overlay = None
    if mesh_enabled:
        mesh_npz = args.mesh_npz
        if mesh_npz is None:
            mesh_npz = os.path.join(PROJECT_ROOT, "data", "movinman_mesh.npz")
        mesh_model = MOVINMeshModel(npz_path=mesh_npz)
        mesh_overlay = MeshOverlay(mesh_model, prim_path=MESH_PRIM_PATH)
        print(
            f"[INFO] Mesh overlay initialized: "
            f"{mesh_model.num_vertices} verts, {mesh_model.faces.shape[0]} faces"
        )

    if articulation is not None:
        set_prim_visibility(CHARACTER_ROOT_PATH, skeleton_visible)

    # ---- Read joint ordering from Isaac Lab ----
    isaac_joint_names = []
    num_dofs = 0
    reorder_map = None

    if articulation is not None:
        isaac_joint_names = list(articulation.data.joint_names)
        num_dofs = len(isaac_joint_names)
        print(f"[INFO] Articulation has {num_dofs} DOFs, control FPS: {control_fps:.1f}")

        if args.print_joints:
            print("\n=== Isaac Lab Joint Order ===")
            for i, name in enumerate(isaac_joint_names):
                print(f"  [{i:3d}] {name}")
            print(f"\n=== Skeleton Joint Order (expected) ===")
            for i, bone in enumerate(SKELETON_JOINT_BONES):
                print(f"  [{i*3:3d}-{i*3+2:3d}] {bone}_x/y/z")
            simulation_app.close()
            return

        reorder_map = build_dof_reorder_map(isaac_joint_names)
        if reorder_map is not None:
            print(f"[INFO] DOF reorder map built (Isaac Lab has different joint ordering)")
        else:
            print(f"[INFO] Joint ordering matches skeleton (no reorder needed)")

    print(f"[INFO] Forward mode: {args.forward_mode} (choices: {', '.join(FORWARD_MODE_CHOICES)})")
    print(f"[INFO] View mode: {args.view_mode}")

    # ---- Keyboard UI (MimicKit sim_env.py pattern) ----
    play_mode = PlayMode.PLAY
    mesh_user_visible = True
    skeleton_user_visible = True

    def _on_keyboard_event(event):
        nonlocal play_mode, mesh_user_visible, skeleton_user_visible
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.ENTER:
                if play_mode == PlayMode.PLAY:
                    play_mode = PlayMode.PAUSE
                    print("[INFO] Paused (Enter to resume, Space to step)")
                else:
                    play_mode = PlayMode.PLAY
                    print("[INFO] Playing")
            elif event.input == carb.input.KeyboardInput.SPACE:
                play_mode = PlayMode.ONE_STEP
            elif event.input == carb.input.KeyboardInput.M:
                if mesh_enabled and mesh_overlay is not None:
                    mesh_user_visible = not mesh_user_visible
                    mesh_overlay.set_visible(mesh_user_visible)
                    print(f"[INFO] Mesh overlay {'shown' if mesh_user_visible else 'hidden'}")
            elif event.input == carb.input.KeyboardInput.K:
                if articulation is not None:
                    skeleton_user_visible = not skeleton_user_visible
                    set_prim_visibility(CHARACTER_ROOT_PATH, skeleton_user_visible)
                    print(f"[INFO] Skeleton {'shown' if skeleton_user_visible else 'hidden'}")

    input_interface = None
    if not HEADLESS and omni is not None:
        input_interface = carb.input.acquire_input_interface()
        app_window = omni.appwindow.get_default_app_window()
        keyboard = app_window.get_keyboard()
        _sub_keyboard = input_interface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    elif not HEADLESS:
        print("[WARN] omni.appwindow unavailable; keyboard controls disabled")

    # ---- Camera tracker ----
    camera = None
    if not HEADLESS and omni is not None:
        camera = CameraTracker(sim, pos=[0.0, -5.0, 3.0], target=[0.0, 0.0, 1.0])

    # ---- Data source setup ----
    receiver = None
    bvh_data = None
    bvh_frame_idx = 0
    bvh_scale = 1.0

    if args.mode == "live":
        from movin_sdk_python.mocap_receiver.mocap_receiver import MocapReceiver
        recorder = None
        if args.record:
            from movin_sdk_python.recording import OscRecorder
            recorder = OscRecorder(args.record, stream_type="movin")
        receiver = MocapReceiver(port=args.port, recorder=recorder)
        receiver.start()
        print(f"[INFO] Live mocap receiver started on port {args.port}")

    elif args.mode == "replay":
        if not args.recording:
            print("ERROR: --recording is required in replay mode")
            simulation_app.close()
            return
        from movin_sdk_python.recording import ReplayMocapReceiver
        receiver = ReplayMocapReceiver(args.recording, realtime=True, loop=True)
        receiver.start()
        print(f"[INFO] Replay receiver started from {args.recording}")

    elif args.mode == "bvh":
        bvh_file = args.bvh_file
        if not os.path.exists(bvh_file):
            print(f"ERROR: BVH file not found: {bvh_file}")
            simulation_app.close()
            return

        bvh_data = read_bvh(bvh_file)

        if args.bvh_scale is not None:
            bvh_scale = args.bvh_scale
        else:
            bvh_scale = detect_bvh_scale(bvh_data)

        num_frames = bvh_data.quats.shape[0]
        print(f"[INFO] Loaded BVH: {bvh_file}")
        print(f"[INFO]   Bones: {len(bvh_data.bones)}, Frames: {num_frames}, FPS: {control_fps:.1f}")
        print(f"[INFO]   Position scale: {bvh_scale} ({'cm->m' if bvh_scale < 1.0 else 'already meters'})")

    # ---- Persistent state for kinematic driving ----
    last_root_pos = np.array([0.0, 0.0, DEFAULT_HIPS_HEIGHT])
    last_root_quat = np.array([1.0, 0.0, 0.0, 0.0])
    last_dof_values = np.zeros(num_dofs)
    last_mesh_local_quats_yup = None
    last_mesh_root_pos_yup = None
    has_first_frame = False

    # Pre-allocate tensors (reuse each frame to avoid CPU->GPU allocation overhead)
    env_ids = torch.tensor([0], dtype=torch.long, device=sim.device)
    root_vel = torch.zeros(1, 6, dtype=torch.float32, device=sim.device)
    root_pose_t = torch.zeros(1, 7, dtype=torch.float32, device=sim.device)
    dof_pos_t = torch.zeros(1, num_dofs, dtype=torch.float32, device=sim.device) if num_dofs > 0 else None
    dof_vel_t = torch.zeros(1, num_dofs, dtype=torch.float32, device=sim.device) if num_dofs > 0 else None
    # Pre-allocate DOF reorder buffer (avoids np.zeros_like per frame)
    dof_reordered = np.zeros(num_dofs) if reorder_map is not None else None

    # ---- Main loop ----
    frame_count = 0
    last_fps_time = time.time()
    playback_start_time = None  # wall-clock time when playback started
    playback_start_frame = 0    # BVH frame index at playback start

    print("[INFO] Starting main loop...")
    print("[INFO] Controls: Enter=pause/play, Space=step, Alt+drag=orbit, scroll=zoom")
    if mesh_enabled and articulation is not None:
        print("[INFO] Controls: M=toggle mesh, K=toggle skeleton")

    while simulation_app.is_running():
        # ---- Pause loop (MimicKit _render pattern) ----
        # Keep rendering + processing keyboard events while paused
        if play_mode == PlayMode.PAUSE:
            if camera is not None:
                camera.update(last_root_pos)
            sim.render()
            playback_start_time = None  # reset so we re-sync on resume
            continue

        # ---- Get new frame data ----
        if args.mode in ("live", "replay"):
            frame = receiver.get_latest_frame()
            if frame is not None:
                if not mesh_only:
                    root_pos, root_quat, dof_values = process_movin_bones_for_isaaclab(
                        frame["bones"],
                        forward_mode=args.forward_mode,
                    )
                    last_root_pos = root_pos
                    last_root_quat = root_quat
                    last_dof_values = dof_values

                if mesh_enabled:
                    mesh_quats, mesh_root = extract_movin_local_quats_yup(frame["bones"])
                    last_mesh_local_quats_yup = mesh_quats
                    last_mesh_root_pos_yup = mesh_root
                    if mesh_only:
                        # In mesh-only mode, derive root pos for camera from mesh root
                        from movin_sdk_python.utils.isaac_lab_utils import yup_to_zup_vec
                        last_root_pos = yup_to_zup_vec(mesh_root)

                has_first_frame = True

        elif args.mode == "bvh":
            # Time-based frame advancement: skip frames if rendering is slow
            # so that motion always plays at real-time speed
            now = time.time()
            if playback_start_time is None:
                playback_start_time = now
                playback_start_frame = bvh_frame_idx

            elapsed = now - playback_start_time
            target_frame = playback_start_frame + int(elapsed * control_fps)
            bvh_frame_idx = target_frame % num_frames

            frame_quats = bvh_data.quats[bvh_frame_idx]
            frame_pos = bvh_data.pos[bvh_frame_idx]

            if not mesh_only:
                root_pos, root_quat, dof_values = process_bvh_frame_for_isaaclab(
                    frame_quats, frame_pos, bvh_data.bones, forward_mode=args.forward_mode
                )
                root_pos = root_pos * bvh_scale
                last_root_pos = root_pos
                last_root_quat = root_quat
                last_dof_values = dof_values

            if mesh_enabled:
                mesh_quats, mesh_root = extract_bvh_local_quats_yup(
                    frame_quats, frame_pos, bvh_data.bones
                )
                last_mesh_local_quats_yup = mesh_quats
                last_mesh_root_pos_yup = mesh_root * bvh_scale
                if mesh_only:
                    from movin_sdk_python.utils.isaac_lab_utils import yup_to_zup_vec
                    last_root_pos = yup_to_zup_vec(mesh_root * bvh_scale)

            has_first_frame = True

        # ---- Write state to articulation ----
        if has_first_frame and articulation is not None:
            if dof_reordered is not None:
                dof_reordered[:] = 0.0
                dof_reordered[reorder_map] = last_dof_values
                dof_src = dof_reordered
            else:
                dof_src = last_dof_values

            # Update pre-allocated tensors in-place (avoids per-frame allocation)
            root_pose_t[0, :3] = torch.from_numpy(last_root_pos).float()
            root_pose_t[0, 3:] = torch.from_numpy(last_root_quat).float()
            dof_pos_t[0, :] = torch.from_numpy(dof_src).float()

            articulation.write_root_link_pose_to_sim(root_pose_t, env_ids)
            articulation.write_root_link_velocity_to_sim(root_vel, env_ids)
            articulation.write_joint_state_to_sim(dof_pos_t, dof_vel_t, env_ids=env_ids)

        # ---- Mesh overlay update ----
        if (
            mesh_enabled
            and mesh_model is not None
            and mesh_overlay is not None
            and last_mesh_local_quats_yup is not None
            and last_mesh_root_pos_yup is not None
        ):
            vertices = mesh_model.pose_vertices(
                last_mesh_local_quats_yup,
                last_mesh_root_pos_yup,
                forward_mode=args.forward_mode,
            )
            mesh_overlay.update_vertices(vertices)
            mesh_overlay.set_visible(mesh_user_visible)

        # ---- Step + render ----
        sim.step()
        if articulation is not None:
            articulation.update(sim.get_physics_dt())

        # ---- Camera tracking ----
        if camera is not None:
            camera.update(last_root_pos)

        # ---- One-step mode: advance one BVH frame then pause ----
        if play_mode == PlayMode.ONE_STEP:
            play_mode = PlayMode.PAUSE

        # ---- FPS tracking ----
        frame_count += 1
        if args.debug:
            now = time.time()
            if now - last_fps_time >= 2.0:
                fps = frame_count / (now - last_fps_time)
                info = f"[DEBUG] Render FPS: {fps:.1f} (target: {control_fps:.0f})"
                if args.mode == "bvh":
                    info += f" | BVH frame: {bvh_frame_idx}/{num_frames}"
                if args.mode in ("live", "replay") and receiver:
                    info += f" | Recv rate: {receiver.get_receive_rate():.1f} Hz"
                print(info)
                frame_count = 0
                last_fps_time = now

    # ---- Cleanup ----
    if receiver is not None:
        if hasattr(receiver, 'recorder') and receiver.recorder is not None:
            receiver.recorder.save()
        receiver.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
