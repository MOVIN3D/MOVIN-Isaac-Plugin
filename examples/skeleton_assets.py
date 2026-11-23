"""Skeleton MJCF selection and USD cache layout for mocap_to_isaaclab.py.

The skeleton and mesh assets ship with MOVIN-SDK-Python
(``SkeletonPreset.mjcf_path`` / ``mesh_npz_path``) in the reference
character's proportions.  By default the example fits them to the actor:
the MJCF through ``fit_skeleton_mjcf`` (this module), the mesh through
``MOVINMeshModel.fit_to_bones`` / ``pose_from_bones`` (in the example).

Isaac Lab needs the MJCF converted to USD, which is slow, so conversions are
cached under ``data/usd_output/skeletons/``, one directory per MJCF content:

    <preset stem>_reference_<hash>/   the SDK's packaged MJCF (``--no_fit``)
    <preset stem>_fitted_<hash>/      an actor-fitted MJCF (written here)

``<hash>`` hashes the MJCF text, so a different actor -- or an updated SDK
asset -- gets its own directory and conversion, the same actor reuses its USD,
and nothing is ever written next to the SDK's packaged files.

Pure Python (no Isaac imports), so it can be unit-tested.
"""

import hashlib
import os
from dataclasses import dataclass

from movin_sdk_python.utils.skeleton_fitting import fit_skeleton_mjcf

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CACHE_ROOT = os.path.join(PROJECT_ROOT, "data", "usd_output", "skeletons")

# Bump to invalidate every cached conversion (e.g. after changing the
# MJCF -> USD converter settings in mocap_to_isaaclab.py).
CACHE_VERSION = "1"


@dataclass(frozen=True)
class SkeletonAsset:
    """The MJCF to drive and where its USD conversion is cached."""

    mjcf_path: str
    usd_dir: str
    usd_file_name: str
    fitted: bool

    @property
    def usd_path(self):
        return os.path.join(self.usd_dir, self.usd_file_name)


def _content_key(text):
    digest = hashlib.sha256(CACHE_VERSION.encode() + b"\0" + text)
    return digest.hexdigest()[:12]


def _write_if_changed(path, text):
    """Write ``text`` (bytes) unless the file already holds it (keeps its mtime)."""
    if os.path.exists(path):
        with open(path, "rb") as f:
            if f.read() == text:
                return
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "wb") as f:
        f.write(text)
    os.replace(tmp, path)


def prepare_skeleton_asset(preset, bones=None, cache_root=DEFAULT_CACHE_ROOT):
    """Pick the skeleton MJCF for a run and the USD cache location for it.

    Args:
        preset: SkeletonPreset whose packaged MJCF is used.
        bones: Bones of one streamed frame (``frame["bones"]``, or
            ``bvh_rest_bones`` for a BVH) to fit the MJCF to, or None for the
            SDK's packaged reference MJCF as-is.
        cache_root: Root of the per-content USD cache directories.

    Returns:
        SkeletonAsset.  For a fitted skeleton the fitted MJCF has been written
        into its cache directory (rewritten only if missing or different).

    Raises:
        ValueError: From ``fit_skeleton_mjcf`` when the bones' hierarchy does
            not match the preset (e.g. a MOVINManV3 stream on the legacy preset).
    """
    stem = os.path.splitext(os.path.basename(preset.mjcf_filename))[0]
    usd_file_name = stem + ".usd"
    if bones is None:
        with open(preset.mjcf_path, "rb") as f:
            key = _content_key(f.read())
        usd_dir = os.path.join(cache_root, f"{stem}_reference_{key}")
        return SkeletonAsset(preset.mjcf_path, usd_dir, usd_file_name, fitted=False)

    text = fit_skeleton_mjcf(bones, preset).encode("utf-8")
    usd_dir = os.path.join(cache_root, f"{stem}_fitted_{_content_key(text)}")
    os.makedirs(usd_dir, exist_ok=True)
    mjcf_path = os.path.join(usd_dir, stem + ".xml")
    _write_if_changed(mjcf_path, text)
    return SkeletonAsset(mjcf_path, usd_dir, usd_file_name, fitted=True)


def proportions_key(bones, body_names):
    """Hashable summary of a frame's body proportions.

    The parent-local offsets ``p`` of the preset's non-root bodies: it changes
    when MOVIN Studio recalibrates or switches the actor, not with motion (the
    root body's offset is its per-frame placement and is left out).  Same rule
    as ``MOVINMeshModel.pose_from_bones`` uses to decide when to refit.
    """
    names = set(body_names[1:])
    return tuple((b["bone_name"], tuple(b["p"])) for b in bones if b["bone_name"] in names)
