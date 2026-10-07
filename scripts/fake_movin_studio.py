"""Stream a BVH file as MOVIN Studio OSC output, for testing without a mocap rig.

Every BVH frame is converted to the /MOVIN/Frame packets MOVIN Studio sends and
sent over UDP, so ``examples/mocap_to_isaaclab.py --mode live`` (and ``--record``)
can be exercised end to end on any machine. The conversion is the inverse of the
receiver's: BVH is right-handed Y-up, the stream is Unity left-handed, so positions
become (-x, y, z) and quaternions (w, x, -y, -z), sent in Unity's (x, y, z, w)
order. A stream root bone ("Root" for MOVINMan, "RootBone" for MOVINManV3) is
prepended above Hips like the real streams, and frames are split into chunks of
``--chunk`` bones per packet.

Usage:
    python scripts/fake_movin_studio.py data/test_V3.bvh --port 11235
    python scripts/fake_movin_studio.py data/Locomotion.bvh --seconds 30
"""

import argparse
import os
import socket
import struct
import sys
import time

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(PROJECT_ROOT, "examples"), os.path.join(PROJECT_ROOT, "movin_sdk_python")):
    if p not in sys.path:
        sys.path.insert(0, p)

from bvh_utils import read_bvh  # noqa: E402

V3_MARKER_BONES = {"Spine3", "Neck1"}


def osc_string(value):
    encoded = value.encode("utf-8") + b"\0"
    return encoded + b"\0" * (-len(encoded) % 4)


def osc_message(address, args):
    """Encode an OSC message with int, float and string arguments."""
    tags, payload = [], []
    for v in args:
        if isinstance(v, str):
            tags.append("s")
            payload.append(osc_string(v))
        elif isinstance(v, int):
            tags.append("i")
            payload.append(struct.pack(">i", v))
        else:
            tags.append("f")
            payload.append(struct.pack(">f", float(v)))
    return osc_string(address) + osc_string("," + "".join(tags)) + b"".join(payload)


def bvh_fps(bvh_path):
    with open(bvh_path) as f:
        for line in f:
            if "Frame Time" in line:
                frame_time = float(line.split()[-1])
                if frame_time > 0:
                    return 1.0 / frame_time
    return 60.0


class BvhStream:
    """Turns BVH frames into MOVIN /MOVIN/Frame OSC packets."""

    def __init__(self, bvh_path, chunk=16, root_bone=None, actor="MOVINMan"):
        self.bvh = read_bvh(bvh_path)
        self.fps = bvh_fps(bvh_path)
        # MOVIN Studio streams metres; BVH exports may be in centimetres.
        self.scale = 0.01 if np.mean(np.abs(self.bvh.pos[:, 0, 1])) > 10.0 else 1.0
        self.is_v3 = bool(V3_MARKER_BONES & set(self.bvh.bones))
        self.root_bone = root_bone or ("RootBone" if self.is_v3 else "Root")
        self.chunk = chunk
        self.actor = actor

    @property
    def num_frames(self):
        return self.bvh.quats.shape[0]

    def packets(self, frame_idx):
        """OSC packets for one frame; ``frame_idx`` wraps around the clip."""
        bvh, f = self.bvh, frame_idx % self.num_frames
        bones = [(0, -1, self.root_bone, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))]
        for j, name in enumerate(bvh.bones):
            p = (bvh.pos[f, j] if j == 0 else bvh.offsets[j]) * self.scale
            w, x, y, z = bvh.quats[f, j]
            bones.append((j + 1, int(bvh.parents[j]) + 1, name,
                          (-p[0], p[1], p[2]), (x, -y, -z, w)))
        chunks = [bones[i:i + self.chunk] for i in range(0, len(bones), self.chunk)]
        packets = []
        for ci, chunk in enumerate(chunks):
            msg = ["12:00:00", self.actor, frame_idx, len(chunks), ci, len(bones), len(chunk)]
            for idx, parent, name, p, q in chunk:
                msg += [idx, parent, name, *map(float, p),
                        0.0, 0.0, 0.0, 1.0,  # rest rotation: identity
                        *map(float, q),
                        1.0, 1.0, 1.0]       # scale
            packets.append(osc_message("/MOVIN/Frame", msg))
        return packets


def stream_bvh(bvh_path, host="127.0.0.1", port=11235, seconds=None, chunk=16,
               root_bone=None, actor="MOVINMan", stop_event=None, verbose=True):
    """Send the BVH at its own frame rate, looping, until ``seconds`` elapse,
    ``stop_event`` is set or the process is interrupted. Returns frames sent."""
    stream = BvhStream(bvh_path, chunk=chunk, root_bone=root_bone, actor=actor)
    if verbose:
        print(f"[fake-studio] {bvh_path}: {len(stream.bvh.bones)} bones, "
              f"{stream.num_frames} frames @ {stream.fps:.1f} fps, "
              f"preset {'movinman_v3' if stream.is_v3 else 'movinman'}, "
              f"root '{stream.root_bone}' -> {host}:{port}", flush=True)

    sent = 0
    t0 = time.monotonic()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while seconds is None or time.monotonic() - t0 < seconds:
            if stop_event is not None and stop_event.is_set():
                break
            frame_idx = int((time.monotonic() - t0) * stream.fps)
            for packet in stream.packets(frame_idx):
                sock.sendto(packet, (host, port))
            sent += 1
            time.sleep(max(0.0, t0 + (frame_idx + 1) / stream.fps - time.monotonic()))
    if verbose:
        print(f"[fake-studio] sent {sent} frames in {time.monotonic() - t0:.1f}s", flush=True)
    return sent


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bvh", help="BVH file to stream (MOVINMan or MOVINManV3)")
    parser.add_argument("--host", default="127.0.0.1", help="destination host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=11235, help="destination UDP port (default: 11235)")
    parser.add_argument("--seconds", type=float, default=None,
                        help="stop after this many seconds (default: run until Ctrl+C)")
    parser.add_argument("--chunk", type=int, default=16, help="bones per OSC packet (default: 16)")
    parser.add_argument("--root_bone", default=None,
                        help="stream root bone name (default: Root, or RootBone for MOVINManV3)")
    parser.add_argument("--actor", default="MOVINMan", help="actor name in the stream")
    args = parser.parse_args()
    try:
        stream_bvh(args.bvh, host=args.host, port=args.port, seconds=args.seconds,
                   chunk=args.chunk, root_bone=args.root_bone, actor=args.actor)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
