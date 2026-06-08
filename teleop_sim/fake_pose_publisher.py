#!/usr/bin/env python3
"""Publish a synthetic SONIC "pose" ZMQ stream — for testing zmq_pose_viz.py
(and any downstream consumer) without a PICO headset or the teleop server.

It emits the exact wire format the real PICO server uses. Where possible it
reuses the repo's own `pack_pose_message` (loaded directly from
gear_sonic/utils/teleop/zmq/zmq_planner_sender.py, which only needs
json/struct/numpy), so this doubles as a contract test of the encoder/decoder.

The synthetic motion is a gentle squat + arm-raise so the rendered G1 is
visibly posed (joints are in ISAACLAB order, matching the real stream).

    python teleop_sim/fake_pose_publisher.py            # bind tcp://*:5555
    python teleop_sim/fake_pose_publisher.py --once      # send a single frame and exit
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import time

import numpy as np
import zmq

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SENDER = os.path.join(_REPO_ROOT, "gear_sonic", "utils", "teleop", "zmq", "zmq_planner_sender.py")

# ISAACLAB 0-based joint indices we animate (see gear_sonic_deploy g1_29dof actuator order).
_L_HIP_PITCH, _L_KNEE, _L_ANKLE_PITCH = 0, 3, 4
_R_HIP_PITCH, _R_KNEE, _R_ANKLE_PITCH = 6, 9, 10
_L_SHOULDER_PITCH, _L_ELBOW = 15, 18
_R_SHOULDER_PITCH, _R_ELBOW = 22, 25


def _load_pack_pose_message():
    """Load the repo's real pack_pose_message by file path (no package import chain)."""
    try:
        spec = importlib.util.spec_from_file_location("_zmq_sender", _SENDER)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.pack_pose_message
    except Exception as exc:  # fall back to a local mirror
        print(f"[pub] using built-in packer ({exc})")
        return _pack_pose_message_mirror


def _pack_pose_message_mirror(pose_data: dict, topic: str = "pose", version: int = 3) -> bytes:
    import json
    HEADER_SIZE = 1280
    _dtype = {np.float32: "f32", np.float64: "f64", np.int32: "i32", np.int64: "i64", np.bool_: "bool"}
    fields, blobs = [], []
    for name, val in pose_data.items():
        val = np.ascontiguousarray(val)
        fields.append({"name": name, "dtype": _dtype.get(val.dtype.type, "f32"),
                       "shape": list(val.shape)})
        if _dtype.get(val.dtype.type, "f32") == "f32" and val.dtype != np.float32:
            val = val.astype(np.float32)
        blobs.append(val.tobytes())
    header = json.dumps({"v": version, "endian": "le", "count": 1, "fields": fields},
                        separators=(",", ":")).encode().ljust(HEADER_SIZE, b"\x00")
    return topic.encode() + header + b"".join(blobs)


def synth_joint_pos(t: float) -> np.ndarray:
    """29-DoF target (ISAACLAB order): squat + arm raise oscillation."""
    jp = np.zeros(29, dtype=np.float32)
    s = 0.5 * (1.0 - math.cos(t))  # 0..1
    # squat
    jp[_L_HIP_PITCH] = jp[_R_HIP_PITCH] = -0.9 * s
    jp[_L_KNEE] = jp[_R_KNEE] = 1.6 * s
    jp[_L_ANKLE_PITCH] = jp[_R_ANKLE_PITCH] = -0.7 * s
    # arm raise + elbow bend
    jp[_L_SHOULDER_PITCH] = jp[_R_SHOULDER_PITCH] = -1.4 * s
    jp[_L_ELBOW] = jp[_R_ELBOW] = 0.9 * s
    return jp


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=5555)
    p.add_argument("--topic", default="pose")
    p.add_argument("--fps", type=float, default=50.0)
    p.add_argument("--period", type=float, default=3.0, help="seconds per squat cycle")
    p.add_argument("--once", action="store_true", help="send a single mid-squat frame and exit")
    args = p.parse_args()

    pack = _load_pack_pose_message()
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{args.port}")
    print(f"[pub] publishing '{args.topic}' on tcp://*:{args.port} (Ctrl-C to stop)")
    time.sleep(0.3)  # let subscribers connect before the first send

    step = 0
    t0 = time.monotonic() if not args.once else None
    try:
        while True:
            t = math.pi if args.once else (time.monotonic() - t0) * (2 * math.pi / args.period)
            jp = synth_joint_pos(t)
            msg = {
                "joint_pos": jp.reshape(1, 29).astype(np.float32),
                "joint_vel": np.zeros((1, 29), dtype=np.float32),
                "body_quat_w": np.array([[[1.0, 0.0, 0.0, 0.0]]], dtype=np.float32),  # [1,1,4] wxyz
                "frame_index": np.array([step], dtype=np.int64),
            }
            sock.send(pack(msg, topic=args.topic))
            step += 1
            if args.once:
                print("[pub] sent one frame")
                break
            time.sleep(1.0 / args.fps)
    except KeyboardInterrupt:
        print("\n[pub] bye")
    finally:
        sock.close(0)


if __name__ == "__main__":
    main()
