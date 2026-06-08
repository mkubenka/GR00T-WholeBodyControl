#!/usr/bin/env python3
"""Replay a recorded PICO teleop session over ZMQ.

`pico_manager_thread_server.py --record <dir>` saves one `pose_NNNNNN.npz` per
sent message (the exact `numpy_data` pose dict). This tool republishes those
files over ZMQ as `pose` messages, so you can:

  * capture a session ONCE on the Ubuntu box (with the headset), then iterate on
    retargeting / visualization anywhere — including a Mac — with no hardware;
  * regression-test downstream consumers against a real recorded stream.

    python teleop_sim/replay_recording.py <record_dir_or_npz> [--loop] [--fps 50]

Then view it:  python teleop_sim/zmq_pose_viz.py --host localhost --port 5555
"""

from __future__ import annotations

import argparse
import glob
import os
import time

import numpy as np
import zmq

import importlib.util

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SENDER = os.path.join(_REPO_ROOT, "gear_sonic", "utils", "teleop", "zmq", "zmq_planner_sender.py")


def _load_pack_pose_message():
    spec = importlib.util.spec_from_file_location("_zmq_sender", _SENDER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.pack_pose_message


def _gather(path: str) -> list[str]:
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "pose_*.npz")))
        if not files:
            files = sorted(glob.glob(os.path.join(path, "*.npz")))
        return files
    return [path]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("recording", help="directory of pose_*.npz, or a single .npz")
    p.add_argument("--port", type=int, default=5555)
    p.add_argument("--topic", default="pose")
    p.add_argument("--fps", type=float, default=50.0)
    p.add_argument("--loop", action="store_true")
    args = p.parse_args()

    files = _gather(args.recording)
    if not files:
        raise SystemExit(f"no .npz recordings found at {args.recording}")
    print(f"[replay] {len(files)} message(s) from {args.recording}")

    pack = _load_pack_pose_message()
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{args.port}")
    print(f"[replay] publishing '{args.topic}' on tcp://*:{args.port} (Ctrl-C to stop)")
    time.sleep(0.3)

    try:
        while True:
            for f in files:
                with np.load(f, allow_pickle=False) as npz:
                    msg = {k: npz[k] for k in npz.files}
                sock.send(pack(msg, topic=args.topic))
                time.sleep(1.0 / args.fps)
            if not args.loop:
                break
    except KeyboardInterrupt:
        print("\n[replay] bye")
    finally:
        sock.close(0)
    print("[replay] done")


if __name__ == "__main__":
    main()
