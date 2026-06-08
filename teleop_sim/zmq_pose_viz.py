#!/usr/bin/env python3
"""Lightweight ZMQ -> MuJoCo visualizer for the SONIC teleop pose stream.

The PICO teleop server (gear_sonic/scripts/pico_manager_thread_server.py) and the
C++ on-robot policy speak a ZMQ "pose" protocol:

    [topic bytes]["pose"][1280-byte JSON header][packed little-endian payload]

The JSON header lists the fields ({name, dtype, shape}); the payload is each
field's raw bytes concatenated in order (see
gear_sonic/utils/teleop/zmq/zmq_planner_sender.py:pack_pose_message).

This tool SUBSCRIBES to that stream and renders the *retargeted G1 reference*
(the exact target the policy would track) in MuJoCo — so you can verify your
PICO trackers end-to-end *without* the full IsaacLab / robot stack.

It needs only `mujoco`, `pyzmq`, `numpy` — all of which have native macOS
(Apple-Silicon) wheels, so this runs on a Mac even though the PICO ingestion
server itself is Linux-only. Point --host at the machine running the teleop
server; ZMQ carries the stream over your LAN.

The streamed fields used here:
  joint_pos    [N, 29]      29 DoF targets, ISAACLAB joint order
  body_quat_w  [N, B, 4]    per-body world orientation (wxyz); body 0 = pelvis/root
(no absolute root translation is streamed — the policy tracks relative — so we
pin a standing root height for visualization.)

Usage (Linux or macOS, headless render):
    python teleop_sim/zmq_pose_viz.py --headless --save-frame /tmp/g1.png

Usage (interactive window):
    # macOS requires the viewer to run under mjpython:
    mjpython teleop_sim/zmq_pose_viz.py --host 192.168.1.50
    # Linux:
    python  teleop_sim/zmq_pose_viz.py --host 192.168.1.50
"""

from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

try:
    import zmq
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyzmq is required: pip install pyzmq") from exc

try:
    import mujoco
except ImportError as exc:  # pragma: no cover
    raise SystemExit("mujoco is required: pip install mujoco") from exc

# Wire-format constants — must match zmq_planner_sender.py.
HEADER_SIZE = 1280
DEFAULT_TOPIC = "pose"

# Maps ISAACLAB joint order (what the stream carries) -> MuJoCo qpos joint order
# (what g1_29dof.xml expects). Lifted verbatim from
# gear_sonic_deploy/visualize_motion.py so the two stay in lockstep.
ISAACLAB_TO_MUJOCO = [
    0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8,
    11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28,
]

# numpy dtypes for the header dtype strings used by pack_pose_message.
_DTYPES = {
    "f32": "<f4", "f64": "<f8",
    "i32": "<i4", "i64": "<i8",
    "u8": "u1", "bool": "?",
}

# Default model, relative to repo root.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MODEL = os.path.join(_REPO_ROOT, "gear_sonic_deploy", "g1", "g1_29dof.xml")


def decode_pose_message(data: bytes, topic: str = DEFAULT_TOPIC) -> dict:
    """Decode one raw ZMQ pose message into {field_name: np.ndarray}."""
    body = data[len(topic):] if data.startswith(topic.encode()) else data
    header_json = body[:HEADER_SIZE].rstrip(b"\x00")
    header = json.loads(header_json)
    payload = body[HEADER_SIZE:]

    out: dict[str, np.ndarray] = {}
    offset = 0
    for field in header["fields"]:
        dtype = np.dtype(_DTYPES[field["dtype"]])
        shape = tuple(field["shape"])
        count = int(np.prod(shape)) if shape else 1
        nbytes = count * dtype.itemsize
        arr = np.frombuffer(payload[offset:offset + nbytes], dtype=dtype).reshape(shape)
        out[field["name"]] = arr
        offset += nbytes
    return out


def pose_to_qpos(msg: dict, root_height: float) -> np.ndarray | None:
    """Build a 36-D G1 qpos (free base + 29 DoF) from a decoded pose message.

    Uses the most recent frame in the buffered message.
    """
    if "joint_pos" not in msg:
        return None
    joint_pos = np.asarray(msg["joint_pos"], dtype=np.float64)
    if joint_pos.ndim == 1:
        joint_pos = joint_pos.reshape(1, -1)
    last = joint_pos[-1]  # [29] isaaclab order
    if last.shape[0] != 29:
        return None
    dof = last[ISAACLAB_TO_MUJOCO]

    # Root orientation: pelvis is body 0 of body_quat_w (wxyz, world). Fall back
    # to upright if absent.
    root_quat = np.array([1.0, 0.0, 0.0, 0.0])
    bq = msg.get("body_quat_w", msg.get("body_quat"))
    if bq is not None:
        bq = np.asarray(bq, dtype=np.float64)
        frame = bq[-1] if bq.ndim == 3 else bq
        root_quat = frame[0] if frame.ndim == 2 else frame
        n = np.linalg.norm(root_quat)
        if n > 1e-6:
            root_quat = root_quat / n
        else:
            root_quat = np.array([1.0, 0.0, 0.0, 0.0])

    qpos = np.zeros(36)
    qpos[:3] = [0.0, 0.0, root_height]
    qpos[3:7] = root_quat
    qpos[7:36] = dof
    return qpos


class PoseSubscriber:
    """Non-blocking SUB socket that keeps only the latest pose message."""

    def __init__(self, host: str, port: int, topic: str = DEFAULT_TOPIC):
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, topic)
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.connect(f"tcp://{host}:{port}")
        self._topic = topic

    def latest(self) -> dict | None:
        if self._sock.poll(timeout=0):
            raw = self._sock.recv(zmq.NOBLOCK)
            return decode_pose_message(raw, self._topic)
        return None

    def close(self):
        self._sock.close(0)


def run_interactive(model, data, sub, args):
    import mujoco.viewer  # imported lazily; needs a display (mjpython on macOS)

    print(f"[viz] subscribed to tcp://{args.host}:{args.port} topic '{args.topic}'")
    print("[viz] waiting for pose messages… (Ctrl-C to quit)")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            msg = sub.latest()
            if msg is not None:
                qpos = pose_to_qpos(msg, args.root_height)
                if qpos is not None:
                    data.qpos[:36] = qpos
                    mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1.0 / args.fps)


def run_headless(model, data, sub, args):
    """Render until a pose arrives (or timeout), save one frame. For CI / no-display."""
    deadline = time.time() + args.wait
    got = False
    while time.time() < deadline:
        msg = sub.latest()
        if msg is not None:
            qpos = pose_to_qpos(msg, args.root_height)
            if qpos is not None:
                data.qpos[:36] = qpos
                got = True
                break
        time.sleep(0.02)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    renderer.update_scene(data, camera=-1)
    pixels = renderer.render()
    _save_png(pixels, args.save_frame)
    print(f"[viz] {'rendered streamed pose' if got else 'no pose received — rendered default pose'} "
          f"-> {args.save_frame}")
    return 0 if got or args.allow_empty else 2


def _save_png(pixels: np.ndarray, path: str):
    try:
        from PIL import Image
        Image.fromarray(pixels).save(path)
    except ImportError:
        # Minimal PNG writer fallback so PIL isn't a hard dependency.
        import struct, zlib
        h, w, _ = pixels.shape
        raw = b"".join(b"\x00" + pixels[y].tobytes() for y in range(h))
        def chunk(tag, body):
            return (struct.pack(">I", len(body)) + tag + body
                    + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF))
        png = (b"\x89PNG\r\n\x1a\n"
               + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
               + chunk(b"IDAT", zlib.compress(raw, 9))
               + chunk(b"IEND", b""))
        with open(path, "wb") as f:
            f.write(png)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="localhost", help="teleop server host (default: localhost)")
    p.add_argument("--port", type=int, default=5555, help="ZMQ PUB port (default: 5555)")
    p.add_argument("--topic", default=DEFAULT_TOPIC)
    p.add_argument("--model", default=DEFAULT_MODEL, help="G1 MJCF path")
    p.add_argument("--root-height", type=float, default=0.793, help="pinned pelvis height (m)")
    p.add_argument("--fps", type=float, default=50.0, help="render/poll rate")
    p.add_argument("--headless", action="store_true", help="render one frame to PNG and exit")
    p.add_argument("--save-frame", default="/tmp/g1_teleop.png")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--wait", type=float, default=10.0, help="headless: seconds to wait for a pose")
    p.add_argument("--allow-empty", action="store_true", help="headless: exit 0 even with no pose")
    args = p.parse_args()

    if not os.path.exists(args.model):
        raise SystemExit(
            f"model not found: {args.model}\n"
            "Pull the G1 meshes first:  git lfs pull --include='gear_sonic_deploy/g1/meshes/*'"
        )

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    sub = PoseSubscriber(args.host, args.port, args.topic)
    try:
        if args.headless:
            raise SystemExit(run_headless(model, data, sub, args))
        run_interactive(model, data, sub, args)
    except KeyboardInterrupt:
        print("\n[viz] bye")
    finally:
        sub.close()


if __name__ == "__main__":
    main()
