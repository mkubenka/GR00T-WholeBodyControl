# `teleop_sim` — sim-first VR teleop visualizer

A lightweight harness for bringing up **PICO full-body VR teleop** of the Unitree
G1, *sim-first*, without needing the full IsaacLab / on-robot stack to see whether
your tracker stream is good.

It subscribes to the same **ZMQ `pose` stream** that the upstream PICO teleop
server (`gear_sonic/scripts/pico_manager_thread_server.py`) and the C++ on-robot
policy use, and renders the **retargeted G1 reference** — i.e. exactly the target
the policy would track — in MuJoCo.

## Why this exists

The repo already ships full-body PICO teleop via the **XRoboToolkit SDK**: it
reads the headset's 24-joint body tracking (head, hands, **waist, feet**),
retargets to the G1, and publishes target poses over ZMQ. What was missing was an
easy way to (a) verify your tracker pose stream end-to-end and (b) do it from a
Mac. This adds that.

> **WebXR can't see your trackers.** A browser (PICO or Quest) only exposes head
> + 2 controllers + hands over WebXR — never the feet/waist Motion Trackers. So
> full-body PICO teleop *must* go through the native XRoboToolkit path, not a
> browser. That's the path this harness targets.

## Where each piece runs

| Component | Machine | Notes |
|---|---|---|
| PICO ingestion + retarget (`pico_manager_thread_server.py`, XRoboToolkit SDK) | **Ubuntu x86_64** | SDK ships Linux-only native libs (`libPXREARobotSDK.so`); CPU-only, no CUDA needed. Connects to the headset over WiFi. |
| **This visualizer** (`zmq_pose_viz.py`) | **macOS *or* Linux** | Pure `mujoco`+`pyzmq`+`numpy`. ZMQ carries the stream over your LAN, so it can watch the Ubuntu box from a Mac. |
| Full SONIC policy sim / RL (IsaacLab) | Ubuntu + NVIDIA CUDA | Separate; not needed to use this harness. |

## Not connected to the Cloudflare web demo

The browser demo (e.g. `alza-demo.robotkit.dev`) is **client-side WASM** and
cannot receive ZMQ (browsers don't speak ZMQ; Workers don't run a ZMQ server).
This teleop path is **local/native**: PICO → teleop server → ZMQ over your LAN →
this visualizer / sim / real G1. Teleoperating the *browser* demo specifically
would need a separate WebSocket bridge (optionally a Cloudflare Durable Object) —
out of scope here.

## Quick self-test (no headset, no Ubuntu)

Proves the decoder + renderer work against the real wire format:

```bash
pip install -r teleop_sim/requirements.txt
git lfs pull --include='gear_sonic_deploy/g1/meshes/*'   # one-time: real STL meshes

# terminal 1 — synthetic squat/arm-raise stream on tcp://*:5555
python teleop_sim/fake_pose_publisher.py

# terminal 2 — render a frame to PNG (works headless / on CI)
python teleop_sim/zmq_pose_viz.py --headless --save-frame /tmp/g1.png
```

Interactive window instead of a PNG:

```bash
# Linux
python  teleop_sim/zmq_pose_viz.py
# macOS — the MuJoCo passive viewer must run under mjpython:
mjpython teleop_sim/zmq_pose_viz.py
```

## Live PICO teleop

1. **Ubuntu** — set up the teleop venv and the XRoboToolkit SDK:
   ```bash
   bash install_scripts/install_pico.sh           # creates .venv_teleop
   ```
   Start the XRoboToolkit **PC service**, put on the PICO (running the
   XRoboToolkit PICO service app, on the same WiFi), then run the teleop server:
   ```bash
   source .venv_teleop/bin/activate
   python gear_sonic/scripts/pico_manager_thread_server.py --manager
   ```
   It publishes the `pose` stream over ZMQ (default port 5555).

2. **Mac or Ubuntu** — watch the retargeted G1 reference:
   ```bash
   python teleop_sim/zmq_pose_viz.py --host <ubuntu-ip> --port 5555
   ```
   Move around — head/hands/waist/feet should drive the rendered G1. This is the
   same target the policy tracks, so if it looks right here, it's right for the
   policy.

## The `pose` wire format (for reference)

`[topic]["pose"][1280-byte JSON header][little-endian payload]`. The JSON header
lists `{name, dtype, shape}` per field; the payload is each field's raw bytes
concatenated in order. Producer: `gear_sonic/utils/teleop/zmq/zmq_planner_sender.py`
(`pack_pose_message`). Fields this viewer uses: `joint_pos[N,29]` (ISAACLAB joint
order), `body_quat_w[N,B,4]` (wxyz, body 0 = pelvis). No absolute root translation
is streamed (the policy tracks relative), so the viewer pins a standing pelvis
height (`--root-height`).

## Files
- `zmq_pose_viz.py` — ZMQ `pose` subscriber → MuJoCo G1 renderer (interactive or headless PNG).
- `fake_pose_publisher.py` — synthetic `pose` stream for testing without hardware.
- `replay_recording.py` — republish a recorded session (`--record` `pose_*.npz`) over ZMQ.
- `run_pico_teleop.sh` — launcher + preflight for the Ubuntu PICO server.
- `requirements.txt` — viewer deps (macOS/Linux).
- `HANDOFF.md` — full runbook for continuing on the Linux box.
