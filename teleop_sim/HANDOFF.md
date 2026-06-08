# Teleop handoff — continue on the Linux box

_Last updated: 2026-06-08. Written on macOS; the next session runs on the Ubuntu +
NVIDIA (CUDA) laptop._

## Goal

Add a **VR teleop option** to GR00T-WholeBodyControl (SONIC) for a Unitree G1.
Hardware: **PICO 4 (preferred — has feet + waist/chest Motion Trackers)** and a
Meta Quest. Chosen direction (locked):

- **Path:** harden the **existing PICO full-body path** (XRoboToolkit SDK), not a
  browser/WebXR option.
- **Target:** **sim-first** (MuJoCo), structured so the same ZMQ stream can later
  drive the real G1.

## The 3 findings that shaped this

1. **Upstream already ships full-body PICO teleop.**
   `gear_sonic/scripts/pico_manager_thread_server.py` reads PICO's 24-joint body
   tracking via the **XRoboToolkit SDK** (`xrobotoolkit_sdk` / `xrt`), retargets
   to the G1, and publishes target poses over **ZMQ** (`pose` topic). The robot
   side already consumes this. So "add teleop" = enable/verify this path + give it
   a sim-first viewer, not build retargeting from scratch.
2. **WebXR cannot see the PICO trackers.** Any browser (PICO or Quest) exposes
   only head + 2 controllers + hands over WebXR — never the feet/waist trackers.
   Full-body PICO teleop therefore **must** use the native XRoboToolkit path.
3. **ZMQ ≠ the Cloudflare web demo.** `alza-demo.robotkit.dev` is client-side
   WASM; it cannot receive ZMQ. Teleop is a local/native LAN path:
   `PICO → teleop server → ZMQ → sim/viz/real G1`. (Teleoperating the browser demo
   would need a separate WebSocket bridge — out of scope.)

## Where things run (machine split)

| Component | Machine | Notes |
|---|---|---|
| PICO ingestion + retarget (`pico_manager_thread_server.py`, XRoboToolkit SDK) | **Ubuntu x86_64** (this box) | SDK is Linux-only (`libPXREARobotSDK.so`); CPU-only. |
| ZMQ→MuJoCo visualizer (`teleop_sim/zmq_pose_viz.py`) | Mac **or** Linux | Pure mujoco+pyzmq+numpy. |
| Full SONIC policy sim / RL (IsaacLab) | Ubuntu + CUDA | The milestone after the viewer. |

## What's DONE (committed + pushed)

- Forked → **`github.com/mkubenka/GR00T-WholeBodyControl`**.
- Branch **`feat/teleop-sim-viz`** (pushed) adds `teleop_sim/`:
  - `zmq_pose_viz.py` — subscribes to the ZMQ `pose` stream, decodes the wire
    format, remaps 29 DoF ISAACLAB→MuJoCo, renders the retargeted G1 in MuJoCo.
    Interactive (mjpython on macOS / plain python on Linux) or `--headless` PNG.
  - `fake_pose_publisher.py` — synthetic `pose` stream (reuses the repo's real
    `pack_pose_message`) to test the viewer without a headset.
  - `replay_recording.py` — republishes a recorded session
    (`pico_manager_thread_server.py --record <dir>` → `pose_*.npz`) over ZMQ, so
    you capture ONCE on the Ubuntu box and then iterate on retarget/viz anywhere
    (incl. the Mac) with no hardware. Tested here against the real field schema.
  - `run_pico_teleop.sh` — launcher + preflight checks for the Ubuntu PICO server.
  - `requirements.txt`, `README.md`.
- **Verified on macOS, headless:** synthetic stream → decode → MuJoCo render, and a
  round-trip that matched the repo's real encoder exactly. (Rendered a correctly
  posed G1: squat + arms raised.)

## NEXT STEPS on the Linux box (in order)

### 0. Get the code
```bash
git clone git@github.com:mkubenka/GR00T-WholeBodyControl.git
cd GR00T-WholeBodyControl
git checkout feat/teleop-sim-viz
git lfs pull --include='gear_sonic_deploy/g1/meshes/*'   # G1 meshes for the viewer
```

### 1. Smoke-test the viewer (no headset) — confirm the toolchain on Linux
```bash
python3 -m venv /tmp/teleop_venv && . /tmp/teleop_venv/bin/activate
pip install -r teleop_sim/requirements.txt
# terminal A:
python teleop_sim/fake_pose_publisher.py --port 5599
# terminal B (interactive window works directly on Linux — no mjpython):
python teleop_sim/zmq_pose_viz.py --host localhost --port 5599
# or headless: python teleop_sim/zmq_pose_viz.py --headless --host localhost --port 5599 --save-frame /tmp/g1.png
```
Expect a G1 doing a slow squat + arm-raise.

### 2. Install the real PICO teleop stack
```bash
bash install_scripts/install_pico.sh        # creates .venv_teleop (uv, py3.10) + builds XRoboToolkit SDK
. .venv_teleop/bin/activate
python -c "import xrobotoolkit_sdk; print('SDK OK')"
```
**Watch for install issues here** (this is the part that couldn't be tested on the
Mac — the SDK has Linux x86_64 prebuilt libs in
`external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64`). If the build
fails, that's the first thing to fix and fold back into the fork. The
`gear_sonic[teleop]` extra pulls `pyzmq, msgpack, pin` (pinocchio, for FK) and
`pyvista` (the VR 3-pt visualizer; skipped on aarch64) — `pin` is the most likely
wheel/build snag; on x86_64 Ubuntu it's a prebuilt wheel.

### 3. Bring up XRoboToolkit + the headset
- Start the **XRoboToolkit PC service** on the Ubuntu box (from
  `XR-Robotics/XRoboToolkit-PC-Service`; check `install_pico.sh` / repo docs for the
  exact binary/launch — verify how it's started).
- PICO 4 on the **same WiFi**, running the **XRoboToolkit PICO service app**;
  calibrate body tracking with the **feet + waist** Motion Trackers (high-accuracy
  24-joint mode).

### 4. Run teleop → watch in sim
```bash
bash teleop_sim/run_pico_teleop.sh           # = python gear_sonic/scripts/pico_manager_thread_server.py --manager
# elsewhere on the LAN (incl. the Mac):
python teleop_sim/zmq_pose_viz.py --host <ubuntu-ip> --port 5555
```
Move around — head/hands/waist/feet should drive the rendered G1. This is the exact
target the policy tracks, so "looks right here" == "right for the policy".

### 5. (Milestone) Close the loop into the policy sim
After the viewer confirms a good stream, drive the actual policy in sim so the G1
*balances* while following you (not just a kinematic pose).

**Architecture (confirmed by reading the code — read this before diving in):**
`run_sim_loop.py` is a **DDS-bridged MuJoCo sim**: `unitree_sdk2py_bridge.py`
makes the sim look like a real G1 over Unitree DDS, "so the WBC policy sees the
sim as a real robot." The ZMQ inside `gear_sonic/utils/mujoco_sim/` is **only for
camera images** (`sensor_server.py`, `image_publish_utils.py`, port 5555 camera /
5558 inference) — it is NOT the pose input. So the loop is:

    PICO → ZMQ `pose` → [policy process] → Unitree DDS → run_sim_loop (MuJoCo)

The pose stream feeds the **policy**, not the sim directly. Two candidate policy
paths to try (both need policy checkpoints via `git lfs pull` of `*.onnx`/`*.pt`,
the `unitree_sdk2py` DDS stack (CycloneDDS), and Linux+CUDA):
  - **gear_sonic_deploy** (the SONIC tracking policy): the C++ deploy
    (`gear_sonic_deploy/`, built via `deploy.sh`) reads the ZMQ `pose` and drives
    the robot; point it at the sim instead of the real robot (`--input-type zmq`).
  - **decoupled_wbc** (`gear_sonic/utils/mujoco_sim/configs.py`:
    `wbc_model_path="policy/stand.onnx,policy/walk.onnx"`,
    `wbc_policy_class="G1DecoupledWholeBodyPolicy"`): the WBC controller already
    wired into `run_sim_loop.py` over DDS.
- First decide which policy is the teleop target (SONIC tracking = the
  gear_sonic_deploy path), then wire `pico server → that policy → DDS sim`. This is
  the real "fully functional in sim" milestone and is Linux/CUDA/checkpoint-gated.

## Technical reference (so you don't re-derive it)

- **ZMQ `pose` wire format:** `[topic]["pose"][1280-byte JSON header][LE payload]`.
  Header = `{"v":ver,"endian":"le","count":1,"fields":[{name,dtype,shape},...]}`
  padded with `\x00`; payload = each field's raw bytes concatenated in order.
  Encoder: `gear_sonic/utils/teleop/zmq/zmq_planner_sender.py::pack_pose_message`.
  Subscriber helper: `gear_sonic/utils/teleop/zmq/zmq_poller.py::ZMQPoller`
  (uses `zmq.CONFLATE`, default `tcp://localhost:5555`).
- **Streamed fields** (from `pico_manager_thread_server.py` ~line 1438): `smpl_pose`,
  `smpl_joints`, `body_quat_w[N,B,4]` (wxyz, body 0 = pelvis), `joint_pos[N,29]`
  (ISAACLAB order), `joint_vel`, `vr_position`, `vr_orientation`, `frame_index`,
  triggers/grips, hand joints. **No absolute root translation** (policy is
  relative) — the viewer pins `--root-height` (default 0.793 m).
- **Joint remap** (`gear_sonic_deploy/visualize_motion.py`):
  `mujoco_dof = isaaclab_dof[ISAACLAB_TO_MUJOCO]`, where `ISAACLAB_TO_MUJOCO =
  [0,3,6,9,13,17,1,4,7,10,14,18,2,5,8,11,15,19,21,23,25,27,12,16,20,22,24,26,28]`.
- **MuJoCo qpos (free base + 29 DoF = 36):** `[:3]`=root pos, `[3:7]`=root quat
  (wxyz), `[7:36]`=29 DoF (MuJoCo order).
- **G1 model used by the viewer:** `gear_sonic_deploy/g1/g1_29dof.xml` (meshdir
  `meshes/`, needs LFS pull). Control freq = **50 Hz** (`base_env.yaml`:
  sim_dt 0.005 × decimation 4).
- **DoF order (ISAACLAB, 0-based):** 0-5 L-leg (hip pitch/roll/yaw, knee, ankle
  pitch/roll), 6-11 R-leg, 12-14 waist (yaw/roll/pitch), 15-21 L-arm
  (shoulder p/r/y, elbow, wrist r/p/y), 22-28 R-arm.

## Open questions to resolve on Linux
- Does `install_pico.sh` build cleanly on this Ubuntu? (Fold any fixes back.)
- Exact way to launch the **XRoboToolkit PC service** + headset pairing.
- `body_quat_w` shape `B` (how many bodies) as actually sent — the viewer handles
  `[N,4]` and `[N,B,4]`, root = index 0; confirm pelvis really is index 0.
- For sim closing-the-loop: does `run_sim_loop.py` accept ZMQ directly, or is ZMQ
  input only wired in the C++ `deploy.sh`? If the latter, may need a small adapter
  that feeds `ZMQPoller` output into the sim loop's reference.

## Repos & locations
- **Fork (work here):** `github.com/mkubenka/GR00T-WholeBodyControl`, branch
  `feat/teleop-sim-viz`. Upstream: `NVlabs/GR00T-WholeBodyControl`.
- Mac clone (reference only): `/Users/mkubenka/Projects/acheron/GR00T-WholeBodyControl`
  (shallow, LFS-skipped except g1 deploy meshes). `/tmp/teleop_venv` was the Mac
  test venv (don't transfer; recreate from `requirements.txt`).
- Git over **SSH** (`git@github.com:...`) — the gh HTTPS token lacks `workflow`
  scope, and SSH is the configured protocol anyway.
- Commit style (user rule): **no** Claude/AI co-author trailers or "Generated with"
  footers in commits/PRs.

## Broader project context (not teleop, but in case it comes up)
Separately in this session we built/deployed the **SONIC browser demo**:
- Repo `RobotKitAI/gear-sonic-demo` (local `…/alza/sonic/site`), deployed via a
  **Cloudflare Worker** at **alza-demo.robotkit.dev** (R2 holds the 2 big ONNX;
  `/api/*` is proxied to Modal with a server-side token; GH Actions auto-deploys on
  push to main, actions pinned to SHAs).
- Two Modal backends: Kimodo **text→motion** and **video→G1** (GVHMR→GMR), both
  token-auth'd, scale-to-zero. The browser demo is a separate world from teleop.

## Resume with Claude on Linux
Open this repo/branch and say something like: _"Read teleop_sim/HANDOFF.md and
continue the PICO teleop work — start at NEXT STEPS step 2 (install the real PICO
stack)."_
