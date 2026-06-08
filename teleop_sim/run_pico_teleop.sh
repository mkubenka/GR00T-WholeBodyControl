#!/usr/bin/env bash
# run_pico_teleop.sh — start the PICO full-body teleop server (Ubuntu x86_64).
#
# Prereqs (one-time):
#   1. bash install_scripts/install_pico.sh        # creates .venv_teleop + XRoboToolkit SDK
#   2. Start the XRoboToolkit PC service on this machine.
#   3. PICO headset on the same WiFi, running the XRoboToolkit PICO service app,
#      body tracking calibrated (feet + waist Motion Trackers for high accuracy).
#
# Then:  bash teleop_sim/run_pico_teleop.sh
# It publishes the ZMQ `pose` stream (default port 5555). Watch it from anywhere
# on the LAN (incl. a Mac):  python teleop_sim/zmq_pose_viz.py --host <this-ip>
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [ "$(uname -s)" != "Linux" ]; then
    echo "[!] The PICO ingestion (XRoboToolkit SDK) is Linux x86_64 only."
    echo "    Run this on your Ubuntu box; use the Mac only for zmq_pose_viz.py."
    exit 1
fi

if [ ! -d .venv_teleop ]; then
    echo "[!] .venv_teleop not found. Run:  bash install_scripts/install_pico.sh"
    exit 1
fi

# shellcheck disable=SC1091
source .venv_teleop/bin/activate

if ! python -c "import xrobotoolkit_sdk" 2>/dev/null; then
    echo "[!] xrobotoolkit_sdk not importable in .venv_teleop."
    echo "    Re-run install_scripts/install_pico.sh (it builds the SDK)."
    exit 1
fi

echo "[OK] .venv_teleop + xrobotoolkit_sdk ready."
echo "[i]  Make sure the XRoboToolkit PC service is running and the PICO is connected."
echo "[i]  Streaming pose over ZMQ — view with: python teleop_sim/zmq_pose_viz.py --host <ip>"
exec python gear_sonic/scripts/pico_manager_thread_server.py --manager "$@"
