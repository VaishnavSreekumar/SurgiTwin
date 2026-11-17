#!/usr/bin/env bash
# demo_phase3.sh: SurgiTwin Phase 3 Live Demonstration Script
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting SurgiTwin Phase 3 Live Demonstration..."
python3 demo_phase3.py
echo "Phase 3 Demonstration finished successfully."

