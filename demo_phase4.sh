#!/usr/bin/env bash
# demo_phase4.sh: SurgiTwin Phase 4 Live Demonstration Script
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting SurgiTwin Phase 4 Live Demonstration..."
python3 demo_phase4.py
echo "Phase 4 Demonstration finished successfully."

