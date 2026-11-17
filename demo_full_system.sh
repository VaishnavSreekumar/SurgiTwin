#!/usr/bin/env bash
# demo_full_system.sh: Master End-to-End System Demonstration Script
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting SurgiTwin Master Full-System End-to-End Demonstration..."
python3 demo_full_system.py
echo "Master Demonstration complete."

