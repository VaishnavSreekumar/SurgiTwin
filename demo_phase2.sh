#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /home/vaish123/robotics/SurgiTwin/install/setup.bash

echo "Starting SurgiTwin Phase 2 Vision-to-Safety Demonstration..."
python3 /home/vaish123/robotics/SurgiTwin/demo_phase2.py

