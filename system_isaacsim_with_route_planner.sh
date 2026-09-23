#!/bin/bash
# Isaac Sim variant of system_simulation_with_route_planner.sh.
#
# Starts lidargen's go2_explore.py (Isaac Sim + the trained Go2 locomotion
# policy + the ROS2 bridge, in the env_isaaclab3 conda env) in place of the
# Unity binary, then launches the trimmed ROS2 stack (in the ros2_humble
# conda env) via system_isaacsim_with_route_planner.launch. See
# ~/.claude/plans/scalable-soaring-yeti.md, Phase 3.
#
# One-time setup, before first use:
#   conda run -n env_isaaclab3 pip install -e /home/ip/repos/research/lidargen
#
# Usage:
#   ./system_isaacsim_with_route_planner.sh --checkpoint /path/to/model.pt [go2_explore.py args...]

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

CONDA_BASE="$(conda info --base)"
source "$CONDA_BASE/etc/profile.d/conda.sh"

# Both processes must agree on the RMW implementation to discover each other.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

conda activate env_isaaclab3
python -m lidargen.envs.go2_explore --headless "$@" &
GO2_EXPLORE_PID=$!
conda deactivate

trap 'kill "$GO2_EXPLORE_PID" 2>/dev/null || true' EXIT

conda activate ros2_humble
source ./install/setup.bash
ros2 launch vehicle_simulator system_isaacsim_with_route_planner.launch
