#!/bin/bash
# ROS2 side of the Isaac Sim setup (replaces the Unity binary + vehicle_simulator).
#
# Run order, two terminals:
#   1) Isaac Sim side (lidargen conda env):
#        conda activate lidargen
#        run-env go2_explore --headless        # or --remote to stream the viewport
#      It publishes /state_estimation (Odometry, map->sensor), TF, and
#      /registered_scan (PointCloud2, map frame), and subscribes /cmd_vel
#      (TwistStamped) -- the same contract the Unity simulator provided.
#   2) This script (launches local_planner, terrain_analysis(_ext),
#      sensor_scan_generation, far_planner, visualization_tools).
#
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp is REQUIRED on every participant:
# Isaac Sim's bundled fastdds and RoboStack's fastdds do not discover each
# other. Any extra shell (rviz2, ros2 topic ...) needs the same export.

# No `set -u`: RoboStack's conda activate scripts reference unbound variables.
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

CONDA_BASE="$(conda info --base)"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate ros2_humble

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source ./install/setup.bash
ros2 launch vehicle_simulator system_isaacsim_with_route_planner.launch
