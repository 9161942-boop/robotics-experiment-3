#!/usr/bin/env bash
set -e

WS="/home/hcl/桌面/ros2_ws"
cd "$WS"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Clean up only processes belonging to this simulation before starting.
pkill -f 'gzserver.*/mecharm_fixed_pick_sim' 2>/dev/null || true
pkill -f 'gzclient.*mecharm_fixed_pick_sim' 2>/dev/null || true
sleep 2

# Start Gazebo in a separate terminal, paused at the manual-control stage.
gnome-terminal -- bash -lc "cd '$WS'; source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; ros2 launch mecharm_fixed_pick_sim fixed_point_experiment.launch.py contact_grasp:=true run_task:=false; exec bash"
sleep 10

# Open the control panel in another terminal.
gnome-terminal -- bash -lc "cd '$WS'; source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; ros2 run mecharm_fixed_pick_sim manual_control_panel; exec bash"
