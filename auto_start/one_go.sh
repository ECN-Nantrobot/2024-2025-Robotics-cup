# !/bin/bash

# This script is used to start the ROS2 nodes and the simulation environment
# in one go. It is intended to be run on startup.
# It will source the ROS2 environment and then launch the simulation.
# Make sure to give this script execute permissions:
# chmod +x one_go.sh

# set installation directory
ENVFILE=/home/pi/2024-2025-Robotics-cup/install/setup.bash

# execute the script in the background
if [ -f ${ENVFILE} ]; then
    echo "Loading ROS2 Env..."
    # source the ROS2 environment
    source /opt/ros/humble/setup.bash
    source /home/pi/2024-2025-Robotics-cup/install/setup.bash
    echo "ROS2 Launching..."
    # launch the nodes
    exec ros2 launch robonav launch_sim.launch.py use_sim_time:=faulse
else
    echo "There is no ${ENVFILE}"
fi
