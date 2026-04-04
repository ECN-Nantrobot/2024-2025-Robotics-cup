#!/bin/bash

# Kill any running instance of the launch file
echo "Stopping existing launch..."
pkill -f 'ros2 launch robonav launch.launch.py'

# Wait a moment for clean shutdown
sleep 2

# Start the launch file again
echo "Restarting launch file..."
ros2 launch robonav launch.launch.py
