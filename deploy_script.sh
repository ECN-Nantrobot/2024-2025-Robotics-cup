#!/bin/bash

# 1. Commit and Push Changes from Raspberry Pi
echo "Commit and Push Changes from Raspberry Pi ..."
ssh pi@10.192.58.70 << 'EOF'
cd ~/2024-2025-Robotics-cup/
git add .
git commit -m "Automatic commit for build and deploy"
git push origin ferdinand_rpi  # Or the branch you're using
EOF

# 2. Get the latest changes from the Raspberry Pi
echo "Get newest changes from Raspberry Pi..."
git pull origin ferdinand_rpi

# 3. Clean and Build the Workspace using colcon
echo "Clean and Build the Workspace..."
colcon build --symlink-install --packages-select robonav

# 4. Check if the symlink is correct and recreate it if necessary
echo "Check and fix the symlink for main_node..."
SYMLINK_PATH="/home/pi/2024-2025-Robotics-cup/install/robonav/lib/robonav/main_node"
BUILD_PATH="/home/pi/2024-2025-Robotics-cup/build/robonav/main_node"

# If the symlink does not exist or is broken, recreate it
if [ ! -L $SYMLINK_PATH ] || [ ! -e $SYMLINK_PATH ]; then
    echo "Symlink is broken or does not exist. Creating new symlink..."
    ln -sf $BUILD_PATH $SYMLINK_PATH
fi

# 5. Ensure the node is executable
echo "Ensure main_node is executable..."
chmod +x $SYMLINK_PATH

# 6. Copy compiled code to Raspberry Pi
echo "Copy compiled code to Raspberry Pi..."
scp -r ~/2024-2025-Robotics-cup/install pi@10.192.58.70:/home/pi/2024-2025-Robotics-cup/

# 7. Run the code on Raspberry Pi
echo "Run the code on Raspberry Pi ..."
ssh pi@10.192.58.70 << 'EOF'
cd ~/2024-2025-Robotics-cup/
source install/setup.bash
ros2 launch robonav launch_sim.launch.py
EOF

echo "Finished!"
