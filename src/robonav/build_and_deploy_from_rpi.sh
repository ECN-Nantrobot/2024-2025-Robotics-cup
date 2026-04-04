#!/bin/bash

# Function to check if the last command was successful
check_command() {
  if [ $? -ne 0 ]; then
    echo "Error: $1 failed. Exiting."
    exit 1
  fi
}

# Define variables
REMOTE_USER="pi"
REMOTE_IP="10.192.58.70"
LOCAL_WORKSPACE="~/2024-2025-Robotics-cup"
REMOTE_WORKSPACE="~/2024-2025-Robotics-cup"
BRANCH="ferdinand_rpi"  # Change this to your branch name if needed

# Step 1: Commit and push changes from the Raspberry Pi
echo "Commit and Push Changes from Raspberry Pi ..."
ssh $REMOTE_USER@$REMOTE_IP << EOF
cd $REMOTE_WORKSPACE
git add .
git commit -m "Automatic commit for build and deploy"
git push origin $BRANCH
EOF
check_command "Git commit and push"

# Step 2: Pull the latest changes on your local machine
echo "Getting the latest changes from the Raspberry Pi..."
git pull origin $BRANCH
check_command "Git pull"

# Step 3: Run the build with colcon
echo "Running colcon build --symlink-install..."
colcon build --symlink-install
check_command "Colcon build"

# Step 4: Copy compiled code back to the Raspberry Pi
echo "Copying compiled code to Raspberry Pi..."
scp -r $HOME/2024-2025-Robotics-cup/install $REMOTE_USER@$REMOTE_IP:$REMOTE_WORKSPACE
check_command "SCP copy"

# Step 5: Run the code on the Raspberry Pi
echo "Running code on Raspberry Pi..."
ssh $REMOTE_USER@$REMOTE_IP << EOF
cd $REMOTE_WORKSPACE
source install/setup.bash
ros2 run robonav launch_sim.launch.py
EOF
check_command "Running ROS2 node"

echo "Finished!"
