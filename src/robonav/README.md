# make it executable
chmod +x /home/pi/2024-2025-Robotics-cup/auto_start/one_go.sh

# make one_go.service symbolic so that you can call one_go.service with systemctl
sudo ln -s /home/pi/2024-2025-Robotics-cup/auto_start/one_go.service /etc/systemd/system

# enable and start one_go.service
sudo systemctl daemon-reload
sudo systemctl enable one_go.service
sudo systemctl start one_go.service
# now the launch will be start when the robot boots

# check if it's running
systemctl status one_go.service



Delete the service:
sudo systemctl stop one_go.service
sudo systemctl disable one_go.service
sudo rm /etc/systemd/system/one_go.service
sudo systemctl daemon-reload


# Live Logs output:
journalctl -u one_go.service -f



# Wifi: 
cd /etc/netplan/50-cloud-init.yaml
sudo netplan apply


# Launch Ros2
colcon build --symlink-install      in the 2024-2025..... 
source install/setup.bash
ros2 launch robonav launch.launch.py


ros2 run robonav led_test_node



I also made a file to auto detect the lidar and esp automatically and map the ports to /dev/esp32 and /dev/rplidar, so you can set teh port in platform.io extension to /dev/esp32

Safe start button needs to be on reset to upload, and press reset while it is building because sometimes it doesnt upload properly, and emergency need to be pressed

Starting and resetting the RPI takes a little bit long, I dont know, maybe because of the LED fucntion but not sure

The RPI Led is on GPIO pin 16