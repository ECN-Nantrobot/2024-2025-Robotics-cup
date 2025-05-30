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


Live Logs output:
journalctl -u one_go.service -f
