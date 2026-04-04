# Robot Operations Notes (Raspberry Pi + ROS2 + ESP32)

This file keeps practical setup and troubleshooting notes for running the robot on the Raspberry Pi.

---

## 1) Make autostart script executable

```bash
chmod +x /home/pi/2024-2025-Robotics-cup/auto_start/one_go.sh
```

---

## 2) Register `one_go.service` with systemd

Create a symbolic link so `systemctl` can manage the service:

```bash
sudo ln -s /home/pi/2024-2025-Robotics-cup/auto_start/one_go.service /etc/systemd/system
```

---

## 3) Enable and start service at boot

```bash
sudo systemctl daemon-reload
sudo systemctl enable one_go.service
sudo systemctl start one_go.service
```

After this, launch should start automatically when the robot boots.

Check status:

```bash
systemctl status one_go.service
```

---

## 4) Delete/remove the service

```bash
sudo systemctl stop one_go.service
sudo systemctl disable one_go.service
sudo rm /etc/systemd/system/one_go.service
sudo systemctl daemon-reload
```

---

## 5) Live logs

```bash
journalctl -u one_go.service -f
```

---

## 6) Wi-Fi/netplan

```bash
cd /etc/netplan/50-cloud-init.yaml
sudo netplan apply
```

---

## 7) Launch ROS2 manually

From repository root (`2024-2025-Robotics-cup`):

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robonav launch.launch.py
```

Test node:

```bash
ros2 run robonav led_test_node
```

---

## 8) Device mapping note

A helper file/rule is used to auto-detect LiDAR and ESP and map ports to:
- `/dev/esp32`
- `/dev/rplidar`

In PlatformIO, you can set upload/monitor port to `/dev/esp32`.

---

## 9) ESP upload notes

- Keep **Safe Start button** on reset for upload.
- Press **reset while building/uploading** if upload fails (this happens sometimes).
- Emergency button needs to be pressed.

---

## 10) Startup/reset behavior note

- Raspberry Pi startup/reset can be a bit slow.
- Possible reason might be LED-related behavior (not confirmed).

Raspberry Pi LED pin used in this project:
- **GPIO 16**
