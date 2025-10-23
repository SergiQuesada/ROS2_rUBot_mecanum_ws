# ✅ ROS 2 Humble Installation and Auto-Startup Setup on Raspberry Pi 4 (Ubuntu 22.04)

This guide explains how to:

- Install ROS 2 Humble (Desktop)
- Install required dependencies
- Build your robot workspace
- Configure a systemd service to launch the robot on boot
- Verify everything is working correctly

---

## 1. Install ROS 2 Humble and Dependencies

### 🔧 Run the installation script:

```bash
chmod +x install_ros2_humble_full.sh
./install_ros2_humble_full.sh
````
This script will:

- Configure locale and repositories

- Install ros-humble-desktop

- Install useful dev and robot packages

- Clone your custom workspace and RPLidar driver

- Resolve dependencies with rosdep

- Build with colcon

- Add environment setup to .bashrc

# ✅ Verify the installation
After it finishes (approx. 30–45 minutes):

````bash
source ~/.bashrc
ros2 doctor
````

You should see:

```css
All 5 checks passed
````
You can also test a basic node:

````bash
ros2 run demo_nodes_cpp talker
````
## 2. Set Up Systemd Service: my_robot.service
This service will launch your bringup automatically on boot.
### Copy files to home directory
- setup_my_robot_service.sh
- start_robot.sh
### 🛠️ Run the setup script:
````bash
chmod +x setup_my_robot_service.sh
./setup_my_robot_service.sh
````
This script will:

- Create the systemd unit at /etc/systemd/system/my_robot.service

- Enable the service so it runs on boot

### 📄 start_robot.sh script content
This file must already exist at /home/ubuntu/start_robot.sh.

>Carefull!: Change ROS_DOMAIN_ID=0 to your Group number.

````bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ROS2_rUBot_mecanum_ws/install/setup.bash
ros2 launch my_robot_bringup my_robot_bringup_hw.launch.py
````
Make sure it's executable:

````bash
chmod +x /home/ubuntu/start_robot.sh
````
## 3. Check the Service
### 🔹 Start it manually (optional):
````bash
sudo systemctl start my_robot.service
````
### 🔹 Check service status:
````bash
systemctl status my_robot.service
````
You should see:

```plaintext
Active: active (running)
````

## 4. View Logs
To debug output and see ROS launch logs:

````bash
journalctl -u my_robot.service -e
````
## 5. Test After Reboot
````bash
sudo reboot
````
Then check again:

````bash
systemctl status my_robot.service
````
If the launch works at boot, everything is correctly configured.

## ✅ Summary Table
| Task                    | Command                                           |
|-------------------------|---------------------------------------------------|
| Run ROS install script  | `./install_ros2_humble_full.sh`                  |
| Verify ROS install      | `ros2 doctor` / `ros2 run demo_nodes_cpp talker` |
| Setup systemd service   | `./setup_my_robot_service.sh`                    |
| Start service manually  | `sudo systemctl start my_robot.service`          |
| Check service status    | `systemctl status my_robot.service`              |
| View service logs       | `journalctl -u my_robot.service -e`              |
| Reboot and verify       | `sudo reboot` + check status again               |

## 📌 File References
| File                                      | Purpose                                |
|-------------------------------------------|----------------------------------------|
| `install_ros2_humble_full.sh`             | Install ROS 2 and all dependencies     |
| `setup_my_robot_service.sh`               | Create and enable the systemd service  |
| `/home/ubuntu/start_robot.sh`             | Launch script for your robot           |
| `/etc/systemd/system/my_robot.service`    | Systemd unit file                      |

## 👌 Notes
The service waits for the network and WiFi (wlan0) before launching

The environment config enables graphical apps (rviz2, rqt)

The workspace is compiled only once (it won’t repeat unnecessarily)

Happy robot launching! 🤖🚀

## Graphical display

Open: 
- Xlaunch
- or MobaXterm

Then run:

```bash
export DISPLAY=192.168.1.3:0.0
rviz2
```
## 3. Complete bringup with Rosbridge and webserver

- Save the 3 service files to /etc/systemd/system/:
    ````bash
    sudo nano /etc/systemd/system/my_robot.service
    sudo nano /etc/systemd/system/my_rosbridge.service
    sudo nano /etc/systemd/system/my_robot_web.service
    ````
- Enable and start all services:
    ```bash
    sudo systemctl daemon-reload
    sudo systemctl enable --now my_robot.service my_rosbridge.service my_robot_web.service
    ````
- Check status of all services:
    ```bash
    systemd-analyze critical-chain my_robot_web.service
    sudo systemctl status my_robot
    sudo systemctl status my_rosbridge
    sudo systemctl status my_robot_web
    sudo journalctl -u my_robot -f
    sudo journalctl -u my_rosbridge -f
    sudo journalctl -u my_robot_web -f
    ````
- Reboot and verify all services start automatically
