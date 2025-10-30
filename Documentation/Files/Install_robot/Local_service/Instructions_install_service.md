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
chmod +x install_ros2_humble_code&manelpuig_project.sh
./install_ros2_humble_code&manelpuig_project.sh
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
- Copy files to home directory
    - my_robot_service.sh
    - start_robot.sh
- Make both scripts executable:
    ````bash
    chmod +x start_robot.sh
    ````
    >Carefull!: Change ROS_DOMAIN_ID=0 to your Group number.
- Start it manually (optional):
    ````bash
    sudo systemctl start my_robot.service
    ````
- Check service status:
    ````bash
    systemctl status my_robot.service
    ````
- View Logs
    ````bash
    journalctl -u my_robot.service -e
    sudo journalctl -u my_robot -f
    ````
- Enable the service for next power-up
    ````shell
    sudo systemctl daemon-reload
    sudo systemctl enable --now my_robot.service
    ````
- Test After Reboot
    ````bash
    sudo reboot
    ````

## 👌 Notes
The service waits for the network and WiFi (wlan0) before launching

The environment config enables graphical apps (rviz2, rqt)

The workspace is compiled only once (it won’t repeat unnecessarily)


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
