# ros2_humble_velodyne_install
shell script for installing the driver for the velodyne lidar 

- Ubuntu 22.04
- 1.6 gb storage

GIT INSTALL:
```
cd && git clone https://github.com/Marine-Robotics-Club/ros2_humble_velodyne_install.git
```
```
cd ~/ros2_humble_velodyne_install && chmod +x ros2_humble_velodyne_install.sh && sudo ./ros2_humble_velodyne_install.sh
```
follow installation steps and enjoy ^_^

NOTE: this will install the velodyne driver and install a Vision2 worskapce 

```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```

need to setup IP address on ethernet communication usually lidar is 192.168.1.201 255.255.255.250

sends to whatever IP



git clone -b humble-devel https://github.com/ros-drivers/velodyne.git

sudo apt-get install python3-colcon-common-extensions
sudo apt-get install ros-humble-diagnostic-updater
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/ros/humble

export diagnostic_updater_DIR=/opt/ros/humble/share/diagnostic_updater/cmake

sudo apt-get install libpcap-dev
sudo apt-get update

colcon build --cmake-clean-cache
sudo apt install ros-humble-velodyne
sudo ufw disable
sudo apt install ufw


prolly want to listen on wireshark for that IP 



