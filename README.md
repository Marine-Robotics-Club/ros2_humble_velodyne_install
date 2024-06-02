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


need to setup IP address on ethernet communication usually lidar is 192.168.1.201 255.255.255.250

sends to whatever IP

prolly want to listen on wireshark for that IP 

follow installation steps and enjoy ^_^

NOTE: this will install the velodyne driver and install a Vision2 worskapce 

```
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```


