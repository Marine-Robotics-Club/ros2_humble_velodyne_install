#!/bin/bash

# Install python3-rosdep2 if not already installed
sudo apt install -y python3-rosdep2

# Ensure the script is run with superuser privileges
if [[ "$EUID" -ne 0 ]]; then
  echo "Please run as root"
  exit 1
fi

# Update package list and install ROS 2 Velodyne driver
apt update
apt install -y ros-humble-velodyne

# Define the ROS 2 workspace
ROS2_WS=~/Vision2
SRC_DIR=$ROS2_WS/src

# Source ROS 2 setup script
source /opt/ros/humble/setup.bash

# Delete existing install, log, and build directories if they exist
if [ -d "$ROS2_WS/install" ]; then
  rm -rf $ROS2_WS/install
fi

if [ -d "$ROS2_WS/log" ]; then
  rm -rf $ROS2_WS/log
fi

if [ -d "$ROS2_WS/build" ]; then
  rm -rf $ROS2_WS/build
fi

# Create ROS 2 workspace if it doesn't exist
if [ ! -d "$SRC_DIR" ]; then
  mkdir -p $SRC_DIR
fi

# Navigate to the src directory
cd $SRC_DIR

# Clone the velodyne repository
if [ ! -d "velodyne" ]; then
  git clone https://github.com/ros-drivers/velodyne.git -b ros2
else
  echo "Velodyne repository already exists in the workspace."
fi

# Create the my_velodyne_launch package
MY_VELODYNE_LAUNCH_DIR=$SRC_DIR/my_velodyne_launch

if [ ! -d "$MY_VELODYNE_LAUNCH_DIR" ]; then
  mkdir -p $MY_VELODYNE_LAUNCH_DIR/launch
  mkdir -p $MY_VELODYNE_LAUNCH_DIR/config
fi

# Create the YAML configuration file for VLP16_hires_db
cat <<EOL > $MY_VELODYNE_LAUNCH_DIR/config/VLP16_hires_db.yaml
velodyne_driver_node:
  ros__parameters:
    device_ip: "192.168.1.201"
    frame_id: "velodyne"
    model: "VLP16"
    rpm: 600
    port: 2368

velodyne_convert_node:
  ros__parameters:
    calibration: "$SRC_DIR/velodyne/velodyne_pointcloud/params/VLP16.yaml"
    min_range: 0.5
    max_range: 100.0
    view_direction: 0.0
    view_width: 360.0
EOL

# Create the launch file for VLP16_hires_db.yaml
cat <<EOL > $MY_VELODYNE_LAUNCH_DIR/launch/vlp16.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='config/VLP16_hires_db.yaml',
            description='Path to the YAML configuration file'
        ),
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=[LaunchConfiguration('config')]
        ),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_convert_node',
            name='velodyne_convert_node',
            output='screen',
            parameters=[LaunchConfiguration('config')]
        ),
    ])
EOL

# Navigate to the workspace root
cd $ROS2_WS

# Build the workspace
colcon build

# Source the setup script
source install/setup.bash

# Print instructions to the user
echo "Velodyne driver installation and setup complete."
echo "To launch the Velodyne driver with the VLP16_hires_db configuration, use the following command:"
echo "source ~/Vision2/install/setup.bash && ros2 launch my_velodyne_launch vlp16.launch.py"
