
#!/bin/bash

sudo apt install python3-rosdep2
# Ensure the script is run with superuser privileges
if [[ "$EUID" -ne 0 ]]; then
  echo "Please run as root"
  exit 1
fi

# Update package list and install ROS 2 Velodyne driver
apt update
apt install -y ros-humble-velodyne

# Create ROS 2 workspace if it doesn't exist
ROS2_WS=~/Vision2
SRC_DIR=$Vision2/src

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

# Navigate to the workspace root
cd $Vision2

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the setup script
source install/setup.bash

# Create a custom launch file directory if it doesn't exist
LAUNCH_DIR=$SRC_DIR/my_velodyne_launch/launch

if [ ! -d "$LAUNCH_DIR" ]; then
  mkdir -p $LAUNCH_DIR
fi

# Create a custom launch file
cat <<EOL > $LAUNCH_DIR/vlp16_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=[{
                'device_ip': '192.168.1.201',  # Set the IP address of your Velodyne LiDAR
                'frame_id': 'velodyne',
                'model': 'VLP16',
                'rpm': 600,
                'port': 2368,
            }],
        ),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_convert_node',
            name='velodyne_convert_node',
            output='screen',
            parameters=[{
                'calibration': '/path/to/calibration/file.yaml',
                'min_range': 0.5,
                'max_range': 100.0,
                'view_direction': 0.0,
                'view_width': 360.0,
            }],
        ),
    ])
EOL

# Build the workspace again to include the new launch file
colcon build

# Print instructions to the user
echo "Velodyne driver installation and setup complete."
echo "To launch the Velodyne driver, use the following command:"
echo "source ~/Vision2/install/setup.bash && ros2 launch my_velodyne_launch vlp16_launch.py"

