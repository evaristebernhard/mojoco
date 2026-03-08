---
description: Initialize ROS 2 Workspace and Python Package
---
1. Create the ROS 2 Workspace and ament_python package:
// turbo
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/mojoco/ros2_ws/src
cd ~/mojoco/ros2_ws/src
ros2 pkg create --build-type ament_python mujoco_pendulum --dependencies rclpy std_msgs sensor_msgs
```

2. Build the workspace:
// turbo
```bash
cd ~/mojoco/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
