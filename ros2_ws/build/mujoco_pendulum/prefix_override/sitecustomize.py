import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jiang/mojoco/ros2_ws/install/mujoco_pendulum'
