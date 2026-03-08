---
description: Install ROS 2 Humble and MuJoCo Python Bindings
---
1. Setup Locale and Sources for ROS 2 Humble:
// turbo
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

2. Install ROS 2 Humble:
// turbo
```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-argcomplete
```

3. Initialize rosdep:
// turbo
```bash
sudo rosdep init || true
rosdep update
```

4. Install pip and MuJoCo + Math tools:
// turbo
```bash
sudo apt install -y python3-pip
pip3 install mujoco numpy scipy control
```
