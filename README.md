# Wave Rover Control

## Description
This is a Ros2 package for basic control of the Wave Share Wave Rover. https://www.waveshare.com/wiki/WAVE_ROVER

This is just a project for me to learn Ros2.

Built using Ros2 Jazzy

## Goals & Progress
Below are the key goals for this project. Completed items are checked off as progress is made.

- ✅ Goal 1: Movement control using cmd_vel/Twist messages
- ✅  Goal 2: Continuous IMU data collection / quaternion conversion
- ❌ Goal 3: Emergency stop function

## Dependencies

This package requires nlohmann-json and LibSerial to work

```sh
sudo apt update
sudo apt install nlohmann-json3-dev libserial-dev
```

## Installation
Instructions on how to install and setup the package

Source your Ros2 Distro
```sh
source /opt/ros/<-ros-distro->/setup.bash
```

Clone repository
```sh
cd ~/ros2_ws/src
git clone https://github.com/Moth-Balls/Wave-Rover-Control.git
```

Build Ros2 packages
```sh
cd ../
colcon build --symlink-install
```

Source install script
```sh
source install/setup.bash
```


