# mBot using Rubik Pi and ROS2

This project lets you control a Makeblock mBot using the Rubik Pi in ROS2.

## Usage

```sh
# install ROS2
sudo apt install ros-apt-source
sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-teleop-twist-keyboard ros-jazzy-imu-complementary-filter

# create a virtualenv for Adafruit libraries
sudo apt install python3 python3-venv python3-pip
python3 -m venv ros2_env
source ros2_env/bin/activate
pip install adafruit-extended-bus adafruit-circuitpython-icm20x

# build the project
cd ros2_ws
colcon build
source install/setup.bash

# launch
ros2 launch mbot_bringup keyboard.py
```

## Licenses

* [Adafruit Python Extended Bus](https://github.com/adafruit/Adafruit_Python_Extended_Bus/blob/main/LICENSE)
* [Adafruit CircuitPython ICM20X](https://github.com/adafruit/Adafruit_CircuitPython_ICM20X/blob/main/LICENSE)
