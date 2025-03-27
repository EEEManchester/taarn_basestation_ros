# taarn_basestation_ros

## Dependencies
- [joy](https://github.com/ros-drivers/joystick_drivers)
- [mavros_msgs](https://github.com/mavlink/mavros)
- [MallARD](https://github.com/EEEManchester/MallARD/tree/ice9-dev) _ice9-dev_ branch

## Installation
Make a new directory for your catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
Clone the repository and dependencies:
```
git clone git@github.com:EEEManchester/taarn_basestation_ros.git
git clone -b ice9-dev git@github.com:EEEManchester/MallARD.git
```
Install other dependencies and build:
```
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

## Use
Launch basestation
```
roslaunch taarn_basestation_bringup basestation.launch
```

## Teleop
Joystick mappings:
```
==== Bluerov and Mallard ====
Linear X:                   Left  joystick vertical
Linear Y:                   Left  joystick horizontal
Yaw:                        Right joystick horizontal
Arming/disarming:           Options button
Attitude control on/off:    Cross   button

==== Bluerov only ====
Depth up:                   Right trigger button R2
Depth down:                 Left  trigger button L2
Depth hold on/off:          Circle  button
```

For configure the joystick mappings and control gains, see [taarn_teleop_joy/README.md](taarn_teleop_joy/README.md).
