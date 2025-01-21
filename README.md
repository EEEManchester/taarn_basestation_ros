# taarn_basestation_ros

## Dependencies
- [MallARD](https://github.com/EEEManchester/MallARD/tree/ice9-dev) _ice9-dev_ branch

There are multiple packages in MallARD, but only ds_cap needs to be built. Target the individual packages when building, e.g.:
```
catkin build ds_cap taarn_basestation_bringup taarn_teleop_joy
```

## Use
Launch basestation
```
roslaunch taarn_basestation_bringup basestation.launch
```

### Joystick mappings
```
==== Bluerov and Mallard ====
Linear X:                   Left joystick horizontal
LInear Y:                   Left joystick horizontal
Yaw:                        Right joystick horizontal
Arming/disarming:           Options button
Attitude control on/off:    Cross button

==== Bluerov only ====
Depth up:                   Right trigger button R2
Depth down:                 Left trigger button L2
Depth hold on/off:          Circl button
```
