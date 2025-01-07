# taarn_basestation_ros

## Dependencies
- [Mallard](https://github.com/EEEManchester/MallARD/tree/Xueliang) _Xueliang branch_
- visual_virtual_tether

## Get started
Launch basestation
```
roslaunch taarn_basestation_bringup basestation.launch
```

## Use
Launch basestation
```
roslaunch taarn_basestation_bringup basestation.launch
```

### Joystick mappings
```
==== Bluerov and Mallard ====
joystick left horizontal:   linear x
joystick left vertical:     linear y
joystick right horizontal:  yaw
options button:             arming/disarming
cross button:               enable/disable attitude control

==== Bluerov only ====
trigger button right R2:    depth up
trigger button left L2:     depth down
circle button:              enable/disable depth hold
```