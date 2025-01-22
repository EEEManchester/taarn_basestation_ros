# taarn_teleop_joy
Joystick teleoperation for Bluerov and Mallard, or other generic ROS-compatible robots that uses mavlink for communication.

## Dependencies
- [joy](https://github.com/ros-drivers/joystick_drivers)
- [mavros_msgs](https://github.com/mavlink/mavros)

## Parameters
See [taarn_teleop_joy/config/bluerov_ps4.config.yaml](taarn_teleop_joy/config/bluerov_ps4.config.yaml) for an example.
### Linear axes
Under the `axis_linear` scope, you can define the buttons for x, y, depth_up, and depth_down.
```
axis_linear:
  x: 1
  y: 0
  depth_up: 5
  depth_down: 2
```

Under the `scale_linear` scope, you can define the scaling factor for x, y, depth_up, and depth_down.
```
scale_linear:
  x: 0.2
  y: -0.2
  depth_up: -0.2
  depth_down: -0.2
```

Under the `offset_linear` scope, you can define the offset for x, y, depth_up, and depth_down.
```
offset_linear:
  x: 0
  y: 0
  depth_up: -1
  depth_down: -1
```
### Angular axes
Under the `axis_angular` scope, you can define the buttons for yaw.
```
axis_angular:
  yaw: 3
```

Under the `scale_angular` scope, you can define the scaling factor for yaw.
```
scale_angular:
  yaw: -0.12
```

Under the `offset_angular` scope, you can define the offset for yaw.
```
offset_angular:
  yaw: 0
```
### Buttons
Under the `arming_button` scope, you can define buttons for toggling arming/disarming, attitude control, and depth hold.
```
arming_button: 9
attitude_control_button: 0
depth_hold_button: 1
```

##### Notes

1. The state of arming and disarming is stored in the local scope, so it will be lost when the node is restarted. Arming and disarming uses mavros service calls.
2. The state of attitude control and depth hold is not stored in the local scope. Upon button press, a Trigger message will be sent to the corresponding ros service call. To use these features, you need to have the corresponding services defined in your robot controller.
