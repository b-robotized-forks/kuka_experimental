kuka_eki_motion_primitives_hw_interface
==========================================

Driver package to control kuka robot using motion primitives like PTP, LIN and CIRC

![Licence](https://img.shields.io/badge/License-BSD-3-Clause-blue.svg)

# Usage notes:
## Standard
**Launch kr5 with mock hw**
```
ros2 launch kuka_ros2_control_support bringup.launch.py description_package:=kuka_kr5_support description_macro_file:=kr5_arc_macro.xacro use_mock_hardware:=true
```
**Start test node for joint trajectory controller:**
```
ros2 launch kuka_ros2_control_support test_joint_trajectory_controller.launch.py
```



## EKI Simulator
**Start EKI simulator**
```
ros2 run kuka_eki_simulator kuka_eki_simulator 
```
## Normal EKI Communication
**Launch kr5 with eki communication**
```
ros2 launch kuka_ros2_control_support bringup.launch.py description_package:=kuka_kr5_support description_macro_file:=kr5_arc_macro.xacro use_eki_communication:=true
```

## EKI Communication with motion primitive driver
**Launch kr5 with motion primitive driver**
```
ros2 launch kuka_ros2_control_support motion_primitives_bringup.launch.py description_package:=kuka_kr5_support description_macro_file:=kr5_arc_macro.xacro
```
**Launch kr3 with motion primitive driver**
```
ros2 launch kuka_ros2_control_support motion_primitives_bringup.launch.py description_package:=kuka_kr3_support description_macro_file:=kr3r540_macro.xacro
```
**Publish dummy commands**
```
ros2 run kuka_eki_motion_primitives_hw_interface send_dummy_motion_primitives.py
```