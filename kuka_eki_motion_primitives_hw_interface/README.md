kuka_eki_motion_primitives_hw_interface
==========================================

Driver package to control kuka robot using motion primitives like PTP, LIN and CIRC

![Licence](https://img.shields.io/badge/License-BSD-3-Clause-blue.svg)

# Usage notes:
## 1 Standard kuka_experimental
**Launch kr5 with mock hw**
```
ros2 launch kuka_ros2_control_support bringup.launch.py description_package:=kuka_kr5_support description_macro_file:=kr5_arc_macro.xacro use_mock_hardware:=true
```
**Start test node for joint trajectory controller:**
```
ros2 launch kuka_ros2_control_support test_joint_trajectory_controller.launch.py
```



## 2 Normal kuka_experimental with EKI Communication
**Start EKI simulator with UDP**
```
ros2 run kuka_eki_simulator kuka_eki_simulator 
```
**Launch kr5 with eki communication**
```
ros2 launch kuka_ros2_control_support bringup.launch.py description_package:=kuka_kr5_support description_macro_file:=kr5_arc_macro.xacro use_eki_communication:=true
```

## 3 H-KA EKI implementation
**Start EKI simulator with TCP**
```
ros2 run kuka_eki_simulator kuka_eki_simulator_tcp
```
**Start Server for EKI communication**
```
ros2 launch robot_interface_eki robot_interface.launch.py
```
**Start Client to send commands**
```
ros2 run robot_interface_eki client.py
```



## 4 EKI Communication with motion primitive driver
### 4.1 motion_primitive_kuka_driver branch using kuka_experimental eki implementation
**Start EKI simulator with UDP**
```
ros2 run kuka_eki_simulator kuka_eki_simulator 
```
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

### 4.2 hka_motion_primitive_kuka_driver branch using robot_interface_eki implementation from Moritz
```
ros2 run kuka_eki_simulator kuka_eki_simulator_tcp
```
**Launch kr3 with motion primitive driver**
```
ros2 launch kuka_ros2_control_support motion_primitives_bringup.launch.py description_package:=kuka_kr3_support description_macro_file:=kr3r540_macro.xacro
```
**Publish dummy commands**
```
ros2 run kuka_eki_motion_primitives_hw_interface send_dummy_motion_primitives.py
```


