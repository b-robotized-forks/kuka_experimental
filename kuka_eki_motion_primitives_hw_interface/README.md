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
```
ros2 launch kuka_ros2_control_support motion_primitives_bringup.launch.py description_package:=kuka_kr3_support description_macro_file:=kr3r540_macro.xacro eki_robot_ip:=10.181.116.51
```
**Publish dummy commands**
```
ros2 run kuka_eki_motion_primitives_hw_interface send_dummy_motion_primitives.py
```

### 4.2 hka_motion_primitive_kuka_driver branch using robot_interface_eki implementation from Moritz
#### With Simulation
```
ros2 run kuka_eki_simulator kuka_eki_simulator_tcp
```
**Launch kr3 with motion primitive driver**
```
ros2 launch kuka_ros2_control_support motion_primitives_bringup.launch.py description_package:=kuka_kr3_support description_macro_file:=kr3r540_macro.xacro
```
#### With r2e cell 2
**Launch kr3 with motion primitive driver**
```
ros2 launch kuka_ros2_control_support motion_primitives_bringup.launch.py description_package:=kuka_kr3_support description_macro_file:=kr3r540_macro.xacro eki_robot_ip:=10.181.116.51
```
#### Publish dummy commands
**Commands from script**
```
ros2 run kuka_eki_motion_primitives_hw_interface send_dummy_motion_primitives.py
```
**Stop command in terminal**
```
ros2 topic pub /motion_primitive_controller/reference industrial_robot_motion_interfaces/msg/MotionPrimitive "{type: 66, blend_radius: 0.0, additional_arguments: [], poses: [], joint_positions: []}" --once
```


# Documentation
## ROS2 Implementation
TODO(mathias31415) explain how the ros2 implementation works

## KRL Implementation
TODO(mathias31415) explain how the krl implementation works

## KRL Files
TODO(mathias31415) write instruction on how to place the KRL files in WorkVisual (Move the instruction to the krl folder)

## EKI TCP connection
To ensure the TCP connection is closed properly, the client needs to disconnect first [source](https://youtu.be/Ne13sBHPGv4?t=863), so the ROS2 side needs to be stopped first. When client disconnects, the `$flag[1]` and `$flag[2]` are set to false (defined in the xml files). This triggers the interrupt to call the reset_interface() and reset_meta_interface() functions. 
```
global interrupt decl 15 when $flag[1] == false do reset_interface()
interrupt on 15
```
This ensures the client (ROS2) can reconnect to the server. During testing/ implementing of the KRL files its recommended to replace `reset_interface()` with `close_interface()` (in `eki.src` and `meta_eki.sub`) to close the connection propperly and dont reset it. This ensures that a new connection can be established after the program is stopped. (robot- and submit-interpreter needs to be restarted (deselect (abwählen) and then select (anwählen) again))

Before transfering a new Version of the KUKA project to the Robot via WorkVisual, stop the ROS2 side and deselect the robot- and submit-interpreter (therefore you need to be in expert mode). If this is not done, the Robotersteuerung needs to get restarted.

TODO(mathias31415) insert images/ screenshots to better explain the stuff