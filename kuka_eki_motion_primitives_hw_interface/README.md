kuka_eki_motion_primitives_hw_interface
==========================================

Driver package to control kuka robot using motion primitives like PTP, LIN and CIRC

![Licence](https://img.shields.io/badge/License-BSD-3-Clause-blue.svg)

# Documentation
## ROS2 Implementation
TODO(mathias31415) explain how the ros2 implementation works
![img](doc/ros2_control_motion_primitives_kuka.drawio.png)

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

# Usage notes:
## With "simulation"
**1. Start simulation**
```
ros2 run kuka_eki_simulator kuka_eki_simulator_tcp
```
> [!NOTE]  
> **This is not a proper simulation, but it can be helpful for debugging.**
> - **Motion Channel:** Currently, only the motion primitive channel is implemented. The meta channel for interrupting movements is not implemented.
> - **Single PTP Commands:** If the simulator receives an XML message containing only a single PTP (point-to-point) command, the robot's joint positions are set directly to those specified in the command.
> - **Cartesian Commands:** Since no inverse kinematics (IK) is implemented, all joint positions are simply set to zero when a Cartesian command is received.
> - **Multi-Primitive XML:** The simulator cannot correctly process XML messages containing multiple motion primitives. However, the received XML is printed, which is usually sufficient for debugging purposes.


**2. Launch KR3 with motion primitive driver**
```
ros2 launch kuka_ros2_control_support motion_primitives_bringup.launch.py description_package:=kuka_kr3_support description_macro_file:=kr3r540_macro.xacro
```
## With ready2educate H-KA cell 2 (adjust robot_ip for other cells)
**Launch kr3 with motion primitive driver**
```
ros2 launch kuka_ros2_control_support motion_primitives_bringup.launch.py description_package:=kuka_kr3_support description_macro_file:=kr3r540_macro.xacro eki_robot_ip:=10.181.116.51
```
## Publish dummy commands
**Commands from python script**
```
ros2 run kuka_eki_motion_primitives_hw_interface send_dummy_motion_primitives.py
```
**Stop command in terminal**
```
ros2 topic pub /motion_primitive_controller/reference industrial_robot_motion_interfaces/msg/MotionPrimitive "{type: 66, blend_radius: 0.0, additional_arguments: [], poses: [], joint_positions: []}" --once
```
**Reset stop command in terminal**
```
ros2 topic pub /motion_primitive_controller/reference industrial_robot_motion_interfaces/msg/MotionPrimitive "{type: 67, blend_radius: 0.0, additional_arguments: [], poses: [], joint_positions: []}" --once
```



# TODOs
- Blending between two motionprimitives not working yet
- execute eki_close("NAME") when programm is stopped to properly close the connection --> without eki_close() its not possible to init a new server when restarting the program. 
- handle additional paramters like velocity and acceleration
- how to check if motion execution ends with success?