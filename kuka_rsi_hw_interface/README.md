# RSI example usage notes
## Config
### Ubuntu PC:
- IP: 193.196.1.30
- Netmask: 255.255.255.0
### Robot RSI Subnet Cell2:
- IP: 193.196.1.20
- Netmask: 255.255.255.0

## With simulation
### Start rsi simulator
```
ros2 run kuka_rsi_simulator kuka_rsi_simulator
```
### Launch rsi hw-interface
```
ros2 launch kuka_ros2_control_support bringup.launch.py description_package:=kuka_kr3_support description_macro_file:=kr3r540_macro.xacro use_rsi_communication:=true rsi_listen_port:=49152 

## With ready2educate cell 2
### Launch rsi hw-interface
```
ros2 launch kuka_ros2_control_support bringup.launch.py description_package:=kuka_kr3_support description_macro_file:=kr3r540_macro.xacro use_rsi_communication:=true rsi_listen_ip:=193.196.1.30 rsi_listen_port:=49152 
```
```
## Publish trajectory point for position_trajectory_controller in terminal
```
ros2 topic pub /position_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
- 'joint_a1'
- 'joint_a2'
- 'joint_a3'
- 'joint_a4'
- 'joint_a5'
- 'joint_a6'
points:
- positions: [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  accelerations: []
  effort: []
  time_from_start:
    sec: 5
    nanosec: 0" --once
```

```
ros2 topic pub /position_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
- 'joint_a1'
- 'joint_a2'
- 'joint_a3'
- 'joint_a4'
- 'joint_a5'
- 'joint_a6'
points:
- positions: [1.57, -1.57, 1.57, 0.0, 1.57, 0.0]
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  accelerations: []
  effort: []
  time_from_start:
    sec: 7
    nanosec: 0" --once
```
```
ros2 topic pub /position_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
- 'joint_a1'
- 'joint_a2'
- 'joint_a3'
- 'joint_a4'
- 'joint_a5'
- 'joint_a6'
points:
- positions: [-1.57, -1.57, 1.57, 0.0, 1.57, 0.0]
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  accelerations: []
  effort: []
  time_from_start:
    sec: 5
    nanosec: 0" --once
```



## Publish trajectory points for position_trajectory_controller with python script
```
ros2 run kuka_rsi_hw_interface send_joint_positions.py
```