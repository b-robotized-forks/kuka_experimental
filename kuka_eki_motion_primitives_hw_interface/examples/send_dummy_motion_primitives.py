#!/usr/bin/env python3

# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Mathias Fuhrer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from industrial_robot_motion_interfaces.msg import MotionPrimitive, MotionArgument
import time

### testing commands for r2e cell
msg_start_sequence = MotionPrimitive()
msg_start_sequence.type = MotionPrimitive.MOTION_SEQUENCE_START

msg_end_sequence = MotionPrimitive()
msg_end_sequence.type = MotionPrimitive.MOTION_SEQUENCE_END

msg_stop = MotionPrimitive()
msg_stop.type = MotionPrimitive.STOP_MOTION

msg_PTP_1 = MotionPrimitive()
msg_PTP_1.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_1.joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
msg_PTP_1.blend_radius = 0.1
msg_PTP_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

msg_PTP_1_slow = MotionPrimitive()
msg_PTP_1_slow.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_1_slow.joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
msg_PTP_1_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]

msg_PTP_11 = MotionPrimitive()
msg_PTP_11.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_11.joint_positions = [0.15, -1.57, 1.57, 0.0, 1.57, 0.0]
msg_PTP_11.blend_radius = 0.1
msg_PTP_11.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

msg_PTP_12 = MotionPrimitive()
msg_PTP_12.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_12.joint_positions = [0.3, -1.57, 1.57, 0.0, 1.57, 0.0]
msg_PTP_12.blend_radius = 0.1
msg_PTP_12.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

msg_PTP_2 = MotionPrimitive()
msg_PTP_2.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_2.joint_positions = [0.5, -1.57, 1.57, 0.0, 1.57, 0.0]
msg_PTP_2.blend_radius = 0.1
msg_PTP_2.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

msg_PTP_2_slow = MotionPrimitive()
msg_PTP_2_slow.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_2_slow.joint_positions = [0.5, -1.57, 1.57, 0.0, 1.57, 0.0]
msg_PTP_2_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]

msg_PTP_21 = MotionPrimitive()
msg_PTP_21.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_21.joint_positions = [0.65, -1.57, 1.57, 0.0, 1.57, 0.0]
msg_PTP_21.blend_radius = 0.1
msg_PTP_21.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

msg_PTP_22 = MotionPrimitive()
msg_PTP_22.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_22.joint_positions = [0.8, -1.57, 1.57, 0.0, 1.57, 0.0]
msg_PTP_22.blend_radius = 0.1
msg_PTP_22.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

msg_PTP_3 = MotionPrimitive()
msg_PTP_3.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_3.joint_positions = [1.0, -1.57, 1.57, 0.0, 1.57, 0.0]
msg_PTP_3.blend_radius = 0.1
msg_PTP_3.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

msg_LIN_1 = MotionPrimitive()
msg_LIN_1.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L1 = PoseStamped()
pose_L1.pose.position.x = 0.240
pose_L1.pose.position.y = 0.0
pose_L1.pose.position.z = 0.550
pose_L1.pose.orientation.x = 0.0
pose_L1.pose.orientation.y = 1.0 
pose_L1.pose.orientation.z = 0.0 
pose_L1.pose.orientation.w = 0.0 
msg_LIN_1.poses = [pose_L1]
msg_LIN_1.blend_radius = 0.1
msg_LIN_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

msg_LIN_1_slow = MotionPrimitive()
msg_LIN_1_slow.type = MotionPrimitive.LINEAR_CARTESIAN
msg_LIN_1_slow.poses = [pose_L1]
msg_LIN_1_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]

msg_LIN_2 = MotionPrimitive()
msg_LIN_2.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L2 = PoseStamped()
pose_L2.pose.position.x = 0.180
pose_L2.pose.position.y = 0.140
pose_L2.pose.position.z = 0.550
pose_L2.pose.orientation.x = 0.0
pose_L2.pose.orientation.y = 1.0 
pose_L2.pose.orientation.z = 0.0 
pose_L2.pose.orientation.w = 0.0 
msg_LIN_2.poses = [pose_L2]
msg_LIN_2.blend_radius = 0.1
msg_LIN_2.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

msg_LIN_2_slow = MotionPrimitive()
msg_LIN_2_slow.type = MotionPrimitive.LINEAR_CARTESIAN
msg_LIN_2_slow.poses = [pose_L2]
msg_LIN_2_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]

msg_LIN_3 = MotionPrimitive()
msg_LIN_3.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L3 = PoseStamped()
pose_L3.pose.position.x = 0.240
pose_L3.pose.position.y = 0.140
pose_L3.pose.position.z = 0.550
pose_L3.pose.orientation.x = 0.0
pose_L3.pose.orientation.y = 1.0 
pose_L3.pose.orientation.z = 0.0 
pose_L3.pose.orientation.w = 0.0 
msg_LIN_3.poses = [pose_L3]
msg_LIN_3.blend_radius = 0.1
msg_LIN_3.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

msg_LIN_4 = MotionPrimitive()
msg_LIN_4.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L4 = PoseStamped()
pose_L4.pose.position.x = 0.240
pose_L4.pose.position.y = -0.140
pose_L4.pose.position.z = 0.550
pose_L4.pose.orientation.x = 0.0
pose_L4.pose.orientation.y = 1.0 
pose_L4.pose.orientation.z = 0.0 
pose_L4.pose.orientation.w = 0.0 
msg_LIN_4.poses = [pose_L4]
msg_LIN_4.blend_radius = 0.1
msg_LIN_4.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

msg_LIN_5 = MotionPrimitive()
msg_LIN_5.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L5 = PoseStamped()
pose_L5.pose.position.x = 0.180
pose_L5.pose.position.y = -0.140
pose_L5.pose.position.z = 0.550
pose_L5.pose.orientation.x = 0.0
pose_L5.pose.orientation.y = 1.0 
pose_L5.pose.orientation.z = 0.0 
pose_L5.pose.orientation.w = 0.0 
msg_LIN_5.poses = [pose_L5]
msg_LIN_5.blend_radius = 0.1
msg_LIN_5.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

msg_moveC_1 = MotionPrimitive()
msg_moveC_1.type = MotionPrimitive.CIRCULAR_CARTESIAN
pose_C1_via = PoseStamped()
pose_C1_via.pose.position.x = 0.340
pose_C1_via.pose.position.y = 0.0
pose_C1_via.pose.position.z = 0.570
pose_C1_via.pose.orientation.x = 0.0
pose_C1_via.pose.orientation.y = 1.0
pose_C1_via.pose.orientation.z = 0.0
pose_C1_via.pose.orientation.w = 0.0
pose_C1_goal = PoseStamped()
pose_C1_goal.pose.position.x = 0.255
pose_C1_goal.pose.position.y = 0.170
pose_C1_goal.pose.position.z = 0.550
pose_C1_goal.pose.orientation.x = 0.0
pose_C1_goal.pose.orientation.y = 1.0
pose_C1_goal.pose.orientation.z = 0.0
pose_C1_goal.pose.orientation.w = 0.05
msg_moveC_1.poses = [pose_C1_goal, pose_C1_via] 
msg_moveC_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

msg_moveC_1_slow = MotionPrimitive()
msg_moveC_1_slow.type = MotionPrimitive.CIRCULAR_CARTESIAN
msg_moveC_1_slow.poses = [pose_C1_goal, pose_C1_via] 
msg_moveC_1_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]



# send all motion primitives directly one after the other
class MotionPublisher(Node):
    def __init__(self):
        super().__init__('motion_publisher')

        self.publisher_ = self.create_publisher(MotionPrimitive, '/motion_primitive_controller/reference', 10)
        # self.messages = [msg_start_sequence, msg_PTP_1, msg_PTP_2, msg_PTP_3, msg_LIN_1, msg_LIN_2, msg_end_sequence]
        # self.messages = [msg_PTP_1, msg_PTP_2, msg_PTP_3, msg_PTP_2, msg_LIN_1, msg_LIN_2]
        # self.messages = [msg_PTP_1, msg_PTP_2, msg_PTP_3, msg_PTP_2, msg_LIN_1, msg_LIN_2, msg_start_sequence, msg_PTP_1, msg_PTP_2, msg_PTP_3, msg_LIN_1, msg_LIN_2, msg_moveC_1, msg_end_sequence, msg_PTP_1, msg_PTP_2]
        # self.messages = [msg_PTP_1, msg_PTP_2, msg_PTP_3, msg_LIN_1, msg_LIN_2, msg_moveC_1]
        # self.messages = [msg_start_sequence, msg_PTP_1, msg_PTP_2, msg_PTP_3, msg_LIN_1, msg_LIN_2, msg_moveC_1, msg_end_sequence]
        # self.messages = [msg_PTP_1, msg_PTP_11, msg_PTP_12, msg_PTP_2, msg_PTP_21, msg_PTP_22, msg_PTP_3, msg_start_sequence, msg_PTP_1, msg_PTP_11, msg_PTP_12, msg_PTP_2, msg_PTP_21, msg_PTP_22, msg_PTP_3, msg_end_sequence]
        # self.messages = [msg_PTP_1, msg_PTP_2, msg_PTP_1_slow, msg_PTP_2_slow]
        # self.messages = [msg_PTP_1, msg_LIN_1, msg_LIN_2, msg_LIN_1_slow, msg_LIN_2_slow]
        # self.messages = [msg_PTP_2, msg_moveC_1, msg_PTP_2_slow, msg_moveC_1_slow]
        # self.messages = [msg_LIN_2, msg_LIN_3, msg_LIN_1, msg_LIN_4, msg_LIN_5, msg_start_sequence, msg_LIN_2, msg_LIN_3, msg_LIN_1, msg_LIN_4, msg_LIN_5, msg_end_sequence]
        self.messages = [msg_LIN_2, msg_LIN_3, msg_LIN_4, msg_LIN_5, msg_start_sequence, msg_LIN_2, msg_LIN_3, msg_LIN_4, msg_LIN_5, msg_end_sequence]
        # self.messages = [msg_PTP_1, msg_PTP_2, msg_PTP_3, msg_PTP_2, msg_PTP_1, msg_start_sequence, msg_PTP_2, msg_PTP_3, msg_PTP_2, msg_PTP_1,  msg_end_sequence]
        self.get_logger().info(f"Number of messages: {len(self.messages)}")

        # Initial delay before the first message
        self.initial_delay = 2.0  # in seconds
        time.sleep(self.initial_delay)
        self.send_all_messages()

    def send_all_messages(self):
        for index, msg in enumerate(self.messages):
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published message {index + 1}: {msg}")
        self.get_logger().info("All messages sent, shutdown with Ctrl+C")


def main(args=None):
    rclpy.init(args=args)
    node = MotionPublisher()

    try:
        rclpy.spin(node)  # Keeps the node running
    except KeyboardInterrupt:
        pass  # Allows clean shutdown with Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()