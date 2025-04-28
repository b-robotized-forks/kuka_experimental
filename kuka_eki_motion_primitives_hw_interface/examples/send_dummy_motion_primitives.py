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

### testing commands for r2e cell (in degrees)    A7 needs to be set?
# _client.move(joints=[0.0, -90.0, 90.0, 0.0, 90.0, 0.0, 0.0])  #position1
# _client.move(joints=[30.0, -90.0, 90.0, 0.0, 90.0, 0.0, 0.0]) #position2
# _client.move(cartesian=[280.0, 0.0, 550.0, -180.0, 0.0, 180.0], lin=False)    #position1
# _client.move(cartesian=[240.0, -140.0, 550.0, -180.0, 0.0, 180.0], lin=False) #position2
# _client.move(cartesian=[280.0, 0.0, 550.0, -180.0, 0.0, 180.0], lin=True) #position1
# _client.move(cartesian=[240.0, -140.0, 550.0, -180.0, 0.0, 180.0], lin=True)  #position2
msg_start_sequence = MotionPrimitive()
msg_start_sequence.type = MotionPrimitive.MOTION_SEQUENCE_START

msg_end_sequence = MotionPrimitive()
msg_end_sequence.type = MotionPrimitive.MOTION_SEQUENCE_END

msg_PTP_1 = MotionPrimitive()
msg_PTP_1.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_1.joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]

msg_PTP_2 = MotionPrimitive()
msg_PTP_2.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_2.joint_positions = [0.5, -1.57, 1.57, 0.0, 1.57, 0.0]

msg_LIN_1 = MotionPrimitive()
msg_LIN_1.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L1 = PoseStamped()
pose_L1.pose.position.x = 0.280
pose_L1.pose.position.y = 0.0
pose_L1.pose.position.z = 0.550
pose_L1.pose.orientation.x = 0.0
pose_L1.pose.orientation.y = 0.0 
pose_L1.pose.orientation.z = 0.0 
pose_L1.pose.orientation.w = 0.0 
msg_LIN_1.poses = [pose_L1]

msg_LIN_2 = MotionPrimitive()
msg_LIN_2.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L2 = PoseStamped()
pose_L2.pose.position.x = 0.240
pose_L2.pose.position.y = -0.140
pose_L2.pose.position.z = 0.550
pose_L2.pose.orientation.x = 0.0
pose_L2.pose.orientation.y = 0.0 
pose_L2.pose.orientation.z = 0.0 
pose_L2.pose.orientation.w = 0.0 
msg_LIN_2.poses = [pose_L2]




# send alternating motion primitives every 2 seconds
# class MotionPublisher(Node):
#     def __init__(self):
#         super().__init__('motion_publisher')
#         self.publisher_ = self.create_publisher(MotionPrimitive, '/motion_primitive_controller/reference', 10)

#         # List of motion messages to alternate between
#         self.messages = [msg_PTP_1, msg_PTP_2]
#         self.current_index = 0

#         # Create a timer that triggers every 2 seconds
#         timer_period = 2.0
#         self.timer = self.create_timer(timer_period, self.send_alternating_message)
#         self.get_logger().info(f"MotionPublisher started – alternating messages every {timer_period} seconds.")

#     def send_alternating_message(self):
#         # Publish the current message
#         msg = self.messages[self.current_index]
#         self.publisher_.publish(msg)
#         self.get_logger().info(f"Published message {self.current_index + 1}")

#         # Switch to the next message in the list
#         self.current_index = (self.current_index + 1) % len(self.messages)


# def main(args=None):
#     rclpy.init(args=args)
#     node = MotionPublisher()

#     try:
#         rclpy.spin(node)  # Keeps the node running
#     except KeyboardInterrupt:
#         pass  # Allows clean shutdown with Ctrl+C
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# send all motion primitives directly one after the other
class MotionPublisher(Node):
    def __init__(self):
        super().__init__('motion_publisher')

        self.publisher_ = self.create_publisher(MotionPrimitive, '/motion_primitive_controller/reference', 10)

        # self.messages = [msg_PTP_1, msg_PTP_2, msg_LIN_1, msg_LIN_2]
        self.messages = [msg_start_sequence, msg_PTP_1, msg_PTP_2, msg_LIN_1, msg_LIN_2, msg_end_sequence]
        self.current_index = 0

        self.get_logger().info(f"Number of messages: {len(self.messages)}")

        self.timer = self.create_timer(0.5, self.send_next_message)

    def send_next_message(self):
        if self.current_index < len(self.messages):
            msg = self.messages[self.current_index]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published message {self.current_index + 1}: {msg}")

            self.current_index += 1
        else:
            self.get_logger().info("All messages sent, shutting down node.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MotionPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()