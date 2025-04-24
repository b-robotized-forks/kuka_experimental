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

msg_PTP_1 = MotionPrimitive()
msg_PTP_1.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_1.joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]

msg_PTP_2 = MotionPrimitive()
msg_PTP_2.type = MotionPrimitive.LINEAR_JOINT
msg_PTP_2.joint_positions = [0.7, -1.57, 1.57, 0.0, 1.57, 0.0]



# # send all primitives
# class MotionPublisher(Node):
#     def __init__(self):
#         super().__init__('motion_publisher')

#         self.publisher_ = self.create_publisher(MotionPrimitive, '/motion_primitive_controller/reference', 10)

#         self.messages = [msg_PTP_1, msg_PTP_2]
#         self.get_logger().info(f"Number of messages: {len(self.messages)}")

#         self.send_all_messages()
    

#     def send_all_messages(self):
#         for i, msg in enumerate(self.messages):
#             self.get_logger().info(f"Sending message {i + 1} of {len(self.messages)}")
#             self.publisher_.publish(msg)
#             self.get_logger().info(f"Sent message {i + 1}: {msg}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = MotionPublisher()
#     rclpy.spin_once(node, timeout_sec=1) 

#     # Cleanup
#     node.destroy_node()
#     rclpy.shutdown()


# send alternating motion primitives every 2 seconds
class MotionPublisher(Node):
    def __init__(self):
        super().__init__('motion_publisher')
        self.publisher_ = self.create_publisher(MotionPrimitive, '/motion_primitive_controller/reference', 10)

        # List of motion messages to alternate between
        self.messages = [msg_PTP_1, msg_PTP_2]
        self.current_index = 0

        # Create a timer that triggers every 2 seconds
        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.send_alternating_message)
        self.get_logger().info(f"MotionPublisher started – alternating messages every {timer_period} seconds.")

    def send_alternating_message(self):
        # Publish the current message
        msg = self.messages[self.current_index]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published message {self.current_index + 1}")

        # Switch to the next message in the list
        self.current_index = (self.current_index + 1) % len(self.messages)


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