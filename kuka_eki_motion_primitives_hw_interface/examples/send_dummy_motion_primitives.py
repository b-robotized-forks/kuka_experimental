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
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from industrial_robot_motion_interfaces.msg import MotionPrimitive, MotionArgument, MotionSequence
from industrial_robot_motion_interfaces.action import ExecuteMotion
from action_msgs.srv import CancelGoal
import threading
import sys


PTP_1 = MotionPrimitive()
PTP_1.type = MotionPrimitive.LINEAR_JOINT
PTP_1.joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_1.blend_radius = 0.1
PTP_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

PTP_1_slow = MotionPrimitive()
PTP_1_slow.type = MotionPrimitive.LINEAR_JOINT
PTP_1_slow.joint_positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_1_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]

PTP_11 = MotionPrimitive()
PTP_11.type = MotionPrimitive.LINEAR_JOINT
PTP_11.joint_positions = [0.15, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_11.blend_radius = 0.1
PTP_11.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

PTP_12 = MotionPrimitive()
PTP_12.type = MotionPrimitive.LINEAR_JOINT
PTP_12.joint_positions = [0.3, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_12.blend_radius = 0.1
PTP_12.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

PTP_2 = MotionPrimitive()
PTP_2.type = MotionPrimitive.LINEAR_JOINT
PTP_2.joint_positions = [0.5, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_2.blend_radius = 0.1
PTP_2.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

PTP_2_slow = MotionPrimitive()
PTP_2_slow.type = MotionPrimitive.LINEAR_JOINT
PTP_2_slow.joint_positions = [0.5, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_2_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]

PTP_21 = MotionPrimitive()
PTP_21.type = MotionPrimitive.LINEAR_JOINT
PTP_21.joint_positions = [0.65, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_21.blend_radius = 0.1
PTP_21.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

PTP_22 = MotionPrimitive()
PTP_22.type = MotionPrimitive.LINEAR_JOINT
PTP_22.joint_positions = [0.8, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_22.blend_radius = 0.1
PTP_22.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

PTP_3 = MotionPrimitive()
PTP_3.type = MotionPrimitive.LINEAR_JOINT
PTP_3.joint_positions = [1.0, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_3.blend_radius = 0.1
PTP_3.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.5),
    MotionArgument(argument_name="acceleration", argument_value=0.5)]

LIN_1 = MotionPrimitive()
LIN_1.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L1 = PoseStamped()
pose_L1.pose.position.x = 0.240
pose_L1.pose.position.y = 0.0
pose_L1.pose.position.z = 0.550
pose_L1.pose.orientation.x = 0.0
pose_L1.pose.orientation.y = 1.0 
pose_L1.pose.orientation.z = 0.0 
pose_L1.pose.orientation.w = 0.0 
LIN_1.poses = [pose_L1]
LIN_1.blend_radius = 0.1
LIN_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

LIN_1_slow = MotionPrimitive()
LIN_1_slow.type = MotionPrimitive.LINEAR_CARTESIAN
LIN_1_slow.poses = [pose_L1]
LIN_1_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]

LIN_2 = MotionPrimitive()
LIN_2.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L2 = PoseStamped()
pose_L2.pose.position.x = 0.180
pose_L2.pose.position.y = 0.140
pose_L2.pose.position.z = 0.550
pose_L2.pose.orientation.x = 0.0
pose_L2.pose.orientation.y = 1.0 
pose_L2.pose.orientation.z = 0.0 
pose_L2.pose.orientation.w = 0.0 
LIN_2.poses = [pose_L2]
LIN_2.blend_radius = 0.1
LIN_2.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

LIN_2_slow = MotionPrimitive()
LIN_2_slow.type = MotionPrimitive.LINEAR_CARTESIAN
LIN_2_slow.poses = [pose_L2]
LIN_2_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]

LIN_3 = MotionPrimitive()
LIN_3.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L3 = PoseStamped()
pose_L3.pose.position.x = 0.240
pose_L3.pose.position.y = 0.140
pose_L3.pose.position.z = 0.550
pose_L3.pose.orientation.x = 0.0
pose_L3.pose.orientation.y = 1.0 
pose_L3.pose.orientation.z = 0.0 
pose_L3.pose.orientation.w = 0.0 
LIN_3.poses = [pose_L3]
LIN_3.blend_radius = 0.1
LIN_3.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

LIN_4 = MotionPrimitive()
LIN_4.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L4 = PoseStamped()
pose_L4.pose.position.x = 0.240
pose_L4.pose.position.y = -0.140
pose_L4.pose.position.z = 0.550
pose_L4.pose.orientation.x = 0.0
pose_L4.pose.orientation.y = 1.0 
pose_L4.pose.orientation.z = 0.0 
pose_L4.pose.orientation.w = 0.0 
LIN_4.poses = [pose_L4]
LIN_4.blend_radius = 0.1
LIN_4.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

LIN_5 = MotionPrimitive()
LIN_5.type = MotionPrimitive.LINEAR_CARTESIAN
pose_L5 = PoseStamped()
pose_L5.pose.position.x = 0.180
pose_L5.pose.position.y = -0.140
pose_L5.pose.position.z = 0.550
pose_L5.pose.orientation.x = 0.0
pose_L5.pose.orientation.y = 1.0 
pose_L5.pose.orientation.z = 0.0 
pose_L5.pose.orientation.w = 0.0 
LIN_5.poses = [pose_L5]
LIN_5.blend_radius = 0.1
LIN_5.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

moveC_1 = MotionPrimitive()
moveC_1.type = MotionPrimitive.CIRCULAR_CARTESIAN
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
moveC_1.poses = [pose_C1_goal, pose_C1_via] 
moveC_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=2.0),
    MotionArgument(argument_name="acceleration", argument_value=2.0)]

moveC_1_slow = MotionPrimitive()
moveC_1_slow.type = MotionPrimitive.CIRCULAR_CARTESIAN
moveC_1_slow.poses = [pose_C1_goal, pose_C1_via] 
moveC_1_slow.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=0.1),
    MotionArgument(argument_name="acceleration", argument_value=0.1)]

# Motions to compare moprim driver and rsi driver
eval_vel = 0.5
eval_acc = 0.5
eval_blend_radius = 0.0

PTP_eval_0 = MotionPrimitive()
PTP_eval_0.type = MotionPrimitive.LINEAR_JOINT
PTP_eval_0.joint_positions = [0, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_eval_0.blend_radius = eval_blend_radius
PTP_eval_0.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=eval_vel),
    MotionArgument(argument_name="acceleration", argument_value=eval_acc)]

PTP_eval_1 = MotionPrimitive()
PTP_eval_1.type = MotionPrimitive.LINEAR_JOINT
PTP_eval_1.joint_positions = [0, -1.1, 1.0, 0.0, 1.57, 0.0]
PTP_eval_1.blend_radius = eval_blend_radius
PTP_eval_1.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=eval_vel),
    MotionArgument(argument_name="acceleration", argument_value=eval_acc)]

PTP_eval_2 = MotionPrimitive()
PTP_eval_2.type = MotionPrimitive.LINEAR_JOINT
PTP_eval_2.joint_positions = [0.7, -0.9, 0.7, 0.0, 1.57, 0.0]
PTP_eval_2.blend_radius = eval_blend_radius
PTP_eval_2.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=eval_vel),
    MotionArgument(argument_name="acceleration", argument_value=eval_acc)]

PTP_eval_3 = MotionPrimitive()
PTP_eval_3.type = MotionPrimitive.LINEAR_JOINT
PTP_eval_3.joint_positions = [1.57, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_eval_3.blend_radius = eval_blend_radius
PTP_eval_3.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=eval_vel),
    MotionArgument(argument_name="acceleration", argument_value=eval_acc)]

PTP_eval_4 = MotionPrimitive()
PTP_eval_4.type = MotionPrimitive.LINEAR_JOINT
PTP_eval_4.joint_positions = [0, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_eval_4.blend_radius = eval_blend_radius
PTP_eval_4.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=eval_vel),
    MotionArgument(argument_name="acceleration", argument_value=eval_acc)]

PTP_eval_5 = MotionPrimitive()
PTP_eval_5.type = MotionPrimitive.LINEAR_JOINT
PTP_eval_5.joint_positions = [0, -1.1, 1.0, 0.0, 1.57, 0.0]
PTP_eval_5.blend_radius = eval_blend_radius
PTP_eval_5.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=eval_vel),
    MotionArgument(argument_name="acceleration", argument_value=eval_acc)]

PTP_eval_6 = MotionPrimitive()
PTP_eval_6.type = MotionPrimitive.LINEAR_JOINT
PTP_eval_6.joint_positions = [-0.7, -0.9, 0.7, 0.0, 1.57, 0.0]
PTP_eval_6.blend_radius = eval_blend_radius
PTP_eval_6.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=eval_vel),
    MotionArgument(argument_name="acceleration", argument_value=eval_acc)]

PTP_eval_7 = MotionPrimitive()
PTP_eval_7.type = MotionPrimitive.LINEAR_JOINT
PTP_eval_7.joint_positions = [-1.57, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_eval_7.blend_radius = eval_blend_radius
PTP_eval_7.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=eval_vel),
    MotionArgument(argument_name="acceleration", argument_value=eval_acc)]

PTP_eval_8 = MotionPrimitive()
PTP_eval_8.type = MotionPrimitive.LINEAR_JOINT
PTP_eval_8.joint_positions = [0, -1.57, 1.57, 0.0, 1.57, 0.0]
PTP_eval_8.blend_radius = eval_blend_radius
PTP_eval_8.additional_arguments = [
    MotionArgument(argument_name="velocity", argument_value=eval_vel),
    MotionArgument(argument_name="acceleration", argument_value=eval_acc)]

class ExecuteMotionClient(Node):
    def __init__(self):
        super().__init__("motion_sequence_client")

        # Initialize action client for ExecuteMotion action
        self._client = ActionClient(
            self, ExecuteMotion, "/motion_primitive_controller/motion_sequence")

        # Initialize client for cancel_goal service
        self._cancel_client = self.create_client(
            CancelGoal, "/motion_primitive_controller/motion_sequence/_action/cancel_goal")

        self._goal_id = None  # To store the goal ID for cancellation

        # Send the motion goal
        self._send_goal()

        # Start a thread to listen for ENTER key press to cancel the goal
        thread = threading.Thread(target=self._wait_for_keypress, daemon=True)
        thread.start()

    def _send_goal(self):
        """Send the motion sequence goal to the action server."""
        self.get_logger().info("Waiting for action server...")
        self._client.wait_for_server()

        goal_msg = ExecuteMotion.Goal()
        goal_msg.trajectory = MotionSequence()

        # Different Primitives
        # goal_msg.trajectory.motions = [PTP_1, PTP_2, PTP_3, PTP_2, LIN_1, LIN_2, PTP_1, PTP_2, PTP_3, LIN_1, LIN_2, moveC_1, PTP_1, PTP_2]
        
        # Fast vs slow movements
        # goal_msg.trajectory.motions = [PTP_2, moveC_1, PTP_2_slow, moveC_1_slow]

        # Rectangle with LIN movements
        # goal_msg.trajectory.motions = [[LIN_2, LIN_3, LIN_4, LIN_5, LIN_2, LIN_3, LIN_4, LIN_5]

        # Small PTP movements to check blending
        # goal_msg.trajectory.motions = [PTP_1, PTP_11, PTP_12, PTP_2, PTP_21, PTP_22, PTP_3]

        # Evaluation sequence with PTP movements (to compare moprim driver and rsi driver)
        goal_msg.trajectory.motions = [PTP_eval_0, PTP_eval_1, PTP_eval_2, PTP_eval_3, PTP_eval_4, PTP_eval_5, PTP_eval_6, PTP_eval_7, PTP_eval_8]

        self.get_logger().info(f"Sending {len(goal_msg.trajectory.motions)} motion primitives as a sequence...")
        send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback called when the action server accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._goal_id = goal_handle.goal_id  # Store goal ID for cancellation

        # Wait for result asynchronously
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receive feedback about the current motion primitive being executed."""
        current_index = feedback_msg.feedback.current_primitive_index
        self.get_logger().info(f"Executing primitive index: {current_index}")

    def result_callback(self, future):
        """Handle the result from the action server after goal finishes or is canceled."""
        result = future.result().result
        if result.error_code == ExecuteMotion.Result.SUCCESSFUL:
            self.get_logger().info("Motion sequence executed successfully!")
        elif result.error_code == ExecuteMotion.Result.CANCELED:
            self.get_logger().info("Motion sequence was canceled.")
        elif result.error_code == ExecuteMotion.Result.FAILED:
            self.get_logger().error("Motion sequence execution failed.")
        else:
            self.get_logger().error(f"Execution failed: {result.error_code} - {result.error_string}")
        rclpy.shutdown()

    def _wait_for_keypress(self):
        """Wait for the user to press ENTER key to cancel the motion sequence."""
        print("Press ENTER to cancel the motion sequence...")
        while True:
            input_str = sys.stdin.readline().strip()
            if input_str == "":
                self.get_logger().info("ENTER key pressed: sending cancel request.")
                self.cancel_goal()
                break

    def cancel_goal(self):
        """Send a cancel request for the currently running goal."""
        if self._goal_id is None:
            self.get_logger().warn("No goal to cancel (goal_id not set yet).")
            return

        if not self._cancel_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Cancel service is not available.")
            return

        request = CancelGoal.Request()
        request.goal_info.goal_id = self._goal_id

        future = self._cancel_client.call_async(request)
        future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """Handle the response from the cancel service call."""
        try:
            response = future.result()
            if response.return_code == 0:
                self.get_logger().info("Cancel request accepted.")
            elif response.return_code == 1:
                self.get_logger().warn("Cancel request rejected.")
            else:
                self.get_logger().warn(f"Cancel returned code: {response.return_code}")
        except Exception as e:
            self.get_logger().error(f"Failed to call cancel service: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ExecuteMotionClient()
    rclpy.spin(node)


if __name__ == "__main__":
    main()