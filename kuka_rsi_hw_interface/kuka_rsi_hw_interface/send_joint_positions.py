#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        # self.publisher_ = self.create_publisher(JointTrajectory, '/position_trajectory_controller/joint_trajectory', 10)
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory' , 10)
        time.sleep(1)  # Allow time for publisher to be ready

    def publish_trajectory_startponit(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]
        msg.points = [JointTrajectoryPoint(positions=[0, -1.57, 1.57, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=3).to_msg())]
        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory startpoint published successfully.')

    def publish_trajectory_sequence(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]

        # Define multiple trajectory points
        points = [
            # JointTrajectoryPoint(positions=[0, -1.57, 1.57, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=).to_msg()),
            JointTrajectoryPoint(positions=[0, -1.1, 1.0, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=1).to_msg()),
            JointTrajectoryPoint(positions=[0.7, -0.9, 0.7, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=2).to_msg()),
            JointTrajectoryPoint(positions=[1.57, -1.57, 1.57, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=3).to_msg()),
            JointTrajectoryPoint(positions=[0, -1.57, 1.57, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=4).to_msg()),
            JointTrajectoryPoint(positions=[0, -1.1, 1.0, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=5).to_msg()),
            JointTrajectoryPoint(positions=[-0.7, -0.9, 0.7, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=6).to_msg()),
            JointTrajectoryPoint(positions=[-1.57, -1.57, 1.57, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=7).to_msg()),
            JointTrajectoryPoint(positions=[0, -1.57, 1.57, 0.0, 1.57, 0.0], velocities=[0.0]*6, time_from_start=rclpy.duration.Duration(seconds=8).to_msg()),
        ]

        msg.points = points
        self.publisher_.publish(msg)
        self.get_logger().info('Trajectory sequence published successfully.')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    node.publish_trajectory_startponit()
    time.sleep(4)
    for _ in range(1):
        node.publish_trajectory_sequence()
        time.sleep(8)   # Adjust time to sequence length
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()