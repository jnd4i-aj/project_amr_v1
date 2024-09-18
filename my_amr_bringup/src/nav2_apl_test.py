#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from angle_transformations import TransformAngle

class NAV2API():
    def __init__(self) -> None:
        self.initial_pose = PoseStamped()

    def set_initial_pose(self):
        rclpy.init()
        nav = BasicNavigator()

        angle_in_quat = TransformAngle().euler_to_quaternion(roll=0.0,
                                                             pitch=0.0,
                                                             yaw=0.0)
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = nav.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = -3.87
        self.initial_pose.pose.position.y = -0.35
        self.initial_pose.pose.position.z = 0.0

        self.initial_pose.pose.orientation.x = angle_in_quat[0]
        self.initial_pose.pose.orientation.y = angle_in_quat[1]
        self.initial_pose.pose.orientation.z = angle_in_quat[2]
        self.initial_pose.pose.orientation.w = angle_in_quat[3]

        nav.setInitialPose(self.initial_pose)

        nav.waitUntilNav2Active()

        rclpy.shutdown()

if __name__ == '__main__':
    NAV2API().set_initial_pose()

