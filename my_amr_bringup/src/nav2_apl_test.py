#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from angle_transformations import TransformAngle


class NAV2API():
    def __init__(self) -> None:
        pass

    def __set_pose(self, navigator: BasicNavigator, pose_x, pose_y, orientation_z):

        angle_in_quat = TransformAngle().euler_to_quaternion(roll=0.0,
                                                             pitch=0.0,
                                                             yaw=orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = angle_in_quat[0]
        pose.pose.orientation.y = angle_in_quat[1]
        pose.pose.orientation.z = angle_in_quat[2]
        pose.pose.orientation.w = angle_in_quat[3]

        return pose

    def set_initial_pose(self):

        rclpy.init()
        nav = BasicNavigator()

        # setting initial pose i.e. 2D pose Estimate 
        # initial_pose = self.__set_pose(nav, -3.87, -0.35, 0.0)
        initial_pose = self.__set_pose(nav, -3.75, -0.35, 0.0)
        # nav.setInitialPose(initial_pose)

        # wait for Nav2 to set inital pose
        nav.waitUntilNav2Active()

        # For sending one single goal
        # goal4 = self.__set_pose(nav, 3.7, -0.3, 0.0)
        # nav.goToPose(goal4)

        # while not nav.isTaskComplete():
        #     status = nav.getFeedback()
        #     print(status)

        # For senfing multiple goals together i.e. for Waypoint Follower
        goal1 = self.__set_pose(nav, 1.5, -0.5, 0.0)
        goal2 = self.__set_pose(nav, 3.0, -1.8, 0.0)
        goal3 = self.__set_pose(nav, 4.0, -0.3, 0.0)

        goal5 = self.__set_pose(nav, 3.0, -1.8, 3.14)
        goal6 = self.__set_pose(nav, 1.8, 1.0, 1.57)
        goal7 = self.__set_pose(nav, 2.3, 2.8, 0.0)
        goal8 = self.__set_pose(nav, 3.1, 1.2, -1.57)

        goal9 = self.__set_pose(nav, 1.8, 1.0, -1.57)
        goal10 = self.__set_pose(nav, 1.0, -0.5, 3.14)
        goal11 = self.__set_pose(nav, -2.5, 1.4, 3.14)

        goal12 = self.__set_pose(nav, -2.9, -0.2, -1.57)
        goal13 = self.__set_pose(nav, -1.0, -2.0, -1.57)
        goal14 = self.__set_pose(nav, -2.5, -3.3, 3.14)

        # waypoints_list = [
        #     goal1,
        #     goal2,
        #     goal3
        # ]

        # waypoints_list = [
        #     goal5,
        #     goal6,
        #     goal7
        #     ]

        # waypoints_list = [
        #     goal9,
        #     goal10,
        #     goal11
        #     ]

        waypoints_list = [
            goal12,
            goal13,
            goal14
            ]

        nav.followWaypoints(waypoints_list)

        while not nav.isTaskComplete():
            status = nav.getFeedback()
        #     # print(status)

        print(nav.getResult())

        rclpy.shutdown()

if __name__ == '__main__':
    NAV2API().set_initial_pose()

