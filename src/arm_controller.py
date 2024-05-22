#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
from brl_pxh_api.brl_pxh_client import BrlPxhClient
import math


class ArmController:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.arm_client = BrlPxhClient()
        self.INIT_DIST = 0.22
        self.INIT_HEIGHT = 0.1
        self.PLACE_HEIGHT_OFFSET = 0.045
        self.DEBUG = True

    def pick_up(self, cargo_frame_name):
        rospy.loginfo('**************************************')
        rospy.loginfo(f'Picking up: {cargo_frame_name}')
        rospy.loginfo('**************************************')
        self._init_arm_pose()
        (height, dist, yaw) = self._get_pick_args(cargo_frame_name)
        self._pick(height, dist, yaw)
        self.arm_client.brl_go_to_sleep_pose(moving_time=1.25)

    def place_at(self, station_frame_name):
        rospy.loginfo('**************************************')
        rospy.loginfo(f'Placing cargo at {station_frame_name}')
        rospy.loginfo('**************************************')
        self._init_arm_pose()
        (height, dist, yaw) = self._get_place_args(station_frame_name)
        self._place(height, dist, yaw)
        self.arm_client.brl_set_ee_cartesian_trajectory(z=0.05, moving_time=0.5)
        self.arm_client.brl_go_to_sleep_pose()

    def _init_arm_pose(self):
        self.arm_client.brl_go_to_sleep_pose()
        self.arm_client.brl_set_ee_pose_components(
                x=self.INIT_DIST,
                z=self.INIT_HEIGHT)

    def _get_pick_args(self, cargo_frame_name):
        tf_base_to_pickup = self.tf_buffer.lookup_transform(
                'px100/base_link',
                cargo_frame_name,
                rospy.Time()).transform
        dist = self._get_dist_base_orig_to_dest_orig(tf_base_to_pickup)
        yaw = self._get_yaw(tf_base_to_pickup.translation.y, dist)
        return tf_base_to_pickup.translation.z, dist, yaw

    def _pick(self, height, dist, yaw):
        self._open_gripper()
        self._move_robot_waist(yaw)
        self._move_robot_limbs(dist, height)
        self._close_gripper()

    def _get_place_args(self, station_frame_name):
        tf_base_to_place = self.tf_buffer.lookup_transform(
                'px100/base_link',
                station_frame_name,
                rospy.Time()).transform
        dist = self._get_dist_base_orig_to_dest_orig(tf_base_to_place)
        yaw = self._get_yaw(tf_base_to_place.translation.y, dist)
        return tf_base_to_place.translation.z, dist, yaw

    def _place(self, height, dist, yaw):
        self._move_robot_waist(yaw)
        self._move_robot_limbs(dist,
                height + self.PLACE_HEIGHT_OFFSET)
        self._open_gripper()
    
    def _get_dist_base_orig_to_dest_orig(self, tf_base_to_dest):
        tr_base_to_dest = [tf_base_to_dest.translation.x,
                tf_base_to_dest.translation.y,
                tf_base_to_dest.translation.z]
        result = np.linalg.norm(np.array(tr_base_to_dest))
        if self.DEBUG:
            rospy.loginfo(f'Distance from base-frame origin to destination-frame origin: {result}')
        return result

    def _get_yaw(self, opposite, hypotenuse):
        if self.DEBUG:
            rospy.loginfo(f'Opposite length: {opposite}')
        yaw = math.asin(abs(opposite) / hypotenuse)
        if opposite < 0:
            yaw = -1 * yaw
        if self.DEBUG:
            rospy.loginfo(f'Yaw: {yaw}')
        return yaw

    def _move_robot_waist(self, yaw):
        self.arm_client.brl_set_single_joint_position('waist',
                yaw)

    def _move_robot_limbs(self,
            x_from_base,
            z_from_base):
        self.arm_client.brl_set_ee_cartesian_trajectory(
                x=x_from_base - self.INIT_DIST,
                z=z_from_base - self.INIT_HEIGHT)

    def _open_gripper(self):
        self.arm_client.brl_open_gripper()

    def _close_gripper(self):
        self.arm_client.brl_close_gripper()

