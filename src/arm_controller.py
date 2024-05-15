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

    def init_arm_pose(self, moving_time=1.25):
        self.arm_client.brl_go_to_sleep_pose(moving_time=moving_time)
        self.arm_client.brl_set_ee_pose_components(
                x=self.INIT_DIST,
                z=self.INIT_HEIGHT,
                moving_time=moving_time)

    def retire_robot(self, moving_time=1.25):
        rospy.loginfo('======================================')
        rospy.loginfo(f'Going to sleep pose')
        rospy.loginfo('======================================')
        self.arm_client.brl_go_to_sleep_pose(moving_time=moving_time)
        rospy.loginfo('======================================')
        rospy.loginfo(f'Went to sleep pose')
        rospy.loginfo('======================================')

    def pick_up(self, cargo_frame_name, moving_time=1.25):
        rospy.loginfo('**************************************')
        rospy.loginfo(f'Picking up: {cargo_frame_name}')
        rospy.loginfo('**************************************')
        self.init_arm_pose(moving_time)
        (height, dist, yaw) = self._get_pick_args(cargo_frame_name)
        self._pick(height, dist, yaw, moving_time)
        self.arm_client.brl_go_to_sleep_pose(moving_time=moving_time)

    def place_at(self, ws_frame_name, moving_time=1.25):
        rospy.loginfo('**************************************')
        rospy.loginfo(f'Placing cargo at {ws_frame_name}')
        rospy.loginfo('**************************************')
        self.init_arm_pose(moving_time)
        (height, dist, yaw) = self._get_place_args(ws_frame_name)
        self._place(height, dist, yaw, moving_time)
        self.arm_client.brl_go_to_sleep_pose(moving_time=moving_time)

    def _get_pick_args(self, cargo_frame_name):
        tf_base_to_pickup = self.tf_buffer.lookup_transform(
                'px100/base_link',
                cargo_frame_name,
                rospy.Time()).transform
        dist = self._get_dist_base_orig_to_dest_orig(tf_base_to_pickup)
        yaw = self._get_yaw(tf_base_to_pickup.translation.y, dist)
        return tf_base_to_pickup.translation.z, dist, yaw

    def _pick(self, height, dist, yaw, moving_time=1.25):
        self._move_robot_waist(yaw, moving_time)
        self._move_robot_limbs(dist, height, moving_time)
        self._open_gripper()
        self._close_gripper()

    def _get_place_args(self, ws_frame_name):
        tf_base_to_place = self.tf_buffer.lookup_transform(
                'px100/base_link',
                ws_frame_name,
                rospy.Time()).transform
        dist = self._get_dist_base_orig_to_dest_orig(tf_base_to_place)
        yaw = self._get_yaw(tf_base_to_place.translation.y, dist)
        return tf_base_to_place.translation.z, dist, yaw

    def _place(self, height, dist, yaw, moving_time=1.25):
        self._move_robot_waist(yaw, moving_time)
        self._move_robot_limbs(dist,
                height + self.PLACE_HEIGHT_OFFSET,
                moving_time)
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

    def _move_robot_waist(self, yaw, moving_time=1.25):
        rospy.loginfo('======================================')
        rospy.loginfo(f'Moving robot waist by: {yaw} radians')
        rospy.loginfo('======================================')
        self.arm_client.brl_set_single_joint_position('waist',
                yaw,
                moving_time=moving_time)
        rospy.loginfo('======================================')
        rospy.loginfo(f'Moved robot waist.')
        rospy.loginfo('======================================')

    def _move_robot_limbs(self,
            x_from_base,
            z_from_base,
            moving_time=1.25):
        rospy.loginfo('======================================')
        rospy.loginfo(f'Moving robot limbs by ({x_from_base - self.INIT_DIST, z_from_base - self.INIT_HEIGHT}) offset from robot\'s base_frame')
        rospy.loginfo('======================================')
        self.arm_client.brl_set_ee_cartesian_trajectory(
                x=x_from_base - self.INIT_DIST,
                z=z_from_base - self.INIT_HEIGHT,
                moving_time=moving_time)
        rospy.loginfo('======================================')
        rospy.loginfo(f'Moved robot\'s limbs.')
        rospy.loginfo('======================================')

    def _open_gripper(self):
        self.arm_client.brl_open_gripper()

    def _close_gripper(self):
        self.arm_client.brl_close_gripper()

