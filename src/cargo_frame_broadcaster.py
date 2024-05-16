#!/usr/bin/env python3

import os
import json
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from actionlib import SimpleActionClient
from assembly_pipeline_framework.msg import (
        CargoRegAction,
        CargoRegGoal)

class CargoFrameBroadcaster:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.seen_cargoes_pub = rospy.Publisher('seen_cargoes',
                String,
                queue_size = 1)
        self.seen_cargoes = set()
        self.cargoes, self.first_cargo_fid_id = self._get_cargo_configs()

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self._broadcast_frames()                
            rate.sleep()

    def _get_cargo_configs(self):
        abs_path_to_cur_file = os.path.dirname(__file__)
        config_file_path = os.path.join(abs_path_to_cur_file,
                '../config/config.json')
        with open(config_file_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
            cargoes = config['cargoes']
            first_cargo_fid_id = cargoes[0]['fid_id']
            return cargoes, first_cargo_fid_id

    def _register_new_cargo(self, cf_id):
        goal = CargoRegGoal()
        goal.cargo_frame_id = cf_id
        goal.last_placed = rospy.Time.now()
        self.client_cargo_reg.send_goal(goal)
        self.client_cargo_reg.wait_for_result()
    
    def _broadcast_frames(self):
        for cargo in self.cargoes: 
            try:
                new_cf = TransformStamped()
                parent_fid = f'fiducial_{cargo["fid_id"]}'
                cf_id = cargo['id']

                self._set_new_cf_header_and_child_frame_id(
                        new_cf,
                        parent_fid,
                        cf_id)
                self._set_new_cf_translation(new_cf, parent_fid)
                self._set_new_cf_rotation(new_cf, parent_fid)

                self.tf_broadcaster.sendTransform(new_cf)
                
                if cf_id not in self.seen_cargoes:
                    self.seen_cargoes_pub.publish(cf_id);
                    self.seen_cargoes.add(cf_id)

            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException, 
                    tf2_ros.ConnectivityException):
                continue

    def _set_new_cf_header_and_child_frame_id(self,
            new_cf,
            parent_fid,
            cargo_frame_id):
        new_cf.header.stamp = rospy.Time.now()
        new_cf.header.frame_id = parent_fid
        new_cf.child_frame_id = cargo_frame_id

    def _set_new_cf_translation(self, new_cf, parent_fid):
        tf_cargo = self.tf_buffer.lookup_transform(parent_fid,
                parent_fid,
                rospy.Time()).transform
        new_cf.transform.translation = tf_cargo.translation
        new_cf.transform.translation.x = tf_cargo.translation.x
        new_cf.transform.translation.y = tf_cargo.translation.y

    def _set_new_cf_rotation(self, new_cf, parent_fid):
        tf_parent_fid_to_base_link = self.tf_buffer.lookup_transform(
                parent_fid,
                'px100/base_link',
                rospy.Time()).transform 
        new_cf.transform.rotation = tf_parent_fid_to_base_link.rotation

if __name__ == '__main__':
    rospy.init_node('cargo_frame_broadcaster')
    try:
        cfb = CargoFrameBroadcaster()
        cfb.run()
    except LookupError as e:
        rospy.loginfo(e) 

