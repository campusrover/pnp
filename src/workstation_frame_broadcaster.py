#!/usr/bin/env python3

import os
import json
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class WorkstationFrameBroadcaster:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.stations = self._get_station_configs()

    def run(self):
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            self._broadcast_frames()
            rate.sleep()

    def _get_station_configs(self):
        abs_path_to_cur_file = os.path.dirname(__file__)
        config_file_path = os.path.join(abs_path_to_cur_file,
                '../config/config.json')
        with open(config_file_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
            stations = config['stations']
            return stations

    def _broadcast_frames(self):
        for station in self.stations:
            try:
                new_wsf = TransformStamped()
                parent_fid = f'fiducial_{station["fid_id"]}'                  
                cf_id = station['station_id']

                self._set_new_ws_header_and_child_frame_id(new_wsf,
                        cf_id,
                        parent_fid)
                self._set_new_ws_translation(new_wsf, parent_fid)
                self._set_new_ws_rotation(new_wsf, parent_fid)
                
                self.tf_broadcaster.sendTransform(new_wsf)
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException, 
                    tf2_ros.ConnectivityException):
                continue

    def _set_new_ws_header_and_child_frame_id(self, 
            new_wsf,
            cf_id,
            parent_fid):
        new_wsf.header.stamp = rospy.Time.now()
        new_wsf.header.frame_id = parent_fid 
        new_wsf.child_frame_id = cf_id

    def _set_new_ws_translation(self, new_wsf, parent_fid):
        tf_fid = self.tf_buffer.lookup_transform(parent_fid, parent_fid, rospy.Time()).transform
        new_wsf.transform.translation = tf_fid.translation
        new_wsf.transform.translation.x = tf_fid.translation.x
        new_wsf.transform.translation.y = tf_fid.translation.y

    def _set_new_ws_rotation(self, new_wsf, parent_fid):
        tf_fid_to_base_link = self.tf_buffer.lookup_transform(
                parent_fid,
                'px100/base_link',
                rospy.Time()).transform 
        new_wsf.transform.rotation = tf_fid_to_base_link.rotation

if __name__ == '__main__':
    rospy.init_node('workstation_frame_broadcaster')
    try:
        wfb = WorkstationFrameBroadcaster()
        wfb.run()
    except LookupError as e:
        rospy.loginfo(e)

