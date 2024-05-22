#!/usr/bin/env python3

import os
import json
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

class PnpFrameBroadcaster:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.seen_cargoes_pub = rospy.Publisher('seen_cargoes',
                String,
                queue_size = 1)
        self.seen_cargoes = set()
        self.stations, self.cargoes, self.drop_off_zone_fid = self._get_configs()

    def _get_configs(self):
        abs_path_to_cur_file = os.path.dirname(__file__)
        config_file_path = os.path.join(abs_path_to_cur_file,
                '../config/config.json')
        with open(config_file_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
            stations = config['stations']
            cargoes = config['cargoes']
            drop_off_zone_fid = config['drop_off_zone_fiducial']
            return stations, cargoes, drop_off_zone_fid

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self._broadcast_station_frames()
            self._broadcast_dz_frame()
            self._broadcast_cargo_frames()
            rate.sleep()

    def _broadcast_station_frames(self):
        for station in self.stations:
            try:
                self._set_and_broadcast_new_frame(TransformStamped(),
                        station['id'],
                        f'fiducial_{station["fid_id"]}')

            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException, 
                    tf2_ros.ConnectivityException):
                continue

    def _broadcast_dz_frame(self):
            try:
                self._set_and_broadcast_new_frame(TransformStamped(),
                        'drop_off_zone',
                        self.drop_off_zone_fid)

            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException, 
                    tf2_ros.ConnectivityException):
                pass
 
    def _broadcast_cargo_frames(self):
        for cargo in self.cargoes: 
            try:
                cargo_frame_id = cargo['id']

                self._set_and_broadcast_new_frame(TransformStamped(),
                        cargo_frame_id,
                        f'fiducial_{cargo["fid_id"]}')

                if cargo_frame_id not in self.seen_cargoes:
                    self.seen_cargoes_pub.publish(cargo_frame_id);
                    self.seen_cargoes.add(cargo_frame_id)

            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException, 
                    tf2_ros.ConnectivityException):
                continue
                
    def _set_and_broadcast_new_frame(self,
            new_frame,
            new_frame_id,
            parent_frame_id):
        self._set_new_frame_ids(new_frame,
                new_frame_id,
                parent_frame_id)
        self._set_new_frame_translation(new_frame,
                parent_frame_id)
        self._set_new_frame_rotation(new_frame,
                parent_frame_id)
        self.tf_broadcaster.sendTransform(new_frame)

    def _set_new_frame_ids(self,
            new_frame,
            new_frame_id,
            parent_frame_id):
        new_frame.header.stamp = rospy.Time.now()
        new_frame.header.frame_id = parent_frame_id
        new_frame.child_frame_id = new_frame_id

    def _set_new_frame_translation(self,
            new_frame,
            parent_frame_id):
        tf_parent_to_parent = self.tf_buffer.lookup_transform(parent_frame_id,
                parent_frame_id,
                rospy.Time()).transform
        new_frame.transform.translation = tf_parent_to_parent.translation

    def _set_new_frame_rotation(self,
            new_frame,
            parent_frame_id):
        tf_parent_to_base_link = self.tf_buffer.lookup_transform(
                parent_frame_id,
                'px100/base_link',
                rospy.Time()).transform 
        new_frame.transform.rotation = tf_parent_to_base_link.rotation

if __name__ == '__main__':
    rospy.init_node('pnp_frame_broadcaster')
    fb = PnpFrameBroadcaster()
    fb.run()

