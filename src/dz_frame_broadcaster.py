#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class DzFrameBroadcaster:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def run(self):
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                new_dzf = TransformStamped()
                parent_fid = f'fiducial_5'                  
                cf_id = 'drop_off_zone' 

                self._set_new_dz_header_and_child_frame_id(new_dzf,
                        cf_id,
                        parent_fid)
                self._set_new_dz_translation(new_dzf, parent_fid)
                self._set_new_dz_rotation(new_dzf, parent_fid)
                
                self.tf_broadcaster.sendTransform(new_dzf)
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException, 
                    tf2_ros.ConnectivityException):
                continue
            rate.sleep()

        
    def _set_new_dz_header_and_child_frame_id(self, 
            new_dzf,
            cf_id,
            parent_fid):
        new_dzf.header.stamp = rospy.Time.now()
        new_dzf.header.frame_id = parent_fid 
        new_dzf.child_frame_id = cf_id

    def _set_new_dz_translation(self, new_dzf, parent_fid):
        tf_fid = self.tf_buffer.lookup_transform(parent_fid, parent_fid, rospy.Time()).transform
        new_dzf.transform.translation = tf_fid.translation
        new_dzf.transform.translation.x = tf_fid.translation.x
        new_dzf.transform.translation.y = tf_fid.translation.y

    def _set_new_dz_rotation(self, new_dzf, parent_fid):
        tf_fid_to_base_link = self.tf_buffer.lookup_transform(
                parent_fid,
                'px100/base_link',
                rospy.Time()).transform 
        new_dzf.transform.rotation = tf_fid_to_base_link.rotation

if __name__ == '__main__':
    rospy.init_node('dz_frame_broadcaster')
    try:
        dzfb = DzFrameBroadcaster()
        dzfb.run()
    except LookupError as e:
        rospy.loginfo(e)

