#!/usr/bin/env python3

import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class FixedMarkerFrameBroadcaster:
    
    def __init__(self):
        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size = 1)
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            ts = TransformStamped()
            ts.header.frame_id = "world"
            ts.header.stamp = rospy.Time.now()
            ts.child_frame_id = "fixed_marker"
            ts.transform.translation.x = 0.0
            ts.transform.translation.y = -0.11049
            ts.transform.translation.z = 0.0

            ts.transform.rotation.x = 0.0
            ts.transform.rotation.y = 0.0
            ts.transform.rotation.z = 0.0
            ts.transform.rotation.w = 1.0

            tfm = TFMessage([ts])
            self.tf_pub.publish(tfm)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fixed_marker_frame_broadcaster')
    FixedMarkerFrameBroadcaster()

