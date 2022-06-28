#!/usr/bin/env python  
import rospy
import time
import tf
from visualization_msgs.msg import Marker


class ObscaleFrameConverter:
    def __init__(self) -> None:
        rospy.init_node('obs_frame_converter')
        self.box_subscriber = rospy.Subscriber('lxz', Marker, self.marker_callback)
        self.obscale_pose_pub = rospy.Publisher('obscale_pose', Marker, queue_size=1)

    def marker_callback(self, data):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                listener.waitForTransform('slider', 'base_link', time, rospy.Duration(1))
                obscale_pose = listener.transformPose('base_link', data.pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            self.obscale_pose_pub.publish(obscale_pose)
            rate.sleep()
        

if __name__ == '__main__':
    converter = ObscaleFrameConverter()
    rospy.spin()