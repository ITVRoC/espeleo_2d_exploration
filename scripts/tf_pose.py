#!/usr/bin/env python  
import roslib

import rospy
import tf

if __name__ == "__main__":
    rospy.init_node('base_tf_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(5))
            (trans,rot) = listener.lookupTransform("map", "base_link", rospy.Time(0))
            print(trans)
            print(rot)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            continue
        rate.sleep()