#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

def callback_LaserScan(data):
    laserVec = data.ranges
    pub = rospy.Publisher("/closest_point", Point, queue_size=10)
    closest_point = Point()

    angle_min = data.angle_min
    angle_max = data.angle_max
    range_min = data.range_min
    range_max = data.range_max
    angle_increment = data.angle_increment

    thetaVec = list()
    thetaVec.append(angle_min)
    k = 0
    for x in laserVec:
		thetaVec.append(thetaVec[k]+angle_increment)
		k = k + 1
    
    #Compute the minimum distance and get the direction of the correspondent point
    delta_m = 1000000
    k_m = -1
    for k in range(len(laserVec)):
    	if(laserVec[k] < delta_m and laserVec[k] > 0.10):
			delta_m = laserVec[k]
			k_m = k

	#Compute the associated direction in the body frame
	phi_m = data.angle_min + k_m*angle_increment
    closest_point.x = delta_m * math.cos(phi_m)
    closest_point.y = delta_m * math.sin(phi_m)
    last_laser_data = rospy.get_time()
    pub.publish(closest_point)

    return

def listener():
    rospy.init_node('closest_point_node', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback_LaserScan)
    rospy.spin()

if __name__ == '__main__':
    listener()