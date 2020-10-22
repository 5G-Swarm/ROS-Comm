#!/usr/bin/env python

import numpy as np
from geometry_msgs.msg import Point, Point32
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2
from dislam.msg import DiSCO

def callback(data):
    # simple conversion
    pc = pc2.read_points(data.LocalMap, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for p in pc:
        pc_list.append([p[0],p[1],p[2]])
    sig = data.signature

    rospy.loginfo("get disco")

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('disco', DiSCO, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
