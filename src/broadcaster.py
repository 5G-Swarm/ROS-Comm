#!/usr/bin/env python
import numpy as np
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from dislam.msg import DiSCO

def broadcaster():
    pub = rospy.Publisher('disco', DiSCO, queue_size=10)
    rospy.init_node('broadcast')

    #filling pointcloud header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    disco_msg = DiSCO()
    sig = np.random.randn(1024)
    disco_msg.signature = sig

    #filling some points
    points = np.random.randn(4096,3)*20
    cloud_points = points.tolist()
    pc_msg = pc2.create_cloud_xyz32(header, cloud_points)
    disco_msg.LocalMap = pc_msg
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(disco_msg)
        print("pub disco")
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcaster()
    except rospy.ROSInterruptException:
        pass
