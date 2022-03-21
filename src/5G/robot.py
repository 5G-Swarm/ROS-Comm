#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
from os.path import join, dirname
sys.path.insert(0, join(dirname(__file__), '../'))

import cv2
import numpy as np
import rospy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from ros_comm.msg import Pose2DArray

import time
from informer import Informer
from proto.python_out import marker_pb2, geometry_msgs_pb2, path_msgs_pb2
from config_5g import cfg_robot1

global_path = None

def parse_path(message):
    global global_path, path_pub
    path = path_msgs_pb2.Path()
    path.ParseFromString(message)

    ros_pose_array = Pose2DArray()
    for pose in path.poses:
        ros_pose = Pose2D()
        ros_pose.x = pose.x
        ros_pose.y = pose.y
        ros_pose.theta = pose.theta

        ros_pose_array.poses.append(ros_pose)

    path_pub.publish(ros_pose_array)

class Client(Informer):
    def send_msg(self, message):
        self.send(message, 'msg')
    
    def send_odm(self, message):
        self.send(message, 'odm')
    
    def path_recv(self):
        print('get !!')
        self.recv('path', parse_path)

def ros_marker2pb(ros_marker):
    marker = marker_pb2.Marker()
    marker.id = ros_marker.id
    marker.pose.position.x = ros_marker.pose.position.x
    marker.pose.position.y = ros_marker.pose.position.y
    marker.pose.position.z = ros_marker.pose.position.z
    marker.pose.orientation.x = ros_marker.pose.orientation.x
    marker.pose.orientation.y = ros_marker.pose.orientation.y
    marker.pose.orientation.z = ros_marker.pose.orientation.z
    marker.pose.orientation.w = ros_marker.pose.orientation.w
    marker.scale.x = ros_marker.scale.x
    marker.scale.y = ros_marker.scale.y
    marker.scale.z = ros_marker.scale.z
    marker.color.r = ros_marker.color.r
    marker.color.g = ros_marker.color.g
    marker.color.b = ros_marker.color.b
    return marker

def parse_ros_marker_list(ros_marker_array):
    marker_list = marker_pb2.MarkerList()
    for ros_mark in ros_marker_array.markers:
        mark = ros_marker2pb(ros_mark)
        marker_list.marker_list.append(mark)
    return marker_list

def callback_mark_array(ros_marker_array):
    print('get !!')
    marker_list = parse_ros_marker_list(ros_marker_array)
    sent_data = marker_list.SerializeToString()
    print('send', len(sent_data))
    ifm.send_msg(sent_data)


def ros_odometry2pb(odometry):
    pose = geometry_msgs_pb2.Pose()
    pose.position.x = odometry.pose.pose.position.x
    pose.position.y = odometry.pose.pose.position.y
    pose.position.z = odometry.pose.pose.position.z
    pose.orientation.x = odometry.pose.pose.orientation.x
    pose.orientation.y = odometry.pose.pose.orientation.y
    pose.orientation.z = odometry.pose.pose.orientation.z
    pose.orientation.w = odometry.pose.pose.orientation.w
    return pose

def callback_odometry(odometry):
    # print(odometry)
    pose = ros_odometry2pb(odometry)
    sent_data = pose.SerializeToString()
    print('send', len(sent_data))
    ifm.send_odm(sent_data)



if __name__ == '__main__':
    rospy.init_node('5g-transfer', anonymous=True)
    rospy.Subscriber('/detection/lidar_detector/objects_markers', MarkerArray, callback_mark_array)
    rospy.Subscriber('/base2map', Odometry, callback_odometry)

    path_pub = rospy.Publisher('global_path', Pose2DArray, queue_size=0)

    # Sender
    ifm = Client(cfg_robot1)

    # rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
