#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
from os.path import join, dirname
sys.path.insert(0, join(dirname(__file__), '../'))

import cv2
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import time
from informer import Informer
from proto.python_out import marker_pb2
from config import cfg_robot1

class Client(Informer):
    def send_msg(self, message):
        self.send(message, 'msg')

# Sender
ifm = Client(cfg_robot1)


def ros2pb(ros_marker):
    marker = marker_pb2.Marker()
    marker.id = ros_marker.id
    marker.position.x = ros_marker.position.x
    marker.position.y = ros_marker.position.y
    marker.position.z = ros_marker.position.z
    marker.orientation.x = ros_marker.orientation.x
    marker.orientation.y = ros_marker.orientation.y
    marker.orientation.z = ros_marker.orientation.z
    marker.orientation.w = ros_marker.orientation.w
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
        mark = ros2pb(ros_mark)
        marker_list.append(mark)
    return marker_list

def callback_mark_array(ros_marker_array):
    marker_list = parse_ros_marker_list(ros_marker_array)
    sent_data = marker_list.SerializeToString()
    ifm.send_msg(sent_data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/detection/lidar_detector/tracking_objects', MarkerArray, callback_mark_array)
    rospy.spin()

    # while True:
    #     callback_mark(ros_marker = None)

if __name__ == '__main__':
    listener()