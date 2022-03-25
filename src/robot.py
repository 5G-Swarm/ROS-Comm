#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist
from ros_comm.msg import Pose2DArray
from autoware_msgs.msg import TrackingObjectMarker, TrackingObjectMarkerArray

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import time
from informer import Informer
from proto.python_out import marker_pb2, geometry_msgs_pb2, path_msgs_pb2, cmd_msgs_pb2
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

    def send_cmd(self, message):
        self.send(message, 'cmd')
    
    def path_recv(self):
        try:
            self.recv('path', parse_path)
        except:
            print('recv path timeout !')

def ros_marker2pb(ros_marker : TrackingObjectMarker):
    marker = marker_pb2.Marker()
    # type: TrackingObjectMarker
    marker.time_stamp = ros_marker.header.stamp.secs
    marker.id = ros_marker.track_id
    marker.pose.position.x = ros_marker.marker.pose.position.x
    marker.pose.position.y = ros_marker.marker.pose.position.y
    marker.pose.position.z = ros_marker.marker.pose.position.z
    marker.pose.orientation.x = ros_marker.marker.pose.orientation.x
    marker.pose.orientation.y = ros_marker.marker.pose.orientation.y
    marker.pose.orientation.z = ros_marker.marker.pose.orientation.z
    marker.pose.orientation.w = ros_marker.marker.pose.orientation.w
    marker.scale.x = ros_marker.marker.scale.x
    marker.scale.y = ros_marker.marker.scale.y
    marker.scale.z = ros_marker.marker.scale.z
    marker.color.r = ros_marker.marker.color.r
    marker.color.g = ros_marker.marker.color.g
    marker.color.b = ros_marker.marker.color.b
    return marker

def parse_ros_marker_list(ros_marker_array: TrackingObjectMarkerArray):
    marker_list = marker_pb2.MarkerList()
    for ros_mark in ros_marker_array.markers:
        mark = ros_marker2pb(ros_mark)
        marker_list.marker_list.append(mark)
    return marker_list

def callback_mark_array(ros_marker_array: TrackingObjectMarkerArray):
    marker_list = parse_ros_marker_list(ros_marker_array)
    sent_data = marker_list.SerializeToString()
    # print('send', len(sent_data))
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

def ros_cmd2pb(ros_cmd):
    cmd = cmd_msgs_pb2.Cmd()
    cmd.v = ros_cmd.linear.x
    cmd.w = ros_cmd.angular.z
    return cmd

def callback_odometry(odometry):
    # print(odometry)
    pose = ros_odometry2pb(odometry)
    sent_data = pose.SerializeToString()
    # print('send', len(sent_data))
    ifm.send_odm(sent_data)

def callback_cmd(ros_cmd):
    cmd = ros_cmd2pb(ros_cmd)
    sent_data = cmd.SerializeToString()
    ifm.send_cmd(sent_data)

def callback_img(ros_img):
    bridge = CvBridge()
    try:
      cv_image = bridge.imgmsg_to_cv2(ros_img, "bgr8")
    except:
      cv_image = bridge.imgmsg_to_cv2(ros_img)
    print(cv_image.shape)
    # sent_data = cv_image
    # ifm.send_cmd(sent_data)

if __name__ == '__main__':
    rospy.init_node('5g-transfer', anonymous=True)
    ifm = Client(cfg_robot1)
    path_pub = rospy.Publisher('global_path', Pose2DArray, queue_size=0)
    rospy.Subscriber('/detection/lidar_detector/objects_markers_withID', TrackingObjectMarkerArray, callback_mark_array)
    rospy.Subscriber('/base2odometry', Odometry, callback_odometry)
    rospy.Subscriber('/camera/color/image_raw', Image, callback_img)
    rospy.Subscriber('/cmd_vel', Twist, callback_cmd)

    # rospy.spin()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
