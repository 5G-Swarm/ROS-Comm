#!/usr/bin/env python
from time import sleep
import numpy as np
import rospy
from rospy.core import xmlrpcapi
import std_msgs.msg
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from dislam.msg import DiSCO

import base64
import json
from informer import Informer
from proto.python_out import DiSLAM_pb2
from config import cfg_robot2
robot_id = '999999' # server ID
dest = '111111' # to robot ID

x_array = None
y_array = None
z_array = None
sig_array = None

x_flag = False
y_flag = False
z_flag = False
sig_flag = False

class Server(Informer):
    def parse_message(self, message):
        # global x_array, y_array, z_array, sig_array, x_flag, y_flag, z_flag, sig_flag
        # message_type = message['Mtype']
        # pri = message['Pri']
        # robot_id = message['Id']
        # dest_id = message['Dest']
        # data = message['Data']
        # # print('Get data:', message_type, robot_id, len(data))
        # data = bytes(data, 'utf-8')
        # data = base64.b64decode(data)
        # data = np.frombuffer(data, dtype=np.float64)
        # if message_type == 'x':
        #     x_array = data
        #     x_flag = True
        # elif message_type == 'y':
        #     y_array = data
        #     y_flag = True
        # elif message_type == 'z':
        #     z_array = data
        #     z_flag = True
        # elif message_type == 'sig':
        #     sig_array = data
        #     sig_flag = True
        msg = DiSLAM_pb2.DiSCO()
        msg.ParseFromString(message)
        print("Get msg:", msg)

# Receive
ifm = Server(cfg_robot2, block=True)

def broadcaster():
    global x_array, y_array, z_array, sig_array, x_flag, y_flag, z_flag, sig_flag
    pub = rospy.Publisher('disco', DiSCO, queue_size=0)
    rospy.init_node('broadcast2')

    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        if x_flag and y_flag and z_flag and sig_flag:
            #filling pointcloud header
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            disco_msg = DiSCO()
            disco_msg.signature = sig_array

            points = np.stack([x_array, y_array, z_array]).T
            cloud_points = points.tolist()
            pc_msg = pc2.create_cloud_xyz32(header, cloud_points)
            disco_msg.LocalMap = pc_msg

            #pub.publish(disco_msg)
            rospy.loginfo("pub disco")
            x_flag = y_flag = z_flag = sig_flag = False

        rate.sleep()

if __name__ == '__main__':
    while True:
        sleep(1)
