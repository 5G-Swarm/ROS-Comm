#!/usr/bin/env python

import numpy as np
from geometry_msgs.msg import Point, Point32
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2
from dislam.msg import DiSCO

from time import sleep
import base64
import json
from informer import Informer, config
config.PUBLICT_IP = '42.192.10.208'#'123.56.58.245'
robot_id = '222222' # robot ID
dest = '000000' # anything
# Sender
ifm = Informer(robot_id, is_server=False, block=True)

def get_data(array, Mtype='x'):
    send_data = {'Mtype':Mtype,
                'Pri':5, 'Id':robot_id,
                'Dest':dest,
                'Data':str(base64.b64encode(array.tobytes()), 'utf-8')}
    send_data = json.dumps(send_data)
    return send_data.encode('utf-8')

sleep_time = 0.07#15
def callback(data):
    # simple conversion
    sig = np.array(data.signature).astype(np.float64)

    pc = pc2.read_points(data.LocalMap, skip_nans=True, field_names=("x", "y", "z"))
    pc_list = []
    for p in pc:
        pc_list.append([p[0],p[1],p[2]])
    pc_array = np.array(pc_list).T
    pc_array.astype(np.float64)
    # send_data = get_data(pc_array[0][:200], 'x')
    send_data = get_data(pc_array[0], 'x')
    rospy.loginfo("send data "+str(len(send_data)))
    ifm.send(send_data)
    sleep(sleep_time)
    send_data = get_data(pc_array[1], 'y')
    ifm.send(send_data)
    sleep(sleep_time)
    send_data = get_data(pc_array[2], 'z')
    ifm.send(send_data)
    sleep(sleep_time)
    send_data = get_data(sig, 'sig')
    rospy.loginfo("send data "+str(len(send_data)))
    ifm.send(send_data)
    sleep(sleep_time)

    rospy.loginfo("get disco")

def listener():
    rospy.init_node('listener2', anonymous=True)
    rospy.Subscriber('disco', DiSCO, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()