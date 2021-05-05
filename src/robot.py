#!/usr/bin/env python

import numpy as np
# import rospy
# from dislam.msg import DiSCO

from time import sleep
import base64
from informer import Informer
from proto.python_out import DiSLAM_pb2
from threading import Thread
import random
from config import cfg_robot1, cfg_robot2

# Sender
ifm = Informer(cfg_robot2, block=True)

# def get_data(array, Mtype='x'):
#     send_data = {'Mtype':Mtype,
#                 'Pri':5, 'Id':robot_id,
#                 'Dest':dest,
#                 'Data':str(base64.b64encode(array.tobytes()), 'utf-8')}
#     send_data = json.dumps(send_data)
#     return send_data.encode('utf-8')

verification_num = 0

def serialize_data():
    # info = message_pb2.RobotInfo()
    # info.mType       = mType
    # info.priority    = priority
    # info.id          = robot_id
    # info.destination = str(base64.b64encode(array.tobytes()), 'utf-8')
    # for i in range(array.shape[1]):
    #     pos = info.pos.add()
    #     pos.x = array[0][i]
    #     pos.y = array[1][i]
    #     pos.z = array[2][i]
    info = DiSLAM_pb2.DiSCO()
    global verification_num
    for i in range(1 * 1024 * 1):
        info.fftr.append(verification_num)
        verification_num += 1
        verification_num %= 65535
    # print("ddddds", info)
    return info

# sleep_time = 0.07#15
def callback():
    # # simple conversion
    # sig = np.array(data.signature).astype(np.float64)

    # pc = pc2.read_points(data.LocalMap, skip_nans=True, field_names=("x", "y", "z"))
    # pc_list = []
    # for p in pc:
    #     pc_list.append([p[0],p[1],p[2]])
    # pc_array = np.array(pc_list).T
    # pc_array.astype(np.float64)
    # send_data = get_data(pc_array[0][:200], 'x')
    # send_data = get_data(pc_array[0], 'x')
    # rospy.loginfo("send data "+str(len(send_data)))
    # ifm.send(send_data)
    # sleep(sleep_time)
    # send_data = get_data(pc_array[1], 'y')
    # ifm.send(send_data)
    # sleep(sleep_time)
    # send_data = get_data(pc_array[2], 'z')
    # ifm.send(send_data)
    # sleep(sleep_time)
    # send_data = get_data(sig, 'sig')
    # rospy.loginfo("send data "+str(len(send_data)))
    # ifm.send(send_data)
    # sleep(sleep_time)
    data = serialize_data()
    data = data.SerializeToString()
    # print("ddd", data_len, data[:8])
    
    
    ifm.send(data) # serialize to binary
    print("send data")

    # rospy.loginfo("get disco")

def listener():
    # rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber('disco', DiSCO, callback)
    # rospy.spin()
    while True:
        callback()
        sleep(1/1)


if __name__ == '__main__':
    listener()
