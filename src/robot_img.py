#!/usr/bin/env python
import cv2
import numpy as np
# import rospy
# from dislam.msg import DiSCO
from time import sleep
from informer import Informer
# from proto.python_out import DiSLAM_pb2
from config import cfg_robot1

# Sender
ifm = Informer(cfg_robot1, block=True)

# verification_num = 0

# def serialize_data():
#     global verification_num
#     info = DiSLAM_pb2.DiSCO()
#     for i in range(1 * 1024 * 1):
#         info.fftr.append(verification_num)
#         info.ffti.append(verification_num)

#     info.signature.append(verification_num)
#     verification_num += 1
#     verification_num %= 65535
#     return info.SerializeToString()

# def callback():
#     data = serialize_data()
#     ifm.send(data)
#     print("send data")

def listener():
    while True:
        img = np.random.randint(0, 255, (720, 1280, 3))
        ret, jpeg=cv2.imencode('.jpg', img)
        data = jpeg.tobytes()

        ifm.send(data)
        print("send data")
        sleep(1/30)

if __name__ == '__main__':
    listener()