#!/usr/bin/env python
from time import sleep
import numpy as np
import cv2
# import rospy
# from rospy.core import xmlrpcapi
# import std_msgs.msg
# from geometry_msgs.msg import Point, Point32
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
# from dislam.msg import DiSCO

from informer import Informer
from proto.python_out import DiSLAM_pb2
from config import cfg_server


def parse_message(message):
    msg = DiSLAM_pb2.DiSCO()
    msg.ParseFromString(message)
    print("Get msg:", msg.signature)

def parse_img(message):
    print("Get img size:",len(message))
    nparr = np.frombuffer(message, np.uint8)
    img = cv2.imdecode(nparr,  cv2.IMREAD_COLOR)
    cv2.imshow('Image',img)
    cv2.waitKey(5)


class Server(Informer):
    # def parse_message(message):
    #     msg = DiSLAM_pb2.DiSCO()
    #     msg.ParseFromString(message)
    #     print("Get msg:", msg.signature)
    
    # def parse_img(message):
    #     print("Get msg:",len(message))
    #     nparr = np.fromstring(message, np.uint8)
    #     img = cv2.imdecode(nparr,  cv2.IMREAD_COLOR)
    #     cv2.imshow('Image',img)
    #     cv2.waitKey(5)

    def img_recv(self):
        self.recv('img', parse_img)

    def msg_recv(self):
        self.recv('msg', parse_message)

# Receive
ifm = Server(cfg_server, block=True)

if __name__ == '__main__':
    while True:
        # keep running for ifm receive thread work
        sleep(1)
