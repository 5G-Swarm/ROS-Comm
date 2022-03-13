#!/usr/bin/env python
from time import sleep
import cv2
import numpy as np
# import rospy
# from rospy.core import xmlrpcapi
# import std_msgs.msg
# from geometry_msgs.msg import Point, Point32
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
# from dislam.msg import DiSCO

from informer import Informer
# from proto.python_out import DiSLAM_pb2
from config import cfg_server
class Server(Informer):
    def parse_message(self, message):
        print("Get msg:",len(message))
        nparr = np.fromstring(message, np.uint8)
        img = cv2.imdecode(nparr,  cv2.IMREAD_COLOR)
        cv2.imshow('Image',img)
        cv2.waitKey(5)
        # msg = DiSLAM_pb2.DiSCO()
        # msg.ParseFromString(message)
        # print("Get msg:", msg.signature)

# Receive
ifm = Server(cfg_server, block=True)

if __name__ == '__main__':
    while True:
        # keep running for ifm receive thread work
        sleep(1)
