#!/usr/bin/env python
import cv2
import numpy as np
# import rospy
# from dislam.msg import DiSCO
import time
from informer import Informer
from proto.python_out import DiSLAM_pb2
from config import cfg_robot1

class Client(Informer):
    def send_msg(self, message):
        self.send(message, 'msg')

    def send_img(self, message):
        self.send(message, 'img')

    def send_sync(self, message):
        self.send(message, 'sync')

# Sender
ifm = Client(cfg_robot1)

verification_num = 0
test_FPS = False
videoShape = (320*1, 1*240)
# videoShape = (1280, 720)

def serialize_data():
    global verification_num
    info = DiSLAM_pb2.DiSCO()
    for i in range(1 * 1024 * 1):
        info.fftr.append(verification_num)
        info.ffti.append(verification_num)

    info.signature.append(verification_num)
    verification_num += 1
    verification_num %= 65535
    return info.SerializeToString()

def callback_disco():
    data = serialize_data()
    ifm.send_msg(data)

def callback_sync():
    data = str(time.time()).encode()
    ifm.send_sync(data)

def callback_img(img):
    # img = cv2.resize(img, (320*4, 4*240))
    ret, jpeg = cv2.imencode('.jpg', img)
    data = jpeg.tobytes()
    ifm.send_img(data)

def listener():
    # rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber('disco', DiSCO, callback)
    # rospy.spin()
    video_reader = cv2.VideoCapture('video.mp4')

    success, img = video_reader.read()
    img = cv2.resize(img, videoShape)

    if test_FPS:
        cnt = 1
        ts = time.time()

    while success:
    # while True:
        # callback_disco()
        callback_img(img)
        # callback_sync()
        success, img = video_reader.read()
        img = cv2.resize(img, videoShape)

        if test_FPS:
            cnt += 1
            new_ts = time.time()
            if (new_ts - ts) > 1:
                print(cnt, '\n\n\n\n')
                cnt = 0
                ts = new_ts

if __name__ == '__main__':
    listener()