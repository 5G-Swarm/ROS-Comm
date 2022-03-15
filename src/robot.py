#!/usr/bin/env python
import cv2
import numpy as np
# import rospy
# from dislam.msg import DiSCO
from time import sleep, time
from informer import Informer
from proto.python_out import DiSLAM_pb2
from config import cfg_robot1

class Client(Informer):
    def send_msg(self, message):
        self.send(message, 'msg')

    def send_img(self, message):
        self.send(message, 'img')

# Sender
ifm = Client(cfg_robot1, block=True)

verification_num = 0

def serialize_data():
    global verification_num
    info = DiSLAM_pb2.DiSCO()
    # for i in range(1 * 1024 * 1):
    for i in range(1 * 128 * 1):
        info.fftr.append(verification_num)
        info.ffti.append(verification_num)

    info.signature.append(verification_num)
    verification_num += 1
    verification_num %= 65535
    return info.SerializeToString()

def callback_disco():
    data = serialize_data()
    ifm.send_msg(data)
    print("send data")

def callback_ts():
    data = str(time()).encode()
    ifm.send_msg(data)
    print("send data")

def callback_img():
    # img = np.random.randint(0, 255, (720, 1280, 3))
    img = np.random.randint(0, 255, (300, 300, 3))
    ret, jpeg = cv2.imencode('.jpg', img)
    data = jpeg.tobytes()
    ifm.send_img(data)
    print("send data")

def callback_video(img):
    # img = cv2.resize(img, (320, 240))
    # img = cv2.resize(img, (320*4, 4*240))
    ret, jpeg = cv2.imencode('.jpg', img)
    data = jpeg.tobytes()
    ifm.send_img(data)
    print("send data")

def listener():
    # rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber('disco', DiSCO, callback)
    # rospy.spin()
    video_reader = cv2.VideoCapture('video.mp4')
    # img = cv2.imread('img.jpg')

    success, img = video_reader.read()

    while success:
    # while True:
        # callback_disco()
        callback_video(img)
        callback_ts()
        sleep(1/10)
        success, img = video_reader.read()

if __name__ == '__main__':
    listener()