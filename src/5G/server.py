#!/usr/bin/env python
import sys
from os.path import join, dirname
sys.path.insert(0, join(dirname(__file__), '../'))

from informer import Informer
from proto.python_out import marker_pb2, geometry_msgs_pb2, path_msgs_pb2, cmd_msgs_pb2
from config_5g import cfg_server
import time
import numpy as np

global_marker_list = None
global_odometry = None
global_cmd = None

def parse_message(message):
    global global_marker_list
    marker_list = marker_pb2.MarkerList()
    marker_list.ParseFromString(message)
    global_marker_list = marker_list.marker_list

def parse_odometry(message):
    global global_odometry
    odometry = geometry_msgs_pb2.Pose()
    odometry.ParseFromString(message)
    global_odometry = odometry

def parse_cmd(message):
    global global_cmd
    cmd = cmd_msgs_pb2.Cmd()
    cmd.ParseFromString(message)
    global_cmd = cmd

class Server(Informer):
    def msg_recv(self):
        self.recv('msg', parse_message)

    def odm_recv(self):
        self.recv('odm', parse_odometry)

    def cmd_recv(self):
        self.recv('cmd', parse_cmd)

    def send_path(self, message):
        self.send(message, 'path')

# Receive
ifm = Server(cfg_server)


def send_path(path_list):
    global ifm
    path = path_msgs_pb2.Path()
    for i in range(len(path_list)):
        pose = path_msgs_pb2.Pose2D()
        pose.x = path_list[i][0]
        pose.y = path_list[i][0]
        pose.theta = path_list[i][0]

        path.poses.append(pose)

    sent_data = path.SerializeToString()
    print('send', len(sent_data))
    ifm.send_path(sent_data)

if __name__ == '__main__':
    while True:
        # keep running for ifm receive thread work
        if global_marker_list is not None:
            print(global_marker_list)

        if global_odometry is not None:
            print(global_odometry)

        if global_cmd is not None:
            print(global_cmd)

        path_list = np.random.rand(20, 3)
        send_path(path_list)
        time.sleep(0.5)
