#!/usr/bin/env python

from informer import Informer
from proto.python_out import marker_pb2
from config import cfg_server
import time

global_marker_list = None

def parse_message(message):
    global global_marker_list
    marker_list = marker_pb2.MarkerList()
    marker_list.ParseFromString(message)
    global_marker_list = marker_list


class Server(Informer):
    def msg_recv(self):
        self.recv('msg', parse_message)

# Receive
ifm = Server(cfg_server)

if __name__ == '__main__':
    while True:
        # keep running for ifm receive thread work
        if global_marker_list is not None:
            print(global_marker_list)
        time.sleep(1)
