#!/usr/bin/env python

from informer import Informer
from proto.python_out import marker_pb2
from config import cfg_server
import time

global_marker = None

def parse_message(message):
    global global_marker
    marker = marker_pb2.Marker()
    marker.ParseFromString(message)
    global_marker = marker


class Server(Informer):
    def msg_recv(self):
        self.recv('msg', parse_message)

# Receive
ifm = Server(cfg_server)

if __name__ == '__main__':
    while True:
        # keep running for ifm receive thread work
        if global_marker is not None:
            print(global_marker)
        time.sleep(1)
