# -*- coding: utf-8 -*-
import time

def send_tcp_package(data, socket, address, port, debug=False):
    socket.sendall(data)