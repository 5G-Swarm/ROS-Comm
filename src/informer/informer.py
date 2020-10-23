# -*- coding: utf-8 -*-
import json
import socket
import threading
from time import sleep
from informer.network import send_tcp_package
import informer.utils as utils
from informer import config

class Informer():
    def __init__(self, robot_id=None, is_server=False, block=True):
        self.robot_id = str(robot_id) if robot_id != None else None
        self.block = block
        self.register_keys = config.REGISTER_KEYS
        self.port_dict = config.PORT_DICT
        self.recv_keys = config.RECV_KEYS
        self.socket_dict = {}
        self.data_dict = {}
        self.connect_state = {}

        for key in self.register_keys:
            self.message_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.message_socket.connect((config.PUBLICT_IP, self.port_dict[key]))

            self.socket_dict[key] = self.message_socket
            self.data_dict[key] = utils.encode_message(
                        data='server' if is_server else 'client',
                        robot_id = self.robot_id,
                        dest = '000000',
                        mtype='register',
                        pri=5)
            self.message_socket.send(self.data_dict[key])
            
            data = self.message_socket.recv(1024)
            data = str(data, encoding = "utf-8")
            try:
                json_data = json.loads(data)
                ip = json_data['Data'].split(':')[0]
                port = int(json_data['Data'].split(':')[1])
                print('Get IP/port', ip, ':', port, 'as', key)
                self.connect_state[key] = True
            except:
                print('Error when connect message.\tGet', data)
            
            
        # wait for connecting
        if self.block:
            print('Waiting for connection ...')
            while set(self.register_keys) != set(self.connect_state.keys()):
                # sleep(0.001)
                print(set(self.register_keys), set(self.connect_state.keys()))
                sleep(1)
        print('Start to work...')
            
        # start receive threads
        for key in self.recv_keys:
            if key in self.register_keys:
                try:
                    receive_func = getattr(self.__class__, key+'_recv')
                except AttributeError:
                    print(self.__class__.__name__, 'has no attribute called', key+'_recv')
                    continue
                recv_thread = threading.Thread(
                    target = receive_func, args=(self,)
                )
                recv_thread.start()

        # debug info
        self.cnt = 0
        self.debug_dict = {}
        self.sim_info = None
        
    def connect(self, key, sock):
        data = ''
        while len(data) < 1:
        	data, address = sock.recvfrom(65535)
        data = str(data, encoding = "utf-8")
        try:
            json_data = json.loads(data)
            ip = json_data['Data'].split(':')[0]
            port = int(json_data['Data'].split(':')[1])
            print('Get IP/port', ip, ':', port, 'as', key)
            self.connect_state[key] = True
        except:
            print('Error when connect', key, '.\tGet', data)

    def send(self, data, debug=False):
        self.socket_dict['message'].sendall(data)

    def message_recv(self):
        while True:
            data, addr = self.socket_dict['message'].recvfrom(655350)
            # print('data len:', len(data))
            if len(data) % 43762 == 0:
                for i in range(len(data)//43762):
                    json_data = json.loads(data[i*43762:(i+1)*43762].decode('utf-8'))
                    self.parse_message(json_data)
                continue
            if len(data) % 10996 == 0:
                for i in range(len(data)//10996):
                    json_data = json.loads(data[i*10996:(i+1)*10996].decode('utf-8'))
                    self.parse_message(json_data)
                continue
            
            try:
                json_data = json.loads(data.decode('utf-8'))
                self.parse_message(json_data)
            except:
                print('Error in message_recv')

    def parse_message(self, message):
        message_type = message['Mtype']
        pri = message['Pri']
        robot_id = message['Id']
        dest_id = message['Dest']
        data = message['Data']
        print('Get data:', message_type, robot_id, dest_id, len(data))
        