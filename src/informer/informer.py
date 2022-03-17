# -*- coding: utf-8 -*-
import random
import socket
import threading
from time import sleep
from proto.python_out import reg_msgs_pb2

class Informer():
    def __init__(self, cfg):
        self.cfg = cfg
        assert cfg['robot_id'] != cfg['random_dest'], f"Cannot set robot ID as random_dest id {cfg['random_dest']}!"
        self.robot_id = cfg['robot_id']
        self.port_dict = self.cfg['port_dict']
        self.send_keys = self.cfg['send_keys']
        self.recv_keys = self.cfg['recv_keys']
        self.socket_dict = {}
        self.data_dict = {}
        self.connect_state = {}

        self._cls_trd = False
        # save the recv threads
        self.trd_list = []

        for key in self.send_keys:
            self.message_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("[-] try to bind", self.cfg['bind_port'][key])
            try:
                self.message_socket.bind(('', self.cfg['bind_port'][key]))
            except:
                self.cfg['bind_port'][key] = random.randint(10000,65536)
                self.message_socket.bind(('', self.cfg['bind_port'][key]))
            print("[-] bind pass", self.cfg['bind_port'][key])
            print(key, (self.cfg['public_ip'], self.port_dict[key]))
            self.connect_state[key] = self.message_socket.connect((self.cfg['public_ip'], self.port_dict[key]))
            print("[-] connect pass")

            self.socket_dict[key] = self.message_socket
        
        print('Start to work...')

        # registration
        for key in self.cfg['bind_port'].keys():
            if key == 'reg': continue
            self.register(self.cfg['dest'], key)
            # wait for registration
        sleep(0.1)
            
        # start receive threads
        for key in self.recv_keys:
            if key in self.send_keys:
                try:
                    receive_func = getattr(self.__class__, key+'_recv')
                except AttributeError:
                    print(self.__class__.__name__, 'has no attribute called', key+'_recv')
                    continue
                recv_thread = threading.Thread(
                    target = receive_func, args=(self,)
                )
                recv_thread.start()
                self.trd_list.append(recv_thread)

######################################
#            Registration            #
######################################

    def register(self, dest_id, key):
        reg_info = reg_msgs_pb2.RegInfo()
        reg_info.hostID = self.robot_id
        reg_info.destID = dest_id
        reg_info.key = key
        reg_info.is_server = self.cfg['is_server']
        reg_info.bind_port = self.cfg['bind_port'][key]
        self.socket_dict[key].sendall(reg_info.SerializeToString())

######################################
#           Message Send             #
######################################

    def send(self, data, key):
        # print("data len:", len(data))
        data_len = len(data).to_bytes(self.cfg['head_length'], 'big')
        data = data_len + data
        try:
            self.socket_dict[key].sendall(data)
        except BrokenPipeError:
            print('Send error ! Connection broken !')


######################################
#           Message Recv             #
######################################

    def recv(self, key, func):
        print('start to recv ...')
        send_data = bytes()
        data_cache = bytes()
        data_length = 0

        while True:
            if self._cls_trd:
                break
            data = self.socket_dict[key].recv(65535)
            if len(data) == 0: continue
            send_data += data

            if len(data_cache):
                send_data = data_cache + send_data
                data_cache = bytes()

            while len(send_data):
                if not data_length:
                    if len(send_data) < self.cfg['head_length']:
                        data_cache += send_data
                        send_data = bytes()
                    else:
                        data_length = int.from_bytes(send_data[:self.cfg['head_length']], 'big')
                        send_data = send_data[self.cfg['head_length']:]
                        
                elif len(send_data) < data_length:
                    data_cache += send_data
                    send_data = bytes()
                else:
                    func(send_data[:data_length])
                    send_data = send_data[data_length:]
                    data_length = 0

    def parse_message(self, message):
        pass

    def close(self):
        self._cls_trd = True
        for i in self.trd_list:
            del i
        for i in self.message_socket:
            i.close()
        