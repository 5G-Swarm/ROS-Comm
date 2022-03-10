# -*- coding: utf-8 -*-
import json
import socket
import threading
from time import sleep
from . import utils
# from . import config
import sys 
from proto.python_out import reg_msgs_pb2

class Informer():
    def __init__(self, cfg, block=True):
        self.cfg = cfg
        assert cfg['robot_id'] != cfg['random_dest'], f"Cannot set robot ID as random_dest id {cfg['random_dest']}!"
        self.robot_id = cfg['robot_id']
        self.block = block
        self.port_dict = self.cfg['port_dict']
        self.send_keys = self.cfg['send_keys']
        self.recv_keys = self.cfg['recv_keys']
        self.socket_dict = {}
        self.data_dict = {}
        self.connect_state = {}

        self._cls_trd = False
        self.trd_list = []

        for key in self.send_keys:
            self.message_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.message_socket.bind(('localhost', self.cfg['bind_port'][key]))
            # print("[-] bind pass")
            self.connect_state[key] = self.message_socket.connect((self.cfg['public_ip'], self.port_dict[key]))
            print("[-] connect pass")

            self.socket_dict[key] = self.message_socket
            # self.data_dict[key] = utils.encode_message(
            #             data='server' if is_server else 'client',
            #             robot_id = self.robot_id,
            #             dest = '000000',
            #             mtype='register',
            #             pri=5)
            # self.message_socket.send(self.data_dict[key])
            
            # data = self.message_socket.recv(1024)
            # data = str(data, encoding = "utf-8")
            # try:
            #     json_data = json.loads(data)
            #     ip = json_data['Data'].split(':')[0]
            #     port = int(json_data['Data'].split(':')[1])
            #     print('Get IP/port', ip, ':', port, 'as', key)
            #     self.connect_state[key] = True
            # except:
            #     print('Error when connect message.\tGet', data)
            
            
        # wait for connecting
        if self.block:
            print('Waiting for connection ...')
            while set(self.send_keys) != set(self.connect_state.keys()):
                # sleep(0.001)
                print(set(self.send_keys), set(self.connect_state.keys()))
                sleep(1)
        print('Start to work...')

        # registration
        self.register(self.cfg['dest'], self.cfg['bind_port']['msg'])
            
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


######################################
#            Registration            #
######################################

    def register(self, dest_id, bind_port):
        reg_info = reg_msgs_pb2.RegInfo()
        reg_info.hostID = self.robot_id
        reg_info.destID = dest_id
        reg_info.is_server = self.cfg['is_server']
        reg_info.bind_port = self.cfg['bind_port']['msg']
        self.socket_dict['reg'].sendall(reg_info.SerializeToString())

######################################
#           Message Send             #
######################################

    def send(self, data):
        print("data len:", len(data))
        data_len = len(data).to_bytes(self.cfg['head_length'], 'big')
        data = data_len + data
        print("send data", data[:20], data[-20:])
        self.socket_dict['msg'].sendall(data)


######################################
#           Message Recv             #
######################################

    def msg_recv(self):
        send_data = bytes()
        data_cache = bytes()
        data_length = 0

        while True:
            if self._cls_trd:
                break

            data = self.socket_dict['msg'].recv(4096)
            # print("real data length is", len(data))
            # print("real data", data[:20], data[-20:])
            send_data += data

            if len(data_cache):
                # print("there is some cache...")
                send_data = data_cache + send_data
                data_cache = bytes()

            while len(send_data):
                # print("len(send_data):", len(send_data), send_data)
                if not data_length:
                    if len(send_data) < self.cfg['head_length']:
                        data_cache += send_data
                        send_data = bytes()
                        # print("Got cache because of length ifm")
                    else:
                        data_length = int.from_bytes(send_data[:self.cfg['head_length']], 'big')
                        # print("resource data_length", send_data[:self.cfg['head_length']])
                        # print("data_length", data_length)
                        # print("data_length byte", send_data[:20], send_data[:self.cfg['head_length']])
                        
                        send_data = send_data[self.cfg['head_length']:]
                        # # check for empty ifm
                        # if data_length:
                        #     send_data = send_data[self.cfg['head_length']:]
                        # else:
                        #     send_data = bytes()
                        
                elif len(send_data) < data_length:
                    data_cache += send_data
                    send_data = bytes()
                    # print("Got cache because of half data")
                else:
                    print("successful receive")
                    self.parse_message(send_data[:data_length])
                    send_data = send_data[data_length:]
                    # print("data remain is ", send_data[:20])
                    data_length = 0
            
            
            # data, _ = self.socket_dict['message'].recvfrom(655350)
            # print('data len:', len(data))
            # if len(buffer_data) % 43762 == 0:
            #     for i in range(len(buffer_data)//43762):
            #         try:
            #             json_data = json.loads(buffer_data[i*43762:(i+1)*43762].decode('utf-8'))
            #             self.parse_message(json_data)
            #         except:
            #             pass
            #     buffer_data = bytes()
            #     continue
            # if len(buffer_data) % 10996 == 0:
            #     for i in range(len(buffer_data)//10996):
            #         try:
            #             json_data = json.loads(buffer_data[i*10996:(i+1)*10996].decode('utf-8'))
            #             self.parse_message(json_data)
            #         except:
            #             pass
            #     buffer_data = bytes()
            #     continue
            # if len(buffer_data) > 43762*5:
            #     buffer_data = bytes()
            #     print("Out of memory !!!")
            #     continue
            
            # try:
            #     json_data = json.loads(data.decode('utf-8'))
            #     self.parse_message(json_data)
            # except:
            #     print('Error in message_recv')

    def parse_message(self, message):
        pass
        # message_type = message['Mtype']
        # pri = message['Pri']
        # robot_id = message['Id']
        # dest_id = message['Dest']
        # data = message['Data']
        # print('Get data:', message_type, robot_id, dest_id, len(data))

    def close(self):
        self._cls_trd = True
        for i in self.trd_list:
            del i
        for i in self.message_socket:
            i.close()
        