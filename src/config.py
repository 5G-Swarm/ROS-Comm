# -*- coding: utf-8 -*-

# PUBLICT_IP = '127.0.0.1'
PUBLICT_IP = '124.223.46.115'

PORT_DICT = {
        'reg':10000,
        'msg':10001,
        'img':10002,
        # 'sync':10003,
}

SEND_KEYS = list(PORT_DICT.keys())
RECV_KEYS = ['msg', 'img']
# RECV_KEYS = ['msg']
# RECV_KEYS = ['img']

HEAD_LENGTH = 8

RANDOM_DEST = '986677'

colors = ['black','white','darkGray','gray','lightGray','red','green','blue','cyan','magenta','yellow','darkRed','darkGreen','darkBlue','darkCyan','darkMagenta','darkYellow']

cfg_robot1 = {
        'robot_id':'111111',
        'dest':'000000',
        'is_server':False,
        'public_ip':PUBLICT_IP,
        'port_dict':PORT_DICT,
        'send_keys':SEND_KEYS,
        'recv_keys':RECV_KEYS,
        'head_length':HEAD_LENGTH,
        'random_dest':RANDOM_DEST,
        'bind_port':{
                'reg':30000,
                'msg':30001,
                'img':30002,
                # 'sync':30003,
        },
        'server_port':{
                'reg':10000,
                'msg':10001,
                'img':10002,
                # 'sync':10003,
        }
}

# cfg_robot2 = {
#         'robot_id':'222222',
#         'dest':'000000',
#         'is_server':False,
#         'public_ip':PUBLICT_IP,
#         'port_dict':PORT_DICT,
#         'recv_keys':RECV_KEYS,
#         'head_length':HEAD_LENGTH,
#         'random_dest':RANDOM_DEST,
#         'bind_port':{
#                 'reg':40000,
#                 'msg':40001,
#         },
#         'server_port':{
#                 'reg':10000,
#                 'msg':10001,
#         }
# }


cfg_server = {
        'robot_id':'000000',
        'dest':'111111',
        'is_server':True,
        'public_ip':PUBLICT_IP,
        'port_dict':PORT_DICT,
        'send_keys':SEND_KEYS,
        'recv_keys':RECV_KEYS,
        'head_length':HEAD_LENGTH,
        'random_dest':RANDOM_DEST,
        'bind_port':{
                'reg':50000,
                'msg':50001,
                'img':50002,
                # 'sync':50003,
        },
        'server_port':{
                'reg':10000,
                'msg':10001,
                'img':10002,
                # 'sync':10003,
        }
}