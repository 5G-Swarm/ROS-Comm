# -*- coding: utf-8 -*-

# PUBLICT_IP = '10.12.120.51'
PUBLICT_IP = '1.15.69.36'
# PUBLICT_IP = '127.0.0.1'

PORT_DICT = {
        'reg':10000,
        'msg':10001,
}

RECV_KEYS = ['msg']

REGISTER_KEYS = list(PORT_DICT.keys())

HEAD_LENGTH = 8

RANDOM_DEST = '986677'

colors = ['black','white','darkGray','gray','lightGray','red','green','blue','cyan','magenta','yellow','darkRed','darkGreen','darkBlue','darkCyan','darkMagenta','darkYellow']

cfg_robot1 = {
        'robot_id':'111111',
        'dest':'000000',
        'is_server':False,
        'public_ip':PUBLICT_IP,
        'port_dict':PORT_DICT,
        'recv_keys':RECV_KEYS,
        'head_length':HEAD_LENGTH,
        'random_dest':RANDOM_DEST,
        'bind_port':{
                'reg':30000,
                'msg':30001,
        },
        'server_port':{
                'reg':10000,
                'msg':10001,
        }
}

cfg_robot2 = {
        'robot_id':'222222',
        'dest':'000000',
        'is_server':False,
        'public_ip':PUBLICT_IP,
        'port_dict':PORT_DICT,
        'recv_keys':RECV_KEYS,
        'head_length':HEAD_LENGTH,
        'random_dest':RANDOM_DEST,
        'bind_port':{
                'reg':40000,
                'msg':40001,
        },
        'server_port':{
                'reg':10000,
                'msg':10001,
        }
}

cfg_server = {
        'robot_id':'000000',
        'dest':'111111',
        'is_server':False,
        'public_ip':PUBLICT_IP,
        'port_dict':PORT_DICT,
        'recv_keys':RECV_KEYS,
        'head_length':HEAD_LENGTH,
        'random_dest':RANDOM_DEST,
        'bind_port':{
                'reg':50000,
                'msg':50001,
        },
        'server_port':{
                'reg':10000,
                'msg':10001,
        }
}
