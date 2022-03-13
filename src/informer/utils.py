# -*- coding: utf-8 -*-
import json

def encode_sensor(v, w, c):
    data = {'v':v, 'w':w, 'c':c}
    data = json.dumps(data).encode()
    return data

def encode_message(data, robot_id, dest, mtype='normal', pri=5):
    data = {'Mtype':mtype, 'Pri':pri, 'Id':robot_id, 'Dest':dest, 'Data':data}
    data = json.dumps(data).encode()
    return data

def to_json(**kwargs):
    return json.dumps(kwargs)
    
def encode_debug_message(messages):
    data = json.dumps(messages).encode()
    return data