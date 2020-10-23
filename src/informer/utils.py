# -*- coding: utf-8 -*-
import json
from .config import PORT_DICT

if 'vision' in PORT_DICT.keys():
    import cv2
    def encode_img(img, isGrey=False):
        if isGrey:
            img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        img = cv2.resize(img, (img.shape[1],img.shape[0]), interpolation=cv2.INTER_AREA)
        ret, jpeg=cv2.imencode('.jpg', img)
        data = jpeg.tobytes()
        return data

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