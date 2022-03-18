#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
from os.path import join, dirname
sys.path.insert(0, join(dirname(__file__), '../'))

import simulator
simulator.load('/home/wang/CARLA_0.9.9.4')
import carla

from simulator import config, set_weather, add_vehicle
from simulator.sensor_manager import SensorManager

import random
import cv2
import numpy as np

import time
from informer import Informer
from config_carla import cfg_robot1
from proto.python_out import carla_msgs_pb2

global_control = carla.VehicleControl()

def pb2carla_ctrl(cmd):
    control = carla.VehicleControl()
    control.throttle = cmd.throttle
    control.steer = cmd.steer
    control.brake = cmd.brake
    control.hand_brake = cmd.hand_brake
    control.reverse = cmd.reverse
    control.manual_gear_shift = cmd.manual_gear_shift
    control.gear = cmd.gear

    return control

def parse_message(message):
    global global_control
    cmd = carla_msgs_pb2.CtrlCmd()
    cmd.ParseFromString(message)
    global_control = pb2carla_ctrl(cmd)

class Client(Informer):
    def send_msg(self, message):
        self.send(message, 'msg')

    def send_img(self, message):
        self.send(message, 'img')

    def send_sync(self, message):
        self.send(message, 'sync')

    def msg_recv(self):
        self.recv('msg', parse_message)

# Sender
ifm = Client(cfg_robot1)
# videoShape = (320*1, 1*240)
videoShape = (1280, 720)

def callback_sync():
    data = str(time.time()).encode()
    ifm.send_sync(data)

def callback_img(img):
    # img = cv2.resize(img, (320*4, 4*240))
    ret, jpeg = cv2.imencode('.jpg', img)
    data = jpeg.tobytes()
    ifm.send_img(data)


global_img = None
def image_callback(data):
    global global_img
    array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8")) 
    array = np.reshape(array, (data.height, data.width, 4)) # RGBA format
    global_img = array[..., :3]

if __name__ == '__main__':
    client = carla.Client(config['host'], config['port'])
    client.set_timeout(config['timeout'])
    world = client.load_world('Town01')
    world.set_weather(carla.WeatherParameters.ClearNoon)
    blueprint = world.get_blueprint_library()
    vehicle = add_vehicle(world, blueprint, vehicle_type='vehicle.audi.a2')
    vehicle.set_simulate_physics(True)

    sensor_dict = {
        'camera':{
            'transform':carla.Transform(carla.Location(x=0.5, y=0.0, z=2.5)),
            'callback':image_callback,
            },
        }
    sm = SensorManager(world, blueprint, vehicle, sensor_dict)
    sm.init_all()

    while True:
        if global_img is not None:
            callback_img(global_img)
            vehicle.apply_control(global_control)
        time.sleep(1/30)

    cv2.destroyAllWindows()
    sm.close_all()
    vehicle.destroy()