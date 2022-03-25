#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pygame
import numpy as np
from scipy.spatial.transform import Rotation as R
from icecream import ic as print
import socket
import threading
import time
import sys
import math
import cv2
from shapely.geometry import Polygon, Point

from informer import Informer
from proto.python_out import marker_pb2, geometry_msgs_pb2, path_msgs_pb2, cmd_msgs_pb2
from config_5g import cfg_server


HOST_ADDRESS = '127.0.0.1'
BLACK = (0, 0, 0)
GREY = (192, 192, 192)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
WHITE = (255, 255, 255)
WINDOW_WIDTH = 1920
WINDOW_HEIGHT = 1080
ROBOT_SIZE = 20
BUTTON_WIDTH = 300
BUTTON_HEIGHT = 100
BUTTON_LIGHT = (170, 170, 170)
BUTTON_DARK = (100, 100, 100)
BUTTON_GOAL_X = 50
BUTTON_GOAL_Y = 50
BUTTON_LASER_X = 50
BUTTON_LASER_Y = 200
BUTTON_SATELLITE_X = 50
BUTTON_SATELLITE_Y = 350
# read map
LASER_MAP = pygame.image.load('./maps/laser_map.jpg')
SATELLITE_MAP = pygame.image.load('./maps/satellite_map.png')
DISPLAY_MAP = LASER_MAP
map_offset = np.array([0, 0])
robot_goal = None
robot_pos = []
robot_heading = []
robot_cmd = []
bounding_box = dict()
path_pos = []
robot_clicked_id = None
robot_img = None
box_clicked_id = None
# flags
map_draging = False
goal_setting = False
robot_clicked = False
view_image = False
box_clicked = False

class Receiver(object):
    def __init__(self, addr=HOST_ADDRESS, port=23333):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = addr
        self.port = port
        self.sock.settimeout(1.0)
        self.sock.bind((self.addr, self.port))
        self.thread = threading.Thread(target=self.receive_data)
        self.thread.start()
        self.timeout = False

    def receive_data(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                data = data.decode("utf-8").split(';')
                MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
                offset = np.array([WINDOW_WIDTH//2 - MAP_WIDTH//2, WINDOW_HEIGHT//2 - MAP_HEIGHT//2])
                global path_pos
                path_pos = np.array([np.array([float(pos.split(',')[0]), float(pos.split(',')[1])]) + offset
                            for pos in data if pos != ''])
                # print(path_pos, len(path_pos))
                self.timeout = False
            except socket.timeout:
                self.timeout = True
            time.sleep(0.01)
            
def parse_message(message):
    global bounding_box
    marker_list = marker_pb2.MarkerList()
    marker_list.ParseFromString(message)
    MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
    offset = np.array([WINDOW_WIDTH//2 - MAP_WIDTH//2, WINDOW_HEIGHT//2 - MAP_HEIGHT//2])
    for marker in marker_list.marker_list:
        try:
            center_pos = np.array([int(1165 - marker.pose.position.y*20), int(741-20*marker.pose.position.x)]) + offset
            orientation = R.from_quat([marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]).as_euler('xyz', degrees=False)[2]
            orientation += np.pi / 2
            height, width = 10*marker.scale.x, 10*marker.scale.y
            vertex_A = center_pos + height*np.array([np.cos(orientation), np.sin(orientation)]) + width*np.array([-np.sin(orientation), np.cos(orientation)])
            vertex_B = center_pos + height*np.array([np.cos(orientation), np.sin(orientation)]) - width*np.array([-np.sin(orientation), np.cos(orientation)])
            vertex_C = center_pos - height*np.array([np.cos(orientation), np.sin(orientation)]) - width*np.array([-np.sin(orientation), np.cos(orientation)])
            vertex_D = center_pos - height*np.array([np.cos(orientation), np.sin(orientation)]) + width*np.array([-np.sin(orientation), np.cos(orientation)])
            marker_id = marker.id
            new_box = np.array([vertex_A, vertex_B, vertex_C, vertex_D])
            # overlap filter
            overlap = False
            p1 = Polygon(new_box)
            for id, pos in bounding_box.items():
                p2 = Polygon(pos)
                if p1.intersects(p2) and id != marker_id:
                    overlap = True
                    break
            if not overlap:
                bounding_box[marker_id] = np.array(new_box)
        except:
            pass
        # print(bounding_box)

def parse_odometry(message):
    global robot_pos, robot_heading
    odometry = geometry_msgs_pb2.Pose()
    odometry.ParseFromString(message)
    MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
    offset = np.array([WINDOW_WIDTH//2 - MAP_WIDTH//2, WINDOW_HEIGHT//2 - MAP_HEIGHT//2])
    robot_pos = [np.array([int(odometry.position.y*20+1165), int(741-20*odometry.position.x)]) + offset]
    robot_heading = [R.from_quat([odometry.orientation.x, odometry.orientation.y, odometry.orientation.z, odometry.orientation.w]).as_euler('xyz', degrees=False)[1]]
    # print(robot_pos)

def parse_cmd(message):
    global robot_cmd
    # print('grt cmd !!!')
    cmd = cmd_msgs_pb2.Cmd()
    cmd.ParseFromString(message)
    robot_cmd = [[cmd.v, cmd.w]]

def send_path(path_list):
    global ifm
    path = path_msgs_pb2.Path()
    for i in range(len(path_list)):
        pose = path_msgs_pb2.Pose2D()
        pose.x = path_list[i][0]
        pose.y = path_list[i][0]
        pose.theta = path_list[i][0]

        path.poses.append(pose)

    sent_data = path.SerializeToString()
    # print('send', len(sent_data))
    ifm.send_path(sent_data)

class Server(Informer):
    def msg_recv(self):
        self.recv('msg', parse_message)

    def odm_recv(self):
        self.recv('odm', parse_odometry)

    def cmd_recv(self):
        self.recv('cmd', parse_cmd)

    def send_path(self, message):
        self.send(message, 'path')


def sendGoal(goal):
    goal_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if len(robot_pos) == 0:
        goal_str = str(goal[0]) + ',' + str(goal[1])
    else:
        MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
        offset = np.array([WINDOW_WIDTH//2 - MAP_WIDTH//2, WINDOW_HEIGHT//2 - MAP_HEIGHT//2])
        pos = robot_pos[0] - offset
        goal_str = str(goal[0]) + ',' + str(goal[1]) + ',' + str(pos[0]) + ',' + str(pos[1])
    goal_sock.sendto(bytes(goal_str, 'ascii'), (HOST_ADDRESS, 23334))

def screen2pos(x, y):
    MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
    pos = np.array([x, y]) - np.array([WINDOW_WIDTH//2 - MAP_WIDTH//2, WINDOW_HEIGHT//2 - MAP_HEIGHT//2])
    return pos

def pos2screen(x, y):
    return x, y

def drawRobots():
    for pos, heading, cmd in zip(robot_pos, robot_heading, robot_cmd):
        pygame.draw.circle(SCREEN, GREEN, pos + map_offset, ROBOT_SIZE)
        pygame.draw.line(SCREEN, BLUE, pos + map_offset, pos + map_offset + min(max(40*cmd[0], 25), 40)*np.array([np.cos(heading+np.pi/2), np.sin(heading+np.pi/2)]), 5)
        
def drawGoal():
    if robot_goal is not None:
        # pygame.draw.circle(SCREEN, GREEN, robot_goal + map_offset, ROBOT_SIZE)
        cicle = (robot_goal + map_offset)
        marker_size = 20
        width = 10
        pygame.draw.line(SCREEN, RED, (cicle[0]-marker_size, cicle[1]-marker_size), (cicle[0]+marker_size, cicle[1]+marker_size), width)
        pygame.draw.line(SCREEN, RED, (cicle[0]-marker_size, cicle[1]+marker_size), (cicle[0]+marker_size, cicle[1]-marker_size), width)


def drawBoundingBox():
    bounding_box_copy = bounding_box.copy()
    for _, pos in bounding_box_copy.items():
        # x, y = pos + map_offset
        # pygame.draw.rect(SCREEN, BLUE, pygame.Rect(x, y, 60, 100), 10)
        pygame.draw.lines(SCREEN, BLUE, True, pos + map_offset, 10)

def drawPath():
    if len(path_pos) > 1:
        pygame.draw.lines(SCREEN, RED, False, path_pos + map_offset, 10)

def drawButton():
    # font settings
    FONT = pygame.font.SysFont('Corbel', 75)

    # get mouse position
    mouse = pygame.mouse.get_pos()

    # button: set goal
    text = FONT.render('Set Goal', True, WHITE)
    if BUTTON_GOAL_X <= mouse[0] <= BUTTON_GOAL_X + BUTTON_WIDTH and BUTTON_GOAL_Y <= mouse[1] <= BUTTON_GOAL_Y + BUTTON_HEIGHT:
        pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_GOAL_X, BUTTON_GOAL_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    else:
        pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_GOAL_X, BUTTON_GOAL_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    SCREEN.blit(text, (BUTTON_GOAL_X+45, BUTTON_GOAL_Y+25))
    # button: laser map
    text = FONT.render('LASER', True, WHITE)
    if BUTTON_LASER_X <= mouse[0] <= BUTTON_LASER_X + BUTTON_WIDTH and BUTTON_LASER_Y <= mouse[1] <= BUTTON_LASER_Y + BUTTON_HEIGHT:
        pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_LASER_X, BUTTON_LASER_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    else:
        pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_LASER_X, BUTTON_LASER_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    SCREEN.blit(text, (BUTTON_LASER_X+60, BUTTON_LASER_Y+25))
    # button: satellite map
    text = FONT.render('SATELLITE', True, WHITE)
    if BUTTON_SATELLITE_X <= mouse[0] <= BUTTON_SATELLITE_X + BUTTON_WIDTH and BUTTON_SATELLITE_Y <= mouse[1] <= BUTTON_SATELLITE_Y + BUTTON_HEIGHT:
        pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_SATELLITE_X, BUTTON_SATELLITE_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    else:
        pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_SATELLITE_X, BUTTON_SATELLITE_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    SCREEN.blit(text, (BUTTON_SATELLITE_X+10, BUTTON_SATELLITE_Y+25))

def drawMessageBox():
    # font settings
    FONT = pygame.font.SysFont('Corbel', 75)

    # get mouse position
    mouse = pygame.mouse.get_pos()

    if robot_clicked:
        # box
        BOX_X, BOX_Y = robot_pos[robot_clicked_id] + map_offset + np.array([25, -150])
        BOX_WIDTH, BOX_HEIGHT = 350, 150
        BOX_COLOR = (255, 255, 255)
        pygame.draw.rect(SCREEN, BOX_COLOR, [BOX_X, BOX_Y, BOX_WIDTH, BOX_HEIGHT])

        # button: view image
        text = FONT.render('View Image', True, WHITE)
        BUTTON_IMAGE_X, BUTTON_IMAGE_Y = robot_pos[robot_clicked_id] + map_offset + np.array([50, -125])
        if BUTTON_IMAGE_X <= mouse[0] <= BUTTON_IMAGE_X + BUTTON_WIDTH and BUTTON_IMAGE_Y <= mouse[1] <= BUTTON_IMAGE_Y + BUTTON_HEIGHT:
            pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_IMAGE_X, BUTTON_IMAGE_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        else:
            pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_IMAGE_X, BUTTON_IMAGE_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        SCREEN.blit(text, (BUTTON_IMAGE_X+10, BUTTON_IMAGE_Y+25))

    if box_clicked:
        # box
        box_center = np.mean(bounding_box[box_clicked_id], axis=0)
        BOX_X, BOX_Y = box_center + map_offset + np.array([25, -150])
        BOX_WIDTH, BOX_HEIGHT = 350, 150
        BOX_COLOR = (255, 255, 255)
        pygame.draw.rect(SCREEN, BOX_COLOR, [BOX_X, BOX_Y, BOX_WIDTH, BOX_HEIGHT])

        # button: get id
        text = FONT.render('Get ID', True, WHITE)
        BUTTON_ID_X, BUTTON_ID_Y = box_center + map_offset + np.array([50, -125])
        if BUTTON_ID_X <= mouse[0] <= BUTTON_ID_X + BUTTON_WIDTH and BUTTON_ID_Y <= mouse[1] <= BUTTON_ID_Y + BUTTON_HEIGHT:
            pygame.draw.rect(SCREEN, BUTTON_LIGHT, [BUTTON_ID_X, BUTTON_ID_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        else:
            pygame.draw.rect(SCREEN, BUTTON_DARK, [BUTTON_ID_X, BUTTON_ID_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
        SCREEN.blit(text, (BUTTON_ID_X+75, BUTTON_ID_Y+25))


def drawMaps():
    WINDOW_WIDTH, WINDOW_HEIGHT = pygame.display.get_surface().get_size()
    MAP_WIDTH, MAP_HEIGHT = DISPLAY_MAP.get_size()
    map_pos = np.array([WINDOW_WIDTH//2 - MAP_WIDTH//2, WINDOW_HEIGHT//2 - MAP_HEIGHT//2]) + map_offset
    SCREEN.blit(DISPLAY_MAP, map_pos)


if __name__ == "__main__":
    pygame.init()
    SCREEN = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))#, pygame.RESIZABLE)
    pygame.display.set_caption('5G Monitor')
    # icon = pygame.image.load('*.png')
    # pygame.display.set_icon(icon)
    CLOCK = pygame.time.Clock()
    SCREEN.fill(GREY)
    data_receiver = Receiver()
    try:
        server = Server(cfg_server)
    except:
        pass

    cnt = 0
    while True:
        start_time = time.time()
        cnt += 1
        SCREEN.fill(GREY)
        drawMaps()
        drawGoal()
        drawRobots()
        drawBoundingBox()
        drawPath()
        drawButton()
        drawMessageBox()

        for event in pygame.event.get():
            mods = pygame.key.get_mods()
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN and mods & pygame.KMOD_CTRL:
                if event.button == 1:            
                    map_draging = True
                    start_pos = event.pos
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # get mouse position
                mouse = pygame.mouse.get_pos()
                # button: set goal
                if BUTTON_GOAL_X <= mouse[0] <= BUTTON_GOAL_X + BUTTON_WIDTH and BUTTON_GOAL_Y <= mouse[1] <= BUTTON_GOAL_Y + BUTTON_HEIGHT:
                    goal_setting = True
                elif goal_setting:
                    goal_setting = False
                    robot_goal = mouse - map_offset
                # button: laser map
                elif BUTTON_LASER_X <= mouse[0] <= BUTTON_LASER_X + BUTTON_WIDTH and BUTTON_LASER_Y <= mouse[1] <= BUTTON_LASER_Y + BUTTON_HEIGHT:
                    DISPLAY_MAP = LASER_MAP
                # button: satellite map
                elif BUTTON_SATELLITE_X <= mouse[0] <= BUTTON_SATELLITE_X + BUTTON_WIDTH and BUTTON_SATELLITE_Y <= mouse[1] <= BUTTON_SATELLITE_Y + BUTTON_HEIGHT:
                    DISPLAY_MAP = SATELLITE_MAP
                # button: robot
                if robot_clicked:
                    # button: view image
                    BUTTON_IMAGE_X, BUTTON_IMAGE_Y = robot_pos[robot_clicked_id] + map_offset + np.array([50, -125])
                    if BUTTON_IMAGE_X <= mouse[0] <= BUTTON_IMAGE_X + BUTTON_WIDTH and BUTTON_IMAGE_Y <= mouse[1] <= BUTTON_IMAGE_Y + BUTTON_HEIGHT:
                        view_image = True
                        print('show image')
                robot_clicked = False
                for idx, pos in enumerate(robot_pos):
                    if math.hypot(mouse[0] - (pos + map_offset)[0], mouse[1] - (pos + map_offset)[1]) <= ROBOT_SIZE:
                        print('click robot {}'.format(idx))
                        robot_clicked = True
                        robot_clicked_id = idx
                        break
                # button: bounding box
                if box_clicked:
                    # button: get id
                    box_center = np.mean(bounding_box[box_clicked_id], axis=0)
                    BUTTON_ID_X, BUTTON_ID_Y = box_center + map_offset + np.array([50, -125])
                    if BUTTON_ID_X <= mouse[0] <= BUTTON_ID_X + BUTTON_WIDTH and BUTTON_ID_Y <= mouse[1] <= BUTTON_ID_Y + BUTTON_HEIGHT:
                        print('get id')
                box_clicked = False
                bounding_box_copy = bounding_box.copy()
                for idx, box in bounding_box_copy.items():
                    p1 = Point(mouse)
                    p2 = Polygon(box + map_offset)
                    if p2.contains(p1):
                        print('click box {}'.format(idx))
                        box_clicked = True
                        box_clicked_id = idx
                        break
            elif event.type == pygame.MOUSEBUTTONUP and mods & pygame.KMOD_CTRL:
                if event.button == 1:            
                    map_draging = False
            elif event.type == pygame.MOUSEMOTION and mods & pygame.KMOD_CTRL:
                if map_draging:
                    end_pos = event.pos
                    map_offset = map_offset + end_pos - start_pos
                    start_pos = end_pos

        # send goal
        if robot_goal is not None:
            if cnt % 10 == 0: 
                sendGoal(screen2pos(*robot_goal))
        
        # view image
        if view_image and robot_img is not None:
            cv2.imshow('Robot Image', robot_img)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                view_image = False
                cv2.destroyAllWindows()

        pygame.display.update()
        end_time = time.time()
        # print('frequency', 1/(end_time-start_time))
