import os
import sys
import time
import serial
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from math import pi, cos, sin, sqrt, atan2

from matplotlib.widgets import Button

from filter import *
from control import *
from vision import *
from my_thymio import MyThymio

GLOBAL_PATHING = 0
LOCAL_AVOIDANCE = 1
REFRESH_RATE = 0.05

th = MyThymio(port="COM3", refreshing_rate=REFRESH_RATE)
time.sleep(1)

def draw_thymio(thymio_position, ax):
    x, y, theta = thymio_position
    l = 60
    ax.scatter(x, y, c='b')
    ax.plot((x, x + l * cos(theta)), (y, y + l * sin(theta)), 'b')
    plt.pause(0.02)

def distance(point1, point2):
    x1, y1, theta1 = point1
    x2, y2, theta2 = point2
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def draw_particles(particles, ax):
    for particle in particles:
        x, y, theta = particle
        ax.scatter(x, y, c='g')

# plotting
matplotlib.use("TkAgg")
plt.ion()
map_fig = plt.figure()
map_ax = map_fig.add_subplot(111)

plt.subplots_adjust(bottom=0.2)
ctrl_ax = plt.axes([0.7, 0.05, 0.1, 0.075])

stop = False
stop_btn = Button(ctrl_ax, 'Stop')
def stop_loop(event):
    global stop
    stop = not stop
    print(stop)
stop_btn.on_clicked(stop_loop)

plt.show()

# constants
thymio_speed_to_mms = 0.4
mms_to_thymio_speed = 1/thymio_speed_to_mms

NUMBER_OF_PARTICLES = 10

particles = []
for i in range(NUMBER_OF_PARTICLES):
    particles.append((0, 0, 0))

theta = 0
current_pos = (0, 0, theta)

way_points_list = [(210, 0, 0), (210, 148.5, 0), (0, 148.5, 0), (0, 0, 0)]
target_pos = way_points_list[0]
point_idx = 0

state = GLOBAL_PATHING

last_update = time.time()
while True:
    current_time = time.time()

    if current_time - last_update > REFRESH_RATE:
        speed_left, speed_right = th.measure_speeds()
        current_pos, particles = update_particles(current_pos, particles, speed_left*thymio_speed_to_mms, speed_right*thymio_speed_to_mms, REFRESH_RATE)

        # if current_pos[0] >=100:
        #     stop = True
        # if current_pos[2] >=pi:
        #     stop = True


        map_ax.clear()
        map_ax.set_xlim(-50, 300)
        map_ax.set_ylim(-50, 300)
        for point in way_points_list:
            x, y, theta = point
            map_ax.scatter(x, y, c='r')
        x, y, theta = target_pos
        map_ax.scatter(x, y, c='g')
        draw_particles(particles, map_ax)
        draw_thymio(current_pos, map_ax)

        if state == GLOBAL_PATHING:
            # print("Global pathing")
            if distance(current_pos, target_pos) < 10:
                point_idx += 1
                if point_idx == len(way_points_list):
                    point_idx = 0
                target_pos = way_points_list[point_idx]


            speed_left, speed_right = controller(current_pos, target_pos)

            if stop:
                th.set_speed(0, 0)
            else:
                th.set_speed(speed_left*mms_to_thymio_speed, speed_right*mms_to_thymio_speed)
                # th.set_speed(-100, 100)

        elif state == LOCAL_AVOIDANCE:
            print("Local avoidance")
