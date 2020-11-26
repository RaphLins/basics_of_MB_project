import os
import sys
import time
import serial
import matplotlib
matplotlib.use("TkAgg")
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

Ts = 0.1
thymio_speed_to_mms = 0.36
mms_to_thymio_speed = 1/thymio_speed_to_mms
NUMBER_OF_PARTICLES = 100
DISP_REFRESH_RATE = 0.3

th = MyThymio(port="COM3", refreshing_rate=2 * Ts)
time.sleep(1)

def draw_thymio(thymio_pos, ax):
    x, y, theta = thymio_pos
    ground_left, ground_right = get_ground_sensor_pos(thymio_pos)
    l = 30
    ax.scatter(x, y, c='b')
    ax.scatter(ground_left[0], ground_left[1], c='r')
    ax.scatter(ground_right[0], ground_right[1], c='r')
    ax.plot((x, x + l * cos(theta)), (y, y + l * sin(theta)), 'b')

def draw_particles(particles, ax):
    for particle in particles:
        x, y, theta = particle.pos
        ax.scatter(x, y, c='m')

def distance(point1, point2):
    x1, y1, theta1 = point1
    x2, y2, theta2 = point2
    return sqrt((x1-x2)**2 + (y1-y2)**2)


# plotting
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
map_ax.imshow(1 - pattern, cmap='Greys', origin='lower')
map_ax.set_xlim(-50, 500)
map_ax.set_ylim(-50, 400)
plt.draw()
plt.show(block=False)

path = [(210, 0, 0), (210, 148.5, 0), (0, 148.5, 0), (0, 0, 0)]
path = [(x[0] + 111, x[1] + 73, x[2]) for x in path]

current_pos = path[-1]

particles = []
for i in range(NUMBER_OF_PARTICLES):
    particles.append(Particle(current_pos))
# for i in range(int(sqrt(NUMBER_OF_PARTICLES))):
#     for j in range(int(sqrt(NUMBER_OF_PARTICLES))):
#         particles.append(Particle((111+i*10, 73+j*10, 0)))

target_pos = path[0]
point_idx = 0

state = GLOBAL_PATHING

last_update = time.time()
last_display_update = time.time()
while True:
    current_time = time.time()
    if current_time - last_update > Ts:
        print(current_time - last_update)
        dt = current_time - last_update
        last_update = current_time
        # measurements
        speed_left, speed_right = th.measure_speeds()
        ground_left_measure, ground_right_measure = th.measure_ground_sensors()

        # position estimation
        particles = update_particles(current_pos, particles, speed_left * thymio_speed_to_mms, speed_right * thymio_speed_to_mms, ground_left_measure, ground_right_measure, dt)
        n = len(particles)
        current_pos = (sum(p.pos[0] for p in particles)/n, sum(p.pos[1] for p in particles)/n, sum(p.pos[2] for p in particles)/n)

        # if current_pos[0] >=path[0][0]:
        #     stop = True
        # if current_pos[2] >=pi:
        #     stop = True

        if state == GLOBAL_PATHING:
            # print("Global pathing")
            if distance(current_pos, target_pos) < 15:
                point_idx += 1
                if point_idx == len(path):
                    point_idx = 0
                target_pos = path[point_idx]


            speed_left, speed_right = controller(current_pos, target_pos)

            if stop:
                th.set_speed(0, 0)
            else:
                th.set_speed(speed_left*mms_to_thymio_speed, speed_right*mms_to_thymio_speed)
                # th.set_speed(100, 100)

        elif state == LOCAL_AVOIDANCE:
            print("Local avoidance")

    current_time = time.time()
    if current_time - last_display_update > DISP_REFRESH_RATE:
        last_display_update = current_time
        map_ax.clear()
        map_ax.imshow(1 - pattern, cmap='Greys', origin='lower')
        map_ax.set_xlim(-50, 500)
        map_ax.set_ylim(-50, 400)
        for point in path:
            x, y, theta = point
            map_ax.scatter(x, y, c='r')
        x, y, theta = target_pos
        map_ax.scatter(x, y, c='g')
        draw_particles(particles, map_ax)
        draw_thymio(current_pos, map_ax)
        plt.draw()
        plt.pause(0.001)