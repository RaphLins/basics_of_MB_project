from .thymio import Thymio
from utils import get_distance, particle_filter, controller, sensor_2_distance, get_obstacle_points, NUMBER_OF_PARTICLES
from visuals import Interface
import serial
import time
import math
import numpy as np
from numpy import sqrt, pi
from myThymio.thymio_constants import *
from enum import Enum

class State(Enum):
    GLOBAL_AVOIDANCE = 1
    LOCAL_AVOIDANCE = 2
    LOCAL_TO_GLOBAL = 3

GROUND_THRESHOLD = 500

LOCAL_AVOIDANCE_DISTANCE_THRESHOLD = 100

WAYPOINT_REACHED_THRESHOLD_MM = 25

OBSTACLE_DISTANCE_THRESHOLD_MM = 100

AVOIDANCE_DISTANCE_MM = 55

LOOP_PATH = [(x[0] + 2*111, x[1] + 147, x[2]) for x in [(2*210, 0, 0), (2*210, 2*148.5, -pi/2), (0, 2*148.5, pi), (0, 0, pi/2)]]
LOOP_PATH = [(x[0] + 111, x[1] + 300, x[2]) for x in [(2*280, 0, 0), (0, 0, pi/2)]]

saved_data = []

class MyThymio(Thymio):

    def __init__(self, port="COM3", initial_pos=LOOP_PATH[0], refreshing_rate=0.1, robot_path=LOOP_PATH, save_data=False):

        if port is None:
            port = Thymio.serial_default_port()
        node_id = 1
        Thymio.__init__(self, serial.Serial(port), node_id, 2*refreshing_rate)
        self.refreshing_rate = refreshing_rate
        self.handshake()

        self.state = "stopped"
        self.pause = False
        self.save_data = save_data

        self.robot_path = robot_path

        self.initial_pos = initial_pos
        self.current_pos, self.target_pos, self.particle_pos_list = initial_pos, None, None

        self.interface = Interface(self.robot_path, {"pause": self.pause_continue_robot_path})

        self.last_update = time.time()
        print("Waiting for setup.")
        time.sleep(3)
        self.set_speed(0, 0)

        self.state = State.GLOBAL_AVOIDANCE

        self.distance_since_obstacle = 0

        print("Completed.")

    def set_speed(self, left, right):
        left = int(round(left))
        right = int(round(right))
        MAX_SPEED = 200

        left = min(MAX_SPEED, max(-MAX_SPEED, left))
        right = min(MAX_SPEED, max(-MAX_SPEED, right))

        if left >= 0:
            self.set_var("motor.left.target", left)
        else:
            self.set_var("motor.left.target", 2 ** 16 + left)
        if right >= 0:
            self.set_var("motor.right.target", right)
        else:
            self.set_var("motor.right.target", 2 ** 16 + right)

    def measure_speeds(self):
        speed_left = self["motor.left.speed"]
        speed_right = self["motor.right.speed"]

        try:
            if speed_left > 2 ** 16 / 2:
                speed_left -= 2 ** 16

            if speed_right > 2 ** 16 / 2:
                speed_right -= 2 ** 16

            if abs(speed_left) < 7:
                speed_left = 0
            if abs(speed_right) < 7:
                speed_right = 0
        except Exception:
            speed_left, speed_right = None, None
            print("could not read speed!!")

        return speed_left, speed_right

    def measure_ground_sensors(self):
        ground_left_measure = self["prox.ground.delta"][0]
        ground_right_measure = self["prox.ground.delta"][1]
        # print(ground_left_measure, ground_right_measure)
        # white = 1000, black = 200
        return ground_left_measure > GROUND_THRESHOLD, ground_right_measure > GROUND_THRESHOLD

    def pause_continue_robot_path(self, event):
        self.pause = not self.pause

    def check_obstacle(self, obstacle_treshold=OBSTACLE_DISTANCE_THRESHOLD_MM, verbose=False):
        """
        Tests whether one of the proximity sensors saw a wall
        param wall_threshold: threshold starting which it is considered that the sensor saw a wall
        param verbose: whether to print status messages or not
        """
        sensor_vals = self['prox.horizontal'][:-2]
        sensors_dist = list(map(sensor_2_distance, sensor_vals, range(5)))

        for i in range(len(sensors_dist)):
            if sensors_dist[i] > obstacle_treshold:
                sensors_dist[i] = obstacle_treshold

        if any([x < obstacle_treshold for x in sensors_dist]):
            if verbose:
                print("\t\t Saw a wall")
            return sensors_dist

        return None

    def avoidance_control(self, point1, point2, verbose=False):
        """
        avoidance control
        """

        # update the minimum distance to goal reached in avoidance

        v_wall = point2 - point1
        v_robot = v_wall[::-1]
        v_robot[1] = -v_robot[1]
        v_sensor = point1
        d_current = (v_robot[0] * v_sensor[0] + v_robot[1] * v_sensor[1]) / get_distance(point2, point1)

        v_theta = AVOIDANCE_DISTANCE_MM * v_wall + (d_current - AVOIDANCE_DISTANCE_MM) * v_robot

        theta = math.atan(v_theta[1]/v_theta[0])

        delta = np.array([500 * math.cos(theta), 500 * math.sin(theta)])
        new_target = list(np.array(self.current_pos[:2]) + delta) + [theta]
        speed_left, speed_right = controller(self.current_pos, new_target)

        return speed_left, speed_right

    def run(self):

        robot_path = self.robot_path.copy()
        self.target_pos = robot_path.pop(0)
        self.particle_pos_list = [self.current_pos for _ in range(NUMBER_OF_PARTICLES)]

        # self.particle_pos_list = []
        # N = int(sqrt(NUMBER_OF_PARTICLES))
        # for i in range(N):
        #     for j in range(N):
        #         self.particle_pos_list.append((111+i*840/N/2, 73+j*600/N/2, 0))

        self.last_update = time.time()

        try:
            while True:
                current_time = time.time()
                if current_time - self.last_update > self.refreshing_rate:
                    time_diff = current_time - self.last_update
                    # print(time_diff)
                    self.last_update = current_time

                    speed_left_measure, speed_right_measure = self.measure_speeds()
                    ground_left_measure, ground_right_measure = self.measure_ground_sensors()

                    if self.save_data:
                        saved_data.append((speed_left_measure, speed_right_measure, ground_left_measure, ground_right_measure, time_diff))

                    self.particle_pos_list = particle_filter(
                        self.particle_pos_list,
                        speed_left_measure * THYMIO_SPEED_TO_MMS,
                        speed_right_measure * THYMIO_SPEED_TO_MMS,
                        ground_left_measure,
                        ground_right_measure,
                        time_diff,
                    )

                    self.current_pos = np.mean(self.particle_pos_list, axis=0)

                    if get_distance(self.current_pos, self.target_pos) < WAYPOINT_REACHED_THRESHOLD_MM:
                        # print("Target reached.")
                        if robot_path:
                            self.target_pos = robot_path.pop(0)
                        else:
                            if self.robot_path == LOOP_PATH:
                                robot_path = self.robot_path.copy()
                            else:
                                if not self.pause:
                                    print("Path completed.")
                                self.pause = True

                    speed_left, speed_right = controller(self.current_pos, self.target_pos)

                    sensors_dist = self.check_obstacle(LOCAL_AVOIDANCE_DISTANCE_THRESHOLD)
                    # print(sensors_dist)

                    if sensors_dist is not None:
                        self.state = State.LOCAL_AVOIDANCE

                    if self.state == State.LOCAL_AVOIDANCE:

                        if sensors_dist is None:
                            self.state = State.LOCAL_TO_GLOBAL
                            self.distance_since_obstacle = 0
                        else:

                            obstSpeedGain = [1, 0, -5, -13, -16]

                            for i in range(5):
                                speed_left += (LOCAL_AVOIDANCE_DISTANCE_THRESHOLD - sensors_dist[i]) * obstSpeedGain[i] / 10
                                speed_right += (LOCAL_AVOIDANCE_DISTANCE_THRESHOLD - sensors_dist[i]) * obstSpeedGain[4 - i] / 10


                    if self.state == State.LOCAL_TO_GLOBAL:
                        speed_left = 40
                        speed_right = 40

                        v = (speed_left_measure + speed_right_measure)*THYMIO_SPEED_TO_MMS / 2
                        self.distance_since_obstacle += v*time_diff
                        # print(self.distance_since_obstacle)

                        if self.distance_since_obstacle >= AVOIDANCE_DISTANCE_MM:
                            self.state = State.GLOBAL_AVOIDANCE
                    # print(self.state)

                    if not self.pause:

                        sensor_distances = self.check_obstacle()
                        # print(sensor_distances)
                        if sensor_distances:
                            # print("we are in local avoidance")
                            point1, point2 = get_obstacle_points(sensor_distances)
                            # if np.inf not in point1 and np.inf not in point2:
                            #     speed_left, speed_right = self.avoidance_control(np.array(point1), np.array(point2))

                        self.set_speed(speed_left * MMS_TO_THYMIO_SPEED, speed_right * MMS_TO_THYMIO_SPEED)
                    else:
                        self.set_speed(0, 0)

                    self.interface.update_viz(self.current_pos, self.target_pos, self.particle_pos_list, ground_left_measure, ground_right_measure)

        except KeyboardInterrupt:
            # self.close()
            print("we exit the run loop")
            self.set_speed(0, 0)
            if self.save_data:
                import json
                with open('saved_data.txt', 'w') as filehandle:
                    json.dump((self.robot_path, self.initial_pos, saved_data), filehandle)


