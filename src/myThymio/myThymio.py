from .thymio import Thymio
from utils import get_distance, particle_filter, controller, sensor_2_distance, get_obstacle_points
from visuals import Interface
import serial
import time
import math
import numpy as np
from myThymio.thymio_constants import *

GROUND_THRESHOLD = 500
NUMBER_OF_PARTICLES = 70

WAYPOINT_REACHED_THRESHOLD_MM = 15

OBSTACLE_DISTANCE_THRESHOLD_MM = 100

AVOIDANCE_DISTANCE_MM = 20

LOOP_PATH = [(x[0] + 111, x[1] + 73, x[2]) for x in [(0, 0, 0), (210, 0, 0), (210, 148.5, 0), (0, 148.5, 0)]]

class MyThymio(Thymio):

    def __init__(self, port="COM3", refreshing_rate=0.1, robot_path=LOOP_PATH):

        if port is None:
            port = Thymio.serial_default_port()
        node_id = 1
        Thymio.__init__(self, serial.Serial(port), node_id, refreshing_rate)
        self.handshake()

        self.state = "stopped"
        self.pause = False

        self.robot_path = robot_path

        self.current_pos, self.target_pos, self.particle_pos_list = None, None, None

        self.interface = Interface(self.robot_path, {"pause": self.pause_continue_robot_path})

        self.last_update = time.time()
        print("Waiting for setup.")
        time.sleep(3)
        self.set_speed(0, 0)
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
        self.current_pos, self.target_pos = robot_path.pop(0), robot_path.pop(0)
        self.particle_pos_list = [self.current_pos for _ in range(NUMBER_OF_PARTICLES)]

        try:
            while True:
                self.interface.update_viz(self.current_pos, self.target_pos, self.particle_pos_list)
                current_time = time.time()
                time_diff = current_time - self.last_update
                self.last_update = current_time

                speed_left, speed_right = self.measure_speeds()
                ground_left_measure, ground_right_measure = self.measure_ground_sensors()

                self.particle_pos_list = particle_filter(
                    self.particle_pos_list,
                    speed_left * THYMIO_SPEED_TO_MMS,
                    speed_right * THYMIO_SPEED_TO_MMS,
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

                # speed_left, speed_right = 50, 50

                if not self.pause:

                    sensor_distances = self.check_obstacle()
                    # print(sensor_distances)
                    if sensor_distances:
                        print("we are in local avoidance")
                        point1, point2 = get_obstacle_points(sensor_distances)
                        # if np.inf not in point1 and np.inf not in point2:
                        #     speed_left, speed_right = self.avoidance_control(np.array(point1), np.array(point2))

                    self.set_speed(speed_left * MMS_TO_THYMIO_SPEED, speed_right * MMS_TO_THYMIO_SPEED)
                else:
                    self.set_speed(0, 0)

        except KeyboardInterrupt:
            # self.close()
            print("we exit the run loop")
            self.set_speed(0, 0)