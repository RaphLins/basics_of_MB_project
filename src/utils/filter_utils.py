import math
import numpy as np
from myThymio.thymio_constants import *
from utils import get_pattern

PATTERN = get_pattern()

def sample_pattern(x, y):
    x = int(round(x))
    y = int(round(y))
    max_y = PATTERN.shape[0]
    max_x = PATTERN.shape[1]
    if x >= max_x or x < 0 or y >= max_y or y < 0:
        return 0
    return PATTERN[y][x]


def get_ground_sensor_pos(thymio_pos):
    x, y, theta = thymio_pos

    forward = np.array([math.cos(theta), math.sin(theta)])
    perp = np.array([forward[1], -forward[0]])

    ground_left = np.array([x, y]) + SENSOR_FORWARD_OFFSET * forward - SENSOR_PERP_OFFSET * perp
    ground_right = np.array([x, y]) + SENSOR_FORWARD_OFFSET * forward + SENSOR_PERP_OFFSET * perp
    return (ground_left[0], ground_left[1]), (ground_right[0], ground_right[1])


def sample_pattern_ground_sensors(thymio_pos):
    ground_left, ground_right = get_ground_sensor_pos(thymio_pos)
    ground_left_val = sample_pattern(ground_left[0], ground_left[1])
    ground_right_val = sample_pattern(ground_right[0], ground_right[1])
    # white = 255, black = 0
    return ground_left_val < 100, ground_right_val < 100


def ground_measurement_probability(thymio_pos, ground_left_measure, ground_right_measure):
    ground_left_val, ground_right_val = sample_pattern_ground_sensors(thymio_pos)
    # print("PATTERN:", ground_left_val, ground_right_val, "measured:", ground_left_measure, ground_right_measure)
    if ground_left_val == ground_left_measure:
        prob_left = 0.8
    else:
        prob_left = 0.2
    if ground_right_val == ground_right_measure:
        prob_right = 0.8
    else:
        prob_right = 0.2
    return prob_right * prob_left


def update_pos(current_pos, speed_left, speed_right, dt):
    x, y, theta = current_pos

    v = (speed_left + speed_right) / 2
    omega = (speed_right - speed_left) / (2 * WHEELS_SPACING)

    x = x + dt * v * math.cos(theta)
    y = y + dt * v * math.sin(theta)
    theta = theta + dt * omega

    return x, y, theta


def draw_particle(particle_pos_list, weights):

    return None


def particle_filter(
    particle_pos_list, speed_left_m, speed_right_m, ground_left_measure, ground_right_measure, dt
):
    M = len(particle_pos_list)
    NEW_GEN_NUMBER = 50
    sigma = 10

    weights = np.empty([len(particle_pos_list) * NEW_GEN_NUMBER])

    new_generation = []
    i = 0
    for particle_pos in particle_pos_list:
        for j in range(NEW_GEN_NUMBER):
            speed_left = np.random.normal(speed_left_m, sigma)
            speed_right = np.random.normal(speed_right_m, sigma)
            new_pos = update_pos(particle_pos, speed_left, speed_right, dt)
            new_generation.append(new_pos)
            weights[i] = ground_measurement_probability(new_pos, ground_left_measure, ground_right_measure)
            i += 1

    # resample
    indices = range(len(new_generation))
    sample = np.random.choice(indices, size=M, replace=True, p=weights / np.sum(weights))

    new_particles = [new_generation[i] for i in sample]

    return new_particles
