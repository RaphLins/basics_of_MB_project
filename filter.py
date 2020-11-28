from math import pi, cos, sin, sqrt, atan2
import numpy as np
import cv2

class Particle:
    def __init__(self, position):
        self.pos = position

pattern = cv2.imread('images/pattern.png')
pattern = cv2.cvtColor(pattern, cv2.COLOR_BGR2GRAY)
pattern = cv2.resize(pattern, dsize=(420, 297), interpolation=cv2.INTER_CUBIC)


WHEELS_SPACING = 50
SENSOR_FORWARD_OFFSET = 50
SENSOR_PERP_OFFSET = 10

def sample_pattern(x, y):
    x = int(round(x))
    y = int(round(y))
    max_y = pattern.shape[0]
    max_x = pattern.shape[1]
    if x >= max_x or x < 0 or y >= max_y or y < 0:
        return 0
    return pattern[y][x]

def get_ground_sensor_pos(thymio_pos):
    x, y, theta = thymio_pos

    forward = np.array([cos(theta), sin(theta)])
    perp = np.array([forward[1], -forward[0]])

    ground_left = np.array([x, y]) + SENSOR_FORWARD_OFFSET * forward - SENSOR_PERP_OFFSET * perp
    ground_right = np.array([x, y]) + SENSOR_FORWARD_OFFSET * forward + SENSOR_PERP_OFFSET * perp
    return (ground_left[0], ground_left[1]), (ground_right[0], ground_right[1])

def sample_pattern_ground_sensors(thymio_pos):
    ground_left, ground_right = get_ground_sensor_pos(thymio_pos)
    ground_left_val = sample_pattern(ground_left[0], ground_left[1])
    ground_right_val = sample_pattern(ground_right[0], ground_right[1])
    #white = 255, black = 0
    return ground_left_val > 100, ground_right_val > 100

def ground_measurement_probability(thymio_pos, ground_left_measure, ground_right_measure):
    ground_left_val, ground_right_val = sample_pattern_ground_sensors(thymio_pos)
    # print("pattern:", ground_left_val, ground_right_val, "measured:", ground_left_measure, ground_right_measure)
    if ground_left_val == ground_left_measure:
        prob_left = 0.8
    else:
        prob_left = 0.2
    if ground_right_val == ground_right_measure:
        prob_right = 0.8
    else:
        prob_right = 0.2
    return prob_right*prob_left

def update_pos(current_pos, speed_left, speed_right, dt):
    x, y, theta = current_pos

    v = (speed_left + speed_right) / 2
    omega = (speed_right - speed_left) / (2 * WHEELS_SPACING)

    x = x + dt * v * cos(theta)
    y = y + dt * v * sin(theta)
    theta = theta + dt * omega

    return x, y, theta

def draw_particle(particles, weights):

    return None

def update_particles(current_pos, particles, speed_left_m, speed_right_m, ground_left_measure, ground_right_measure, dt):
    prob = ground_measurement_probability(current_pos, ground_left_measure, ground_right_measure)

    M = len(particles)
    NEW_GEN_NUMBER = 20
    sigma = 10

    weights = np.empty([len(particles)*NEW_GEN_NUMBER])

    new_generation = []
    i = 0
    for particle in particles:
        for j in range(NEW_GEN_NUMBER):
            speed_left = np.random.normal(speed_left_m, sigma)
            speed_right = np.random.normal(speed_right_m, sigma)
            new_pos = update_pos(particle.pos, speed_left, speed_right, dt)
            new_particle = Particle(new_pos)
            new_generation.append(new_particle)
            weights[i] = ground_measurement_probability(new_particle.pos, ground_left_measure, ground_right_measure)
            i+=1

    # resample
    new_particles = np.random.choice(new_generation, size=M, replace=True, p=weights/np.sum(weights))

    return new_particles