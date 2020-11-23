from math import pi, cos, sin, sqrt, atan2
import numpy as np
WHEELS_SPACING = 48

def update_pos(current_pos, speed_left, speed_right, dt):
    x, y, theta = current_pos

    v = (speed_left + speed_right) / 2
    omega = (speed_right - speed_left) / (2 * WHEELS_SPACING)

    x = x + dt * v * cos(theta)
    y = y + dt * v * sin(theta)
    theta = theta + dt * omega

    return x, y, theta

def update_particles(current_pos, particles, speed_left_m, speed_right_m, dt):
    new_particles = []
    for particle in particles:
        sigma = 20
        speed_left = np.random.normal(speed_left_m, sigma)
        speed_right = np.random.normal(speed_right_m, sigma)
        particle = update_pos(particle, speed_left, speed_right, dt)
        new_particles.append(particle)

    current_pos = update_pos(current_pos, speed_left_m, speed_right_m, dt)
    return current_pos, new_particles