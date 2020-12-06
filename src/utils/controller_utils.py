from math import pi, cos, sin, sqrt, atan2
from thymio_constants import WHEELS_SPACING


def normalize_angle(angle):
    new_angle = angle
    while new_angle <= -pi:
        new_angle += 2*pi
    while new_angle > pi:
        new_angle -= 2*pi
    return new_angle


def controller(current_pos, target_pos):
    x, y, theta = current_pos
    x_d, y_d, theta_d = target_pos

    x_r = x - x_d
    y_r = y - y_d
    rho = sqrt(x_r ** 2 + y_r ** 2)
    gamma = normalize_angle(atan2(y_r, x_r) - theta + pi)
    delta = normalize_angle(gamma + theta + theta_d)

    if gamma == 0:
        gamma = 0.0001

    k1 = 1.2  # affects linear speed
    k2 = 0.8  # affects rotational speed
    k3 = 0  # affects how much getting to the final angle matters

    v = k1 * rho * cos(gamma)**3
    omega = k2 * gamma + k1 * sin(gamma) * cos(gamma) / gamma * (gamma + k3 * delta)

    v = min(45, max(-45, v))
    omega = min(2.5, max(-2.5, omega))

    speed_left = (v - WHEELS_SPACING * omega / 2)
    speed_right = (v + WHEELS_SPACING * omega / 2)
    return speed_left, speed_right
