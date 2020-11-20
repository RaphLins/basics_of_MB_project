from math import pi, cos, sin, sqrt, atan2
WHEELS_SPACING = 47

def update_pos(current_pos, speed_left, speed_right, dt):
    x, y, theta = current_pos

    v = (speed_left + speed_right) / 2
    omega = (speed_right - speed_left) / (2 * WHEELS_SPACING)

    x = x + dt * v * cos(theta)
    y = y + dt * v * sin(theta)
    theta = theta + dt * omega

    return (x, y, theta)