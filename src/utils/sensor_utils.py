import numpy as np
import math
from utils import get_distance


REGRESSIONS_PARAMS = {
    0: [-0.00297, 14.4],
    1: [-0.00373, 18.1],
    2: [-0.00334, 16.4],
    3: [-0.00327, 15.9],
    4: [-0.00316, 15.4],
}

# Thymio outline
CENTER_OFFSET_MM = np.array([55, 55])
THYMIO_COORD_MM = (
    np.array([[0, 0], [110, 0], [110, 85], [102, 93], [80, 104], [55, 110], [31, 105], [9, 94], [0, 85], [0, 0]])
    - CENTER_OFFSET_MM
)

# Sensor positions and orientations
SENSOR_COORD_MM = np.array([[9, 94], [31, 105], [55, 110], [80, 104], [102, 93], [85, 0], [25, 0]]) - CENTER_OFFSET_MM

SENSOR_ANGLE_RAD = np.array([120, 105, 90, 75, 60, -90, -90]) * math.pi / 180


def sensor_2_distance(val, sensor_id):
    a, b = REGRESSIONS_PARAMS[sensor_id]

    distance = a * val + b

    if distance > 15:
        return np.inf

    else:
        return 10 * distance


def get_obstacle_pos(dist_to_sensor, sensor_id):
    """returns the relative pos of an obstacle detected by one sensor"""
    deltas_from_sensor = np.array(
        [dist_to_sensor * math.cos(SENSOR_ANGLE_RAD[sensor_id]), dist_to_sensor * math.sin(SENSOR_ANGLE_RAD[sensor_id])]
    )
    obstacle_pos = SENSOR_COORD_MM[sensor_id] + deltas_from_sensor
    return list(obstacle_pos)


def get_obstacle_points(sensor_distances):
    sensor_ids = list(np.argpartition(sensor_distances, 2)[:2])
    point1, point2 = list(map(get_obstacle_pos, [sensor_distances[i] for i in sensor_ids], sensor_ids))
    if get_distance(point1, (0, 0)) > get_distance(point2, (0, 0)):
        return point2, point1
    else:
        return point1, point2
