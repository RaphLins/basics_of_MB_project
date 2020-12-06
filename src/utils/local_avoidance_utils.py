from thymio_constants import LOCAL_AVOIDANCE_DISTANCE_THRESHOLD, THYMIO_SPEED_TO_MMS


def potential_field(sensors_dist, speed_left, speed_right):
    obstSpeedGain = [1, 0, -5, -13, -16]

    for i in range(5):
        speed_left += ((LOCAL_AVOIDANCE_DISTANCE_THRESHOLD - sensors_dist[i])
                       * obstSpeedGain[i] / 10)
        speed_right += ((LOCAL_AVOIDANCE_DISTANCE_THRESHOLD - sensors_dist[i])
                        * obstSpeedGain[4-i] / 10)
    return speed_left, speed_right


def update_dist_obst(speed_left_measure, speed_right_measure, time_diff, distance_since_obstacle):
    v = (speed_left_measure + speed_right_measure)*THYMIO_SPEED_TO_MMS / 2
    distance_since_obstacle += v*time_diff

    return distance_since_obstacle
