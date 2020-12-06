import cv2
import math


def sample_image(obstacle_map, x, y, outside_value=255):
    x = int(round(x))
    y = int(round(y))
    max_y = obstacle_map.shape[0]
    max_x = obstacle_map.shape[1]
    if x >= max_x or x < 0 or y >= max_y or y < 0:
        return outside_value
    else:
        return obstacle_map[y][x]


def get_pattern(pattern_path="images/checkboard.png"):
    if not pattern_path:
        pattern_path = "images/checkboard.png"

    pattern = cv2.bitwise_not(cv2.imread(pattern_path)[::-1, :, :])
    pattern = cv2.cvtColor(pattern, cv2.COLOR_BGR2GRAY)
    # _, pattern = cv2.threshold(pattern, 70, 255, cv2.THRESH_BINARY)
    pattern = cv2.resize(pattern, dsize=((420-5)*2, (297-5)*2))
    return pattern


def get_distance(point1, point2):
    """returns the distance between 2 points with format (x, y, theta)"""
    if len(point1) == 3:
        x1, y1, _ = point1
    else:
        x1, y1 = point1

    if len(point2) == 3:
        x2, y2, _ = point2
    else:
        x2, y2 = point2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
