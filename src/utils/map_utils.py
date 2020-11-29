import cv2
import math


def get_pattern(pattern_path="images/pattern.png"):
    if not pattern_path:
        pattern_path = "images/pattern.png"

    pattern = 1 - cv2.imread(pattern_path)
    pattern = cv2.cvtColor(pattern, cv2.COLOR_BGR2GRAY)
    pattern = cv2.resize(pattern, dsize=(420, 297), interpolation=cv2.INTER_CUBIC)
    return pattern


def get_distance(point1, point2):
    """returns the distance between 2 points with format (x, y, tetha)"""
    if len(point1) == 3:
        x1, y1, _ = point1
    else:
        x1, y1 = point1

    if len(point2) == 3:
        x2, y2, _ = point2
    else:
        x2, y2 = point2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
