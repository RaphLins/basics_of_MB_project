import matplotlib.pyplot as plt
import cv2
import numpy as np


def find_rectangle(threshold_image, minimum_area, arclen, plot=True):
    # Detecting shapes in image by selecting region with same colors or intensity.
    contours, _ = cv2.findContours(threshold_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    board = np.array([])

    # Searching through every region selected to find the required polygon.
    for cnt in contours:
        area = cv2.contourArea(cnt)

        # Shortlisting the regions based on there area.
        if area > minimum_area:
            approx = cv2.approxPolyDP(cnt,
                                      arclen
                                      * cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                board = approx

    if not board.any():
        return
    else:
        if plot:
            n = len(board)
            for i in range(n):
                vertice1 = board[i][0]
                vertice2 = board[(i+1) % n][0]
                plt.scatter(vertice1[0], vertice1[1], c='r')
                plt.plot((vertice1[0], vertice2[0]), (vertice1[1], vertice2[1]), 'r')

    top_left_idx = None
    for i in range(len(board)):
        vertice1 = board[i][0]
        if vertice1[0] < threshold_image.shape[0]/2 and vertice1[1] < threshold_image.shape[1]/2:
            top_left_idx = i
            break
    new_board = []
    for i in range(len(board)):
        idx = (top_left_idx + i) % 4
        new_board.append(board[idx][0])

    return new_board


def remove_black(img, max_black):
    black_min = np.array([0, 0, 0])
    black_max = np.array([max_black, max_black, max_black])

    mask = cv2.inRange(img, black_min, black_max)

    img_no_black = img.copy()
    img_no_black[mask > 0] = (255, 255, 255)
    return img_no_black


def remove_white(img, min_white):
    black_min = np.array([min_white, min_white, min_white])
    black_max = np.array([255, 255, 255])

    mask = cv2.inRange(img, black_min, black_max)

    img_no_min_white = img.copy()
    img_no_min_white[mask > 0] = (0, 0, 0)

    return img_no_min_white
