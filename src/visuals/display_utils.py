import matplotlib.pyplot as plt
import cv2

def imshow(cv2_img):
    plt.imshow(cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB), origin='lower')
