import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import *
from matplotlib.pyplot import *


TOTAL_CNT = 50

class DecideObstacle:
    
    def __init__(self):
        self.kind_of_obstacle = "None"
        

    def extractcolor(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        height, width = img.shape[:2]

        

        return self.kind_of_obstacle
