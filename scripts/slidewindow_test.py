#!/usr/bin/env python
#-*- coding: utf-8 -*-

import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import *
from matplotlib.pyplot import *
from warper import Warper


warper = Warper()
# ======================================== added =======================================================
import rospy
from sensor_msgs.msg import Image # sensor_msgs/Image 타입을 사용하기 위해 추가한 부분
from cv_bridge import CvBridge # Ros image -- > OpenCV Image로 변경하기 위해 필요한 부분

# ======================================== added =======================================================
from warper import Warper


warper = Warper()

class SlideWindow:
    def __init__(self):
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.slidewindow) # Subscriber 생성자 필수 입력 3가지 --> Subscribe 하려는 Topic, 해당 Topic의 Message Type, 동작 callback 함수 

        self.bridge = CvBridge()
        
        
        self.left_fit = None
        self.right_fit = None
        self.leftx = None
        self.rightx = None
        self.initialized = False

    
    def slidewindow(self,_data):

        # img = warper.warp(usb_cam/image_rect_color)
        if self.initialized == False:
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) # 창의 크기를 조정할 수 있도록 함(이 부분이 없으면 AUTUSIZE 적용됨)
            cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 160, 255, nothing)
            cv2.createTrackbar('low_V', 'Simulator_Image', 145, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
            self.initialized = True
        cv2_image = self.bridge.imgmsg_to_cv2(_data)


        # H(Hue : 색상), S(Saturation : 채도), V(Value : 명도)
        low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
        low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
        low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
        high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
        high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
        high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')


        cv2_image = self.bridge.imgmsg_to_cv2(_data)
        



        lower_lane = np.array([low_H, low_S, low_V]) # 픽셀의 hsv값이 lower와 upper 사이에 있다 -> '1' / 없다 -> '0'
        upper_lane = np.array([high_H, high_S, high_V])

        lane_image = cv2.inRange(cv2_image, lower_lane, upper_lane) # crop + hsv 

        cv2.imshow("lane_image", lane_image)
        warp_image = warper.warp(lane_image)
        cv2.imshow("Warp Image", warp_image)
  


        x_location = None
        # init out_img, height, width        
        out_img = np.dstack((warp_image, warp_image, warp_image)) * 255 # ??? 
        height = warp_image.shape[0]
        width = warp_image.shape[1]


        # num of windows and init the height
        window_height = 5
        nwindows = 30
        
        # find nonzero location in _data, nonzerox, nonzeroy is the array flatted one dimension by x,y 
        nonzero = warp_image.nonzero()
        #print nonzero 
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        #print nonzerox
        # init data need to sliding windows
        margin = 20 
        minpix = 10
        left_lane_inds = []
        right_lane_inds = []    
        
        # first location and segmenation location finder
        # draw line
        # 130 -> 150 -> 180
        pts_left = np.array([[width/2 - 70, height], [width/2 - 70, height - 60], [width/2 - 170, height - 80], [width/2 - 170, height]], np.int32)
        cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)
        pts_right = np.array([[width/2 + 20, height], [width/2 + 20, height - 80], [width/2 + 120, height - 110], [width/2 + 120, height]], np.int32)
        cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)
        #pts_center = np.array([[width/2 + 90, height], [width/2 + 90, height - 150], [width/2 - 60, height - 231], [width/2 - 60, height]], np.int32)
        #cv2.polylines(out_img, [pts_center], False, (0,0,255), 1)
        pts_catch = np.array([[0, 340], [width, 340]], np.int32)
        cv2.polylines(out_img, [pts_catch], False, (0,120,120), 1)



      
        # indicies before start line(the region of pts_left)
        # 337 -> 310
        good_left_inds = ((nonzerox >= width/2 - 170) & (nonzeroy >= nonzerox * 0.33 + 300) & (nonzerox <= width/2 - 90)).nonzero()[0]
        good_right_inds = ((nonzerox >= width/2 + 20) & (nonzeroy >= nonzerox * (-0.48) + 500) & (nonzerox <= width/2 + 120)).nonzero()[0]

        # left line exist, lefty current init
        line_exist_flag = None 
        y_current = None
        x_current = None
        good_center_inds = None
        p_cut = None

        # check the minpix before left start line
        # if minpix is enough on left, draw left, then draw right depends on left
        # else draw right, then draw left depends on right
        if len(good_left_inds) > minpix:
            line_flag = 1
            x_current = np.int(np.mean(nonzerox[good_left_inds]))
            y_current = np.int(np.mean(nonzeroy[good_left_inds]))
            max_y = y_current
        elif len(good_right_inds) > minpix:
            line_flag = 2
            x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
            y_current = np.int(np.max(nonzeroy[good_right_inds]))
        else:
            line_flag = 3
            # indicies before start line(the region of pts_center)
            # good_center_inds = ((nonzeroy >= nonzerox * 0.45 + 132) & (nonzerox >= width/2 - 60) & (nonzerox <= width/2 + 90)).nonzero()[0] 
            # p_cut is for the multi-demensional function
            # but curve is mostly always quadratic function so i used polyfit( , ,2)
        #    if nonzeroy[good_center_inds] != [] and nonzerox[good_center_inds] != []:
        #        p_cut = np.polyfit(nonzeroy[good_center_inds], nonzerox[good_center_inds], 2)

        if line_flag != 3:
            # it's just for visualization of the valid inds in the region
            for i in range(len(good_left_inds)):
                    _data = cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 1, (0,255,0), -1)
            # window sliding and draw
            for window in range(0, nwindows):
                if line_flag == 1: 
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    # draw rectangle
                    # 0.33 is for width of the road
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low + int(width * 0.27), win_y_low), (win_x_high + int(width * 0.27), win_y_high), (255, 0, 0), 1)
                    # indicies of dots in nonzerox in one square
                    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                    # check num of indicies in square and put next location to current 
                    if len(good_left_inds) > minpix:
                        x_current = np.int(np.mean(nonzerox[good_left_inds]))
                    
                    elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
                        p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2) 
                        x_current = np.int(np.polyval(p_left, win_y_high))
                    # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
                    if win_y_low >= 338 and win_y_low < 344:
                    # 0.165 is the half of the road(0.33)
                        x_location = x_current + int(width * 0.135) 
                else: # change line from left to right above(if)
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    cv2.rectangle(out_img, (win_x_low - int(width * 0.27), win_y_low), (win_x_high - int(width * 0.27), win_y_high), (0, 255, 0), 1)
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                    if len(good_right_inds) > minpix:
                        x_current = np.int(np.mean(nonzerox[good_right_inds]))
                    elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
                        p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2) 
                        x_current = np.int(np.polyval(p_right, win_y_high))
                    if win_y_low >= 338 and win_y_low < 344:
                    # 0.165 is the half of the road(0.33)
                        x_location = x_current - int(width * 0.135) 

                left_lane_inds.extend(good_left_inds)
        #        right_lane_inds.extend(good_right_inds)  

            #left_lane_inds = np.concatenate(left_lane_inds)
            #right_lane_inds = np.concatenate(right_lane_inds)

        #else:
            """
            # it's just for visualization of the valid inds in the region
            # for i in range(len(good_center_inds)):
            #     _data = cv2.circle(out_img, (nonzerox[good_center_inds[i]], nonzeroy[good_center_inds[i]]), 1, (0,0,255), -1)
            # try: 
            #    for window in range(0, nwindows):
            #        x_current = int(np.polyval(p_cut, max_y - window * window_height))
            #        if x_current - margin >= 0:
            #            win_x_low = x_current - margin
            #            win_x_high = x_current + margin
            #            win_y_low = max_y - (window + 1) * window_height
            #            win_y_high = max_y - (window) * window_height
            #            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
            #            cv2.rectangle(out_img, (win_x_low - int(width * 0.23), win_y_low), (win_x_high - int(width * 0.23), win_y_high), (0, 255, 0), 1)
            #            if win_y_low >= 338 and win_y_low < 344:
            #                x_location = x_current - int(width * 0.115)
            #except:
            #    pass
            """
        cv2.imshow("out_img", out_img)

        cv2.waitKey(1) 
        print("x_location : {}".format(x_location))

        return out_img, x_location

def nothing(x):
    pass


def run(): # 실제 코드 동작 시 실행할 부분
    rospy.init_node("camera_example") # Ros Master에 Node 등록하는 함수
    new_class = SlideWindow() # 실제 동작을 담당할 Object
    rospy.spin() # Ros Node가 종료되지 않고 Callback 함수를 정상적으로 실행해주기 위한 부분

if __name__ == "__main__" : # 해당 Python 코드를 직접 실행할 때만 동작하도록 하는 부분
    run() # 실제 동작하는 코드 
