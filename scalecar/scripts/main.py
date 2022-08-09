#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image # sensor_msgs/Image 타입을 사용하기 위해 추가한 부분

from cv_bridge import CvBridge # Ros image -- > OpenCV Image로 변경하기 위해 필요한 부분\
import cv2
import numpy as np

from warper import Warper
from slidewindow import SlideWindow

bridge = CvBridge()
warper = Warper()
slidewindow = SlideWindow()

slide_x_location = 0
ref_slide_x_location = 320
initialized = False

cv_image = None



def main():
    rospy.init_node("main_control") 
    rospy.loginfo("main in !!!!!!!!!!!!!!!!!!!!!!!!!!")
    initialized = False # lane_callback 
    rospy.Subscriber("usb_cam/image_rect_color", Image, lane_callback) # Subscriber 생성자 필수 입력 3가지 --> Subscribe 하려는 Topic, 해당 Topic의 Message Type, 동작 callback 함수 
    drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)

    rospy.Timer(rospy.Duration(1.0/30.0), timer_callback) # 1초에 5번 처리 


    rospy.spin()


def timer_callback(_event):
        # if self.right_lane_x == 0: # 차선이 검출되지 않는 경우 멈추도록
        #     self.stop() 
        #     rospy.loginfo("lane is not detected")
        #     return
            
        if warning: # object 탐지 시 멈추도록  --> 이런 로직 이용해서 정적 장애물 회피 로직으로 수정하면 될 듯 
            stop()
            rospy.loginfo("LiDAR object is detected!!")
        else:
            # default 주행

            follow_lane()

            publishing_data = AckermannDriveStamped()
            publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            publishing_data.header.frame_id = "base_link"
            publishing_data.drive.steering_angle = 0.0
            publishing_data.drive.speed = 1.0 
            drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
            rospy.loginfo("Publish Control Data!")


        # self.follow_lane()

def follow_lane(): # 차선데이터를 받아서 차선을 따라서 주행하도록 하는 함수

    error_lane = ref_slide_x_location - slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
    # error가 음수 --> 오른쪽 차선이랑 멈 --> 오른쪽으로 조향(sttering_angle < 0)
    # error가 양수 --> 오른쪽 차선이랑 가까움 --> 왼쪽으로 조향(sttering_angle > 0)
    rospy.loginfo("error lane = {}".format(self.error_lane))
    publishing_data = AckermannDriveStamped()
    publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
    publishing_data.header.frame_id = "base_link"
    publishing_data.drive.steering_angle = error_lane * 0.001 # error 정도에 따라서 조향 
    publishing_data.drive.speed = 0.7
    drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
    rospy.loginfo("Publish Control Data!")



def lane_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    global initialized
    rospy.loginfo("lane_callback in!!!!!!!!")
    if initialized == False :
        cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) # 창의 크기를 조정할 수 있도록 함(이 부분이 없으면 AUTUSIZE 적용됨)
        cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, nothing)
        cv2.createTrackbar('low_S', 'Simulator_Image', 160, 255, nothing)
        cv2.createTrackbar('low_V', 'Simulator_Image', 145, 255, nothing)
        cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
        cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
        cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
        initialized = True


    cv2.imshow("cv2_image", cv_image)

    # H(Hue : 색상), S(Saturation : 채도), V(Value : 명도)
    low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
    low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
    low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
    high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
    high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
    high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')


    cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # BGR to HSV
    



    lower_lane = np.array([low_H, low_S, low_V]) # 픽셀의 hsv값이 lower와 upper 사이에 있다 -> '1' / 없다 -> '0'
    upper_lane = np.array([high_H, high_S, high_V])

    lane_image = cv2.inRange(cv_image, lower_lane, upper_lane) # crop + hsv 
    cv2.imshow("lane_img", lane_image)
    lane_detection(lane_image)


def stop(): 
    publishing_data = AckermannDriveStamped()
    publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
    publishing_data.header.frame_id = "base_link"
    publishing_data.drive.steering_angle = 0.0
    publishing_data.drive.speed = 0.0
    drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
    rospy.loginfo("Stop Vehicle!")

def nothing(x):
    pass



def lane_detection(lane_image):
    global slide_x_location
    warped_img = warper.warp(lane_image)
    cv2.imshow("warped_img", warped_img)
    slide__img, slide_x_location = slidewindow.slidewindow(warped_img)
    cv2.imshow("slide_img", slide_img)


if __name__ == '__main__':
    main()