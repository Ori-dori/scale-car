#!/usr/bin/env python
#-*- coding: utf-8 -*-

from this import d
import rospy

# Pulisher 1개가 필요
# Topic name --> /high_level/ackermann_cmd_mux/input/nav_0
# Message Type --> ackermann_msgs/AckermannDriveStamped
# Steering_angle --> 입력 가능한 범위가 -0.34 ~ +0.34 까지 입력 가능
# speed --> 실차량 기준은 -10m/s ~ 10m/s (권장 사항은 -2.5~ 2.5m/s 만 사용하는 것을 권장)

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image # sensor_msgs/Image 타입을 사용하기 위해 추가한 부분

from cv_bridge import CvBridge # Ros image -- > OpenCV Image로 변경하기 위해 필요한 부분\
import cv2
import numpy as np

from warper import Warper
from slidewindow import SlideWindow

class Controller():
    def __init__(self):
        # self.right_lane_x = 0  # 실제 검출된 차선과 카메라 가운데 사이의 거리 0으로 초기화
        # self.ref_right_lane_x = 255 # 차선을 어느 위치에 놓고 주행할 지  -> x 값이 ref_x 값에 맞춰지도록 주행(reference)
        self.warning = False # 초기에는 안전하다고 가정하고 시작
        self.curve_angle = 0
        self.speed = 0
        self.x_location = 0 # middle line gap
        self.initialized = False # lane_callback
        self.slide_img = None 
        self.slide_x_location = 0
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.bridge = CvBridge()
        self.error_lane = 320
        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.lane_callback) # Subscriber 생성자 필수 입력 3가지 --> Subscribe 하려는 Topic, 해당 Topic의 Message Type, 동작 callback 함수 
        # rospy.Subscriber("right_lane_moment_x", Int32, self.lane_callback) # 오른쪽 차선 momentum x 값 subscribe
        # rospy.Subscriber("lidar_warning", String, self.warning_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        # rostopic hz /right_lane_moment --> 평균 hz : 24  --> 들어오는 카메라 데이터에 대해서 모두 처리해주기 위해서 30으로 늘림
        #rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback) # 1초에 5번 처리 -> =: 아래 주석된 내용

        
        # self.rate = rospy.Rate(5) # 데이터 publish하는 주기를 정하는 함수(1초에 5번)
        # while not rospy.is_shutdown():
        #     self.publish_data()
        #     self.rate.sleep() # 1초에 5번 보내고, 남은 시간동안에는 쉼
    
#        drive contorl

    def warning_callback(self, _data):
        if _data.data == "warning":
            self.warning = True
        elif _data.data == "safe":
            self.warning = False
        else:
            rospy.logwarn("Unkown warning state!!")


    def lane_callback(self, _data): 

        rospy.loginfo("lane_callback in!!!!!!!!")

        if self.initialized == False :
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) # 창의 크기를 조정할 수 있도록 함(이 부분이 없으면 AUTUSIZE 적용됨)
            cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 160, 255, nothing)
            cv2.createTrackbar('low_V', 'Simulator_Image', 145, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
            self.initialized = True

        cv_image = self.bridge.imgmsg_to_cv2(_data, "bgr8")
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
        self.lane_detection(lane_image)
        #self.follow_lane()

        cv2.waitKey(1)

    def timer_callback(self, _event):
        # if self.right_lane_x == 0: # 차선이 검출되지 않는 경우 멈추도록
        #     self.stop() H 
        #     rospy.loginfo("lane is not detected")
        #     return
        rospy.loginfo("Timer Callback in!!!!!")
        try :
            self.follow_lane()
        except :
            pass
        # if self.warning: # object 탐지 시 멈추도록  --> 이런 로직 이용해서 정적 장애물 회피 로직으로 수정하면 될 듯 
        #     self.stop()
        #     rospy.loginfo("LiDAR object is detected!!")
        # else:
        #     # default 주행

        #     self.follow_lane()

            # publishing_data = AckermannDriveStamped()
            # publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            # publishing_data.header.frame_id = "base_link"
            # publishing_data.drive.steering_angle = 0.0
            # publishing_data.drive.speed = 1.0 
            # self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
            # rospy.loginfo("Publish Control Data!")


        # self.follow_lane()


  
    def lane_detection(self, lane_image) :
        warped_img = self.warper.warp(lane_image)
        cv2.imshow("warped_img", warped_img)
        self.slide_img, self.slide_x_location = self.slidewindow.slidewindow(warped_img)
        cv2.imshow("slide_img", self.slide_img)
    

    def follow_lane(self): # 차선데이터를 받아서 차선을 따라서 주행하도록 하는 함수
    
        
        self.error_lane = 320 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
        # error가 음수 --> 오른쪽 차선이랑 멈 --> 오른쪽으로 조향(sttering_angle < 0)
        # error가 양수 --> 오른쪽 차선이랑 가까움 --> 왼쪽으로 조향(sttering_angle > 0)

        
        rospy.loginfo("error lane = {}".format(self.error_lane))
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = self.error_lane * 0.005 # error 정도에 따라서 조향 
        publishing_data.drive.speed = 0.3
        self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
        rospy.loginfo("Publish Control Data!")


    def stop(self): 
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = 0.0
        publishing_data.drive.speed = 0.0
        self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
        rospy.loginfo("Stop Vehicle!")




    ####################################################################################
    # protect child

    ####################################################################################
    # dynamic obstacle

    ####################################################################################
    # static obstacle

    ####################################################################################
    # Labacon Mission

    ####################################################################################


def nothing(x):
    pass


def run():
    rospy.init_node("main_class_run")
    control = Controller()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timer_callback) # 1초에 5번 처리 -> =: 아래 주석된 내용
    rospy.spin()

if __name__ == "__main__":
    run()