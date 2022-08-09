#!/usr/bin/env python
#-*- coding: utf-8 -*-

from this import d
import rospy
from time import sleep

# Steering_angle --> 입력 가능한 범위가 -0.34 ~ +0.34 까지 입력 가능
# speed --> 실차량 기준은 -10m/s ~ 10m/s (권장 사항은 -2.5~ 2.5m/s 만 사용하는 것을 권장)

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image 

from obstacle_detector.msg import Obstacles
# from obstacle_detector.msg import SegmentObstacle
# from obstacle_detector.msg import CircleObstacle

from cv_bridge import CvBridge 
import cv2
import numpy as np

from warper import Warper
from slidewindow import SlideWindow
from rabacon_drive import ClusterLidar

class Controller():
    def __init__(self):
        
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.rabacon_mission = ClusterLidar()
        self.bridge = CvBridge()

        # first lane direction 
        self.current_lane = "LEFT"
        self.change_lane_flag = 0

        self.warning = False # 초기에는 안전하다고 가정하고 시작

        self.initialized = False # lane_callback

        self.slide_img = None 
        self.slide_x_location = 0

        self.obstacle_cnt = 0

        self.lane_cnt = 0

        self.detect_car = ""

        self.rabacon_mission_flag = False
        self.rabacon_mission_angle = 180

        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.lane_callback) 
        rospy.Subscriber("lidar_warning", String, self.warning_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
       
        rospy.Subscriber("", Obstacles, self.rabacon_callback)
    


    def warning_callback(self, _data):
        if _data.data == "warning":
            self.warning = True
        elif self.rabacon_mission_flag == True :
            self.rabacon_mission_flag = True
        elif _data.data == "safe":
            self.warning = False
        elif _data.data == "STOPPED_CAR":
            self.detect_car = "CAR_DETECT"
        else:
            rospy.logwarn("Unkown warning state!!")


    def lane_callback(self, _data): 

        # rospy.loginfo("lane_callback in!!!!!!!!")

        # if self.initialized == False :
        #     cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL)
        #     cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, nothing)
        #     cv2.createTrackbar('low_S', 'Simulator_Image', 220, 255, nothing)
        #     cv2.createTrackbar('low_V', 'Simulator_Image', 145, 255, nothing)
        #     cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
        #     cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
        #     cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
        #     self.initialized = True

        cv_image = self.bridge.imgmsg_to_cv2(_data, "bgr8")
        # ret, lane_image2 = cv2.threshold(lane_image, 120, 255, cv2.THRESH_BINARY)
        if self.initialized == False:
            cv2.namedWindow("lane_image", cv2.WINDOW_NORMAL)
            cv2.createTrackbar('gray', 'lane_image', 0, 255, nothing)
            self.initialized = True

        gray = cv2.getTrackbarPos('gray', 'lane_image')


        # H(Hue : 색상), S(Saturation : 채도), V(Value : 명도)
        # low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
        # low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
        # low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
        # high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
        # high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
        # high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')
        

        # cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) # BGR to HSV
        lane_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)



        # lower_lane = np.array([low_H, low_S, low_V]) # 픽셀의 hsv값이 lower와 upper 사이에 있다 -> '1' / 없다 -> '0'
        # upper_lane = np.array([high_H, high_S, high_V])
        lane_image2 = cv2.inRange(lane_image, gray, 255)
        # lane_image = cv2.inRange(cv_image, lower_lane, upper_lane)

        cv2.imshow("lane_image", lane_image2)
        

        self.lane_detection(lane_image2)


        cv2.waitKey(1)

    def timer_callback(self, _event):
        # if self.right_lane_x == 0: # 차선이 검출되지 않는 경우 멈추도록
        #     self.stop() H 
        #     rospy.loginfo("lane is not detected")
        #     return
        # rospy.loginfo("Timer Callback in!!!!!")
        
        
        
        try :
            self.follow_lane()
        except :
            pass
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

    def rabacon_callback(self, _rabacon) :
        self.rabacon_mission_flag, self.rabacon_mission_angle = self.rabacon_mission.rabacon(_rabacon.circles)

  
    def lane_detection(self, lane_image) :
        kernel_size = 5
        blur_img = cv2.GaussianBlur(lane_image,(kernel_size, kernel_size), 0)
        warped_img = self.warper.warp(blur_img)
        cv2.imshow("warped_img", warped_img)
        self.slide_img, self.slide_x_location, self.current_lane = self.slidewindow.slidewindow(warped_img)
        cv2.imshow("slide_img", self.slide_img)
    


    def follow_lane(self): # 차선데이터를 받아서 차선을 따라서 주행하도록 하는 함수
        
        if self.warning: # object 탐지 시 멈추도록  --> 이런 로직 이용해서 정적 장애물 회피 로직으로 수정하면 될 듯 
            self.stop()
        elif self.rabacon_mission_flag == True :
            self.follow_rabacon()
        elif self.detect_car == "CAR_DETECT" and self.current_lane == "RIGHT" :
            #self.lane_change = True
            self.change_line_left()
        elif self.detect_car == "CAR_DETECT" and self.current_lane =="LEFT" :
            self.change_line_right()
        
            # rospy.loginfo("LiDAR object is detected!!")
        else:
            rospy.loginfo("DEFAULT DRIVE!!!!!!!!!")

            self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
            # error가 음수 --> 오른쪽 차선이랑 멈 --> 오른쪽으로 조향(sttering_angle < 0)
            # error가 양수 --> 오른쪽 차선이랑 가까움 --> 왼쪽으로 조향(sttering_angle > 0)

            # rospy.loginfo("slide_x_location = {}".format(self.slide_x_location))        
            # rospy.loginfo("error lane = {}".format(self.error_lane))
            publishing_data = AckermannDriveStamped()
            publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            publishing_data.header.frame_id = "base_link"
            publishing_data.drive.steering_angle = self.error_lane * 0.005 # error 정도에 따라서 조향 
            publishing_data.drive.speed = 0.3
            self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
        # rospy.loginfo("Publish Control Data!")


    def stop(self): 
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = 0.0
        publishing_data.drive.speed = 0.0
        self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
        # rospy.loginfo("Stop Vehicle!")




    ####################################################################################
    # protect child

    ####################################################################################
    # dynamic obstacle

    ####################################################################################
    # static obstacle
    # def change_line_left(self) :
    #     publishing_data = AckermannDriveStamped()
    #     publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
    #     publishing_data.header.frame_id = "base_link"
    #     publishing_data.drive.speed = 0.3
    #     ch_angle = 0
    #     for i in range(10) :
    #         ch_angle+=0.1
    #         publishing_data.drive.steering_angle+=ch_angle
    #         # sleep(0.01)
    #         self.drive_pub.publish(publishing_data)
    #     self.change_line_right(self)
    cnt_lane = 0
    def change_line_left(self) :
        self.cnt_lane += 1
        rospy.loginfo("change_left!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = 0.2
        publishing_data.drive.steering_angle = 3
        self.drive_pub.publish(publishing_data)
        
        sleep(0.05)
        # publishing_data.drive.steering_angle -= 0.5
        # self.drive_pub.publish(publishing_data)
        self.detect_car = " "

        # if self.cnt_lane % 7 == 0: 
        #     publishing_data.drive.steering_angle -= 0.5
        #     self.follow_lane()

    def change_line_right(self) :
        self.cnt_lane += 1
        rospy.loginfo("change_right!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = 0.2
        publishing_data.drive.steering_angle = -3
        self.drive_pub.publish(publishing_data)
        
        sleep(0.05)
        # publishing_data.drive.steering_angle -= 0.5
        # self.drive_pub.publish(publishing_data)
        self.detect_car = " "   
    ####################################################################################
    # Labacon Mission
    def follow_rabacon(self) :
        rospy.loginfo("rabacon_drive!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = 0.2
        publishing_data.drive.steering_angle = self.rabacon_mission_angle
        self.drive_pub.publish(publishing_data)

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
