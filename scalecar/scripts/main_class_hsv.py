#!/usr/bin/env python
#-*- coding: utf-8 -*-

from this import d
import rospy
from time import sleep, time
import statistics
from collections import deque

# Steering_angle --> 입력 가능한 범위가 -0.34 ~ +0.34 까지 입력 가능
# speed --> 실차량 기준은 -10m/s ~ 10m/s (권장 사항은 -2.5~ 2.5m/s 만 사용하는 것을 권장)


## HSV 50 50 30 그늘 있을 때
## battery 9.47

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, String, Float32
from sensor_msgs.msg import Image 
from fiducial_msgs.msg import Fiducial, FiducialArray
from obstacle_detector.msg import Obstacles

from cv_bridge import CvBridge 
import cv2
import numpy as np

from warper import Warper
from slidewindow import SlideWindow
from rabacon_drive import ClusterLidar
# from What_is_obstacle import DecideObstacle

class Controller():

    def __init__(self):

        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.bridge = CvBridge()
        self.rabacon_mission = ClusterLidar()
        # self.decide_obstacle = DecideObstacle()

        # initialization
        # competition day
        self.current_lane = "RIGHT" # which lane
        self.is_safe = True  # assume safe in the beginning
        self.initialized = False # lane callback
        self.speed_lane = 0.7 # default speed
        self.speed_rabacon = 0.5 # rabacon speed
        self.speed_turn = 0.3 # change to left or right speed
        self.speed_slow = 0.3 # slow down speed(child protection zone)
        self.speed_stop = 0.0 # stop

        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0
        self.current_lane_window = ""

        # for rabacon mission
        self.rabacon_mission_flag = False

        # for slow down mission(child protection zone)
        self.slow_flag = 0
        self.slow_down_flag = 0
        self.slow_t1 = 0.0
        self.sign_data = 0
        self.child_cnt = 0

        # for static obstacle
        self.static_flag = 0
        self.turn_left_flag = 0
        self.turn_right_flag = 0

        # for dynamic obstacle
        self.dynamic_flag = 0
        self.dynamic_flag2 = 0

        # for obstacle type detection
        #self.stop_cnt = 0

        self.obstacle_kind = "None"
        self.obstacle_img = []

        self.red_cnt = 0 # rabacon
        self.green_cnt = 0 # dynamic obstacle
        self.black_cnt = 0 # static obstacle

        self.y_list = []
        self.y_list_sort = []
        self.dynamic_obs_cnt = 0
        self.static_cnt = 0


        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)

        rospy.Subscriber("usb_cam/image_rect_color", Image, self.lane_callback) 
        # rospy.Subscriber("obstacle_mission", String, self.warning_callback)
        rospy.Subscriber("lidar_warning", String, self.warning_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)
        rospy.Subscriber("object_condition", Float32, self.object_callback) 
        
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)
        # rospy.Subscriber("lidar_warning", String, self.warning_callback)
        rospy.Subscriber("rabacon_drive", Float32, self.rabacon_callback)
        rospy.Subscriber("obstacles", Obstacles, self.rabacon_callback)
     

    def object_callback(self, _data):
        # self.dynamic_flag2 = _data.data
        if self.dynamic_flag != 1 or len(self.y_list) <= 19:
            self.y_list.append(_data.data)
            if len(self.y_list) >= 21 :
                del self.y_list[0]
        # rospy.loginfo("data:{}, last:{}, length:{}".format(_data.data, self.y_list[-1], len(self.y_list)))
        
        # if _data.data == "STATIC":
        #     rospy.logwarn("STATIC DeteCTED!!!!! TURN ")
        #     self.static_flag = 1
        # elif _data.data == "DYNAMIC":
        #     self.dynamic_flag = 1
        #     self.y_list = []
            #rospy.loginfo("DYNAMIC DETECTED, Pleas Wait !!")
        else:
            rospy.logwarn("Unknown warning state!")
            # rospy.logwarn("Data is : {}".format(_data.data))

    def warning_callback(self, _data):

        if self.rabacon_mission_flag == True :
            self.rabacon_mission_flag = True
        elif _data.data == "safe":
            self.is_safe = True
            #self.stop_cnt = 0
            self.y_list = []
            if self.dynamic_flag == 1 :
                self.dynamic_obs_cnt += 1
                if self.dynamic_obs_cnt >= 50 : # 50
                    self.dynamic_flag = 0
                    self.dynamic_obs_cnt = 0
            self.static_flag = 0
            # self.dynamic_flag = 0
        elif _data.data == "WARNING":
            self.is_safe = False
            rospy.loginfo("WARNING!")
        else:
            pass
            # rospy.logwarn("Unkown warning state!!")
            # rospy.logwarn("Data is : {}".format(_data.data))
        

    def rabacon_callback(self, _data) :

        self.rabacon_mission_flag = _data.data
        if self.rabacon_mission_flag < 10.0 :
            self.rabacon_mission = 1
            # self.y_list = []
            # self.y_list_sort = []
        else :
            self.rabacon_mission = 0

    
    # def obstacle_choice(self, crop_img) :

    #     r, g, b = self.decide_obstacle(crop_img)
    #     self.red_cnt += r
    #     self.green_cnt += g
    #     self.black_cnt += b
    #     #rospy.loginfo("red:", self.red_cnt, ", green:", self.green_cnt, ", black:", self.black_cnt)
    #     if self.red_cnt >= 30 :
    #         self.obstacle_kind = "rabacon"
    #     elif self.green_cnt >= 30 :
    #         self.obstacle_kind = "dynamic"
    #     else :
    #         self.obstacle_kind = "static"

    def lane_callback(self, _data):  
         
# ========  preprocessing via gray ==============================

        # cv_image = self.bridge.imgmsg_to_cv2(_data, "bgr8")

        # if self.initialized == False:
        #     cv2.namedWindow("lane_image", cv2.WINDOW_NORMAL)
        #     cv2.createTrackbar('gray', 'lane_image', 0, 255, nothing)
        #     self.initialized = True

        # gray = cv2.getTrackbarPos('gray', 'lane_image')

        # lane_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # lane_image2 = cv2.inRange(lane_image, gray, 255)

        # cv2.imshow("lane_image", lane_image2)

        # self.lane_detection(lane_image2)

        # cv2.waitKey(1)

# ========== preprocessing via HSV ==========================

        if self.initialized == False:
            cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) 
            cv2.createTrackbar('low_H', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('low_S', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('low_V', 'Simulator_Image', 50, 255, nothing)
            cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
            cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
            self.initialized = True

        cv2_image = self.bridge.imgmsg_to_cv2(_data)
        self.obstacle_img = cv2_image


        cv2.imshow("original", cv2_image) 

        low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
        low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
        low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
        high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
        high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
        high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')


        cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV) # BGR to HSV

        lower_lane = np.array([low_H, low_S, low_V]) # 
        upper_lane = np.array([high_H, high_S, high_V])

        lane_image = cv2.inRange(cv2_image, lower_lane, upper_lane)

        cv2.imshow("Lane Image", lane_image)
        self.lane_detection(lane_image)

        cv2.waitKey(1)


    def timer_callback(self, _event):
        try :
            self.follow_lane()
        except :
            pass


  
    def lane_detection(self, lane_image) :
        kernel_size = 5
        blur_img = cv2.GaussianBlur(lane_image,(kernel_size, kernel_size), 0)
        warped_img = self.warper.warp(blur_img)
        cv2.imshow("warped_img", warped_img)
        self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warped_img)
        cv2.imshow("slide_img", self.slide_img)
        # rospy.loginfo("CURRENT LANE WINDOW: {}".format(self.current_lane_window))


    def follow_lane(self): # 차선데이터를 받아서 차선을 따라서 주행하도록 하는 함수
                
        # slow_down_sign 
        if self.slow_down_flag == 1:
            
            #rospy.loginfo("sign data :{}".format(self.sign_data))
            if self.sign_data == 3:
                rospy.loginfo(" ===============   SLOW DETECTED, WAIT!!!! ============")
                self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
                publishing_data = AckermannDriveStamped()
                publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
                publishing_data.header.frame_id = "base_link"
                publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향 
                publishing_data.drive.speed = self.speed_lane
                self.drive_pub.publish(publishing_data) #  하는 부분 

            elif self.sign_data == 0 :
                rospy.loginfo("************* SLOW DOWN *****************")        
                self.child_cnt = 0        
                if self.slow_flag == 0:
                    self.slow_t1 = rospy.get_time()
                    self.slow_flag = 1
                t2 = rospy.get_time()
                #rospy.loginfo("t1 :{}, t2 : {}".format(self.slow_t1, t2))
                # the car should stop for more than 14 seconds
                while t2-self.slow_t1 <= 15 :
                    rospy.loginfo("************* SLOW DOWN *****************")
                    #rospy.loginfo("slow_down time{}, {}".format(self.slow_t1,t2))
                    self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
                    publishing_data = AckermannDriveStamped()
                    publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
                    publishing_data.header.frame_id = "base_link"
                    publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향 
                    publishing_data.drive.speed = self.speed_slow
                    self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분
                    t2 = rospy.get_time()
                self.slow_down_flag = 0
                self.slow_flag = 0
                # self.y_list = []
        
        # rabacon
        elif self.rabacon_mission == 1 :
            self.rabacon_drive()
            ## RABACON_OUT_LANE : COMPETITION DAY
            self.current_lane = "RIGHT"
            # self.y_list = []
            # self.current_lane = "LEFT"


        # obstacle
        elif self.is_safe == False:
            self.y_list_sort = sorted(self.y_list, key = lambda x:x)
            rospy.loginfo("Y_LIST{}".format(self.y_list))
            # rospy.loginfo()

            # if self.static_flag != 1 and self.is_safe == False:
            #     self.stop()
            # if self.dynamic_flag2 != 77.0:
            #     self.dynamic_flag = 1
                # if len(self.y_list ) >= 50 :
            # self.y_list = sorted(self.y_list, key = lambda x:x)
            # if self.is_safe == True :
            #     self.follow_lane()

            # # rospy.loginfo("y: {}, {}".format(self.y_list[5], self.y_list[-5]))
            # rospy.logwarn("average:{},{}".format(statistics.mean(self.y_list[5:15]), statistics.mean(self.y_list[-15:-5])))
            if len(self.y_list) <= 19 :
                self.stop()
                rospy.loginfo("obstacle_stop, dynamic_flag = {}", format(self.dynamic_flag))

            elif abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])) >= 0.17 or self.y_list_sort[10] < -0.15:
                self.dynamic_flag = 1
                self.static_flag = 0
                # self.y_list.clear
                rospy.loginfo("dynamic")
                # rospy.loginfo(self.y_list_sort)
                rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))
            else:
                self.static_cnt += 1
                if self.static_cnt >= 10 :
                    self.static_flag = 1
                    self.dynamic_flag = 0
                    self.static_cnt = 0
                rospy.loginfo("static")
                # rospy.loginfo(self.y_list_sort)
                rospy.loginfo(abs(statistics.mean(self.y_list_sort[0:1]) - statistics.mean(self.y_list_sort[-2:-1])))

            #dynamic
            if self.dynamic_flag == 1 and self.is_safe == False :
                rospy.logwarn("DYNAMIC OBSTACLE")
                self.stop()
                


            # static obstacle
            elif self.static_flag == 1:
                rospy.loginfo("STATIC OBSTACLE")
                # if the car is driving depending on "right" window
                if self.current_lane == "RIGHT":
                    rospy.logwarn("IN RIGHT")
                    if self.turn_left_flag == 0:
                        self.turn_left_t1 = rospy.get_time()
                        self.turn_left_flag = 1
                    t2 = rospy.get_time()
                    #rospy.loginfo("turn time{}, {}".format(self.turn_left_t1,t2))

                    while t2-self.turn_left_t1 <= 1.0:
                        self.change_line_left()
                        t2 = rospy.get_time()
                    while t2- self.turn_left_t1 <= 1.25 :
                        self.change_line_right()
                        t2 = rospy.get_time()
                    self.current_lane = "LEFT"
                    self.static_flag = 0
                    self.turn_left_flag = 0
                    # self.follow_lane()
                    # while self.current_lane_window != 'LEFT' :
                    #     self.change_line_left()
                    # self.current_lane = "LEFT"
                    # self.static_flag = 0
                    # self.follow_lane()
                
                # if the car is driving depending on "left" window
                elif self.current_lane == "LEFT":
                    rospy.logwarn("IN LEFT")
                    if self.turn_right_flag == 0:
                        self.turn_right_t1 = rospy.get_time()
                        self.turn_right_flag = 1
                    t2 = rospy.get_time()
                    #rospy.loginfo("turn time{}, {}".format(self.turn_right_t1,t2))

                    while t2-self.turn_right_t1 <= 1.0:
                        self.change_line_right()
                        t2 = rospy.get_time()
                    while t2-self.turn_right_t1 <= 1.25 :
                        self.change_line_left()
                        t2 = rospy.get_time()
                    self.current_lane = "RIGHT"
                    self.static_flag = 0
                    self.turn_right_flag = 0
                    # self.follow_lane()
            
            # dynamic obstacle
            # stop while dynamic flag is moving
            # if self.dynamic_flag == 1 and self.is_safe == False:
            #     rospy.logwarn("DYNAMIC OBSTACLE")
            #     self.stop()
            
            
            
        # no obstalce, no rabacon, no slow sign, just drive
        else:
            rospy.loginfo("DEFAULT DRIVE!!!!!!!!!")
            self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
            publishing_data = AckermannDriveStamped()
            publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            publishing_data.header.frame_id = "base_link"
            publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향 
            publishing_data.drive.speed = self.speed_lane
            self.drive_pub.publish(publishing_data) #  하는 부분 
            # if self.current_lane_window == "MID" :
            #     publishing_data = AckermannDriveStamped()
            #     publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            #     publishing_data.header.frame_id = "base_link"
            #     publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향 
            #     publishing_data.drive.speed = self.speed_lane
            #     self.drive_pub.publish(publishing_data) #  하는 부분 


    # if the car find any obstacle, the car should stop.
    def stop(self): 
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = 0.0
        publishing_data.drive.speed = self.speed_stop
        self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
        #rospy.loginfo("Stop Vehicle!")
        #self.stop_cnt += 1
        #print("STOP_CNT{}".format(self.stop_cnt))
        


    ####################################################################################
    # protect child
    def child_sign_callback(self, _data):
        try :
            # : {}".format(_data.data))

            if _data.data == 3:
                self.child_cnt += 1
                if self.child_cnt >=20 :
                    self.sign_data = _data.data
                    self.slow_down_flag = 1
                    self.child_cnt = 0
            else :
                self.sign_data = 0
                # self.slow_down_flag = 0
            #rospy.loginfo(" sign data_callback  : {}".format(self.sign_data))

            #if _data.data == 3:
            #    self.slow_down_flag = 1
                
        except :
            pass
        
    ####################################################################################
    # dynamic obstacle

    ####################################################################################
    # static obstacle
    def change_line_left(self) :
        
        #rospy.loginfo("change_left!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = self.speed_turn
        publishing_data.drive.steering_angle = 0.3
        self.drive_pub.publish(publishing_data)

    def change_line_right(self) :
        
        #rospy.loginfo("change_right!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        #left_time = time()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = self.speed_turn
        publishing_data.drive.steering_angle = -0.3
        self.drive_pub.publish(publishing_data)
        
    ####################################################################################
    # Rabacon Mission
    def rabacon_drive(self) :

        rospy.loginfo("rabacon_drive!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = self.speed_rabacon
        publishing_data.drive.steering_angle = self.rabacon_mission_flag * -1.0
        #print(self.rabacon_mission_flag * -1)
        self.drive_pub.publish(publishing_data)
        

    ####################################################################################


def nothing(x):
    pass


def run():

    rospy.init_node("main_class_run")
    control = Controller()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timer_callback) 
    rospy.spin()

if __name__ == "__main__":
    run()
