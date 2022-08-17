#!/usr/bin/env python
#-*- coding: utf-8 -*-

from this import d
import rospy
from time import sleep, time

# Steering_angle --> 입력 가능한 범위가 -0.34 ~ +0.34 까지 입력 가능
# speed --> 실차량 기준은 -10m/s ~ 10m/s (권장 사항은 -2.5~ 2.5m/s 만 사용하는 것을 권장)

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

class Controller():
    def __init__(self):

        self.stop_cnt = 0
        
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.bridge = CvBridge()
        self.rabacon_mission = ClusterLidar()

        self.callback_flag = False

        self.double = False

        self.drive_angle = 0

        # first lane direction 
        self.current_lane = "RIGHT"

        # 초기에는 안전하다고 가정하고 시작
        self.is_safe = True 
        self.detect_cnt = 0
        self.last_cnt = 0

        # lane_callback
        self.initialized = False 

        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0
        self.current_lane_window = ""

        # for child sign to slow down 
        self.slow_down_flag = 0

        # for static obstacle variable initialization
        self.detect_car = ""
        self.obs_do = True
        self.obs_notdo = False
        # for dynamic obstacle variable initialization
        self.detect_dyn_obs = "NONE"

        self.rabacon_mission_flag = False
        self.rabacon_mission_angle = 0

        self.is_obs_static = 0
        self.is_obs_dynamic = 0

        self.slow_flag = 0

        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.lane_callback) 
        # rospy.Subscriber("obstacle_mission", String, self.warning_callback)
        #rospy.Subscriber("lidar_warning", String, self.warning_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)
        
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)
        rospy.Subscriber("lidar_warning", String, self.warning_callback)
        # rospy.Subscriber("rabacon_drive", Float32, self.rabacon_callback)
        # rospy.Subscriber("obstacles", Obstacles, self.rabacon_callback)
     

    def warning_callback(self, _data):
        # rospy.loginfo("point_cnt: {}".format(_data.data))
        if self.rabacon_mission_flag == True :
            self.rabacon_mission_flag = True
        elif _data.data == "safe":
            self.is_safe = True
            self.stop_cnt = 0
        elif _data.data == "WARNING":
            self.detect_car = "CAR_DETECT"
            self.is_safe = False
        elif _data.data == "DYNAMIC_OBS":
            self.detect_dyn_obs = "OBS_DETECT"
            self.is_safe = False
        elif _data.data > 0:
            self.detect_car = "OBSTACLE DETECTION"
            self.detect_cnt = _data.data
            rospy.logwarn("OBSTACLE DETECTION")
            
        elif _data.data == 0:
            self.detect_cnt = 0
            rospy.logwarn("깔-끔")
            
        else:
            rospy.logwarn("Unkown warning state!!")

    def obs_condition(self):
        rospy.logwarn("obs_condition")
        self.detect_car = "DEFAULT"
        rospy.loginfo("obstacle detection")
        
        static_cnt = 0
        dynamic_cnt = 0
        self.last_cnt = self.detect_cnt
        # one_time_msg = rospy.wait_for_message("lidar_warning", Int32, timeout=None)
        rospy.loginfo("&&&&&&& last_cnt : {}".format(self.last_cnt))
        rospy.loginfo("&&&&&&& detect_cnt : {}".format(self.detect_cnt))
        
        # a = time.time()
        # b = time.time()
        # while b- a < 5 :
        #     if self.last_cnt-1 <= self.detect_cnt <= self.last_cnt+1:        
        #         static_cnt += 1
        #     else : 
        #         dynamic_cnt += 1
        #     b = time.time()

        for i in range(400):
            rospy.loginfo("STOP : DETECTION{} ".format(i))

            if self.last_cnt-1 <= self.detect_cnt <= self.last_cnt+1:        
                static_cnt += 1
            else : 
                dynamic_cnt += 1

        print("static_cnt: {}, dynamic_cnt: {}".format(static_cnt, dynamic_cnt))

        # b = time.time()
        # print("???????????", b-a)

        if static_cnt > dynamic_cnt :
            rospy.logwarn("static obstacle")
            rospy.loginfo("static obstacle")
            self.last_cnt = 0
            self.is_obs_static = 1
        else:
            rospy.logwarn("dynamic obstacle")
            rospy.loginfo("dynamic obstacle")
            self.last_cnt = 0
            self.is_obs_dynamic = 1
        

    def rabacon_callback(self, _data) :
        self.rabacon_mission_flag = _data.data
        if self.rabacon_mission_flag < 10.0 :
            self.rabacon_drive()

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
        # if self.right_lane_x == 0: # 차선이 검출되지 않는 경우 멈추도록
        #     self.stop() H 
        #     rospy.loginfo("lane is not detected")
        #     return
        # rospy.loginfo("Timer Callback in!!!!!")       
        try :
            if self.callback_flag == False :
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

        # if self.detect_car == "CAR_DETECT" and flag == False:
        #     rospy.loginfo("#################################################")  
        #     self.stop()

        #     # obs is static
        #     if self.stop_cnt >= 300 :
        #         self.stop_cnt = 0
        #         # rospy.loginfo("STATIC OBSTACLE IS DETECTED!!!!!!!!, current lane : {}".format(self.current_lane))
                
        #         if self.current_lane == "RIGHT":
        #             self.callback_flag = True
        #             for i in range(500) :
        #                 self.detect_car = "safe"
        #                 print("@@@@@@@@@@@@@@@1")
        #                 self.change_line_left()

        #             if self.current_lane_window == "MID" :
        #                 print("@@@@@@@@@@@@@@@2")
        #                 for i in range(250) :
        #                     self.change_line_left()

        #             self.is_obs_static = 0
        #             self.current_lane = "LEFT"
        #             self.callback_flag = False
        #             for i in range(500) :
        #                 self.follow_lane(True)
                    
        #         elif self.current_lane == "LEFT":
        #             self.callback_flag = True
        #             for i in range(600) :
        #                 self.change_line_right()
        #             if self.current_lane_window == "MID" :
        #                 for i in range(300) :
        #                     self.change_line_right()
        #             self.is_obs_static = 0
        #             self.current_lane = "RIGHT"
        #             self.callback_flag = False
        #             self.follow_lane(True)
            
            
                
        # slow_down_sign 
        if self.slow_down_flag == 1:
            rospy.loginfo("************* SLOW DOWN *****************")
            for i in range(500) : 
                self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
                publishing_data = AckermannDriveStamped()
                publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
                publishing_data.header.frame_id = "base_link"
                publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향 
                publishing_data.drive.speed = 0.7
                self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분
            
            
            
            t2 = rospy.get_time()
            rospy.loginfo("t1 :{}, t2 : {}".format(t1, t2))
            while t2-t1 <= 14 :
                rospy.loginfo("slow_down time{}, {}".format(t1,t2))
                self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
                publishing_data = AckermannDriveStamped()
                publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
                publishing_data.header.frame_id = "base_link"
                publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향 
                publishing_data.drive.speed = 0.2
                self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분
                t2 = rospy.get_time()
            self.slow_down_flag = 0
            rospy.loinfo("time : {}".format(t2-t1))
        
        else:
            rospy.loginfo("DEFAULT DRIVE!!!!!!!!!")

            
           
            self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
            publishing_data = AckermannDriveStamped()
            publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            publishing_data.header.frame_id = "base_link"
            publishing_data.drive.steering_angle = self.error_lane * 0.003 # error 정도에 따라서 조향 
            publishing_data.drive.speed = 0.7
            self.drive_pub.publish(publishing_data) #  하는 부분 
    # rospy.loginfo("Publish Control Data!")
        
        
    def stop(self): 
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = 0.0
        publishing_data.drive.speed = 0.0
        self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
        rospy.loginfo("Stop Vehicle!")
        self.stop_cnt += 1
        print("STOP_CNT{}".format(self.stop_cnt))
        


    ####################################################################################
    # protect child
    def child_sign_callback(self, _data):
        try :
            rospy.loginfo(" ===============    DATA :  {}".format(_data.data))
            if _data.data == 3:
                self.slow_down_flag = 1
        except :
            pass
        
    ####################################################################################
    # dynamic obstacle

    ####################################################################################
    # static obstacle
    def change_line_left(self) :
        
        rospy.loginfo("change_left!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = 0.3
        publishing_data.drive.steering_angle = 5
        self.drive_pub.publish(publishing_data)

    def change_line_right(self) :
        
        rospy.loginfo("change_right!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        #left_time = time()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = 0.3
        publishing_data.drive.steering_angle = -5
        self.drive_pub.publish(publishing_data)
        
    ####################################################################################
    # Labacon Mission
    # def rabacon_drive(self) :
    #     rospy.loginfo("rabacon_drive!!!!!!!!!!!!!!!!!")
    #     publishing_data = AckermannDriveStamped()
    #     publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
    #     publishing_data.header.frame_id = "base_link"
    #     publishing_data.drive.speed = 0.2
    #     publishing_data.drive.steering_angle = self.rabacon_mission_flag
    #     self.drive_pub.publish(publishing_data)
        
        
    #     sleep(1)

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