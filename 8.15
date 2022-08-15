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
        
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.bridge = CvBridge()
        self.rabacon_mission = ClusterLidar()

        self.callback_flag = False

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

        self.is_obs_static =0

        rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.lane_callback) 
        # rospy.Subscriber("obstacle_mission", String, self.warning_callback)
        #rospy.Subscriber("lidar_warning", String, self.warning_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)
        
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)
        rospy.Subscriber("lidar_warning", Int32, self.warning_callback)
        # rospy.Subscriber("rabacon_drive", Float32, self.rabacon_callback)
        # rospy.Subscriber("obstacles", Obstacles, self.rabacon_callback)
     

    def warning_callback(self, _data):
        rospy.loginfo("point_cnt: {}".format(_data.data))
        if self.rabacon_mission_flag == True :
            self.rabacon_mission_flag = True
        elif _data.data == "safe":
            self.is_safe = True
        elif _data.data == "STOPPED_CAR":
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
        rospy.logwarn("flag:{}".format(self.callback_flag))
        if self.detect_car == "OBSTACLE DETECTION" :
            rospy.loginfo("obstacle detection")
            
            
            self.last_cnt = self.detect_cnt
            rospy.sleep(2)
            # one_time_msg = rospy.wait_for_message("lidar_warning", Int32, timeout=None)
            # rospy.loginfo("&&&&&&& one msg : {}".format(one_time_msg))
            rospy.loginfo("&&&&&&& last_cnt : {}".format(self.last_cnt))
            rospy.loginfo("&&&&&&& detect_cnt : {}".format(self.detect_cnt))

            print("CURRENT_LANE: {}".format(self.current_lane))

            if self.last_cnt-3 <= self.detect_cnt <= self.last_cnt+3:        
                rospy.logwarn("static obstacle")
                rospy.loginfo("static obstacle")
                self.last_cnt = 0
                if self.current_lane == "LEFT":
                    for i in range(600) :
                        print("***************{}****************".format(i))
                        self.change_line_right()
                    if self.current_lane_window == "MID" :
                        for i in range(250) :
                            self.change_line_right()
                   
                    self.current_lane = "RIGHT"
                    self.detect_car == "DEFAULT"
                    #elf.follow_lane()
                    
                elif self.current_lane == "RIGHT":
                    for i in range(600) :
                        self.change_line_left()
                    if self.current_lane_window == "MID" :
                        for i in range(250) :
                            self.change_line_left()
                    
                    self.current_lane = "LEFT"
                    self.detect_car == "DEFAULT"
                    #self.follow_lane()
                # self.callback_flag = False
                self.follow_lane()


                              
            else:
                self.last_cnt = 0
                rospy.logwarn("dynamic obstacle")
                rospy.loginfo("dynamic obstacle")
                #self.callback_flag = False
                self.follow_lane()
        else:
            rospy.logwarn("obs")
            # self.callback_flag = False
            self.follow_lane()
                 
                


    def rabacon_callback(self, _data) :
        self.rabacon_mission_flag = _data.data
        if self.rabacon_mission_flag < 10.0 :
            self.rabacon_drive()

    def lane_callback(self, _data):        
        cv_image = self.bridge.imgmsg_to_cv2(_data, "bgr8")

        if self.initialized == False:
            cv2.namedWindow("lane_image", cv2.WINDOW_NORMAL)
            cv2.createTrackbar('gray', 'lane_image', 0, 255, nothing)
            self.initialized = True

        gray = cv2.getTrackbarPos('gray', 'lane_image')

        lane_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        lane_image2 = cv2.inRange(lane_image, gray, 255)

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
        rospy.loginfo("CURRENT LANE WINDOW: {}".format(self.current_lane_window))
  
  


    def follow_lane(self): # 차선데이터를 받아서 차선을 따라서 주행하도록 하는 함수

       
        # rospy.loginfo("CURRENT_LANE : {}".format(self.current_lane))
        # self.child_sign_callback()
        # if self.warning: # object 탐지 시 멈추도록  --> 이런 로직 이용해서 정적 장애물 회피 로직으로 수정하면 될 듯 
        #     self.stop()

        # elif self.rabacon_mission_flag != 1000.0 :
        #     rospy.loginfo("************* RABACON *****************")
        #     self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
        #     publishing_data = AckermannDriveStamped()
        #     publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        #     publishing_data.header.frame_id = "base_link"
        #     publishing_data.drive.steering_angle = self.rabacon_mission_flag # error 정도에 따라서 조향 
        #     publishing_data.drive.speed = 0.2
        #     self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분
        
        # elif self.detect_dyn_obs == "OBS_DETECT":

        #     while (not self.is_safe):
        #         rospy.loginfo("************* STOP (OBS DETECTED) *****************")
        #         self.stop()

        #rospy.logwarn("?????")

                    
                        
                # cnt_list = []
            # cnt_set = set()
            # for i in range(10):rospy.Subscriber("lidar_warning", Int32, self.warning_callback)
            #     print("cnt_list:", cnt_list)
            #     print("cnt_set:", cnt_set)
            #     print("cnt:", self.detect_cnt)
            #     cnt_list.append(self.detect_cnt)
            #     cnt_set.add(self.detect_cnt)
            # rospy.loginfo("cnt_set:", cnt_set)
            # print("cnt_set:", cnt_set)
            # if len(cnt_set) < 3:
            #     rospy.loginfo("static obstacle")
            # else:
            #     rospy.loginfo("dynamic obstacle")
            # cnt_set.clear()
            #self.follow_lane()


        #if self.detect_car == "CAR_DETECT" and self.current_lane == "RIGHT":

        #     for i in range(1000):


        #     self.callback_flag = True
        #     #self.lane_change = True
        #     for i in range(500) :
        #         self.change_line_left()
        #     if self.current_lane_window == "MID" :
        #         for i in range(250) :
        #             self.change_line_left()
        

            # mid_cnt = 0
            # if self.current_lane_window == "MID" :
            #     mid_cnt += 1
            # else :
            #     mid_cnt = 0  
            # if mid_cnt >= 5 :
            #     while self.current_lane_window != "MID" :
            #         self.change_line_left()
            #     # if self.current_lane_window != "MID" :
            #     while self.current_lane_window == "MID" :
            #         self.change_line_left()
            # self.current_lane = "LEFT"
            # self.detect_car == "DEFAULT"
            # self.follow_lane()
            # self.callback_flag = False

            
        # elif self.detect_car == "CAR_DETECT" and self.current_lane =="LEFT" and self.is_obs_static == 1:
        #     self.callback_flag = True
        #     for i in range(500) :
        #         self.change_line_right()
        #     mid_cnt = 0
        #     if self.current_lane_window == "MID" :
        #         print("moremoremoremoremoremoremormeroemromaeo")
        #         for i in range(250) :
        #             self.change_line_right()
            # if self.current_lane_window == "MID" :
            #     mid_cnt += 1

            # if mid_cnt >= 3 :
            #     while self.current_lane_window != "MID" :
            #         print("195@@@@@@@")
            #         self.change_line_right()
            #     while self.current_lane_window == "MID" :
            #         self.change_line_right()
            #         print("@@@@@@@@@@@@@")

            # self.current_lane = "RIGHT"
            # self.detect_car = "DEFAULT"
            # self.follow_lane()
            # self.callback_flag = False
            # rospy.loginfo("LiDAR object is detected!!")


        if self.detect_car == "OBSTACLE DETECTION" and self.callback_flag == False :
            rospy.loginfo("#################################################")    
            self.callback_flag = True   
            self.stop()
            #self.callback_flag = False
            
            
                
        # slow_down_sign 
        elif self.slow_down_flag == 1:
            rospy.loginfo("************* SLOW DOWN *****************")
            self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
            publishing_data = AckermannDriveStamped()
            publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            publishing_data.header.frame_id = "base_link"
            publishing_data.drive.steering_angle = self.error_lane * 0.005 # error 정도에 따라서 조향 
            publishing_data.drive.speed = 0.2
            self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분
        
        else:
            rospy.loginfo("DEFAULT DRIVE!!!!!!!!!")
            rospy.loginfo("callbackflag : {}, {}".format(self.callback_flag, self.detect_car))
            self.callback_flag = False
            
            self.error_lane = 280 - self.slide_x_location # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
            # error가 음수 --> 오른쪽 차선이랑 멈 --> 오른쪽으로 조향(sttering_angle < 0)
            # error가 양수 --> 오른쪽 차선이랑 가까움 --> 왼쪽으로 조향(sttering_angle > 0)

            # rospy.loginfo("slide_x_location = {}".format(self.slide_x_location))        
            # rospy.loginfo("error lane = {}".format(self.error_lane))
            publishing_data = AckermannDriveStamped()
            publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            publishing_data.header.frame_id = "base_link"
            publishing_data.drive.steering_angle = self.error_lane * 0.005 # error 정도에 따라서 조향 
            publishing_data.drive.speed = 0.5
            self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
        # rospy.loginfo("Publish Control Data!")
        
        

    def stop(self): 
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = 0.0
        publishing_data.drive.speed = 0.0
        self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
        rospy.loginfo("Stop Vehicle!")
        rospy.sleep(1)
        self.obs_condition()
        rospy.loginfo("move please")
        rospy.sleep(1)




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
    
    def change_line_left(self) :
        
        # rospy.loginfo("change_left!!!!!!!!!!!!!!!!!")
        # publishing_data = AckermannDriveStamped()
        # left_time = time()
        # publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        # publishing_data.header.frame_id = "base_link"
        # publishing_data.drive.speed = 0.2
        # publishing_data.drive.steering_angle = 3
        # self.drive_pub.publish(publishing_data)

        
        rospy.loginfo("change_left!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = 0.3
        publishing_data.drive.steering_angle = 5
        self.drive_pub.publish(publishing_data)
        #self.callback_flag = False  
        # sleep(10)
        # publishing_data.drive.steering_angle -= 0.5
        # self.drive_pub.publish(publishing_data)
        # self.detect_car = " "

        # if self.cnt_lane % 7 == 0: 
        #     publishing_data.drive.steering_angle -= 0.5

        

    def change_line_right(self) :
        
        # rospy.loginfo("change_right!!!!!!!!!!!!!!!!!")
        # publishing_data = AckermannDriveStamped()
        
        # publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        # publishing_data.header.frame_id = "base_link"
        # publishing_data.drive.speed = 0.2
        # publishing_data.drive.steering_angle = -3
        # self.drive_pub.publish(publishing_data)
        
        rospy.loginfo("change_right!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        #left_time = time()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = 0.3
        publishing_data.drive.steering_angle = -5
        self.drive_pub.publish(publishing_data)
        #self.callback_flag = False  
        # sleep(10)
        # publishing_data.drive.steering_angle -= 0.5
        # self.drive_pub.publish(publishing_data)
        # self.detect_car = " "   

        
    ####################################################################################
    # Labacon Mission
    def rabacon_drive(self) :
        rospy.loginfo("rabacon_drive!!!!!!!!!!!!!!!!!")
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.speed = 0.2
        publishing_data.drive.steering_angle = self.rabacon_mission_flag
        self.drive_pub.publish(publishing_data)
        
        
        sleep(1)

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
