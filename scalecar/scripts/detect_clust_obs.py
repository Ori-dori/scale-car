#!/usr/bin/env python
#-*- coding: utf-8 -*-


# =============== pulish 'warning or safe' and 'y' to 'main'====================

import rospy
import math
import time
# from collections import deque
# from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32, Float32
from obstacle_detector.msg import Obstacles

class LidarReceiver():
    def __init__(self):
        # rospy.loginfo("LiDAR Receiver Object is Created")
        # rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/raw_obstacles", Obstacles, self.lidar_callback)
        # rospy.Subscriber("clustering", Obstacles, self.clustering_callback)
        self.warning_pub = rospy.Publisher("lidar_warning", String, queue_size=5)
        self.object_pub = rospy.Publisher("object_condition", Float32, queue_size=5)

        self.count_flag = 0
        self.flag_flag = 0
        self.count_t1 = 0
        self.x1 = 0
        self.x2 = 0

    def lidar_callback(self, _data):
        #rospy.logwarn("lidar callback")
        #rospy.loginfo("x:{}".format(_data.circles[0].center.x))

        # ROI
        left_y = -0.2
        right_y = 0.2
        front_x = -1.6
        back_x = 0
        WARNING_CNT = 1

        self.point_cnt = 0
        self.dynamic_cnt = 0


        # 동적장애물 왼쪽 기둥 인식 code
        for i in _data.circles :
            # if 0.15 < i.center.y < 0.65 and -1 < i.center.x < 0 :
            #     self.dynamic_cnt += 1
            #     if self.dynamic_cnt >= 1 :
            #         self.object_pub.publish(77.0)
                    

            if left_y < i.center.y < right_y and front_x < i.center.x < back_x:
                rospy.loginfo("point_cnt:{}, Y:{}, X:{}".format(self.point_cnt, i.center.y, i.center.x))
                self.object_pub.publish(i.center.y)
                
            if left_y < i.center.y < right_y and -1.3 < i.center.x < 0:
                self.point_cnt += 1            # if self.flag_flag == 0 :







            #     #self.flag_flag = 1
            #     if self.count_flag == 0:
            #         self.count_t1 = rospy.get_time()
            #         self.x1 = _data.circles[0].center.x
            #         #rospy.loginfo("x1:{}".format(self.x1))
            #         self.count_flag = 1
            #     t2 = rospy.get_time()

            #     # stop for a few seconds
            #     while t2-self.count_t1 <= 3:
            #         #rospy.loginfo("************* CHECKING *****************")
            #         # rospy.loginfo("count time{}, {}".format(self.slow_t1,t2))
            #         t2 = rospy.get_time()
            #     self.count_flag = 0
            #     self.x2 = _data.circles[0].center.x
            #     self.flag_flag = 1
                
            #     #rospy.loginfo("x1:{}, x2:{}".format(self.x1, self.x2))
            #     if self.x1-self.x2 <= 0.01:
            #         #rospy.loginfo("publish STATIC")
            #         self.object_pub.publish("STATIC")
            #         self.flag_flag = 2
            #     else:
            #         #rospy.loginfo("publish DYNAMIC")            # if self.flag_flag == 0 :
            #     #self.flag_flag = 1
            #     if self.count_flag == 0:
            #         self.count_t1 = rospy.get_time()
            #         self.x1 = _data.circles[0].center.x
            #         #rospy.loginfo("x1:{}".format(self.x1))
            #         self.count_flag = 1
            #     t2 = rospy.get_time()

            #     # stop for a few seconds
            #     while t2-self.count_t1 <= 3:
            #         #rospy.loginfo("************* CHECKING *****************")
            #         # rospy.loginfo("count time{}, {}".format(self.slow_t1,t2))
            #         t2 = rospy.get_time()
            #     self.count_flag = 0
            #     self.x2 = _data.circles[0].center.x
            #     self.flag_flag = 1
                
            #     #rospy.loginfo("x1:{}, x2:{}".format(self.x1, self.x2))
            #     if self.x1-self.x2 <= 0.01:
            #         #rospy.loginfo("publish STATIC")
            #         self.object_pub.publish("STATIC")
            #         self.flag_flag = 2
            #     else:
            #         #rospy.loginfo("publish DYNAMIC")
            #         self.object_pub.publish("DYNAMIC")
            #         self.flag_flag = 2
            #         self.object_pub.publish("DYNAMIC")
            #         self.flag_flag = 2
            #rospy.loginfo("-----------------------------")
                    
            if self.point_cnt >= WARNING_CNT:
                self.warning_pub.publish("WARNING")
                
            # if self.flag_flag == 0 :
            #     #self.flag_flag = 1
            #     if self.count_flag == 0:
            #         self.count_t1 = rospy.get_time()
            #         self.x1 = _data.circles[0].center.x
            #         #rospy.loginfo("x1:{}".format(self.x1))
            #         self.count_flag = 1
            #     t2 = rospy.get_time()

            #     # stop for a few seconds
            #     while t2-self.count_t1 <= 3:
            #         #rospy.loginfo("************* CHECKING *****************")
            #         # rospy.loginfo("count time{}, {}".format(self.slow_t1,t2))
            #         t2 = rospy.get_time()
            #     self.count_flag = 0
            #     self.x2 = _data.circles[0].center.x
            #     self.flag_flag = 1
                
            #     #rospy.loginfo("x1:{}, x2:{}".format(self.x1, self.x2))
            #     if self.x1-self.x2 <= 0.01:
            #         #rospy.loginfo("publish STATIC")
            #         self.object_pub.publish("STATIC")
            #         self.flag_flag = 2
            #     else:
            #         #rospy.loginfo("publish DYNAMIC")
            #         self.object_pub.publish("DYNAMIC")
            #         self.flag_flag = 2

                #rospy.loginfo("OBJECT is DETECTED")
            else:
                self.warning_pub.publish("safe")

                #rospy.loginfo("safe")
                self.point_cnt = 0

        # for static obs
        # ranges? ??? ROI ?? object ???? cnt++
        #for i, distance in enumerate(_data.ranges):
#             if MIN_ANGLE <= theta_deg[i] <= MAX_ANGLE:
#                 if WARNING_MIN_RANGE < distance < WARNING_MAX_RANGE:
#                     point_cnt += 1
#                 if distance < min_distance :
#                     min_distance = distance
#                 # if distance <= 1.6 :
#             #if 175 <= theta_deg[1] <= 185:
#             #    object = True

#         # for dynamic obs
#         for i, distance in enumerate(_data.ranges):
#             if MIN_ANGLE_2 <= theta_deg[i] <= MAX_ANGLE_2:
#                 if distance <= WARNING_RANGE_2:
#                     point_cnt_2 += 1
#                 if distance < min_distance :
#                     min_distance = distance

                    
#         # rospy.loginfo("range_man : {}".format(min_distance))


       
#         if point_cnt >= WARNING_CNT:
#             self.warning_pub.publish("STOPPED_CAR")
#             rospy.loginfo("STOPPED CARDETECT!!")
#             # rospy.loginfo("Warning!!")
#         elif point_cnt_2 >= WARNING_CNT_2:
#             self.warning_pub.publish("DYNAMIC_OBS")
#             rospy.loginfo("DYNAMIC OBS DETECTED!!")
#         else:
#             self.warning_pub.publish("safe")
#             rospy.loginfo("Safe!!")
# # 
def run():
    rospy.init_node("lidar_example")
    new_class = LidarReceiver()
    rospy.spin()


if __name__ == '__main__':
    run()
