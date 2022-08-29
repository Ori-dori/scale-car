#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
import time
# from collections import deque
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32
# from obstacle_detector.msg import Obstacles

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        # rospy.Subscriber("clustering", Obstacles, self.clustering_callback)
        #self.warning_pub = rospy.Publisher("lidar_warning", String, queue_size=5) # ROI ?? ?? ?? cnt? ??? ?? ???? ??? ? publish ?? topic 
        self.warning_pub = rospy.Publisher("lidar_warning", Int32, queue_size=5)
        self.car_cnt = 0
    
    # def clustering_callback(self, _data) :
    #     rospy.loginfo("Clustering####")


    def lidar_callback(self, _data):

        # max distance = 1.8m
        # ROI ?? ?(degree??) : ??? 180??

        # for static obs
        MIN_ANGLE = 170 # degree 
        MAX_ANGLE = 190 # degree
        WARNING_MAX_RANGE = 1.3 # meter
        WARNING_MIN_RANGE = 0.3
        WARNING_CNT = 5 # 5? ???? ??? ????? ??, 1?? ?? ??? ?? : noise? ?? ??? ??? ?? ??

        # for dynamic obs
        MIN_ANGLE_2 = 160
        MAX_ANGLE_2 = 200
        WARNING_RANGE_2 = 0.2 # meter
        WARNING_CNT_2 = 3 


        theta_deg = []
        for i in range(len(_data.ranges)):
            tmp_theta = (_data.angle_min + i * _data.angle_increment) 
            if tmp_theta >= _data.angle_max:
                tmp_theta = _data.angle_max
            theta_deg.append(tmp_theta * 180 / math.pi)
        # rospy.loginfo(theta_deg)




        point_cnt = 0 # for static obs
        point_cnt_2 = 0 # for dynamic obs
        min_distance = 10000000


        for i, distance in enumerate(_data.ranges):
            if MIN_ANGLE <= theta_deg[i] <= MAX_ANGLE:
                if WARNING_MIN_RANGE < distance and distance < WARNING_MAX_RANGE:
                    point_cnt += 1
        if point_cnt > 3 :
            self.warning_pub.publish(point_cnt)
        #rospy.loginfo("point-cnt: {}".format(point_cnt))
        #if point_cnt >= WARNING_CNT:
            #self.warning_pub.publish("STOPPED_CAR")
            #rospy.loginfo("STOPPED CAR DETECTION")
        #else:
            #self.warning_pub.publish("safe")
            #rospy.loginfo("safe")

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
