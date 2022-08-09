#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math

# from collections import deque
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
# from obstacle_detector.msg import Obstacles

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        # rospy.Subscriber("clustering", Obstacles, self.clustering_callback)
        self.warning_pub = rospy.Publisher("lidar_warning", String, queue_size=5) # ROI 내에 위험 물체 cnt가 정해진 개수 이상으로 쌓였을 때 publish 하는 topic 
        self.car_cnt = 0
    
    # def clustering_callback(self, _data) :
    #     rospy.loginfo("Clustering####")


    def lidar_callback(self, _data):

        # max distance = 1.8m
        # ROI 기준 값(degree기준) : 정면이 180도임
        MIN_ANGLE = 160 # degree 
        MAX_ANGLE = 210 # degree
        WARNING_RANGE = 1.0 # meter
        WARNING_CNT = 5 # 5개 이상으로 잡히면 위험하다고 판단, 1개로 하면 안되는 이유 : noise로 인해 실수로 멈추는 경우 방지

        theta_deg = []
        for i in range(len(_data.ranges)):
            tmp_theta = (_data.angle_min + i * _data.angle_increment) 
            if tmp_theta >= _data.angle_max:
                tmp_theta = _data.angle_max
            theta_deg.append(tmp_theta * 180 / math.pi)
        # rospy.loginfo(theta_deg)




        point_cnt = 0
        min_distance = 10000000
        object = False

        # ranges를 돌면서 ROI 내에 object 탐지되면 cnt++
        for i, distance in enumerate(_data.ranges):
            if MIN_ANGLE <= theta_deg[i] <= MAX_ANGLE:
                if distance < WARNING_RANGE:
                    point_cnt += 1
                if distance < min_distance :
                    min_distance = distance
                # if distance <= 1.6 :
            #if 175 <= theta_deg[1] <= 185:
            #    object = True

                    
        # rospy.loginfo("range_man : {}".format(min_distance))


        #if object:
        #    self.warning_pub.publish("CAR_DETECT")
        #    rospy.loginfo("Object")

        if point_cnt >= WARNING_CNT:
            self.warning_pub.publish("STOPPED_CAR")
            rospy.loginfo("STOPPED CARDETECT!!")
            # rospy.loginfo("Warning!!")
        else:
            self.warning_pub.publish("safe")
            rospy.loginfo("Safe!!")

def run():
    rospy.init_node("lidar_example")
    new_class = LidarReceiver()
    rospy.spin()


if __name__ == '__main__':
    run()