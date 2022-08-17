#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from time import sleep
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, Float32

# from std_msgs.msg import String
# from obstacle_detector.msg import Obstacles
# from obstacle_detector.msg import SegmentObstacle
# from geometry_msgs.msg import Point
# from race.msg import drive_values




class ClusterLidar :

    def __init__(self) :
        rospy.Subscriber("/raw_obstacles", Obstacles, self.rabacon)
        self.rabacon_pub = rospy.Publisher("rabacon_drive", Float32, queue_size = 5)
        self.angle = 0.0 

    # select close rabacon
    def rabacon(self, _data)  :
        plus_x_list = []
        for i in _data.circles :
            if -1 < i.center.x < 0 and abs(i.center.y) < 1:
                plus_x_list.append(i)
        #print(plus_x_list)
        if len(plus_x_list) >= 4 :
            # left_rabacons, right_rabacons = self.sel_close_rabacon(_data.circles)
            raba = self.sel_close_rabacon(plus_x_list)
            # self.drive_angle = self.cal_angle(left_rabacons, right_rabacons)
            self.rabacon_pub.publish(raba)
        else :
            self.rabacon_pub.publish(1000.0)

    def cal_angle(self, left_rabacons, right_rabacons) :
        left_angle = abs(math.atan(left_rabacons[0].center.y/left_rabacons[0].center.x))
        right_angle =  abs(math.atan(right_rabacons[0].center.y/right_rabacons[0].center.x))
        rospy.loginfo("left angle = {} right angle = {}".format(left_angle, right_angle))
        return (left_angle - right_angle)*0.1


    def sel_close_rabacon(self, rabacons) :
        left_rabacon = []
        right_rabacon = []

        raba_sum = 0
        raba_cnt = 0
        raba = 0
        for r in rabacons :
            if r.center.y < 0 :
                left_rabacon.append(r)
            else :
                right_rabacon.append(r)
        left_rabacon = sorted(left_rabacon, key = lambda x : x.center.x)
        right_rabacon = sorted(right_rabacon, key = lambda x : x.center.x)
        if len(left_rabacon) == len(right_rabacon) :
            for circle in rabacons :
                raba_sum += circle.center.y
            raba = raba_sum / (len(left_rabacon) + len(right_rabacon))
        elif len(left_rabacon) > len(right_rabacon) :
            for i in range(len(right_rabacon)) :
                raba_sum += left_rabacon[i].center.y
                raba_sum += right_rabacon[i].center.y
            raba = raba_sum /(len(right_rabacon)*2)
        else :
            for i in range(len(left_rabacon)) :
                raba_sum += left_rabacon[i].center.y
                raba_sum += right_rabacon[i].center.y
            raba = raba_sum /(len(left_rabacon)*2)
        print("raba", raba)
        print(rabacons)
        print("RABACON LENGTH", len(rabacons))
        # print("0:" ,rabacons[0].center.x)
        # print("1:" ,rabacons[1].center.x)
        # print("2:" ,rabacons[2].center.x)o

        # for circle in rabacons :
        #     if 0 > circle.center.x > -1.0:
        #         raba_sum += circle.center.y
        #         raba_cnt += 1
        # if raba_cnt:
        #     raba = raba_sum / raba_cnt
        # else:
        #     raba = 0

        #     if circle.center.x > 0 and circle.center.y > 0 :
        #         left_rabacon.append(circle)
        #     elif circle.center.x > 0 and circle.center.y < 0 :
        #         right_rabacon.append(circle)
        # left_rabacon = sorted(left_rabacon, key = lambda x : x.center.x)[:3]
        # right_rabacon = sorted(right_rabacon, key = lambda x : x.center.x)[:3]

        # return left_rabacon, right_rabacon
        return raba

def run() :
    rospy.init_node("rabacon_drive")
    cluster = ClusterLidar()
    rospy.spin()

if __name__=='__main__' :
    run()