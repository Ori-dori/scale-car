#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        self.warning_pub = rospy.Publisher("lidar_warning", String, queue_size=5) # ROI 내에 위험 물체 cnt가 정해진 개수 이상으로 쌓였을 때 publish 하는 topic 

    def lidar_callback(self, _data):
        # 입력된 LiDAR 데이터 중, 차량 정면에 해당하는 부분의 데이터가 일정 거리 이하일 경우, 위험하다는 메시지를 전달
        # 확인해야할 내용 : 차량 정면에 해당하는 부분이 어디까지인지를 정해야함(-10~ 10도? or y 기준 -1.0~1.0?)
        # 일정 거리를 몇으로 할지? 얼마나 가까우면 위험하다고 판단?, 하나만 들어와도 위험하다고 할지, 아니면 여러개? 

        # ROI 기준 값(degree기준) : 정면이 180도임
        MIN_ANGLE = 170 # degree 
        MAX_ANGLE = 190 # degree
        WARNING_RANGE = 0.8 # meter
        WARNING_CNT = 5 # 5개 이상으로 잡히면 위험하다고 판단, 1개로 하면 안되는 이유 : noise로 인해 실수로 멈추는 경우 방지

        theta_deg = []
        for i in range(len(_data.ranges)):
            tmp_theta = (_data.angle_min + i * _data.angle_increment) 
            if tmp_theta >= _data.angle_max:
                tmp_theta = _data.angle_max
            theta_deg.append(tmp_theta * 180 / math.pi)
        # rospy.loginfo(theta_deg)

        point_cnt = 0
        # ranges를 돌면서 ROI 내에 object 탐지되면 cnt++
        for i, distance in enumerate(_data.ranges):
            if MIN_ANGLE <= theta_deg[i] <= MAX_ANGLE:
                if distance < WARNING_RANGE:
                    point_cnt += 1
            
        if point_cnt >= WARNING_CNT:
            self.warning_pub.publish("warning")
            rospy.loginfo("Warning!!")
        else:
            self.warning_pub.publish("safe")
            rospy.loginfo("Safe!!")

def run():
    rospy.init_node("lidar_example")
    new_class = LidarReceiver()
    rospy.spin()


if __name__ == '__main__':
    run()