#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy

# Pulisher 1개가 필요
# Topic name --> /high_level/ackermann_cmd_mux/input/nav_0
# Message Type --> ackermann_msgs/AckermannDriveStamped
# Steering_angle --> 입력 가능한 범위가 -0.34 ~ +0.34 까지 입력 가능
# speed --> 실차량 기준은 -10m/s ~ 10m/s (권장 사항은 -2.5~ 2.5m/s 만 사용하는 것을 권장)

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32

class Controller():
    def __init__(self):
        self.right_lane_x = 0  # 실제 검출된 차선과 카메라 가운데 사이의 거리 0으로 초기화
        self.ref_right_lane_x = 255 # 차선을 어느 위치에 놓고 주행할 지  -> x 값이 ref_x 값에 맞춰지도록 주행(reference)

        rospy.Subscriber("right_lane_moment_x", Int32, self.lane_callback) # 오른쪽 차선 momentum x 값 subscribe
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.timer_callback) # 1초에 5번 처리 -> =: 아래 주석된 내용
        
        # self.rate = rospy.Rate(5) # 데이터 publish하는 주기를 정하는 함수(1초에 5번)
        # while not rospy.is_shutdown():
        #     self.publish_data()
        #     self.rate.sleep() # 1초에 5번 보내고, 남은 시간동안에는 쉼
    
    def lane_callback(self, _data): 
        self.right_lane_x = _data.data 

    def timer_callback(self, _event):
        if self.right_lane_x == 0:
            self.stop() 
            return
        
        self.follow_lane()

        # default 주행
        publishing_data = AckermannDriveStamped()
        publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
        publishing_data.header.frame_id = "base_link"
        publishing_data.drive.steering_angle = -0.34
        publishing_data.drive.speed = 1.0 
        self.drive_pub.publish(publishing_data) # 실제 publish 하는 부분 
        rospy.loginfo("Publish Control Data!")
  

    def follow_lane(self): # 차선데이터를 받아서 차선을 따라서 주행하도록 하는 함수
            self.error_lane = self.ref_right_lane_x - self.right_lane_x # error가 음수 --> 오른쪽 차선이랑 멈 / error가 양수 --> 오른쪽 차선이랑 가까움
            rospy.loginfo("error lane = {}".format(self.error_lane))
            publishing_data = AckermannDriveStamped()
            publishing_data.header.stamp = rospy.Time.now() # 이 데이터를 보낼 때의 시점
            publishing_data.header.frame_id = "base_link"
            publishing_data.drive.steering_angle = -0.34
            publishing_data.drive.speed = 2.0
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



def run():
    rospy.init_node("control_example")
    Controller()
    rospy.spin()

if __name__ == "__main__":
    run()