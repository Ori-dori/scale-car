import rospy
import math
from time import sleep
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from std_msgs.msg import String

# for static obs
MIN_ANGLE = 170 # degree 
MAX_ANGLE = 190 # degree
WARNING_MAX_RANGE = 1.0 # meter
WARNING_MIN_RANGE = 0.7
WARNING_CNT = 5 # 5? ???? ??? ????? ??, 1?? ?? ??? ?? : noise? ?? ??? ??? ?? ??

# for dynamic obs
MIN_ANGLE_2 = 160
MAX_ANGLE_2 = 200
WARNING_RANGE_2 = 0.2 # meter
WARNING_CNT_2 = 3 





class Obstacle_Mission :

    def __init__(self) :
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacle_callback)
        self.mission_pub = rospy.Publisher("obstacle_mission", String, queue_size = 5)
        self.angle = 0.0 

    def obstacle_callback(self, _data) :
        # obstacle_interested = []
        obstacle_flag = "Default"
        for i in _data.circles() :
            print("circle", i.center.x, i.center.y)
            if i.center.x < 0.3 and abs(i.center.y) <0.6 :
                obstacle_flag = "DYNAMIC_OBS"
            elif i.center.x < 1.0 and i.center.x > 0.7 and abs(i.center.y) < 0.6 and obstacle_flag =="Default":
                # obstacle_interested.append(i)
                obstacle_flag = "STOPPED_CAR"
        
        self.mission_pub.publish(obstacle_flag)
            



        

def run() :
    rospy.init_node("obstacle_mission")
    ob_mission = Obstacle_Mission()
    rospy.spin()

if __name__=='__main__' :
    run()