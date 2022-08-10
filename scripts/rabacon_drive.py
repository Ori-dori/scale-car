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
            if i.center.x > 0 :
                plus_x_list.append(i)
        #print(plus_x_list)
        if len(plus_x_list) >= 6 :
            left_rabacons, right_rabacons = self.sel_close_rabacon(_data.circles)
            self.drive_angle = self.cal_angle(left_rabacons, right_rabacons)
            self.rabacon_pub.publish(self.drive_angle)
        else :
            self.rabacon_pub.publish(1000.0)

    def cal_angle(self, left_rabacons, right_rabacons) :
        left_angle = abs(math.atan(left_rabacons[0].center.y/left_rabacons[0].center.x))
        right_angle =  abs(math.atan(right_rabacons[0].center.y/right_rabacons[0].center.x))
        return (left_angle - right_angle)*0.0174


    def sel_close_rabacon(self, rabacons) :
        left_rabacon = []
        right_rabacon = []
        for circle in rabacons :
            if circle.center.x > 0 and circle.center.y > 0 :
                left_rabacon.append(circle)
            elif circle.center.x > 0 and circle.center.y < 0 :
                right_rabacon.append(circle)
        left_rabacon = sorted(left_rabacon, key = lambda x : x.center.x)[:2]
        right_rabacon = sorted(right_rabacon, key = lambda x : x.center.x)[:2]

        return left_rabacon, right_rabacon

def run() :
    rospy.init_node("rabacon_drive")
    cluster = ClusterLidar()
    rospy.spin()

if __name__=='__main__' :
    run()