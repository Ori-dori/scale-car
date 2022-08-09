import rospy
import math
# from std_msgs.msg import String
# from obstacle_detector.msg import Obstacles
# from obstacle_detector.msg import SegmentObstacle
# from geometry_msgs.msg import Point
# from race.msg import drive_values

class ClusterLidar :

    def __init__(self) :
        self.angle = 0 
        self.is_rabacon_mission = False

    # select close rabacon
    def rabacon(self, seg)  :
        left_rabacons, right_rabacons = self.sel_close_rabacon(seg.circles)
        self.drive_angle = self.cal_angle(left_rabacons, right_rabacons)
        if seg.circles >= 6 :
            self.is_rabacon_mission = True
        else :
            self.is_rabacon_mission = False
        return self.is_rabacon_mission, self.drive_angle * 0.017

    def cal_angle(self, left_rabacons, right_rabacons) :
        left_angle = math.atan(left_rabacons[0].y/left_rabacons[0].x)
        right_angle =  math.atan(right_rabacons[0].y/right_rabacons[0].x)


        return left_angle - right_angle
        # left_angle = math.atan(left_angle[0].x/)
        

    def sel_close_rabacon(self, rabacons) :
        left_rabacon = []
        right_rabacon = []
        for circle in rabacons :
            if circle.x > 0 and circle.y > 0 :
                left_rabacon.append(circle)
            elif circle.x > 0 and circle.y < 0 :
                right_rabacon.append(circle)
        left_rabacon = sorted(left_rabacon, key = lambda x : x.x)[:2]
        right_rabacon = sorted(right_rabacon, key = lambda x : x.x)[:2]

        return left_rabacon, right_rabacon