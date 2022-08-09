import rospy
import math
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from race.msg import drive_values
from math import atan2, degrees
from vision_distance.msg import Colorcone_lidar, ColorconeArray_lidar 


class ClusterLidar :

    def __init__(self) :
        self.left_rabacon = []
        self.right_rabacon = []
        self.angle = 0
        self.is_rabacon_mission = False

    # select close rabacon
    def sel_rabacon(self, con, obstacle, seg)  :
        left_rabacons, right_rabacons = self.sel_close_rabacon(seg.circles)
        drive_angle = self.cal_angle(left_rabacons, right_rabacons)
        if seg.circles >= 6 :
            self.is_rabacon_mission = True
        else :
            self.is_rabacon_mission = False
        return self.is_rabacon_mission, drive_angle

    def cal_angle(self, left_rabacons, right_rabacons) :
        left_angle = math.atan(left_rabacons[0].x/left_rabacons[0].y)
        right_angle =  math.atan(right_rabacons[0].x/right_rabacons[0].y)
        return left_angle - right_angle
        # left_angle = math.atan(left_angle[0].x/)
        

    def sel_close_rabacon(self, rabacons) :
        left_rabacon = []
        right_rabacon = []
        for circle in rabacons :
            if circle.y > 0 and circle.x < 0 :
                left_rabacon.append(circle)
            elif circle.y > 0 and circle.x >0 :
                right_rabacon.append(circle)
        left_rabacon = sorted(left_rabacon, key = lambda x : x.y)[:2]
        right_rabacon = sorted(right_rabacon, key = lambda x : x.y)[:2]

        return left_rabacon, right_rabacon


