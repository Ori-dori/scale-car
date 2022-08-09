#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy


from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image 
from fiducial_msgs.msg import Fiducial, FiducialArray

class Sign():
    def __init__(self):
        
        rospy.Subscriber("/fiducial_vertices", Fiducial, self.child_sign_callback)
        self.sign_id_pub = rospy.Publisher("sign_id", Int32, queue_size=1)
    

    def child_sign_callback(self, _data):
        self.sign_id = _data.fiducial_id
        rospy.loginfo("################## ID :self.sign_id", self.sign_id)
        self.sign_id_pub.publish(sign_id)




def run():
    rospy.init_node("sign_id")
    new_class = Sign()
    rospy.spin()


if __name__ == '__main__':
    run()