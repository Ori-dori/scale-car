#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy # Python에서 ROS를 활용할 수 있도록 하는 부분

# sensor_msgs/Image 타입의 usb_cam/image_raw 토픽을 받는 코드 작성 예정 
from sensor_msgs.msg import Image # sensor_msgs/Image 타입을 사용하기 위해 추가한 부분
from cv_bridge import CvBridge # Ros image -- > OpenCV Image로 변경하기 위해 필요한 부분
import cv2
import numpy as np
from warper import Warper
import matplotlib.pyplot as plt

def nothing(x):
    pass



warper = Warper()

        #rospy.loginfo("CAMERA Reciever Object is Created")
        #rospy.Subscriber("usb_cam/image_raw", Image, self.camera_callback) # Subscriber 생성자 필수 입력 3가지 --> Subscribe 하려는 Topic, 해당 Topic의 Message Type, 동작 callback 함수 
Simulator_Image = cv2.imread('/home/wego/Pictures/lane_test.png', cv2.IMREAD_COLOR)
bridge = CvBridge()
cv2.namedWindow("Simulator_Image", cv2.WINDOW_NORMAL) # 창의 크기를 조정할 수 있도록 함(이 부분이 없으면 AUTUSIZE 적용됨)
cv2.createTrackbar('low_H', 'Simulator_Image', 0, 255, nothing)
cv2.createTrackbar('low_S', 'Simulator_Image', 0, 255, nothing)
cv2.createTrackbar('low_V', 'Simulator_Image', 130, 255, nothing)
cv2.createTrackbar('high_H', 'Simulator_Image', 255, 255, nothing)
cv2.createTrackbar('high_S', 'Simulator_Image', 255, 255, nothing)
cv2.createTrackbar('high_V', 'Simulator_Image', 255, 255, nothing)
initialized = True
#rospy.loginfo(len(_data.data))
#rospy.loginfo(type(_data.data))
#print(Simulator_Image)
#print(type(Simulator_Image))
cv2_image = Simulator_Image
#rospy.loginfo(cv2_image.shape)
#rospy.loginfo(type(cv2_image))

# R = cv2.getTrackbarPos('R', 'Simulator_Image')
# G = cv2.getTrackbarPos('G', 'Simulator_Image')
# B = cv2.getTrackbarPos('B', 'Simulator_Image')

# H(Hue : 색상), S(Saturation : 채도), V(Value : 명도)
low_H = cv2.getTrackbarPos('low_H', 'Simulator_Image')
low_S = cv2.getTrackbarPos('low_S', 'Simulator_Image')
low_V = cv2.getTrackbarPos('low_V', 'Simulator_Image')
high_H = cv2.getTrackbarPos('high_H', 'Simulator_Image')
high_S = cv2.getTrackbarPos('high_S', 'Simulator_Image')
high_V = cv2.getTrackbarPos('high_V', 'Simulator_Image')

# cv2.line(cv2_image, (300, 200), (540, 400), (255, 0, 0), 5) # 그릴 이미지, 시작 픽셀, 끝 픽셀, 색깔(BGR순), 두께
# cv2.circle(cv2_image, (320, 240), 20, (0, 0, 255), -1) # 그릴 이미지, 중심 픽셀, 반지름, 색깔, 두께(-1 : 채워진 원)
# font = cv2.FONT_HERSHEY_SIMPLEX
# cv2.putText(cv2_image, 'OpenCV', (10, 200), font, 4, (255, 255, 255), 2, cv2.LINE_AA) # 그릴 이미지, 적을 텍스트, 시작 위치(좌측 상단), 글씨체, 글씨크기, 색깔, 라인그리는 방법
cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

crop_image = cv2_image.copy()[241:480, :, :] # [세로(0~480), 가로(0~640), ???], ROI 영역 추출하는 부분 -> V ROI 찾아볼 것 
# cv2.imshow("crop_image", crop_image)
# cv2.line(crop_image, (300, 200), (540, 400), (255, 0, 0), 5)



lower_lane = np.array([low_H, low_S, low_V]) # 픽셀의 hsv값이 lower와 upper 사이에 있다 -> '1' / 없다 -> '0'
upper_lane = np.array([high_H, high_S, high_V])

lane_image = cv2.inRange(crop_image, lower_lane, upper_lane) # crop + hsv 

# moment 계산 
# M = cv2.moments(lane_image)
# self.x = int(M['m10']/M['m00'])
# self.y = int(M['m01']/M['m00'])

# cv2.circle(crop_image, (self.x, self.y), 5, (0, 255, 0), -1)
# cv2.imshow("result_image", crop_image)


print(lane_image)

print(cv2_image)


cv2.imshow("lane Image", lane_image)

cv2.imshow("Simulator_Image", cv2_image)
# channel_1, channel_2, channel_3 = cv2.split(cv2_image)
# cv2.imshow("Simulator_Image, 1", channel_1) # Simulator_Image 창에다가 cv2_image를 띄워라
# cv2.imshow("Simulator_Image, 2", channel_2) # Simulator_Image 창에다가 cv2_image를 띄워라
# cv2.imshow("Simulator_Image, 3", channel_3) # Simulator_Image 창에다가 cv2_image를 띄워라


#warp
warp_image = warper.warp(lane_image)
print(warp_image)
cv2.imshow("Warp Image", warp_image)


cv2.waitKey(1) # imshow 뒤에 waitKey를 적어주어야 함 (1ms를 기다려보겠다)



# ====================================== Canny image example ==============================================
# import rospy
# import cv2
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# class img_converter: 
#     def __init__(self):
#         self.canny_pub = rospy.Publisher("Canny_img", Image, queue_size=5)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

#     def callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print("converting error")
#             print(e)

#         canny_img = cv2.Canny(cv_image, 100, 150)

#         try:
#             self.canny_pub.publish(self.bridge.cv2_to_imgmsg(canny_img, "mono8"))
#         except CvBridgeError as e:
#             print("publish error")
#             print(e)

# def run():
#     rospy.init_node('image_converter', anonymous=True)
#     ic = img_converter()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Program down")
#     cv2.destroyAllWindows()

# if __name__=="__main__":
#     run()