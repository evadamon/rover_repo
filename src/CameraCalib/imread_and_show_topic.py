#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()
i = 0
j = 0
picture_path = './img/calib'

def image_callback(msg):
    global i
    global j
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
	if i == 1:                 
	    filename = picture_path + str(j) + '.png'
	    print(filename)        
	    cv2.imwrite(filename, cv2_img)
	    j += 1
	    i = 0
	i += 1
    show_image(cv2_img)

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)
	
def main():
    rospy.init_node('image_listener')
    image_topic = "/camera/rgb/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    cv2.namedWindow("Image Window", 1)
    rospy.spin()

if __name__ == '__main__':
    main()
