#!/usr/bin/env python

import cv2
import numpy as np
#import pyrealsense2 as rs
import os

#pipe = rs.pipeline()
#cfg = rs.config()
#cfg.enable_stream(rs.stream.color)
#
#pipe.start(cfg)
cap = cv2.VideoCapture(0)
i = 0
j = 0
picture_path = './img/calib'
#os.chdir('/home/pi/catkin_rover/src/rr_control_input_manager/scripts/Camera_Calibration/img')
#os.chdir('~/Desktop/img')
while(True):    
    #frames = pipe.wait_for_frames()
    ret, frame = cap.read()
    #c_left = frames.get_color_frame()
    #c_left_data = np.array(c_left.get_data())
    
    #img_grey = cv2.cvtColor(c_left_data, cv2.COLOR_BGR2GRAY) 
    img_grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('depth', img_grey)

    if i == 20:                 
        filename = picture_path + str(j) + '.png'
	print(filename)        
	cv2.imwrite(filename, img_grey)
        j += 1
        i = 0
    i += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
#pipe.stop()
cap.release()    
cv2.destroyAllWindows()

