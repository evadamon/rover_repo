#!/usr/bin/env python3

import cv2
import numpy as np
import cv2.aruco as aruco
import math

class ArucoTracker():

    def __init__(self,marker_size,camera_matrix,camera_distortion):
        
        #The CvBridge command is only needed if the images are coming in via ROS
        #self.bridge = CvBridge()

        self.marker_size = marker_size
        self.camera_matrix = camera_matrix
        self.camera_distortion = camera_distortion

        self.R_flip = np.zeros((3,3), dtype=np.float32)
        self.R_flip[0,0]=1.0
        self.R_flip[1,1]=-1.0
        self.R_flip[2,2]=-1.0

        #The following commands establish which Aruco dictionary will be in use.
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.parameters = aruco.DetectorParameters_create()
        
        #initialize video stream
        self.vid=cv2.VideoCapture(0)
        
        #initialize an empty frame within the class
        self.frame = np.zeros([960, 1280, 3],dtype=np.uint8)
        self.gray = np.zeros([960, 1280, 3],dtype=np.uint8)

        self.font = cv2.FONT_HERSHEY_PLAIN
        
    def rotationMatrixToEulerAngles(self, R):
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6
        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        singular = sy < 1e-6
        
        if not singular:
            x = math.atan2(R[2,1], R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([x,y,z])        
        
    def dispImage(self):
        #self.frame=frame
        cv2.imshow('Webcam Feed - hit q to quit',self.frame)
        
    def captureImage(self):
        ret, self.frame = self.vid.read()
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        
    def trackAruco(self):
        corners, ids, rejected = aruco.detectMarkers(image=self.gray, dictionary=self.aruco_dict, parameters=self.parameters)
        
        if not ids is None:
            rvec = []
            tvec = []
            ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.camera_distortion)
            for markers in ret[0]:
                rvec.append(markers[0])
            for markers in ret[1]:
                tvec.append(markers[0])     
            #print(rvec)
            # only for video
            aruco.drawDetectedMarkers(self.frame, corners)
            for i in range(len(rvec)):
                aruco.drawAxis(self.frame, self.camera_matrix, self.camera_distortion, rvec[i], tvec[i], 5)

            for i in range(len(ids)):
                x = tvec[i][0]
                z = tvec[i][2]
                    
                R_ct = np.matrix(cv2.Rodrigues(rvec[i])[0])
                R_tc = R_ct.T

                roll_camera, pitch_camera, yaw_camera = self.rotationMatrixToEulerAngles(self.R_flip*R_tc)
#                self.pose.marker_id.append(ids[i][0])
#                 self.pose.distance.append(z)
#                 self.pose.orientation.append(math.degrees(pitch_camera))
#                 self.pose.marker_position.append(x)
            

            str_attitude = 'CAMERA Attitude r=%4.0f p=%4.0f y=%4.0f'%(math.degrees(roll_camera),math.degrees(pitch_camera), math.degrees(yaw_camera))
            cv2.putText(self.frame, str_attitude, (0,250),self.font,1,(0,255,0),2,cv2.LINE_AA)

            str_position = 'Marker Position x=%4.0f y=%4.0f z=%4.0f'%(tvec[0][0], tvec[0][1], tvec[0][2])
            cv2.putText(self.frame, str_position, (0,150),self.font,1,(0,255,0),2,cv2.LINE_AA)
        
            marker_id = 'Marker ID %4.0f'%(ids[i][0])
            cv2.putText(self.frame, marker_id, (0,200), self.font,1,(0,255,0),2,cv2.LINE_AA)

if __name__ == '__main__':
    calib_path = "/home/rover1/catkin_rover/src/rr_control_input_manager/orbweaver/src/"
    camera_mat = np.loadtxt(calib_path+'matrix.txt', delimiter=',')
    camera_dist = np.loadtxt(calib_path+'dist.txt', delimiter=',')
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    marker_sz = 18
    aruco_tracker = ArucoTracker(marker_size=marker_sz, camera_matrix=camera_mat, camera_distortion=camera_dist)
    
    while(True):

        aruco_tracker.captureImage()
        aruco_tracker.trackAruco()
        aruco_tracker.dispImage()
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    aruco_tracker.vid.release()
    cv2.destroyAllWindows()
