import cv2
#import time
import numpy as np
from cv2 import aruco
import math


marker_size = 100

with open ('camer_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

cap = cv2.VideoCapture(0)

camera_width = 640
camera_height = 480
camera_frame_rate = 40

cap.set(2, camera_width)
cap.set(4, camera_height)
cap.set(5, camera_frame_rate)


while True:

    ret, frame = cap.read()

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray_frame, aruco_dict, camera_matrix, camera_distortion)

    if ids is not None:
        ret, frame = cap.read()
        Detected_Id = ids[0][0]

        print(Detected_Id)
        aruco.drawDetectedMarkers(frame, corners)

        rvec_list_all, tvec_list_all, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        rvec = rvec_list_all[0][0]
        tvec = tvec_list_all[0][0]

        #aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 100)


        tvec_str = "X=%4.0f  Y=%4.0f Z=%4.0f "%(tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, tvec_str, (20,406), cv2.FONT_HERSHEY_PLAIN, 1.5,(0,0,255), 1, cv2.LINE_AA)
        X_Axis = tvec[0]
        Y_Axis = tvec[1]
        Z_Axis = tvec[2]

        print("X=%4.0f  Y=%4.0f Z=%4.0f "%(X_Axis, Y_Axis, Z_Axis))

    cv2.imshow('frame', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"): break


cap.release()
cv2.destroyAllWindows()
