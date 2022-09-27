# Importing Libraries
import serial
import time
import cv2
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


#arduino = serial.Serial(port='COM6', baudrate=115200, timeout=.1)


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

        aruco.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 100)


        tvec_str = "X=%4.0f  Y=%4.0f Z=%4.0f "%(tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, tvec_str, (20,406), cv2.FONT_HERSHEY_PLAIN, 1.5,(0,0,255), 1, cv2.LINE_AA)
        X_Axis = tvec[0]
        Y_Axis = tvec[1]
        Z_Axis = tvec[2]

        print("X=%4.0f  Y=%4.0f Z=%4.0f "%(X_Axis, Y_Axis, Z_Axis))

        #def PWM_map(Value, in_min, in_max, out_min, out_max):
            #return int((Value-in_min)* (out_max-out_min)/(in_max-in_min) + out_min)

        #X_PWM = PWM_map(X_Axis, -2000, 2000, 1000, 2000)
        #Y_PWM = PWM_map(Y_Axis, -2000, 2000, 1000, 2000)

        #x_pwm_string =  str(X_PWM)
        #Z_PWM = PWM_map(Z_Axis, -2000, 2000, 1, 255)

        #xy_pwm_string =  str(X_PWM)+"-"+ "\n" +str(Y_PWM)

        #print("PWM value of X_AXis: %4.0f  PWM value of Y_AXis: %4.0f   "%(X_PWM, Y_PWM))
        #print("PWM value of X_AXis: %s   "%(X_PWM))

        #arduino.write(bytes(str(X_PWM),'utf-8'))
        # arduino.write(bytes (str(Y_PWM),'utf-8'))

    cv2.imshow('frame', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"): break


cap.release()
cv2.destroyAllWindows()
