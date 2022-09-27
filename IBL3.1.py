import cv2
#import time
import numpy as np
from cv2 import aruco
import sys, time, math

id_to_find  = 12
marker_size  = 10 #- [cm]

font = cv2.FONT_HERSHEY_PLAIN

marker_size = 100

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])



with open ('camer_cal.npy', 'rb') as f:
    camera_matrix = np.load(f)
    camera_distortion = np.load(f)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

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

    if ids is not None and ids[0] == id_to_find:
        ret, frame = cap.read()
        Detected_Id = ids[0][0]

        print(Detected_Id)

        rvec_list_all, tvec_list_all, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        rvec = rvec_list_all[0][0]
        tvec = tvec_list_all[0][0]

        aruco.drawDetectedMarkers(frame, corners)
        #aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 100)

        #tvec_str = "X=%4.0f  Y=%4.0f Z=%4.0f "%(tvec[0], tvec[1], tvec[2])
        #cv2.putText(frame, tvec_str, (20,406), cv2.FONT_HERSHEY_PLAIN, 1.5,(0,0,255), 1, cv2.LINE_AA)
        #X_Axis = tvec[0]
        #Y_Axis = tvec[1]
        #Z_Axis = tvec[2]

        str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T

        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                            math.degrees(yaw_marker))
        cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        pos_camera = -R_tc*np.matrix(tvec).T

        str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], -(pos_camera[1]), pos_camera[2])
        cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        print("X=%4.0f  Y=%4.0f Z=%4.0f "%(pos_camera[0], -(pos_camera[1]), pos_camera[2]))

        #-- Get the attitude of the camera respect to the frame
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
        str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                            math.degrees(yaw_camera))
        cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        

    cv2.imshow('frame', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"): break


cap.release()
cv2.destroyAllWindows()
