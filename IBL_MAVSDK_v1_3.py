import cv2
#import time
import numpy as np
from cv2 import aruco
import sys, time, math
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
pos_1=0.00
pos_2=0.00
pos_3=0.00

pos_1a=0.0
pos_2a=0.0
pos_3a=0.0

pos1=0
pos2=0
pos3=0
id_to_find  = 12
marker_size  = 10 #- [cm]

font = cv2.FONT_HERSHEY_PLAIN

marker_size = 100
system_addr = "localhost"#"serial:///dev/ttyUSB0"#"localhost"
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def convert_to_NED(forward, right, down,yaw):
    theta = yaw * (math.pi / 180)
    north = forward * math.cos(theta) - right * math.sin(theta)
    east = forward * math.sin(theta) - right * math.cos(theta)
    return north, east, down

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

async def print_mode(drone):
    async for mode in drone.telemetry.flight_mode():
        return mode;
    
async def print_odometry(drone):
    async for odo in drone.telemetry.odometry():
        print(f"{odo.position_body.x_m},{odo.position_body.y_m},{odo.position_body.z_m}\n")
        
async def print_gps(drone):
    async for gps in drone.telemetry.gps_info():
        print(gps)

async def print_height(drone):
    async for height in drone.telemetry.raw_gps():
        print("height:%f"%height.absolute_altitude_m)

async def run(system_addr):
    print("main loop")
    """ Does Offboard control using position NED coordinates. """
    a=1
    c=1
    drone = System()
    await drone.connect(system_address=system_addr)
    await drone.telemetry.set_rate_odometry(5)
    #await drone.telemetry.set_rate_distance_sensor(10)
    #asyncio.ensure_future(print_odometry(drone))
    #asyncio.ensure_future(print_gps(drone))
    #asyncio.ensure_future(print_height(drone))
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok :#and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
        
    async for gps_info in drone.telemetry.heading():
        yaw = gps_info.heading_deg
        print(yaw)
        break
    
    while c ==1:
        print("inside")
        b = asyncio.ensure_future(print_mode(drone))
        await b
        j = b.result()
        print(j.value)
        if j.value == 11:
            print("correct")
            c=0

    async for odo in drone.telemetry.odometry():
        print(f"{odo.position_body.x_m},{odo.position_body.y_m},{odo.position_body.z_m}\n")
        x=round(odo.position_body.x_m,2)
        y=round(odo.position_body.y_m,2)
        z=round(odo.position_body.z_m,2)
        break
    
    '''await drone.action.arm()
    await asyncio.sleep(2)'''


    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    
    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    #asyncio.ensure_future(print_height(drone))
    
    n, e, d = convert_to_NED(0.0,0.0, 0.0,yaw)
    await drone.offboard.set_position_ned(PositionNedYaw(x+n, y+e, z+d, yaw))
    await asyncio.sleep(5)
    
    while a == 1:
        key,pos_1,pos_2,pos_3=cv_val()

        pos_1a=float((pos_1*1)/100.0)
        pos_2a=float((pos_2*1)/100.0)
        pos_3a=float((pos_3*1)/100.0)
        print("XA=%f  YA=%f ZA=%f"%(pos_1a, pos_2a, pos_3a))
        if key == ord("q"): break
        n, e, d = convert_to_NED(pos_2a, pos_1a, 0.0,yaw)
        await drone.offboard.set_position_ned(PositionNedYaw(x+n, y+e, z+0.0 , yaw))
        #await asyncio.sleep(5)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")
    await drone.action.land()

def cv_val():
    global pos_1, pos_2, pos_3, pos_1a, pos_2a, pos_3a
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
        pos_1=float(pos_camera[0])
        pos_2=float(pos_camera[1])
        pos_3=float(pos_camera[2])
        #-- Get the attitude of the camera respect to the frame
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
        str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                            math.degrees(yaw_camera))
        cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
    
    cv2.imshow('frame', frame)
    key = cv2.waitKey(1) & 0xFF


    return key,pos_1,pos_2,pos_3
    print("X1=%4.0f  Y1=%4.0f Z1=%4.0f"%(pos_1, pos_2, pos_3))
        
while True:
    try:
        loop = asyncio.get_event_loop()
        
        loop.run_until_complete(run(system_addr))
    except KeyboardInterrupt:
        sys.exit()
        


cap.release()
cv2.destroyAllWindows()

