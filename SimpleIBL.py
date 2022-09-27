import cv2
import time
import numpy as np
from cv2 import aruco
import math

cv2.namedWindow("Image Feed")
cv2.moveWindow("Image Feed",159, -25)
index = -1
cap = cv2.VideoCapture(index, cv2.CAP_V4L)
#ret, buffer = stream.read()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#cap.set(cv2.CAP_PROP_FRAME_FPS, 40)

prev_frame_time = time.time()


while True:

    ret, frame = cap.read()
    new_frame_time = time.time()
    fps = 1/(new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    cv2.putText(frame, "FPS" + str(int(fps)), (10,40), cv2.FONT_HERSHEY_PLAIN, 3,(100,255,0), 2, cv2.LINE_AA)

    #cv2.imshow('frame', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"): break


cap.release()
cv2.destroyAllWindows()
