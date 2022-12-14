import numpy as np
import cv2
import glob

cb_width = 10
cb_height = 7
cb_square_size = 12

# Termination creiteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points
cb_3D_points = np.zeros((cb_width * cb_height, 3), np.float32)
cb_3D_points[:,:2] = np.mgrid[0:cb_width, 0:cb_height].T.reshape(-1,2) * cb_square_size

#Arrays to store object points and image  points from all the images.

list_cb_3d_points = []   # 3d points in real worls space
list_cb_2d_img_points = []  # 2d points in image plane

list_images = glob.glob('*.jpg')

for frame_name in list_images:
    img = cv2.imread(frame_name)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (10,7), None)

    if ret == True:
        list_cb_3d_points.append(cb_3D_points)

        corners2 = cv2.cornerSubPix(gray, corners, (12,12), (-1,-1), criteria)
        list_cb_2d_img_points.append(corners2)

        # Draw and dispaly the corner
        cv2.drawChessboardCorners(img, (cb_width, cb_height), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(list_cb_3d_points, list_cb_2d_img_points, gray.shape[::-1], None, None)

print("Calibration Matrix: ")
print(mtx)
print("Disortion: ", dist)

with open ('camer_cal.npy', "wb") as f:
    np.save(f, mtx)
    np.save(f, dist)
