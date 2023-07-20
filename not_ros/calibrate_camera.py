import numpy as np
import cv2

import os
import json
# Number of object points 
num_intersections_in_x = 10  
num_intersections_in_y = 7

# Size of square in meters
square_size = 0.025

# Arrays to store 3D points and 2D image points
obj_points = []
img_points = []

# Prepare expected object 3D object points (0,0,0), (1,0,0) ...
object_points = np.zeros((num_intersections_in_x*num_intersections_in_y,3), np.float32)
object_points[:,:2] = np.mgrid[0:num_intersections_in_x, 0:num_intersections_in_y].T.reshape(-1,2)
object_points = object_points*square_size
#print current dir
print("Current dir: ", os.getcwd())

imgs=os.listdir('not_ros/images')

img_shape=None

for img_name in imgs:
    img = cv2.imread(os.path.join('not_ros/images',img_name))
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img_shape= gray.shape[::-1]

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (num_intersections_in_x,num_intersections_in_y),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        obj_points.append(object_points)
        img_points.append(corners)

        # Draw and display the corners
        drawn_img = cv2.drawChessboardCorners(img, (num_intersections_in_x,num_intersections_in_y), corners,ret)
        #cv2.imshow('img',drawn_img)
        #cv2.waitKey(500)

    cv2.destroyAllWindows()
print("img_shape = ", img_shape)

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points,img_shape,None,None)

K=np.array(mtx)
D=np.array(dist)

print("K = ", K)
print("D = ", D)

#write in a json file
with open ('not_ros/camera_params.json', 'w') as f:
    #prettu print
    json.dump({'K': K.tolist(), 'D': D.tolist()}, f, indent=4, sort_keys=True)