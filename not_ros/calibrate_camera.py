import numpy as np
import cv2

import os
import json
import yaml

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Number of object points 
num_intersections_in_x = 10  
num_intersections_in_y = 7

# Size of square in meters
square_size = 0.025

# Arrays to store 3D points and 2D image points
obj_points = []
img_points = []


def create_param_file(params):
    #create a yaml file in the format:
    '''
    image_width: 640
    image_height: 480
    camera_name: usb_cam
    camera_matrix:
    rows: 3
    cols: 3
    data: [445.10792629,   0.0 ,337.72928031,  0.0 ,  445.48201089 ,246.31395522,  0.0,0.0,1.0]
    distortion_model: plumb_bob
    distortion_coefficients:
    rows: 1
    cols: 5
    data: [ 0.11847053, -0.35759139 , 0.00148395,  0.00276497 , 0.61140408]
    rectification_matrix:
    rows: 3
    cols: 3
    data: [1, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 1]
    projection_matrix:
    rows: 3
    cols: 4
    data: [445.10792629,   0.0 ,        337.72928031, 0.0, 445.48201089 ,246.31395522, 0.0, 0.0, 1.0, 0.000000, 1.000000, 0.000000]
    '''

    #create a dictionary
    data = dict()
    data['image_width'] = params['width']
    data['image_height'] = params['height']
    data['camera_name'] = 'usb_cam'
    data['camera_matrix'] = dict()
    data['camera_matrix']['rows'] = 3
    data['camera_matrix']['cols'] = 3
    data['camera_matrix']['data'] = params['K'].flatten().tolist()
    data['distortion_model'] = 'plumb_bob'
    data['distortion_coefficients'] = dict()
    data['distortion_coefficients']['rows'] = 1
    data['distortion_coefficients']['cols'] = 5
    data['distortion_coefficients']['data'] = params['D'].flatten().tolist()
    data['rectification_matrix'] = dict()
    data['rectification_matrix']['rows'] = 3
    data['rectification_matrix']['cols'] = 3
    data['rectification_matrix']['data'] = [1, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 1]
    data['projection_matrix'] = dict()
    data['projection_matrix']['rows'] = 3
    data['projection_matrix']['cols'] = 4
    data['projection_matrix']['data'] = params['K'].flatten().tolist() + [0.0, 1.0, 0.0, 0.0]

    #write in a yaml file
    with open ('params/camera_params.yaml', 'w') as f:
        #prettu print
        yaml.dump(data, f, default_flow_style=None)

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
        #refine
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        img_points.append(corners2)

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

params={'K': K, 'D': D, "width": img_shape[0], "height": img_shape[1]}

create_param_file(params)