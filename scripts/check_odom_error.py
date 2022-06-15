#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import cos
from math import sin
import numpy as np
from math import sqrt
import message_filters

'''
This node take the estimated position and the ground truth position and compute the error between them
It's important to notice that the ground truth position initially is with respect to the global frame of gazebo, 
we need it to be with respect to the robot starting position
'''


def measure_error(est_odom,gt_odom,pub,rate):
    
    #estimated position
    x=est_odom.pose.pose.position.x
    y=est_odom.pose.pose.position.y
    z=est_odom.pose.pose.position.z
    print("estimated position|  x:%.4f  y:%.4f  z:%.4f" % (x,y,z))

    #Ground truth position

    #global coordinates
    gt_x=gt_odom.pose.pose.position.x
    gt_y=gt_odom.pose.pose.position.y
    gt_z=gt_odom.pose.pose.position.z

    #robot starting position
    r_x= -1.1046
    r_y= -14.8783
    r_z= 1.16
    theta=1.7
    '''
    #homogeneous matrix 2d
    #starting pose of robot
    pose=[
        [cos(theta), -sin(theta), r_x],
        [sin(theta), cos(theta), r_y],
        [0,0,1]
    ]
    '''
    #inverse computed with matlab
    inv_pose=[
        [cos(theta), sin(theta), -r_x*cos(theta) - r_y*sin(theta) ],
        [-sin(theta), cos(theta), r_x*sin(theta) - r_y*cos(theta)],
        [0,0,1]
    ]

    global_pos=np.transpose([gt_x,gt_y,1])
    
    #ground truth position with respect to starting position
    local_pos=np.matmul(inv_pose,global_pos)
    gt_x=local_pos[0]
    gt_y=local_pos[1]

    print("Real position|  x:%.4f  y:%.4f  z:%.4f " % (gt_x,gt_y,z))
    
    
    #distance between ground truth position and estimated position
    error=sqrt((gt_x-x)**2+(gt_y-y)**2)
    print("error: %.4f" % error)
    print()
    #publish ground truth position in correct frame
    if not rospy.is_shutdown():
        odom = Odometry()
        current_time = rospy.Time.now()
        odom.header.stamp = current_time
        odom.header.frame_id = "map"

        # set the position
        odom.pose.pose = Pose(Point(gt_x, gt_y, 0.), est_odom.pose.pose.orientation)

        # set the velocity
        odom.child_frame_id = "base_footprint"
        #odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        pub.publish(odom)
        rate.sleep()


def check_odom_error():
    rospy.init_node('check_odom', anonymous=True)

    pub = rospy.Publisher('/ground_truth2', Odometry, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    est_odom = message_filters.Subscriber('/odom', Odometry)
    gt_odom = message_filters.Subscriber('/ground_truth', Odometry)


    queue_size=10
    ts = message_filters.TimeSynchronizer([est_odom, gt_odom], queue_size)
    ts.registerCallback(lambda est_odom, gt_odom: measure_error(est_odom,gt_odom,pub,rate))
    rospy.spin()


if __name__ == '__main__':
    check_odom_error()