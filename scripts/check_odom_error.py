#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import cos
from math import sin
import numpy as np
from math import sqrt
import message_filters
from tf.transformations import euler_from_quaternion, quaternion_from_euler


'''
This node take the estimated position and the ground truth position and compute the error between them
It's important to notice that the ground truth position initially is with respect to the global frame of gazebo, 
we need it to be with respect to the robot starting position
'''

def convert_gt(gt_odom,start_gt_pose):
    #global coordinates
    gt_x=gt_odom.pose.pose.position.x
    gt_y=gt_odom.pose.pose.position.y
    gt_z=gt_odom.pose.pose.position.z

    #robot starting position
    r_x= start_gt_pose.pose.pose.position.x
    r_y= start_gt_pose.pose.pose.position.y
    r_z= start_gt_pose.pose.pose.position.z


    #covnert quaternion to euler
    q = start_gt_pose.pose.pose.orientation

    '''
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    '''

    orientation_list = [q.x, q.y, q.z, q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    

    theta= yaw
    
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

    print("Real position|  x:%.4f  y:%.4f  z:%.4f " % (gt_x,gt_y,gt_z))
    
    #header
    
    new_gt=Odometry()   
    new_gt.header.stamp = rospy.Time.now()
    new_gt.header.frame_id = "odom"
    new_gt.pose.pose.position.x=gt_x
    new_gt.pose.pose.position.y=gt_y
    new_gt.pose.pose.position.z=gt_z
    
    #current_gt_pose.child_frame_id = "base_footprint"

    return new_gt

    


def measure_error(est_odom,gt_odom,pub,rate,start_gt_pose):
    
    if not start_gt_pose: return
    
    #estimated position
    x=est_odom.pose.pose.position.x
    y=est_odom.pose.pose.position.y
    z=est_odom.pose.pose.position.z
    print("estimated position|  x:%.4f  y:%.4f  z:%.4f" % (x,y,z))

    #Ground truth position
    gt_odom=convert_gt(gt_odom,start_gt_pose)
    gt_odom.pose.pose.position.z=z

    # publish the message
    pub.publish(gt_odom)
    rate.sleep()

    gt_x=gt_odom.pose.pose.position.x
    gt_y=gt_odom.pose.pose.position.y
    #euclidean distance between ground truth position and estimated position
    error=sqrt((gt_x-x)**2+(gt_y-y)**2)
    print("error: %.4f" % error)
    print()
    


def check_odom_error():
    print("Starting odom error node")
    rospy.init_node('check_odom', anonymous=True)

    #get only a sinlge message, used to set the global starting position of the robot in gazebo
    start_gt_pose = rospy.wait_for_message('/ground_truth', Odometry, timeout=20)

    #fixed ground truth, wth respect to starting position
    pub = rospy.Publisher('/ground_truth2', Odometry, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    est_odom = message_filters.Subscriber('/odom', Odometry)
    gt_odom = message_filters.Subscriber('/ground_truth', Odometry)

    queue_size=10
    ts = message_filters.ApproximateTimeSynchronizer([est_odom, gt_odom],queue_size,0.1)
    ts.registerCallback(lambda est_odom, gt_odom: measure_error(est_odom,gt_odom,pub,rate,start_gt_pose))


    rospy.spin()



if __name__ == '__main__':
    check_odom_error()