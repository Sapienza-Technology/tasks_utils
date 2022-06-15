#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import cos
from math import sin
import numpy as np

old_x=1000
old_y=1000
old_z=1000
thres=0.01



global real_start_x
global real_start_y
global real_start_z



def callback(data,count):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    x=data.pose.pose.position.x
    y=data.pose.pose.position.y
    z=data.pose.pose.position.z
    if count[0]%10==0: print("estimated position:\n  x:%.4f  y:%.4f  z:%.4f" % (x,y,z))
    count[0]+=1

    #if abs(old_x-x)>thres or abs(old_y-y)>thres or abs(old_z-z)>thres:
        
    #    old_y=y
    #    old_x=x
    #    old_z=z

def real_pos(data,first_receiving,count_real,pub,rate):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    #ground truth position received, global coordinates
    #x is y
    x=data.pose.pose.position.x
    y=data.pose.pose.position.y
    z=data.pose.pose.position.z
    #print("Raw Real position:\n  x:%.4f  y:%.4f  z:%.4f " % (y,x,z))
    
    #must transform in robot coordinates w.r.t initial position

    #initial data of robot
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

    global_pos=np.transpose([x,y,1])
    
    #ground truth position with respect to starting position
    local_pos=np.matmul(inv_pose,global_pos)
    x=local_pos[0]
    y=local_pos[1]
    #z don't change, remain the same
    
    if count_real[0]%10==0:
        #x and y switched
        print("Real position:\n  x:%.4f  y:%.4f  z:%.4f " % (x,y,z))
    count_real[0]+=1
    if not rospy.is_shutdown():
        odom = Odometry()
        current_time = rospy.Time.now()
        odom.header.stamp = current_time
        odom.header.frame_id = "map"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), data.pose.pose.orientation)

        # set the velocity
        odom.child_frame_id = "base_footprint"
        #odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        pub.publish(odom)
        rate.sleep()

def listener():
    count=[0]
    count_real=[0]
    first_receiving=True
    est_pos=[0,0,0]
    rospy.init_node('check_odom', anonymous=True)
    rospy.Subscriber("/odom", Odometry, lambda x: callback(x,count))
    rospy.Subscriber("/ground_truth", Odometry,lambda x: real_pos(x,first_receiving,count_real,pub,rate))
    
    pub = rospy.Publisher('/ground_truth2', Odometry, queue_size=10)
    rate = rospy.Rate(30) # 10hz

    
    rospy.spin()

if __name__ == '__main__':
    #global first_receiving
    #first_receiving=True
    listener()