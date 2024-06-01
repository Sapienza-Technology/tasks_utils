
import rospy
from  geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

import math
import time
import numpy as np


#** Rover Parameters
wheel_radius=0.08 #radius of the wheel [m]

wheel_positions=[  #*wheel distances from the x axis and y axis [m]
    [0.51,0.4],    #front left
    [0.51,-0.4],   #front right
    [0,0.47],      #mid left
    [0,-0.47],     #mid right
    [-0.51,0.41],  #rear left
    [-0.51,-0.41]  #rear right
]

D=None #distance between the wheels along y axis [m],if None computed at runtime from wheel_positions.

#** Firmware parameters
max_steer=math.pi/4             #max steering angle [rad]
MAX_V=1                         #max linear velocity [m/s]
IN_PLACE_VEL= MAX_V/3           #velocity for in place rotation
in_place_delay=4                #How much to wait for wheel to be in position before starting inplace movement [s]
rover_threshold_stopped=3.0     #how many seconds to wait when rover is stopped before resetting the wheels [s]
turning_ratio_threshold=0.6     #threshold on v/w ratio to avoid turning radius too small in ackermann steering 
enable_steer_limit=False        #enable steering angle limit, if true the steering angle will be limited to max_steer


#** FUNCTIONS

#Convert linear velocities to angular velocities for the wheels
def linear2angular(l):
    new_l=l/wheel_radius
    return new_l.tolist()

def compute_rover_width():
    #get the distance between the wheels along y axis
    #compare all the pairs of wheels and get the maximum distance
    if D is not None and D>0:
        print("Selected custom rover width: ",D)
        return D
    
    max_width=0
    for i in range(0,len(wheel_positions),2):
        width=abs(wheel_positions[i][1]-wheel_positions[i+1][1])
        if width>max_width:
            max_width=width
    #print("Rover width: ",max_width)
    return max_width

#utility function to preety print the rover state, made to facilitate terminal reset if needed
def print_rover_state(string):
    sep="----"
    print("\n{}{}{}".format(sep,string,sep))

def reset_wheels(params):
    print_rover_state("RESET WHEELS")
    null_vels=np.zeros(6)
    null_angles=np.zeros(4)
    publish_velocities(null_vels,null_angles,params)

#Publish the computed velocities and angles
def publish_velocities(velocities,angles,params):
    while params["mutex_publishing"]==True:
        time.sleep(0.05)
    params["mutex_publishing"]=True

    pub=params["pub"]
    print("Sending new command:\n   vels:{}\n   angles:{}".format(np.round(velocities,3),np.round(angles,3)))
    #Buid message by explicitly setting layout and dimensions
    msg=Float32MultiArray()
    msg.layout.data_offset=0
    msg.layout.dim.append(MultiArrayDimension())
    msg.layout.dim[0].size=10

    #Adding data and publishing
    vels=list(velocities)
    angles=list(angles)
    data=vels+angles
    msg.data=data
    pub.publish(msg)
    params["mutex_publishing"]=False

def is_in_place_rotation_command(data):
    if abs(data.linear.x)<0.001 and abs(data.angular.z)>0.001:
        return True
    return False

def is_stop_command(data):
    if abs(data.linear.x)<0.001 and abs(data.angular.z)<0.001:
        return True
    return False
'''
Given a twist message, compute the wheel velocities
for a 6 driving wheels rover with 4 steering wheels
'''
def compute_wheel_velocities(data,params):

    v=data.linear.x
    w=data.angular.z

    #*ROBOT STOPPED
    #Stop wheels but maintain angle, it will be reset after some time
    if is_stop_command(data):
        print_rover_state("ROVER STOP")
        wheel_velocities=np.zeros(6)
        wheel_angles=params["last_angles"]
        params["last_stopped_time"]=time.time()
        params["is_robot_stopped"]=True
        return wheel_velocities,wheel_angles
    else:
        params["is_robot_stopped"]=False

    #*IN PLACE ROTATION
    if is_in_place_rotation_command(data):
        print_rover_state("IN PLACE ROTATION")
        wheel_angles=[-max_steer,max_steer,max_steer,-max_steer]
        
        for i in range(len(wheel_positions)):
            wheel_x=wheel_positions[i][0]
            wheel_y=wheel_positions[i][1]
            
            r=math.sqrt(wheel_x*wheel_x+wheel_y*wheel_y)

            #cambia segno se ruota in senso sbagliato
            sgn= 1 if wheel_y < 0 else -1
            if w<0: 
                sgn*=-1
            vel = w*r*sgn
            wheel_velocities[i]=IN_PLACE_VEL*sgn
        wheel_velocities=linear2angular(np.array(wheel_velocities))
        return wheel_velocities,wheel_angles
    
    #* GOING STRAIGHT
    if abs(w)<1e-3:
        print_rover_state("ACKERMANN: STRAIGHT")
        wheel_velocities=[v,v,v,v,v,v]
        wheel_angles= np.zeros(4)
        params["last_angles"]=wheel_angles #update last angles
        return wheel_velocities,wheel_angles


    #* ACKERMANN STEERING

    #*Check turning radius
    #Threshold should activate only in ackermann steering
    if w!=0 and not is_in_place_rotation_command(data) and not is_stop_command(data):
        ratio=abs(v/w)
        if ratio<=turning_ratio_threshold:
            #decrease w so that new ratio is equal to threshold
            new_w = v/ (turning_ratio_threshold*w)
            print(f"WARN: v/w ratio too small. {np.round(v,3)}/{np.round(w,3)}={np.round(ratio,3)}")
            print(f"Decreasing w to {np.round(new_w,3)} to avoid small turning radius")
            w=new_w

    #Compute parameters for ackermann steering
    d=params["rover_width"]

    vl=v - w*d/2
    vr=v + w*d/2
    
    turning_radius=d*(vl+vr)/(vr-vl)/2
    turning_rate=(vr-vl)/d

    print_rover_state("ACKERMANN: Turning with radius: {}, rate: {} ".format(
        np.round(turning_radius,3),np.round(turning_rate,3)))
    
    wheel_velocities=np.zeros(6)
    wheel_angles=np.zeros(4)

    #Compute wheel velocities
    for i in range(len(wheel_positions)):
        wheel_x=wheel_positions[i][0]
        wheel_y=wheel_positions[i][1]
        
        r=math.sqrt(wheel_x*wheel_x+(turning_radius-wheel_y)*(turning_radius-wheel_y))

        sgn= 1 if turning_radius-wheel_y > 0 else -1
        vel = turning_rate*r*sgn
        wheel_velocities[i]=vel
    
    #*compute wheel angles

    steer_idx=0 #which steering wheel is being considered
    for i in range(len(wheel_positions)):
        
        #ignore middle wheels, they do not steer
        if i==2 or i==3:
            continue

        wheel_x,wheel_y = wheel_positions[i]

        angle=math.atan2(wheel_x,turning_radius-wheel_y)

        #flip angle by 180 deg if steering rotation is past 90 deg
        if angle > math.pi/2:
            angle-=math.pi
        elif angle < -math.pi/2:
            angle+=math.pi
        
        if enable_steer_limit:
            if angle > max_steer:
                print("WARN: angle > max_steer, angle: ",angle)
            wheel_angles[steer_idx]=min(angle,max_steer)

        wheel_angles[steer_idx]=angle
        steer_idx+=1

    #Convert linear velocities to angular velocities and return
    wheel_velocities=linear2angular(np.array(wheel_velocities))
    params["last_angles"]=wheel_angles #update last angles
    vels=np.array(wheel_velocities)
    angles=np.array(wheel_angles)
    return vels,angles

def callback(data,params):
    #new command received, reset is no more valid
    params["rover_reset_done"]=False 
    null_vels=np.zeros(6)
    null_angles=np.zeros(4)

    velocities,angles=compute_wheel_velocities(data,params)
    print(f"Inputs: linear: {np.round(data.linear.x,3)} angular: {np.round(data.angular.z,3)}")
    #print("output velocities: "+np.round(velocities,3)+" angles: "+np.round(angles,3))

    if is_in_place_rotation_command(data):
        if params["in_place_configuration"]==False:
            params["in_place_configuration"]=True
            # Send first only the steering angles and wait for the wheels to be in place
            print("Rotation in place: setting steering angles...")
            publish_velocities(null_vels,angles,params)
            time.sleep(in_place_delay)
            print("Rotation in place: starting rotation...")
    else:
        #If I was in place, wait wheel resetting before sending new velocities
        if params["in_place_configuration"]==True:
            #TODO do not reset to 0, set to steering angle or wait based on how much the wheel has to rotate
            print("Rover is in configuration in place, waiting for wheel to be in place")
            publish_velocities(null_vels,null_angles,params)
            time.sleep(in_place_delay)
        params["in_place_configuration"]=False

    #Publish computed velocity
    publish_velocities(velocities,angles,params)

    params["rate"].sleep()
    

def main():
    #publish a array
    pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    rospy.init_node('firmware_CC8', anonymous=True)
    rate = rospy.Rate(30)

    print("\n---------Firmware CC8 Started----------\n")
    D=compute_rover_width()

    #Init common parameters and variables
    params={
        "last_angles": np.zeros(4),                         #last angles sent to the wheels,used to keep wheels steered when rover is stopped 
        "is_robot_stopped": True,                           #flag to check if the rover is stopped
        "last_stopped_time": time.time(),                   #last time the rover was stopped
        "pub": pub,                                         #publisher
        "in_place_configuration": False,                    #flag to check if the rover is in place configuration
        "mutex_publishing": False,                          #mutex to avoid publishing simultaneously from different threads
        "rover_reset_done": False,                          #flag to check if the rover wheels have been reset    
        "rover_width": D,                                   #rover width
        "rate": rate                                        #rate of the main loop
    }
    
    rospy.Subscriber("/cmd_vel", Twist, lambda x: callback(x,params)) 

    try:
        while not rospy.is_shutdown():
            #Check if the robot is stopped for more than some seconds. If so, reset the wheels angles to 0
            is_robot_stopped=params["is_robot_stopped"]
            last_stopped_time=params["last_stopped_time"]
            reset_done=params["rover_reset_done"]
            if not reset_done and is_robot_stopped and time.time()-last_stopped_time > rover_threshold_stopped:
                
                #reset only if at least one angle is not 0
                if sum(params["last_angles"])>0:
                    #print("rover stopped for more than {} seconds, resetting wheels".format(rover_threshold_stopped)) 
                    #reset wheels angles
                    reset_wheels(params)
                    params["rover_reset_done"]=True

            time.sleep(0.1) #don't do the check continuously
    except KeyboardInterrupt:
        print("keyboard interrupt")

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("keyboard interrupt")
        pass
