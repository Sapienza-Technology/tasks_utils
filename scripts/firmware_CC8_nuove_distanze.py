
import rospy
from  geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import math
import time
import numpy as np

import threading

wheel_radius=0.8
d=0.94 #distance between the wheels along y axis

#FRONT, MID, REAR (first left, then right)
wheel_positions=[ 
    [0.51,0.4],
    [0.51,-0.4],
    [0,0.47],
    [0,-0.47],
    [-0.51,0.41],
    [-0.51,-0.41]
]

max_steer=math.pi/4
MAX_V=1
MAX_W=1/wheel_radius
IN_PLACE_VEL= MAX_V/4

last_angles=[0,0,0,0]
is_robot_stopped=False
rover_threshold_stopped=3.0 #how many seconds to wait when stopped before resetting the wheels
last_stopped_time=0
thread_check_stopped=None



def linear2angular(l):
    new_l=l/0.08
    return new_l.tolist()

def publish_velocities(pub,velocities,angles):
    print("sending velocities: "+str(velocities)+" angles: "+str(angles))
    msg=Float32MultiArray()
    msg.data=velocities+angles
    pub.publish(msg)

'''
Thread that check if the robot is stopped for more than rover_threshold_stopped seconds.
If so, reset the wheels angles to 0
'''
def check_robot_stopped(params):
    print("thread for resetting wheel after {} seconds of rover stopped started".format(params["rover_threshold_stopped"]))
    pub=params["pub"]
    rover_threshold_stopped=params["rover_threshold_stopped"]
    try:
        while True:
            is_robot_stopped=params["is_robot_stopped"]
            last_stopped_time=params["last_stopped_time"]
            if is_robot_stopped and time.time()-last_stopped_time > rover_threshold_stopped:
                #print("rover stopped for more than {} seconds, resetting wheels".format(rover_threshold_stopped))
                #reset wheels angles
                velocities=[0,0,0,0,0,0]
                angles=[0,0,0,0]

                #create multliarray
                publish_velocities(pub,velocities,angles)
                time.sleep(0.5)
            else:
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass





'''
Given a twist message, compute the wheel velocities
for a 6 driving wheels rover with 4 steering wheels
'''
def compute_wheel_velocities(data,params):
    print("\n")

    #to return
    wheel_velocities=[0,0,0,0,0,0]
    wheel_angles=[0,0,0,0]
    last_angles=params["last_angles"]

    v=data.linear.x
    w=data.angular.z
    #w=-w #TODO resolve?

    turning_radius=None
    turning_rate=None

    vl=v - w*d/2
    vr=v + w*d/2
    print("vl: "+str(vl)+" vr: "+str(vr))
    
    #ROBOT STOPPED, do not reset the wheels,other thread will do it
    if abs(v)<0.001 and abs(w)<0.001:
        print("robot fermo")
        wheel_velocities=[0,0,0,0,0,0]
        wheel_angles=last_angles
        params["last_stopped_time"]=time.time()
        params["is_robot_stopped"]=True
        return wheel_velocities,wheel_angles
    else:
        params["is_robot_stopped"]=False
    

    #IN PLACE ROTATION
    if abs(w)>0.001 and abs(v)<0.001:
        print("in place rotation")
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
    
  
    #ACKERMANN STEERING
    if vl==vr:
        turning_radius=0
        turning_rate=0
    else:
        turning_radius=d*(vl+vr)/(vr-vl) /2
        turning_rate=(vr-vl)/d 
    #print("turning_radius: "+str(turning_radius)+" turning_rate: "+str(turning_rate))
    if turning_radius==0:
        wheel_velocities=[vl,vl,vl,vl,vl,vl]
        wheel_angles=[0,0,0,0]
    else:
        for i in range(len(wheel_positions)):
            wheel_x=wheel_positions[i][0]
            wheel_y=wheel_positions[i][1]
            
            r=math.sqrt(wheel_x*wheel_x+(turning_radius-wheel_y)*(turning_radius-wheel_y))

            sgn= 1 if turning_radius-wheel_y > 0 else -1
            vel = turning_rate*r*sgn
            wheel_velocities[i]=vel/wheel_radius
        
        #compute wheel angles
        steer_idx=0
        for i in range(len(wheel_positions)):
            #ignore middle wheels
            if i==2 or i==3:
                continue

            wheel_x=wheel_positions[i][0]
            wheel_y=wheel_positions[i][1]

            angle=math.atan2(wheel_x,turning_radius-wheel_y)
            #angle=math.pi/2-angle

            #flip angle by 180 deg if steering rotation is past 90 deg
            if angle > math.pi/2:
                angle-=math.pi
            elif angle < -math.pi/2:
                angle+=math.pi
            
            #if angle > max_steer:
            #    print("WARN: angle > max_steer, angle: ",angle)
            #wheel_angles[steer_idx]=min(angle,max_steer)
            wheel_angles[steer_idx]=angle
            steer_idx+=1
    wheel_velocities=linear2angular(np.array(wheel_velocities))
    params["last_angles"]=wheel_angles
    return wheel_velocities,wheel_angles

def callback(data,pub,rate,params):
    in_place_delay=3 #How much to wait for wheel to be in position before starting inplace movement

    velocities,angles=compute_wheel_velocities(data,params)
    print("input linear: "+str(data.linear.x)+" angular: "+str(data.angular.z))
    print("output velocities: "+str(velocities)+" angles: "+str(angles))

    if abs(angles[0])==max_steer and abs(angles[1])==max_steer:
        if params["in_place_configuration"]==False:
            params["in_place_configuration"]=True
            print("IN place, sending only steer")
            temp_velocities=[0,0,0,0,0,0]
            publish_velocities(pub,temp_velocities,angles)
            time.sleep(in_place_delay)
            print("IN place, starting rotation")

    else:
        #If I was in place, wait wheel resetting before sending new velocities
        if params["in_place_configuration"]==True:
            print("in place configuration, waiting before moving")
            publish_velocities(pub,[0,0,0,0,0,0],[0,0,0,0])
            time.sleep(in_place_delay)
        params["in_place_configuration"]=False

    #create multliarray
    print("sending velocities: "+str(velocities)+" angles: "+str(angles))
    publish_velocities(pub,velocities,angles)

    rate.sleep()
    

def main():
    #publish a array
    pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    rospy.init_node('firmware_CC8', anonymous=True)
    rate = rospy.Rate(10)
    print("\n\n\navviato nodo firmware velocity\n\n\n")
    params={
        "in_place_configuration": False,
        "current_configuration": "linear", #linear or in place
        "last_angles": last_angles,
        "is_robot_stopped": is_robot_stopped,
        "rover_threshold_stopped": rover_threshold_stopped,
        "last_stopped_time": last_stopped_time,
        "pub": pub,
    }
    print("starting subscriber")
    #subscriber pass data using lambda function
    rospy.Subscriber("/cmd_vel", Twist, lambda x: callback(x,pub,rate,params))

    # Create a new thread that targets the check_robot_stopped function
    thread_check_stopped = threading.Thread(target=check_robot_stopped, args=(params,))

    # Start the thread
    thread_check_stopped.start()  
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        #stop thread
        thread_check_stopped.join()
