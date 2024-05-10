"""
Firmware for the CC8 rover.
It subscribes to messages on the /cmd_vel topic and computes the wheel velocities and angles for the 6 driving wheels and 4 steering wheels.

It also checks if the rover is not going to the desired speed and increase the speed of the wheels accordingly.

"""
import rospy
from  geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

import math
import time
import numpy as np

import threading

#** Rover parameters
wheel_radius=0.8
d=0.94 #distance between the wheels along y axis
#FRONT, MID, REAR (first left, then right)
wheel_positions=[ 
    [0.245,0.4],
    [0.245,-0.4],
    [0,0.47],
    [0,-0.47],
    [-0.2575,0.43],
    [-0.2575,-0.43]
]

#** Firmware parameters
max_steer=math.pi/4
MAX_V=1
MAX_W=MAX_V/wheel_radius
IN_PLACE_VEL= MAX_V/4
in_place_delay=3 #How much to wait for wheel to be in position before starting inplace movement
enable_check_reset_wheel=False
odometry_topic="/odom"


#** Variables
last_angles=[0,0,0,0]
is_robot_stopped=False
rover_threshold_stopped=3.0 #how many seconds to wait when stopped before resetting the wheels
last_stopped_time=0
thread_check_wheel_reset=None

current_speed = 0


def linear2angular(l):
    new_l=l/0.08
    return new_l.tolist()

def publish_velocities(pub,velocities,angles,params):
    print("sending velocities: "+str(velocities)+" angles: "+str(angles))
    params["last_angles"]=angles
    msg=Float32MultiArray()
    msg.data=velocities+angles
    pub.publish(msg)

def odometry_callback(data,params):
    params["current_speed"] = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2)
    current_speed=params["current_speed"]
    current_twist=params.get("current_twist",None)
    if current_twist is not None and params["in_place_configuration"]==False:
        #if is not an in place rotation
        if abs(current_twist.linear.x)>0.001:
            desired_speed = current_twist.linear.x
            speed_error = desired_speed - current_speed
            print("current_speed {} desired_speed {} speed_error {}".format(current_speed,desired_speed,speed_error))
            
            current_wheel_vel=params.get("current_wheels_vel",None)
            if current_wheel_vel is None or len(current_wheel_vel)==0:
                return
            
            # PD controller
            Kp = 0.5  # Proportional gain
            Kd = 0.01  # Derivative gain
            previous_error = params.get("previous_error", 0)
            derivative = (speed_error - previous_error) / 0.1  # Assuming the function is called every 0.1 seconds
            correction = Kp * speed_error + Kd * derivative
            print("computed speed correction: ", correction)

            # Update wheel velocities
            print("Previous wheel velocities: ", current_wheel_vel)
            for i in range(len(current_wheel_vel)):
                current_wheel_vel[i] += correction
                # current_wheel_vel[i] = min(MAX_W, current_wheel_vel[i])
                # current_wheel_vel[i] = max(-MAX_W, current_wheel_vel[i])

            params["previous_error"] = speed_error

            print("Updated wheel velocities: ", current_wheel_vel)

            #create multliarray
            publish_velocities(params["pub"],current_wheel_vel,params["last_angles"],params)



'''
Thread that check if the robot is stopped for more than rover_threshold_stopped seconds.
If so, reset the wheels angles to 0
'''
def check_wheel_reset(params):
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
                publish_velocities(pub,velocities,angles,params)
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
        params["current_wheels_vel"]=[]
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

        #I don't want PD controller to interfere with in place rotation
        params["current_wheels_vel"]= []
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

    params["current_wheels_vel"]=wheel_velocities
    return wheel_velocities,wheel_angles

def callback(data,pub,rate,params):
    params["current_twist"]=data
    velocities,angles=compute_wheel_velocities(data,params)
    print("input linear: "+str(data.linear.x)+" angular: "+str(data.angular.z))
    print("output velocities: "+str(velocities)+" angles: "+str(angles))

    if abs(angles[0])==max_steer and abs(angles[1])==max_steer:
        if params["in_place_configuration"]==False:
            params["in_place_configuration"]=True
            print("IN place, sending only steer")
            temp_velocities=[0,0,0,0,0,0]
            publish_velocities(pub,temp_velocities,angles,params)
            time.sleep(in_place_delay)
            print("IN place, starting rotation")

    else:
        #If I was in place, wait wheel resetting before sending new velocities
        if params["in_place_configuration"]==True:
            print("in place configuration, waiting before moving")
            publish_velocities(pub,[0,0,0,0,0,0],[0,0,0,0],params)
            time.sleep(in_place_delay)
        params["in_place_configuration"]=False

    #create multliarray
    print("sending velocities: "+str(velocities)+" angles: "+str(angles))
    publish_velocities(pub,velocities,angles,params)

    rate.sleep()
    

def main():
    #publish a array
    pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    rospy.init_node('firmware_CC8', anonymous=True)
    rate = rospy.Rate(10)
    print("\n\n\navviato nodo firmware velocity\n\n\n")
    params={
        "in_place_configuration": False,
        "in_place_delay": in_place_delay,
        "current_configuration": "linear", #linear or in place
        "last_angles": last_angles,
        "is_robot_stopped": is_robot_stopped,
        "rover_threshold_stopped": rover_threshold_stopped,
        "last_stopped_time": last_stopped_time,
        "pub": pub,
        "current_speed": current_speed
    }
    print("starting subscriber")
    #subscriber pass data using lambda function

    rospy.Subscriber(odometry_topic, Odometry, lambda x: odometry_callback(x,params))

    rospy.Subscriber("/cmd_vel", Twist, lambda x: callback(x,pub,rate,params))

    if enable_check_reset_wheel:
        # Create a new thread that targets the check_wheel_reset function
        thread_check_wheel_reset = threading.Thread(target=check_wheel_reset, args=(params,))

        # Start the thread
        thread_check_wheel_reset.start()  
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        #stop thread
        if enable_check_reset_wheel:
            thread_check_wheel_reset.join()
