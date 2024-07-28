
import rospy
from  geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from nav_msgs.msg import Odometry

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
max_steer=math.pi/4                     #max steering angle [rad]
MAX_V=1                                 #max linear velocity [m/s]
MAX_W = MAX_V/wheel_radius              #max angular velocity [rad/s]
desired_in_place_velocity = math.pi/6   #desired velocity for in place rotation [rad/s]
in_place_delay=2                        #How much to wait for wheel to be in position before starting inplace movement [s]
min_wheel_velocity = 0.1                #minimum velocity actuated for the driving wheels, if a lower speed is computed, it is clamped to 0. Used to avoid zzzzt soudn

turning_ratio_threshold=0.6             #threshold on v/w ratio to avoid turning radius too small in ackermann steering 
in_place_on_sharp_turn = True           #if true, the rover will rotate in place when turning with a small radius,otherwise it will decrease the angular velocity

enable_steer_limit = False              #enable steering angle limit, if true the steering angle will be limited to max_steer
enable_lock_steer  = True               #enable lock steering, if true the steering angle will be locked to the last angle when the rover is stopped
enable_steer_reset = True               #enable steering reset, if true the steering angle will be reset to 0 after some time when the rover is stopped
steer_reset_timeout= 3.0                #how many seconds to wait when rover is stopped before resetting the wheels [s]

enable_velocity_feedback = True         #USE a PD controller to adjust the velocity based on the feedback from an odometry message
velocity_feedback_KP = 0.5              #Proportional gain for the velocity feedback controller
velocity_feedback_KD = 0.01             #Derivative gain for the velocity feedback controller


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

    #Controls
    for (i,v) in enumerate(velocities):

        #do not actuate small velocities
        if v<min_wheel_velocity:
            velocities[i]=0

        #do not actuate too large velocities
        if v>MAX_W:
            velocities[i]=MAX_W
        elif v<-MAX_W:
            velocities[i]=-MAX_W
         
    #Adding data and publishing
    vels=list(velocities)
    angles=list(angles)
    data=vels+angles
    msg.data=data
    pub.publish(msg)
    params["mutex_publishing"]=False
    params["current_wheels_vel"]=velocities

def is_in_place_rotation_command(data):
    if abs(data.linear.x)<0.001 and abs(data.angular.z)>0.001:
        return True
    return False

def is_in_place_type(type):
    if type=="IN_PLACE":
        return True
    return False

def is_stop_command(data):
    if abs(data.linear.x)<0.001 and abs(data.angular.z)<0.001:
        return True
    return False

#Compute the wheel velocities and angles for in place rotation
def get_in_place_outputs(w):
    wheel_velocities=np.zeros(6)
    wheel_angles=[-max_steer,max_steer,max_steer,-max_steer]
    for i in range(len(wheel_positions)):
        wheel_x=wheel_positions[i][0]
        wheel_y=wheel_positions[i][1]
        
        r=math.sqrt(wheel_x*wheel_x+wheel_y*wheel_y)
        #cambia segno se ruota in senso sbagliato
        sgn= 1 if wheel_y < 0 else -1
        if w<0: 
            sgn*=-1

        wheel_velocities[i]=desired_in_place_velocity*r*sgn
    wheel_velocities=linear2angular(np.array(wheel_velocities))
    return wheel_velocities,wheel_angles,"IN_PLACE"


def odometry_callback(data,params):
    current_linear_speed = data.twist.twist.linear.x
    current_ang_speed = data.twist.twist.angular.z

    if current_linear_speed==0 and current_ang_speed==0:
        #This odometry message does not contain twist information
        rospy.logerr_once("Odometry message does not contain twist information, feedback will be ignored!")

        #Compute speed based on difference of positions
        last_odom_feedback=params.get("last_odom_feedback",None)
        last_time=last_odom_feedback.header.stamp.to_sec() if last_odom_feedback is not None else None
        if last_time is None:
            params["last_odom_feedback"]=data
            return
        last_pos=last_odom_feedback.pose.pose.position

        current_pos=data.pose.pose.position
        current_time=data.header.stamp.to_sec()

        dt=current_time-last_time
        if dt==0:
            return
        
        #I need to compute a linear and angular speed
        #positions are in the global frame, I need speeds in the robot frame (linear: X axis, angular: Z axis)
        dx=current_pos.x-last_pos.x
        dy=current_pos.y-last_pos.y
        dz=current_pos.z-last_pos.z

        #Compute linear speed
        linear_speed=math.sqrt(dx*dx+dy*dy)/dt

        #Compute angular speed
        angular_speed=dz/dt

        #Convert angular speed to robot frame
        #I need to rotate the vector (dx,dy) by -dz
        new_x=dx*np.cos(dz)+dy*np.sin(dz)
        new_y=-dx*np.sin(dz)+dy*np.cos(dz)

        #The new vector is (new_x,new_y)
        #The angle is the angle of the vector with the x axis
        angular_speed=math.atan2(new_y,new_x)

        #Store the computed speeds
        current_linear_speed=linear_speed
        current_ang_speed=angular_speed

        #TODO verify if this is correct, for now feedback with no twist is ignored

        return
    
    current_wheel_vel=params.get("current_wheels_vel",None)
    if current_wheel_vel is None or len(current_wheel_vel)==0:
        return
    
    #Skip correction when waiting to start in place rotation
    if params["in_place_configuration"]==True and np.all(np.abs(current_wheel_vel)<0.01):
        return
    
    last_cmd_vel=params["last_cmd_vel"]
    desired_linear=last_cmd_vel.linear.x
    if last_cmd_vel.angular.z>0:
        desired_angular=desired_in_place_velocity
    else:
        desired_angular=-desired_in_place_velocity
    linear_error = desired_linear - current_linear_speed
    angular_error = desired_angular - current_ang_speed

    error=linear_error
    if params["in_place_configuration"]==True:
        error=angular_error
        print("Correcting on in place rotation:\n\tcurrent_angular_speed={}\n\tdesired_angular_speed={}\n\tangular_error={}".format(current_ang_speed,desired_angular,angular_error))
    else:
        print("Correcting velocity:\n\tcurrent_linear_speed={}\n\tdesired_linear_speed={}\n\tlinear_error={}".format(current_linear_speed,desired_linear,linear_error))
       
    
    #** PD controller
    correction = 0
    #*Proportional term
    proportional = velocity_feedback_KP * error
    correction += proportional

    #*Derivative term
    derivative=0
    previous_error = params.get("previous_error", 0)
    time_now = time.time()
    time_previous_error = params.get("time_previous_error")
    if time_previous_error is None:
        #*Ignore derivative term for the first iteration
        params["time_previous_error"]=time_now
    else:
        dt=time_now-time_previous_error
        derivative = (error - previous_error) / dt  

    correction += velocity_feedback_KD * derivative

    print(f"Correction: {correction}. Proportional: {proportional}. Derivative: {derivative}")

    params["time_previous_error"]=time_now

    # Update wheel velocities
    previous_velocities = [x for x in current_wheel_vel]
    for i in range(len(current_wheel_vel)):
        if not params["in_place_configuration"]:
            current_wheel_vel[i] += correction
        else:
            wheel_y=wheel_positions[i][1]
            sgn= 1 if wheel_y < 0 else -1
            current_wheel_vel[i]+= (correction*sgn)

        current_wheel_vel[i] = min(MAX_W, current_wheel_vel[i])
        current_wheel_vel[i] = max(-MAX_W, current_wheel_vel[i])

    params["previous_error"] = error

    print("Previous wheel vels: ", np.round(previous_velocities,3))
    print("Corrected wheel vels: ", np.round(current_wheel_vel,3))
    print("maintaing angles:",params["last_angles"])

    publish_velocities(current_wheel_vel,params["last_angles"],params)

    params["rate"].sleep()

'''
Given a twist message, compute the wheel velocities
for a 6 driving wheels rover with 4 steering wheels
'''
def compute_wheel_velocities(data,params):
    if data.linear.x>MAX_V:
        data.linear.x=MAX_V
    elif data.linear.x<-MAX_V:
        data.linear.x=-MAX_V

    v=data.linear.x
    w=data.angular.z

    #*ROBOT STOPPED
    if is_stop_command(data):
        print_rover_state("ROVER STOP")
        wheel_velocities=np.zeros(6)
        wheel_angles=np.zeros(4)
        if enable_lock_steer and not params["in_place_configuration"]:
            #Stop wheels but maintain angle
            wheel_angles=params["last_angles"]
        params["last_stopped_time"]=time.time()
        params["is_robot_stopped"]=True
        return wheel_velocities,wheel_angles,"STOP"
    else:
        params["is_robot_stopped"]=False

    #*IN PLACE ROTATION
    if is_in_place_rotation_command(data):
        print_rover_state("IN PLACE ROTATION")
        return get_in_place_outputs(w)
    
    #* GOING STRAIGHT
    if abs(w)<1e-3:
        print_rover_state("ACKERMANN: STRAIGHT")
        wheel_velocities=[v,v,v,v,v,v]
        wheel_velocities= linear2angular(np.array(wheel_velocities))
        wheel_angles= np.zeros(4)
        #params["last_angles"]=wheel_angles #update last angles
        return wheel_velocities,wheel_angles,"STRAIGHT"


    #* ACKERMANN STEERING

    #*Check turning radius
    #Threshold should activate only in ackermann steering
    if w!=0 and not is_in_place_rotation_command(data) and not is_stop_command(data):
        ratio=abs(v/w)
        if ratio<=turning_ratio_threshold:
            print("WARN: v/w ratio too small. {}/{}={}".format(np.round(v,3),np.round(w,3),np.round(ratio,3)))
            if in_place_on_sharp_turn:
                print("Starting in place rotation to avoid small turning radius")
                return get_in_place_outputs(w)
            else:
                #decrease w so that new ratio is equal to threshold
                new_w = v/ (turning_ratio_threshold)
                new_w*=np.sign(w)
                print(f"Decreasing w to {np.round(new_w,3)} to avoid small turning radius, new ratio is {np.round(v/new_w,3)}")
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

    #* Compute wheel velocities
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
        
        wheel_angles[steer_idx]=angle

        #Clamp steering angle to a maximum value
        if enable_steer_limit:
            if abs(angle) > max_steer:
                print("WARN: angle > max_steer, angle: ",angle)
            if angle > max_steer:
                wheel_angles[steer_idx]=max_steer
            elif angle < -max_steer:
                wheel_angles[steer_idx]=-max_steer

        steer_idx+=1

    #Convert linear velocities to angular velocities and return
    wheel_velocities=linear2angular(np.array(wheel_velocities))
    #params["last_angles"]=wheel_angles #update last angles
    vels=np.array(wheel_velocities)
    angles=np.array(wheel_angles)
    
    return vels,angles,"ACKERMANN"

def callback(data,params):
    
    #new command received, reset is no more valid
    params["rover_reset_done"]=False 
    null_vels=np.zeros(6)
    null_angles=np.zeros(4)

    velocities,angles,type=compute_wheel_velocities(data,params)
    print(f"Inputs: linear: {np.round(data.linear.x,3)} angular: {np.round(data.angular.z,3)}")
    #print("output velocities: "+np.round(velocities,3)+" angles: "+np.round(angles,3))
    if is_in_place_type(type):
        if params["in_place_configuration"]==False:
            params["in_place_configuration"]=True
            # Send first only the steering angles and wait for the wheels to be in place
            print("Rotation in place: setting steering angles...")
            cmd_vel= Twist()
            cmd_vel.linear.x=0
            params["last_cmd_vel"]=cmd_vel
            params["last_angles"]=angles
            publish_velocities(null_vels,angles,params)
            time.sleep(in_place_delay)
            print("Rotation in place: starting rotation...")
    else:
        #If I was in place, wait wheel resetting before sending new velocities
        if params["in_place_configuration"]==True:
            #TODO do not reset to 0, set to steering angle or wait based on how much the wheel has to rotate
            print("Rover is in configuration in place, waiting for wheel to be in place")
            cmd_vel= Twist()
            cmd_vel.linear.x=0
            params["last_cmd_vel"]=cmd_vel
            params["last_angles"]=null_angles
            publish_velocities(null_vels,null_angles,params)
            time.sleep(in_place_delay)
        params["in_place_configuration"]=False

    #Publish computed velocity
    params["last_cmd_vel"]=data
    params["last_angles"]=angles
    print("pippo")
    #TODO here random rotation after stopped
    publish_velocities(velocities,angles,params)

    params["rate"].sleep()
    

def main():
    #publish a array
    pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    rospy.init_node('firmware_CC8', anonymous=True)
    rate = rospy.Rate(30)


    D=compute_rover_width()

    print("\n---------Firmware CC8 Started----------\n")

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
        "rate": rate,                                        #rate of the main loop
        "last_cmd_vel": Twist(),
    }

    odometry_topic=rospy.get_param("~odometry_topic","/zed2i/zed_node/odom")
    

    rospy.Subscriber("/cmd_vel", Twist, lambda x: callback(x,params)) 

    if enable_velocity_feedback:
        rospy.Subscriber(odometry_topic, Odometry, lambda x: odometry_callback(x,params))
   


    try:
        while not rospy.is_shutdown():
            #Check if the robot is stopped for more than some seconds. If so, reset the wheels angles to 0
            is_robot_stopped=params["is_robot_stopped"]
            last_stopped_time=params["last_stopped_time"]
            reset_done=params["rover_reset_done"]
            should_reset= (time.time()-last_stopped_time > steer_reset_timeout)
            #Steer reset does not make sense if lock steering is disabled
            is_enabled = enable_steer_reset and enable_lock_steer
            if is_enabled and not reset_done and is_robot_stopped and should_reset:
                
                #reset only if at least one angle is not 0
                if np.any(np.abs(params["last_angles"])>0.01):
                    #print("rover stopped for more than {} seconds, resetting wheels".format(steer_reset_timeout)) 
                    #reset wheels angles
                    params["last_cmd_vel"]=Twist()
                    params["last_angles"]=np.zeros(4)
                    reset_wheels(params)
                    params["rover_reset_done"]=True

            rate.sleep()
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