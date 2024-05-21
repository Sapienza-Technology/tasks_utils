
import rospy
from std_msgs.msg import String, Float32
#import twist message
from  geometry_msgs.msg import Twist
#import float32 multiarray
from std_msgs.msg import Float32MultiArray
import math
import time
import numpy as np
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

last_angles = [0, 0, 0, 0]

max_steer=math.pi/4
MAX_V=1
MAX_W=1/wheel_radius
IN_PLACE_VEL= MAX_V/2

def linear2angular(l):
    new_l=l/0.08
    return new_l.tolist()

def publish_velocities(pub,velocities,angles):
    formatted_velocities = [f"{v:.3f}" for v in velocities]
    formatted_angles = [f"{a:.3f}" for a in angles]

    print("\tsending velocities: "+str(formatted_velocities)+" angles: "+str(formatted_angles))
    msg=Float32MultiArray()
    msg.data=velocities+angles
    pub.publish(msg)

'''
Given a twist message, compute the wheel velocities
for a 6 driving wheels rover with 4 steering wheels
'''
def compute_wheel_velocities(data, params):
    print("\n")
    #to return
    wheel_velocities=[0,0,0,0,0,0]
    wheel_angles=[0,0,0,0]

    v=data.linear.x
    w=data.angular.z
    #w=-w #TODO resolve?

    turning_radius=None
    turning_rate=None

    vl=v - w*d/2
    vr=v + w*d/2
    print(f"vl: {vl:.3f}, vr: {vr:3f}")

    if abs(v)<0.001 and abs(w)<0.001:
        print("--- ROBOT STOPPED: waiting for move the steers")
        wheel_velocities=[0,0,0,0,0,0]
        wheel_angles=params["last_angles"]
        print(f"lstopped, last_angles: {last_angles}")
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
    print("turning_radius: "+str(turning_radius)+" turning_rate: "+str(turning_rate))
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
    print(f"last save: {wheel_angles}")
    return wheel_velocities,wheel_angles

def callback(data,pub,rate,params):
    velocities,angles=compute_wheel_velocities(data, params)

    print(f"input linear: {data.linear.x:.3f}, angular: {data.angular.z:.3f}")
    #print("output velocities: "+str(velocities)+" angles: "+str(angles))

    in_place_delay = 3

    msg=Float32MultiArray()
    if abs(angles[0])==max_steer and abs(angles[1])==max_steer:
        if params["in_place_configuration"]==False:
            params["in_place_configuration"]=True
            print("IN place, sending only steer")
            temp_velocities=[0,0,0,0,0,0]
            
            #msg.data=temp_velocities+angles
            #pub.publish(msg)
            #time.sleep(3)

            print("--- IN PLACE: stopping wheels")
            publish_velocities(pub,temp_velocities,last_angles)
            time.sleep(in_place_delay/2)

            print("--- IN PLACE: turn steers")
            publish_velocities(pub, temp_velocities, angles)
            time.sleep(in_place_delay)


            print("--- IN place: starting rotation")
    else:
        
        if params["in_place_configuration"]==True:

            print("--- WAS IN PLACE: stopping wheels")
            publish_velocities(pub,[0,0,0,0,0,0],[-max_steer,max_steer,max_steer,-max_steer])
            time.sleep(in_place_delay/2)   

            print("--- WAS IN PLACE: turning steers")
            publish_velocities(pub,[0,0,0,0,0,0],angles)
            time.sleep(in_place_delay)         

        params["in_place_configuration"]=False


    #create multliarray
    #msg.data=velocities+angles
    #pub.publish(msg)

    publish_velocities(pub, velocities, angles)

    rate.sleep()
    

def main():
    #publish a array
    pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    rospy.init_node('firmware_teensy', anonymous=True)
    rate = rospy.Rate(10)
    print("\n\n\navviato nodo firmware velocity\n\n\n")
    params={
        "in_place_configuration": False,
        "current_configuration": "linear", #linear or in place
        "pub": pub,
        "last_angles": last_angles,
        "last_stopped_time": time.time(),
        "is_robot_stopped": True

    }
    #subscriber pass data using lambda function
    rospy.Subscriber("/cmd_vel", Twist, lambda x: callback(x,pub,rate,params))

    #thread_check_stopped = threading.Thread(target=check_robot_stopped, args=(params,))

    # Start the thread
    #thread_check_stopped.start()  

    #rospy.spin()

    rover_threshold_stopped = 5

    while not rospy.is_shutdown():
        if params["is_robot_stopped"] == True and params['in_place_configuration'] == False:
            last_stopped_time=  params["last_stopped_time"]
            if abs( time.time() -last_stopped_time ) >= rover_threshold_stopped and params["is_robot_stopped"] == True:

                print(f"--- ROBOT STOPPED: passed more than {rover_threshold_stopped}, resetting wheels")
                velocities=[0,0,0,0,0,0]
                angles=[0,0,0,0]

                #create multliarray
                publish_velocities(pub,velocities,angles)
                time.sleep(1)    
            
                params["last_stopped_time"] = time.time()
                params["is_robot_stopped"] = False


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass