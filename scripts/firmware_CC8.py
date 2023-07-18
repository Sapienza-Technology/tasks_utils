
import rospy
from std_msgs.msg import String, Float32
#import twist message
from  geometry_msgs.msg import Twist
#import float32 multiarray
from std_msgs.msg import Float32MultiArray
import math

wheel_radius=0.8
d=0.6 #distance between the wheels along y axis

#FRONT, MID, REAR (first left, then right)
wheel_positions=[ 
    [0.3,0.3],
    [0.3,-0.3],
    [0,0.3],
    [0,-0.3],
    [-0.3,0.3],
    [-0.3,-0.3]
]

max_steer=math.pi/2
'''
Given a twist message, compute the wheel velocities
for a 6 driving wheels rover with 4 steering wheels
'''
def compute_wheel_velocities(data):
    print("\n")
    #to return
    wheel_velocities=[0,0,0,0,0,0]
    wheel_angles=[0,0,0,0]

    v=data.linear.x
    w=data.angular.z

    turning_radius=None
    turning_rate=None

    vl=v - w*d/2
    vr=v + w*d/2
    print("vl: "+str(vl)+" vr: "+str(vr))

    #IN PLACE ROTATION
    if abs(w)>0.001 and abs(v)<0.001:
        print("in place rotation")
        wheel_angles=[max_steer,-max_steer,max_steer,-max_steer]
        
        for i in range(len(wheel_positions)):
            wheel_x=wheel_positions[i][0]
            wheel_y=wheel_positions[i][1]
            
            r=math.sqrt(wheel_x*wheel_x+wheel_y*wheel_y)

            #camba segno se ruota in senso sbagliato
            sgn= 1 if wheel_y > 0 else -1
            vel = w*r*sgn
            wheel_velocities[i]=vel/wheel_radius
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

            #flip angle by 180 deg if steering rotation is past 90 deg
            if angle > math.pi/2:
                angle-=math.pi
            elif angle < -math.pi/2:
                angle+=math.pi
            
            if angle > max_steer:
                print("WARN: angle > max_steer, angle: ",angle)
            wheel_angles[steer_idx]=min(angle,max_steer)
            steer_idx+=1
    
    return wheel_velocities,wheel_angles

def callback(data,pub,rate):
    velocities,angles=compute_wheel_velocities(data)
    print("input linear: "+str(data.linear.x)+" angular: "+str(data.angular.z))
    print("output velocities: "+str(velocities)+" angles: "+str(angles))
    #pub.publish(velocities)
    #rate.sleep()
    

def main():
    #publish a array
    pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
    rospy.init_node('firmware_teensy', anonymous=True)
    rate = rospy.Rate(10)
    print("\n\n\navviato nodo firmware velocity\n\n\n")

    #subscriber pass data using lambda function
    rospy.Subscriber("/cmd_vel", Twist, lambda x: callback(x,pub,rate))
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
