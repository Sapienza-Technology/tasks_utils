
import rospy
from std_msgs.msg import String, Float32
#import twist message
from  geometry_msgs.msg import Twist

R=0.08
L=0.515
enc_res=356.3

def callback(data,pubR,pubL,rate):
    #print("received data: ", data)
    linear=data.linear.x
    angular=data.angular.z
    V_r=linear+angular*L/2
    V_l=linear-angular*L/2
    W_r=V_r/R
    W_l=V_l/R
    print("V_r: ", W_r, "V_l: ", W_l)
    #publish
    pubR.publish(W_r)
    pubL.publish(W_l)
    rate.sleep()
    

def main():
    pubR = rospy.Publisher('/destra', Float32, queue_size=10)
    pubL = rospy.Publisher('/sinistra', Float32, queue_size=10)
    rospy.init_node('firmware_teensy', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #subscriber pass data using lambda function
    rospy.Subscriber("/cmd_vel", Twist, lambda x: callback(x,pubR,pubL,rate))
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
