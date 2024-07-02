#ROS node that subscribe to a Odometry topic and publish the TF from odom to base_link

import rospy
import tf
from nav_msgs.msg import Odometry

def callback(data, base_frame,br):
    br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z),
                     (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
                     data.header.stamp,
                     base_frame,
                     "odom")
    
def main():
    rospy.init_node('tf_from_topic', anonymous=True)
    base_frame= rospy.get_param('~base_frame', 'base_link')
    br= tf.TransformBroadcaster()
    rospy.Subscriber("odom", Odometry, lambda data: callback(data,base_frame,br))
    rospy.spin()

if __name__ == '__main__':
    main()