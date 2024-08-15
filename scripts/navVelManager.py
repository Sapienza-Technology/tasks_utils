import threading
import rospy
from enum import Enum

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty,EmptyResponse



class VelocityPublisher(threading.Thread):

    """
    Relay class. A thread that take messages from two topics and publish them to a third topic.
    It subscribes to a topic for autonomous navigation and manual navigation and publishes to a topic for velocity commands.
    If manual navigation is activated, it will publish the manual velocity command, otherwise it will publish the autonomous velocity command.

    """
    def __init__(self, variables):
        super(VelocityPublisher, self).__init__()

        #*Topics
        self.manual_vel_topic = variables["manual_vel_topic"]
        self.autonomous_vel_topic = variables["autonomous_vel_topic"]
        self.manual_lora_vel_topic = variables["manual_lora_vel_topic"]
        self.vel_topic = variables["vel_topic"] #Output velocity topic

        self.manual_vel_sub = rospy.Subscriber(self.manual_vel_topic, Twist, self.manual_vel_callback)
        self.autonomous_vel_sub = rospy.Subscriber(self.autonomous_vel_topic, Twist, self.autonomous_vel_callback)
        self.manual_lora_vel_sub = rospy.Subscriber(self.manual_lora_vel_topic, Twist, self.manual_lora_vel_callback)

        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)
        

        
        #*Control variables
        self.manual_vel = Twist()
        self.autonomous_vel = Twist()
        self.manual_lora_vel = Twist()
        self.rover_state = None
        self.states = variables["states"]

        #* LED STUFF
        self.led_lights= {
            self.states.IDLE : "r",
            self.states.MANUAL : "b",
            self.states.MANUAL_LORA : "y",
            self.states.AUTONOMOUS_RUNNING : "g",
        }
        self.led_pub = rospy.Publisher(variables["led_topic"], String, queue_size=1)

        #*Threading
        self.condition = threading.Condition()
        self.done = False
        self.new_manual_command = False
        self.new_autonomous_command = False
        self.new_lora_command = False
        self.start()

    def manual_vel_callback(self, msg):
        self.condition.acquire()
        self.new_manual_command = True
        self.manual_vel = msg
        self.condition.notify()
        self.condition.release()

    def autonomous_vel_callback(self, msg):
        self.condition.acquire()
        self.new_autonomous_command = True
        self.autonomous_vel = msg
        self.condition.notify()
        self.condition.release()
    
    def manual_lora_vel_callback(self, msg):
        self.condition.acquire()
        self.new_lora_command = True
        self.manual_lora_vel = msg
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.join()

    def run(self):
        try:
            while not self.done and not rospy.is_shutdown():
                self.condition.acquire()
                if not self.rover_state==self.states.IDLE:
                    if self.rover_state==self.states.MANUAL:
                        if self.new_manual_command:
                            self.vel_pub.publish(self.manual_vel)
                            self.new_manual_command = False
                    elif self.rover_state==self.states.MANUAL_LORA:
                        if self.new_lora_command:
                            self.vel_pub.publish(self.manual_lora_vel)
                            self.new_lora_command = False
                    elif self.rover_state==self.states.AUTONOMOUS_RUNNING:
                        if self.new_autonomous_command:
                            self.vel_pub.publish(self.autonomous_vel)
                            self.new_autonomous_command = False
                self.condition.wait()
                self.condition.release()
        except KeyboardInterrupt:
            print("ctrl c detected")

    def stop_rover(self):
        twist = Twist()
        self.vel_pub.publish(twist)
    
    def update_state(self,new_state):
        self.rover_state = new_state

        self.stop_rover()
        
        print("state now is:",self.rover_state)

        self.led_pub.publish(self.led_lights[self.rover_state])
        return EmptyResponse()
        


def main():
    try:
        rospy.init_node("navVelManager")

        
        state = Enum('state', ['IDLE','MANUAL','MANUAL_LORA','AUTONOMOUS_RUNNING'])

        variables = {
            "manual_vel_topic": rospy.get_param("~manual_vel_topic", "/cmd_vel_manual"),
            "autonomous_vel_topic": rospy.get_param("~autonomous_vel_topic", "/cmd_vel_auto"),
            "manual_lora_vel_topic": rospy.get_param("~manual_lora_vel_topic", "/cmd_vel_lora"),
            "vel_topic": rospy.get_param("~vel_topic", "/cmd_vel"),
            "led_topic": rospy.get_param("~led_topic", "/comando"),
            "states": state
        }
        velPublisher = VelocityPublisher(variables)
        manual_service= rospy.Service("CC8_manual", Empty, lambda x: velPublisher.update_state(state.MANUAL))
        auto_service=   rospy.Service("CC8_auto",   Empty, lambda x: velPublisher.update_state(state.AUTONOMOUS_RUNNING))
        manual_service= rospy.Service("CC8_lora",   Empty, lambda x: velPublisher.update_state(state.MANUAL_LORA))
        idle_service=   rospy.Service("CC8_idle",   Empty, lambda x: velPublisher.update_state(state.IDLE))

        rospy.spin()
        #TODO perchè non si chiude con ctrl+c? :(
    except KeyboardInterrupt:
        print("NAVMANAGER: exiting...")
        velPublisher.stop()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("NAVMANAGER: exiting...")