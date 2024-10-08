#!/usr/bin/env python3
"""
ROS node that waits for input of the user and send the command to the firmware of the carotator
"""

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, String

import sys, select, termios, tty
from tasks_utils.IO_utils import *
#NEW
def chatter_callback(data):
    print(data.data+"\r\n")


def deep_sample_callback(data,params):
    if data.data == params["deep_sample"]:
        return
    params["deep_sample"] = data.data
    params["is_updated"] = True

def surface_sample_callback(data,params):
    if data.data == params["surface_sample"]:
        return
    params["surface_sample"] = data.data
    params["is_updated"] = True

def print_terminal(params):
    clear_terminal()
    print(boot_msg)
    surface_data = np.round(params["surface_sample"],2)
    deep_data = np.round(params["deep_sample"],2)
    stats_string = stats_msg.format(params["led_status"],surface_data,deep_data)
    style_print(stats_string,bcolors.OKCYAN)

def publish_command(command,prev_command,pub):
    if command!=prev_command:
        pub.publish(command)

def main():
    settings = saveTerminalSettings()

    rospy.init_node('firmware_drilling', anonymous=True)

    settings=termios.tcgetattr(sys.stdin)
    print(boot_msg)

    params = {
        "stepper_topic": "stepper_commands",
        "drill_topic": "drill_commands",
        "led_topic": "led_commands",
        "load_cell_deep_topic": "load_cell_deep",
        "load_cell_surface_topic": "load_cell_surface",
        "deep_sample": 0,
        "surface_sample": 0,
        "led_status": "OFF",
        "is_updated": False

    }

    rospy.Subscriber(params["load_cell_deep_topic"], Float32, deep_sample_callback,params)
    rospy.Subscriber(params["load_cell_surface_topic"], Float32, surface_sample_callback,params)

    #TODO check type of topics in firmware drill
    #NEW
    pub = rospy.Publisher('comando', String, queue_size=10)
    clear_terminal()
    print(boot_msg)
    rospy.Subscriber('chatter', String, chatter_callback)
    prev_command = ""
    while not rospy.is_shutdown():
        try:
            if params["is_updated"]:
                print_terminal(params)
                params["is_updated"] = False
            
            command=""
            key = getKey(settings,0.1)
            
            if key == 'w' or key == 'W':
                command='w'
                #move drill u
                print("Moving drill up")
                pass
            elif key == 's' or key == 'S':
                #move drill down
                command='s'
                print("Moving drill down")
                pass
            elif key == ' ':
                command='k'
                clear_terminal()
                print(boot_msg)
                print("Stopping...")
                #stop 
                pass
            elif key=="e":
                print("surface up")
                command="e"
            elif key=="q":
                print("surface down")
                command="q"
            elif key == 'g' or key == 'G':
                command='g'
                #print('Green')
                pass
            elif key == 'r' or key == 'R':
                command='r'
                #print('Red')
                pass
            elif key == 'b' or key == 'B':
                command='b'
                #print('Blue')
                pass
            elif key == 'y' or key == 'Y':
                command='y'
                #print('Yellow')
                pass
            elif key == 'p' or key == 'P':
                command='p'
                #print('Purple')
                pass
            elif key == '+':
                command='+'
                print('+1000 deep factor')
                pass
            elif key == '-':
                command='-'
                print('-1000 deep factor')
                pass
            elif key == '*':
                command='*'
                print('+1000 surface factor')
                pass
            elif key == '_':
                command= '_'
                print('-1000 surface factor')
                pass
            elif key == '>':
                command= '>'
                print('Increased drilling speed')
                pass
            elif key == '<':
                command = '<'
                print('Decreased drlling speed')
                pass
            elif key == '\x03':
                print("Exiting...")
                break
            if command!="":
                publish_command(command,prev_command,pub)
                prev_command =command
            
        except KeyboardInterrupt:
            print("Exiting...")
            break

    restoreTerminalSettings(settings)

        
if __name__ == '__main__':
    main()